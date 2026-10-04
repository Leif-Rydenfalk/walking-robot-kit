//! The bus owner. One thread, one Io, every exchange a `bus_batch` call carrying whole frames (echo
//! included). Never two processes on the bus: this thread is the only writer after `bus_begin(1_000_000)`.
//!
//! Each tick (~30 Hz with a full leg):
//!   1. commands: stop-all first, then torque, set-ID (runs to completion here, nothing else on the bus),
//!      wave and manual moves (engineer view only, and only on the `--ids` motion set);
//!   2. one telemetry read (regs 40..70) of every roster ID, plus the EEPROM block (5..17) of one ID in
//!      rotation, in ONE RPC;
//!   3. a ping chunk of the 0..253 scan in a second RPC (shorter wait: an empty ID costs the whole wait),
//!      a full scan about every second, so rows appear and leave as servos are plugged.
//! A streak of failed RPCs is a wedged bus (2026-10-04 13:20 and 14:32): the daemon restarts the MCU app
//! itself (io.rs `recover`), rescans and says so on the page.
//!
//! Wave semantics ported from servo_live.py (measured good on 2026-10-02): one sync-write packet per
//! half-swing; safety: any servo >= 55 C stops the wave and rests it limp until <= 45 C; two reads under
//! 5.5 V stop everything.

use crate::feetech::{self, Telemetry};
use crate::io::Io;
use crate::json::J;
use crate::limits::{self, Lim, Limits};
use crate::registry::{self, Registry};
use crate::roster::{Read, Roster, Verdict};
use crate::setid;
use std::collections::HashMap;
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub const TEMP_MAX: u8 = 55;
pub const TEMP_RESUME: u8 = 45;
pub const V_MIN: f32 = 5.5;
pub const SPEED: u16 = 2400;
pub const ACC: u8 = 60;
pub const fn read_reply(n: usize) -> usize {
    n + 6
}
pub const READ_LEN: usize = 31;
pub const LO: u16 = 1024;
pub const HI: u16 = 3072;
pub const PING_CHUNK: u8 = 2;
pub const STALL_STREAK: u32 = 5;
/// A stall is a stretch of TIME with no answer, not a count of short deadlines. With poll_get waiting
/// 900 ms, five failures already mean seconds; this floor stops a burst of fast failures (a refused
/// socket, say) from calling the bus dead before it has really gone quiet. Measured 2026-10-04: nine
/// MCU-app restarts fired for stalls that had lasted 1.5 s.
pub const STALL_SECS: f64 = 3.0;
pub const RECOVER_GAP_S: f64 = 20.0;
/// After a reconnect that did not help, how long before climbing to an MCU app restart.
pub const SOFT_GAP_S: f64 = 4.0;
/// MCU app restarts allowed in one outage before the daemon stops restarting and just keeps
/// reconnecting (it never gives up and never needs a human to restart it).
pub const MAX_HARD_PER_OUTAGE: u32 = 3;
/// poll_get answering while the MCU sweeps nothing this long = it rebooted and lost Serial1.
pub const NO_SWEEP_REOPEN: Duration = Duration::from_millis(2000);

#[derive(Clone)]
pub struct ServoState {
    pub id: u8,
    pub online: bool,
    pub t: Option<Telemetry>,
}

#[derive(Clone, Default)]
pub struct Latency {
    pub last_ms: Option<f64>,
    pub median_ms: Option<f64>,
    pub min_ms: Option<f64>,
    pub max_ms: Option<f64>,
    pub n: usize,
}

#[derive(Clone, Default)]
pub struct BusHealth {
    pub ok: bool,
    pub fail_streak: u32,
    pub stalls: u64,
    pub recovers: u64,
    /// socket reconnects that opened the bus again without restarting the MCU app
    pub soft_recovers: u64,
    /// calls that ran out of time, and the ones the peer answered after we gave up (`late` > 0 is
    /// proof the MCU was alive through the stall and only slow)
    pub timeouts: u64,
    pub late: u64,
    pub late_ms: f64,
    /// from the sketch: frames it refused because the UART TX ring would not drain, and calls that gave up
    /// waiting for the bus mutex
    pub tx_stuck: u32,
    pub lock_lost: u32,
    /// The same two counters as they stood when the CURRENT outage began, and the highest either has
    /// reached in this daemon's life.
    ///
    /// The live pair is per-sketch-run, and the cure for a dark MCU is restarting the sketch, so the
    /// counters that would say WHY it went dark are zeroed by the only thing that brings it back.
    /// Measured 2026-10-04 21:51:16 CST: the MCU went dark, two app restarts recovered it, and
    /// afterwards `tx_stuck` and `lock_lost` both read 0 - which says nothing at all about the
    /// outage, while looking exactly like "the TX path was fine".
    pub tx_stuck_pre: u32,
    pub lock_lost_pre: u32,
    pub tx_stuck_max: u32,
    pub lock_lost_max: u32,
    pub last_error: String,
    pub stalled_since: Option<f64>,
    pub note: String,
    pub rpcs: u64,
    pub fails: u64,
    pub hz: f64,
    pub rpc_ms: f64,     // moving average of one bus_batch round trip
    pub rpc_max_ms: f64, // worst in the last 2 s window
}

#[derive(Clone)]
pub struct Shared {
    pub ids: Vec<u8>, // motion set (engineer view)
    pub servos: Vec<ServoState>,
    pub wave_on: bool,
    pub wave_amp_deg: f64,
    pub wave_period: f64,
    pub wave_started: Option<String>,
    pub moves: u64,
    pub mode: &'static str,
    pub why: String,
    pub cooling: bool,
    pub t: f64,
    pub started_unix: f64,
    pub hist: Vec<[f64; 4]>,
    pub lat: Latency,
    pub stop_ms: Option<f64>,
    pub cmds: u64,
    pub roster: Roster,
    pub registry: Registry,
    pub health: BusHealth,
    pub busy: Option<String>,
    pub last_setid: Option<J>,
    pub tz_h: i64,
    pub fake: bool,
    pub held: std::collections::BTreeSet<u8>,
    pub limits: Limits,
    /// LED matrix face sent by this daemon: (frames sent, last sent unix, last error)
    pub matrix: (u64, f64, String),
    /// outside controller on the raw door: (who, last call unix, calls, goals clamped, cap)
    pub raw: (String, f64, u64, u64, u16),
}

impl Shared {
    pub fn new(ids: Vec<u8>, registry: Registry) -> Self {
        Shared {
            ids: ids.clone(),
            servos: ids.iter().map(|&id| ServoState { id, online: false, t: None }).collect(),
            wave_on: false,
            wave_amp_deg: 45.0,
            wave_period: 3.0,
            wave_started: None,
            moves: 0,
            mode: "idle",
            why: String::new(),
            cooling: false,
            t: 0.0,
            started_unix: now_unix(),
            hist: Vec::with_capacity(902),
            lat: Latency::default(),
            stop_ms: None,
            cmds: 0,
            roster: Roster::default(),
            registry,
            health: BusHealth::default(),
            busy: None,
            last_setid: None,
            tz_h: 8,
            fake: false,
            held: Default::default(),
            limits: Limits::default(),
            matrix: (0, 0.0, String::new()),
            raw: (String::new(), 0.0, 0, 0, 0),
        }
    }

    /// The EEPROM angle limits (regs 9/11) last read from `id`.
    pub fn ee_band(&self, id: u8) -> Option<(u16, u16)> {
        self.roster.rows.get(&id).and_then(|r| r.eeprom.as_ref()).map(|e| (e.min_angle, e.max_angle))
    }

    /// One roster row in plain words for the customer page.
    fn row_json(&self, id: u8, now: f64) -> J {
        let r = &self.roster.rows[&id];
        let v = self.roster.verdict(id);
        let why = match &v {
            Verdict::Duplicate(w) | Verdict::Weak(w) => w.clone(),
            _ => String::new(),
        };
        let mut o = vec![
            ("id".to_string(), J::n(id as f64)),
            ("name".into(), J::s(&self.registry.name(id))),
            ("verdict".into(), J::s(v.code())),
            ("why".into(), J::s(&why)),
            ("stale".into(), J::Bool(r.stale(now))),
            ("seen_ago".into(), J::n(((now - r.last_seen) * 10.0).round() / 10.0)),
            ("clean".into(), J::n(r.clean() as f64)),
            ("reads".into(), J::n(r.reads() as f64)),
            ("registered".into(), J::Bool(self.registry.assigned.contains_key(&id))),
            ("held".into(), J::Bool(self.held.contains(&id))),
        ];
        if let Some(t) = &r.tele {
            o.push(("pos".into(), J::n(t.pos as f64)));
            o.push(("deg".into(), J::n((t.pos as f64 * 3600.0 / 4096.0).round() / 10.0)));
            o.push(("speed".into(), J::n(t.speed as f64)));
            o.push(("load_pct".into(), J::n(t.load as f64 / 10.0)));
            o.push(("current_ma".into(), J::n((t.current as f64 * 6.5).round())));
            o.push(("temp".into(), J::n(t.temp as f64)));
            o.push(("volt".into(), J::n(t.volt as f64)));
            o.push(("torque".into(), J::Bool(t.torque)));
            o.push(("torque_limit".into(), J::n(t.torque_limit as f64)));
            o.push(("moving".into(), J::Bool(t.moving != 0)));
            o.push(("goal".into(), J::n(t.goal as f64)));
        }
        if let Some(e) = &r.eeprom {
            o.push((
                "ee".into(),
                J::obj(vec![
                    ("id", J::n(e.id as f64)),
                    ("return_delay_us", J::n(e.return_delay_us as f64)),
                    ("min_angle", J::n(e.min_angle as f64)),
                    ("max_angle", J::n(e.max_angle as f64)),
                    ("max_torque", J::n(e.max_torque as f64)),
                ]),
            ));
        }
        o.push(("n_clean".into(), J::n(r.n_clean as f64)));
        o.push(("n_garbled".into(), J::n(r.n_garbled as f64)));
        o.push(("n_nothing".into(), J::n(r.n_nothing as f64)));
        if let Some((p50, p99, mx)) = crate::roster::pctl(&r.gaps) {
            o.push(("gap_ms".into(), J::Arr(vec![J::n((p50 * 10.0).round() as f64 / 10.0), J::n((p99 * 10.0).round() as f64 / 10.0), J::n((mx * 10.0).round() as f64 / 10.0)])));
        }
        if let Some(l) = self.limits.map.get(&id) {
            o.push(("lim".into(), Limits::lim_json(l)));
        }
        if let Some(b) = self.limits.band(id, r.eeprom.as_ref().map(|e| (e.min_angle, e.max_angle))) {
            o.push(("band".into(), J::Arr(vec![J::n(b.0 as f64), J::n(b.1 as f64)])));
        }
        if let Some(sn) = self.limits.seen.get(&id) {
            o.push(("seen".into(), Limits::seen_json(sn)));
        }
        let lens = r.garbled_lengths();
        if !lens.is_empty() {
            o.push(("garbled_lengths".into(), J::Arr(lens.iter().map(|(l, c)| J::Arr(vec![J::n(*l as f64), J::n(*c as f64)])).collect())));
        }
        J::Obj(o)
    }

    pub fn bus_json(&self) -> J {
        let now = now_unix();
        let ids = self.roster.ids();
        let dups: Vec<J> = ids.iter().filter(|&&i| matches!(self.roster.verdict(i), Verdict::Duplicate(_))).map(|&i| J::n(i as f64)).collect();
        let h = &self.health;
        J::obj(vec![
            ("t", J::n(now)),
            ("tz_h", J::n(self.tz_h as f64)),
            ("fake", J::Bool(self.fake)),
            ("rows", J::Arr(ids.iter().map(|&i| self.row_json(i, now)).collect())),
            ("dups", J::Arr(dups)),
            (
                "events",
                J::Arr(
                    self.roster
                        .events
                        .iter()
                        .rev()
                        .take(20)
                        .map(|e| J::obj(vec![("t", J::n(e.t)), ("id", e.id.map(|i| J::n(i as f64)).unwrap_or(J::Nil)), ("kind", J::s(e.kind)), ("text", J::s(&e.text))]))
                        .collect(),
                ),
            ),
            (
                "scan",
                J::obj(vec![
                    ("n", J::n(self.roster.scans as f64)),
                    ("found", J::Arr(self.roster.last_scan.iter().map(|&i| J::n(i as f64)).collect())),
                    ("age", J::n(if self.roster.last_scan_t > 0.0 { ((now - self.roster.last_scan_t) * 10.0).round() / 10.0 } else { -1.0 })),
                ]),
            ),
            (
                "health",
                J::obj(vec![
                    ("ok", J::Bool(h.ok)),
                    ("fail_streak", J::n(h.fail_streak as f64)),
                    ("stalls", J::n(h.stalls as f64)),
                    ("recovers", J::n(h.recovers as f64)),
                    ("soft_recovers", J::n(h.soft_recovers as f64)),
                    ("timeouts", J::n(h.timeouts as f64)),
                    ("late", J::n(h.late as f64)),
                    ("late_ms", J::n((h.late_ms * 10.0).round() / 10.0)),
                    ("tx_stuck", J::n(h.tx_stuck as f64)),
                    ("lock_lost", J::n(h.lock_lost as f64)),
                    ("tx_stuck_pre", J::n(h.tx_stuck_pre as f64)),
                    ("lock_lost_pre", J::n(h.lock_lost_pre as f64)),
                    ("tx_stuck_max", J::n(h.tx_stuck_max as f64)),
                    ("lock_lost_max", J::n(h.lock_lost_max as f64)),
                    ("last_error", J::s(&h.last_error)),
                    ("stalled_since", h.stalled_since.map(J::n).unwrap_or(J::Nil)),
                    ("note", J::s(&h.note)),
                    ("rpcs", J::n(h.rpcs as f64)),
                    ("fails", J::n(h.fails as f64)),
                    ("up_s", J::n((now - self.started_unix).round())),
                    ("hz", J::n((h.hz * 10.0).round() / 10.0)),
                    ("rpc_ms", J::n((h.rpc_ms * 10.0).round() / 10.0)),
                    ("rpc_max_ms", J::n((h.rpc_max_ms * 10.0).round() / 10.0)),
                ]),
            ),
            ("raw", J::obj(vec![("who", J::s(&self.raw.0)), ("last_t", J::n(self.raw.1)), ("calls", J::n(self.raw.2 as f64)), ("clamped", J::n(self.raw.3 as f64)), ("cap", J::n(self.raw.4 as f64))])),
            ("matrix", J::obj(vec![("frames_sent", J::n(self.matrix.0 as f64)), ("last_sent_t", J::n(self.matrix.1)), ("last_err", J::s(&self.matrix.2))])),
            ("busy", self.busy.clone().map(J::Str).unwrap_or(J::Nil)),
            ("last_setid", self.last_setid.clone().unwrap_or(J::Nil)),
            ("registry", self.registry.to_json().get("assigned").cloned().unwrap_or(J::Nil)),
            ("joints", J::Arr(registry::JOINTS.iter().map(|(i, n)| J::Arr(vec![J::n(*i as f64), J::s(n)])).collect())),
        ])
    }

    /// The engineer page's state document (the original shape) plus `bus` for the customer page.
    pub fn to_json(&self, hist_n: usize, light: bool) -> J {
        let mut servos = Vec::new();
        for s in &self.servos {
            let mut o = vec![("id".to_string(), J::n(s.id as f64)), ("ok".to_string(), J::Bool(s.online))];
            if let (true, Some(t)) = (s.online, &s.t) {
                let deg = (t.pos as f64 - LO as f64) * 90.0 / (HI - LO) as f64;
                o.push(("pos".into(), J::n(t.pos as f64)));
                o.push(("deg".into(), J::n((deg * 10.0).round() / 10.0)));
                o.push(("speed".into(), J::n(t.speed as f64)));
                o.push(("load".into(), J::n(t.load as f64)));
                o.push(("volt".into(), J::n(t.volt as f64)));
                o.push(("temp".into(), J::n(t.temp as f64)));
                o.push(("moving".into(), J::n(t.moving as f64)));
                o.push(("current".into(), J::n(t.current as f64)));
                o.push(("status".into(), J::n(t.status as f64)));
                o.push(("torque".into(), J::Bool(t.torque)));
            }
            servos.push(J::Obj(o));
        }
        let mut o = vec![
            ("ids".into(), J::Arr(self.ids.iter().map(|&i| J::n(i as f64)).collect())),
            ("servos".into(), J::Arr(servos)),
            ("wave".into(), J::Bool(self.wave_on)),
            ("amp".into(), J::n(self.wave_amp_deg)),
            ("period".into(), J::n(self.wave_period)),
            ("started".into(), self.wave_started.clone().map(J::Str).unwrap_or(J::Nil)),
            ("moves".into(), J::n(self.moves as f64)),
            ("mode".into(), J::s(self.mode)),
            ("why".into(), J::s(&self.why)),
            ("cooling".into(), J::Bool(self.cooling)),
            ("t".into(), J::n(self.t)),
            ("bus_ok".into(), J::Bool(self.health.ok)),
            ("reconnects".into(), J::n(self.health.recovers as f64)),
            ("cmds".into(), J::n(self.cmds as f64)),
            ("batch_rpcs".into(), J::n(self.health.rpcs as f64)),
            ("lo".into(), J::n(LO as f64)),
            ("hi".into(), J::n(HI as f64)),
            (
                "lat".into(),
                J::obj(vec![
                    ("last", opt(self.lat.last_ms)),
                    ("median", opt(self.lat.median_ms)),
                    ("min", opt(self.lat.min_ms)),
                    ("max", opt(self.lat.max_ms)),
                    ("n", J::n(self.lat.n as f64)),
                    ("note", J::s("command received by the daemon -> first read where the encoder moved >= 8 counts")),
                ]),
            ),
            ("stop_ms".into(), opt(self.stop_ms)),
            ("bus".into(), self.bus_json()),
        ];
        if !light {
            let start = self.hist.len().saturating_sub(hist_n);
            o.push(("hist".into(), J::Arr(self.hist[start..].iter().map(|p| J::Arr(p.iter().map(|&x| J::n(x)).collect())).collect())));
        }
        J::Obj(o)
    }
}

fn opt(v: Option<f64>) -> J {
    v.map(J::n).unwrap_or(J::Nil)
}

pub enum Cmd {
    Move { id: u8, deg: f64 },
    Torque { id: Option<u8>, on: bool },
    Wave { amp_deg: f64, period: f64 },
    StopWave,
    StopAll,
    SetId { old: u8, new: u8, joint: String, who: String, reply: Option<Sender<J>> },
    /// Manual control : hold = goal at the present
    /// position, torque limit 300 (30 %) or lower, torque on; release = torque off.
    Hold { id: u8, on: bool },
    /// Goal in counts for a held servo, clamped to its angle limits (refused when it has none).
    Goal { id: u8, pos: u16 },
    /// "Record" on the page: forget this servo's recorded range and start again from the next torque-off read.
    RecordLimits { id: u8 },
    /// Save limits from recorded ends (given, or the daemon's own `seen` range), write EEPROM 9/11, read back.
    SaveLimits { id: u8, ends: Option<(u16, u16)>, who: String, source: String, reply: Option<Sender<J>> },
    /// Raw frames from an outside controller through `rawgate` (reads pass; goals clamped to limits; torque capped).
    Raw { frames: Vec<Vec<u8>>, expects: Vec<usize>, wait_us: i64, cap: u16, who: String, reply: Sender<J> },
    /// Move a servo's OWN zero to where it is standing now: the one EEPROM write the raw door cannot do.
    Recentre { id: u8, who: String, reply: Option<Sender<J>> },
    /// Which way is up: scan the I2C bus, or read the MPU-6050 (beginning it first if it has not begun).
    Imu { scan: bool, reply: Sender<J> },
}

pub const SPEED_MANUAL: u16 = 1500; // steps/s
pub const ACC_MANUAL: u8 = 40;
pub const HOLD_CAP: u16 = 300;
/// Ceiling for an outside caller's torque on the raw door. 51/20:00: homing on the floor must
/// lift the body ("it has to lift itself"); at 300 the knee could not. The 55 C stop and the 1 s watchdog stay.
pub const RAW_CAP_MAX: u16 = 700;
/// The one count that means zero on every the robot joint (firmware model.rs:206,
/// `counts_to_rad(raw) = 2*pi*raw/4096 - pi`). `recentre` moves a servo's own zero here and nowhere else.
pub const CENTRE: u16 = 2048;
/// Read-back tolerance after a recentre. The servo computes its own correction, so a few counts of
/// quantisation are normal; 30 counts (2.6 deg) is not, and means it did not take the write.
pub const CENTRE_TOL: u16 = 10;
/// A zero taken while the joint is moving is not a zero: two reads this far apart must agree this closely.
pub const STILL_MS: u64 = 150;
pub const STILL_COUNTS: u16 = 4;
/// How long the servo is given to write its own correction before the position is read back.
pub const MIDPOINT_MS: u64 = 120;
/// MPU-6050 scales, fixed in the sketch's `imu_begin` and stated once here: +/- 500 dps and +/- 4 g.
/// The daemon reports RAW LSB and these two numbers; the conversion to rad/s and g lives with the policy,
/// the same way the servo register map lives on this side and not in the sketch.
pub const IMU_GYRO_LSB_PER_DPS: f64 = 65.5;
pub const IMU_ACCEL_LSB_PER_G: f64 = 8192.0;
/// Where the MPU-6050 sits with AD0 grounded (see docs/electronics.md).
pub const IMU_ADDR: u8 = 0x68;
/// The I2C buses the sketch declares, in the order `Wire.h` names them. Read off the official core's own
/// variant overlay (arduino/ArduinoCore-zephyr, variants/arduino_uno_q_stm32u585xx, fetched 2026-10-04):
/// `zephyr,user` declares `i2cs = <&i2c2>, <&i2c4>, <&i2c3>`. we wired the IMU to the HEADER on
/// 2026-10-04 because he had no Qwiic cable, which is the first of these, not the second.
pub const IMU_BUS_NAMES: [&str; 3] = [
    "Wire (i2c2, header D20 PB11 = SDA, D21 PB10 = SCL)",
    "Wire1 (i2c4, the Qwiic socket, PD13/PD12)",
    "Wire2 (i2c3, PC1/PC0)",
];

pub fn bus_name(b: u8) -> String {
    IMU_BUS_NAMES.get(b as usize).map(|s| s.to_string()).unwrap_or_else(|| format!("bus {b}, which this build does not know"))
}

pub fn now_unix() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
}

pub fn hhmmss() -> String {
    let secs = now_unix() as i64 % 86400;
    format!("{:02}:{:02}:{:02}", secs / 3600, secs % 3600 / 60, secs % 60)
}

/// "2026-10-04 15:02 CST" style stamp at a fixed UTC offset (the board clock is UTC; we reads China time).
pub fn stamp(unix: f64, tz_h: i64) -> String {
    let s = unix as i64 + tz_h * 3600;
    let (days, rem) = (s.div_euclid(86400), s.rem_euclid(86400));
    // civil-from-days (H. Hinnant)
    let z = days + 719468;
    let era = z.div_euclid(146097);
    let doe = z - era * 146097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + (m <= 2) as i64;
    let zone = if tz_h == 8 { " CST".to_string() } else { format!(" UTC{tz_h:+}") };
    format!("{y:04}-{m:02}-{d:02} {:02}:{:02}:{:02}{zone}", rem / 3600, rem % 3600 / 60, rem % 60)
}

/// A raw caller (the policy, 50 Hz) silent this long with torque on = it died: the daemon turns its servos off.
pub const RAW_QUIET: Duration = Duration::from_millis(1000);

pub struct Engine {
    /// the MPU-6050 has been woken and configured, read back off the chip (never assumed)
    imu_began: bool,
    imu_reads: u64,
    /// The newest IMU sample that rode out on a poll_get reply: (14 raw bytes, its age on the MCU in ms,
    /// when this daemon received it). Serving /api/imu from here is the difference between 49.7 Hz and
    /// 32.4 Hz of sweep, measured 2026-10-05 01:0x: an IMU read over its own RPC costs ~34 ms of round
    /// trip, and the sweep already asks the MCU for a reply every tick.
    imu_cache: Option<(Vec<u8>, u16, Instant)>,
    /// which bus the sensor answered on; -1 until one has
    imu_bus: i32,
    pub io: Box<dyn Io>,
    pub sh: Arc<Mutex<Shared>>,
    ids: Vec<u8>,
    side: HashMap<u8, bool>,
    next_due: HashMap<u8, Instant>,
    phase_t0: Option<Instant>,
    sag: u32,
    probe: Option<(u8, u16, Instant)>,
    probe_iters: u32,
    scan_at: u8,
    scan_found: Vec<u8>,
    scan_started: f64,
    ee_turn: usize,
    tick_n: u64,
    last_recover: f64,
    first_fail: f64,
    hz_t0: Instant,
    hz_n: u32,
    /// The self-heal ladder for one outage: a socket reconnect first, then up to MAX_HARD_PER_OUTAGE MCU app
    /// restarts spaced by RECOVER_GAP_S, then reconnects for ever. Any RPC that answers resets all of it.
    /// It must not restart-loop the app, and a restart that does not heal must be followed by reconnecting
    /// our own router connection rather than by waiting for a human.
    soft_tried: bool,
    hard_done: u32,
    /// the last time the MCU reported it had swept the bus (see NO_SWEEP_REOPEN)
    last_sweep: Instant,
    stall_said: bool,
    next_gap: f64,
    /// None = not asked yet; Some(false) = the sketch has no poll loop (fall back to one bus_batch per tick)
    poll_mode: Option<bool>,
    poll_sent: Option<Vec<u8>>,
    begun: bool,
    stuck_at: Option<Instant>,
    /// `--matrix-frame`: the face file another service writes; this engine is the only one that sends it.
    pub matrix_path: Option<String>,
    matrix_checked: Option<Instant>,
    matrix_mtime: Option<SystemTime>,
    matrix_rgb: Option<(u16, u16, u16)>,
    matrix_said: f64,
    /// servos a raw caller (the policy) turned torque on and has not turned off
    pub raw_on: std::collections::BTreeSet<u8>,
    pub raw_last: Option<Instant>,
}

impl Engine {
    pub fn new(io: Box<dyn Io>, sh: Arc<Mutex<Shared>>) -> Self {
        let ids = sh.lock().unwrap().ids.clone();
        Engine {
            imu_began: false,
            imu_reads: 0,
            imu_cache: None,
            imu_bus: -1,
            io,
            sh,
            ids,
            side: HashMap::new(),
            next_due: HashMap::new(),
            phase_t0: None,
            sag: 0,
            probe: None,
            probe_iters: 0,
            scan_at: 0,
            scan_found: Vec::new(),
            scan_started: now_unix(),
            ee_turn: 0,
            tick_n: 0,
            last_recover: 0.0,
            first_fail: 0.0,
            hz_t0: Instant::now(),
            hz_n: 0,
            soft_tried: false,
            hard_done: 0,
            last_sweep: Instant::now(),
            stall_said: false,
            next_gap: 0.0,
            poll_mode: None,
            poll_sent: None,
            begun: false,
            stuck_at: None,
            matrix_path: None,
            matrix_checked: None,
            matrix_mtime: None,
            matrix_rgb: None,
            matrix_said: 0.0,
            raw_on: Default::default(),
            raw_last: None,
        }
    }

    fn batch(&mut self, frames: &[Vec<u8>], expects: &[usize], wait_us: i64, timeout: Duration) -> Option<Vec<Vec<u8>>> {
        let t0 = Instant::now();
        let r = self.io.batch(frames, expects, wait_us, timeout);
        self.account(r, t0)
    }

    /// Health bookkeeping for one RPC: rate, timing, failure streak, stall.
    fn account<T>(&mut self, r: Result<T, String>, t0: Instant) -> Option<T> {
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        let (timeouts, late, late_ms) = self.io.lateness();
        let mut s = self.sh.lock().unwrap();
        s.health.timeouts = timeouts;
        s.health.late = late;
        s.health.late_ms = late_ms;
        s.health.rpcs += 1;
        s.health.rpc_ms = if s.health.rpc_ms == 0.0 { ms } else { s.health.rpc_ms * 0.95 + ms * 0.05 };
        s.health.rpc_max_ms = s.health.rpc_max_ms.max(ms);
        match r {
            Ok(v) => {
                // the bus answered: the whole ladder is rearmed (each wedge 16:06-16:13 came when a servo was
                // plugged or unplugged); RECOVER_GAP_S still spaces the restarts
                self.soft_tried = false;
                self.hard_done = 0;
                self.stall_said = false;
                self.next_gap = 0.0;
                if s.health.fail_streak >= STALL_STREAK {
                    s.health.note = format!("bus answering again at {}", stamp(now_unix(), s.tz_h));
                }
                s.health.fail_streak = 0;
                s.health.ok = true;
                s.health.stalled_since = None;
                Some(v)
            }
            Err(e) => {
                drop(s);
                self.note_fail(e);
                None
            }
        }
    }

    /// One place where a failed call is written down, so every caller counts a wedge the same way.
    ///
    /// It used to live inline in `account()`, and the `bus_begin` branch of the main loop kept its own
    /// copy that bumped `fail_streak` and `last_error` but never `stalls` or `stalled_since`. Measured
    /// 2026-10-04 21:12:19-21:18:54 CST: the MCU app went away, the ladder ran its full course (two
    /// reconnects, two app restarts, 6.5 minutes with no bus) and `health.stalls` read **0** the whole
    /// way, because every one of those failures was a `bus_begin` and `bus_begin` was accounted by hand.
    /// `stalls` is the number the soak calls a wedge, so the one outage worth counting was the one
    /// outage that did not count.
    /// Open the bus if it is not open. `false` means it is still shut and the caller must not use it.
    ///
    /// This is a method rather than four lines inside the loop so a test can drive it: the outage of
    /// 21:12:19 CST was invisible precisely because nothing could reach this code without a board.
    fn begin_tick(&mut self) -> bool {
        if self.begun {
            return true;
        }
        match self.io.begin() {
            Ok(()) => {
                self.begun = true;
                true
            }
            Err(e) => {
                self.note_fail(format!("bus_begin: {e}"));
                self.maybe_recover();
                false
            }
        }
    }

    fn note_fail(&mut self, e: String) {
        let mut s = self.sh.lock().unwrap();
        // The MCU forgot the bus. `bus_batch answered 0 frames for N` is the sketch's `if (!g_baud)
        // return out;` - it rebooted (someone restarted the app, a reflash, a crash) and Serial1 was
        // never opened again; `method bus_batch not available` is the router's table during the same
        // window. Measured 2026-10-04 20:36:50 CST: 2315 failed calls in a row, every one of them
        // curable by one bus_begin, while the daemon sat in "restarted once, waiting for a human".
        if e.contains("answered 0 frames") || e.contains("not available") {
            self.begun = false;
        }
        if s.health.fail_streak == 0 {
            self.first_fail = now_unix();
            // snapshot before the cure destroys the evidence
            s.health.tx_stuck_pre = s.health.tx_stuck;
            s.health.lock_lost_pre = s.health.lock_lost;
        }
        s.health.fail_streak += 1;
        s.health.fails += 1;
        s.health.last_error = e;
        if s.health.fail_streak >= STALL_STREAK && now_unix() - self.first_fail >= STALL_SECS && s.health.stalled_since.is_none() {
            s.health.stalls += 1;
            s.health.stalled_since = Some(self.first_fail);
            s.health.ok = false;
        }
    }

    fn sync_move(&mut self, targets: &[(u8, u16)]) -> Option<()> {
        let entries: Vec<(u8, Vec<u8>)> = targets.iter().map(|&(id, pos)| (id, feetech::move_data(pos, SPEED, ACC))).collect();
        let pkt = feetech::sync_write(feetech::REG_ACC, &entries);
        self.batch(&[pkt], &[0], 300, Duration::from_millis(100)).map(|_| ())
    }

    fn torque(&mut self, ids: &[u8], on: bool) -> Option<()> {
        if ids.is_empty() {
            return Some(());
        }
        let entries: Vec<(u8, Vec<u8>)> = ids.iter().map(|&id| (id, vec![on as u8])).collect();
        let pkt = feetech::sync_write(feetech::REG_TORQUE, &entries);
        self.batch(&[pkt], &[0], 300, Duration::from_millis(100)).map(|_| ())
    }

    fn read_pos(&mut self, id: u8) -> Option<u16> {
        let pkt = feetech::read(id, 56, 2);
        let got = self.batch(&[pkt.clone()], &[read_reply(2)], 2500, Duration::from_millis(100))?;
        let r = feetech::replies(&got[0], &pkt).into_iter().next()?;
        Some(*r.data.first()? as u16 | (*r.data.get(1)? as u16) << 8)
    }

    /// Telemetry of every roster ID (+ one EEPROM block) in one RPC, then a ping chunk in another.
    pub fn roster_tick(&mut self) {
        if self.poll_mode != Some(false) && self.poll_tick() {
            return;
        }
        self.batch_tick();
    }

    /// The MCU sweeps the bus itself (sketch poll loop); one compact poll_get per tick. False = not available,
    /// the caller falls back to batch_tick.
    fn poll_tick(&mut self) -> bool {
        let ids = self.sh.lock().unwrap().roster.ids();
        let t = now_unix();
        if self.poll_sent.as_ref() != Some(&ids) {
            let t0 = Instant::now();
            match self.io.poll_set(&ids) {
                Err(e) if e.contains("not available") => {
                    self.poll_mode = Some(false);
                    return false;
                }
                r => {
                    if self.account(r, t0).is_none() {
                        self.after_tick(t);
                        return true;
                    }
                    self.poll_mode = Some(true);
                    self.poll_sent = Some(ids.clone());
                }
            }
        }
        let t0 = Instant::now();
        let r = self.io.poll_get();
        let Some(raw) = self.account(r, t0) else {
            self.after_tick(t);
            return true;
        };
        let sweeps = self.feed_snapshot(&raw, &ids, t);
        // A sweeping MCU reports sweeps > 0 every tick. Zero for seconds while IDs are polled means the MCU
        // rebooted and lost Serial1 (`loop()` idles while `g_baud` is 0): poll_get still answers, so nothing
        // fails and nothing recovers - the page just goes quietly stale. Open the bus again.
        if sweeps > 0 || ids.is_empty() {
            self.last_sweep = Instant::now();
        } else if self.last_sweep.elapsed() > NO_SWEEP_REOPEN {
            self.last_sweep = Instant::now();
            self.begun = false;
            self.poll_sent = None;
            let mut s = self.sh.lock().unwrap();
            let tz = s.tz_h;
            s.health.note = format!("the MCU answered but swept nothing for {:.0} s: opening the bus again ({})", NO_SWEEP_REOPEN.as_secs_f64(), stamp(now_unix(), tz));
            let txt = s.health.note.clone();
            s.roster.event(now_unix(), None, "stall", txt);
        }
        self.after_tick(t);
        true
    }

    /// Parse poll_get: [seq lo, seq hi, sweeps, n, (id, kind, a, b) x n, full id, full len, data.., ee id, ee len,
    /// data.., wraps, n found, found ids..]
    fn feed_snapshot(&mut self, raw: &[u8], ids: &[u8], t: f64) -> u8 {
        let mut k = 0usize;
        let mut take = |n: usize| -> Option<&[u8]> {
            let r = raw.get(k..k + n)?;
            k += n;
            Some(r)
        };
        let Some(h) = take(4) else { return 0 };
        let (sweeps, n) = (h[2], h[3] as usize);
        let Some(rows) = take(4 * n) else { return 0 };
        let rows = rows.to_vec();
        let Some(fh) = take(2) else { return 0 };
        let (fid, flen) = (fh[0], fh[1] as usize);
        let full = take(flen).map(|d| d.to_vec()).unwrap_or_default();
        let Some(eh) = take(2) else { return 0 };
        let (eid, elen) = (eh[0], eh[1] as usize);
        let ee = take(elen).map(|d| d.to_vec()).unwrap_or_default();
        let Some(sc) = take(2) else { return 0 };
        let (wraps, nf) = (sc[0], sc[1] as usize);
        let found = take(nf).map(|d| d.to_vec()).unwrap_or_default();
        // The IMU trailer, when the sketch is new enough to send one: [have, 14 bytes, age lo, age hi].
        // An older sketch simply ends here and the cache is left alone, so /api/imu falls back to its RPC.
        if let Some(tr) = take(17) {
            if tr[0] == 1 {
                self.imu_cache = Some((tr[1..15].to_vec(), tr[15] as u16 | (tr[16] as u16) << 8, Instant::now()));
            }
        }
        let now = now_unix();
        let mut s = self.sh.lock().unwrap();
        if sweeps > 0 {
            let full_tel = if flen == READ_LEN { feetech::telemetry(&full) } else { None };
            for r in rows.chunks(4) {
                let id = r[0];
                if !ids.contains(&id) {
                    continue; // the MCU still swept an older set
                }
                let read = match r[1] {
                    0 | 3 => {
                        let mut tel = match (&full_tel, id == fid) {
                            (Some(f), true) => f.clone(),
                            _ => s.roster.rows.get(&id).and_then(|x| x.tele.clone()).unwrap_or_default(),
                        };
                        tel.pos = r[2] as u16 | (r[3] as u16) << 8;
                        Read::Clean(tel)
                    }
                    2 => Read::Garbled(r[2] as usize),
                    _ => Read::Nothing,
                };
                s.roster.feed(id, t, read);
            }
            if elen == feetech::EE_LEN as usize {
                if let Some(e) = feetech::eeprom(&ee) {
                    s.roster.feed_eeprom(eid, e);
                }
            }
            s.roster.note_verdicts(t);
        }
        if self.scan_at == 0 && self.scan_found.is_empty() && self.scan_started == 0.0 {
            self.scan_started = t;
        }
        for &id in &found {
            if !self.scan_found.contains(&id) {
                self.scan_found.push(id);
            }
            s.roster.pinged(id, now);
        }
        if wraps > 0 {
            s.roster.scan_done(&self.scan_found.clone(), self.scan_started, now);
            self.scan_found.clear();
            self.scan_started = now;
        }
        sweeps
    }

    fn batch_tick(&mut self) {
        // Budget (2026-10-04 15:55, measured): one bus_batch RPC costs ~12 ms before any servo byte, because the
        // router <-> MCU link is 115200 baud. So a tick is ONE RPC: the position of every servo (8-byte replies),
        // the full 40..70 read of one servo in rotation, one EEPROM read every 4th tick, and and
        // the next PING_CHUNK IDs of the 0..253 scan on every tick (2 IDs: ~2 ms, a full scan ~2.5 s).
        let ids = self.sh.lock().unwrap().roster.ids();
        let t = now_unix();
        let n = ids.len();
        let mut frames: Vec<Vec<u8>> = Vec::new();
        let mut expects: Vec<usize> = Vec::new();
        let full_id = if n > 0 { Some(ids[self.tick_n as usize % n]) } else { None };
        for &id in &ids {
            frames.push(feetech::read(id, 56, 2));
            expects.push(read_reply(2));
        }
        if let Some(f) = full_id {
            frames.push(feetech::read(f, feetech::READ_ADDR, feetech::READ_LEN));
            expects.push(read_reply(READ_LEN));
        }
        let ee_id = if n > 0 && self.tick_n % 4 == 0 {
            self.ee_turn = (self.ee_turn + 1) % n;
            Some(ids[self.ee_turn])
        } else {
            None
        };
        if let Some(e) = ee_id {
            frames.push(feetech::read(e, feetech::EE_ADDR, feetech::EE_LEN));
            expects.push(read_reply(feetech::EE_LEN as usize));
        }
        let ping_at = frames.len();
        if self.scan_at == 0 {
            self.scan_started = t;
            self.scan_found.clear();
        }
        let chunk: Vec<u8> = (self.scan_at..self.scan_at.saturating_add(PING_CHUNK).min(254)).collect();
        for &i in &chunk {
            frames.push(feetech::ping(i));
            expects.push(read_reply(0)); // the URT-2 sends no echo
        }
        // a read answers within ~0.9 ms (506 us return delay + bytes); a missing ID waits the whole cap
        let Some(out) = self.batch(&frames, &expects, 1000, Duration::from_millis(400)) else {
            self.after_tick(t);
            return;
        };
        let body = |k: usize| -> Vec<u8> {
            let raw = out.get(k).cloned().unwrap_or_default();
            if raw.starts_with(&frames[k]) { raw[frames[k].len()..].to_vec() } else { raw }
        };
        let now = now_unix();
        let mut s = self.sh.lock().unwrap();
        let full = full_id.and_then(|f| match setid::classify(&body(n), f, READ_LEN) {
            setid::Got::Data(d) => feetech::telemetry(&d),
            _ => None,
        });
        for (k, &id) in ids.iter().enumerate() {
            let r = match setid::classify(&body(k), id, 2) {
                setid::Got::Data(d) => {
                    let pos = d[0] as u16 | (d[1] as u16) << 8;
                    let mut tel = match (&full, Some(id) == full_id) {
                        (Some(f), true) => f.clone(),
                        _ => s.roster.rows.get(&id).and_then(|r| r.tele.clone()).unwrap_or_default(),
                    };
                    tel.pos = pos;
                    Read::Clean(tel)
                }
                setid::Got::Garbled(g) => Read::Garbled(g),
                setid::Got::Nothing => Read::Nothing,
            };
            s.roster.feed(id, t, r);
        }
        if let Some(e) = ee_id {
            if let setid::Got::Data(d) = setid::classify(&body(n + 1), e, feetech::EE_LEN as usize) {
                if let Some(ee) = feetech::eeprom(&d) {
                    s.roster.feed_eeprom(e, ee);
                }
            }
        }
        s.roster.note_verdicts(t);
        if let Some(&last) = chunk.last() {
            for (k, &id) in chunk.iter().enumerate() {
                if !body(ping_at + k).is_empty() {
                    self.scan_found.push(id);
                    s.roster.pinged(id, now);
                }
            }
            if last >= 253 {
                s.roster.scan_done(&self.scan_found.clone(), self.scan_started, now);
                self.scan_at = 0;
            } else {
                self.scan_at = last + 1;
            }
        }
        drop(s);
        self.after_tick(t);
    }

    /// The engineer view's motion set reads from the roster.
    fn after_tick(&mut self, t: f64) {
        let mut s = self.sh.lock().unwrap();
        let fresh: Vec<(u8, u16, bool)> = s.roster.rows.values().filter(|r| r.last_clean == t).filter_map(|r| r.tele.as_ref().map(|x| (r.id, x.pos, x.torque))).collect();
        for (id, pos, tq) in fresh {
            s.limits.observe(id, pos, tq, t);
        }
        let mut row = [t, f64::NAN, f64::NAN, f64::NAN];
        for k in 0..s.servos.len() {
            let id = s.servos[k].id;
            let (online, tel) = match s.roster.rows.get(&id) {
                Some(r) => (!r.stale(t), r.tele.clone()),
                None => (false, None),
            };
            s.servos[k].online = online;
            s.servos[k].t = tel.clone();
            if k < 3 {
                if let (true, Some(tt)) = (online, tel) {
                    row[k + 1] = tt.pos as f64;
                }
            }
        }
        s.t = t;
        s.hist.push(row);
        if s.hist.len() > 900 {
            s.hist.drain(..100);
        }
    }

    /// The bus has gone quiet: climb the self-heal ladder. Reconnect our own router socket first (under a
    /// second, no hole in the bus), then restart the MCU app, up to MAX_HARD_PER_OUTAGE times spaced by
    /// RECOVER_GAP_S, then go on reconnecting for ever. It never stops trying and never asks for a human:
    /// on 2026-10-04 at 19:50:51 and 20:09:04 the old "restarted once, now waiting" branch left the bus dead
    /// until the daemon itself was restarted by hand, and a plain reconnect would have fixed both.
    fn maybe_recover(&mut self) {
        let (streak, tz) = {
            let s = self.sh.lock().unwrap();
            (s.health.fail_streak, s.tz_h)
        };
        let now = now_unix();
        if streak < STALL_STREAK || now - self.first_fail < STALL_SECS {
            if streak > 0 {
                std::thread::sleep(Duration::from_millis(50));
            }
            return;
        }
        if now - self.last_recover < self.next_gap {
            std::thread::sleep(Duration::from_millis(50));
            return;
        }
        // which rung: reconnect, or an MCU app restart?
        let hard = self.soft_tried && self.hard_done < MAX_HARD_PER_OUTAGE;
        if !self.stall_said {
            self.stall_said = true;
            let mut s = self.sh.lock().unwrap();
            s.health.note = format!("bus stopped answering at {}; reconnecting the router socket", stamp(self.first_fail, tz));
            let txt = s.health.note.clone();
            s.roster.event(now, None, "stall", txt);
        }
        self.last_recover = now;
        if hard {
            self.hard_done += 1;
            self.soft_tried = false;
            self.next_gap = RECOVER_GAP_S;
            let mut s = self.sh.lock().unwrap();
            s.health.note = format!("reconnect did not help; restarting the MCU app (restart {} of {MAX_HARD_PER_OUTAGE} this outage)", self.hard_done);
            let txt = s.health.note.clone();
            s.roster.event(now, None, "stall", txt);
        } else {
            self.soft_tried = true;
            self.next_gap = if self.hard_done >= MAX_HARD_PER_OUTAGE { RECOVER_GAP_S } else { SOFT_GAP_S };
        }
        let r = self.io.recover(hard);
        let mut s = self.sh.lock().unwrap();
        if hard {
            s.health.recovers += 1;
        } else {
            s.health.soft_recovers += 1;
        }
        let what = if hard { "MCU app restart" } else { "reconnect" };
        let txt = match r {
            Ok(m) => format!("{m}; rescanning ({})", stamp(now_unix(), tz)),
            Err(e) => format!("{what} did not bring the bus back: {e}"),
        };
        s.health.note = txt.clone();
        s.roster.event(now_unix(), None, "recover", txt);
        drop(s);
        self.scan_at = 0;
        self.poll_sent = None;
        self.poll_mode = None;
        self.last_sweep = Instant::now(); // the MCU needs a moment to start sweeping after a restart
    }

    /// Take hold of one servo without a jump: goal = where it is, cap 300, torque on, each read back.
    pub fn hold(&mut self, id: u8, on: bool) {
        let t = now_unix();
        if !on {
            self.torque(&[id], false);
            let mut s = self.sh.lock().unwrap();
            s.held.remove(&id);
            s.roster.event(t, Some(id), "release", format!("ID {id} released: torque off"));
            return;
        }
        let verdict = self.sh.lock().unwrap().roster.verdict(id);
        let r = (|| -> Result<String, String> {
            if verdict != crate::roster::Verdict::Ok {
                return Err(format!("ID {id} is not reading clean ({}): not holding it", verdict.code()));
            }
            let pos = match setid::read(&mut *self.io, id, 56, 2)? {
                setid::Got::Data(d) => d[0] as u16 | (d[1] as u16) << 8,
                _ => return Err(format!("ID {id} did not read clean, not holding it")),
            };
            let cap = [HOLD_CAP as u8, (HOLD_CAP >> 8) as u8];
            let lim = match setid::read(&mut *self.io, id, feetech::REG_TORQUE_LIMIT, 2)? {
                setid::Got::Data(d) => d[0] as u16 | (d[1] as u16) << 8,
                _ => return Err(format!("ID {id}: torque limit did not read back")),
            };
            if lim > HOLD_CAP {
                setid::write(&mut *self.io, id, feetech::REG_TORQUE_LIMIT, &cap)?;
            }
            setid::write(&mut *self.io, id, feetech::REG_ACC, &feetech::move_data(pos, SPEED_MANUAL, ACC_MANUAL))?;
            setid::write(&mut *self.io, id, feetech::REG_TORQUE, &[1])?;
            let lim2 = match setid::read(&mut *self.io, id, feetech::REG_TORQUE_LIMIT, 2)? {
                setid::Got::Data(d) => d[0] as u16 | (d[1] as u16) << 8,
                _ => 9999,
            };
            if lim2 > HOLD_CAP {
                setid::write(&mut *self.io, id, feetech::REG_TORQUE, &[0])?;
                return Err(format!("ID {id}: torque limit reads {lim2}, above {HOLD_CAP}: torque left off"));
            }
            Ok(format!("ID {id} held at {pos} counts, torque limit {lim2} ({} %)", lim2 / 10))
        })();
        let mut s = self.sh.lock().unwrap();
        match r {
            Ok(m) => {
                s.held.insert(id);
                s.roster.event(t, Some(id), "hold", m);
            }
            Err(e) => s.roster.event(t, Some(id), "refused", e),
        }
    }

    /// Latest goal per held servo, clamped to its angle limits, in ONE sync-write.
    pub fn goals(&mut self, goals: &[(u8, u16)]) {
        let mut latest: Vec<(u8, u16)> = Vec::new();
        {
            let mut s = self.sh.lock().unwrap();
            let t = now_unix();
            for &(id, pos) in goals {
                if !s.held.contains(&id) {
                    continue;
                }
                // a slider dragged past the end stops AT the limit (never past it); no limits = no motion at all
                let ee = s.ee_band(id);
                let p = match s.limits.band(id, ee) {
                    Some((lo, hi)) => {
                        let p = pos.clamp(lo, hi);
                        if p != pos && s.limits.should_say(id, t) {
                            s.roster.event(t, Some(id), "limit", format!("ID {id}: goal {pos} is past its limits {lo}..{hi}: stopped at {p}"));
                        }
                        p
                    }
                    None => {
                        if s.limits.should_say(id, t) {
                            let why = s.limits.check(id, pos, ee).unwrap_err();
                            s.roster.event(t, Some(id), "refused", why);
                        }
                        continue;
                    }
                };
                latest.retain(|x| x.0 != id);
                latest.push((id, p));
            }
        }
        if latest.is_empty() {
            return;
        }
        let entries: Vec<(u8, Vec<u8>)> = latest.iter().map(|&(id, p)| (id, feetech::move_data(p, SPEED_MANUAL, ACC_MANUAL))).collect();
        let pkt = feetech::sync_write(feetech::REG_ACC, &entries);
        if self.batch(&[pkt], &[0], 300, Duration::from_millis(100)).is_some() {
            self.sh.lock().unwrap().moves += latest.len() as u64;
        }
    }

    /// Limits from the recorded ends minus 3 deg, kept in the limits file, then written to EEPROM 9/11 (unlock 55,
    /// write, lock) and read back. Refused while the servo is held or its torque is on, and from a wrapped or short
    /// recording. The software clamp takes the new band even if the EEPROM write fails (and says so).
    pub fn save_limits(&mut self, id: u8, ends: Option<(u16, u16)>, who: String, source: String) -> J {
        let t = now_unix();
        let fail = |why: String| J::obj(vec![("ok", J::Bool(false)), ("id", J::n(id as f64)), ("why", J::s(&why))]);
        let (ends, held, tz) = {
            let s = self.sh.lock().unwrap();
            let e = match ends {
                Some(e) => Ok(e),
                None => match s.limits.seen.get(&id) {
                    None => Err(format!("ID {id}: nothing recorded yet: press Record and turn the joint end to end")),
                    Some(sn) if sn.wrapped => Err(format!("ID {id}: the recording crossed 4095 -> 0, so min/max mean nothing: not saved")),
                    Some(sn) => Ok((sn.min, sn.max)),
                },
            };
            (e, s.held.contains(&id), s.tz_h)
        };
        let ends2 = ends.clone();
        let r = (|| -> Result<(Lim, Result<String, String>), String> {
            let (rmin, rmax) = ends2?;
            let (min, max) = limits::from_ends(rmin, rmax)?;
            if held {
                return Err(format!("ID {id} is held by the slider: release it first"));
            }
            match setid::read(&mut *self.io, id, feetech::REG_TORQUE, 1)? {
                setid::Got::Data(d) if d[0] == 0 => {}
                setid::Got::Data(_) => return Err(format!("ID {id} has torque on: turn it off before writing limits")),
                _ => return Err(format!("ID {id} did not answer clean: nothing written")),
            }
            let lim = Lim { min, max, rec_min: rmin, rec_max: rmax, margin: limits::MARGIN, when: stamp(t, tz), who: who.clone(), source: source.clone(), eeprom: String::new() };
            let io = &mut *self.io;
            let ee = (|| -> Result<String, String> {
                setid::write(io, id, feetech::REG_LOCK, &[0])?;
                setid::write(io, id, feetech::REG_MIN_ANGLE, &[min as u8, (min >> 8) as u8])?;
                setid::write(io, id, feetech::REG_MAX_ANGLE, &[max as u8, (max >> 8) as u8])?;
                setid::write(io, id, feetech::REG_LOCK, &[1])?;
                let back = match setid::read(io, id, feetech::REG_MIN_ANGLE, 4)? {
                    setid::Got::Data(d) => (d[0] as u16 | (d[1] as u16) << 8, d[2] as u16 | (d[3] as u16) << 8),
                    _ => return Err("regs 9..12 did not read back".into()),
                };
                if back != (min, max) {
                    return Err(format!("regs 9/11 read back {}/{} instead of {min}/{max}", back.0, back.1));
                }
                match setid::read(io, id, feetech::REG_LOCK, 1)? {
                    setid::Got::Data(d) if d[0] == 1 => {}
                    _ => return Err("EEPROM lock (55) did not read back 1".into()),
                }
                Ok(format!("regs 9/11 read back {min}/{max}, lock 1, at {}", stamp(now_unix(), tz)))
            })();
            Ok((lim, ee))
        })();
        let mut s = self.sh.lock().unwrap();
        match r {
            Err(why) => {
                s.roster.event(t, Some(id), "refused", format!("limits not saved: {why}"));
                fail(why)
            }
            Ok((mut lim, ee)) => {
                let ee_ok = ee.is_ok();
                lim.eeprom = ee.clone().unwrap_or_default();
                let (min, max) = (lim.min, lim.max);
                s.limits.set(id, lim);
                s.limits.said.remove(&id);
                let saved = s.limits.save();
                if let Some(e) = s.roster.rows.get_mut(&id).and_then(|r| r.eeprom.as_mut()) {
                    if ee_ok {
                        e.min_angle = min;
                        e.max_angle = max;
                    }
                }
                let why = match (&ee, &saved) {
                    (Ok(m), Ok(())) => format!("ID {id} limits {min}..{max} (recorded {}..{} minus 3 deg) by {who}: {m}", ends.as_ref().map(|e| e.0).unwrap_or(0), ends.as_ref().map(|e| e.1).unwrap_or(0)),
                    (Err(e), _) => format!("ID {id} limits {min}..{max} saved in the daemon, EEPROM write FAILED: {e}"),
                    (_, Err(e)) => format!("ID {id} limits {min}..{max} written to EEPROM, limits file FAILED: {e}"),
                };
                s.roster.event(t, Some(id), if ee_ok && saved.is_ok() { "limits" } else { "refused" }, why.clone());
                J::obj(vec![
                    ("ok", J::Bool(ee_ok && saved.is_ok())),
                    ("id", J::n(id as f64)),
                    ("min", J::n(min as f64)),
                    ("max", J::n(max as f64)),
                    ("why", J::s(&why)),
                ])
            }
        }
    }

    /// Which way is up. `scan` answers every I2C address that ACKed; otherwise one MPU-6050 sample.
    ///
    /// the robot had no inertial sensor until 2026-10-04, so the walking policy took gyro and gravity from a
    /// simulated duck and could step but not balance. This is the one owner of the IMU RPC, for the same
    /// reason it is the one owner of the servo bus: a second router client calling into the MCU stalled
    /// every method within 14 s on 2026-10-04 18:06.
    ///
    /// Raw LSB go out, not degrees: the scales are reported beside them and the policy does the maths.
    /// One shape for both roads to a sample, so a reader cannot tell them apart by accident - and `via`
    /// says which road it came down, because "the sweep brought it" and "I asked for it" have very
    /// different costs and a page that cannot see the difference cannot report it.
    fn imu_json(&self, b: &[u8], via: &str) -> J {
        let w = |i: usize| ((b[i] as i16) << 8 | b[i + 1] as i16) as f64;
        J::obj(vec![
            ("ok", J::Bool(true)),
            ("accel_lsb", J::Arr(vec![J::n(w(0)), J::n(w(2)), J::n(w(4))])),
            ("temp_lsb", J::n(w(6))),
            ("gyro_lsb", J::Arr(vec![J::n(w(8)), J::n(w(10)), J::n(w(12))])),
            ("gyro_lsb_per_dps", J::n(IMU_GYRO_LSB_PER_DPS)),
            ("accel_lsb_per_g", J::n(IMU_ACCEL_LSB_PER_G)),
            ("addr", J::n(IMU_ADDR as f64)),
            ("bus", J::n(self.imu_bus as f64)),
            // -1 is reachable and honest: the sketch's sampler keeps running across a daemon restart, so a
            // snapshot can arrive in poll_get before THIS daemon has ever called imu_begin and learned the bus.
            ("bus_name", J::s(&if self.imu_bus < 0 {
                "not known to this daemon: the sample came from the sketch's own sampler, which was started \
                 before this daemon connected. /api/imu/scan names the bus.".to_string()
            } else {
                bus_name(self.imu_bus as u8)
            })),
            ("reads", J::n(self.imu_reads as f64)),
            ("via", J::s(via)),
        ])
    }

    pub fn imu(&mut self, scan: bool) -> J {
        if scan {
            return match self.io.imu_scan() {
                Ok(v) => J::obj(vec![
                    ("ok", J::Bool(true)),
                    // [[bus, addr], ...]. The bus matters as much as the address: the same sensor on the
                    // header and on the Qwiic socket is two different buses with one address.
                    ("found", J::Arr(v.iter().map(|(b, a)| J::Arr(vec![J::n(*b as f64), J::n(*a as f64)])).collect())),
                    ("addrs", J::Arr(v.iter().map(|(_, a)| J::n(*a as f64)).collect())),
                    ("bus_names", J::Arr(IMU_BUS_NAMES.iter().map(|n| J::s(n)).collect())),
                    ("why", J::s(&if v.is_empty() {
                        "nothing ACKed on any I2C bus: check the four wires, that VCC is 3.3 V, and that SDA \
                         and SCL are not swapped".to_string()
                    } else {
                        v.iter().map(|(b, a)| format!("0x{a:02x} on {}", bus_name(*b)))
                            .collect::<Vec<_>>().join(", ")
                    })),
                ]),
                Err(e) => J::obj(vec![("ok", J::Bool(false)), ("why", J::s(&e))]),
            };
        }
        // The sweep brings a sample every tick for free. Only fall through to an RPC when it is missing
        // or stale, because that RPC is what costs the bus a third of its rate.
        if let Some((b, age, got)) = self.imu_cache.clone() {
            if got.elapsed() < Duration::from_millis(400) && age < 500 {
                self.imu_reads += 1;
                return self.imu_json(&b, "poll_get");
            }
        }
        if !self.imu_began {
            match self.io.imu_begin(IMU_ADDR) {
                Err(e) => return J::obj(vec![("ok", J::Bool(false)), ("why", J::s(&e))]),
                Ok(v) => {
                    // every value here was read BACK off the chip; ok=0 means no bus carried a working one
                    if v.first() != Some(&1) {
                        let who = v.get(2).copied().unwrap_or(-1);
                        return J::obj(vec![
                            ("ok", J::Bool(false)),
                            ("who_am_i", J::n(who as f64)),
                            ("why", J::s(&format!(
                                "no MPU-6050 at 0x{IMU_ADDR:02x} on any of {} I2C buses: the last WHO_AM_I read \
                                 was {who} (want 104), registers back {v:?}. Scan before trusting the wiring.",
                                IMU_BUS_NAMES.len()
                            ))),
                        ]);
                    }
                    self.imu_bus = v.get(1).copied().unwrap_or(-1) as i32;
                    self.imu_began = true;
                }
            }
        }
        match self.io.imu_read() {
            Err(e) => {
                self.imu_began = false; // make the next call configure it again rather than read a dead chip
                J::obj(vec![("ok", J::Bool(false)), ("why", J::s(&e))])
            }
            Ok(b) if b.len() == 14 => {
                self.imu_reads += 1;
                self.imu_json(&b, "imu_read")
            }
            Ok(_) => {
                self.imu_began = false;
                J::obj(vec![
                    ("ok", J::Bool(false)),
                    ("why", J::s("the sketch read nothing from the MPU-6050: it answered the address and then \
                                  gave no burst, which is what an unplugged SDA after a good begin looks like")),
                ])
            }
        }
    }

    /// Move this servo's own zero to where it is standing now, so present position reads `CENTRE`.
    ///
    /// Why it exists (2026-10-04): the robot's horns went on at any angle, and three servos read past the encoder
    /// wrap (knee 4523, hip pitch 4918, head yaw 5670), where `limits::from_ends` refuses any band and a servo
    /// with no band gets no goal and no torque. The fix is the servo's own position correction, EEPROM reg 31.
    /// `tools/official_zero.py` tried it through the raw door and it did nothing on all 15 servos: the door only
    /// passes RAM 40..49, so the midpoint byte arrived while the EEPROM lock (reg 55) was still 1 and the servo
    /// discarded it. Reg 55 is outside the door on purpose, so this is the narrow door that can open it, and it
    /// is the only place in this daemon that writes reg 31.
    ///
    /// It refuses, and writes nothing, unless: the servo answers clean, torque is OFF, the slider is not holding
    /// it, and it is STILL across two reads. After the write it READS THE POSITION BACK and says FAILED, never ok,
    /// if the servo did not land on CENTRE. A saved band is shifted by the same delta in the same unlocked window,
    /// because a band recorded in the old frame points somewhere else once the frame moves.
    pub fn recentre(&mut self, id: u8, who: String) -> J {
        let t = now_unix();
        let (held, tz, band) = {
            let s = self.sh.lock().unwrap();
            (s.held.contains(&id), s.tz_h, s.limits.map.get(&id).cloned())
        };
        let r = (|| -> Result<(u16, i32, Option<(u16, u16)>, String), String> {
            if held {
                return Err(format!("ID {id} is held by the slider: release it first"));
            }
            match setid::read(&mut *self.io, id, feetech::REG_TORQUE, 1)? {
                setid::Got::Data(d) if d[0] == 0 => {}
                setid::Got::Data(_) => return Err(format!("ID {id} has torque on: turn it off before moving its zero")),
                _ => return Err(format!("ID {id} did not answer clean: nothing written")),
            }
            let read_pos = |io: &mut dyn Io| -> Result<u16, String> {
                match setid::read(io, id, feetech::REG_POS, 2)? {
                    setid::Got::Data(d) => Ok(d[0] as u16 | (d[1] as u16) << 8),
                    _ => Err(format!("ID {id}: position did not read clean")),
                }
            };
            let before = read_pos(&mut *self.io)?;
            std::thread::sleep(Duration::from_millis(STILL_MS));
            let again = read_pos(&mut *self.io)?;
            let moved = (again as i32 - before as i32).abs();
            if moved > STILL_COUNTS as i32 {
                return Err(format!(
                    "ID {id} moved {moved} counts in {STILL_MS} ms: hold it still, a zero taken while it moves is not a zero"
                ));
            }
            let io = &mut *self.io;
            setid::write(io, id, feetech::REG_LOCK, &[0])?;
            let done = (|| -> Result<(u16, Option<(u16, u16)>), String> {
                setid::write(io, id, feetech::REG_TORQUE, &[feetech::MIDPOINT])?;
                std::thread::sleep(Duration::from_millis(MIDPOINT_MS));
                setid::write(io, id, feetech::REG_TORQUE, &[0])?;
                let after = read_pos(io)?;
                let off = (after as i32 - CENTRE as i32).abs();
                if off > CENTRE_TOL as i32 {
                    return Err(format!(
                        "position reads {after} after the write, {off} counts off {CENTRE}: the servo did not take it"
                    ));
                }
                // the band was measured in the old frame; move it by what the frame moved, or it points elsewhere
                let shifted = match band.as_ref() {
                    None => None,
                    Some(b) => {
                        let d = after as i32 - before as i32;
                        let (lo, hi) = (b.min as i32 + d, b.max as i32 + d);
                        if lo < 0 || hi > 4095 {
                            return Err(format!(
                                "the saved band {}..{} moves to {lo}..{hi}, outside one turn: record it again instead",
                                b.min, b.max
                            ));
                        }
                        let (lo, hi) = (lo as u16, hi as u16);
                        setid::write(io, id, feetech::REG_MIN_ANGLE, &[lo as u8, (lo >> 8) as u8])?;
                        setid::write(io, id, feetech::REG_MAX_ANGLE, &[hi as u8, (hi >> 8) as u8])?;
                        match setid::read(io, id, feetech::REG_MIN_ANGLE, 4)? {
                            setid::Got::Data(d2) => {
                                let back = (d2[0] as u16 | (d2[1] as u16) << 8, d2[2] as u16 | (d2[3] as u16) << 8);
                                if back != (lo, hi) {
                                    return Err(format!("regs 9/11 read back {}/{} instead of {lo}/{hi}", back.0, back.1));
                                }
                            }
                            _ => return Err("regs 9..12 did not read back".into()),
                        }
                        Some((lo, hi))
                    }
                };
                Ok((after, shifted))
            })();
            // the lock goes back on whatever happened above, so a failure never leaves EEPROM open
            let relock = setid::write(io, id, feetech::REG_LOCK, &[1]);
            let (after, shifted) = done?;
            relock?;
            match setid::read(io, id, feetech::REG_LOCK, 1)? {
                setid::Got::Data(d) if d[0] == 1 => {}
                _ => return Err("EEPROM lock (55) did not read back 1 after the write".into()),
            }
            let ofs = match setid::read(io, id, feetech::REG_OFS, 2)? {
                setid::Got::Data(d) => d[0] as i32 | (d[1] as i32) << 8,
                _ => return Err("reg 31 (position correction) did not read back".into()),
            };
            Ok((after, ofs, shifted, stamp(now_unix(), tz)))
        })();
        let mut s = self.sh.lock().unwrap();
        match r {
            Err(why) => {
                s.roster.event(t, Some(id), "refused", format!("zero not moved: {why}"));
                J::obj(vec![("ok", J::Bool(false)), ("id", J::n(id as f64)), ("why", J::s(&why))])
            }
            Ok((after, ofs, shifted, when)) => {
                if let Some((lo, hi)) = shifted {
                    if let Some(l) = s.limits.map.get_mut(&id) {
                        l.min = lo;
                        l.max = hi;
                        l.source = format!("{} (band moved with the zero on {when})", l.source);
                        l.eeprom = format!("regs 9/11 read back {lo}/{hi} at {when}");
                    }
                    s.limits.save().ok();
                    if let Some(e) = s.roster.rows.get_mut(&id).and_then(|r| r.eeprom.as_mut()) {
                        e.min_angle = lo;
                        e.max_angle = hi;
                    }
                }
                let why = format!(
                    "ID {id} zero moved by {who}: present position reads {after}, correction (reg 31) {ofs}, band {}, at {when}",
                    shifted.map(|(a, b)| format!("{a}..{b}")).unwrap_or_else(|| "none saved".into())
                );
                s.roster.event(t, Some(id), "recentre", why.clone());
                J::obj(vec![
                    ("ok", J::Bool(true)),
                    ("id", J::n(id as f64)),
                    ("pos", J::n(after as f64)),
                    ("ofs", J::n(ofs as f64)),
                    ("min", shifted.map(|(a, _)| J::n(a as f64)).unwrap_or(J::Nil)),
                    ("max", shifted.map(|(_, b)| J::n(b as f64)).unwrap_or(J::Nil)),
                    ("why", J::s(&why)),
                ])
            }
        }
    }

    /// Every 250 ms: if the face file changed (110 bytes: 104 pixels 0..7 row-major 13x8, then LED3 r,g,b u16 LE
    /// 0..4095) and is under 3 s old, send mx_draw; set_led3_color only when the colour changed. Spec from
    /// 18:08 (sim/unoq_matrix_bus.py writes the file 4x/s).
    /// Every 5 s, read the sketch's own stuck counters. They are the measurement that says whether a wedge was
    /// the UART TX ring refusing to drain or something else.
    pub fn stuck_tick(&mut self) {
        if self.stuck_at.is_some_and(|t| t.elapsed() < Duration::from_secs(5)) {
            return;
        }
        self.stuck_at = Some(Instant::now());
        if let Ok((tx, lock)) = self.io.stuck() {
            let mut s = self.sh.lock().unwrap();
            if tx > s.health.tx_stuck || lock > s.health.lock_lost {
                let t = now_unix();
                s.roster.event(t, None, "bus", format!("the MCU refused {tx} frame(s) because its UART TX ring would not drain, and gave up on the bus mutex {lock} time(s)"));
            }
            s.health.tx_stuck = tx;
            s.health.lock_lost = lock;
            s.health.tx_stuck_max = s.health.tx_stuck_max.max(tx);
            s.health.lock_lost_max = s.health.lock_lost_max.max(lock);
        }
    }

    pub fn matrix_tick(&mut self) {
        let Some(path) = self.matrix_path.clone() else { return };
        if self.matrix_checked.is_some_and(|t| t.elapsed() < Duration::from_millis(250)) {
            return;
        }
        self.matrix_checked = Some(Instant::now());
        let Ok(meta) = std::fs::metadata(&path) else { return };
        let Ok(mtime) = meta.modified() else { return };
        if Some(mtime) == self.matrix_mtime || mtime.elapsed().map(|d| d.as_secs_f64() > 3.0).unwrap_or(true) {
            return;
        }
        self.matrix_mtime = Some(mtime);
        let Ok(b) = std::fs::read(&path) else { return };
        let t = now_unix();
        if b.len() != 110 || b[..104].iter().any(|&p| p > 7) {
            if t - self.matrix_said > 60.0 {
                self.matrix_said = t;
                self.sh.lock().unwrap().roster.event(t, None, "refused", format!("matrix frame {path}: {} bytes or a pixel above 7: not drawn", b.len()));
            }
            return;
        }
        let le = |k: usize| (b[k] as u16 | (b[k + 1] as u16) << 8).min(4095);
        let rgb = (le(104), le(106), le(108));
        let send_rgb = if self.matrix_rgb != Some(rgb) { Some(rgb) } else { None };
        match self.io.matrix(Some(&b[..104]), send_rgb) {
            Ok(()) => {
                if send_rgb.is_some() {
                    self.matrix_rgb = Some(rgb);
                }
                let mut s = self.sh.lock().unwrap();
                s.matrix.0 += 1;
                s.matrix.1 = t;
            }
            Err(e) => {
                self.sh.lock().unwrap().matrix.2 = format!("{} {e}", stamp(t, 8));
                if t - self.matrix_said > 60.0 {
                    self.matrix_said = t;
                    self.sh.lock().unwrap().roster.event(t, None, "matrix", format!("matrix frame not sent: {e}"));
                }
            }
        }
    }

    /// The raw caller went quiet (crashed, killed, network) with servos it torqued still on: torque them off.
    pub fn raw_watchdog(&mut self) {
        if self.raw_on.is_empty() || self.raw_last.map(|t| t.elapsed() < RAW_QUIET).unwrap_or(false) {
            return;
        }
        let ids: Vec<u8> = std::mem::take(&mut self.raw_on).into_iter().collect();
        self.torque(&ids, false);
        let t = now_unix();
        let mut s = self.sh.lock().unwrap();
        let who = s.raw.0.clone();
        s.roster.event(t, None, "watchdog", format!("{who} sent nothing for {} ms with torque on: torque off {ids:?}", RAW_QUIET.as_millis()));
        s.why = format!("raw caller {who} went quiet: torque off {ids:?} at {}", hhmmss());
    }

    /// One outside batch through the gate, executed as one RPC; replies hex per caller frame.
    pub fn raw(&mut self, frames: &[Vec<u8>], expects: &[usize], wait_us: i64, cap: u16, who: &str) -> J {
        let t = now_unix();
        let cap = cap.min(RAW_CAP_MAX);
        let gated = {
            let s = self.sh.lock().unwrap();
            let band = |id: u8| s.limits.band(id, s.ee_band(id));
            crate::rawgate::gate(frames, expects, &band, cap)
        };
        let g = match gated {
            Ok(g) => g,
            Err(why) => {
                let mut s = self.sh.lock().unwrap();
                if s.limits.should_say(254, t) {
                    s.roster.event(t, None, "refused", format!("raw batch from {who}: {why}"));
                }
                return J::obj(vec![("ok", J::Bool(false)), ("why", J::s(&why))]);
            }
        };
        let t0 = Instant::now();
        let wait = wait_us.clamp(100, 5000);
        // one router timeout (os error 11) must not end a policy run: the same frames once more
        let out = self.batch(&g.frames, &g.expects, wait, Duration::from_millis(400)).or_else(|| self.batch(&g.frames, &g.expects, wait, Duration::from_millis(400)));
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        if out.is_some() {
            self.raw_on.extend(g.on.iter().copied());
            for i in &g.off {
                self.raw_on.remove(i);
            }
        } else {
            self.raw_on.extend(g.on.iter().copied()); // unknown whether it landed: the watchdog turns it off
        }
        self.raw_last = Some(Instant::now());
        let mut s = self.sh.lock().unwrap();
        s.raw = (who.to_string(), t, s.raw.2 + 1, s.raw.3 + g.clamped.len() as u64, cap);
        for &(id, want, got) in &g.clamped {
            if s.limits.should_say(id, t) {
                s.roster.event(t, Some(id), "limit", format!("{who}: goal {want} for ID {id} is past its limits: sent {got}"));
            }
        }
        match out {
            None => J::obj(vec![("ok", J::Bool(false)), ("why", J::s(&format!("bus: {}", s.health.last_error)))]),
            Some(raws) => {
                let hex = |b: &[u8]| b.iter().map(|x| format!("{x:02x}")).collect::<String>();
                let replies: Vec<J> = g.keep.iter().map(|&k| J::s(&hex(raws.get(k).map(|v| v.as_slice()).unwrap_or(&[])))).collect();
                J::obj(vec![
                    ("ok", J::Bool(true)),
                    ("replies", J::Arr(replies)),
                    ("clamped", J::n(g.clamped.len() as f64)),
                    ("ms", J::n((ms * 100.0).round() / 100.0)),
                ])
            }
        }
    }

    pub fn setid(&mut self, old: u8, new: u8, joint: String, who: String) -> J {
        let (mut reg, on_bus, tz) = {
            let mut s = self.sh.lock().unwrap();
            s.busy = Some(format!("Changing ID {old} to {new}"));
            let on: Vec<u8> = s.roster.rows.values().filter(|r| !r.stale(now_unix())).map(|r| r.id).collect();
            (s.registry.clone(), on, s.tz_h)
        };
        let req = setid::Req { old, new, joint, who };
        let r = setid::set_id(&mut *self.io, &mut reg, &on_bus, &req, &stamp(now_unix(), tz))
            .unwrap_or_else(|e| J::obj(vec![("ok", J::Bool(false)), ("written", J::Nil), ("why", J::s(&format!("bus error during set-ID: {e}")))]));
        let mut s = self.sh.lock().unwrap();
        s.registry = reg;
        s.busy = None;
        let ok = r.get("ok") == Some(&J::Bool(true));
        let written = r.get("written") == Some(&J::Bool(true));
        let why = r.get("why").and_then(|w| w.as_str()).unwrap_or("").to_string();
        let t = now_unix();
        if written {
            s.roster.forget(old);
            s.roster.forget(new);
        }
        s.roster.event(t, Some(new), if ok { "setid" } else { "refused" }, if ok { why } else { format!("set ID {old} -> {new}: {why}") });
        s.last_setid = Some(r.clone());
        drop(s);
        self.scan_at = 0;
        r
    }
}

pub fn run(io: Box<dyn Io>, sh: Arc<Mutex<Shared>>, rx: std::sync::mpsc::Receiver<Cmd>, matrix_path: Option<String>) {
    let mut eng = Engine::new(io, sh.clone());
    eng.matrix_path = matrix_path;
    let ids = eng.ids.clone();
    let mut lat_samples: Vec<f64> = Vec::with_capacity(64);
    let mut stop_pending: Option<Instant> = None;
    let mut pending: std::collections::VecDeque<Cmd> = Default::default();
    let mut last_roster = Instant::now() - Duration::from_secs(1);
    loop {
        let tick = Instant::now();
        if !eng.begin_tick() {
            std::thread::sleep(Duration::from_millis(500));
            continue;
        }

        // --- commands first: stop-all can never wait behind a wave or a poll ---
        let mut stop_all = false;
        let mut stop_wave = false;
        let mut wave_start: Option<(f64, f64)> = None;
        let mut moves: Vec<(u8, f64)> = Vec::new();
        let mut torques: Vec<(Option<u8>, bool)> = Vec::new();
        let mut setids = Vec::new();
        let mut holds: Vec<(u8, bool)> = Vec::new();
        let mut goals: Vec<(u8, u16)> = Vec::new();
        let mut saves = Vec::new();
        let mut recentres = Vec::new();
        let mut imus = Vec::new();
        let mut raws = Vec::new();
        let mut n = 0u64;
        while let Some(cmd) = pending.pop_front().or_else(|| rx.try_recv().ok()) {
            n += 1;
            match cmd {
                Cmd::StopAll => stop_all = true,
                Cmd::StopWave => stop_wave = true,
                Cmd::Wave { amp_deg, period } => wave_start = Some((amp_deg, period)),
                Cmd::Move { id, deg } => moves.push((id, deg)),
                Cmd::Torque { id, on } => torques.push((id, on)),
                Cmd::SetId { old, new, joint, who, reply } => setids.push((old, new, joint, who, reply)),
                Cmd::Hold { id, on } => holds.push((id, on)),
                Cmd::Goal { id, pos } => goals.push((id, pos)),
                Cmd::RecordLimits { id } => {
                    let mut s = sh.lock().unwrap();
                    s.limits.reset_seen(id);
                    s.roster.event(now_unix(), Some(id), "record", format!("ID {id}: recording its range from now: turn it by hand end to end, torque off"));
                }
                Cmd::SaveLimits { id, ends, who, source, reply } => saves.push((id, ends, who, source, reply)),
                Cmd::Recentre { id, who, reply } => recentres.push((id, who, reply)),
                Cmd::Imu { scan, reply } => imus.push((scan, reply)),
                Cmd::Raw { frames, expects, wait_us, cap, who, reply } => raws.push((frames, expects, wait_us, cap, who, reply)),
            }
        }
        sh.lock().unwrap().cmds += n;
        let all: Vec<u8> = {
            let s = sh.lock().unwrap();
            let mut v = s.roster.ids();
            for i in &ids {
                if !v.contains(i) {
                    v.push(*i);
                }
            }
            v
        };
        if stop_all {
            sh.lock().unwrap().held.clear();
            eng.phase_t0 = None;
            eng.torque(&all, false);
            stop_pending = Some(Instant::now());
            let mut s = sh.lock().unwrap();
            s.wave_on = false;
            s.mode = "idle";
            s.cooling = false;
            s.why = format!("STOP ALL: torque off at {}", hhmmss());
        }
        if stop_wave && !stop_all && eng.phase_t0.is_some() {
            eng.torque(&ids, false);
            eng.phase_t0 = None;
            let mut s = sh.lock().unwrap();
            s.wave_on = false;
            s.mode = "idle";
            s.why = format!("wave stopped at {}", hhmmss());
        }
        for (id, on) in &torques {
            let list = id.map(|i| vec![i]).unwrap_or_else(|| all.clone());
            eng.torque(&list, *on);
            if !on {
                let mut s = sh.lock().unwrap();
                for i in &list {
                    s.held.remove(i);
                }
            }
        }
        for (id, on) in holds {
            eng.hold(id, on);
        }
        if !stop_all {
            eng.goals(&goals);
        }
        let had_raw = !raws.is_empty();
        if stop_all {
            eng.raw_on.clear();
            if let Ok(mut p) = crate::policy::POLICY.try_lock() {
                p.stop();
            }
        }
        for (frames, expects, wait_us, cap, who, reply) in raws {
            if stop_all {
                reply.send(J::obj(vec![("ok", J::Bool(false)), ("why", J::s("STOP ALL"))])).ok();
                continue;
            }
            let r = eng.raw(&frames, &expects, wait_us, cap, &who);
            reply.send(r).ok();
        }
        eng.raw_watchdog();
        for (id, ends, who, source, reply) in saves {
            let r = eng.save_limits(id, ends, who, source);
            if let Some(tx) = reply {
                tx.send(r).ok();
            }
        }
        for (id, who, reply) in recentres {
            let r = eng.recentre(id, who);
            if let Some(tx) = reply {
                tx.send(r).ok();
            }
        }
        for (scan, reply) in imus {
            let r = eng.imu(scan);
            reply.send(r).ok();
        }
        for (old, new, joint, who, reply) in setids {
            let r = eng.setid(old, new, joint, who);
            if let Some(tx) = reply {
                tx.send(r).ok();
            }
        }
        if let Some((amp, period)) = wave_start {
            if ids.is_empty() {
                sh.lock().unwrap().why = "no motion IDs (--ids): the wave is off on this daemon".into();
            } else {
                if eng.phase_t0.is_none() {
                    eng.phase_t0 = Some(Instant::now());
                    for (k, &id) in ids.iter().enumerate() {
                        eng.next_due.insert(id, tick + Duration::from_secs_f64(k as f64 * period / ids.len() as f64));
                        eng.side.insert(id, false);
                    }
                }
                let mut s = sh.lock().unwrap();
                s.wave_amp_deg = amp;
                s.wave_period = period;
                s.wave_started.get_or_insert_with(hhmmss);
                s.wave_on = true;
                s.mode = "wave";
                s.why.clear();
                s.cooling = false;
            }
        }
        for (id, deg) in &moves {
            if !ids.contains(id) {
                continue; // motion only on the --ids set
            }
            if eng.phase_t0.is_some() {
                eng.phase_t0 = None;
                let mut s = sh.lock().unwrap();
                s.wave_on = false;
                s.mode = "manual";
                s.why = format!("manual move took over at {}", hhmmss());
            }
            let from = sh.lock().unwrap().roster.rows.get(id).and_then(|r| r.tele.as_ref()).map(|t| t.pos).unwrap_or((LO + HI) / 2);
            // 0..90 deg spans THIS servo's limits (the old global 1024..3072 band would drive the jaw, resting at 288,
            // far past its end stop); outside 0..90 or no limits = refused
            let target = {
                let mut s = sh.lock().unwrap();
                let ee = s.ee_band(*id);
                let t = match s.limits.band(*id, ee) {
                    Some(b) if (0.0..=90.0).contains(deg) => Ok(band_counts(b, *deg)),
                    Some((lo, hi)) => Err(format!("ID {id}: move to {deg} deg refused: 0..90 spans its limits {lo}..{hi}")),
                    None => Err(s.limits.check(*id, deg_to_counts(*deg), ee).unwrap_err()),
                };
                if let Err(why) = &t {
                    s.roster.event(now_unix(), Some(*id), "refused", why.clone());
                }
                t
            };
            let Ok(target) = target else { continue };
            if eng.sync_move(&[(*id, target)]).is_some() {
                sh.lock().unwrap().moves += 1;
            }
            eng.probe = Some((*id, from, Instant::now()));
            eng.probe_iters = 0;
            let mut s = sh.lock().unwrap();
            if s.mode == "idle" {
                s.mode = "manual";
            }
        }

        // --- wave scheduling ---
        let wave_on = sh.lock().unwrap().wave_on;
        if wave_on {
            let (amp_deg, period) = {
                let s = sh.lock().unwrap();
                (s.wave_amp_deg, s.wave_period)
            };
            let t0 = eng.phase_t0.unwrap_or_else(Instant::now);
            let mut due: Vec<(u8, u16)> = Vec::new();
            let bands: HashMap<u8, Option<(u16, u16)>> = {
                let s = sh.lock().unwrap();
                ids.iter().map(|&id| (id, s.limits.band(id, s.ee_band(id)))).collect()
            };
            for &id in &ids {
                let next = eng.next_due.entry(id).or_insert(t0);
                if tick >= *next {
                    let side = eng.side.entry(id).or_insert(false);
                    *side = !*side;
                    *next += Duration::from_secs_f64(period / 2.0);
                    // centred in this servo's own limits, the swing never wider than they are; no limits = it sits out
                    if let Some(pos) = bands[&id].map(|b| wave_counts(b, amp_deg, *side)) {
                        due.push((id, pos));
                    }
                }
            }
            if !due.is_empty() && eng.sync_move(&due).is_some() {
                sh.lock().unwrap().moves += due.len() as u64;
            }
        }

        // --- latency probe or roster ---
        if let Some((id, from, t0)) = eng.probe {
            eng.probe_iters += 1;
            if let Some(pos) = eng.read_pos(id) {
                if pos.abs_diff(from) >= 8 {
                    let ms = t0.elapsed().as_secs_f64() * 1000.0;
                    lat_samples.push(ms);
                    if lat_samples.len() > 50 {
                        lat_samples.remove(0);
                    }
                    let mut v = lat_samples.clone();
                    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
                    let mut s = sh.lock().unwrap();
                    s.lat.last_ms = Some(ms);
                    s.lat.n += 1;
                    s.lat.median_ms = Some(v[v.len() / 2]);
                    s.lat.min_ms = v.first().copied();
                    s.lat.max_ms = v.last().copied();
                    eng.probe = None;
                }
            }
            if eng.probe_iters > 40 {
                eng.probe = None;
            }
        } else if !had_raw || last_roster.elapsed() >= Duration::from_millis(18) {
            // an outside controller's batch woke the loop early: the roster keeps its own 20 ms rhythm
            last_roster = Instant::now();
            eng.roster_tick();
            eng.tick_n += 1;
            safety(&mut eng, &sh, &ids, &mut stop_pending, tick);
        }
        if eng.begun && sh.lock().unwrap().health.fail_streak == 0 {
            eng.matrix_tick();
            eng.stuck_tick();
        }
        eng.maybe_recover();

        eng.hz_n += 1;
        if eng.hz_t0.elapsed() >= Duration::from_secs(2) {
            let hz = eng.hz_n as f64 / eng.hz_t0.elapsed().as_secs_f64();
            let mut g = sh.lock().unwrap();
            g.health.hz = hz;
            g.health.rpc_max_ms = 0.0;
            drop(g);
            eng.hz_n = 0;
            eng.hz_t0 = Instant::now();
        }
        // sleep out the 20 ms tick, but wake at once for a command (a policy batch must not wait a whole tick)
        let left = Duration::from_millis(20).saturating_sub(last_roster.elapsed().min(tick.elapsed()));
        if let Ok(c) = rx.recv_timeout(left) {
            pending.push_back(c);
        }
    }
}

/// Hot -> limp and cool; sag twice in a row -> stop and stay stopped; stop-all verified by telemetry.
fn safety(eng: &mut Engine, sh: &Arc<Mutex<Shared>>, ids: &[u8], stop_pending: &mut Option<Instant>, tick: Instant) {
    let mut s = sh.lock().unwrap();
    let motion: Vec<&ServoState> = s.servos.iter().filter(|v| v.online).collect();
    let temps: Vec<u8> = motion.iter().filter_map(|v| v.t.as_ref().map(|t| t.temp)).collect();
    let volts: Vec<f32> = motion.iter().filter_map(|v| v.t.as_ref().map(|t| t.volt)).collect();
    let mut why = String::new();
    if let Some(mx) = temps.iter().max() {
        if *mx >= TEMP_MAX {
            why = format!("hot: {mx} C");
        }
    }
    if volts.iter().any(|&v| v < V_MIN) {
        eng.sag += 1;
        if eng.sag >= 2 {
            why = format!("voltage sag: {:.1} V", volts.iter().cloned().fold(f32::MAX, f32::min));
        }
    } else {
        eng.sag = 0;
    }
    if !why.is_empty() && s.wave_on {
        s.wave_on = false;
        s.mode = "idle";
        s.why = format!("{} at {}", why, hhmmss());
        s.cooling = why.starts_with("hot");
        drop(s);
        eng.phase_t0 = None;
        eng.torque(ids, false);
        return;
    }
    if s.cooling && temps.iter().max().is_some_and(|mx| *mx <= TEMP_RESUME) && !s.wave_on {
        s.cooling = false;
        s.wave_on = true;
        s.mode = "wave";
        s.why = format!("cooled to {} C, resumed at {}", temps.iter().max().unwrap(), hhmmss());
        if eng.phase_t0.is_none() {
            eng.phase_t0 = Some(Instant::now());
            let period = s.wave_period;
            for (k, &id) in ids.iter().enumerate() {
                eng.next_due.insert(id, tick + Duration::from_secs_f64(k as f64 * period / ids.len() as f64));
                eng.side.insert(id, false);
            }
        }
    }
    if let Some(t0) = *stop_pending {
        let now = now_unix();
        let online: Vec<_> = s.roster.rows.values().filter(|r| !r.stale(now)).collect();
        if !online.is_empty() && online.iter().all(|r| r.tele.as_ref().map(|t| !t.torque).unwrap_or(true)) {
            s.stop_ms = Some(t0.elapsed().as_secs_f64() * 1000.0);
            *stop_pending = None;
        }
    }
}

/// 0..90 deg onto one servo's limit band.
pub fn band_counts((lo, hi): (u16, u16), deg: f64) -> u16 {
    (lo as f64 + deg.clamp(0.0, 90.0) / 90.0 * (hi - lo) as f64).round().clamp(lo as f64, hi as f64) as u16
}

/// One wave end for a servo with band (lo, hi): centre +- amp (deg on the old scale), never past lo/hi.
pub fn wave_counts((lo, hi): (u16, u16), amp_deg: f64, side: bool) -> u16 {
    let center = (lo as f64 + hi as f64) / 2.0;
    let amp = (counts_per_deg() * amp_deg).min((hi - lo) as f64 / 2.0);
    (if side { center + amp } else { center - amp }).round().clamp(lo as f64, hi as f64) as u16
}

pub fn deg_to_counts(deg: f64) -> u16 {
    (LO as f64 + deg / 90.0 * (HI - LO) as f64).round().clamp(0.0, 4095.0) as u16
}

fn counts_per_deg() -> f64 {
    (HI - LO) as f64 / 90.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::{split_frames, FakeBus};

    #[test]
    fn degree_mapping() {
        assert_eq!(deg_to_counts(0.0), LO);
        assert_eq!(deg_to_counts(90.0), HI);
        assert_eq!(deg_to_counts(45.0), 2048);
    }

    #[test]
    fn split_layout() {
        let raw = [3u8, 10, 20, 30, 0, 2, 200, 201];
        assert_eq!(split_frames(&raw), vec![vec![10, 20, 30], vec![], vec![200, 201]]);
    }

    #[test]
    fn stamp_is_china_time() {
        // 2026-10-04 07:02:10 UTC = 15:02:10 CST
        assert_eq!(stamp(1791097330.0, 8), "2026-10-04 15:02:10 CST");
    }

    // ---- /api/imu: which way is up -----------------------------------------------------------------
    // the robot had no inertial sensor until 2026-10-04, so every one of these is about not letting a missing or
    // half-working sensor look like a working one. A policy fed a quiet default would walk off a table.

    fn with_imu(v: [i16; 7]) -> Engine {
        on_bus(0, v)
    }

    /// The same sensor, wired to a different I2C bus. 0 = Wire, the header pins we used on 2026-10-04;
    /// 1 = Wire1, the Qwiic socket he had no cable for.
    fn on_bus(b: u8, v: [i16; 7]) -> Engine {
        let mut bus = FakeBus::new(&[(13, 2048)]);
        bus.imu = Some((IMU_ADDR, v));
        bus.imu_on_bus = b;
        engine(bus)
    }

    #[test]
    fn imu_reads_a_sample_in_raw_counts() {
        // flat and still: 1 g straight down on Z at 8192 LSB per g, no rotation
        let mut e = with_imu([0, 0, 8192, 0, 0, 0, 0]);
        let r = e.imu(false);
        assert!(ok(&r), "{}", why(&r));
        match r.get("accel_lsb") {
            Some(J::Arr(a)) => assert_eq!(a[2], J::n(8192.0)),
            _ => panic!("no accel"),
        }
        assert_eq!(r.get("accel_lsb_per_g").and_then(|x| x.i64()), Some(8192));
        assert_eq!(r.get("gyro_lsb_per_dps"), Some(&J::n(65.5)));
    }

    #[test]
    fn imu_keeps_the_sign_of_a_negative_reading() {
        // upside down: the sign is the whole point of a gravity vector, and a u16 would lose it
        let mut e = with_imu([0, 0, -8192, 0, -1310, 0, 0]);
        let r = e.imu(false);
        match (r.get("accel_lsb"), r.get("gyro_lsb")) {
            (Some(J::Arr(a)), Some(J::Arr(g))) => {
                assert_eq!(a[2], J::n(-8192.0));
                assert_eq!(g[0], J::n(-1310.0));
            }
            _ => panic!("no sample"),
        }
    }

    #[test]
    fn imu_says_not_ok_when_the_chip_is_not_there() {
        let mut e = engine(FakeBus::new(&[(13, 2048)]));   // no imu on this bus
        let r = e.imu(false);
        assert!(!ok(&r), "a missing IMU answered ok");
        assert!(why(&r).contains("WHO_AM_I"), "{}", why(&r));
        assert!(r.get("bus").is_none(), "a sensor that was never found was given a bus");
    }

    #[test]
    fn imu_never_invents_a_sample() {
        let mut e = engine(FakeBus::new(&[(13, 2048)]));
        let r = e.imu(false);
        assert!(r.get("accel_lsb").is_none(), "a failed read still carried numbers");
        assert!(r.get("gyro_lsb").is_none());
    }

    #[test]
    fn imu_reports_a_read_that_fails_after_a_good_begin() {
        // SDA pulled out after configuring: the address still ACKs, the burst does not come
        let mut bus = FakeBus::new(&[(13, 2048)]);
        bus.imu = Some((IMU_ADDR, [0, 0, 8192, 0, 0, 0, 0]));
        bus.imu_read_fails = true;
        let mut e = engine(bus);
        let r = e.imu(false);
        assert!(!ok(&r));
        assert!(why(&r).contains("unplugged SDA"), "{}", why(&r));
    }

    #[test]
    fn imu_finds_the_sensor_on_the_header_bus() {
        // we had no Qwiic cable on 2026-10-04 and wired it to D20/D21, which is Wire, not Wire1
        let mut e = on_bus(0, [0, 0, 8192, 0, 0, 0, 0]);
        let r = e.imu(false);
        assert!(ok(&r), "{}", why(&r));
        assert_eq!(r.get("bus").and_then(|x| x.i64()), Some(0));
        assert!(r.get("bus_name").and_then(|x| x.as_str()).unwrap().contains("header"), "{:?}", r.get("bus_name"));
    }

    #[test]
    fn imu_finds_the_same_sensor_on_the_qwiic_bus() {
        // and when the Qwiic cable turns up and it moves, nothing else has to change
        let mut e = on_bus(1, [0, 0, 8192, 0, 0, 0, 0]);
        let r = e.imu(false);
        assert!(ok(&r), "{}", why(&r));
        assert_eq!(r.get("bus").and_then(|x| x.i64()), Some(1));
        assert!(r.get("bus_name").and_then(|x| x.as_str()).unwrap().contains("Qwiic"));
    }

    #[test]
    fn imu_reads_the_same_sample_whichever_bus_it_is_on() {
        for b in [0u8, 1, 2] {
            let mut e = on_bus(b, [0, 0, -8192, 0, 655, 0, 0]);
            let r = e.imu(false);
            assert!(ok(&r), "bus {b}: {}", why(&r));
            match r.get("accel_lsb") {
                Some(J::Arr(a)) => assert_eq!(a[2], J::n(-8192.0), "bus {b}"),
                _ => panic!("bus {b}: no accel"),
            }
        }
    }

    #[test]
    fn imu_scan_says_which_bus_as_well_as_which_address() {
        let mut e = on_bus(1, [0, 0, 8192, 0, 0, 0, 0]);
        let r = e.imu(true);
        assert!(ok(&r));
        match r.get("found") {
            Some(J::Arr(a)) => assert_eq!(a[0], J::Arr(vec![J::n(1.0), J::n(IMU_ADDR as f64)])),
            _ => panic!("no found list"),
        }
        assert!(why(&r).contains("Qwiic"), "{}", why(&r));
    }

    #[test]
    fn imu_names_every_bus_it_looked_on_when_it_finds_nothing() {
        let mut e = engine(FakeBus::new(&[(13, 2048)]));
        let r = e.imu(false);
        assert!(!ok(&r));
        assert!(why(&r).contains("any of 3 I2C buses"), "{}", why(&r));
    }

    #[test]
    fn imu_scan_says_what_actually_answered() {
        let mut e = with_imu([0, 0, 8192, 0, 0, 0, 0]);
        let r = e.imu(true);
        assert!(ok(&r));
        match r.get("addrs") {
            Some(J::Arr(a)) => assert_eq!(a, &vec![J::n(IMU_ADDR as f64)]),
            _ => panic!("no addrs"),
        }
    }

    #[test]
    fn imu_scan_on_an_empty_bus_says_so_rather_than_nothing() {
        let mut e = engine(FakeBus::new(&[(13, 2048)]));
        let r = e.imu(true);
        assert!(ok(&r), "a scan that ran is a measurement even when it found nothing");
        assert!(why(&r).contains("nothing ACKed"), "{}", why(&r));
    }

    #[test]
    fn imu_configures_once_and_then_only_reads() {
        let mut e = with_imu([0, 0, 8192, 0, 0, 0, 0]);
        assert!(ok(&e.imu(false)));
        assert!(ok(&e.imu(false)));
        assert_eq!(e.imu(false).get("reads").and_then(|x| x.i64()), Some(3));
    }

    // ---- the IMU rides the sweep -----------------------------------------------------------------
    // Measured on the board 2026-10-05: /api/imu polled at 10 Hz dropped the sweep from 49.7 Hz to 32.4 Hz,
    // about 34 ms of round trip per imu_read RPC. The sketch now puts the snapshot in the poll_get reply and
    // the daemon serves /api/imu from that, so the policy's 50 Hz costs the bus nothing.

    /// A poll-capable fake running the piggyback sketch, plus the counter of imu_read RPCs it was asked for.
    fn with_imu_in_poll(age_ms: u16, trailer: bool) -> (Engine, std::sync::Arc<std::sync::atomic::AtomicU32>) {
        let mut bus = FakeBus::new(&[(13, 2048)]);
        bus.poll_capable = true;
        bus.imu = Some((IMU_ADDR, [0, 0, 8192, 0, -1310, 0, 0]));
        bus.imu_in_poll = trailer;
        bus.imu_snap_age_ms = age_ms;
        let reads = bus.imu_read_rpcs.clone();
        (engine(bus), reads)
    }

    fn rpcs(c: &std::sync::Arc<std::sync::atomic::AtomicU32>) -> u32 {
        c.load(std::sync::atomic::Ordering::Relaxed)
    }

    #[test]
    fn the_imu_sample_rides_the_sweep_and_costs_no_rpc() {
        let (mut e, reads) = with_imu_in_poll(20, true);
        assert!(ok(&e.imu(false)), "the first call still configures the chip");
        let first = rpcs(&reads);
        scan(&mut e, 130);
        let r = e.imu(false);
        assert!(ok(&r), "{}", why(&r));
        assert_eq!(r.get("via").and_then(|x| x.as_str()), Some("poll_get"), "{}", crate::json::dump(&r));
        assert_eq!(rpcs(&reads), first, "the bus was asked for a sample it already had");
    }

    #[test]
    fn the_sample_that_rides_the_sweep_is_the_same_sample() {
        // a piggybacked read that quietly lost a sign or a byte order would fly the policy into the floor
        let (mut e, _) = with_imu_in_poll(20, true);
        assert!(ok(&e.imu(false)), "the sketch samples only after an imu_begin, on the board and here");
        scan(&mut e, 130);
        let r = e.imu(false);
        assert_eq!(r.get("via").and_then(|x| x.as_str()), Some("poll_get"));
        match (r.get("accel_lsb"), r.get("gyro_lsb")) {
            (Some(J::Arr(a)), Some(J::Arr(g))) => {
                assert_eq!(a[2], J::n(8192.0));
                assert_eq!(g[0], J::n(-1310.0));
            }
            _ => panic!("no sample"),
        }
    }

    #[test]
    fn an_older_sketch_still_answers_through_the_rpc() {
        // the trailer is new. A board flashed yesterday sends a poll_get that simply ends, and /api/imu
        // must keep working rather than report a sensor that is sitting right there.
        let (mut e, reads) = with_imu_in_poll(20, false);
        scan(&mut e, 130);
        let r = e.imu(false);
        assert!(ok(&r), "{}", why(&r));
        assert_eq!(r.get("via").and_then(|x| x.as_str()), Some("imu_read"));
        assert!(rpcs(&reads) >= 1, "nothing was ever read");
    }

    #[test]
    fn a_stale_snapshot_is_not_served() {
        // the sketch says how old its snapshot is. 800 ms of it is the I2C sampler having stopped, and a
        // policy fed a frozen attitude is exactly the failure this whole file exists to refuse.
        let (mut e, reads) = with_imu_in_poll(800, true);
        assert!(ok(&e.imu(false)));
        let first = rpcs(&reads);
        scan(&mut e, 130);
        let r = e.imu(false);
        assert_eq!(r.get("via").and_then(|x| x.as_str()), Some("imu_read"), "a half-second-old sample was served as now");
        assert!(rpcs(&reads) > first);
    }

    #[test]
    fn a_cache_the_sweep_stopped_refilling_is_not_served() {
        // the other half: the snapshot was fresh when it arrived, but no sweep has come back since, which is
        // what a wedged bus looks like from here. Age on the MCU cannot catch that; age on OUR side can.
        let (mut e, reads) = with_imu_in_poll(20, true);
        assert!(ok(&e.imu(false)));
        scan(&mut e, 130);
        assert_eq!(e.imu(false).get("via").and_then(|x| x.as_str()), Some("poll_get"));
        let before = rpcs(&reads);
        let (b, age, _) = e.imu_cache.clone().expect("the sweep filled it");
        e.imu_cache = Some((b, age, Instant::now() - Duration::from_millis(600)));
        assert_eq!(e.imu(false).get("via").and_then(|x| x.as_str()), Some("imu_read"));
        assert!(rpcs(&reads) > before);
    }

    // ---- /api/recentre: the one EEPROM write the raw door cannot do -------------------------------
    // Written against the 2026-10-04 failure: official_zero.py sent the midpoint byte through the raw door to
    // 15 servos and not one took it, because reg 55 (the EEPROM lock) reads 1 and the door never touches it.

    fn ok(v: &J) -> bool {
        matches!(v.get("ok"), Some(J::Bool(true)))
    }

    fn why(v: &J) -> String {
        v.get("why").and_then(|x| x.as_str()).unwrap_or("").to_string()
    }

    fn pos_of(e: &mut Engine, id: u8) -> u16 {
        match setid::read(&mut *e.io, id, feetech::REG_POS, 2) {
            Ok(setid::Got::Data(d)) => d[0] as u16 | (d[1] as u16) << 8,
            _ => panic!("no position"),
        }
    }

    #[test]
    fn recentre_moves_a_wrapped_servo_onto_the_official_zero() {
        // right knee read 4523 on 2026-10-04: past the wrap, so no band could be saved and it could not be driven
        let mut e = engine(FakeBus::new(&[(13, 4523)]));
        let r = e.recentre(13, "test".into());
        assert!(ok(&r), "{}", why(&r));
        assert_eq!(pos_of(&mut e, 13), CENTRE, "the servo's own zero did not move");
    }

    #[test]
    fn recentre_leaves_the_eeprom_lock_back_on() {
        let mut e = engine(FakeBus::new(&[(13, 3000)]));
        assert!(ok(&e.recentre(13, "test".into())));
        match setid::read(&mut *e.io, 13, feetech::REG_LOCK, 1) {
            Ok(setid::Got::Data(d)) => assert_eq!(d[0], 1, "EEPROM left unlocked"),
            _ => panic!("lock did not read"),
        }
    }

    #[test]
    fn recentre_writes_the_position_correction() {
        let mut e = engine(FakeBus::new(&[(13, 3000)]));
        let r = e.recentre(13, "test".into());
        assert_eq!(r.get("ofs").and_then(|x| x.i64()), Some(3000 - 2048), "reg 31 was not written");
    }

    #[test]
    fn recentre_refuses_a_servo_with_torque_on() {
        let mut bus = FakeBus::new(&[(13, 3000)]);
        bus.on(13)[0].mem[40] = 1;
        let mut e = engine(bus);
        let r = e.recentre(13, "test".into());
        assert!(!ok(&r));
        assert!(why(&r).contains("torque on"), "{}", why(&r));
        assert_eq!(pos_of(&mut e, 13), 3000, "it wrote anyway");
    }

    #[test]
    fn recentre_refuses_a_servo_that_is_moving() {
        // two reads 150 ms apart must agree: a zero taken while a person is still adjusting the leg is not a zero
        let mut bus = FakeBus::new(&[(13, 3000)]);
        bus.drift = 80;
        let mut e = engine(bus);
        let r = e.recentre(13, "test".into());
        assert!(!ok(&r));
        assert!(why(&r).contains("still"), "{}", why(&r));
    }

    #[test]
    fn recentre_refuses_a_servo_that_is_not_there() {
        let mut e = engine(FakeBus::new(&[(13, 3000)]));
        let r = e.recentre(99, "test".into());
        assert!(!ok(&r), "a silent ID must not come back ok");
    }

    #[test]
    fn recentre_moves_a_saved_band_with_the_zero() {
        // a band measured in the old frame points somewhere else once the frame moves
        let mut e = engine(FakeBus::new(&[(21, 2500)]));
        e.sh.lock().unwrap().limits.set(21, Lim {
            min: 2400, max: 2600, rec_min: 2366, rec_max: 2634, margin: limits::MARGIN,
            when: "test".into(), who: "test".into(), source: "test".into(), eeprom: String::new(),
        });
        let r = e.recentre(21, "test".into());
        assert!(ok(&r), "{}", why(&r));
        assert_eq!(r.get("min").and_then(|x| x.i64()), Some(1948), "band did not move with the zero");
        assert_eq!(r.get("max").and_then(|x| x.i64()), Some(2148));
        let s = e.sh.lock().unwrap();
        assert_eq!((s.limits.map[&21].min, s.limits.map[&21].max), (1948, 2148), "the daemon kept the old band");
    }

    #[test]
    fn recentre_refuses_when_the_band_would_leave_the_turn() {
        // a stale band, recorded before the horn was moved: shifting it by 1648 counts puts it past 4095, and
        // a band that cannot follow the zero must stop the write rather than be quietly clipped
        let mut e = engine(FakeBus::new(&[(21, 400)]));
        e.sh.lock().unwrap().limits.set(21, Lim {
            min: 3000, max: 3900, rec_min: 2966, rec_max: 3934, margin: limits::MARGIN,
            when: "test".into(), who: "test".into(), source: "test".into(), eeprom: String::new(),
        });
        let r = e.recentre(21, "test".into());
        assert!(!ok(&r));
        assert!(why(&r).contains("outside one turn"), "{}", why(&r));
    }

    #[test]
    fn recentre_says_failed_when_the_servo_does_not_take_it() {
        // the 2026-10-04 shape exactly: the byte arrives, nothing happens, and the caller must be told so
        let mut bus = FakeBus::new(&[(13, 3000)]);
        bus.eeprom_stuck = true;
        let mut e = engine(bus);
        let r = e.recentre(13, "test".into());
        assert!(!ok(&r), "a write that did nothing was reported as success");
        assert!(why(&r).contains("did not take it"), "{}", why(&r));
    }

    fn engine(bus: FakeBus) -> Engine {
        let sh = Arc::new(Mutex::new(Shared::new(vec![], Registry::default())));
        let mut e = Engine::new(Box::new(bus), sh);
        e.begun = true;
        e
    }

    fn scan(e: &mut Engine, ticks: usize) {
        for _ in 0..ticks {
            e.roster_tick();
            e.tick_n += 1;
        }
    }

    #[test]
    fn fake_bus_with_two_servos_on_one_id_raises_the_banner() {
        let mut bus = FakeBus::new(&[(100, 2980), (13, 240)]);
        bus.servos.push(crate::io::FakeServo::new(1, 250));
        bus.servos.push(crate::io::FakeServo::new(1, 700));
        let mut e = engine(bus);
        scan(&mut e, 128 + 45); // one full scan, then enough reads to judge
        let s = e.sh.lock().unwrap();
        assert_eq!(s.roster.ids(), vec![1, 13, 100]);
        let j = s.bus_json();
        assert_eq!(j.get("dups"), Some(&J::Arr(vec![J::n(1.0)])), "{}", crate::json::dump(&j));
        assert!(s.roster.events.iter().any(|ev| ev.kind == "duplicate"));
    }

    #[test]
    fn negative_control_one_servo_per_id_no_banner() {
        let mut e = engine(FakeBus::new(&[(100, 2980), (13, 240), (14, 682)]));
        scan(&mut e, 128 + 45);
        let s = e.sh.lock().unwrap();
        assert_eq!(s.bus_json().get("dups"), Some(&J::Arr(vec![])));
        assert_eq!(s.roster.verdict(13), Verdict::Ok);
        // the EEPROM rotation reached every row
        assert!(s.roster.rows.values().all(|r| r.eeprom.is_some()));
    }

    #[test]
    fn an_unplugged_servo_leaves_after_a_scan() {
        let mut e = engine(FakeBus::new(&[(100, 2980), (13, 240)]));
        scan(&mut e, 130);
        assert!(e.sh.lock().unwrap().roster.rows.contains_key(&13));
        // unplug 13 (the fake bus is behind the Box: rebuild the engine state around a new bus)
        let sh = e.sh.clone();
        let mut e2 = Engine::new(Box::new(FakeBus::new(&[(100, 2980)])), sh);
        e2.begun = true;
        // pretend the last reply was 3 s ago
        e2.sh.lock().unwrap().roster.rows.get_mut(&13).unwrap().last_seen -= 3.0;
        e2.sh.lock().unwrap().roster.rows.get_mut(&13).unwrap().last_clean -= 3.0;
        scan(&mut e2, 130);
        let s = e2.sh.lock().unwrap();
        assert!(!s.roster.rows.contains_key(&13), "{:?}", s.roster.ids());
        assert!(s.roster.events.iter().any(|ev| ev.kind == "left" && ev.id == Some(13)));
    }

    #[test]
    fn a_stalled_bus_recovers_by_itself() {
        // the wedge is in the MCU: a reconnect cannot clear it, so the ladder climbs to an app restart
        let mut bus = FakeBus::new(&[(100, 2980)]);
        bus.stall_from = Some(5);
        let mut e = engine(bus);
        for _ in 0..12 {
            e.roster_tick();
            e.first_fail -= 4.0; // the streak has lasted longer than STALL_SECS
            e.last_recover -= 100.0; // and the gaps between rungs have passed
            e.maybe_recover();
        }
        let s = e.sh.lock().unwrap();
        assert_eq!(s.health.recovers, 1, "one MCU app restart was enough");
        assert_eq!(s.health.soft_recovers, 1, "and a reconnect was tried first");
        assert!(s.roster.events.iter().any(|ev| ev.kind == "stall"));
        assert!(s.roster.events.iter().any(|ev| ev.kind == "recover"));
        assert!(s.health.ok, "answering again after the restart");
    }

    #[test]
    fn a_rebooted_mcu_is_opened_again_instead_of_failing_for_ever() {
        // 2026-10-04 20:36:50 CST: someone restarted the MCU app beside the daemon. The sketch came back with
        // Serial1 closed, every bus_batch answered "0 frames for 19", and the daemon logged 2315 of those in a
        // row while its note said "restarted once... restart it by hand". One bus_begin was the whole cure.
        let mut bus = FakeBus::new(&[(100, 2980)]);
        bus.forgot = true;
        let mut e = engine(bus);
        e.roster_tick();
        assert!(!e.begun, "an empty bus_batch must make the daemon open the bus again");
        assert!(e.io.begin().is_ok());
        e.begun = true;
        e.roster_tick();
        assert!(e.begun, "and once it is open, it stays open");
        assert_eq!(e.sh.lock().unwrap().health.recovers, 0, "no MCU app restart was needed for this");
    }

    #[test]
    fn a_rebooted_mcu_in_poll_mode_is_caught_by_its_silent_sweep() {
        // the nastier half of the same fault: poll_get still ANSWERS, so nothing fails and nothing recovers.
        // Only the sweep counter gives it away.
        let mut bus = FakeBus::new(&[(100, 2980)]);
        bus.poll_capable = true;
        bus.forgot = true;
        let mut e = engine(bus);
        e.sh.lock().unwrap().roster.pinged(100, now_unix());
        e.roster_tick();
        assert!(e.begun, "two seconds have not passed yet");
        e.last_sweep -= NO_SWEEP_REOPEN + Duration::from_millis(100);
        e.roster_tick();
        assert!(!e.begun, "a poll_get that answers while nothing is swept is a closed bus");
        let s = e.sh.lock().unwrap();
        assert!(s.roster.events.iter().any(|ev| ev.text.contains("swept nothing")), "{:?}", s.roster.events.back());
    }

    #[test]
    fn a_socket_wedge_is_healed_by_a_reconnect_alone() {
        // the MCU is fine and only our own router connection went bad: no 20 s MCU app restart for that
        let mut bus = FakeBus::new(&[(100, 2980)]);
        bus.stall_from = Some(5);
        bus.stall_soft_heals = true;
        let mut e = engine(bus);
        for _ in 0..12 {
            e.roster_tick();
            e.first_fail -= 4.0;
            e.last_recover -= 100.0;
            e.maybe_recover();
        }
        let s = e.sh.lock().unwrap();
        assert_eq!(s.health.recovers, 0, "the MCU app was never restarted");
        assert_eq!(s.health.soft_recovers, 1);
        assert!(s.health.ok);
    }

    #[test]
    fn a_short_hiccup_never_restarts_anything() {
        // five failed calls inside STALL_SECS is a slow peer, not a dead bus: the old code restarted the
        // MCU app here (nine times on 2026-10-04 19:43-20:34, each costing ~20 s of bus)
        let mut bus = FakeBus::new(&[(100, 2980)]);
        bus.stall_from = Some(3);
        let mut e = engine(bus);
        for _ in 0..10 {
            e.roster_tick();
            e.maybe_recover();
        }
        let s = e.sh.lock().unwrap();
        assert!(s.health.fail_streak >= STALL_STREAK, "the calls did fail");
        assert_eq!(s.health.recovers, 0, "no MCU app restart inside {STALL_SECS} s");
        assert_eq!(s.health.soft_recovers, 0, "and no reconnect either");
        assert_eq!(s.health.stalled_since, None, "not even called a stall yet");
    }

    #[test]
    fn a_bus_that_stays_dead_keeps_trying_for_ever() {
        // the old branch stopped after one restart and asked for a human; twice on 2026-10-04 that left the
        // bus dead until the daemon was restarted by hand
        let mut bus = FakeBus::new(&[(100, 2980)]);
        bus.stall_from = Some(5);
        bus.stall_sticky = true;
        let mut e = engine(bus);
        for _ in 0..60 {
            e.roster_tick();
            e.first_fail -= 4.0;
            e.last_recover -= 100.0;
            e.maybe_recover();
        }
        let s = e.sh.lock().unwrap();
        assert_eq!(s.health.recovers, MAX_HARD_PER_OUTAGE as u64, "the app restarts are capped");
        assert!(s.health.soft_recovers > MAX_HARD_PER_OUTAGE as u64, "and it goes on reconnecting: {}", s.health.soft_recovers);
        assert!(!s.health.note.starts_with("waiting"), "it never parks waiting for a human: {}", s.health.note);
        assert!(!s.health.ok);
    }

    #[test]
    fn the_mcu_counters_survive_the_restart_that_cures_the_fault() {
        // Measured 2026-10-04 21:51:16 CST: the MCU went dark, the ladder recovered it with two app
        // restarts, and afterwards tx_stuck and lock_lost both read 0. They read 0 because the sketch
        // they live in had been restarted - the cure wipes the only witness, and a wiped witness looks
        // exactly like "the TX path was fine". The pre-outage snapshot and the high-water mark survive.
        let mut e = engine(FakeBus::new(&[(100, 2980)]));
        {
            let mut s = e.sh.lock().unwrap();
            s.health.tx_stuck = 4;
            s.health.lock_lost = 2;
            s.health.tx_stuck_max = 4;
            s.health.lock_lost_max = 2;
        }
        e.note_fail("bus_begin: mcu error: method bus_begin not available".into());
        {
            let s = e.sh.lock().unwrap();
            assert_eq!((s.health.tx_stuck_pre, s.health.lock_lost_pre), (4, 2),
                       "the counters as the outage began must be kept");
        }
        // the MCU comes back fresh, as it does after an app restart
        {
            let mut s = e.sh.lock().unwrap();
            s.health.tx_stuck = 0;
            s.health.lock_lost = 0;
        }
        e.account::<()>(Ok(()), Instant::now());
        let s = e.sh.lock().unwrap();
        assert_eq!(s.health.tx_stuck, 0, "the live counter follows the new sketch run");
        assert_eq!((s.health.tx_stuck_pre, s.health.lock_lost_pre), (4, 2),
                   "but what the outage started with is still readable");
        assert_eq!((s.health.tx_stuck_max, s.health.lock_lost_max), (4, 2),
                   "and so is the highest either reached");
    }

    #[test]
    fn a_bus_that_will_not_open_is_counted_as_a_wedge_like_any_other() {
        // Measured 2026-10-04 21:12:19-21:18:54 CST: the MCU app went away, the ladder ran its full
        // course (reconnect, app restart, reconnect, app restart, 6.5 minutes with no bus), and
        // `health.stalls` read 0 the whole way, because the begin path counted failures by hand and
        // its hand-rolled copy never touched `stalls` or `stalled_since`. `stalls` is the number the
        // soak calls a wedge, so the one outage worth counting was the one that did not count.
        let mut bus = FakeBus::new(&[(100, 2980)]);
        bus.begin_fails = true;
        let mut e = engine(bus);
        e.begun = false;
        // the bus stays shut until the ladder's MCU app restart clears it, as it did on the board
        let mut opened_after = None;
        for i in 0..STALL_STREAK + 8 {
            if e.begin_tick() {
                opened_after = Some(i);
                break;
            }
            if i + 1 == STALL_STREAK {
                let s = e.sh.lock().unwrap();
                assert_eq!(s.health.stalls, 1, "a bus that will not open is one wedge, not zero");
                assert!(s.health.stalled_since.is_some(), "and the wedge has a start time");
                assert!(!s.health.ok);
                assert!(s.health.last_error.starts_with("bus_begin:"), "{}", s.health.last_error);
            }
            e.first_fail -= 4.0;
            e.last_recover -= 100.0;
        }
        let opened_after = opened_after.expect("the ladder must get bus_begin answering again");
        assert!(opened_after >= STALL_STREAK, "it opened before it was ever counted: {opened_after}");
        {
            let s = e.sh.lock().unwrap();
            assert_eq!(s.health.recovers, 1, "one MCU app restart was what opened it");
        }
        // and the wedge closes the same way any other one does
        e.account::<()>(Ok(()), Instant::now());
        let s = e.sh.lock().unwrap();
        assert_eq!(s.health.stalls, 1, "it is still one wedge, not a new one");
        assert!(s.health.stalled_since.is_none(), "but it is over");
        assert!(s.health.ok);
    }

    #[test]
    fn hold_caps_at_300_without_a_jump_and_goals_only_move_held_servos() {
        let mut e = engine(FakeBus::new(&[(13, 240), (14, 682)]));
        scan(&mut e, 130);
        e.goals(&[(13, 2000)]);
        assert!(!e.sh.lock().unwrap().held.contains(&13));
        e.hold(13, true);
        let s = e.sh.lock().unwrap();
        assert!(s.held.contains(&13), "{:?}", s.roster.events.back());
        drop(s);
        e.goals(&[(13, 1800), (14, 3000)]);
        e.hold(13, false);
        assert!(!e.sh.lock().unwrap().held.contains(&13));
    }

    /// The jaw (34) rests at 288. . With no limits a slider goal is refused
    /// and the servo stays put; Save writes EEPROM 9/11 = recorded ends minus 3 deg and reads them back; afterwards a
    /// goal past the end stops AT the limit, one inside goes through, move/wave use the servo's own band.
    #[test]
    fn jaw_limits_refuse_past_limit_goals_and_the_servo_does_not_move() {
        let mut e = engine(FakeBus::new(&[(34, 288)]));
        scan(&mut e, 130);
        e.hold(34, true);
        assert!(e.sh.lock().unwrap().held.contains(&34));
        e.goals(&[(34, 2048)]);
        assert_eq!(e.read_pos(34), Some(288), "no limits saved: the goal must be refused");
        assert!(e.sh.lock().unwrap().roster.events.iter().any(|v| v.kind == "refused" && v.text.contains("no angle limits")));
        // saving while held is refused, nothing written
        let r = e.save_limits(34, Some((45, 390)), "test".into(), "test".into());
        assert_eq!(r.get("ok"), Some(&J::Bool(false)));
        e.hold(34, false);
        let r = e.save_limits(34, Some((45, 390)), "test".into(), "test".into());
        assert_eq!(r.get("ok"), Some(&J::Bool(true)), "{}", crate::json::dump(&r));
        match setid::read(&mut *e.io, 34, 9, 4).unwrap() {
            setid::Got::Data(d) => assert_eq!((d[0] as u16 | (d[1] as u16) << 8, d[2] as u16 | (d[3] as u16) << 8), (79, 356)),
            g => panic!("{g:?}"),
        }
        assert_eq!(e.sh.lock().unwrap().limits.band(34, None), Some((79, 356)));
        e.hold(34, true);
        e.goals(&[(34, 2048)]);
        assert_eq!(e.read_pos(34), Some(356), "past the end: stops at the limit, never past it");
        e.goals(&[(34, 200)]);
        assert_eq!(e.read_pos(34), Some(200), "inside the limits the goal goes through");
        e.goals(&[(34, 10)]);
        assert_eq!(e.read_pos(34), Some(79));
        assert!(e.sh.lock().unwrap().roster.events.iter().any(|v| v.kind == "limit"));
        // move and wave use this band, not the global 1024..3072
        assert_eq!(band_counts((79, 356), 0.0), 79);
        assert_eq!(band_counts((79, 356), 90.0), 356);
        assert_eq!(wave_counts((79, 356), 45.0, true), 356);
        assert_eq!(wave_counts((79, 356), 45.0, false), 79);
        assert!(deg_to_counts(45.0) > 356, "the old mapping would have sent the jaw to 2048");
    }

    /// Negative control: the same goal sent around the clamp (raw sync-write) to a servo with factory EEPROM limits
    /// does move it, so the test above would see a past-limit move if the clamp were gone.
    #[test]
    fn negative_control_a_goal_around_the_clamp_moves_the_jaw() {
        let mut e = engine(FakeBus::new(&[(34, 288)]));
        scan(&mut e, 130);
        e.torque(&[34], true);
        e.sync_move(&[(34, 2048)]);
        assert_eq!(e.read_pos(34), Some(2048));
    }

    /// Torque byte (reg 40) of `id`, read through the raw door like the policy reads it.
    fn torque_reg(e: &mut Engine, id: u8) -> u8 {
        let r = e.raw(&[crate::feetech::read(id, 40, 1)], &[7], 3000, 50, "test");
        let h = match r.get("replies") { Some(J::Arr(a)) => a[0].as_str().unwrap().to_string(), _ => panic!("{r:?}") };
        u8::from_str_radix(&h[10..12], 16).unwrap()
    }

    #[test]
    fn a_policy_that_goes_quiet_with_torque_on_is_turned_off_by_the_daemon() {
        let mut e = engine(FakeBus::new(&[(22, 1200)]));
        scan(&mut e, 130);
        e.sh.lock().unwrap().limits.set(22, crate::limits::Lim { min: 781, max: 1737, rec_min: 747, rec_max: 1771, margin: 34, when: "t".into(), who: "test".into(), source: "test".into(), eeprom: String::new() });
        let r = e.raw(&[crate::feetech::write(22, 40, &[1])], &[6], 3000, 50, "policy test");
        assert_eq!(r.get("ok"), Some(&J::Bool(true)));
        assert_eq!(torque_reg(&mut e, 22), 1);
        e.raw_watchdog();
        assert_eq!(torque_reg(&mut e, 22), 1, "a caller still talking keeps its torque");
        e.raw_last = Some(Instant::now() - RAW_QUIET - Duration::from_millis(10));
        e.raw_watchdog();
        assert_eq!(torque_reg(&mut e, 22), 0, "silent for over a second: torque off");
        assert!(e.raw_on.is_empty());
        // negative control: a caller that turned its own torque off leaves nothing for the watchdog to do
        e.raw(&[crate::feetech::write(22, 40, &[1])], &[6], 3000, 50, "policy test");
        e.raw(&[crate::feetech::write(22, 40, &[0])], &[6], 3000, 50, "policy test");
        assert!(e.raw_on.is_empty());
    }

    #[test]
    fn a_short_recording_is_not_saved() {
        let mut e = engine(FakeBus::new(&[(34, 288)]));
        scan(&mut e, 130);
        let r = e.save_limits(34, None, "test".into(), "test".into());
        assert_eq!(r.get("ok"), Some(&J::Bool(false)), "the fake jaw never moved: a 0 deg range must be refused");
        assert!(e.sh.lock().unwrap().limits.map.is_empty());
    }

    /// Modeled ms per tick (one bus_batch); the loop floor is 20 ms, so Hz = 1000 / max(ms, 20) for a roster of `n` servos (plus the adapter) under a link model.
    fn modeled_hz(n: usize, m: crate::io::LinkModel) -> f64 {
        modeled(n, m, false)
    }

    fn modeled(n: usize, m: crate::io::LinkModel, poll: bool) -> f64 {
        let mut servos: Vec<(u8, u16)> = (0..n).map(|k| (10 + k as u8, 2048)).collect();
        servos.push((100, 3000));
        let mut bus = FakeBus::new(&servos);
        bus.link = Some(m);
        bus.poll_capable = poll;
        let mut e = engine(bus);
        scan(&mut e, 130); // everyone joined
        assert_eq!(e.sh.lock().unwrap().roster.ids().len(), n + 1);
        let before = e.io.modeled_ms();
        for _ in 0..200 {
            e.roster_tick();
        }
        (e.io.modeled_ms() - before) / 200.0
    }

    #[test]
    fn link_model_matches_the_board_and_says_what_50_hz_with_16_servos_needs() {
        use crate::io::LinkModel;
        let now = LinkModel { fixed_ms: 5.5, baud: 115_200.0, return_delay_us: 506.0 };
        let one = modeled_hz(0, now);
        assert!((one - 17.4).abs() < 3.0, "1 servo: modeled {one:.1} ms per tick, measured 17.4");
        let rows = [
            ("115200 baud, 506 us", now),
            ("1 Mbaud, 506 us", LinkModel { baud: 1_000_000.0, ..now }),
            ("115200 baud, 20 us", LinkModel { return_delay_us: 20.0, ..now }),
            ("1 Mbaud, 20 us", LinkModel { baud: 1_000_000.0, return_delay_us: 20.0, ..now }),
        ];
        for (name, m) in rows {
            let (a, b) = (modeled_hz(0, m), modeled_hz(15, m));
            eprintln!("MODEL {name}: 1 servo {a:.1} ms ({:.0} Hz), 16 servos {b:.1} ms ({:.0} Hz)", 1000.0 / a.max(20.0), 1000.0 / b.max(20.0));
        }
        let p16 = modeled(15, now, true);
        eprintln!("MODEL MCU poll loop, 115200 baud, 506 us: 16 servos {p16:.1} ms ({:.0} Hz)", 1000.0 / p16.max(20.0));
        assert!(p16 < 20.0, "poll loop must fit 16 servos in a 20 ms tick: {p16:.1} ms");
    }

    #[test]
    fn poll_loop_roster_banner_and_leave() {
        let mut bus = FakeBus::new(&[(1, 240), (1, 900), (13, 500), (100, 2980)]);
        bus.poll_capable = true;
        let mut e = engine(bus);
        scan(&mut e, 130);
        assert_eq!(e.poll_mode, Some(true));
        let s = e.sh.lock().unwrap();
        assert_eq!(s.roster.ids(), vec![1, 13, 100]);
        assert!(matches!(s.roster.verdict(1), crate::roster::Verdict::Duplicate(_)), "{:?}", s.roster.verdict(1));
        assert_eq!(s.roster.verdict(13), crate::roster::Verdict::Ok);
        assert!(s.roster.rows[&13].eeprom.is_some(), "EEPROM block arrives through the snapshot");
        assert!(s.roster.rows[&13].tele.as_ref().unwrap().volt > 7.0, "full block arrives through the snapshot");
    }

    #[test]
    fn hold_is_refused_on_a_duplicate_id() {
        let mut e = engine(FakeBus::new(&[(1, 240), (1, 900), (13, 500)]));
        scan(&mut e, 130);
        e.hold(1, true);
        assert!(!e.sh.lock().unwrap().held.contains(&1));
        e.hold(13, true);
        assert!(e.sh.lock().unwrap().held.contains(&13));
    }

    #[test]
    fn setid_through_the_engine_refuses_a_registry_taken_id() {
        let mut e = engine(FakeBus::new(&[(1, 1485), (100, 2980)]));
        e.sh.lock().unwrap().registry.assigned.insert(
            12,
            crate::registry::Entry { joint: "Right hip pitch".into(), since: "2026-10-04 13:10".into(), source: "test".into() },
        );
        scan(&mut e, 130);
        let r = e.setid(1, 12, String::new(), "test".into());
        assert_eq!(r.get("written"), Some(&J::Bool(false)));
        let r = e.setid(1, 13, "Right knee".into(), "test".into());
        assert_eq!(r.get("ok"), Some(&J::Bool(true)), "{}", crate::json::dump(&r));
        assert_eq!(e.sh.lock().unwrap().registry.assigned[&13].joint, "Right knee");
    }
}
