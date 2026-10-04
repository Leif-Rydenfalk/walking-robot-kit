//! Bus I/O: one `batch` shape for the real bus (RouterBridge -> MCU `bus_batch`) and for the fake bus the
//! tests and `--fake` runs use. Frames are whole packets; every frame gets its own [n, bytes..] entry back,
//! no echo: the URT-2 on this board does not hear its own TX (measured 2026-10-04), so an expected byte count is the
//! reply alone; counting an echo made the MCU wait out wait_us on every frame.
//!
//! Recovery, in two steps. On 2026-10-04 the Python recorder hit `bus_batch timed out` and never recovered
//! until the MCU app was restarted (from 13:20:00, just after a set-ID ran beside it, and from 14:32:01), so
//! the daemon learned to do that itself. Restarting the MCU app costs about 20 s of dead bus, and on
//! 2026-10-04 19:43-20:34 it fired nine times in 50 minutes for stalls that lasted 1.5 s. `recover(hard)` is
//! now a ladder: `false` only reconnects the router socket and calls `bus_begin` again (under a second), and
//! `true` stops and starts the MCU app as before. bus.rs climbs it, soft first.
//!
//! Twice that evening (19:50:51, 20:09:04) the app restart left the router answering
//! `method bus_begin not available` while `mx_shape` still worked on the same MCU: the bus methods, which the
//! sketch registers FIRST, had not come back. Giving up there left the daemon dead until a human restarted it.
//! The hard path now waits 30 s for the methods to reappear, reconnecting the socket as it goes, and says which
//! of the two shapes it saw.

use crate::bridge::Bridge;
use crate::feetech;
use crate::msgpack::Value;
use std::process::Command;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

pub trait Io: Send {
    fn batch(&mut self, frames: &[Vec<u8>], expects: &[usize], wait_us: i64, timeout: Duration) -> Result<Vec<Vec<u8>>, String>;
    fn begin(&mut self) -> Result<(), String>;
    /// `hard = false`: reconnect the router socket and open the bus again (under a second).
    /// `hard = true`: stop and start the MCU app, then wait for its methods to come back.
    fn recover(&mut self, hard: bool) -> Result<String, String>;
    fn connected(&self) -> bool;
    fn reconnects(&self) -> u64;
    /// (calls that ran out of time, replies that arrived after their caller gave up, the latest such lateness in ms)
    fn lateness(&self) -> (u64, u64, f64) {
        (0, 0, 0.0)
    }
    /// The sketch's own counters: (frames refused because the UART TX ring would not drain, calls that gave up
    /// waiting for the bus mutex). Err("not available") on a sketch from before 2026-10-04 21:0x.
    fn stuck(&mut self) -> Result<(u32, u32), String> {
        Err("method bus_stuck not available".into())
    }
    /// MCU poll loop (sketch 3d4c20d+): which IDs the MCU sweeps. Err with "not available" = an older sketch.
    fn poll_set(&mut self, _ids: &[u8]) -> Result<(), String> {
        Err("method poll_set not available".into())
    }
    /// The newest sweep, compact (see the sketch's poll_get comment for the layout).
    fn poll_get(&mut self) -> Result<Vec<u8>, String> {
        Err("method poll_get not available".into())
    }
    /// The LED matrix frame (104 pixels 0..7) and/or LED3 colour, sent by the bus owner so the MCU has ONE router
    /// client (2026-10-04 18:06: a second client calling mx_draw at 4/s stalled every MCU method within 14 s).
    fn matrix(&mut self, _frame: Option<&[u8]>, _rgb: Option<(u16, u16, u16)>) -> Result<(), String> {
        Ok(())
    }
    /// Every (bus, 7-bit address) pair that ACKed, across every I2C bus the sketch has. The wiring
    /// measured, not assumed: we had no Qwiic cable on 2026-10-04 and wired the sensor to the header
    /// instead, which is a different bus (Wire = i2c2 = D20/D21; Wire1 = i2c4 = the Qwiic socket).
    fn imu_scan(&mut self) -> Result<Vec<(u8, u8)>, String> {
        Err("method imu_scan not available".into())
    }
    /// Wake and configure the MPU-6050 on whichever bus carries it.
    /// -> [ok, bus, who_am_i, pwr, cfg, gyro_cfg, accel_cfg, smplrt], every value READ BACK from the chip,
    /// so a write that did not land cannot look like a configured sensor.
    fn imu_begin(&mut self, _addr: u8) -> Result<Vec<i64>, String> {
        Err("method imu_begin not available".into())
    }
    /// The 14-byte burst from 0x3B: ax ay az temp gx gy gz, big-endian i16 each. Empty = the read failed.
    fn imu_read(&mut self) -> Result<Vec<u8>, String> {
        Err("method imu_read not available".into())
    }
    /// Fake bus only: the modeled board time spent so far (LinkModel).
    fn modeled_ms(&self) -> f64 {
        0.0
    }
}

/// bus_batch reply layout: [n1, b.., n2, b..] per frame, a zero-length entry kept (poll pairs by index).
pub fn split_frames(raw: &[u8]) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    let mut k = 0usize;
    while k < raw.len() {
        let n = raw[k] as usize;
        k += 1;
        let end = (k + n).min(raw.len());
        out.push(raw[k..end].to_vec());
        k = end;
    }
    out
}

/// How long one MCU RPC may take before the daemon gives up on it. The MCU serves RPCs one at a time
/// on one thread, so a poll_get queues behind whatever bus_batch is running; measured round trips are
/// 17 ms typical and 430 ms at the worst seen (2026-10-04 20:30, with the raw door and the matrix sender
/// both in the queue). The old 300 ms gave up inside the normal spread, and five of those in 1.5 s
/// triggered an MCU-app restart. These are deadlines for a SLOW peer, not a liveness test: the stall
/// decision is time-based and lives in bus.rs.
pub const POLL_TIMEOUT: Duration = Duration::from_millis(900);
pub const MX_TIMEOUT: Duration = Duration::from_millis(900);
pub const BEGIN_TIMEOUT: Duration = Duration::from_millis(2500);

pub struct RouterIo {
    pub bridge: Bridge,
    pub app: String,
}

fn run_for(cmd: &[&str], cap: Duration) -> Result<String, String> {
    let mut child = Command::new(cmd[0])
        .args(&cmd[1..])
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .map_err(|e| format!("{}: {e}", cmd.join(" ")))?;
    let t0 = Instant::now();
    loop {
        match child.try_wait() {
            Ok(Some(st)) => {
                let out = child.wait_with_output().map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string()).unwrap_or_default();
                return if st.success() { Ok(out) } else { Err(format!("{} exited {st}: {out}", cmd.join(" "))) };
            }
            Ok(None) if t0.elapsed() > cap => {
                child.kill().ok();
                return Err(format!("{} still running after {:?}, killed", cmd.join(" "), cap));
            }
            Ok(None) => std::thread::sleep(Duration::from_millis(100)),
            Err(e) => return Err(e.to_string()),
        }
    }
}

impl Io for RouterIo {
    fn batch(&mut self, frames: &[Vec<u8>], expects: &[usize], wait_us: i64, timeout: Duration) -> Result<Vec<Vec<u8>>, String> {
        // msgpack bin, the shape the Python RouterTransport sends and the sketch's std::vector<uint8_t> takes
        let mut flat: Vec<u8> = Vec::with_capacity(256);
        for (f, e) in frames.iter().zip(expects) {
            flat.push((*e).min(250) as u8);
            flat.extend_from_slice(f);
        }
        let r = self.bridge.call("bus_batch", vec![Value::Bin(flat), Value::Int(wait_us)], timeout)?;
        let raw = r.as_bytes().ok_or_else(|| format!("bus_batch answered {r:?}"))?;
        let out = split_frames(&raw);
        if out.len() != frames.len() {
            return Err(format!("bus_batch answered {} frames for {}", out.len(), frames.len()));
        }
        Ok(out)
    }

    fn begin(&mut self) -> Result<(), String> {
        // the sketch answers 1 (opened with the bus mutex held) or 2 (opened while another call still held it);
        // anything else means the bus is not open, and an unchecked reply once hid exactly that
        match self.bridge.call("bus_begin", vec![Value::Int(1_000_000)], BEGIN_TIMEOUT)? {
            Value::Int(1) => Ok(()),
            Value::Int(2) => Ok(()),
            v => Err(format!("bus_begin answered {v:?}, not 1 or 2: the bus is not open")),
        }
    }

    fn recover(&mut self, hard: bool) -> Result<String, String> {
        // evidence first: which methods does the router still route? (2026-10-04 15:18: after a wedge the router
        // answered "method bus_batch not available" while the LED methods on the same MCU still worked)
        self.bridge.drop_socket();
        let mut seen = Vec::new();
        for m in ["bus_baud", "mx_shape"] {
            let r = self.bridge.call(m, vec![], Duration::from_millis(800));
            seen.push(format!("{m}: {}", match r { Ok(v) => format!("{v:?}"), Err(e) => e }));
        }
        let before = seen.join("; ");
        if !hard {
            // step 1: a fresh socket and a fresh bus_begin. No MCU app restart, so no 20 s hole.
            self.bridge.drop_socket();
            let mut last = String::new();
            for k in 0..6 {
                match self.begin() {
                    Ok(()) => return Ok(format!("router socket reconnected, bus opened again after {} tr{} (router said: {before})", k + 1, if k == 0 { "y" } else { "ies" })),
                    Err(e) => last = e,
                }
                std::thread::sleep(Duration::from_millis(400));
                self.bridge.drop_socket();
            }
            return Err(format!("reconnect did not open the bus: {last} (router said: {before})"));
        }
        eprintln!("recover: before restart {before}");
        self.bridge.drop_socket();
        let stop = run_for(&["arduino-app-cli", "app", "stop", &self.app], Duration::from_secs(60));
        let start = run_for(&["arduino-app-cli", "app", "start", &self.app], Duration::from_secs(90))?;
        std::thread::sleep(Duration::from_secs(2));
        let mut last = String::new();
        // 30 tries, 1 s apart, dropping the socket every fifth: after a restart the router has shown up to
        // ten seconds of "method bus_begin not available" while the sketch re-registers (2026-10-04 19:50, 20:08)
        for k in 0..30 {
            match self.begin() {
                Ok(()) => {
                    let stop = stop.err().map(|e| format!(" (stop said: {e})")).unwrap_or_default();
                    let _ = start;
                    return Ok(format!("MCU app {} restarted{stop}, bus opened after {:.0} s (router before: {before})", self.app, k as f64 + 2.0));
                }
                Err(e) => last = e,
            }
            std::thread::sleep(Duration::from_secs(1));
            if k % 5 == 4 {
                self.bridge.drop_socket();
            }
        }
        Err(format!("MCU app restarted but bus_begin still fails after 30 s: {last}"))
    }

    fn lateness(&self) -> (u64, u64, f64) {
        (self.bridge.timeouts, self.bridge.late, self.bridge.late_ms_max)
    }

    fn stuck(&mut self) -> Result<(u32, u32), String> {
        let v = self.bridge.call("bus_stuck", vec![], Duration::from_millis(800))?;
        let n = v.as_i64().ok_or_else(|| format!("bus_stuck answered {v:?}"))? as u32;
        Ok((n >> 16, n & 0xFFFF))
    }

    fn connected(&self) -> bool {
        self.bridge.connected()
    }

    fn poll_set(&mut self, ids: &[u8]) -> Result<(), String> {
        self.bridge.call("poll_set", vec![Value::Bin(ids.to_vec())], POLL_TIMEOUT).map(|_| ())
    }

    fn poll_get(&mut self) -> Result<Vec<u8>, String> {
        let r = self.bridge.call("poll_get", vec![], POLL_TIMEOUT)?;
        r.as_bytes().ok_or_else(|| format!("poll_get answered {r:?}"))
    }

    fn imu_scan(&mut self) -> Result<Vec<(u8, u8)>, String> {
        let r = self.bridge.call("imu_scan", vec![], Duration::from_secs(10))?;
        let b = r.as_bytes().ok_or_else(|| format!("imu_scan answered {r:?}"))?;
        if b.len() % 2 != 0 {
            return Err(format!("imu_scan answered {} bytes: it reports (bus, address) pairs", b.len()));
        }
        Ok(b.chunks(2).map(|c| (c[0], c[1])).collect())
    }

    fn imu_begin(&mut self, addr: u8) -> Result<Vec<i64>, String> {
        // dlpf 3 (42 Hz gyro, 4.9 ms group delay) and div 4 (200 Hz) suit a 50 Hz control loop
        let r = self.bridge.call("imu_begin", vec![Value::Int(addr as i64), Value::Int(3), Value::Int(4)], MX_TIMEOUT)?;
        match r {
            Value::Arr(a) => Ok(a.iter().map(|x| x.as_i64().unwrap_or(-1)).collect()),
            other => Err(format!("imu_begin answered {other:?}")),
        }
    }

    fn imu_read(&mut self) -> Result<Vec<u8>, String> {
        let r = self.bridge.call("imu_read", vec![], POLL_TIMEOUT)?;
        let b = r.as_bytes().ok_or_else(|| format!("imu_read answered {r:?}"))?;
        // [0] = the sketch tried and failed; [14, ..] = a real sample. A short frame is never silently padded.
        match b.first() {
            Some(&14) if b.len() == 15 => Ok(b[1..].to_vec()),
            Some(&0) => Ok(vec![]),
            _ => Err(format!("imu_read answered {} bytes, expected 15", b.len())),
        }
    }

    fn reconnects(&self) -> u64 {
        self.bridge.reconnects
    }

    fn matrix(&mut self, frame: Option<&[u8]>, rgb: Option<(u16, u16, u16)>) -> Result<(), String> {
        if let Some(f) = frame {
            self.bridge.call("mx_draw", vec![Value::Bin(f.to_vec())], MX_TIMEOUT)?;
        }
        if let Some((r, g, b)) = rgb {
            self.bridge.call("set_led3_color", vec![Value::Int(r as i64), Value::Int(g as i64), Value::Int(b as i64)], MX_TIMEOUT)?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------------------------------
// The fake bus: servos with a register file, duplicates that garble each other, and a stall switch.
// ---------------------------------------------------------------------------------------------------

#[derive(Clone)]
pub struct FakeServo {
    pub mem: [u8; 71],
}

impl FakeServo {
    pub fn new(id: u8, pos: u16) -> Self {
        let mut mem = [0u8; 71];
        mem[5] = id;
        mem[7] = 253; // return delay 506 us, as measured on every HD-1910 so far
        mem[11] = 0xff;
        mem[12] = 0x0f; // max angle 4095
        mem[16] = 0xe8;
        mem[17] = 0x03; // max torque 1000 (factory)
        mem[48] = 0xe8;
        mem[49] = 0x03;
        mem[55] = 1; // EEPROM locked
        mem[56] = pos as u8;
        mem[57] = (pos >> 8) as u8;
        mem[62] = 74; // 7.4 V
        mem[63] = 31; // 31 C
        FakeServo { mem }
    }

    pub fn id(&self) -> u8 {
        self.mem[5]
    }

    fn status(&self, data: &[u8]) -> Vec<u8> {
        let mut body = vec![self.id(), data.len() as u8 + 2, 0];
        body.extend_from_slice(data);
        let sum: u32 = body.iter().map(|&b| b as u32).sum();
        let mut p = vec![0xff, 0xff];
        p.extend_from_slice(&body);
        p.push(!(sum as u8));
        p
    }

    fn write(&mut self, addr: usize, data: &[u8]) {
        for (k, b) in data.iter().enumerate() {
            let a = addr + k;
            if a >= 71 {
                break;
            }
            // EEPROM (below 40) takes a write only while unlocked; the lock itself is always writable
            if a < 40 && self.mem[55] != 0 {
                continue;
            }
            self.mem[a] = *b;
        }
        // Writing 128 to reg 40 is Feetech's midpoint command: the servo stores a position correction in EEPROM
        // reg 31 so its present pose reads 2048. It is an EEPROM write, so the lock (55) blocks it, which is
        // exactly what happened on all 15 the robot servos on 2026-10-04 and why /api/recentre exists.
        if addr == 40 && data.first() == Some(&128) {
            self.mem[40] = 0;
            if self.mem[55] == 0 {
                let pos = self.mem[56] as i32 | (self.mem[57] as i32) << 8;
                let ofs = pos - 2048;
                self.mem[31] = ofs as u8;
                self.mem[32] = (ofs >> 8) as u8;
                self.mem[56] = 2048u16 as u8;
                self.mem[57] = (2048u16 >> 8) as u8;
            }
            return;
        }
        // a torqued servo reaches its goal (42..43) at once on the fake bus, stopping at its EEPROM angle limits
        // (regs 9/11) the way the real one does
        if self.mem[40] == 1 {
            let le = |a: usize| self.mem[a] as u16 | (self.mem[a + 1] as u16) << 8;
            let (lo, hi) = (le(9), le(11));
            let mut g = le(42);
            if lo < hi {
                g = g.clamp(lo, hi);
            }
            self.mem[56] = g as u8;
            self.mem[57] = (g >> 8) as u8;
        }
    }
}

#[derive(Default)]
pub struct FakeBus {
    pub servos: Vec<FakeServo>,
    pub echo: bool,
    pub rpcs: u64,
    pub stall_from: Option<u64>, // RPC count from which every call times out until recover()
    pub recovers: u64,
    pub soft_recovers: u64,
    /// the wedge is in the socket, so reconnecting alone clears it (no MCU app restart needed)
    pub stall_soft_heals: bool,
    pub stall_sticky: bool, // a restart does not help (a bad lead holding the bus)
    /// the MCU rebooted and lost Serial1 (`g_baud == 0`): bus_batch answers nothing and the sweep stops,
    /// and only bus_begin cures it. Measured on the board 2026-10-04 20:36:50 CST.
    pub forgot: bool,
    /// bus_begin itself fails: the MCU app is not there to answer it. This is the shape of a board
    /// reboot (measured 21:12:19-21:18:54 CST 2026-10-04, "method bus_begin not available"), and it is
    /// the one outage the daemon never counted, because the begin path kept its own bookkeeping.
    pub begin_fails: bool,
    /// Time model of one bus_batch on the real board, to compare designs without the hardware.
    pub link: Option<LinkModel>,
    pub modeled_ms: f64,
    /// Emulates the sketch's poll loop when Some; None = an old sketch without poll_set/poll_get.
    pub poll: Option<Vec<u8>>,
    pub poll_capable: bool,
    poll_scan: u8,
    poll_turn: u32,
    garble: u64,
    /// counts added to every position read, so a test can be a joint a person is still moving
    pub drift: u16,
    /// A fake MPU-6050: Some((addr, [ax, ay, az, temp, gx, gy, gz])) in raw LSB, None = nothing on the I2C bus.
    pub imu: Option<(u8, [i16; 7])>,
    /// which bus it is wired to: 0 = Wire (header D20/D21), 1 = Wire1 (Qwiic), 2 = Wire2
    pub imu_on_bus: u8,
    pub imu_began: bool,
    /// the sensor answers the scan but every burst read fails: an unplugged SDA after a good begin
    pub imu_read_fails: bool,
    /// the sketch that carries the IMU snapshot in the poll_get reply (17 trailing bytes:
    /// have, 14 raw, age lo, age hi). false = an older sketch, so the daemon must fall back to
    /// the imu_read RPC. This is the whole point of the piggyback: that RPC costs ~34 ms of
    /// round trip and dropped the measured sweep from 49.7 Hz to 32.4 Hz at 10 Hz polling.
    pub imu_in_poll: bool,
    /// how old the snapshot in that trailer says it is
    pub imu_snap_age_ms: u16,
    /// every imu_read RPC the daemon actually made. Shared, because by the time a test wants the
    /// number the fake is inside a `Box<dyn Io>` and cannot be looked at any other way.
    pub imu_read_rpcs: Arc<AtomicU32>,
    /// the EEPROM ignores writes even while unlocked: the shape of a servo that silently refuses the
    /// midpoint command, which is how it failed on all 15 the robot servos on 2026-10-04
    pub eeprom_stuck: bool,
}

/// One bus_batch's time on the board (2026-10-04 15:58, measured): a fixed RPC cost (router + MCU dispatch;
/// bus_baud min 5.5 ms), every msgpack byte on the router <-> MCU UART (10 bits per byte at `baud`), and the servo
/// bus itself at 1 Mbit/s: the request, then the servo's return delay and its reply, or the full wait for an ID
/// that does not answer. Calibrated against the live daemon: 1 servo + 2 pings = 17.4 ms average.
#[derive(Clone, Copy, Debug)]
pub struct LinkModel {
    pub fixed_ms: f64,
    pub baud: f64,
    pub return_delay_us: f64,
}

impl LinkModel {
    pub fn batch_ms(&self, frames: &[Vec<u8>], replies: &[usize], wait_us: i64) -> f64 {
        let tx: usize = 12 + frames.iter().map(|f| 1 + f.len()).sum::<usize>();
        let rx: usize = 8 + replies.iter().map(|r| 1 + r).sum::<usize>();
        let link = (tx + rx) as f64 * 10.0 / self.baud * 1000.0;
        let bus: f64 = frames
            .iter()
            .zip(replies)
            .map(|(f, &r)| f.len() as f64 * 0.01 + if r > 0 { self.return_delay_us / 1000.0 + r as f64 * 0.01 } else { wait_us as f64 / 1000.0 })
            .sum();
        self.fixed_ms + link + bus
    }
}

impl FakeBus {
    pub fn new(servos: &[(u8, u16)]) -> Self {
        FakeBus { servos: servos.iter().map(|&(id, p)| FakeServo::new(id, p)).collect(), echo: false, ..Default::default() }
    }

    pub fn on(&mut self, id: u8) -> Vec<&mut FakeServo> {
        self.servos.iter_mut().filter(|s| s.id() == id).collect()
    }

    fn answer(&mut self, pkt: &[u8]) -> Vec<u8> {
        if pkt.len() < 6 {
            return vec![];
        }
        let (id, inst) = (pkt[2], pkt[4]);
        let params = &pkt[5..pkt.len() - 1];
        if id == feetech::BROADCAST {
            if inst == feetech::INST_SYNC_WRITE && params.len() >= 2 {
                let (addr, ln) = (params[0] as usize, params[1] as usize);
                for ch in params[2..].chunks(ln + 1) {
                    if ch.len() == ln + 1 {
                        for s in self.on(ch[0]) {
                            s.write(addr, &ch[1..]);
                        }
                    }
                }
            }
            return vec![];
        }
        let n = self.servos.iter().filter(|s| s.id() == id).count();
        if n == 0 {
            return vec![];
        }
        let data: Vec<u8> = match inst {
            feetech::INST_PING => vec![],
            feetech::INST_READ if params.len() == 2 => {
                let drift = self.drift;
                let s = self.servos.iter_mut().find(|s| s.id() == id).unwrap();
                let (a, l) = (params[0] as usize, params[1] as usize);
                if drift > 0 && a <= 56 && a + l > 57 {
                    let p = (s.mem[56] as u16 | (s.mem[57] as u16) << 8).wrapping_add(drift);
                    s.mem[56] = p as u8;
                    s.mem[57] = (p >> 8) as u8;
                }
                s.mem[a.min(71)..(a + l).min(71)].to_vec()
            }
            feetech::INST_WRITE if !params.is_empty() => {
                let reply_from = self.servos.iter().position(|s| s.id() == id).unwrap();
                let addr = params[0] as usize;
                let stuck = self.eeprom_stuck;
                for s in self.on(id) {
                    // a servo whose EEPROM ignores writes still answers: the write looks like it worked
                    if stuck && (addr < 40 || params.get(1) == Some(&128)) {
                        continue;
                    }
                    s.write(addr, &params[1..]);
                }
                // the status packet comes from the servo at its (possibly new) ID
                return self.servos[reply_from].status(&[]);
            }
            _ => vec![],
        };
        if n >= 2 {
            // two servos answer at once: one reply in four survives intact, the rest overlap into 1..6 bytes
            self.garble += 1;
            if self.garble % 4 != 0 {
                let k = 1 + (self.garble % 6) as usize;
                return (0..k).map(|i| 0xf0 ^ (i as u8 * 37)).collect();
            }
        }
        self.servos.iter().find(|s| s.id() == id).unwrap().status(&data)
    }
}

impl Io for FakeBus {
    fn batch(&mut self, frames: &[Vec<u8>], expects: &[usize], _wait_us: i64, _timeout: Duration) -> Result<Vec<Vec<u8>>, String> {
        self.rpcs += 1;
        if self.stall_from.is_some_and(|n| self.rpcs >= n) {
            return Err("router call timed out".into());
        }
        if self.forgot {
            return Err(format!("bus_batch answered 0 frames for {}", frames.len()));
        }
        let mut out = Vec::new();
        for (f, &e) in frames.iter().zip(expects) {
            let mut got = if self.echo { f.clone() } else { vec![] };
            got.extend(self.answer(f));
            got.truncate(e.min(250));
            out.push(got);
        }
        if let Some(m) = self.link {
            let lens: Vec<usize> = out.iter().map(|o| o.len()).collect();
            self.modeled_ms += m.batch_ms(frames, &lens, _wait_us);
        }
        Ok(out)
    }

    fn begin(&mut self) -> Result<(), String> {
        if self.begin_fails {
            return Err("mcu error: method bus_begin not available".into());
        }
        self.forgot = false;
        Ok(())
    }

    fn recover(&mut self, hard: bool) -> Result<String, String> {
        if !hard {
            self.soft_recovers += 1;
            if self.stall_soft_heals && !self.stall_sticky {
                self.stall_from = None;
                return Ok("fake bus: router socket reconnected".into());
            }
            return Err("fake bus: reconnect did not open the bus".into());
        }
        self.recovers += 1;
        if self.stall_sticky {
            return Err("fake bus: restarted, bus_begin still fails".into());
        }
        self.stall_from = None;
        self.begin_fails = false;
        Ok("fake bus: MCU app restarted".into())
    }

    fn connected(&self) -> bool {
        self.stall_from.is_none()
    }

    fn reconnects(&self) -> u64 {
        self.recovers
    }

    fn modeled_ms(&self) -> f64 {
        self.modeled_ms
    }

    fn poll_set(&mut self, ids: &[u8]) -> Result<(), String> {
        if !self.poll_capable {
            return Err("method poll_set not available".into());
        }
        self.rpcs += 1;
        self.poll = Some(ids.to_vec());
        Ok(())
    }

    /// One sweep per get, laid out exactly like the sketch's poll_get.
    fn imu_scan(&mut self) -> Result<Vec<(u8, u8)>, String> {
        Ok(self.imu.map(|(a, _)| vec![(self.imu_on_bus, a)]).unwrap_or_default())
    }

    fn imu_begin(&mut self, addr: u8) -> Result<Vec<i64>, String> {
        match self.imu {
            Some((a, _)) if a == addr => {
                self.imu_began = true;
                Ok(vec![1, self.imu_on_bus as i64, 0x68, 0x01, 3, 0x08, 0x08, 4])
            }
            // the chip is not there: no bus is named and who_am_i reads -1. Never a quiet default.
            _ => Ok(vec![0, -1, -1, -1, -1, -1, -1, -1]),
        }
    }

    fn imu_read(&mut self) -> Result<Vec<u8>, String> {
        self.imu_read_rpcs.fetch_add(1, Ordering::Relaxed);
        if !self.imu_began || self.imu_read_fails {
            return Ok(vec![]);
        }
        let Some((_, v)) = self.imu else { return Ok(vec![]) };
        let mut out = Vec::with_capacity(14);
        for x in v {
            out.push((x >> 8) as u8);
            out.push(x as u8);
        }
        Ok(out)
    }

    fn poll_get(&mut self) -> Result<Vec<u8>, String> {
        if !self.poll_capable {
            return Err("method poll_get not available".into());
        }
        self.rpcs += 1;
        if self.stall_from.is_some_and(|n| self.rpcs >= n) {
            return Err("router call timed out".into());
        }
        let ids = self.poll.clone().unwrap_or_default();
        if self.forgot {
            // the sketch's loop() idles while g_baud is 0: the RPC answers, the sweep count stays 0
            let mut o = vec![0, 0, 0, ids.len() as u8];
            o.extend(std::iter::repeat(0).take(4 * ids.len() + 6));
            return Ok(o);
        }
        let mut o = vec![self.poll_turn as u8, (self.poll_turn >> 8) as u8, 1, ids.len() as u8];
        for &id in &ids {
            let r = self.answer(&feetech::read(id, 56, 2));
            if r.len() == 8 && r[2] == id {
                o.extend_from_slice(&[id, if r[4] != 0 { 3 } else { 0 }, r[5], r[6]]);
            } else if r.is_empty() {
                o.extend_from_slice(&[id, 1, 0, 0]);
            } else {
                o.extend_from_slice(&[id, 2, r.len() as u8, 0]);
            }
        }
        let block = |me: &mut Self, id: u8, addr: u8, len: u8| -> Vec<u8> {
            let r = me.answer(&feetech::read(id, addr, len));
            if r.len() == len as usize + 6 && r[2] == id {
                let mut b = vec![id, len];
                b.extend_from_slice(&r[5..5 + len as usize]);
                b
            } else {
                vec![0, 0]
            }
        };
        if ids.is_empty() {
            o.extend_from_slice(&[0, 0, 0, 0]);
        } else {
            let fid = ids[self.poll_turn as usize % ids.len()];
            let b = block(self, fid, 40, 31);
            o.extend(b);
            if self.poll_turn % 8 == 0 {
                let eid = ids[(self.poll_turn as usize / 8) % ids.len()];
                let b = block(self, eid, 5, 13);
                o.extend(b);
            } else {
                o.extend_from_slice(&[0, 0]);
            }
        }
        let mut found = Vec::new();
        let mut wraps = 0u8;
        for _ in 0..2 {
            let id = self.poll_scan;
            if !self.answer(&feetech::ping(id)).is_empty() {
                found.push(id);
            }
            self.poll_scan = self.poll_scan.wrapping_add(1);
            if self.poll_scan > 253 {
                self.poll_scan = 0;
                wraps += 1;
            }
        }
        self.poll_turn += 1;
        o.push(wraps);
        o.push(found.len() as u8);
        o.extend(found);
        if self.imu_in_poll {
            // exactly the sketch's trailer: have, the 14 burst bytes, then the snapshot's age LE
            let have = self.imu_began && !self.imu_read_fails && self.imu.is_some();
            o.push(have as u8);
            match (have, self.imu) {
                (true, Some((_, v))) => {
                    for x in v {
                        o.push((x >> 8) as u8);
                        o.push(x as u8);
                    }
                }
                _ => o.extend(std::iter::repeat_n(0u8, 14)),
            }
            o.push(self.imu_snap_age_ms as u8);
            o.push((self.imu_snap_age_ms >> 8) as u8);
        }
        if let Some(m) = self.link {
            // the RPC carries only the snapshot; the sweep itself runs on the MCU between gets
            self.modeled_ms += m.fixed_ms + (14 + 8 + o.len()) as f64 * 10.0 / m.baud * 1000.0;
        }
        Ok(o)
    }
}
