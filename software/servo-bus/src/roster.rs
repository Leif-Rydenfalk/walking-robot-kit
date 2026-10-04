//! The live roster: one row per servo ID the bus answers on, built from a continuous ping scan of
//! 0..253 interleaved with telemetry reads. A row is what the bus says NOW, never an aggregate of a
//! recording (the 14:32 fault of the servo.bench page: rows listed "clean 96-100%" for servos that
//! had been unplugged for minutes). So every row carries `last seen`, greys out after STALE_S without
//! a reply, and leaves the roster once a full scan that started after its last reply did not find it.
//!
//! Duplicate IDs. Two servos on one ID both answer every read and their packets overlap on the wire:
//! bytes come back but do not parse (2026-10-04 ~14:10: ID 1 read 10/40 clean, reply lengths 1-6 bytes,
//! while the adapter at 100 read 40/40). That signature, garbled replies on one ID while another ID
//! reads clean, is the duplicate verdict. Two clean encoders disagreeing by more than JUMP counts on
//! consecutive reads is the second signature (servo-bench `classify`, same thresholds).

use crate::feetech::{Eeprom, Telemetry};
use std::collections::{BTreeMap, VecDeque};

pub const WINDOW: usize = 40; // reads kept per ID for the clean-read rate
pub const STALE_S: f64 = 2.0; // no reply this long: the row greys out
pub const CLEAN_OK: f64 = 0.97;
pub const DUP_GARBLED: f64 = 0.20; // garbled share at or above this, with another ID clean: two servos
pub const JUMP: i32 = 300; // counts between consecutive clean reads: two encoders
pub const MIN_READS: usize = 10;

#[derive(Clone, Debug, PartialEq)]
pub enum Read {
    Clean(Telemetry),
    Garbled(usize), // bytes came back (count, echo stripped) but no valid status packet
    Nothing,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Mark {
    Clean(u16),
    Garbled(u8),
    Nothing,
}

#[derive(Clone, Debug)]
pub struct Row {
    pub id: u8,
    pub first_seen: f64,
    pub last_seen: f64,  // any bytes back (read or ping)
    pub last_clean: f64, // a parsed telemetry read
    pub tele: Option<Telemetry>,
    pub eeprom: Option<Eeprom>,
    win: VecDeque<Mark>,
    /// Since the row joined, for the benchmark :
    /// every read's verdict, and the ms between consecutive clean reads (last GAPS of them).
    pub n_clean: u64,
    pub n_garbled: u64,
    pub n_nothing: u64,
    pub gaps: VecDeque<f32>,
}

pub const GAPS: usize = 1024;

/// (p50, p99, max) of a set of ms values.
pub fn pctl(v: &VecDeque<f32>) -> Option<(f32, f32, f32)> {
    if v.is_empty() {
        return None;
    }
    let mut s: Vec<f32> = v.iter().copied().collect();
    s.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let at = |q: f64| s[((s.len() - 1) as f64 * q).round() as usize];
    Some((at(0.5), at(0.99), *s.last().unwrap()))
}

impl Row {
    fn new(id: u8, t: f64) -> Self {
        Row { id, first_seen: t, last_seen: t, last_clean: 0.0, tele: None, eeprom: None, win: VecDeque::with_capacity(WINDOW), n_clean: 0, n_garbled: 0, n_nothing: 0, gaps: VecDeque::with_capacity(GAPS) }
    }

    pub fn reads(&self) -> usize {
        self.win.len()
    }

    pub fn clean(&self) -> usize {
        self.win.iter().filter(|m| matches!(m, Mark::Clean(_))).count()
    }

    pub fn garbled(&self) -> usize {
        self.win.iter().filter(|m| matches!(m, Mark::Garbled(_))).count()
    }

    pub fn clean_frac(&self) -> f64 {
        if self.win.is_empty() {
            0.0
        } else {
            self.clean() as f64 / self.win.len() as f64
        }
    }

    /// Reply lengths of the garbled reads, as (length, count): the evidence shown with a duplicate.
    pub fn garbled_lengths(&self) -> Vec<(u8, usize)> {
        let mut m: BTreeMap<u8, usize> = BTreeMap::new();
        for k in &self.win {
            if let Mark::Garbled(n) = k {
                *m.entry(*n).or_default() += 1;
            }
        }
        m.into_iter().collect()
    }

    /// Consecutive clean positions that disagree by more than JUMP counts (circular, 4096 per turn).
    pub fn jumps(&self) -> usize {
        let pos: Vec<i32> = self.win.iter().filter_map(|m| if let Mark::Clean(p) = m { Some(*p as i32) } else { None }).collect();
        pos.windows(2)
            .filter(|w| {
                let d = (w[0] - w[1]).rem_euclid(4096);
                d.min(4096 - d) > JUMP
            })
            .count()
    }

    pub fn stale(&self, now: f64) -> bool {
        let since = if self.last_clean > 0.0 { self.last_clean } else { self.first_seen };
        now - since > STALE_S
    }

    fn push(&mut self, m: Mark) {
        if self.win.len() == WINDOW {
            self.win.pop_front();
        }
        self.win.push_back(m);
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Verdict {
    Checking,
    Ok,
    Duplicate(String),
    Weak(String),
    Silent,
}

impl Verdict {
    pub fn code(&self) -> &'static str {
        match self {
            Verdict::Checking => "checking",
            Verdict::Ok => "ok",
            Verdict::Duplicate(_) => "duplicate",
            Verdict::Weak(_) => "weak",
            Verdict::Silent => "silent",
        }
    }
}

#[derive(Clone, Debug)]
pub struct Event {
    pub t: f64,
    pub id: Option<u8>,
    pub kind: &'static str,
    pub text: String,
}

#[derive(Clone, Debug, Default)]
pub struct Roster {
    pub rows: BTreeMap<u8, Row>,
    pub events: VecDeque<Event>,
    pub scans: u64,
    pub last_scan: Vec<u8>,
    pub last_scan_t: f64,
    dup_said: BTreeMap<u8, bool>,
    /// Every event is also appended here as one JSON line, so a bus stop can be lined up against what was
    /// plugged in at the time. Board: /home/arduino/servo-bus-events.jsonl.
    pub log_path: Option<String>,
}

impl Roster {
    pub fn ids(&self) -> Vec<u8> {
        self.rows.keys().copied().collect()
    }

    pub fn event(&mut self, t: f64, id: Option<u8>, kind: &'static str, text: String) {
        if self.events.len() == 60 {
            self.events.pop_front();
        }
        if let Some(p) = &self.log_path {
            use std::io::Write;
            let line = crate::json::dump(&crate::json::J::obj(vec![
                ("t", crate::json::J::n(t)),
                ("id", id.map(|i| crate::json::J::n(i as f64)).unwrap_or(crate::json::J::Nil)),
                ("kind", crate::json::J::s(kind)),
                ("text", crate::json::J::s(&text)),
            ]));
            if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(p) {
                let _ = writeln!(f, "{line}");
            }
        }
        self.events.push_back(Event { t, id, kind, text });
    }

    fn join(&mut self, id: u8, t: f64) -> &mut Row {
        if !self.rows.contains_key(&id) {
            self.rows.insert(id, Row::new(id, t));
            self.event(t, Some(id), "joined", format!("ID {id} answered on the bus"));
        }
        self.rows.get_mut(&id).unwrap()
    }

    /// One telemetry read of `id` at time t.
    pub fn feed(&mut self, id: u8, t: f64, r: Read) {
        let row = match self.rows.get_mut(&id) {
            Some(r) => r,
            None => {
                if r == Read::Nothing {
                    return;
                }
                self.join(id, t)
            }
        };
        match r {
            Read::Clean(tel) => {
                row.n_clean += 1;
                if row.last_clean > 0.0 {
                    if row.gaps.len() == GAPS {
                        row.gaps.pop_front();
                    }
                    row.gaps.push_back(((t - row.last_clean) * 1000.0) as f32);
                }
                row.push(Mark::Clean(tel.pos));
                row.last_seen = t;
                row.last_clean = t;
                row.tele = Some(tel);
            }
            Read::Garbled(n) => {
                row.n_garbled += 1;
                row.push(Mark::Garbled(n.min(255) as u8));
                row.last_seen = t;
            }
            Read::Nothing => {
                row.n_nothing += 1;
                row.push(Mark::Nothing)
            }
        }
    }

    pub fn feed_eeprom(&mut self, id: u8, e: Eeprom) {
        if let Some(r) = self.rows.get_mut(&id) {
            r.eeprom = Some(e);
        }
    }

    /// A ping answered (any bytes back) during the scan: the ID is on the bus.
    pub fn pinged(&mut self, id: u8, t: f64) {
        self.join(id, t).last_seen = t;
    }

    /// A full 0..253 scan finished. It started at `started`; a row it did not find, whose last reply is older
    /// than both the scan start and STALE_S, has left the bus.
    pub fn scan_done(&mut self, found: &[u8], started: f64, now: f64) {
        self.scans += 1;
        self.last_scan = found.to_vec();
        self.last_scan_t = now;
        let gone: Vec<u8> = self
            .rows
            .values()
            .filter(|r| !found.contains(&r.id) && r.last_seen < started && now - r.last_seen > STALE_S)
            .map(|r| r.id)
            .collect();
        for id in gone {
            self.rows.remove(&id);
            self.dup_said.remove(&id);
            self.event(now, Some(id), "left", format!("ID {id} left the bus (a full scan did not find it)"));
        }
    }

    pub fn forget(&mut self, id: u8) {
        self.rows.remove(&id);
        self.dup_said.remove(&id);
    }

    /// Verdict for one row, judged against the others ("garbled here while another ID reads clean").
    pub fn verdict(&self, id: u8) -> Verdict {
        let Some(r) = self.rows.get(&id) else { return Verdict::Silent };
        let n = r.reads();
        if n < MIN_READS {
            return Verdict::Checking;
        }
        let g = r.garbled();
        let others_clean = self.rows.values().any(|o| o.id != id && o.reads() >= MIN_READS && o.clean_frac() >= CLEAN_OK);
        let lens = r.garbled_lengths().iter().map(|(l, c)| format!("{l} B x{c}")).collect::<Vec<_>>().join(", ");
        if g as f64 >= DUP_GARBLED * n as f64 && (others_clean || g * 2 >= n) {
            return Verdict::Duplicate(format!(
                "{} of {n} reads clean, {g} garbled (reply lengths {lens}){}",
                r.clean(),
                if others_clean { " while other servos read clean" } else { "" }
            ));
        }
        if r.jumps() >= 2 {
            return Verdict::Duplicate(format!("clean reads jump by more than {JUMP} counts {} times: two encoders answer", r.jumps()));
        }
        if r.clean_frac() >= CLEAN_OK {
            return Verdict::Ok;
        }
        if r.clean() == 0 && g == 0 {
            return Verdict::Silent;
        }
        Verdict::Weak(format!("{} of {n} reads clean: loose cable, connector or supply", r.clean()))
    }

    /// Raise a duplicate event once when a row turns duplicate, and once when it clears.
    pub fn note_verdicts(&mut self, t: f64) {
        let ids = self.ids();
        for id in ids {
            let dup = matches!(self.verdict(id), Verdict::Duplicate(_));
            let was = self.dup_said.get(&id).copied().unwrap_or(false);
            if dup != was {
                self.dup_said.insert(id, dup);
                if dup {
                    self.event(t, Some(id), "duplicate", format!("2+ servos answer on ID {id}: unplug one"));
                } else {
                    self.event(t, Some(id), "clear", format!("ID {id} reads clean again"));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tel(pos: u16) -> Telemetry {
        Telemetry { pos, ..Default::default() }
    }

    #[test]
    fn duplicate_when_garbled_beside_a_clean_servo() {
        // today's signature: ID 1 10/40 with 1-6 byte replies while 100 reads 40/40
        let mut r = Roster::default();
        for k in 0..40 {
            let t = k as f64 * 0.04;
            r.feed(100, t, Read::Clean(tel(2980)));
            r.feed(1, t, if k % 4 == 0 { Read::Clean(tel(500)) } else { Read::Garbled(1 + k % 6) });
        }
        assert!(matches!(r.verdict(1), Verdict::Duplicate(_)), "{:?}", r.verdict(1));
        assert_eq!(r.verdict(100), Verdict::Ok);
        r.note_verdicts(2.0);
        assert!(r.events.iter().any(|e| e.kind == "duplicate" && e.id == Some(1)));
    }

    #[test]
    fn negative_control_clean_servos_are_not_duplicates() {
        let mut r = Roster::default();
        for k in 0..40 {
            r.feed(13, k as f64 * 0.04, Read::Clean(tel(240 + k)));
            r.feed(14, k as f64 * 0.04, Read::Clean(tel(682)));
        }
        assert_eq!(r.verdict(13), Verdict::Ok);
        assert_eq!(r.verdict(14), Verdict::Ok);
    }

    #[test]
    fn a_lone_lost_read_is_weak_not_duplicate() {
        let mut r = Roster::default();
        for k in 0..40 {
            r.feed(100, k as f64 * 0.04, Read::Clean(tel(2980)));
            r.feed(13, k as f64 * 0.04, if k % 5 == 0 { Read::Nothing } else { Read::Clean(tel(240)) });
        }
        assert!(matches!(r.verdict(13), Verdict::Weak(_)), "{:?}", r.verdict(13));
    }

    #[test]
    fn two_encoders_jumping_is_duplicate() {
        let mut r = Roster::default();
        for k in 0..40 {
            r.feed(1, k as f64 * 0.04, Read::Clean(tel(if k % 2 == 0 { 250 } else { 1700 })));
        }
        assert!(matches!(r.verdict(1), Verdict::Duplicate(_)));
    }

    #[test]
    fn a_row_leaves_only_after_a_scan_that_started_after_its_last_reply() {
        let mut r = Roster::default();
        r.pinged(12, 10.0);
        r.feed(12, 10.0, Read::Clean(tel(959)));
        // a scan that started before the last reply proves nothing
        r.scan_done(&[], 9.0, 12.5);
        assert!(r.rows.contains_key(&12));
        // within STALE_S: kept (greyed only)
        r.scan_done(&[], 10.5, 11.0);
        assert!(r.rows.contains_key(&12));
        // a later scan without it, more than STALE_S since: gone
        r.scan_done(&[], 11.0, 13.0);
        assert!(!r.rows.contains_key(&12));
        assert!(r.events.iter().any(|e| e.kind == "left"));
    }
}
