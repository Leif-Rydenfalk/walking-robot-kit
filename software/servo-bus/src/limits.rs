//! Per-servo angle limits. 07: "make sure we never move the srevos past their limit and have a
//! torque / current limit per soervo and collect data amount their min max"; 17:32: "are we recording the servo
//! minimum and maximum. i dont want to break the jaw for example".
//!
//! Three parts, all inside the bus owner:
//!   - `seen`: every torque-off position read of every servo widens that servo's min/max (the recorder). we turns a
//!     joint by hand end to end; "Record" on the page resets one servo's range first. A jump of more than half a turn
//!     between two reads marks the range `wrapped` (it crossed 4095 -> 0) and Save refuses it.
//!   - `map`: the saved limits per ID (file `--limits`, next to the registry on the board): the recorded ends pulled
//!     in by MARGIN (3 deg) each side, with who, when, where the ends came from and the EEPROM read-back.
//!   - `band`/`check`: the one clamp every motion path asks. No saved limits and factory EEPROM (0..4095) = no band =
//!     the target is refused. The EEPROM limits (regs 9/11) are written from `map` by `Engine::write_limits`, so the
//!     servo refuses past-limit goals itself as well.

use crate::json::{self, J};
use std::collections::BTreeMap;

/// 4096 counts per turn.
pub const COUNTS_PER_DEG: f64 = 4096.0 / 360.0;
/// 3 deg in from each recorded end stop (the brief, servo-limits-task.md).
pub const MARGIN: u16 = 34;
/// A saved band narrower than this (about 5 deg) is a recording that never reached the ends.
pub const MIN_SPAN: u16 = 57;

#[derive(Clone, Debug, PartialEq)]
pub struct Lim {
    pub min: u16,
    pub max: u16,
    pub rec_min: u16,
    pub rec_max: u16,
    pub margin: u16,
    pub when: String,
    pub who: String,
    pub source: String,
    /// "regs 9/11 read back <min>/<max> at <stamp>", empty until the EEPROM write is proven
    pub eeprom: String,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Seen {
    pub min: u16,
    pub max: u16,
    pub last: u16,
    pub n: u64,
    pub since: f64,
    pub wrapped: bool,
}

#[derive(Clone, Debug, Default)]
pub struct Limits {
    pub path: Option<String>,
    pub map: BTreeMap<u8, Lim>,
    pub seen: BTreeMap<u8, Seen>,
    pub log: Vec<J>,
    /// last refusal event per ID (the page slider can send 50 goals a second; one event per second is enough)
    pub said: BTreeMap<u8, f64>,
}

/// Recorded ends -> limits, or why not.
pub fn from_ends(rec_min: u16, rec_max: u16) -> Result<(u16, u16), String> {
    if rec_max > 4095 || rec_min >= rec_max {
        return Err(format!("recorded ends {rec_min}..{rec_max} are not a range"));
    }
    let (lo, hi) = (rec_min + MARGIN, rec_max.saturating_sub(MARGIN));
    if hi < lo || hi - lo < MIN_SPAN {
        return Err(format!(
            "recorded range {rec_min}..{rec_max} ({:.0} deg) is too small after 3 deg each side: turn the joint end to end first",
            (rec_max - rec_min) as f64 / COUNTS_PER_DEG
        ));
    }
    Ok((lo, hi))
}

impl Limits {
    pub fn load(path: &str) -> Limits {
        let mut l = Limits { path: Some(path.to_string()), ..Default::default() };
        if let Some(v) = std::fs::read_to_string(path).ok().and_then(|s| json::parse(&s)) {
            l.read(&v);
        }
        l
    }

    pub fn read(&mut self, v: &J) {
        if let Some(J::Obj(a)) = v.get("servos") {
            for (k, e) in a {
                let Ok(id) = k.parse::<u8>() else { continue };
                let n = |f: &str| e.get(f).and_then(|x| x.i64()).unwrap_or(-1);
                let s = |f: &str| e.get(f).and_then(|x| x.as_str()).unwrap_or("").to_string();
                let (min, max) = (n("min"), n("max"));
                if !(0..=4095).contains(&min) || !(0..=4095).contains(&max) || min >= max {
                    continue;
                }
                self.map.insert(
                    id,
                    Lim {
                        min: min as u16,
                        max: max as u16,
                        rec_min: n("rec_min").clamp(0, 4095) as u16,
                        rec_max: n("rec_max").clamp(0, 4095) as u16,
                        margin: n("margin").clamp(0, 4095) as u16,
                        when: s("when"),
                        who: s("who"),
                        source: s("source"),
                        eeprom: s("eeprom"),
                    },
                );
            }
        }
        if let Some(J::Arr(l)) = v.get("log") {
            self.log = l.clone();
        }
    }

    pub fn lim_json(l: &Lim) -> J {
        J::obj(vec![
            ("min", J::n(l.min as f64)),
            ("max", J::n(l.max as f64)),
            ("rec_min", J::n(l.rec_min as f64)),
            ("rec_max", J::n(l.rec_max as f64)),
            ("margin", J::n(l.margin as f64)),
            ("when", J::s(&l.when)),
            ("who", J::s(&l.who)),
            ("source", J::s(&l.source)),
            ("eeprom", J::s(&l.eeprom)),
        ])
    }

    pub fn seen_json(s: &Seen) -> J {
        J::obj(vec![("min", J::n(s.min as f64)), ("max", J::n(s.max as f64)), ("n", J::n(s.n as f64)), ("since", J::n(s.since)), ("wrapped", J::Bool(s.wrapped))])
    }

    pub fn to_json(&self) -> J {
        J::obj(vec![
            ("what", J::s("Per-servo angle limits in counts (4096 per turn): recorded end stops pulled in 3 deg each side. Owned by servo-bus on the board; every motion command is clamped to these and the servo EEPROM (regs 9/11) holds the same values.")),
            ("servos", J::Obj(self.map.iter().map(|(id, l)| (id.to_string(), Self::lim_json(l))).collect())),
            ("log", J::Arr(self.log.clone())),
        ])
    }

    pub fn save(&self) -> Result<(), String> {
        let Some(p) = &self.path else { return Ok(()) };
        let tmp = format!("{p}.tmp");
        std::fs::write(&tmp, json::dump(&self.to_json())).map_err(|e| format!("write {tmp}: {e}"))?;
        std::fs::rename(&tmp, p).map_err(|e| format!("rename {p}: {e}"))
    }

    /// One position read of `id`. Only torque-off reads count: a held servo is where the software put it.
    pub fn observe(&mut self, id: u8, pos: u16, torque: bool, t: f64) {
        if torque || pos > 4095 {
            return;
        }
        let s = self.seen.entry(id).or_insert(Seen { min: pos, max: pos, last: pos, n: 0, since: t, wrapped: false });
        if s.n > 0 && s.last.abs_diff(pos) > 2048 {
            s.wrapped = true;
        }
        s.min = s.min.min(pos);
        s.max = s.max.max(pos);
        s.last = pos;
        s.n += 1;
    }

    /// "Record" on the page: start this servo's range again from the next read.
    pub fn reset_seen(&mut self, id: u8) {
        self.seen.remove(&id);
    }

    /// The band a target must stay inside: saved limits first, else non-factory EEPROM limits, else none.
    pub fn band(&self, id: u8, eeprom: Option<(u16, u16)>) -> Option<(u16, u16)> {
        if let Some(l) = self.map.get(&id) {
            return Some((l.min, l.max));
        }
        eeprom.filter(|&(a, b)| a < b && !(a == 0 && b >= 4095))
    }

    /// Ok(pos) inside the band, else why it is refused.
    pub fn check(&self, id: u8, pos: u16, eeprom: Option<(u16, u16)>) -> Result<u16, String> {
        match self.band(id, eeprom) {
            None => Err(format!("ID {id} has no angle limits yet: record and save them first (target {pos} refused)")),
            Some((lo, hi)) if pos < lo || pos > hi => Err(format!("ID {id}: target {pos} is outside its limits {lo}..{hi}: refused")),
            Some(_) => Ok(pos),
        }
    }

    /// True when a refusal event for `id` is due (at most one a second).
    pub fn should_say(&mut self, id: u8, t: f64) -> bool {
        let due = self.said.get(&id).map(|&l| t - l >= 1.0).unwrap_or(true);
        if due {
            self.said.insert(id, t);
        }
        due
    }

    pub fn set(&mut self, id: u8, l: Lim) {
        self.log.push(J::obj(vec![
            ("t", J::s(&l.when)),
            ("id", J::n(id as f64)),
            ("min", J::n(l.min as f64)),
            ("max", J::n(l.max as f64)),
            ("rec_min", J::n(l.rec_min as f64)),
            ("rec_max", J::n(l.rec_max as f64)),
            ("who", J::s(&l.who)),
            ("source", J::s(&l.source)),
            ("eeprom", J::s(&l.eeprom)),
        ]));
        self.map.insert(id, l);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ends_pull_in_three_degrees() {
        assert_eq!(from_ends(45, 390), Ok((79, 356)));
        assert!(from_ends(300, 340).is_err(), "40 counts is not end to end");
        assert!(from_ends(390, 45).is_err());
    }

    #[test]
    fn no_limits_no_motion_and_factory_eeprom_is_no_limit() {
        let l = Limits::default();
        assert!(l.check(34, 288, None).is_err());
        assert!(l.check(34, 288, Some((0, 4095))).is_err());
        assert_eq!(l.check(34, 288, Some((79, 356))), Ok(288));
    }

    #[test]
    fn saved_limits_win_and_refuse_outside() {
        let mut l = Limits::default();
        let (min, max) = from_ends(45, 390).unwrap();
        l.set(34, Lim { min, max, rec_min: 45, rec_max: 390, margin: MARGIN, when: "t".into(), who: "test".into(), source: "test".into(), eeprom: String::new() });
        assert_eq!(l.check(34, 200, Some((0, 4095))), Ok(200));
        assert!(l.check(34, 2048, None).is_err(), "the old global band centre would break the jaw");
        assert!(l.check(34, 78, None).is_err());
        assert!(l.check(34, 357, None).is_err());
        // round trip through the file shape
        let mut l2 = Limits::default();
        l2.read(&json::parse(&json::dump(&l.to_json())).unwrap());
        assert_eq!(l2.map[&34], l.map[&34]);
    }

    #[test]
    fn recorder_keeps_torque_off_ends_and_flags_a_wrap() {
        let mut l = Limits::default();
        for (p, tq) in [(288, false), (45, false), (390, false), (3000, true), (200, false)] {
            l.observe(34, p, tq, 1.0);
        }
        let s = &l.seen[&34];
        assert_eq!((s.min, s.max, s.n, s.wrapped), (45, 390, 4, false));
        l.observe(34, 4090, false, 2.0);
        assert!(l.seen[&34].wrapped);
        l.reset_seen(34);
        assert!(!l.seen.contains_key(&34));
    }
}
