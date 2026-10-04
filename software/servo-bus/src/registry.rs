//! The servo ID registry: which ID belongs to which physical servo, and every change with time, old,
//! new, joint and who. 28, after a new servo was given ID 12 while another servo had
//! held 12 since 13:10: "alright thats a massive fuck up". So an ID written to a servo is TAKEN until the
//! registry says it moved, whether or not that servo is plugged in right now, and set-ID refuses it.
//!
//! The page owns the JSON file (`--registry`, on the board next to the daemon). The joint table follows the
//! Open Duck Mini numbering: the same ID means the same joint in every unit.

use crate::json::{self, J};
use std::collections::BTreeMap;

/// SERVO-IDS.md "The table": ID -> joint, the plain words the page shows.
pub const JOINTS: &[(u8, &str)] = &[
    (10, "Right hip yaw"),
    (11, "Right hip roll"),
    (12, "Right hip pitch"),
    (13, "Right knee"),
    (14, "Right ankle"),
    (20, "Left hip yaw"),
    (21, "Left hip roll"),
    (22, "Left hip pitch"),
    (23, "Left knee"),
    (24, "Left ankle"),
    (30, "Neck pitch"),
    (31, "Head pitch"),
    (32, "Head yaw"),
    (33, "Head roll"),
    (34, "Mouth"),
    (100, "Bench adapter"),
    (200, "IMU board"),
];

pub const FACTORY_ID: u8 = 1;

pub fn joint_of(id: u8) -> Option<&'static str> {
    JOINTS.iter().find(|j| j.0 == id).map(|j| j.1)
}

#[derive(Clone, Debug, PartialEq)]
pub struct Entry {
    pub joint: String,
    pub since: String,
    pub source: String,
}

#[derive(Clone, Debug, Default)]
pub struct Registry {
    pub path: Option<String>,
    pub assigned: BTreeMap<u8, Entry>,
    pub log: Vec<J>,
}

impl Registry {
    pub fn load(path: &str) -> Registry {
        let mut r = Registry { path: Some(path.to_string()), ..Default::default() };
        if let Some(v) = std::fs::read_to_string(path).ok().and_then(|s| json::parse(&s)) {
            r.read(&v);
        }
        r
    }

    pub fn read(&mut self, v: &J) {
        if let Some(J::Obj(a)) = v.get("assigned") {
            for (k, e) in a {
                if let Ok(id) = k.parse::<u8>() {
                    let s = |f: &str| e.get(f).and_then(|x| x.as_str()).unwrap_or("").to_string();
                    self.assigned.insert(id, Entry { joint: s("joint"), since: s("since"), source: s("source") });
                }
            }
        }
        if let Some(J::Arr(l)) = v.get("log") {
            self.log = l.clone();
        }
    }

    pub fn to_json(&self) -> J {
        J::obj(vec![
            ("what", J::s("Servo ID registry: an ID here is written to a physical servo and is taken until a change moves it. Owned by servo-bus on the board; mirrored into SERVO-IDS.md.")),
            (
                "assigned",
                J::Obj(
                    self.assigned
                        .iter()
                        .map(|(id, e)| {
                            (id.to_string(), J::obj(vec![("joint", J::s(&e.joint)), ("since", J::s(&e.since)), ("source", J::s(&e.source))]))
                        })
                        .collect(),
                ),
            ),
            ("log", J::Arr(self.log.clone())),
        ])
    }

    pub fn save(&self) -> Result<(), String> {
        let Some(p) = &self.path else { return Ok(()) };
        let tmp = format!("{p}.tmp");
        std::fs::write(&tmp, json::dump(&self.to_json())).map_err(|e| format!("write {tmp}: {e}"))?;
        std::fs::rename(&tmp, p).map_err(|e| format!("rename {tmp}: {e}"))
    }

    /// The name a person reads for this ID: the registry's joint, else the joint table, else plain words.
    pub fn name(&self, id: u8) -> String {
        if let Some(e) = self.assigned.get(&id) {
            if !e.joint.is_empty() {
                return e.joint.clone();
            }
        }
        if let Some(j) = joint_of(id) {
            return j.to_string();
        }
        if id == FACTORY_ID {
            return "New servo (factory ID 1)".into();
        }
        format!("Unassigned (ID {id})")
    }

    /// Is `new` free to write onto the servo now at `old`? None = free; Some(why) = taken.
    pub fn taken(&self, new: u8, old: u8) -> Option<String> {
        if new == old {
            return None;
        }
        self.assigned.get(&new).map(|e| {
            format!("ID {new} is already given to another servo ({}, since {}) even if it is not plugged in now", e.joint, e.since)
        })
    }

    /// A free temporary ID in 101..199 (not on the bus, not in the registry): our 14:28 way out.
    pub fn temporary(&self, on_bus: &[u8]) -> Option<u8> {
        (101..=199u8).find(|i| !self.assigned.contains_key(i) && !on_bus.contains(i))
    }

    pub fn record(&mut self, t: &str, old: u8, new: u8, joint: &str, who: &str, checks: J) {
        let moved = self.assigned.remove(&old);
        let joint = if joint.is_empty() { moved.as_ref().map(|e| e.joint.clone()).unwrap_or_else(|| self.name(new)) } else { joint.to_string() };
        self.assigned.insert(new, Entry { joint: joint.clone(), since: t.to_string(), source: format!("set by {who} on the bus page") });
        self.log.push(J::obj(vec![
            ("t", J::s(t)),
            ("old", J::n(old as f64)),
            ("new", J::n(new as f64)),
            ("joint", J::s(&joint)),
            ("who", J::s(who)),
            ("checks", checks),
        ]));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_registered_id_is_taken_even_when_unplugged() {
        let mut r = Registry::default();
        r.assigned.insert(12, Entry { joint: "Right hip pitch".into(), since: "2026-10-04 13:10".into(), source: "SERVO-IDS.md".into() });
        assert!(r.taken(12, 1).is_some());
        assert!(r.taken(13, 1).is_none());
        // renaming a servo onto its own ID is not a collision
        assert!(r.taken(12, 12).is_none());
        assert_eq!(r.temporary(&[101, 100]), Some(102));
    }

    #[test]
    fn record_moves_the_entry_and_round_trips() {
        let mut r = Registry::default();
        r.assigned.insert(112, Entry { joint: "Right hip pitch (temporary)".into(), since: "x".into(), source: "y".into() });
        r.record("2026-10-04 15:00", 112, 12, "Right hip pitch", "we", J::Nil);
        assert!(!r.assigned.contains_key(&112));
        assert_eq!(r.assigned[&12].joint, "Right hip pitch");
        let mut back = Registry::default();
        back.read(&json::parse(&json::dump(&r.to_json())).unwrap());
        assert_eq!(back.assigned, r.assigned);
        assert_eq!(back.log.len(), 1);
    }
}
