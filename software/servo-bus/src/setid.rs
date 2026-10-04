//! Safe set-ID, ported from robot `sim/unoq_leg_real.py` `safe_setid` and the jig rules in
//! SERVO-IDS.md, run INSIDE the bus owner so nothing else reads the bus while it writes (2026-10-04 13:20:
//! a set-ID ran beside the recorder and the bus wedged).
//!
//! Refused, nothing written, when:
//!   - the old ID does not read 30 of 30 clean (two servos on it: a write would renumber both, 2026-10-03 17:43);
//!   - the new ID answers on the bus now;
//!   - the new ID is in the registry for another servo, plugged in or not 28);
//!   - the new ID is 0, 254 (broadcast) or 255, or equals the old.
//! A refusal offers a free temporary 1xx ID. After the write: 30 of 30 clean at the new ID, the old one gone,
//! reg 5 read back; then the bench defaults, max torque (16) and torque limit (48) = 300 (30 %) and torque
//! off, each read back. The change is appended to the registry with time, old, new, joint and who.

use crate::feetech::{self, REG_ID, REG_LOCK, REG_MAX_TORQUE, REG_TORQUE_LIMIT, REG_TORQUE};
use crate::io::Io;
use crate::json::J;
use crate::registry::Registry;
use std::time::Duration;

pub const CLEAN_N: usize = 30;
pub const CAP: u16 = 300;

#[derive(Debug, PartialEq)]
pub enum Got {
    Data(Vec<u8>),
    Garbled(usize),
    Nothing,
}

fn one(io: &mut dyn Io, pkt: Vec<u8>, expect: usize, wait_us: i64) -> Result<Vec<u8>, String> {
    let out = io.batch(&[pkt.clone()], &[expect], wait_us, Duration::from_millis(300))?;
    let raw = out.into_iter().next().unwrap_or_default();
    Ok(if raw.starts_with(&pkt) { raw[pkt.len()..].to_vec() } else { raw })
}

/// Classify the bytes that came back for a read of `n` registers at `id` (echo already stripped).
pub fn classify(body: &[u8], id: u8, n: usize) -> Got {
    match feetech::replies(body, &[]).into_iter().find(|r| r.id == id && r.data.len() == n) {
        Some(r) => Got::Data(r.data),
        None if body.is_empty() => Got::Nothing,
        None => Got::Garbled(body.len()),
    }
}

pub fn ping(io: &mut dyn Io, id: u8) -> Result<bool, String> {
    Ok(!one(io, feetech::ping(id), 6, 1000)?.is_empty())
}

pub fn read(io: &mut dyn Io, id: u8, addr: u8, n: u8) -> Result<Got, String> {
    let body = one(io, feetech::read(id, addr, n), n as usize + 6, 3000)?;
    Ok(classify(&body, id, n as usize))
}

pub fn write(io: &mut dyn Io, id: u8, addr: u8, data: &[u8]) -> Result<(), String> {
    one(io, feetech::write(id, addr, data), 6, 3000).map(|_| ())
}

fn clean_reads(io: &mut dyn Io, id: u8) -> Result<usize, String> {
    let mut good = 0;
    for _ in 0..CLEAN_N {
        if matches!(read(io, id, 56, 15)?, Got::Data(_)) {
            good += 1;
        }
    }
    Ok(good)
}

fn answers(io: &mut dyn Io, id: u8) -> Result<bool, String> {
    for _ in 0..3 {
        if ping(io, id)? {
            return Ok(true);
        }
    }
    Ok(false)
}

fn u16_at(io: &mut dyn Io, id: u8, addr: u8) -> Result<Option<u16>, String> {
    Ok(match read(io, id, addr, 2)? {
        Got::Data(d) => Some(d[0] as u16 | (d[1] as u16) << 8),
        _ => None,
    })
}

pub struct Req {
    pub old: u8,
    pub new: u8,
    pub joint: String,
    pub who: String,
}

fn refuse(req: &Req, why: String, suggest: Option<u8>, extra: Vec<(&str, J)>) -> J {
    let mut o = vec![
        ("ok", J::Bool(false)),
        ("written", J::Bool(false)),
        ("old", J::n(req.old as f64)),
        ("new", J::n(req.new as f64)),
        ("why", J::s(&why)),
        ("suggest", suggest.map(|s| J::n(s as f64)).unwrap_or(J::Nil)),
    ];
    o.extend(extra);
    J::obj(o)
}

/// Checks that need no bus traffic: number range, the registry, the roster's view of the bus.
pub fn precheck(req: &Req, reg: &Registry, on_bus: &[u8]) -> Option<(String, Option<u8>)> {
    let mut busy: Vec<u8> = on_bus.to_vec();
    busy.push(req.new);
    let temp = reg.temporary(&busy);
    if req.new == 0 || req.new >= 254 {
        return Some((format!("ID {} cannot be used: pick 1..253", req.new), temp));
    }
    if req.new == req.old {
        return Some((format!("the servo already has ID {}", req.new), None));
    }
    if !on_bus.contains(&req.old) {
        return Some((format!("no servo answers on ID {} right now", req.old), None));
    }
    if on_bus.contains(&req.new) {
        return Some((format!("ID {} is on the bus now ({}): two servos would share it", req.new, reg.name(req.new)), temp));
    }
    if let Some(why) = reg.taken(req.new, req.old) {
        return Some((why, temp));
    }
    None
}

pub fn set_id(io: &mut dyn Io, reg: &mut Registry, on_bus: &[u8], req: &Req, now: &str) -> Result<J, String> {
    if let Some((why, temp)) = precheck(req, reg, on_bus) {
        return Ok(refuse(req, why, temp, vec![]));
    }
    let mut busy: Vec<u8> = on_bus.to_vec();
    busy.push(req.new);
    let temp = reg.temporary(&busy);
    let good = clean_reads(io, req.old)?;
    let before = format!("{good}/{CLEAN_N}");
    if good < CLEAN_N {
        return Ok(refuse(
            req,
            format!("ID {} read {before} clean: 2+ servos are on this ID (or a bad cable). Unplug all but one and try again", req.old),
            None,
            vec![("clean_before", J::s(&before))],
        ));
    }
    if answers(io, req.new)? {
        return Ok(refuse(req, format!("ID {} answers on the bus now: two servos would share it", req.new), temp, vec![("clean_before", J::s(&before))]));
    }

    write(io, req.old, REG_LOCK, &[0])?;
    write(io, req.old, REG_ID, &[req.new])?;
    write(io, req.new, REG_LOCK, &[1])?;
    let after = clean_reads(io, req.new)?;
    let old_gone = !answers(io, req.old)?;
    let reg5 = match read(io, req.new, REG_ID, 1)? {
        Got::Data(d) => Some(d[0]),
        _ => None,
    };
    let id_ok = after == CLEAN_N && old_gone && reg5 == Some(req.new);

    // bench defaults: torque caps 300 (30 %) and torque off, each read back
    let cap = [CAP as u8, (CAP >> 8) as u8];
    write(io, req.new, REG_LOCK, &[0])?;
    write(io, req.new, REG_MAX_TORQUE, &cap)?;
    write(io, req.new, REG_LOCK, &[1])?;
    write(io, req.new, REG_TORQUE_LIMIT, &cap)?;
    write(io, req.new, REG_TORQUE, &[0])?;
    let max_torque = u16_at(io, req.new, REG_MAX_TORQUE)?;
    let torque_limit = u16_at(io, req.new, REG_TORQUE_LIMIT)?;
    let torque = match read(io, req.new, REG_TORQUE, 1)? {
        Got::Data(d) => Some(d[0] != 0),
        _ => None,
    };
    let caps_ok = max_torque == Some(CAP) && torque_limit == Some(CAP) && torque == Some(false);

    let checks = J::obj(vec![
        ("clean_before", J::s(&before)),
        ("clean_after", J::s(&format!("{after}/{CLEAN_N}"))),
        ("old_gone", J::Bool(old_gone)),
        ("reg5", reg5.map(|v| J::n(v as f64)).unwrap_or(J::Nil)),
        ("max_torque_16", max_torque.map(|v| J::n(v as f64)).unwrap_or(J::Nil)),
        ("torque_limit_48", torque_limit.map(|v| J::n(v as f64)).unwrap_or(J::Nil)),
        ("torque_on", torque.map(J::Bool).unwrap_or(J::Nil)),
    ]);
    if reg5 == Some(req.new) || after > 0 {
        reg.record(now, req.old, req.new, &req.joint, &req.who, checks.clone());
        reg.save()?;
    }
    let ok = id_ok && caps_ok;
    let why = if ok {
        format!("ID {} -> {}: {after}/{CLEAN_N} clean at {}, {} gone, caps 300, torque off", req.old, req.new, req.new, req.old)
    } else if !id_ok {
        format!("written, but the read-back is not clean: {after}/{CLEAN_N} at {}, old gone {old_gone}, reg5 {reg5:?}", req.new)
    } else {
        format!("ID changed, but the caps did not read back: max torque {max_torque:?}, limit {torque_limit:?}, torque {torque:?}")
    };
    Ok(J::obj(vec![
        ("ok", J::Bool(ok)),
        ("written", J::Bool(true)),
        ("old", J::n(req.old as f64)),
        ("new", J::n(req.new as f64)),
        ("joint", J::s(&reg.name(req.new))),
        ("who", J::s(&req.who)),
        ("why", J::s(&why)),
        ("checks", checks),
        ("suggest", temp.map(|s| J::n(s as f64)).unwrap_or(J::Nil)),
    ]))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::FakeBus;
    use crate::registry::Entry;

    fn req(old: u8, new: u8) -> Req {
        Req { old, new, joint: String::new(), who: "test".into() }
    }

    fn reg12() -> Registry {
        let mut r = Registry::default();
        r.assigned.insert(12, Entry { joint: "Right hip pitch".into(), since: "2026-10-04 13:10".into(), source: "SERVO-IDS.md".into() });
        r.assigned.insert(100, Entry { joint: "Bench adapter".into(), since: "2026-10-03".into(), source: "SERVO-IDS.md".into() });
        r
    }

    #[test]
    fn writes_reads_back_caps_and_records() {
        let mut bus = FakeBus::new(&[(1, 240), (100, 2980)]);
        let mut reg = reg12();
        let r = set_id(&mut bus, &mut reg, &[1, 100], &Req { joint: "Right knee".into(), ..req(1, 13) }, "2026-10-04 15:00").unwrap();
        assert_eq!(r.get("ok"), Some(&J::Bool(true)), "{}", crate::json::dump(&r));
        let s = &bus.on(13)[0].mem;
        assert_eq!((s[5], s[16] as u16 | (s[17] as u16) << 8, s[48] as u16 | (s[49] as u16) << 8, s[40], s[55]), (13, 300, 300, 0, 1));
        assert_eq!(reg.assigned[&13].joint, "Right knee");
        assert_eq!(reg.log.len(), 1);
    }

    #[test]
    fn two_servos_on_the_old_id_are_refused_and_nothing_is_written() {
        let mut bus = FakeBus::new(&[(1, 250), (1, 700), (100, 2980)]);
        let mut reg = reg12();
        let r = set_id(&mut bus, &mut reg, &[1, 100], &req(1, 13), "t").unwrap();
        assert_eq!(r.get("written"), Some(&J::Bool(false)));
        assert!(r.get("why").unwrap().as_str().unwrap().contains("2+ servos"));
        assert_eq!(bus.servos.iter().filter(|s| s.id() == 1).count(), 2, "both still at 1");
        assert!(reg.log.is_empty());
    }

    #[test]
    fn a_registry_taken_id_is_refused_even_unplugged_with_a_temporary_offer() {
        // today's fault: 12 was given at 13:10 to a servo that is not on the bus now
        let mut bus = FakeBus::new(&[(1, 1485), (13, 240), (14, 682), (100, 2980)]);
        let mut reg = reg12();
        let r = set_id(&mut bus, &mut reg, &[1, 13, 14, 100], &req(1, 12), "t").unwrap();
        assert_eq!(r.get("written"), Some(&J::Bool(false)));
        assert!(r.get("why").unwrap().as_str().unwrap().contains("already given"));
        assert_eq!(r.get("suggest").and_then(|s| s.num()), Some(101.0));
        assert_eq!(bus.on(1).len(), 1, "nothing written");
        assert_eq!(bus.rpcs, 0, "refused before touching the bus");
    }

    #[test]
    fn an_id_on_the_bus_is_refused() {
        let mut bus = FakeBus::new(&[(1, 300), (13, 240)]);
        let mut reg = Registry::default();
        let r = set_id(&mut bus, &mut reg, &[1, 13], &req(1, 13), "t").unwrap();
        assert!(r.get("why").unwrap().as_str().unwrap().contains("on the bus now"));
        // the roster can be a moment behind: the live ping check refuses too
        let r = set_id(&mut bus, &mut reg, &[1], &req(1, 13), "t").unwrap();
        assert_eq!(r.get("written"), Some(&J::Bool(false)), "{}", crate::json::dump(&r));
        assert_eq!(bus.on(1).len(), 1);
    }

    #[test]
    fn negative_control_bad_numbers() {
        let mut bus = FakeBus::new(&[(1, 300)]);
        let mut reg = Registry::default();
        for new in [0u8, 254, 255, 1] {
            let r = set_id(&mut bus, &mut reg, &[1], &req(1, new), "t").unwrap();
            assert_eq!(r.get("written"), Some(&J::Bool(false)), "new {new}");
        }
    }
}
