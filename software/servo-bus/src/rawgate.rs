//! The raw-batch door for an outside controller (the the robot walking policy, 19: "run the neural
//! network continuously and move it 5% at 5% the torque of it running"), so the policy rides the daemon instead of
//! opening the bus as a second master. Every frame is checked before it reaches the bus:
//!   - ping (0x01), read (0x02), sync read (0x82): passed as they are;
//!   - write (0x03) and sync write (0x83) only to RAM 40..48 (torque, acc, goal, time, speed, torque limit);
//!     EEPROM, the lock (55) and anything else are refused;
//!   - a goal (42..43) is clamped to that servo's limit band; a servo without a band cannot be given a goal or torque;
//!   - a torque limit (48) above `cap` is lowered to `cap`, and every torque-on is preceded by 48 = `cap`.
//! The caller gets one reply per frame it sent (injected cap writes are hidden), plus how many goals were clamped.

use crate::feetech::{self, INST_PING, INST_READ, INST_SYNC_WRITE, INST_WRITE};

pub const INST_SYNC_READ: u8 = 0x82;
pub const RAM_LO: u8 = 40;
pub const RAM_HI: u8 = 48; // last writable byte (torque limit high byte is 49)

#[derive(Debug, PartialEq)]
pub struct Gated {
    pub frames: Vec<Vec<u8>>,
    pub expects: Vec<usize>,
    /// index into `frames` of each frame the caller sent (the others are injected cap writes)
    pub keep: Vec<usize>,
    pub clamped: Vec<(u8, u16, u16)>,
    /// servos this batch turns torque on / off (the daemon's watchdog turns the "on" ones off when the caller goes quiet)
    pub on: Vec<u8>,
    pub off: Vec<u8>,
}

/// (id, inst, params) of one well-formed Feetech packet, or why not.
pub fn parse(p: &[u8]) -> Result<(u8, u8, Vec<u8>), String> {
    if p.len() < 6 || p[0] != 0xFF || p[1] != 0xFF || p[3] as usize + 4 != p.len() {
        return Err(format!("not one Feetech packet ({} bytes)", p.len()));
    }
    let sum: u32 = p[2..p.len() - 1].iter().map(|&b| b as u32).sum();
    if !(sum as u8) != p[p.len() - 1] {
        return Err("bad checksum".into());
    }
    Ok((p[2], p[4], p[5..p.len() - 1].to_vec()))
}

/// Check one RAM write of `data` at `addr` for servo `id`; returns the data to send (goal clamped, cap applied).
fn gate_write(id: u8, addr: u8, data: &[u8], band: &dyn Fn(u8) -> Option<(u16, u16)>, cap: u16, clamped: &mut Vec<(u8, u16, u16)>) -> Result<(Vec<u8>, Option<bool>), String> {
    let end = addr as usize + data.len(); // one past the last byte written
    if addr < RAM_LO || end > RAM_HI as usize + 2 {
        return Err(format!("ID {id}: write to {addr}..{} refused (only RAM 40..49)", end - 1));
    }
    let mut d = data.to_vec();
    let len = d.len();
    let at = |reg: usize| reg.checked_sub(addr as usize).filter(|k| *k < len);
    let mut torque = None;
    if let Some(k) = at(40) {
        if d[k] != 0 && band(id).is_none() {
            return Err(format!("ID {id} has no angle limits: torque on refused"));
        }
        torque = Some(d[k] != 0);
    }
    if let (Some(k), Some(k2)) = (at(42), at(43)) {
        let g = d[k] as u16 | (d[k2] as u16) << 8;
        let (lo, hi) = band(id).ok_or_else(|| format!("ID {id} has no angle limits: goal {g} refused"))?;
        let c = g.clamp(lo, hi);
        if c != g {
            clamped.push((id, g, c));
            d[k] = c as u8;
            d[k2] = (c >> 8) as u8;
        }
    } else if at(42).is_some() || at(43).is_some() {
        return Err(format!("ID {id}: half a goal register written"));
    }
    if let (Some(k), Some(k2)) = (at(48), at(49)) {
        let v = (d[k] as u16 | (d[k2] as u16) << 8).min(cap);
        d[k] = v as u8;
        d[k2] = (v >> 8) as u8;
    } else if at(48).is_some() || at(49).is_some() {
        return Err(format!("ID {id}: half a torque-limit register written"));
    }
    Ok((d, torque))
}

fn cap_write(ids: &[u8], cap: u16) -> Vec<u8> {
    let entries: Vec<(u8, Vec<u8>)> = ids.iter().map(|&i| (i, vec![cap as u8, (cap >> 8) as u8])).collect();
    feetech::sync_write(feetech::REG_TORQUE_LIMIT, &entries)
}

pub fn gate(frames: &[Vec<u8>], expects: &[usize], band: &dyn Fn(u8) -> Option<(u16, u16)>, cap: u16) -> Result<Gated, String> {
    let mut g = Gated { frames: vec![], expects: vec![], keep: vec![], clamped: vec![], on: vec![], off: vec![] };
    for (k, p) in frames.iter().enumerate() {
        let (id, inst, params) = parse(p)?;
        let exp = expects.get(k).copied().unwrap_or(0);
        match inst {
            INST_PING | INST_READ | INST_SYNC_READ => {
                g.keep.push(g.frames.len());
                g.frames.push(p.clone());
                g.expects.push(exp);
            }
            INST_WRITE => {
                let (addr, data) = params.split_first().ok_or("empty write")?;
                let (d, tq) = gate_write(id, *addr, data, band, cap, &mut g.clamped)?;
                if tq == Some(true) {
                    g.frames.push(cap_write(&[id], cap));
                    g.expects.push(0);
                    g.on.push(id);
                } else if tq == Some(false) {
                    g.off.push(id);
                }
                let mut pp = vec![*addr];
                pp.extend_from_slice(&d);
                g.keep.push(g.frames.len());
                g.frames.push(feetech::packet(id, INST_WRITE, &pp));
                g.expects.push(exp);
            }
            INST_SYNC_WRITE => {
                if params.len() < 2 || params[1] == 0 || (params.len() - 2) % (params[1] as usize + 1) != 0 {
                    return Err("malformed sync write".into());
                }
                let (addr, n) = (params[0], params[1] as usize);
                let mut entries = Vec::new();
                let mut on_ids = Vec::new();
                for e in params[2..].chunks(n + 1) {
                    let (d, tq) = gate_write(e[0], addr, &e[1..], band, cap, &mut g.clamped)?;
                    if tq == Some(true) {
                        on_ids.push(e[0]);
                    } else if tq == Some(false) {
                        g.off.push(e[0]);
                    }
                    entries.push((e[0], d));
                }
                if !on_ids.is_empty() {
                    g.frames.push(cap_write(&on_ids, cap));
                    g.expects.push(0);
                    g.on.extend(&on_ids);
                }
                g.keep.push(g.frames.len());
                g.frames.push(feetech::sync_write(addr, &entries));
                g.expects.push(exp);
            }
            _ => return Err(format!("instruction 0x{inst:02x} refused")),
        }
    }
    Ok(g)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn band(id: u8) -> Option<(u16, u16)> {
        (id == 22).then_some((781, 1737))
    }

    #[test]
    fn reads_pass_goals_clamp_eeprom_refused() {
        let r = feetech::read(22, 56, 2);
        let g = gate(&[r.clone()], &[8], &band, 50).unwrap();
        assert_eq!(g.frames, vec![r]);
        // goal 3000 for 22 is past 1737: clamped
        let sw = feetech::sync_write(42, &[(22, vec![3000u16 as u8, (3000u16 >> 8) as u8])]);
        let g = gate(&[sw], &[0], &band, 50).unwrap();
        assert_eq!(g.clamped, vec![(22, 3000, 1737)]);
        let (_, _, p) = parse(&g.frames[0]).unwrap();
        assert_eq!(p[3] as u16 | (p[4] as u16) << 8, 1737);
        // a servo without limits gets no goal and no torque
        assert!(gate(&[feetech::sync_write(42, &[(23, vec![0, 4])])], &[0], &band, 50).is_err());
        assert!(gate(&[feetech::write(23, 40, &[1])], &[6], &band, 50).is_err());
        // EEPROM (angle limits, lock) is never written through this door
        assert!(gate(&[feetech::write(22, 9, &[0, 0])], &[6], &band, 50).is_err());
        assert!(gate(&[feetech::write(22, 55, &[0])], &[6], &band, 50).is_err());
        // torque off always passes
        assert!(gate(&[feetech::write(23, 40, &[0])], &[6], &band, 50).is_ok());
    }

    #[test]
    fn torque_on_brings_the_cap_and_a_higher_limit_is_lowered() {
        let g = gate(&[feetech::sync_write(40, &[(22, vec![1])])], &[0], &band, 50).unwrap();
        assert_eq!(g.frames.len(), 2);
        assert_eq!(g.keep, vec![1], "the caller sees one reply, the injected cap write is hidden");
        assert_eq!((g.on.clone(), g.off.clone()), (vec![22], vec![]));
        assert_eq!(gate(&[feetech::write(22, 40, &[0])], &[6], &band, 50).unwrap().off, vec![22]);
        let (_, _, p) = parse(&g.frames[0]).unwrap();
        assert_eq!((p[0], p[3] as u16 | (p[4] as u16) << 8), (48, 50));
        let g = gate(&[feetech::write(22, 48, &[0xe8, 0x03])], &[6], &band, 50).unwrap();
        let (_, _, p) = parse(&g.frames[0]).unwrap();
        assert_eq!(p[1] as u16 | (p[2] as u16) << 8, 50);
        // the runner's 7-byte block 41..47 (acc, goal, time, speed) is clamped in place
        let blk = feetech::sync_write(41, &[(22, vec![0, 0x10, 0x00, 0, 0, 0x58, 0x02])]);
        let g = gate(&[blk], &[0], &band, 50).unwrap();
        let (_, _, p) = parse(&g.frames[0]).unwrap();
        assert_eq!(p[4] as u16 | (p[5] as u16) << 8, 781);
    }
}
