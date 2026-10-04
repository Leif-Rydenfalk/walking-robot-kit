//! Feetech-style TTL bus packets: `FF FF id len inst params.. chk`. Register map as read back
//! from the HD-1910 on 2026-10-02 (STS table): 40 torque enable, 41 acceleration (x100 steps/s^2),
//! 42 goal position, 44 goal time, 46 goal speed, 56 present position, 58 speed, 60 load,
//! 62 voltage (0.1 V), 63 temperature (C), 65 status, 66 moving, 69 current (raw).

pub const BROADCAST: u8 = 0xFE;
pub const INST_PING: u8 = 0x01;
pub const INST_READ: u8 = 0x02;
pub const INST_WRITE: u8 = 0x03;
pub const INST_SYNC_WRITE: u8 = 0x83;

pub const REG_TORQUE: u8 = 40;
pub const REG_ACC: u8 = 41;
/// One telemetry read: 40..70 inclusive, so torque and goal come back with position.
pub const READ_ADDR: u8 = 40;
pub const READ_LEN: u8 = 31;

pub fn packet(id: u8, inst: u8, params: &[u8]) -> Vec<u8> {
    let mut p = Vec::with_capacity(params.len() + 6);
    p.extend_from_slice(&[0xFF, 0xFF, id, params.len() as u8 + 2, inst]);
    p.extend_from_slice(params);
    let sum: u32 = p[2..].iter().map(|&b| b as u32).sum();
    p.push(!(sum as u8));
    p
}

pub fn read(id: u8, addr: u8, n: u8) -> Vec<u8> {
    packet(id, INST_READ, &[addr, n])
}

/// One sync-write packet, no replies: `[addr, len, (id, data..)..]`.
pub fn sync_write(addr: u8, entries: &[(u8, Vec<u8>)]) -> Vec<u8> {
    let len = entries.first().map(|e| e.1.len()).unwrap_or(0) as u8;
    let mut params = vec![addr, len];
    for (id, d) in entries {
        params.push(*id);
        params.extend_from_slice(d);
    }
    packet(BROADCAST, INST_SYNC_WRITE, &params)
}

/// Data for a sync write at 41: acceleration, goal position, goal time 0, goal speed.
pub fn move_data(pos: u16, speed: u16, acc: u8) -> Vec<u8> {
    vec![acc, pos as u8, (pos >> 8) as u8, 0, 0, speed as u8, (speed >> 8) as u8]
}

#[derive(Debug, Clone, PartialEq)]
pub struct Reply {
    pub id: u8,
    pub err: u8,
    pub data: Vec<u8>,
}

/// Valid status packets in `raw`, skipping our own echo when the half-duplex adapter returns it.
pub fn replies(raw: &[u8], sent: &[u8]) -> Vec<Reply> {
    let mut out = Vec::new();
    let mut k = if raw.starts_with(sent) { sent.len() } else { 0 };
    while k + 6 <= raw.len() {
        if raw[k] == 0xFF && raw[k + 1] == 0xFF && raw[k + 2] != 0xFF {
            let ln = raw[k + 3] as usize;
            let end = k + 4 + ln;
            if ln >= 2 && end <= raw.len() {
                let sum: u32 = raw[k + 2..end - 1].iter().map(|&b| b as u32).sum();
                if !(sum as u8) == raw[end - 1] {
                    out.push(Reply { id: raw[k + 2], err: raw[k + 4], data: raw[k + 5..end - 1].to_vec() });
                    k = end;
                    continue;
                }
            }
        }
        k += 1;
    }
    out
}

/// Feetech sign convention: bit 15 is the sign, not two's complement.
pub fn s16(lo: u8, hi: u8) -> i32 {
    let v = lo as i32 | (hi as i32) << 8;
    if v & 0x8000 != 0 {
        -(v & 0x7fff)
    } else {
        v
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Telemetry {
    pub torque: bool,
    pub torque_limit: u16, // reg 48 (RAM, 0..1000 = 0..100 %)
    pub goal: u16,
    pub pos: u16,
    pub speed: i32,
    pub load: i32,
    pub volt: f32,
    pub temp: u8,
    pub status: u8,
    pub moving: u8,
    pub current: i32,
}

/// Decode a READ_ADDR/READ_LEN reply.
pub fn telemetry(d: &[u8]) -> Option<Telemetry> {
    if d.len() < READ_LEN as usize {
        return None;
    }
    let at = |reg: usize| reg - READ_ADDR as usize;
    let u = |reg: usize| d[at(reg)] as u16 | (d[at(reg) + 1] as u16) << 8;
    Some(Telemetry {
        torque: d[at(40)] != 0,
        torque_limit: u(48),
        goal: u(42),
        pos: u(56),
        speed: s16(d[at(58)], d[at(58) + 1]),
        load: s16(d[at(60)], d[at(60) + 1]),
        volt: d[at(62)] as f32 / 10.0,
        temp: d[at(63)],
        status: d[at(65)],
        moving: d[at(66)],
        current: s16(d[at(69)], d[at(69) + 1]),
    })
}

pub fn write(id: u8, addr: u8, data: &[u8]) -> Vec<u8> {
    let mut p = vec![addr];
    p.extend_from_slice(data);
    packet(id, INST_WRITE, &p)
}

pub fn ping(id: u8) -> Vec<u8> {
    packet(id, INST_PING, &[])
}

pub const REG_ID: u8 = 5;
pub const REG_LOCK: u8 = 55;
pub const REG_MIN_ANGLE: u8 = 9; // EEPROM, 2 bytes
pub const REG_MAX_ANGLE: u8 = 11; // EEPROM, 2 bytes
pub const REG_OFS: u8 = 31; // EEPROM, 2 bytes: position correction. 0 on every the robot servo (read on the
                            // evening of 2026-10-04); writing it is what moves a servo's own zero.
pub const REG_POS: u8 = 56; // RAM, 2 bytes: present position
/// Written to REG_TORQUE (40) it is not a torque state: the servo takes its present pose as count 2048 by
/// writing REG_OFS itself. Refused while the EEPROM lock (55) is 1, which is how it failed on 2026-10-04.
pub const MIDPOINT: u8 = 128;
pub const REG_MAX_TORQUE: u8 = 16; // EEPROM, 0..1000
pub const REG_TORQUE_LIMIT: u8 = 48; // RAM, 0..1000
/// The EEPROM block read once per servo in rotation: 5..17 (id .. max torque).
pub const EE_ADDR: u8 = 5;
pub const EE_LEN: u8 = 13;

/// EEPROM settings that matter on the bench: the stored ID, return delay, angle limits, the torque cap.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Eeprom {
    pub id: u8,
    pub return_delay_us: u16, // reg 7 x 2 us
    pub min_angle: u16,       // reg 9
    pub max_angle: u16,       // reg 11
    pub max_torque: u16,      // reg 16
}

pub fn eeprom(d: &[u8]) -> Option<Eeprom> {
    if d.len() < EE_LEN as usize {
        return None;
    }
    let u = |reg: usize| d[reg - 5] as u16 | (d[reg - 4] as u16) << 8;
    Some(Eeprom { id: d[0], return_delay_us: d[2] as u16 * 2, min_angle: u(9), max_angle: u(11), max_torque: u(16) })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ping_packet_matches_bus_py() {
        // bus.py: packet(0xFE, 1) == ff ff fe 02 01 fe
        assert_eq!(packet(BROADCAST, INST_PING, &[]), vec![0xff, 0xff, 0xfe, 0x02, 0x01, 0xfe]);
        // the read bus.py used for position: id 1, addr 56, 2 bytes
        assert_eq!(read(1, 56, 2), vec![0xff, 0xff, 0x01, 0x04, 0x02, 0x38, 0x02, 0xbe]);
    }

    #[test]
    fn reply_after_echo() {
        let sent = read(1, 56, 2);
        let mut raw = sent.clone();
        // status packet: id 1, len 4, err 0, data a9 05 (1449)
        let body = [0x01u8, 0x04, 0x00, 0xa9, 0x05];
        let sum: u32 = body.iter().map(|&b| b as u32).sum();
        raw.extend_from_slice(&[0xff, 0xff]);
        raw.extend_from_slice(&body);
        raw.push(!(sum as u8));
        let r = replies(&raw, &sent);
        assert_eq!(r, vec![Reply { id: 1, err: 0, data: vec![0xa9, 0x05] }]);
        // a flipped checksum is refused
        let n = raw.len();
        raw[n - 1] ^= 1;
        assert!(replies(&raw, &sent).is_empty());
    }

    #[test]
    fn sync_write_layout() {
        let p = sync_write(REG_ACC, &[(1, move_data(2473, 1200, 40)), (2, move_data(1449, 1200, 40))]);
        assert_eq!(&p[..7], &[0xff, 0xff, 0xfe, 2 + 2 + 2 * 8, INST_SYNC_WRITE, REG_ACC, 7]);
        assert_eq!(&p[7..15], &[1, 40, 0xa9, 0x09, 0, 0, 0xb0, 0x04]);
    }

    #[test]
    fn write_packet_and_eeprom_block() {
        // unlock EEPROM on id 1: ff ff 01 04 03 37 00 c0
        assert_eq!(write(1, REG_LOCK, &[0]), vec![0xff, 0xff, 0x01, 0x04, 0x03, 0x37, 0x00, 0xc0]);
        let mut d = vec![0u8; 13];
        d[0] = 23; // reg 5
        d[2] = 253; // reg 7: 506 us measured on every servo so far
        d[6] = 0xff; d[7] = 0x0f; // reg 11..12 = 4095
        d[11] = 0x2c; d[12] = 0x01; // reg 16..17 = 300
        let e = eeprom(&d).unwrap();
        assert_eq!((e.id, e.return_delay_us, e.min_angle, e.max_angle, e.max_torque), (23, 506, 0, 4095, 300));
    }

    #[test]
    fn sign_bit() {
        assert_eq!(s16(0x41, 0x00), 65);
        assert_eq!(s16(0x41, 0x80), -65);
    }
}
