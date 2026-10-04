//! HTTP + WebSocket, hand-rolled because the crate has zero dependencies by design (the
//! latency path is this code, the router socket, and the bus — nothing else). Routes:
//!
//!   GET  /             the bus page for customers (web/bus.html): live roster, duplicate banner, set ID
//!   GET  /engineer     the original control page (web/index.html): wave and sliders on the --ids set
//!   GET  /api/state    full state snapshot, chart window included
//!   GET  /api/bus      the roster document the bus page draws
//!   GET  /api/registry the ID registry (who has which ID, every change)
//!   POST /api/cmd      one command, same JSON the WebSocket takes (REST parity for scripts)
//!   GET  /api/imu          one MPU-6050 sample (raw LSB + the scales); /api/imu/scan lists I2C addresses
//!   POST /api/recentre {"id":13,"who":"we"}: move that servo's own zero (EEPROM 31) to where it stands now
//!   POST /api/setid    {"old":1,"new":13,"joint":"Right knee","who":"we"}: runs to completion, answers the report
//!   GET  /api/policy   the walking policy process (Walk tab): running, amp, cap, log tail
//!   POST /api/policy   {"op":"start","amp":0.05,"cap":50,"who":"we"} or {"op":"stop"} (src/policy.rs)
//!   GET  /ws           live: state ticks at ~50 Hz, commands in, chart window every 200 ms
//!   GET  /ws/bus       live: the roster document at ~20 Hz

use crate::bus::{Cmd, Shared};
use crate::json::{self, J};
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};
use std::time::Duration;

pub const PAGE: &str = include_str!("../web/index.html");
pub const BUS_PAGE: &str = include_str!("../web/bus.html");
const WS_GUID: &str = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

pub fn serve(listener: TcpListener, sh: Arc<Mutex<Shared>>, tx: Sender<Cmd>) -> ! {
    for stream in listener.incoming() {
        let Ok(stream) = stream else { continue };
        let sh = sh.clone();
        let tx = tx.clone();
        std::thread::spawn(move || {
            let _ = conn(stream, sh, tx);
        });
    }
    unreachable!("listener.incoming() never ends")
}

fn conn(mut s: TcpStream, sh: Arc<Mutex<Shared>>, tx: Sender<Cmd>) -> std::io::Result<()> {
    s.set_nodelay(true).ok();
    let req = read_request(&mut s)?;
    let path = req.0.clone();
    if path == "/ws" || path == "/ws/bus" {
        let key = req.1.iter().find(|(k, _)| k.eq_ignore_ascii_case("sec-websocket-key")).map(|(_, v)| v.clone());
        return ws(s, sh, tx, key, path == "/ws/bus");
    }
    let header = |name: &str| req.1.iter().find(|(k, _)| k.eq_ignore_ascii_case(name)).map(|(_, v)| v.clone());
    match (req.2.as_str(), path.as_str()) {
        ("GET", "/") | ("GET", "/index.html") => respond(&mut s, 200, BUS_PAGE, "text/html; charset=utf-8"),
        ("GET", "/engineer") => respond(&mut s, 200, PAGE, "text/html; charset=utf-8"),
        // The probe door. `expect_body = "servo-bus"` is a marker only this app emits, and
        // `/api/no-such-route-xyz` really does 404 here (the `_` arm below), so a green probe means
        // this app answered AS ITSELF rather than "something holds the port".
        // It is not a ping: `bus_ok` and `hz` are read off the live engine, so a daemon whose bus is
        // dead answers 200 with `"ok": false` and the probe can go red on a process that is running.
        ("GET", "/api/health") => {
            let body = sh
                .lock()
                .map(|v| {
                    let b = v.bus_json();
                    let pick = |k: &str| b.get("health").and_then(|h| h.get(k)).cloned().unwrap_or(json::J::Nil);
                    let rows = match b.get("rows") {
                        Some(json::J::Arr(a)) => a.len(),
                        _ => 0,
                    };
                    json::dump(&json::J::obj(vec![
                        ("app", json::J::s("servo-bus")),
                        ("what", json::J::s("owns the the robot servo bus on the UNO Q")),
                        ("ok", pick("ok")),
                        ("hz", pick("hz")),
                        ("stalls", pick("stalls")),
                        ("recovers", pick("recovers")),
                        ("soft_recovers", pick("soft_recovers")),
                        ("timeouts", pick("timeouts")),
                        ("late", pick("late")),
                        ("tx_stuck", pick("tx_stuck")),
                        ("tx_stuck_pre", pick("tx_stuck_pre")),
                        ("lock_lost", pick("lock_lost")),
                        ("lock_lost_pre", pick("lock_lost_pre")),
                        ("up_s", pick("up_s")),
                        ("servos", json::J::n(rows as f64)),
                        ("note", pick("note")),
                    ]))
                })
                .unwrap_or_else(|_| "{}".into());
            respond(&mut s, 200, &body, "application/json")
        }
        ("GET", "/api/bus") => {
            let body = sh.lock().map(|v| json::dump(&v.bus_json())).unwrap_or_else(|_| "{}".into());
            respond(&mut s, 200, &body, "application/json")
        }
        ("GET", "/api/registry") => {
            let body = sh.lock().map(|v| json::dump(&v.registry.to_json())).unwrap_or_else(|_| "{}".into());
            respond(&mut s, 200, &body, "application/json")
        }
        ("POST", "/api/setid") => {
            let v = json::parse(&req.3);
            let num = |k: &str| v.as_ref().and_then(|v| v.get(k)).and_then(|x| x.i64()).filter(|n| (0..=255).contains(n)).map(|n| n as u8);
            let text = |k: &str| v.as_ref().and_then(|v| v.get(k)).and_then(|x| x.as_str()).unwrap_or("").chars().take(60).collect::<String>();
            let (Some(old), Some(new)) = (num("old"), num("new")) else {
                return respond(&mut s, 400, "{\"error\":\"old and new IDs (0..255) required\"}", "application/json");
            };
            // the gate names a signed-in person; a page user types a name; else the page
            let who = header("x-ce-person").filter(|p| !p.is_empty()).or_else(|| Some(text("who")).filter(|w| !w.is_empty())).unwrap_or_else(|| "bus page".into());
            let (rtx, rrx) = std::sync::mpsc::channel();
            tx.send(Cmd::SetId { old, new, joint: text("joint"), who, reply: Some(rtx) }).ok();
            match rrx.recv_timeout(Duration::from_secs(60)) {
                Ok(r) => respond(&mut s, 200, &json::dump(&r), "application/json"),
                Err(_) => respond(&mut s, 504, "{\"error\":\"set-ID did not finish in 60 s\"}", "application/json"),
            }
        }
        ("GET", "/api/limits") => {
            let body = sh.lock().map(|v| {
                let mut o = v.limits.to_json();
                if let J::Obj(ref mut a) = o {
                    a.push(("seen".into(), J::Obj(v.limits.seen.iter().map(|(id, sn)| (id.to_string(), crate::limits::Limits::seen_json(sn))).collect())));
                }
                json::dump(&o)
            }).unwrap_or_else(|_| "{}".into());
            respond(&mut s, 200, &body, "application/json")
        }
        // {id} starts that servo's range again; {id, who, min?, max?} saves (ends given = from another recorder)
        ("POST", "/api/limits/record") | ("POST", "/api/limits/save") => {
            let v = json::parse(&req.3);
            let num = |k: &str| v.as_ref().and_then(|v| v.get(k)).and_then(|x| x.i64());
            let text = |k: &str| v.as_ref().and_then(|v| v.get(k)).and_then(|x| x.as_str()).unwrap_or("").chars().take(80).collect::<String>();
            let Some(id) = num("id").filter(|n| (0..=253).contains(n)).map(|n| n as u8) else {
                return respond(&mut s, 400, "{\"error\":\"id (0..253) required\"}", "application/json");
            };
            if path.ends_with("record") {
                tx.send(Cmd::RecordLimits { id }).ok();
                return respond(&mut s, 200, "{\"ok\":true}", "application/json");
            }
            let ends = match (num("min"), num("max")) {
                (Some(a), Some(b)) if (0..=4095).contains(&a) && (0..=4095).contains(&b) => Some((a as u16, b as u16)),
                (None, None) => None,
                _ => return respond(&mut s, 400, "{\"error\":\"min and max must both be 0..4095 counts\"}", "application/json"),
            };
            let who = header("x-ce-person").filter(|p| !p.is_empty()).or_else(|| Some(text("who")).filter(|w| !w.is_empty())).unwrap_or_else(|| "bus page".into());
            let source = Some(text("source")).filter(|w| !w.is_empty()).unwrap_or_else(|| if ends.is_some() { "ends given in the request".into() } else { "recorded by the daemon (torque-off reads)".into() });
            let (rtx, rrx) = std::sync::mpsc::channel();
            tx.send(Cmd::SaveLimits { id, ends, who, source, reply: Some(rtx) }).ok();
            match rrx.recv_timeout(Duration::from_secs(20)) {
                Ok(r) => respond(&mut s, 200, &json::dump(&r), "application/json"),
                Err(_) => respond(&mut s, 504, "{\"error\":\"limits save did not finish in 20 s\"}", "application/json"),
            }
        }
        // which way is up: /api/imu reads one MPU-6050 sample, /api/imu/scan lists what ACKs on I2C.
        // Read-only, and it moves nothing, so a person can tilt the robot by hand and watch the numbers.
        ("GET", "/api/imu") | ("GET", "/api/imu/scan") | ("POST", "/api/imu") => {
            let (rtx, rrx) = std::sync::mpsc::channel();
            tx.send(Cmd::Imu { scan: path.ends_with("/scan"), reply: rtx }).ok();
            match rrx.recv_timeout(Duration::from_secs(5)) {
                Ok(r) => respond(&mut s, 200, &json::dump(&r), "application/json"),
                Err(_) => respond(&mut s, 504, "{\"ok\":false,\"why\":\"the IMU read did not finish in 5 s\"}", "application/json"),
            }
        }
        // {id, who}: move THAT servo's own zero to where it is standing now (EEPROM reg 31). The one write the
        // raw door cannot do, because reg 55 is outside it on purpose. Refuses on torque on, on a slider hold, on
        // a servo that is moving, and on a read-back that is not 2048. One ID per call, deliberately.
        ("POST", "/api/recentre") => {
            let v = json::parse(&req.3);
            let num = |k: &str| v.as_ref().and_then(|v| v.get(k)).and_then(|x| x.i64());
            let text = |k: &str| v.as_ref().and_then(|v| v.get(k)).and_then(|x| x.as_str()).unwrap_or("").chars().take(80).collect::<String>();
            let Some(id) = num("id").filter(|n| (0..=253).contains(n)).map(|n| n as u8) else {
                return respond(&mut s, 400, "{\"error\":\"id (0..253) required\"}", "application/json");
            };
            let who = header("x-ce-person").filter(|p| !p.is_empty()).or_else(|| Some(text("who")).filter(|w| !w.is_empty())).unwrap_or_else(|| "bus page".into());
            let (rtx, rrx) = std::sync::mpsc::channel();
            tx.send(Cmd::Recentre { id, who, reply: Some(rtx) }).ok();
            match rrx.recv_timeout(Duration::from_secs(20)) {
                Ok(r) => respond(&mut s, 200, &json::dump(&r), "application/json"),
                Err(_) => respond(&mut s, 504, "{\"error\":\"recentre did not finish in 20 s\"}", "application/json"),
            }
        }
        // an outside controller (the walking policy) sends raw Feetech frames through the daemon's gate:
        // {frames: [hex], expects: [n], wait_us, cap, who} -> {ok, replies: [hex], clamped, ms} (src/rawgate.rs)
        ("POST", "/api/batch") => {
            let v = json::parse(&req.3);
            let unhex = |h: &str| -> Option<Vec<u8>> { (0..h.len()).step_by(2).map(|i| h.get(i..i + 2).and_then(|x| u8::from_str_radix(x, 16).ok())).collect() };
            let frames: Option<Vec<Vec<u8>>> = match v.as_ref().and_then(|v| v.get("frames")) {
                Some(J::Arr(a)) => a.iter().map(|x| x.as_str().and_then(unhex)).collect(),
                _ => None,
            };
            let Some(frames) = frames.filter(|f| !f.is_empty() && f.len() <= 32) else {
                return respond(&mut s, 400, "{\"error\":\"frames: 1..32 hex Feetech packets\"}", "application/json");
            };
            let expects: Vec<usize> = match v.as_ref().and_then(|v| v.get("expects")) {
                Some(J::Arr(a)) => a.iter().map(|x| x.i64().unwrap_or(0).clamp(0, 250) as usize).collect(),
                _ => vec![0; frames.len()],
            };
            let num = |k: &str, d: i64| v.as_ref().and_then(|v| v.get(k)).and_then(|x| x.i64()).unwrap_or(d);
            let who = v.as_ref().and_then(|v| v.get("who")).and_then(|x| x.as_str()).unwrap_or("raw client").chars().take(40).collect::<String>();
            let (rtx, rrx) = std::sync::mpsc::channel();
            tx.send(Cmd::Raw { frames, expects, wait_us: num("wait_us", 3000), cap: num("cap", 0).clamp(0, 1000) as u16, who, reply: rtx }).ok();
            match rrx.recv_timeout(Duration::from_secs(2)) {
                Ok(r) => respond(&mut s, 200, &json::dump(&r), "application/json"),
                Err(_) => respond(&mut s, 504, "{\"ok\":false,\"why\":\"daemon did not answer in 2 s\"}", "application/json"),
            }
        }
        ("GET", "/api/policy") => {
            let body = crate::policy::POLICY.lock().map(|mut p| json::dump(&p.to_json())).unwrap_or_else(|_| "{}".into());
            respond(&mut s, 200, &body, "application/json")
        }
        ("POST", "/api/policy") => {
            let v = json::parse(&req.3);
            let op = v.as_ref().and_then(|v| v.get("op")).and_then(|x| x.as_str()).unwrap_or("").to_string();
            let f = |k: &str| v.as_ref().and_then(|v| v.get(k)).and_then(|x| x.num());
            let who = header("x-ce-person").filter(|p| !p.is_empty())
                .or_else(|| v.as_ref().and_then(|v| v.get("who")).and_then(|x| x.as_str()).map(|w| w.chars().take(40).collect()).filter(|w: &String| !w.is_empty()))
                .unwrap_or_else(|| "bus page".into());
            let mut p = crate::policy::POLICY.lock().unwrap();
            let r = match op.as_str() {
                "start" => p.start(f("amp").unwrap_or(0.05), f("cap").unwrap_or(50.0) as i64, &who),
                "stop" => {
                    p.stop();
                    Ok(())
                }
                _ => Err("op must be start or stop".into()),
            };
            let mut o = p.to_json();
            drop(p);
            if let J::Obj(ref mut a) = o {
                a.insert(0, ("ok".into(), J::Bool(r.is_ok())));
                if let Err(e) = &r {
                    a.insert(1, ("why".into(), J::s(e)));
                }
            }
            if r.is_ok() {
                let t = crate::bus::now_unix();
                if let Ok(mut sh) = sh.lock() {
                    sh.roster.event(t, None, "policy", format!("{who}: policy {op} (amp {}, torque cap {})", f("amp").unwrap_or(0.05), f("cap").unwrap_or(50.0)));
                }
            }
            respond(&mut s, 200, &json::dump(&o), "application/json")
        }
        ("GET", "/api/state") => {
            let body = sh.lock().map(|v| json::dump(&v.to_json(240, false))).unwrap_or_else(|_| "{}".into());
            respond(&mut s, 200, &body, "application/json")
        }
        ("POST", "/api/cmd") => {
            let cmd = json::parse(&req.3).and_then(|v| to_cmd(&v));
            match cmd {
                Some(c) => {
                    tx.send(c).ok();
                    respond(&mut s, 200, "{\"ok\":true}", "application/json")
                }
                None => respond(&mut s, 400, "{\"error\":\"bad command\"}", "application/json"),
            }
        }
        _ => respond(&mut s, 404, "no such path", "text/plain"),
    }
}

/// (path, headers, method, body)
type Req = (String, Vec<(String, String)>, String, String);

fn read_request(s: &mut TcpStream) -> std::io::Result<Req> {
    s.set_read_timeout(Some(Duration::from_secs(10))).ok();
    let mut buf = Vec::with_capacity(1024);
    let mut chunk = [0u8; 2048];
    let head_end = loop {
        let n = s.read(&mut chunk)?;
        if n == 0 {
            return Err(std::io::Error::new(std::io::ErrorKind::UnexpectedEof, "closed"));
        }
        buf.extend_from_slice(&chunk[..n]);
        if let Some(p) = find(&buf, b"\r\n\r\n") {
            break p + 4;
        }
        if buf.len() > 32 * 1024 {
            return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, "header too big"));
        }
    };
    let head = String::from_utf8_lossy(&buf[..head_end]).into_owned();
    let mut lines = head.split("\r\n");
    let first = lines.next().unwrap_or("");
    let mut parts = first.split_whitespace();
    let method = parts.next().unwrap_or("").to_string();
    let path = parts.next().unwrap_or("/").split('?').next().unwrap_or("/").to_string();
    let mut headers = Vec::new();
    for l in lines {
        if let Some((k, v)) = l.split_once(':') {
            headers.push((k.trim().to_string(), v.trim().to_string()));
        }
    }
    let clen = headers
        .iter()
        .find(|(k, _)| k.eq_ignore_ascii_case("content-length"))
        .and_then(|(_, v)| v.parse::<usize>().ok())
        .unwrap_or(0)
        .min(64 * 1024);
    let mut body = buf[head_end..].to_vec();
    while body.len() < clen {
        let n = s.read(&mut chunk)?;
        if n == 0 {
            break;
        }
        body.extend_from_slice(&chunk[..n]);
    }
    body.truncate(clen);
    Ok((path, headers, method, String::from_utf8_lossy(&body).into_owned()))
}

fn find(hay: &[u8], needle: &[u8]) -> Option<usize> {
    hay.windows(needle.len()).position(|w| w == needle)
}

fn respond(s: &mut TcpStream, code: u16, body: &str, ctype: &str) -> std::io::Result<()> {
    let head = format!("HTTP/1.1 {code} OK\r\nContent-Type: {ctype}\r\nContent-Length: {}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n", body.len());
    s.write_all(head.as_bytes())?;
    s.write_all(body.as_bytes())
}

fn to_cmd(v: &J) -> Option<Cmd> {
    let op = v.get("op")?.as_str()?;
    Some(match op {
        "move" => Cmd::Move { id: v.get("id")?.i64()? as u8, deg: v.get("deg")?.num()? },
        "torque" => Cmd::Torque { id: v.get("id").and_then(|x| x.i64()).map(|i| i as u8), on: v.get("on")?.boolean()? },
        "wave" => Cmd::Wave {
            amp_deg: v.get("amp").and_then(|x| x.num()).unwrap_or(45.0),
            period: v.get("period").and_then(|x| x.num()).unwrap_or(3.0),
        },
        "hold" => Cmd::Hold { id: v.get("id")?.i64()? as u8, on: v.get("on")?.boolean()? },
        "goal" => Cmd::Goal { id: v.get("id")?.i64()? as u8, pos: v.get("pos")?.num()?.round().clamp(0.0, 4095.0) as u16 },
        "stop" => Cmd::StopWave,
        "stopall" => Cmd::StopAll,
        _ => return None,
    })
}

// --- WebSocket ---

fn ws(mut s: TcpStream, sh: Arc<Mutex<Shared>>, tx: Sender<Cmd>, key: Option<String>, bus_only: bool) -> std::io::Result<()> {
    let Some(key) = key else { return respond(&mut s, 400, "missing key", "text/plain") };
    let accept = b64(&sha1(format!("{key}{WS_GUID}").as_bytes()));
    let head = "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: ".to_string() + &accept + "\r\n\r\n";
    s.write_all(head.as_bytes())?;
    s.set_read_timeout(Some(Duration::from_millis(20))).ok();
    let mut frag: Vec<u8> = Vec::new();
    let mut tick: u64 = 0;
    loop {
        // one client frame per turn (commands are small and rare); then a state push
        match read_frame(&mut s, &mut frag)? {
            Frame::Text(txt) => {
                if let Some(c) = json::parse(&txt).and_then(|v| to_cmd(&v)) {
                    tx.send(c).ok();
                }
            }
            Frame::Ping(p) => write_frame(&mut s, 0x8A, &p)?,
            Frame::Close => {
                write_frame(&mut s, 0x88, &[]).ok();
                return Ok(());
            }
            Frame::None => {}
        }
        tick += 1;
        let light = tick % 10 != 0;
        let body = {
            let guard = sh.lock().map_err(|_| broken())?;
            json::dump(&if bus_only { guard.bus_json() } else { guard.to_json(240, light) })
        };
        write_frame(&mut s, 0x81, body.as_bytes())?;
    }
}

fn broken() -> std::io::Error {
    std::io::Error::new(std::io::ErrorKind::ConnectionAborted, "ws closed")
}

enum Frame {
    Text(String),
    Ping(Vec<u8>),
    Close,
    None,
}

fn read_frame(s: &mut TcpStream, frag: &mut Vec<u8>) -> std::io::Result<Frame> {
    let mut hdr = [0u8; 2];
    match s.read_exact(&mut hdr) {
        Ok(()) => {}
        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock || e.kind() == std::io::ErrorKind::TimedOut => return Ok(Frame::None),
        Err(e) => return Err(e),
    }
    let opcode = hdr[0] & 0x0f;
    let fin = hdr[0] & 0x80 != 0;
    let masked = hdr[1] & 0x80 != 0;
    let mut len = (hdr[1] & 0x7f) as usize;
    if len == 126 {
        let mut b = [0u8; 2];
        s.read_exact(&mut b)?;
        len = u16::from_be_bytes(b) as usize;
    } else if len == 127 {
        let mut b = [0u8; 8];
        s.read_exact(&mut b)?;
        len = u64::from_be_bytes(b) as usize;
    }
    if len > 1 << 20 {
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, "frame too big"));
    }
    let mut mask = [0u8; 4];
    if masked {
        s.read_exact(&mut mask)?;
    }
    let mut payload = vec![0u8; len];
    s.read_exact(&mut payload)?; // reads may take several turns of the 20 ms timeout; that is fine
    if masked {
        for (i, b) in payload.iter_mut().enumerate() {
            *b ^= mask[i % 4];
        }
    }
    match opcode {
        0x8 => Ok(Frame::Close),
        0x9 => Ok(Frame::Ping(payload)),
        0x1 | 0x2 => {
            frag.extend_from_slice(&payload);
            if fin {
                let full = std::mem::take(frag);
                Ok(Frame::Text(String::from_utf8_lossy(&full).into_owned()))
            } else {
                Ok(Frame::None) // continuation frames extend `frag`
            }
        }
        _ => Ok(Frame::None),
    }
}

fn write_frame(s: &mut TcpStream, opcode: u8, payload: &[u8]) -> std::io::Result<()> {
    let mut out = Vec::with_capacity(payload.len() + 10);
    out.push(0x80 | opcode);
    let n = payload.len();
    if n < 126 {
        out.push(n as u8);
    } else if n < 65536 {
        out.push(126);
        out.extend_from_slice(&(n as u16).to_be_bytes());
    } else {
        out.push(127);
        out.extend_from_slice(&(n as u64).to_be_bytes());
    }
    out.extend_from_slice(payload);
    s.write_all(&out)
}

// --- SHA-1 and base64, only for the one-time handshake accept key ---

fn sha1(data: &[u8]) -> [u8; 20] {
    let mut h: [u32; 5] = [0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0];
    let ml = (data.len() as u64) * 8;
    let mut msg = data.to_vec();
    msg.push(0x80);
    while msg.len() % 64 != 56 {
        msg.push(0);
    }
    msg.extend_from_slice(&ml.to_be_bytes());
    for block in msg.chunks(64) {
        let mut w = [0u32; 80];
        for (i, c) in block.chunks(4).enumerate() {
            w[i] = u32::from_be_bytes([c[0], c[1], c[2], c[3]]);
        }
        for i in 16..80 {
            w[i] = (w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16]).rotate_left(1);
        }
        let (mut a, mut b, mut c, mut d, mut e) = (h[0], h[1], h[2], h[3], h[4]);
        for (i, &wi) in w.iter().enumerate() {
            let (f, k) = match i {
                0..=19 => ((b & c) | (!b & d), 0x5A827999u32),
                20..=39 => (b ^ c ^ d, 0x6ED9EBA1),
                40..=59 => ((b & c) | (b & d) | (c & d), 0x8F1BBCDC),
                _ => (b ^ c ^ d, 0xCA62C1D6),
            };
            let tmp = a.rotate_left(5).wrapping_add(f).wrapping_add(e).wrapping_add(k).wrapping_add(wi);
            e = d;
            d = c;
            c = b.rotate_left(30);
            b = a;
            a = tmp;
        }
        h[0] = h[0].wrapping_add(a);
        h[1] = h[1].wrapping_add(b);
        h[2] = h[2].wrapping_add(c);
        h[3] = h[3].wrapping_add(d);
        h[4] = h[4].wrapping_add(e);
    }
    let mut out = [0u8; 20];
    for (i, v) in h.iter().enumerate() {
        out[i * 4..i * 4 + 4].copy_from_slice(&v.to_be_bytes());
    }
    out
}

const B64: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

fn b64(data: &[u8]) -> String {
    let mut out = String::with_capacity((data.len() + 2) / 3 * 4);
    for c in data.chunks(3) {
        let b = [c[0], *c.get(1).unwrap_or(&0), *c.get(2).unwrap_or(&0)];
        let n = (u32::from(b[0]) << 16) | (u32::from(b[1]) << 8) | u32::from(b[2]);
        out.push(B64[(n >> 18) as usize & 63] as char);
        out.push(B64[(n >> 12) as usize & 63] as char);
        out.push(if c.len() > 1 { B64[(n >> 6) as usize & 63] as char } else { '=' });
        out.push(if c.len() > 2 { B64[n as usize & 63] as char } else { '=' });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sha1_known_vector() {
        // RFC 3174 / well-known: sha1("abc")
        assert_eq!(
            sha1(b"abc").iter().map(|b| format!("{b:02x}")).collect::<String>(),
            "a9993e364706816aba3e25717850c26c9cd0d89d"
        );
    }

    #[test]
    fn b64_known_vector() {
        assert_eq!(b64(b"foobar"), "Zm9vYmFy");
        assert_eq!(b64(b"foob"), "Zm9vYg==");
    }

    #[test]
    fn cmd_parsing() {
        assert!(matches!(
            to_cmd(&json::parse(r#"{"op":"move","id":3,"deg":12.5}"#).unwrap()),
            Some(Cmd::Move { id: 3, deg }) if (deg - 12.5).abs() < 1e-9
        ));
        assert!(to_cmd(&json::parse(r#"{"op":"nope"}"#).unwrap()).is_none());
    }
}
