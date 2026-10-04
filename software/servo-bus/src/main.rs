//! servo-bus: the Rust daemon that owns the servo bus, running on the Uno Q itself.
//!
//! It speaks the RouterBridge msgpack protocol directly on /var/run/arduino-router.sock, drives
//! the MCU app servo-bus-bridge over the bus_batch fast path, and serves the live control page
//! on :8938 with WebSocket state at about 50 Hz. From a computer over USB, reach it with
//! `adb forward tcp:8938 tcp:8938`.
//!
//! Build (Mac, cross): cargo zigbuild --release --target aarch64-unknown-linux-gnu
//! Run (board):        ./servo-bus --port 8938 --ids 1,2,3

mod bridge;
mod bus;
mod feetech;
mod http;
mod io;
mod json;
mod limits;
mod msgpack;
mod policy;
mod rawgate;
mod registry;
mod roster;
mod setid;

use std::sync::mpsc;
use std::sync::{Arc, Mutex};

fn arg(name: &str, default: &str) -> String {
    let a: Vec<String> = std::env::args().collect();
    a.iter().position(|x| x == name).and_then(|i| a.get(i + 1).cloned()).unwrap_or_else(|| default.to_string())
}

fn flag(name: &str) -> bool {
    std::env::args().any(|x| x == name)
}

fn main() {
    let port: u16 = arg("--port", "8938").parse().unwrap_or(8938);
    let sock = arg("--sock", bridge::DEFAULT_SOCK);
    // motion set for the engineer view's wave and sliders; empty = no motion at all (the customer default)
    let ids: Vec<u8> = arg("--ids", "").split(',').filter_map(|x| x.trim().parse().ok()).collect();
    let reg_path = arg("--registry", "servo-ids.json");
    let app = arg("--mcu-app", "user:servo-bus-bridge");
    let tz: i64 = arg("--tz", "8").parse().unwrap_or(8);
    let fake = flag("--fake");
    if flag("--probe") {
        // one ping chunk per call, timed: the bus_batch path alone, no roster, no page
        let n: usize = arg("--probe", "100").parse().unwrap_or(100);
        let tmo: u64 = arg("--timeout-ms", "1000").parse().unwrap_or(1000);
        let mut io = io::RouterIo { bridge: bridge::Bridge::new(&sock), app };
        println!("begin {:?}", io::Io::begin(&mut io));
        let t0 = std::time::Instant::now();
        for k in 0..n {
            let base = ((k * 8) % 256).min(246) as u8;
            let frames: Vec<Vec<u8>> = (base..base + 8).map(feetech::ping).collect();
            let ex: Vec<usize> = frames.iter().map(|f| f.len() + 6).collect();
            let t = std::time::Instant::now();
            let r = io::Io::batch(&mut io, &frames, &ex, 1000, std::time::Duration::from_millis(tmo));
            let ms = t.elapsed().as_secs_f64() * 1000.0;
            if r.is_err() || k < 5 || ms > 60.0 {
                println!("call {k} ids {base}.. {:.1} ms {:?}", ms, r.as_ref().map(|v| v.iter().map(|x| x.len()).collect::<Vec<_>>()));
            }
            if r.is_err() {
                std::process::exit(1);
            }
        }
        println!("ok {n} calls in {:.1} s", t0.elapsed().as_secs_f64());
        return;
    }
    let registry = registry::Registry::load(&reg_path);
    let (tx, rx) = mpsc::channel::<bus::Cmd>();
    let mut shared = bus::Shared::new(ids.clone(), registry);
    shared.roster.log_path = Some(arg("--events", &if reg_path.ends_with("-ids.json") { reg_path.replace("-ids.json", "-events.jsonl") } else { reg_path.replace(".json", "-events.jsonl") }));
    let lim_path = arg("--limits", &if reg_path.ends_with("-ids.json") { reg_path.replace("-ids.json", "-limits.json") } else { reg_path.replace(".json", "-limits.json") });
    shared.limits = limits::Limits::load(&lim_path);
    shared.tz_h = tz;
    shared.fake = fake;
    let sh = Arc::new(Mutex::new(shared));
    let io: Box<dyn io::Io> = if fake {
        // a demo bench: the adapter, two right-leg servos and two factory-ID servos plugged in together
        let mut b = io::FakeBus::new(&[(100, 2980), (13, 240), (14, 682)]);
        b.servos.push(io::FakeServo::new(1, 250));
        b.servos.push(io::FakeServo::new(1, 700));
        Box::new(b)
    } else {
        Box::new(io::RouterIo { bridge: bridge::Bridge::new(&sock), app })
    };
    {
        let sh = sh.clone();
        let mx = Some(arg("--matrix-frame", "")).filter(|p| !p.is_empty());
        policy::POLICY.lock().unwrap().dir = Some(arg("--policy-dir", "")).filter(|p| !p.is_empty());
        std::thread::spawn(move || bus::run(io, sh, rx, mx));
    }
    let listener = match std::net::TcpListener::bind(("0.0.0.0", port)) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("bind :{port}: {e}");
            std::process::exit(1);
        }
    };
    println!("servo-bus ready on :{port} motion ids {:?} registry {reg_path}{}", ids, if fake { " FAKE BUS" } else { "" });
    use std::io::Write;
    std::io::stdout().flush().ok();
    http::serve(listener, sh, tx);
}
