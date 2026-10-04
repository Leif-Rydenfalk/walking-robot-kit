//! RouterBridge client on `/var/run/arduino-router.sock`, the same wire protocol as the
//! board's `bridge.py` (verified on CPH08 2026-09-11): one call in flight at a time, because
//! the bus thread is this socket's only user and a second in-flight call would only queue
//! behind the first inside the router anyway.
//!
//! A TIMEOUT IS NOT A BROKEN LINK (measured 2026-10-04 19:43-20:34, nine wedges). Until then every
//! timeout dropped the socket, so one slow RPC turned into: reconnect, send again, time out again
//! (the router was still working through the first request), five times in 1.5 s, "bus stopped
//! answering", and a 20 s MCU-app restart. The router journal shows the amplification plainly -
//! eight or nine `Connection closed by peer` / `Accepted connection` pairs in the five seconds
//! before every single restart. The socket is now KEPT across a timeout: the abandoned msgid is
//! remembered, its late reply is read, counted (`late`) and discarded, and the stream stays framed.
//! `late > 0` is the proof that the peer was alive and merely slow; only a real I/O error, a closed
//! socket or a stream that stopped making sense drops the connection.

use crate::msgpack::{decode, encode, Value};
use std::collections::VecDeque;
use std::io::{ErrorKind, Read, Write};
use std::os::unix::net::UnixStream;
use std::time::{Duration, Instant};

pub const DEFAULT_SOCK: &str = "/var/run/arduino-router.sock";

/// Every timeout message starts with this, so a caller can tell "the peer was slow" from
/// "the method is gone" or "the socket broke" without parsing errno text.
pub const TIMEOUT_PREFIX: &str = "router timeout";

/// Abandoned msgids we still watch for a late reply. More than this outstanding at once means the
/// peer really has stopped, and the socket is dropped.
const MAX_ABANDONED: usize = 48;
/// A late reply older than this is nobody's; stop watching for it.
const ABANDON_TTL: Duration = Duration::from_secs(30);
/// A framed stream never needs this much buffer: more means the stream stopped making sense.
const MAX_BUF: usize = 1 << 20;

pub struct Bridge {
    path: String,
    sock: Option<UnixStream>,
    buf: Vec<u8>,
    msgid: i64,
    pub reconnects: u64,
    /// msgids whose call gave up waiting, and when it gave up
    abandoned: VecDeque<(i64, Instant)>,
    /// calls that ran out of time (the peer may still answer)
    pub timeouts: u64,
    /// replies that arrived after their caller had given up: the peer was alive, just late
    pub late: u64,
    /// how late the latest such reply was, in ms (how long past its own deadline we cannot know;
    /// this is measured from the moment the call gave up)
    pub late_ms_max: f64,
}

impl Bridge {
    pub fn new(path: &str) -> Self {
        Bridge {
            path: path.to_string(),
            sock: None,
            buf: Vec::with_capacity(4096),
            msgid: 0,
            reconnects: 0,
            abandoned: VecDeque::new(),
            timeouts: 0,
            late: 0,
            late_ms_max: 0.0,
        }
    }

    fn connect(&mut self) -> Result<(), String> {
        let s = UnixStream::connect(&self.path).map_err(|e| format!("router socket {}: {e}", self.path))?;
        self.sock = Some(s);
        self.buf.clear();
        self.abandoned.clear(); // a new connection can never carry the old connection's replies
        self.reconnects += 1;
        Ok(())
    }

    pub fn drop_socket(&mut self) {
        self.sock = None;
        self.buf.clear();
        self.abandoned.clear();
    }

    pub fn connected(&self) -> bool {
        self.sock.is_some()
    }

    /// outstanding abandoned msgids (a wedge in progress shows here before anything else does)
    pub fn waiting(&self) -> usize {
        self.abandoned.len()
    }

    /// One RPC. A timeout keeps the connection (the reply is still coming); any other failure drops it.
    pub fn call(&mut self, method: &str, params: Vec<Value>, timeout: Duration) -> Result<Value, String> {
        if self.sock.is_none() {
            self.connect()?;
        }
        self.msgid = (self.msgid + 1) & 0x7fff_ffff;
        let id = self.msgid;
        let mut out = Vec::with_capacity(64);
        encode(&Value::Arr(vec![Value::Int(0), Value::Int(id), Value::Str(method.into()), Value::Arr(params)]), &mut out);
        let t0 = Instant::now();
        match self.exchange(&out, id, timeout) {
            Ok(r) => r,
            Err(Fail::Timeout) => {
                self.timeouts += 1;
                self.abandoned.push_back((id, Instant::now()));
                let now = Instant::now();
                while self.abandoned.front().is_some_and(|&(_, t)| now.duration_since(t) > ABANDON_TTL) {
                    self.abandoned.pop_front();
                }
                let over = self.abandoned.len() >= MAX_ABANDONED;
                if over {
                    self.drop_socket();
                }
                Err(format!(
                    "{TIMEOUT_PREFIX}: {method} (id {id}) did not answer in {:.0} ms, {} call(s) still unanswered{}",
                    t0.elapsed().as_secs_f64() * 1000.0,
                    if over { MAX_ABANDONED } else { self.abandoned.len() },
                    if over { "; socket dropped" } else { "" }
                ))
            }
            Err(Fail::Broken(e)) => {
                self.drop_socket();
                Err(e)
            }
        }
    }

    fn exchange(&mut self, out: &[u8], id: i64, timeout: Duration) -> Result<Result<Value, String>, Fail> {
        let s = self.sock.as_mut().ok_or_else(|| Fail::Broken("not connected".into()))?;
        s.write_all(out).map_err(|e| Fail::Broken(format!("router write: {e}")))?;
        let deadline = Instant::now() + timeout;
        let mut chunk = [0u8; 4096];
        loop {
            while let Some((v, n)) = decode(&self.buf) {
                self.buf.drain(..n);
                if let Value::Arr(a) = v {
                    if a.len() == 4 && a[0] == Value::Int(1) {
                        if a[1] == Value::Int(id) {
                            return Ok(match &a[2] {
                                Value::Nil => Ok(a[3].clone()),
                                e => Err(format!("mcu error: {e:?}")),
                            });
                        }
                        // a reply for a call that already gave up: the peer was alive all along
                        if let Value::Int(old) = a[1] {
                            if let Some(k) = self.abandoned.iter().position(|&(x, _)| x == old) {
                                let (_, when) = self.abandoned.remove(k).unwrap();
                                self.late += 1;
                                self.late_ms_max = self.late_ms_max.max(when.elapsed().as_secs_f64() * 1000.0);
                            }
                        }
                    }
                }
            }
            if self.buf.len() > MAX_BUF {
                return Err(Fail::Broken(format!("router stream desync: {} bytes buffered without a whole message", self.buf.len())));
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Err(Fail::Timeout);
            }
            let s = self.sock.as_mut().ok_or_else(|| Fail::Broken("not connected".into()))?;
            s.set_read_timeout(Some(left.max(Duration::from_millis(1)))).ok();
            let n = match s.read(&mut chunk) {
                Ok(n) => n,
                // SO_RCVTIMEO fired: that is this call's deadline, not a broken socket
                Err(e) if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut => continue,
                Err(e) if e.kind() == ErrorKind::Interrupted => continue,
                Err(e) => return Err(Fail::Broken(format!("router read: {e}"))),
            };
            if n == 0 {
                return Err(Fail::Broken("router closed the socket".into()));
            }
            self.buf.extend_from_slice(&chunk[..n]);
        }
    }
}

enum Fail {
    Timeout,
    Broken(String),
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::net::UnixListener;
    use std::sync::mpsc;

    /// A stand-in router: reads whole requests, answers each one after `delay`, and can be told to
    /// hang up. Replies carry the request's own msgid, like the real router.
    fn fake_router(name: &str, delay: Duration, hang_up_after: Option<usize>) -> (String, mpsc::Receiver<String>) {
        let path = format!("{}/sbc-test-{name}-{}.sock", std::env::temp_dir().display(), std::process::id());
        std::fs::remove_file(&path).ok();
        let l = UnixListener::bind(&path).unwrap();
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            for conn in l.incoming() {
                let Ok(mut s) = conn else { break };
                tx.send("accept".to_string()).ok();
                let mut buf: Vec<u8> = Vec::new();
                let mut served = 0usize;
                let mut chunk = [0u8; 4096];
                loop {
                    if hang_up_after.is_some_and(|k| served >= k) {
                        break;
                    }
                    match s.read(&mut chunk) {
                        Ok(0) | Err(_) => break,
                        Ok(n) => buf.extend_from_slice(&chunk[..n]),
                    }
                    while let Some((v, n)) = decode(&buf) {
                        buf.drain(..n);
                        let Value::Arr(a) = v else { continue };
                        let (Value::Int(id), Value::Str(m)) = (a[1].clone(), a[2].clone()) else { continue };
                        tx.send(m.clone()).ok();
                        std::thread::sleep(delay);
                        let mut out = Vec::new();
                        encode(&Value::Arr(vec![Value::Int(1), Value::Int(id), Value::Nil, Value::Int(42)]), &mut out);
                        if s.write_all(&out).is_err() {
                            break;
                        }
                        served += 1;
                        if hang_up_after.is_some_and(|k| served >= k) {
                            break;
                        }
                    }
                }
            }
        });
        (path, rx)
    }

    #[test]
    fn a_late_reply_is_counted_and_the_socket_survives() {
        let (path, _rx) = fake_router("late", Duration::from_millis(220), None);
        let mut b = Bridge::new(&path);
        // the peer takes 220 ms; we wait 60 ms
        let e = b.call("poll_get", vec![], Duration::from_millis(60)).unwrap_err();
        assert!(e.starts_with(TIMEOUT_PREFIX), "{e}");
        assert_eq!(b.reconnects, 1, "a timeout must not reconnect");
        assert!(b.connected(), "a timeout must not drop the socket");
        assert_eq!(b.waiting(), 1);
        // the next call waits long enough: it reads the late reply, counts it, and gets its own
        let v = b.call("poll_get", vec![], Duration::from_millis(2000)).unwrap();
        assert_eq!(v, Value::Int(42));
        assert_eq!(b.late, 1, "the late reply proves the peer was alive");
        assert!(b.late_ms_max > 0.0);
        assert_eq!(b.reconnects, 1, "still the first connection");
        assert_eq!(b.waiting(), 0);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn a_slow_peer_never_makes_us_reconnect() {
        let (path, rx) = fake_router("slow", Duration::from_millis(400), None);
        let mut b = Bridge::new(&path);
        for _ in 0..5 {
            assert!(b.call("poll_get", vec![], Duration::from_millis(20)).is_err());
        }
        assert_eq!(b.reconnects, 1, "five timeouts, one connection");
        assert_eq!(b.timeouts, 5);
        // the stand-in router accepted exactly once (the real router logged 8-9 accepts per wedge)
        let accepts = rx.try_iter().filter(|m| m == "accept").count();
        assert_eq!(accepts, 1);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn a_closed_socket_reconnects() {
        let (path, _rx) = fake_router("closed", Duration::from_millis(0), Some(1));
        let mut b = Bridge::new(&path);
        assert_eq!(b.call("bus_baud", vec![], Duration::from_millis(500)).unwrap(), Value::Int(42));
        // the stand-in hung up after one reply: the next call sees it and reconnects
        let mut reconnected = false;
        for _ in 0..3 {
            if b.call("bus_baud", vec![], Duration::from_millis(500)).is_ok() {
                reconnected = true;
                break;
            }
        }
        assert!(reconnected, "a hung-up socket must be replaced, not kept");
        assert!(b.reconnects >= 2);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn a_reply_for_an_unknown_id_is_skipped_not_returned() {
        // the reply stream carries [1, 999, nil, 7] before [1, id, nil, 42]: the call must take the
        // second, never the first, or every later read would be one reply out of step
        let path = format!("{}/sbc-test-skew-{}.sock", std::env::temp_dir().display(), std::process::id());
        std::fs::remove_file(&path).ok();
        let l = UnixListener::bind(&path).unwrap();
        std::thread::spawn(move || {
            let (mut s, _) = l.accept().unwrap();
            let mut chunk = [0u8; 4096];
            let n = s.read(&mut chunk).unwrap();
            let (v, _) = decode(&chunk[..n]).unwrap();
            let Value::Arr(a) = v else { return };
            let mut out = Vec::new();
            encode(&Value::Arr(vec![Value::Int(1), Value::Int(999), Value::Nil, Value::Int(7)]), &mut out);
            encode(&Value::Arr(vec![Value::Int(1), a[1].clone(), Value::Nil, Value::Int(42)]), &mut out);
            s.write_all(&out).ok();
            std::thread::sleep(Duration::from_millis(200));
        });
        let mut b = Bridge::new(&path);
        assert_eq!(b.call("poll_get", vec![], Duration::from_millis(1000)).unwrap(), Value::Int(42));
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn a_peer_that_never_answers_eventually_drops_the_socket() {
        let path = format!("{}/sbc-test-dead-{}.sock", std::env::temp_dir().display(), std::process::id());
        std::fs::remove_file(&path).ok();
        let l = UnixListener::bind(&path).unwrap();
        std::thread::spawn(move || {
            let mut keep = Vec::new();
            for c in l.incoming() {
                match c {
                    Ok(s) => keep.push(s), // accept and say nothing, ever
                    Err(_) => break,
                }
            }
        });
        let mut b = Bridge::new(&path);
        for _ in 0..MAX_ABANDONED {
            assert!(b.call("poll_get", vec![], Duration::from_millis(1)).is_err());
        }
        assert!(!b.connected(), "a peer that answers nothing at all does get the socket dropped");
        assert_eq!(b.late, 0);
        std::fs::remove_file(&path).ok();
    }
}
