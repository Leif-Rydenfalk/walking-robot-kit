//! The Walk tab's process: start and stop the walking policy (robot-control `sim/policy_through_daemon.py`) from the
//! page, so our 2026-10-04 18:19 order ("run the neural network continuously and move it 5% at 5% the torque of it
//! running") needs no terminal. The policy talks to the bus only through `POST /api/batch` (src/rawgate.rs); this
//! module only owns the child process: one at a time, its log, and the stop (STOP file, then a kill after 4 s).
//! The daemon's raw watchdog turns the leg off if the child dies with torque on.

use crate::json::J;
use std::process::{Child, Command, Stdio};
use std::sync::Mutex;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub const RUNNER: &str = "sim/policy_through_daemon.py";
pub const LOG: &str = "out/unoq-leg/daemon_policy.log";
pub const AMP_MAX: f64 = 0.25;
pub const CAP_MAX: i64 = 300;

pub struct Run {
    child: Child,
    since: f64,
    amp: f64,
    cap: i64,
    who: String,
    stop_asked: Option<Instant>,
}

pub struct Policy {
    pub dir: Option<String>,
    run: Option<Run>,
    last_exit: String,
}

pub static POLICY: Mutex<Policy> = Mutex::new(Policy { dir: None, run: None, last_exit: String::new() });

fn now() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
}

impl Policy {
    pub fn available(&self) -> bool {
        self.dir.as_ref().map(|d| std::path::Path::new(d).join(RUNNER).exists()).unwrap_or(false)
    }

    /// Reap a finished child; kill one that ignored the STOP file for 4 s.
    fn reap(&mut self) {
        let Some(r) = self.run.as_mut() else { return };
        match r.child.try_wait() {
            Ok(Some(st)) => {
                self.last_exit = format!("ended {} ({st})", crate::bus::hhmmss());
                self.run = None;
            }
            Ok(None) => {
                if r.stop_asked.map(|t| t.elapsed() > Duration::from_secs(4)).unwrap_or(false) {
                    r.child.kill().ok();
                    r.child.wait().ok();
                    self.last_exit = format!("killed {} after the STOP file was ignored for 4 s", crate::bus::hhmmss());
                    self.run = None;
                }
            }
            Err(e) => {
                self.last_exit = format!("lost the child: {e}");
                self.run = None;
            }
        }
    }

    pub fn start(&mut self, amp: f64, cap: i64, who: &str) -> Result<(), String> {
        self.reap();
        if self.run.is_some() {
            return Err("the policy is already running: stop it first".into());
        }
        let Some(dir) = self.dir.clone().filter(|_| self.available()) else {
            return Err(format!("no policy runner on this board ({RUNNER} under --policy-dir)"));
        };
        if !(0.0..=AMP_MAX).contains(&amp) || amp == 0.0 {
            return Err(format!("amp {amp} is outside 0..{AMP_MAX}"));
        }
        if !(1..=CAP_MAX).contains(&cap) {
            return Err(format!("torque cap {cap} is outside 1..{CAP_MAX}"));
        }
        let log = std::path::Path::new(&dir).join(LOG);
        if let Some(p) = log.parent() {
            std::fs::create_dir_all(p).ok();
        }
        let f = std::fs::File::create(&log).map_err(|e| format!("log {}: {e}", log.display()))?;
        let f2 = f.try_clone().map_err(|e| e.to_string())?;
        std::fs::remove_file(std::path::Path::new(&dir).join("STOP")).ok();
        let child = Command::new("python3")
            .args(["-u", RUNNER, "--amp", &format!("{amp}"), "--cap", &cap.to_string(), "--name", &format!("page_{}", now() as u64)])
            .current_dir(&dir)
            .env("PYTHONPATH", "py")
            .stdin(Stdio::null())
            .stdout(f)
            .stderr(f2)
            .spawn()
            .map_err(|e| format!("python3 {RUNNER}: {e}"))?;
        self.run = Some(Run { child, since: now(), amp, cap, who: who.to_string(), stop_asked: None });
        Ok(())
    }

    pub fn stop(&mut self) {
        if let (Some(r), Some(dir)) = (self.run.as_mut(), self.dir.as_ref()) {
            std::fs::write(std::path::Path::new(dir).join("STOP"), b"stop from the page\n").ok();
            r.stop_asked.get_or_insert_with(Instant::now);
        }
        self.reap();
    }

    pub fn to_json(&mut self) -> J {
        self.reap();
        let tail = self
            .dir
            .as_ref()
            .and_then(|d| std::fs::read_to_string(std::path::Path::new(d).join(LOG)).ok())
            .map(|s| s.lines().rev().take(6).collect::<Vec<_>>().into_iter().rev().map(|l| l.chars().take(200).collect::<String>()).collect::<Vec<_>>().join("\n"))
            .unwrap_or_default();
        let mut o = vec![("available", J::Bool(self.available())), ("running", J::Bool(self.run.is_some())), ("last_exit", J::s(&self.last_exit)), ("tail", J::s(&tail))];
        if let Some(r) = &self.run {
            o.push(("since", J::n(r.since)));
            o.push(("amp", J::n(r.amp)));
            o.push(("cap", J::n(r.cap as f64)));
            o.push(("who", J::s(&r.who)));
            o.push(("pid", J::n(r.child.id() as f64)));
            o.push(("stopping", J::Bool(r.stop_asked.is_some())));
        }
        J::obj(o)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn refuses_without_a_runner_and_outside_the_bounds() {
        let d = std::env::temp_dir().join(format!("sbc-policy-{}", std::process::id()));
        std::fs::create_dir_all(d.join("sim")).unwrap();
        let mut p = Policy { dir: Some(d.to_string_lossy().into()), run: None, last_exit: String::new() };
        assert!(p.start(0.05, 50, "t").unwrap_err().contains("no policy runner"));
        // a stand-in runner that waits for the STOP file like run_loop does
        std::fs::write(d.join(RUNNER), "import os,time\nwhile not os.path.exists('STOP'): time.sleep(0.05)\nprint('stopped')\n").unwrap();
        assert!(p.start(0.5, 50, "t").unwrap_err().contains("amp"), "50 % motion is refused");
        assert!(p.start(0.05, 1000, "t").unwrap_err().contains("torque cap"), "full torque is refused");
        p.start(0.05, 50, "t").unwrap();
        assert!(p.start(0.05, 50, "t").is_err(), "one run at a time");
        p.stop();
        let t = Instant::now();
        while p.run.is_some() && t.elapsed() < Duration::from_secs(3) {
            std::thread::sleep(Duration::from_millis(50));
            p.reap();
        }
        assert!(p.run.is_none(), "the STOP file ends the run");
        assert!(p.to_json().get("tail").and_then(|x| x.as_str()).unwrap_or("").contains("stopped"));
        std::fs::remove_dir_all(&d).ok();
    }
}
