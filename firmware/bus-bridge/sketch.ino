// servo-bus MCU half (Uno Q app `servo-bus-bridge`).
// Raw packet pass-through to a Feetech-style TTL servo bus on D0/D1 (Serial1) via a URT-2.
// The sketch knows frame boundaries and nothing else: the Linux side (the Rust daemon)
// builds every packet and parses every reply, so the register map lives in one place.
//
// bus_batch is the fast path. One RouterBridge call (~7 ms round trip, measured 2026-10-02)
// carries many bus transactions, and each one returns as soon as its expected byte count
// is in, instead of one RPC per received byte (bus_rx), which cost ~27 calls per read.
//
//   in : frames = [expect, FF FF id len inst params.. chk] repeated; wait_us = per-frame cap
//   out: [n, n received bytes] per frame, echo included (the URT-2 hears its own TX)
//
// bus_begin / bus_send / bus_rx / bus_baud are kept so bus.py and servo_live.py still work.
//
// imu_scan / imu_begin / imu_read / imu_stats read an MPU-6050 on whichever I2C bus it is wired to
// (header D20/D21 = Wire, Qwiic = Wire1), because the walking policy needs a gyro and a gravity
// direction and nothing else on the robot has one.
//
//
// Also drives the board's 13 x 8 LED matrix (3 grayscale bits, 0..7) and the RGB LED3, so the leg's
// live stats can show while it walks 07 "use the aurdino led matrix as a output
// device for me to see statistics"). The MCU runs one sketch at a time, so the matrix lives here;
// mx_* and set_led3_color are byte-compatible with the unoq-matrix app.
#include <vector>
#include <utility>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <Arduino_RPClite.h>
// Arduino_RouterBridge 0.4.3 runs every RPC handler on a thread with a 500-byte stack (UPDATE_THREAD_STACK_SIZE,
// no #ifndef) and puts a 256-byte RPCRequest plus packer and unpacker on it. The MCU wedged every 30 s - 3 min
// on 2026-10-04 (15:53-16:06) with every method dead, bus_baud and led3_state included. We reach the private
// thread id to stop that thread and run Bridge.update() on our own 8 KB stack instead (see setup()).
#include <Arduino_RouterBridge.h>
#include <Arduino_LED_Matrix.h>
#include <Wire.h>
// Reading the private BridgeClass::upd_tid: an explicit template instantiation may name a private member.
template <typename Tag, typename Tag::type M> struct Rob { friend typename Tag::type get(Tag) { return M; } };
struct UpdTid { typedef k_tid_t BridgeClass::*type; friend type get(UpdTid); };
template struct Rob<UpdTid, &BridgeClass::upd_tid>;
struct ReadMx { typedef struct k_mutex BridgeClass::*type; friend type get(ReadMx); };
template struct Rob<ReadMx, &BridgeClass::read_mutex>;
struct WriteMx { typedef struct k_mutex BridgeClass::*type; friend type get(WriteMx); };
template struct Rob<WriteMx, &BridgeClass::write_mutex>;

K_MUTEX_DEFINE(bus_mtx);
static uint8_t g_rx[64];
static int g_rxn = 0;
static int g_baud = 0;

// NOTHING ON THIS MCU MAY WAIT FOR EVER (2026-10-04 20:53:28 and 20:55:49 CST). The sketch serves RPCs one at a
// time on one thread, so a single handler that blocks takes every method down with it: both wedges that evening
// killed bus_baud and mx_shape too, and those touch no hardware at all. The one unbounded wait was
// arduino::ZephyrSerial::write (cores/arduino/zephyrSerial.cpp): it loops `while (1)` putting bytes into the TX
// ring and yielding whenever the ring is full, so a UART whose TX interrupt has stopped firing holds the caller
// for ever. Two rules now: take the bus mutex with a deadline, and never hand Serial1 more bytes than the ring
// has room for (then ring_buf_put takes them all and write() cannot loop).
#define BUS_LOCK_MS 300
static bool bus_lock() { return k_mutex_lock(&bus_mtx, K_MSEC(BUS_LOCK_MS)) == 0; }
static uint32_t g_tx_stuck = 0;   // frames refused because the TX ring would not drain
static uint32_t g_lock_lost = 0;  // calls that gave up waiting for the bus mutex

// Reopen the port: uart_configure + uart_irq_rx_enable, which is the only cure the sketch has for a UART that
// stopped transmitting, short of the Linux side restarting the app.
static void bus_reopen() {
  if (!g_baud) return;
  Serial1.end();
  Serial1.begin((unsigned long)g_baud);
}

// Write one whole frame or none of it, never blocking. Returns false when the TX ring has no room, which means
// the UART is not draining; the port is reopened and the caller reports the frame as unsent.
static bool tx_frame(const uint8_t* b, size_t n) {
  if ((size_t)Serial1.availableForWrite() < n) {
    g_tx_stuck++;
    bus_reopen();
    return false;
  }
  Serial1.write(b, n);
  return true;
}
int bus_stuck() { return (int)((g_tx_stuck & 0xFFFF) << 16 | (g_lock_lost & 0xFFFF)); }

// Drain what is waiting, bounded. The unbounded `while (Serial1.available()) Serial1.read();` spun forever when a
// servo kept talking (our left hip servo, 2026-10-04 15:3x-15:46: the whole bus and every RPC went quiet, even
// bus_baud, because the MCU's one RPC thread never left the loop). Returns the bytes dropped (g_junk counts them).
static uint32_t g_junk = 0;
static int bus_drain() {
  int n = 0;
  uint32_t t0 = micros();
  while (Serial1.available() && n < 512 && (uint32_t)(micros() - t0) < 2000u) { Serial1.read(); n++; }
  g_junk += n;
  return n;
}
int bus_junk() { return (int)g_junk; }

// Wait for a packet to leave, bounded. Serial1.flush() waits for transmit-complete with no limit; a UART left in
// an error state by a hot-plug glitch would hold the RPC thread (and bus_mtx) forever, which is what the wedges
// at 15:53-16:02 looked like: every RPC dead, bus_baud included, with the drain already bounded. The bytes take
// 10 bits each at the bus baud; a reply cannot start before the servo's return delay anyway.
static void tx_wait(size_t n) {
  uint32_t us = (uint32_t)((uint64_t)n * 10u * 1000000u / (uint32_t)(g_baud ? g_baud : 1000000)) + 30u;
  delayMicroseconds(us);
}

// Returns 1 when the bus was opened with the mutex held, 2 when it was opened while another call still held it
// (that call is stuck inside Serial1 and reopening the port is the only thing that can free it), never 0.
int bus_begin(int baud) {
  bool got = bus_lock();
  if (!got) g_lock_lost++;
  if (g_baud) Serial1.end();
  Serial1.begin((unsigned long)baud);
  g_baud = baud;
  if (got) k_mutex_unlock(&bus_mtx);
  return got ? 1 : 2;
}

// Sends tx, collects everything that comes back within wait_ms (echo included). Returns byte count.
int bus_send(std::vector<uint8_t> tx, int wait_ms) {
  if (!g_baud) return -1;
  if (wait_ms < 1) wait_ms = 1; if (wait_ms > 200) wait_ms = 200;
  if (!bus_lock()) { g_lock_lost++; return -2; }
  bus_drain();
  if (!tx_frame(tx.data(), tx.size())) { k_mutex_unlock(&bus_mtx); return -3; }
  tx_wait(tx.size());
  g_rxn = 0;
  uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < (uint32_t)wait_ms) {
    while (Serial1.available() && g_rxn < (int)sizeof g_rx) g_rx[g_rxn++] = (uint8_t)Serial1.read();
  }
  int n = g_rxn;
  k_mutex_unlock(&bus_mtx);
  return n;
}

int bus_rx(int i) { return (i >= 0 && i < g_rxn) ? (int)g_rx[i] : -1; }
int bus_baud() { return g_baud; }

std::vector<uint8_t> bus_batch(std::vector<uint8_t> frames, int wait_us) {
  std::vector<uint8_t> out;
  if (!g_baud) return out;
  if (wait_us < 100) wait_us = 100; if (wait_us > 20000) wait_us = 20000;
  if (!bus_lock()) { g_lock_lost++; return out; }   // empty: the daemon reads that as a closed bus and reopens it
  out.reserve(frames.size() + 64);                  // no realloc while bytes are arriving at 10 us apart
  size_t k = 0;
  while (k + 5 <= frames.size()) {
    int expect = frames[k];
    size_t plen = 4 + (size_t)frames[k + 4];          // FF FF id len ... : len counts inst+params+chk
    if (frames[k + 1] != 0xFF || frames[k + 2] != 0xFF || k + 1 + plen > frames.size()) break;
    bus_drain();
    if (!tx_frame(&frames[k + 1], plen)) break;
    tx_wait(plen);
    size_t at = out.size();
    out.push_back(0);
    int n = 0;
    uint32_t t0 = micros();
    while (n < expect && n < 250 && (uint32_t)(micros() - t0) < (uint32_t)wait_us) {
      while (Serial1.available() && n < 250) { out.push_back((uint8_t)Serial1.read()); n++; }
    }
    out[at] = (uint8_t)n;
    k += 1 + plen;
  }
  k_mutex_unlock(&bus_mtx);
  return out;
}

// ---- MPU-6050 on I2C: which way is up (we soldered one on 2026-10-04 22:18) ----------------------------
// the robot had no inertial sensor, so the walking policy took its gyro and gravity from a simulated duck and could
// step but not balance. The part is a GY-521 at address 0x68 (AD0 to GND).
//
// WHICH BUS IT IS ON IS MEASURED, NEVER ASSUMED. we had no Qwiic cable, so it went onto the header:
// SCL = D21 = PB10, SDA = D20 = PB11. That is a DIFFERENT bus from the Qwiic socket. Read off the official
// core's own variant overlay (arduino/ArduinoCore-zephyr, variants/arduino_uno_q_stm32u585xx, fetched
// 2026-10-04): `zephyr,user` declares `i2cs = <&i2c2>, <&i2c4>, <&i2c3>`, and libraries/Wire/Wire.h names
// them in that order, so:
//
//   Wire  = i2c2   the header pins, D20 PB11 (SDA) and D21 PB10 (SCL). The overlay's own comments say
//                  "D20 - PB11" and "D21 - PB10", which is where the sensor is now.
//   Wire1 = i2c4   the Qwiic socket, i2c4_scl_pd12 and i2c4_sda_pd13
//   Wire2 = i2c3   i2c3_scl_pc0 and i2c3_sda_pc1
//
// i2c2 carries `zephyr,deferred-init` in that overlay, so it is not brought up until something calls begin().
// That is what imu_wire_begin does, and it is why a scan must begin a bus before scanning it.
//
// Every call here tries EVERY declared bus and reports which one answered, because a wire moved from the
// header to the Qwiic socket must not turn into a sensor that silently stopped existing.
//
//   imu_scan()                  -> [bus, addr, ...] for 0x68/0x69 only; imu_scan_all() sweeps 0x08..0x77
//   imu_begin(addr, dlpf, div)  -> [ok, bus, who_am_i, pwr, cfg, gyro_cfg, accel_cfg, smplrt] read BACK
//   imu_read()                  -> [n, 14 bytes] ax ay az temp gx gy gz, big-endian int16 each, or [0]
//   imu_stats()                 -> [ok lo, ok hi, err lo, err hi, addr, began, bus]
//
// Scales are fixed here and stated once: gyro +/-500 dps = 65.5 LSB per deg/s, accel +/-4 g = 8192 LSB per g.
// The Linux side owns every conversion, the same way it owns the servo register map.
//
// NOTHING ON THIS MCU MAY WAIT FOR EVER - and checking Wire's return value is NOT enough to promise that.
// This core's TwoWire calls Zephyr i2c_write/i2c_read straight through (libraries/Wire/Wire.cpp:117,138),
// and the STM32 driver waits on its completion semaphore with K_FOREVER. A sensor that holds SDA down, or
// an interrupt that never arrives, blocks the CALLER for ever, and before 2026-10-05 00:4x that caller was
// the RPC thread: /api/health went to hz 0 with 98 timeouts while bus_batch sat unanswered behind one
// stuck I2C read. Every transaction below therefore runs on imu_thread and NOWHERE else. The four RPC
// handlers touch a snapshot and a few flags, so the worst a wedged I2C bus can now cost is the IMU.
#define IMU_PWR1 0x6B
#define IMU_CFG 0x1A
#define IMU_SMPLRT 0x19
#define IMU_GYRO_CFG 0x1B
#define IMU_ACCEL_CFG 0x1C
#define IMU_BURST 0x3B
#define IMU_WHOAMI 0x75

// The buses this build has, in the order Wire.h declares them. If a future variant drops one, the compile
// fails here and loudly, which is the right place for that to happen.
static TwoWire* const IMU_BUSES[] = {&Wire, &Wire1, &Wire2};
#define IMU_NBUS ((int)(sizeof IMU_BUSES / sizeof IMU_BUSES[0]))
static bool imu_bus_up[IMU_NBUS];
static uint8_t imu_addr = 0;
static int imu_bus = -1;              // which bus answered; -1 = none has
static bool imu_began = false;
static uint32_t imu_ok_n = 0, imu_err_n = 0;

static TwoWire* imu_wire_begin(int b) {
  if (b < 0 || b >= IMU_NBUS) return nullptr;
  if (!imu_bus_up[b]) {
    IMU_BUSES[b]->begin();          // i2c2 is deferred-init in the variant: nothing works before this
    IMU_BUSES[b]->setClock(400000);
    imu_bus_up[b] = true;
  }
  return IMU_BUSES[b];
}

// -1 when the chip did not answer, so a caller can never mistake a dead bus for a register that reads 0
static int imu_reg(int b, uint8_t addr, uint8_t reg) {
  TwoWire* w = imu_wire_begin(b);
  if (!w) return -1;
  w->beginTransmission(addr);
  w->write(reg);
  if (w->endTransmission(false) != 0) return -1;
  if (w->requestFrom((int)addr, 1) != 1) return -1;
  return w->read();
}

static bool imu_write(int b, uint8_t addr, uint8_t reg, uint8_t val) {
  TwoWire* w = imu_wire_begin(b);
  if (!w) return false;
  w->beginTransmission(addr);
  w->write(reg);
  w->write(val);
  return w->endTransmission(true) == 0;
}

static bool imu_acks(int b, uint8_t addr) {
  TwoWire* w = imu_wire_begin(b);
  if (!w) return false;
  w->beginTransmission(addr);
  return w->endTransmission(true) == 0;
}

// [bus, addr, bus, addr, ...] so one call answers "is it wired to the header or the Qwiic socket"
static std::vector<uint8_t> imu_scan_i2c(bool full) {
  std::vector<uint8_t> found;
  for (int b = 0; b < IMU_NBUS; b++) {
    // 0x68 and 0x69 are the MPU-6050's only two addresses. A full 0x08..0x77 sweep is 112 probes a bus,
    // each one a timeout when nothing is there, and polling it cost the servo bus on 2026-10-05.
    for (uint8_t a = full ? 0x08 : 0x68; a <= (full ? 0x77 : 0x69); a++) {
      if (imu_acks(b, a)) {
        found.push_back((uint8_t)b);
        found.push_back(a);
      }
    }
  }
  return found;
}

// Tries every bus until one carries an MPU-6050 at `addr`, then configures it there. Everything is READ
// BACK from the chip: a write that did not land must not look like a configured sensor.
static std::vector<int> imu_begin_i2c(int addr, int dlpf, int div) {
  std::vector<int> out;
  uint8_t a = (uint8_t)(addr ? addr : 0x68);
  if (dlpf < 0 || dlpf > 6) dlpf = 3;    // 44 Hz accel / 42 Hz gyro, 4.9 ms group delay, for a 50 Hz loop
  if (div < 0 || div > 255) div = 4;     // 1 kHz / (1 + 4) = 200 Hz sample rate
  imu_began = false;
  imu_addr = 0;
  imu_bus = -1;
  int last_who = -1;
  for (int b = 0; b < IMU_NBUS; b++) {
    if (!imu_acks(b, a)) continue;                 // nothing at that address on this bus: next bus
    if (!imu_write(b, a, IMU_PWR1, 0x80)) continue;  // reset
    delay(100);
    if (!imu_write(b, a, IMU_PWR1, 0x01)) continue;  // wake, clock from the X gyro
    delay(10);
    imu_write(b, a, IMU_CFG, (uint8_t)dlpf);
    imu_write(b, a, IMU_SMPLRT, (uint8_t)div);
    imu_write(b, a, IMU_GYRO_CFG, 0x08);           // +/- 500 dps
    imu_write(b, a, IMU_ACCEL_CFG, 0x08);          // +/- 4 g
    delay(10);
    int who = imu_reg(b, a, IMU_WHOAMI), pwr = imu_reg(b, a, IMU_PWR1), cfg = imu_reg(b, a, IMU_CFG);
    int gy = imu_reg(b, a, IMU_GYRO_CFG), ac = imu_reg(b, a, IMU_ACCEL_CFG), sr = imu_reg(b, a, IMU_SMPLRT);
    last_who = who;
    if (who == 0x68 && pwr >= 0 && (pwr & 0x40) == 0 && gy == 0x08 && ac == 0x08) {
      imu_began = true;
      imu_addr = a;
      imu_bus = b;
      out.push_back(1); out.push_back(b); out.push_back(who); out.push_back(pwr);
      out.push_back(cfg); out.push_back(gy); out.push_back(ac); out.push_back(sr);
      return out;
    }
  }
  // nothing usable anywhere: say so with the last WHO_AM_I seen, never with a bus number
  out.push_back(0); out.push_back(-1); out.push_back(last_who);
  out.push_back(-1); out.push_back(-1); out.push_back(-1); out.push_back(-1); out.push_back(-1);
  return out;
}

static std::vector<uint8_t> imu_read_i2c() {
  std::vector<uint8_t> out;
  if (!imu_began || imu_bus < 0) { out.push_back(0); return out; }
  TwoWire* w = imu_wire_begin(imu_bus);
  if (!w) { out.push_back(0); return out; }
  w->beginTransmission(imu_addr);
  w->write(IMU_BURST);
  if (w->endTransmission(false) != 0) { imu_err_n++; out.push_back(0); return out; }
  int n = w->requestFrom((int)imu_addr, 14);
  if (n != 14) { imu_err_n++; out.push_back(0); return out; }
  out.push_back(14);
  for (int i = 0; i < 14; i++) out.push_back((uint8_t)w->read());
  imu_ok_n++;
  return out;
}

// ---- the sampler thread: the only code in this sketch allowed to touch I2C --------------------------
K_THREAD_STACK_DEFINE(imu_stack, 2048);
static struct k_thread imu_thread;
static volatile uint32_t imu_seq = 0;           // seqlock: odd while writing, even when the snapshot is whole
static uint8_t imu_snap[14];
static volatile uint32_t imu_snap_ms = 0;
static volatile uint32_t imu_loops = 0;         // proves the thread is alive; frozen = it is stuck in I2C
static volatile int imu_want_begin = 0, imu_want_scan = 0, imu_scan_full = 0;
static volatile int imu_req_addr = 0x68, imu_req_dlpf = 3, imu_req_div = 4;
static int imu_begin_out[8] = {0, -1, -1, -1, -1, -1, -1, -1};
static uint8_t imu_scan_out[32];
static volatile int imu_scan_n = 0;
static volatile uint32_t imu_begin_seq = 0, imu_scan_seq = 0;
static volatile uint32_t imu_miss = 0;          // consecutive failed reads
#define IMU_MISS_GIVE_UP 10                     // after this the thread stops touching the bus until a begin
#define IMU_PERIOD_MS 20                        // 50 Hz, the policy's own rate: the I2C runs on this
                                                // thread and rides out on poll_get, so it costs no RPCs
#define IMU_STALE_MS 500                        // a snapshot older than this is not a reading

static void imu_loop(void*, void*, void*) {
  while (true) {
    imu_loops++;
    if (imu_want_scan) {
      std::vector<uint8_t> f = imu_scan_i2c(imu_scan_full != 0);
      int n = (int)f.size(); if (n > (int)sizeof imu_scan_out) n = (int)sizeof imu_scan_out;
      for (int i = 0; i < n; i++) imu_scan_out[i] = f[i];
      imu_scan_n = n;
      imu_scan_seq++;
      imu_want_scan = 0;
    }
    if (imu_want_begin) {
      std::vector<int> r = imu_begin_i2c(imu_req_addr, imu_req_dlpf, imu_req_div);
      for (int i = 0; i < 8 && i < (int)r.size(); i++) imu_begin_out[i] = r[i];
      imu_miss = 0;
      imu_begin_seq++;
      imu_want_begin = 0;
    }
    if (imu_began && imu_miss < IMU_MISS_GIVE_UP) {
      std::vector<uint8_t> r = imu_read_i2c();
      if (r.size() == 15 && r[0] == 14) {
        imu_seq++;                        // odd: a reader that sees this retries rather than tearing
        __sync_synchronize();
        for (int i = 0; i < 14; i++) imu_snap[i] = r[i + 1];
        __sync_synchronize();
        imu_seq++;                        // even again: whole
        imu_snap_ms = millis();
        imu_miss = 0;
      } else {
        imu_miss++;
      }
    }
    k_msleep(IMU_PERIOD_MS);
  }
}

// Wait for the sampler to answer, without ever blocking on it. A wedged I2C bus means the sampler never
// comes back, and then this returns the honest "not yet" instead of taking the RPC thread down with it.
static bool imu_wait(volatile uint32_t* seq, uint32_t was, int ms) {
  for (int i = 0; i < ms; i += 10) {
    if (*seq != was) return true;
    k_msleep(10);
  }
  return false;
}

// ---- the four RPC faces: snapshots and flags, never a bus transaction -------------------------------
static std::vector<uint8_t> imu_scan_req(bool full) {
  uint32_t was = imu_scan_seq;
  imu_scan_full = full ? 1 : 0;
  imu_want_scan = 1;
  std::vector<uint8_t> out;
  // This sleeps the RPC thread, so a scan stalls bus_batch for as long as it waits. 2.5 s is a deliberate,
  // rare diagnostic; the daemon's own imu_scan timeout is 10 s, so it hears the honest empty answer.
  if (!imu_wait(&imu_scan_seq, was, 2500)) return out;      // empty = the sampler did not answer in time
  for (int i = 0; i < imu_scan_n; i++) out.push_back(imu_scan_out[i]);
  return out;
}
// Two names rather than an argument, so the daemon's existing zero-argument call keeps working and the
// expensive sweep has to be asked for on purpose.
std::vector<uint8_t> imu_scan() { return imu_scan_req(false); }
std::vector<uint8_t> imu_scan_all() { return imu_scan_req(true); }

std::vector<int> imu_begin(int addr, int dlpf, int div) {
  uint32_t was = imu_begin_seq;
  imu_req_addr = addr ? addr : 0x68;
  imu_req_dlpf = dlpf;
  imu_req_div = div;
  imu_want_begin = 1;
  std::vector<int> out;
  // 800 ms and not longer: the daemon gives this call 900 ms (MX_TIMEOUT), and an answer it never hears
  // leaves a call unanswered on the bridge, which is what a backed-up link looks like from the outside.
  if (!imu_wait(&imu_begin_seq, was, 800)) {
    // -2 is not a WHO_AM_I: it says the I2C bus did not come back, which is a different fault from a
    // chip that answered wrongly, and a caller that cannot tell them apart will chase the wrong wire.
    out.push_back(0); out.push_back(-1); out.push_back(-2);
    out.push_back(-1); out.push_back(-1); out.push_back(-1); out.push_back(-1); out.push_back(-1);
    return out;
  }
  for (int i = 0; i < 8; i++) out.push_back(imu_begin_out[i]);
  return out;
}

// Read the snapshot without locking: a mutex here could be held by a sampler stuck inside I2C, and then
// the RPC thread would wait on it - exactly the failure this whole arrangement exists to prevent.
static bool imu_snapshot(uint8_t out[14], uint32_t* age_ms) {
  for (int t = 0; t < 8; t++) {
    uint32_t s1 = imu_seq;
    if (s1 & 1) { k_yield(); continue; }     // odd: the sampler is mid-write, so this copy would tear
    __sync_synchronize();
    for (int i = 0; i < 14; i++) out[i] = imu_snap[i];
    __sync_synchronize();
    if (imu_seq == s1) {
      *age_ms = millis() - imu_snap_ms;
      return imu_snap_ms != 0;
    }
  }
  return false;
}

std::vector<uint8_t> imu_read() {
  std::vector<uint8_t> out;
  uint8_t tmp[14];
  uint32_t age = 0;
  bool got = imu_snapshot(tmp, &age);
  if (!got || age > IMU_STALE_MS) { out.push_back(0); return out; }
  out.push_back(14);
  for (int i = 0; i < 14; i++) out.push_back(tmp[i]);
  return out;
}

std::vector<int> imu_stats() {
  std::vector<int> out;
  out.push_back((int)(imu_ok_n & 0xFFFF));
  out.push_back((int)(imu_ok_n >> 16));
  out.push_back((int)(imu_err_n & 0xFFFF));
  out.push_back((int)(imu_err_n >> 16));
  out.push_back((int)imu_addr);
  out.push_back(imu_began ? 1 : 0);
  out.push_back(imu_bus);
  out.push_back((int)(millis() - imu_snap_ms));     // how old the newest sample is, ms
  out.push_back((int)imu_miss);                     // consecutive failures; IMU_MISS_GIVE_UP = stopped
  out.push_back((int)(imu_loops & 0xFFFF));         // frozen between two calls = the sampler is stuck in I2C
  return out;
}

// ---- poll loop: the MCU sweeps the bus itself (2026-10-04,  ---------------------
// One RPC costs ~5.5 ms plus ~87 us per byte on the 115200-baud router link, so the daemon cannot afford a
// request and a reply per servo per tick. loop() sweeps instead: the position of every polled ID, one full
// 40..70 read in rotation, an EEPROM 5..17 read every 8th sweep and the next 2 IDs of the 0..253 ping scan.
// poll_get hands the daemon the newest sweep in a compact form (4 bytes a servo). bus_batch still works
// beside it (set-ID, torque, goals): both take bus_mtx per transaction.
//   poll_set(ids)  -> number of IDs polled (max 32); empty = sweep only the scan
//   poll_get()     -> [seq lo, seq hi, sweeps since last get, n, (id, kind, a, b) x n,
//                      full id, full len, data.., ee id, ee len, data.., wraps, n found, found ids..]
//      kind 0 = clean, position a | b << 8; 1 = nothing; 2 = garbled, a = bytes; 3 = clean but the servo's error
//      byte is set (overload, heat...), position a | b << 8
static uint8_t p_ids[32];
static int p_n = 0;
K_MUTEX_DEFINE(p_mtx);
static uint8_t p_cur[32][4];
static uint8_t p_full[34];  // id, len, 31 data bytes
static uint8_t p_ee[15];    // id, len, 13 data bytes
static uint8_t p_found[64];
static int p_nfound = 0, p_wraps = 0, p_sweeps = 0;
static uint16_t p_seq = 0;
static uint8_t p_scan = 0;
static uint32_t p_turn = 0;

static uint8_t fs_sum(const uint8_t* b, int n) { int s = 0; for (int i = 0; i < n; i++) s += b[i]; return (uint8_t)~s; }

// one transaction: send, wait up to cap_us for `expect` bytes; returns bytes in r
static int txn(const uint8_t* pkt, int plen, uint8_t* r, int expect, uint32_t cap_us) {
  if (!bus_lock()) { g_lock_lost++; return 0; }
  bus_drain();
  if (!tx_frame(pkt, plen)) { k_mutex_unlock(&bus_mtx); return 0; }
  tx_wait(plen);
  int n = 0;
  uint32_t t0 = micros();
  while (n < expect && (uint32_t)(micros() - t0) < cap_us) {
    while (Serial1.available() && n < expect) r[n++] = (uint8_t)Serial1.read();
  }
  k_mutex_unlock(&bus_mtx);
  return n;
}

static int read_pkt(uint8_t id, uint8_t addr, uint8_t len, uint8_t* pkt) {
  pkt[0] = 0xFF; pkt[1] = 0xFF; pkt[2] = id; pkt[3] = 4; pkt[4] = 0x02; pkt[5] = addr; pkt[6] = len;
  pkt[7] = fs_sum(pkt + 2, 5);
  return 8;
}

// a clean status packet from `id` carrying `len` data bytes? returns the error byte, or -1
static int clean(const uint8_t* r, int n, uint8_t id, int len) {
  if (n != len + 6 || r[0] != 0xFF || r[1] != 0xFF || r[2] != id || r[3] != len + 2) return -1;
  if (fs_sum(r + 2, len + 3) != r[len + 5]) return -1;
  return r[4];
}

static bool p_on = false;
int poll_set(std::vector<uint8_t> ids) {
  k_mutex_lock(&p_mtx, K_FOREVER);
  p_on = true;
  p_n = 0;
  for (size_t i = 0; i < ids.size() && p_n < 32; i++) p_ids[p_n++] = ids[i];
  k_mutex_unlock(&p_mtx);
  return p_n;
}

std::vector<uint8_t> poll_get() {
  std::vector<uint8_t> o;
  k_mutex_lock(&p_mtx, K_FOREVER);
  o.push_back(p_seq & 0xFF); o.push_back(p_seq >> 8); o.push_back(p_sweeps > 255 ? 255 : p_sweeps); o.push_back(p_n);
  for (int i = 0; i < p_n; i++) for (int k = 0; k < 4; k++) o.push_back(p_cur[i][k]);
  for (int k = 0; k < 2 + p_full[1]; k++) o.push_back(p_full[k]);
  for (int k = 0; k < 2 + p_ee[1]; k++) o.push_back(p_ee[k]);
  o.push_back(p_wraps > 255 ? 255 : p_wraps);
  o.push_back(p_nfound);
  for (int i = 0; i < p_nfound; i++) o.push_back(p_found[i]);
  // IMU trailer, 17 bytes, always: [have, 14 raw bytes, age lo, age hi]. The sweep already asks for this
  // reply every cycle, so the IMU costs NOTHING extra on the link. Measured 2026-10-05 01:0x: reading it
  // over its own RPC at 10 Hz took the sweep from 49.7 Hz to 32.4 Hz - about 34 ms of round trip each,
  // which is latency and not bytes. An older daemon stops parsing before this and is unaffected.
  {
    uint8_t t14[14];
    uint32_t age = 0;
    bool have = imu_snapshot(t14, &age);
    if (age > 65535) age = 65535;
    o.push_back(have ? 1 : 0);
    for (int i = 0; i < 14; i++) o.push_back(have ? t14[i] : 0);
    o.push_back((uint8_t)(age & 0xFF));
    o.push_back((uint8_t)(age >> 8));
  }
  p_full[0] = 0; p_full[1] = 0; p_ee[0] = 0; p_ee[1] = 0;  // each block is handed over once
  p_sweeps = 0; p_wraps = 0; p_nfound = 0;
  k_mutex_unlock(&p_mtx);
  return o;
}

static void sweep() {
  uint8_t ids[32]; int n;
  k_mutex_lock(&p_mtx, K_FOREVER);
  n = p_n; memcpy(ids, p_ids, n);
  k_mutex_unlock(&p_mtx);
  uint8_t pkt[8], r[40], cur[32][4];
  for (int i = 0; i < n; i++) {
    read_pkt(ids[i], 56, 2, pkt);
    int got = txn(pkt, 8, r, 8, 1000);
    int e = clean(r, got, ids[i], 2);
    cur[i][0] = ids[i];
    if (e >= 0) { cur[i][1] = e ? 3 : 0; cur[i][2] = r[5]; cur[i][3] = r[6]; }
    else if (got == 0) { cur[i][1] = 1; cur[i][2] = 0; cur[i][3] = 0; }
    else { cur[i][1] = 2; cur[i][2] = (uint8_t)got; cur[i][3] = 0; }
  }
  uint8_t full[34] = {0}, ee[15] = {0};
  if (n > 0) {
    uint8_t id = ids[p_turn % n];
    read_pkt(id, 40, 31, pkt);
    int got = txn(pkt, 8, r, 37, 1500);
    if (clean(r, got, id, 31) >= 0) { full[0] = id; full[1] = 31; memcpy(full + 2, r + 5, 31); }
    if (p_turn % 8 == 0) {
      uint8_t eid = ids[(p_turn / 8) % n];
      read_pkt(eid, 5, 13, pkt);
      got = txn(pkt, 8, r, 19, 1500);
      if (clean(r, got, eid, 13) >= 0) { ee[0] = eid; ee[1] = 13; memcpy(ee + 2, r + 5, 13); }
    }
  }
  uint8_t found[2]; int nf = 0, wrapped = 0;
  for (int k = 0; k < 2; k++) {
    uint8_t id = p_scan;
    pkt[0] = 0xFF; pkt[1] = 0xFF; pkt[2] = id; pkt[3] = 2; pkt[4] = 0x01; pkt[5] = fs_sum(pkt + 2, 3);
    int got = txn(pkt, 6, r, 6, 1000);
    if (got > 0) found[nf++] = id;
    p_scan = (uint8_t)(p_scan + 1);
    if (p_scan > 253) { p_scan = 0; wrapped = 1; }
  }
  p_turn++;
  k_mutex_lock(&p_mtx, K_FOREVER);
  if (n == p_n) {
    memcpy(p_cur, cur, sizeof(cur[0]) * n);
    if (full[1]) memcpy(p_full, full, sizeof(full));
    if (ee[1]) memcpy(p_ee, ee, sizeof(ee));
  }
  for (int k = 0; k < nf && p_nfound < 64; k++) p_found[p_nfound++] = found[k];
  p_wraps += wrapped;
  p_sweeps++;
  p_seq++;
  k_mutex_unlock(&p_mtx);
}

// ---- LED matrix + LED3 (own mutex: never waits on the servo bus) -----------
Arduino_LED_Matrix matrix;
K_MUTEX_DEFINE(mx_mtx);
static const int MX_N = 13 * 8;
static uint8_t g_mx[MX_N];
static uint32_t g_mx_frames = 0;

void mx_draw(std::vector<uint8_t> frame) {     // row-major, 104 bytes, 0..7; anything else is refused
  if ((int)frame.size() != MX_N) return;
  k_mutex_lock(&mx_mtx, K_FOREVER);
  for (int i = 0; i < MX_N; i++) g_mx[i] = frame[i] > 7 ? 7 : frame[i];
  matrix.draw(g_mx);
  g_mx_frames++;
  k_mutex_unlock(&mx_mtx);
}
int mx_frames() { return (int)(g_mx_frames & 0x7FFFFFFF); }
int mx_pixel(int i) { return (i >= 0 && i < MX_N) ? (int)g_mx[i] : -1; }
int mx_shape() { return (13 << 8) | 8; }

static uint16_t g_r = 0, g_g = 0, g_b = 0;
void set_led3_color(int r, int g, int b) {     // 0..4095 each
  r = r < 0 ? 0 : (r > 4095 ? 4095 : r); g = g < 0 ? 0 : (g > 4095 ? 4095 : g); b = b < 0 ? 0 : (b > 4095 ? 4095 : b);
  analogWrite(LED3_R, r); analogWrite(LED3_G, g); analogWrite(LED3_B, b);
  g_r = r; g_g = g; g_b = b;
}
int led3_state() { return (int)((((uint32_t)g_r >> 4) << 16) | (((uint32_t)g_g >> 4) << 8) | ((uint32_t)g_b >> 4)); }

// our replacement for the library's bridge thread
K_THREAD_STACK_DEFINE(rpc_stack, 8192);
static struct k_thread rpc_thread;
static void rpc_loop(void*, void*, void*) {
  while (true) {
    if (Bridge) Bridge.update();
    k_yield();
  }
}
// free bytes left on the library's own 500-byte stack when we stopped it, and on ours now (-1 = not measurable)
static int g_old_free = -2;
int bridge_stack() {
  size_t fr = 0;
#ifdef CONFIG_INIT_STACKS
  int ours = k_thread_stack_space_get(&rpc_thread, &fr) == 0 ? (int)fr : -1;
#else
  int ours = -1;
#endif
  return (g_old_free & 0xFFFF) << 16 | (ours & 0xFFFF);
}

void setup() {
  analogWriteResolution(12);
  set_led3_color(0, 0, 0);
  matrix.begin();
  matrix.setGrayscaleBits(3);
  matrix.clear();
  Bridge.begin();
  // the servo bus first: it is what keeps the leg walking
  Bridge.provide("bus_begin", bus_begin);
  Bridge.provide("bus_send", bus_send);
  Bridge.provide("bus_rx", bus_rx);
  Bridge.provide("bus_baud", bus_baud);
  Bridge.provide("bus_junk", bus_junk);
  Bridge.provide("bus_stuck", bus_stuck);
  Bridge.provide("bus_batch", bus_batch);
  Bridge.provide("mx_draw", mx_draw);
  Bridge.provide("mx_frames", mx_frames);
  Bridge.provide("mx_pixel", mx_pixel);
  Bridge.provide("mx_shape", mx_shape);
  Bridge.provide("set_led3_color", set_led3_color);
  Bridge.provide("led3_state", led3_state);
  Bridge.provide("bridge_stack", bridge_stack);
  Bridge.provide("poll_set", poll_set);
  Bridge.provide("poll_get", poll_get);
  // which way is up (2026-10-04): the sensor is optional, so these are provided whether or not it is plugged in
  Bridge.provide("imu_scan", imu_scan);
  Bridge.provide("imu_scan_all", imu_scan_all);
  Bridge.provide("imu_begin", imu_begin);
  Bridge.provide("imu_read", imu_read);
  Bridge.provide("imu_stats", imu_stats);
  // hand RPC handling to a thread with a real stack
  k_tid_t old = Bridge.*get(UpdTid());
#ifdef CONFIG_INIT_STACKS
  size_t fr = 0;
  g_old_free = k_thread_stack_space_get(old, &fr) == 0 ? (int)fr : -1;
#else
  g_old_free = -1;
#endif
  // hold both library mutexes so the old thread is between steps and owns neither, then end it
  struct k_mutex* rm = &(Bridge.*get(ReadMx()));
  struct k_mutex* wm = &(Bridge.*get(WriteMx()));
  k_mutex_lock(rm, K_FOREVER);
  k_mutex_lock(wm, K_FOREVER);
  k_thread_abort(old);
  k_mutex_unlock(wm);
  k_mutex_unlock(rm);
  k_thread_create(&rpc_thread, rpc_stack, K_THREAD_STACK_SIZEOF(rpc_stack), rpc_loop, NULL, NULL, NULL,
                  UPDATE_THREAD_PRIORITY, 0, K_NO_WAIT);
  // The IMU gets its own thread, one priority step below the RPC thread, because every I2C call in this
  // core can wait for ever and the servo bus may not wait behind it.
  k_thread_create(&imu_thread, imu_stack, K_THREAD_STACK_SIZEOF(imu_stack), imu_loop, NULL, NULL, NULL,
                  UPDATE_THREAD_PRIORITY + 1, 0, K_NO_WAIT);
}
// the sweep runs only once the daemon has opened the bus and asked for it (poll_set); before that loop() idles.
// It only READS and PINGS: no goal, torque or EEPROM write ever comes from the MCU on its own.
void loop() {
  if (!g_baud) { delay(5); return; }
  k_mutex_lock(&p_mtx, K_FOREVER);
  bool any = p_n > 0 || p_on;
  k_mutex_unlock(&p_mtx);
  if (!any) { delay(5); return; }
  sweep();
  // one tick off the bus per sweep. The sweep holds no mutex here, but it ran back to back with nothing but a
  // k_yield, which only reaches threads of equal or higher priority; a measured poll_get waited up to 430 ms.
  k_msleep(1);
}
