# Software

A Rust daemon that owns the servo bus, and a handful of Python tools that talk
to it.

## servo-bus

One process on the board's Linux side. It is the only thing allowed to touch
the bus, so nothing can fight it for the chain.

It has no dependencies. `Cargo.toml` has an empty `[dependencies]` on purpose:
the msgpack client, the HTTP server, the WebSocket and the JSON are all in
`src/`, because every one of them sits on the latency path.

### Build and install

    rustup target add aarch64-unknown-linux-gnu
    cargo install cargo-zigbuild        # and install Zig from ziglang.org
    cd servo-bus
    cargo zigbuild --release --target aarch64-unknown-linux-gnu

    cd ..
    BOARD=arduino@<board address> ./install.sh
    ./install.sh --adb                   # or over the USB cable

Then open `http://<board address>:8938/`.

### Without hardware

    cd servo-bus && cargo run --release -- --fake --port 8938

A fake bus with a few servos on it, including two sharing ID 1 so you can see
the duplicate warning. Tests run the same way: `cargo test`, no hardware.

### The page

| address | what |
|---|---|
| `/` | the roster: every servo's angle, load, current, temperature, supply voltage, torque state, limits and reply delay, updated 50 times a second. Set ID, torque off, duplicate-ID warning |
| `/?control` | the same, plus a position slider per servo |
| `/api/bus`, `/ws/bus` | the same data as JSON, polled or streamed every 20 ms |
| `/api/batch` | one call, many servos, for an outside controller |

Nothing moves unless you ask it to. The daemon starts with torque untouched,
and the page refuses to hold a servo whose ID is shared or which reads badly.

### How it keeps the servos safe

Every servo gets an angle band and goals outside it are refused, not clamped
silently. A servo with no recorded band cannot be driven at all; its slider
stays disabled. You record a band by turning the joint by hand end to end with
torque off, which the daemon watches, then saving it. The saved band is the
recorded ends minus three degrees each side, written to the servo's EEPROM and
read back.

A jaw that rests across twenty degrees and a hip that swings two hundred have
no business sharing one global limit. That is why the band is per servo.

### Speed

On an UNO Q, fifty roster updates a second. One update costs about 10.7 ms with
two servos; the link model puts sixteen servos at 16.7 ms. The limit is the
router-to-MCU link at 115200 baud, which costs about 5.5 ms per call plus 87 µs
per byte.

## tools

Small Python scripts that talk to the daemon over HTTP. No dependencies beyond
the standard library.

| tool | what |
|---|---|
| `bus_volts.py` | every servo's supply voltage and one verdict line. Safe is 5.0 to 8.4 V |
| `record_limits.py` | walk one joint end to end and save the band |
| `limits_table.py` | the bands as a table |
| `pose.py` | hold the robot in a pose by hand, record it, then have it go there by itself |
| `zero_now.py` | read the current pose and treat it as zero |
| `fte.py` | the shared client the others use: Feetech packets through the daemon's raw door |
