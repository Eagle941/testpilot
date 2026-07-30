# Replay

`replay` is a Rust WebAssembly library for automated flight testing in
Microsoft Flight Simulator 2020, initially targeting the FlyByWire A32NX.
It replays timestamped flight-control inputs at simulator frame rate and
streams the configured aircraft response to a telemetry file.

The simulator-independent core currently implements strict replay
configuration parsing, incremental per-signal scenario cursors, optional
streaming scenario validation, irregular-time linear interpolation, and affine
input-range conversion. It also provides an MSFS-compatible WASM build and a
`replayer` gauge entry point with simulator-clock playback and calculator-code
input injection. Input interception and telemetry recording remain to be
implemented.

## MVP scope

The MVP injects these continuous inputs:

- `sidestick_pitch_position`
- `sidestick_roll_position`

It records these responses:

- `pitch`
- `roll`
- `elevator_position`
- `aileron_position`

Injection names are stored as arbitrary logical strings so the core format is
not coupled to a predefined signal enum. Each injection also provides a
`variable` string containing its prefixed simulator destination, such as
`K:AXIS_ELEVATOR_SET` or `L:SOME_LOCAL_VARIABLE`. CSV columns remain logical
source-data names and do not contain simulator identifiers.

## MVP configuration

The MVP configuration format is TOML:

```toml
format_version = 1
aircraft_target = "flybywire-a32nx"
input_file = "scenario.csv"

[inject.0]
name = "sidestick_pitch_position"
variable = "K:AXIS_ELEVATOR_SET"
time_column = "sidestick_pitch_position.time"
value_column = "sidestick_pitch_position.value"
source_range = [-25.0, 25.0]
simulator_range = [-16383.0, 16384.0]

[inject.1]
name = "sidestick_roll_position"
variable = "K:AXIS_AILERONS_SET"
time_column = "sidestick_roll_position.time"
value_column = "sidestick_roll_position.value"
source_range = [-25.0, 25.0]
simulator_range = [-16383.0, 16384.0]

[record.0]
name = "pitch"
unit = "degrees"
range = [-180.0, 180.0]

[record.1]
name = "roll"
unit = "degrees"
range = [-180.0, 180.0]

[record.2]
name = "elevator_position"
unit = "position_16k"
range = [-16384.0, 16384.0]

[record.3]
name = "aileron_position"
unit = "position_16k"
range = [-16384.0, 16384.0]
```

The repository provides this default as `replayer_config.toml`. The installation
script copies it to the hardcoded package-relative configuration path.

`inject` and `record` section indexes are zero-based, contiguous, and define
stable processing and output-column order. Missing indexes, duplicate signal
names, and reused time or value columns are invalid. The required `variable`
field preserves its simulator prefix so the adapter can select the appropriate
`msfs-rs` interface; the parser stores the identifier without interpreting it.

For the MVP, the module reads the package-relative, lowercase filename
`SimObjects/AirPlanes/FlyByWire_A320_NEO/replayer_config.toml`. Relative
`input_file` paths are resolved from the same
`SimObjects/AirPlanes/FlyByWire_A320_NEO/` directory. `format_version` governs
both the TOML configuration and its scenario CSV contract.

The MVP fixes behavior that does not need to vary by configuration:

- input time zero is the instant an explicitly armed run enters the running
  state;
- both sidestick signals are continuous and use linear interpolation;
- interpolated source values are converted to simulator values using the
  configured ranges;
- telemetry is sampled every MSFS frame after that frame's interpolated inputs
  are injected;
- telemetry is written incrementally with bounded buffering.

Loading the WASM module alone must not start control injection. There is no
configured duration or row limit.

## Clock and run lifecycle

Scenario-relative time is elapsed simulator-clock time from the start of the
run. Wall-clock time and frame counts do not control playback. The MVP assumes
that MSFS is never paused during a run and that simulation rate remains `1x`;
other timing modes are outside the validated MVP behavior.

`L:REPLAYER_ARMED` is the library-owned arming variable. The module initializes
it to `0` and remains idle. Setting it to `1` loads the configuration and opens
one read-only scenario cursor per injection. The MVP skips a full-file
preflight pass and assumes the scenario is correctly formatted. Each cursor
reads at most one row per simulator frame until it has two samples. Setting the
LVAR back to `0` while running will request an abort once playback is
implemented.

While running, replay commands take precedence over local pilot controls. The
simulator adapter must use an A32NX-compatible, verified input-bypass mechanism;
merely racing local input events is not acceptable. Autopilot configuration is
an operator precondition: the MVP does not engage, disengage, or change
autopilot modes. Scenarios requiring autopilot arbitration or mode changes are
outside MVP scope.

After the final sample or an abort request, the module stops injecting and
intercepting controls, removes its replay overrides, resets
`L:REPLAYER_ARMED` to `0`, and returns control to the user. It does not restore
prior control positions or autopilot modes. On a failure, it performs the same
best-effort control release, flushes and closes telemetry where possible,
retains the partial telemetry file under its normal timestamped name, reports
the error, and exits its WASM event loop without panicking. Aborted runs also
retain their partial, unmarked telemetry file.

## Scenario CSV

The MVP uses a rectangular paired-column layout. Every injected signal has an
adjacent time column and value column, and each populated pair is one explicit
`(time, value)` sample. A row associates samples by their ordinal position
within each signal; samples on the same row do not need to have the same time.

```csv
sidestick_pitch_position.time,sidestick_pitch_position.value,sidestick_roll_position.time,sidestick_roll_position.value
0.000,0.000,0.000,0.000
0.150,10.000,0.200,5.000
0.425,-5.000,0.700,0.000
0.700,0.000,,
```

Within each signal's column pair, samples are stored densely from the first
data row. If one signal has fewer samples, only trailing pairs are empty. A
time and value must either both be present or both be empty; interior gaps and
half-populated pairs are invalid.

All `.time` column values are scenario-relative seconds. The MVP does not
support another timestamp unit.

This remains ordinary CSV with one header and homogeneous numeric columns, so
batch tools can read the complete table directly. For example, a tool can
select one signal's two columns, drop trailing empty rows, and obtain an
`N × 2` array without parsing custom blocks or mixed record types.

The repository's default `scenario.csv` contains 81 samples at 10 Hz over 8
seconds. It uses a 0.25 Hz sine wave with ±25% amplitude: sidestick pitch
completes one cycle from 0 to 4 seconds while roll remains neutral, then roll
completes one cycle from 4 to 8 seconds while pitch remains neutral.

For each configured signal:

- configured time and value columns must exist exactly once;
- timestamps must be finite, non-negative, and strictly increasing;
- values must be finite and within the signal's configured `source_range`;
- the first point must be at `0` seconds;
- all injected signals must have the same final timestamp for the MVP.

At every MSFS frame, each signal is linearly interpolated between its two
surrounding points using their actual timestamps. The interpolated source value
`v` in `source_range = [x, y]` is then converted to the simulator range
`[a, b]` with:

```text
simulator_value = a + (v - x) * (b - a) / (y - x)
```

All range endpoints must be finite and each lower endpoint must be less than
its upper endpoint. For the MVP axis events, `simulator_range` must remain
within `[-16383, 16384]` and expresses the raw value written to MSFS. Invalid or
out-of-range values are rejected, not clamped. Interpolation is performed before
conversion so scenario values and validation remain in the configured source
scale. The simulator adapter writes the converted value directly without
additional scaling or sign conversion.

The final point is injected exactly, then the scenario completes. Source files
are streamed with bounded lookahead so duration is limited by available
storage rather than RAM.

## Telemetry CSV

Telemetry is saved beside the input scenario. Its file name is generated from
the host UTC date and time captured when the replay begins, using the
Windows-safe form `telemetry_YYYYMMDDTHHMMSS.csv`. If that exact name already
exists, the run fails rather than overwriting it. The first column is `time`, followed
by each configured `record.N` signal in numeric section order. With the complete
MVP selection the header is:

```csv
time,pitch,roll,elevator_position,aileron_position
```

`time` is scenario-relative simulator-clock time in seconds.
`pitch` and `roll` are aggregate MSFS aircraft attitudes; `elevator_position`
and `aileron_position` are aggregate MSFS control-surface positions, not
individual left/right A32NX surfaces. A row is sampled after input injection on
every MSFS frame and is streamed incrementally with deterministic numeric
formatting and bounded buffering.

## MSFS WASM build

The build configuration is aligned with the local FlyByWire aircraft repository
at `C:\Users\Giuseppe\source\repos\a32nx`, branch `fs2020-master`, commit
`81461a72be047a9e91e1b1d647ef01cae86565ad`. In particular, this project uses:

- Rust `1.93.0` with the `wasm32-wasip1` target;
- a `cdylib` artifact and release LTO/stripping;
- the A32NX WASM target features, linker mode, and exported runtime symbols;
- the locally installed MSFS SDK WASI sysroot at
  `C:\MSFS SDK\WASM\wasi-sysroot`;
- `msfs-rs` from its `main` branch, pinned by `Cargo.lock` to
  `2f697b9aac9fa3c00474f901a7f7ee4218cf534b`.

The crate emits only a `cdylib`, matching the A32NX Rust gauge build. Adding an
`rlib` crate type to the same WASM build prevents the required dead-code
elimination and retains unsupported SDK imports, causing MSFS module
instantiation to fail. Simulator-independent unit tests still run on the host
with `cargo test`.

The current machine has the MSFS SDK installed at `C:\MSFS SDK`, so the linker
paths are configured directly in `.cargo/config.toml`; no `build.rs` or Docker
container is required. Developers with the SDK in another location must update
the two SDK paths in that file.

Build and package the module natively with:

```sh
sh scripts/build-wasm.sh
```

The script first runs `cargo build --release --target wasm32-wasip1`, then
post-processes the raw module with the same A32NX systems-library flags:

```sh
wasm-opt -O1 --signext-lowering --enable-bulk-memory \
  --enable-nontrapping-float-to-int
```

The raw Cargo artifact remains at
`target/wasm32-wasip1/release/replay.wasm`. The deployable MVP artifact is
`target/wasm32-wasip1/release/replay-msfs.wasm`. `wasm-opt` must be available
on `PATH`. Host-side `cargo test` does not link against the MSFS SDK.

A successful build and post-processing pass verifies the WASM structure, SDK
linkage, and A32NX-compatible lowering, not A32NX behavior. Compatibility still
requires an in-simulator test against the referenced A32NX branch/version.

## A32NX smoke-test installation

Close MSFS, then build and install the current gauge into the local A32NX
Community package from Git Bash:

```sh
sh scripts/install.sh
```

The script uses the hardcoded package path
`D:\MSFS\Packages\Community\flybywire-aircraft-a320-neo` and performs these
operations:

1. Runs `scripts/build-wasm.sh`.
2. Overwrites the aircraft panel's `replay.wasm` with the deployable artifact.
3. Copies the repository's `replayer_config.toml` and minimal `scenario.csv`
   into the aircraft directory.
4. Adds the `htmlgauge04` entry under `[VCockpit17]` if it is absent.
5. Updates or adds the configuration, scenario, `panel.cfg`, and `replay.wasm` entries in
   package-root `layout.json`, including exact byte sizes and Windows FILETIME
   timestamps.

The operation is idempotent for the expected A32NX package structure: rerunning
it replaces the module and refreshes the same gauge and layout entries. Python
must be available on `PATH`. This provisional script intentionally performs no
backups, conflict checks, or rollback, and it does not launch MSFS or modify
`manifest.json`.

The current smoke test initializes `L:REPLAYER_ARMED` to `0` and reads it on
every MSFS `PreUpdate` event. The simulator-independent `ArmState` struct owns
the previous sample, and its `start` method returns `true` only when the value
changes from exactly `0` to exactly `1`. On that transition, the gauge reads
`SimObjects/AirPlanes/FlyByWire_A320_NEO/replayer_config.toml`, opens one
independent `scenario.csv` reader per injection, and logs the cursor count. The
arm frame reads only each reader's CSV header. Each subsequent `PreUpdate`
consumes at most one data row per cursor until all cursors have two samples,
then logs `REPLAYER: scenario cursors ready` and captures
`E:SIMULATION TIME` as scenario time zero. On every subsequent `PreUpdate`, each
cursor advances by at most one row when elapsed scenario time passes its next
sample timestamp. The gauge prints elapsed simulator seconds, each injection's
logical name and simulator variable, the previous and next `(time, value)` rows,
the interpolated source value, and its affine-converted simulator value. A file
or parsing failure resets
the armed LVAR to `0` and terminates the gauge task without panicking. No
full-file scenario validation runs in the MVP, and disarm transitions have no
behavior.

To validate incremental loading, run the installer, load the A32NX, and set
`L:REPLAYER_ARMED` to `1` with an LVAR or calculator-code tool. Verify the
console reports the cursor count followed by elapsed simulator seconds, the
ready message, and one time-varying interpolation-row pair per configured
injection on every frame. Each converted simulator value is written through
legacy calculator code to its configured `K:` event or `L:` variable before it
is logged. Telemetry recording is not yet implemented.

## A32NX MVP mappings

These adapter mappings are based on the YourControls FS2020 A32NX definition,
version `0.12.3`, from repository tree
`4e32af561a82f1f998fbe4b0b0db0efe2642cdf2`. The default configuration stores
these low-level identifiers in each injection's `variable` field.

| Logical signal | Direction | MSFS/A32NX interface | Native unit/conversion |
| --- | --- | --- | --- |
| `sidestick_pitch_position` | inject | `K:AXIS_ELEVATOR_SET`; readback `L:A32NX_SIDESTICK_POSITION_Y` | configured raw axis value in `[-16383, 16384]`, written directly to the event |
| `sidestick_roll_position` | inject | `K:AXIS_AILERONS_SET`; readback `L:A32NX_SIDESTICK_POSITION_X` | configured raw axis value in `[-16383, 16384]`, written directly to the event |
| `pitch` | record | `A:PLANE PITCH DEGREES` | degrees |
| `roll` | record | `A:PLANE BANK DEGREES` | degrees |
| `elevator_position` | record | `A:ELEVATOR POSITION` | `Position 16k` |
| `aileron_position` | record | `A:AILERON POSITION` | `Position 16k` |

References:

- [YourControls A32NX definition](https://github.com/Sequal32/yourcontrols/blob/master/definitions/FS2020/aircraft/FlyByWire%20Simulations%20-%20Airbus%20A320-251N.yaml)
- [YourControls controls definition](https://github.com/Sequal32/yourcontrols/blob/master/definitions/FS2020/modules/controls.yaml)
- [YourControls physics definition](https://github.com/Sequal32/yourcontrols/blob/master/definitions/FS2020/modules/physics.yaml)
- [FlyByWire aircraft](https://github.com/flybywiresim/aircraft)
- [`msfs-rs`](https://github.com/flybywiresim/msfs-rs)

The YourControls mappings are the accepted baseline for these simple MVP
signals. The simulator adapter must still confirm that each event and variable
is accessible from the selected `msfs-rs` WASM revision. Any observed mismatch
with the current A32NX must be documented and resolved before claiming
in-simulator compatibility.
