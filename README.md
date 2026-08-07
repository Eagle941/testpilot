# TestPilot

`testpilot` is a Rust WebAssembly library for automated flight testing in
Microsoft Flight Simulator 2020, initially targeting the FlyByWire A32NX.
It replays timestamped flight-control inputs at simulator frame rate and
streams the configured aircraft response to a telemetry file.

The simulator-independent core currently implements strict replay
configuration parsing, incremental per-signal scenario cursors, optional
streaming scenario validation, irregular-time linear interpolation, and affine
input-range conversion. It also provides an MSFS-compatible WASM build and a
`testpilot` gauge entry point with simulator-clock playback and calculator-code
input injection and bounded, incremental telemetry recording. Input
interception remains to be implemented.

## Repository layout

Current source layout:

- `src/` contains the crate modules:
  - `lib.rs` (crate entry, tests module wiring)
  - `config.rs`, `playback.rs`, `recording.rs`, `cursor.rs`, `replayer.rs`, `simulator.rs`,
    `gauge.rs`, `gauge_runtime.rs`, and `error.rs`.
- `src/tests/` contains helper modules used by host-side tests (`playback`, `shared`, `validation`).
- `example/` contains `replayer_config.toml` and `scenario.csv`.
- `scripts/` contains build, dev, and install helpers.

### Where to edit

- Configuration format and validation: `src/config.rs`
- Scenario cursors and interpolation: `src/cursor.rs`
- Replay scheduling and frame orchestration: `src/replayer.rs`
- MSFS simulation adapter and live loop: `src/simulator.rs`, `src/gauge.rs`, `src/gauge_runtime.rs`
- Telemetry output: `src/recording.rs`

## MVP scope

The MVP injects these continuous inputs:

- `sidestick_pitch_position`
- `sidestick_roll_position`

It records these responses:

- `pitch`
- `roll`
- `elevator_position`
- `aileron_position`

Injection and recording names are stored as arbitrary logical strings, so the
core format is not coupled to predefined signal enums. Each entry also provides
a `variable` string containing its prefixed simulator identifier, such as
`K:AXIS_ELEVATOR_SET`, `A:PLANE PITCH DEGREES`, or `L:SOME_LOCAL_VARIABLE`.
CSV columns remain logical source-data names and do not contain simulator
identifiers.

## MVP configuration

The MVP configuration format is TOML:

```toml
format_version = 1
input_file = "scenario.csv"

[inject.0]
name = "sidestick_pitch_position"
variable = "K:AXIS_ELEVATOR_SET"
source_range = [-25.0, 25.0]
simulator_range = [-16383.0, 16384.0]

[inject.1]
name = "sidestick_roll_position"
variable = "K:AXIS_AILERONS_SET"
source_range = [-25.0, 25.0]
simulator_range = [-16383.0, 16384.0]

[record.0]
name = "pitch"
variable = "A:PLANE PITCH DEGREES"
unit = "radians"
max_sampling_rate = 60.0

[record.1]
name = "roll"
variable = "A:PLANE BANK DEGREES"
unit = "radians"

[record.2]
name = "elevator_position"
variable = "A:ELEVATOR POSITION"
unit = "position"

[record.3]
name = "aileron_position"
variable = "A:AILERON POSITION"
unit = "position"
```

The repository provides this default as `example/replayer_config.toml`. The
installation script copies it to `/work/replayer_config.toml` together with the
default scenario.

`inject` and `record` section indexes are zero-based, contiguous, and define
stable processing and output-column order. Missing indexes and empty or
duplicate signal names are invalid. Each injection's CSV columns are derived
from its logical `name` as `<name>.time` and `<name>.value`; they are not
configured separately. The required `variable` field preserves its simulator
prefix so the adapter can select the appropriate `msfs-rs` interface. Each
recorded `A:` variable also requires a non-empty `unit`; units are rejected for
other recording prefixes.

For the MVP, the module reads the lowercase filename
`/work/replayer_config.toml` from the package-specific writable MSFS mount.
Relative `input_file` paths are resolved from `/work`. `format_version` governs
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
preflight pass and assumes the scenario is correctly formatted. Initialization
reads the first two samples for every cursor. Subsequent simulator frames read
forward until every cursor brackets the current scenario time or reaches EOF.
Setting the LVAR back to `0` while running has no effect in the current MVP;
operator-requested abort handling is a future requirement.

While running, replay commands take precedence over local pilot controls. The
simulator adapter must use an A32NX-compatible, verified input-bypass mechanism;
merely racing local input events is not acceptable. Autopilot configuration is
an operator precondition: the MVP does not engage, disengage, or change
autopilot modes. Scenarios requiring autopilot arbitration or mode changes are
outside MVP scope.

After the final sample, the module stops injecting, resets
`L:REPLAYER_ARMED` to `0`, and returns control to the user. It does not restore
prior control positions or autopilot modes. On a failure, it performs the same
best-effort cleanup, flushes and closes telemetry where possible, retains the
partial telemetry file under its normal timestamped name, reports the error,
and exits its WASM event loop without panicking. Operator-requested abort
handling and input interception remain to be implemented.

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

The repository's default `example/scenario.csv` demonstrates unequal series lengths.
Sidestick pitch has four samples ending at 20 seconds, while sidestick roll has
five samples ending at 40 seconds. The pitch pair is empty on the final CSV row.

For each configured signal:

- the derived `<name>.time` and `<name>.value` columns must exist exactly once;
- timestamps must be finite, non-negative, and strictly increasing;
- values must be finite and within the signal's configured `source_range`;
- the first point must be at `0` seconds.

At every MSFS frame, each signal is linearly interpolated between its two
surrounding points using their actual timestamps. When a shorter series reaches
its final sample, that value is held while the remaining series continue. The
replay completes after every configured series reaches its final sample. The interpolated source value
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

In MSFS, telemetry is saved in the package-specific writable `/work` mount. On
the validated Microsoft Store installation, this is exposed to the host under:

```text
%LOCALAPPDATA%\Packages\Microsoft.FlightSimulator_8wekyb3d8bbwe\LocalState\packages\flybywire-aircraft-a320-neo\work
```

Host-side tests save telemetry beside their input scenario. The file name is
generated from the host UTC date and time captured when the replay begins,
using the Windows-safe form `telemetry_YYYYMMDDTHHMMSS.csv`. If that exact name
already exists, the run fails rather than overwriting it.

Each configured `record.N` signal contributes an adjacent
`<signal>.time,<signal>.value` pair in numeric section order. This is the same
rectangular paired-column shape used by scenario input, so a telemetry file can
be selected directly as a later replay's input. With the complete MVP selection
the header is:

```csv
pitch.time,pitch.value,roll.time,roll.value,elevator_position.time,elevator_position.value,aileron_position.time,aileron_position.value
```

Each configured `record.N` signal can optionally set `max_sampling_rate`.
Without this field, a signal is sampled every MSFS frame after that frame's input
injection. When set to `N` hertz, that signal is sampled no more often than
once per `1 / N` scenario seconds.

Rows are only emitted when at least one configured signal is due. For a given row,
due signals include their shared elapsed timestamp in `.time` and their value in
`.value`; non-due signals emit empty cells in both columns. `pitch` and `roll` are
aggregate MSFS aircraft attitudes. `elevator_position` and `aileron_position` are
aggregate MSFS control-surface positions, not individual A32NX surfaces.

Rows are written incrementally with deterministic numeric formatting and
bounded buffering. Telemetry is flushed on completion and failure. Failures
retain the partial file under its normal timestamped name rather than deleting
or renaming it. Future abort handling must provide the same behavior.

## MSFS WASM build

The MSFS WASM module uses:

- Rust `1.93.0` with the `wasm32-wasip1` target;
- a `cdylib` artifact and release LTO/stripping;
- the target features, linker mode, and exported runtime symbols required by the
  MSFS gauge environment;
- the MSFS SDK WASI sysroot;
- `msfs-rs`.

The crate emits only a `cdylib`. Simulator-independent unit tests still run on
the host with `cargo test`.

Build and package the module natively with:

```sh
sh scripts/dev-env/run.sh ./scripts/build-wasm.sh
```

The script first runs `cargo build --release --target wasm32-wasip1`, then
post-processes the raw module with compatibility-lowering flags required by the
MSFS WASM environment.

The raw Cargo artifact remains at
`target/wasm32-wasip1/release/testpilot.wasm`. The deployable artifact is
`target/wasm32-wasip1/release/testpilot-msfs.wasm`.
Host-side `cargo test` does not link against the MSFS SDK.

A successful build and post-processing pass verifies the WASM structure and SDK
linkage, not simulator or aircraft behavior. Runtime compatibility still
requires an in-simulator test against the intended MSFS and aircraft versions.

## MSFS installation (for A32NX)

Close MSFS, then build and install the gauge:

```powershell
./scripts/install.ps1 "C:\path\to\flybywire-aircraft-a320-neo"
```

The required argument is the target Community package directory. The script
expects the current test aircraft, panel, and layout paths. On a Microsoft Store
installation, it derives the package-specific work directory from
`%LOCALAPPDATA%`. Other installations can provide it explicitly:

```powershell
./scripts/install.ps1 "C:\path\to\flybywire-aircraft-a320-neo" "C:\path\to\package\work"
```

The script performs these operations:

1. Runs `scripts/build-wasm.sh`.
2. Overwrites the aircraft panel's `testpilot.wasm` with the deployable artifact.
3. Copies `example/replayer_config.toml` and `example/scenario.csv` into the
   package-specific work directory.
4. Adds the `htmlgauge04` entry under `[VCockpit17]` if it is absent.
5. Updates or adds the `panel.cfg` and `testpilot.wasm` entries in package-root
   `layout.json`, including exact byte sizes and Windows FILETIME timestamps.

Python must be available on `PATH`. This provisional script intentionally
performs no backups, conflict checks, or rollback, and it does not launch MSFS
or modify `manifest.json`.

To validate incremental playback, run the installer, load the target aircraft,
and set `L:REPLAYER_ARMED` to `1`.
Verify the console reports the cursor count and ready message, then verify the
configured controls follow the scenario. Each converted simulator value is
written through legacy calculator code to its configured `K:` event or `L:`
variable. Verify that a timestamped telemetry CSV is created in the
package-specific `/work` mount and contains one paired time/value column set per
configured recording.

## MVP simulator mappings

The default configuration stores the following low-level simulator identifiers
in each injection's `variable` field. These interfaces are part of TestPilot's
adapter configuration and must be validated against the selected simulator and
aircraft versions.

| Logical signal | Direction | Simulator interface | Native unit/conversion |
| --- | --- | --- | --- |
| `sidestick_pitch_position` | inject | `K:AXIS_ELEVATOR_SET`; readback `L:A32NX_SIDESTICK_POSITION_Y` | configured raw axis value in `[-16383, 16384]`, written directly to the event |
| `sidestick_roll_position` | inject | `K:AXIS_AILERONS_SET`; readback `L:A32NX_SIDESTICK_POSITION_X` | configured raw axis value in `[-16383, 16384]`, written directly to the event |
| `pitch` | record | `A:PLANE PITCH DEGREES` | degrees |
| `roll` | record | `A:PLANE BANK DEGREES` | degrees |
| `elevator_position` | record | `A:ELEVATOR POSITION` | `Position 16k` |
| `aileron_position` | record | `A:AILERON POSITION` | `Position 16k` |

### External references

The following independent projects are useful sources for cross-checking MSFS
interfaces and aircraft compatibility.

- [YourControls A32NX definition](https://github.com/Sequal32/yourcontrols/blob/master/definitions/FS2020/aircraft/FlyByWire%20Simulations%20-%20Airbus%20A320-251N.yaml)
- [YourControls controls definition](https://github.com/Sequal32/yourcontrols/blob/master/definitions/FS2020/modules/controls.yaml)
- [YourControls physics definition](https://github.com/Sequal32/yourcontrols/blob/master/definitions/FS2020/modules/physics.yaml)
- [FlyByWire aircraft](https://github.com/flybywiresim/aircraft)
- [`msfs-rs`](https://github.com/flybywiresim/msfs-rs)

## TODO

- Support multiple replay configurations through
  `/work/replayer_selection.toml`. Reserve `L:REPLAYER_ARMED = 0` for idle and
  use each configured positive numeric value to select and start its associated
  configuration. Each selected configuration continues to identify its own
  scenario through `input_file`, so configuration and scenario selection cannot
  become inconsistent. No cockpit UI is required for the initial implementation.
