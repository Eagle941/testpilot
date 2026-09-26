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
    `gauge.rs`, `gauge_runtime.rs`, `initialisation.rs`, `aircraft_initialisation.rs`, and `error.rs`.
- `src/tests/` contains helper modules used by host-side tests (`playback`, `shared`, `validation`).
- `example/` contains `replayer_config.toml` and `scenario.csv`.
- `scripts/` contains build, dev, and install helpers.

### Where to edit

- Configuration format and validation: `src/config.rs`
- Scenario cursors and interpolation: `src/cursor.rs`
- Replay scheduling and frame orchestration: `src/replayer.rs`
- MSFS simulation adapter and live loop: `src/simulator.rs`, `src/gauge.rs`, `src/gauge_runtime.rs`
- Telemetry output: `src/recording.rs`

`GaugeRuntime` coordinates arming, aircraft detection, initialisation and playback start.
`Replayer` prepares scenario cursors, starts playback explicitly, and advances
frames using a supplied simulator timestamp; it does not receive a simulator
adapter. `SimulatorAdapter` provides only low-level reads, writes, validation and
the clock, including a generic aircraft-string read operation. The separate
`A32nxInitialiser` component owns aircraft detection, loading submission and
mass/balance readback behind the small `AircraftInitialiser` interface, allowing
runtime tests to substitute it independently. The runtime has one explicit
constructor taking the replayer, a `Box<dyn SimulatorAdapter>` and a
`Box<dyn AircraftInitialiser>`. Neither `GaugeRuntime` nor `AircraftInitialiser`
has generic type parameters; initialisation methods receive a
`&mut dyn SimulatorAdapter`. Tests retain shared handles to fake state for clock
control and operation assertions. `Initialisation` contains only
the simulator-independent tolerance and deadline checks.

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

An optional section can demand actual aircraft mass and balance before replay:

```toml
[initialisation]
zfw = 60000.0 # zero-fuel weight, kg
gw = 65000.0  # gross weight, kg
gwcg = 25.0  # gross-weight centre of gravity, percent MAC
```

All three fields are required when the section is present. Values must be numeric
and finite; masses must be positive and `gw >= zfw`. Unknown fields are rejected.
Aircraft-specific loading limits will be validated by the aircraft initialisation component.
This optional addition uses `format_version = 1`; omitting the section retains
immediate playback on arming. No timeout, tolerance or unit fields are configurable.

**Simulator initialisation is not implemented yet.** Aircraft detection is implemented;
the A32NX component's submission
and readback operations contain TODO comments and return typed errors rather
than panicking. Enabling this section on a detected A32NX currently fails safely
on arming, even if the aircraft already meets the targets. Unsupported or
unidentified aircraft log that initialisation was skipped and start replay immediately,
without enforcing mass/CG targets. The readiness gate is implemented and
tested with a fake simulator. Future integration must change and read actual
aircraft loading/fuel/balance, not merely flight-management entries.

Aircraft support is checked once on each arm frame requesting initialisation,
using `(A:ATC MODEL, string)` and the existence of `L:A32NX_IS_READY`.
The supported model is `A20N`; the configured
localisation key `TT:ATCCOM.AC_MODEL_A20N.0.text` is also accepted. Comparisons
ignore case and surrounding whitespace. `A32nxInitialiser` owns both checks.
For a matching model, the adapter calls `check_named_variable` with the unprefixed
name `A32NX_IS_READY`. It does not register the variable or read its value: an
existing variable with value zero still counts as supported. Detection ignores
livery titles. This is a practical interface check, not guaranteed package identity.

Detection has only `Supported` and `Unsupported` outcomes. Other models,
unavailable, empty, or invalid string results, a missing readiness variable, or
a failed existence lookup take the unsupported path and do
not stop the run. No aircraft detection is performed when `[initialisation]` is
absent. Configured replay input/recording errors and failures after supported
initialisation begins retain their existing terminal error handling.

Detection references (source verification does not establish in-simulator compatibility):

- The SDK's [`check_named_variable`](https://docs.flightsimulator.com/html/Programming_Tools/WASM/Gauge_API/check_named_variable.htm)
  returns an existing local variable ID or `-1`, without creating a variable.
  The [A32NX flight-control module](https://github.com/flybywiresim/aircraft/blob/2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2/fbw-a32nx/src/wasm/fbw_a320/src/FlyByWireInterface.cpp#L303)
  registers `A32NX_IS_READY`; its value is not needed for identification.
- The MSFS 2020 SDK documents [`ATC MODEL` as a string](https://docs.flightsimulator.com/html/Programming_Tools/SimVars/Aircraft_SimVars/Aircraft_RadioNavigation_Variables.htm)
  and [string results from the WASM calculator API](https://docs.flightsimulator.com/html/Programming_Tools/WASM/Gauge_API/execute_calculator_code.htm).
- FlyByWire's [base aircraft configuration](https://github.com/flybywiresim/aircraft/blob/2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2/fbw-a32nx/src/base/flybywire-aircraft-a320-neo/SimObjects/AirPlanes/FlyByWire_A320_NEO/aircraft.cfg)
  sets `atc_model` to the A20N localisation key. A [developer report](https://devsupport.flightsimulator.com/t/untranslated-values-returned-by-simconnect/3537)
  describes MSFS returning the raw key through SimConnect; the matcher accepts
  both that key and the A20N model code. Actual WASM readback remains to be
  confirmed by the manual validation below.
- The locked [`msfs-rs` revision](https://github.com/flybywiresim/msfs-rs/tree/2f697b9aac9fa3c00474f901a7f7ee4218cf534b)
  exposes this API through `msfs::sys`. The adapter checks the returned pointer and
  UTF-8 before copying the string, allowing detection failures to skip safely.

`inject` and `record` section indexes are zero-based, contiguous, and define
stable processing and output-column order. Missing indexes and empty or
duplicate signal names are invalid. Each injection's CSV columns are derived
from its logical `name` as `<name>.time` and `<name>.value`; they are not
configured separately. The required `variable` field preserves its simulator
prefix so the adapter can select the appropriate `msfs-rs` interface. Each
recorded `A:` variable also requires a non-empty `unit`; units are rejected for
other recording prefixes.

At least one `inject.N` entry is required. Recordings are optional: omit all
`record.N` entries or use an empty `[record]` table to replay inputs alone.

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
When `[initialisation]` is present and aircraft detection reports support, the
arm frame submits the targets once and enters an initialising state. Each simulator frame checks actual aircraft mass
and balance. Playback starts immediately on the first frame where all three
values simultaneously meet these inclusive absolute tolerances:

- ZFW: within 100 kg of its target;
- GW: within 100 kg of its target;
- GWCG: within 0.01 percentage points of MAC (25.00 accepts 24.99 through 25.01).

There is no dwell period or continuing readiness check once replay starts.
The deadline is 30 elapsed simulator seconds from the arm frame, under the
unpaused, 1x assumption. At or after 30 seconds, timeout takes precedence over
readiness and reports the targets and latest available snapshot. Backwards
simulator time, non-finite readback, and adapter failures terminate the run.
Waiting does not inject replay controls, advance scenario cursors, or create
telemetry. On readiness, the module creates telemetry using the current host
UTC timestamp, starts scenario time at zero, and injects then samples that frame.
Overlapping starts are rejected while initialising as well as while running.

Setting the LVAR back to `0` while initialising or running has no effect in the current MVP;
operator-requested abort handling is a future requirement.

While running, replay commands take precedence over local pilot controls. The
simulator adapter must use an A32NX-compatible, verified input-bypass mechanism;
merely racing local input events is not acceptable. Autopilot configuration is
an operator precondition: the MVP does not engage, disengage, or change
autopilot modes. Scenarios requiring autopilot arbitration or mode changes are
outside MVP scope.

After the final sample, the module stops injecting, resets
`L:REPLAYER_ARMED` and its edge detector to `0`, and returns control to the user.
A new arm can be recognised on the next frame without an intervening idle frame.
It does not restore
prior control positions or autopilot modes. On a failure, it performs the same
best-effort cleanup, flushes and closes telemetry where possible, retains the
partial telemetry file under its normal timestamped name, and reports the error.
Terminal failures stop processing further updates, including new arming requests;
the gauge awaits event-stream closure before returning without panicking, including
when the initial arming reset fails during gauge setup. Reload
the gauge before another attempt. Initialisation failures create no telemetry file.
Operator-requested abort
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

Each configured `record.N` and `inject.N` signal contributes an adjacent
`<signal>.time,<signal>.value` pair in numeric section order. This is the same
rectangular paired-column shape used by scenario input, so a telemetry file can
be selected directly as a later replay's input. With the complete MVP selection
the header is:

```csv
pitch.time,pitch.value,roll.time,roll.value,elevator_position.time,elevator_position.value,aileron_position.time,aileron_position.value,sidestick_pitch_position.time,sidestick_pitch_position.value,sidestick_roll_position.time,sidestick_roll_position.value
```

Each configured `record.N` signal can optionally set `max_sampling_rate`.
Without this field, a signal is sampled every MSFS frame after that frame's input
injection. When set to `N` hertz, that signal is sampled no more often than
once per `1 / N` scenario seconds.

When recordings are configured, rows are only emitted when at least one recording
signal is due. For a given row, due signals include their shared elapsed timestamp
in `.time` and their
value in `.value`; non-due recording signals emit empty cells in both columns.
Injection columns are always written on emitted rows using the injected simulator
values for that frame (after interpolation and conversion).

With no recordings configured, the timestamped telemetry file contains only
injection columns and a row for every replay frame.

`pitch` and `roll` are aggregate MSFS aircraft attitudes. `elevator_position`
and `aileron_position` are aggregate MSFS control-surface positions, not
individual A32NX surfaces.

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
configured recording and one paired time/value column set per configured injection.

### Gauge reload validation

On `PreKill`, the gauge stops replay and performs best-effort telemetry flushing
and arming reset. It ignores further playback updates while awaiting event-stream
closure on `PostKill`, then performs final best-effort cleanup and returns. This
keeps `msfs-rs` from polling an already completed async gauge during reload.

Manual validation (requires MSFS; not established by host tests):

- Record the MSFS 2020 build and A32NX channel/version or commit used, together
  with the locked `msfs-rs` revision
  `2f697b9aac9fa3c00474f901a7f7ee4218cf534b`.
- Install the packaged WASM and use `example/replayer_config.toml` with
  `example/scenario.csv`. Check the initial flight load and several aircraft
  reloads: each should reach the arming wait message without a WASM exception.
- Arm a replay and reload before it finishes. Verify injection stops,
  `L:REPLAYER_ARMED` resets to `0`, and the partial telemetry CSV remains readable
  with its buffered rows flushed. The new instance must wait for a fresh arm.
- Inspect `telemetry_YYYYMMDDTHHMMSS.csv` in the package-specific `/work` mount.
  On Microsoft Store installations this is
  `%LOCALAPPDATA%\Packages\Microsoft.FlightSimulator_8wekyb3d8bbwe\LocalState\packages\flybywire-aircraft-a320-neo\work`.

### Initialisation validation

Record the MSFS 2020 build, A32NX channel/version or commit, and locked `msfs-rs`
revision with each manual check. With the shipped example's initialisation section
commented out, verify the existing scenario still starts on arming and records in
the `/work` location above. Then enable the example's three targets and reload
using the A32NX. Record the `ATC MODEL` string, and repeat with a custom livery
whose title differs. With either accepted A20N model string and `A32NX_IS_READY`
registered (whether zero or one), arming must report the
unimplemented submission error, reset `L:REPLAYER_ARMED`
to `0`, and produce no replay control writes or telemetry file. Further arming
must not restart the failed gauge; reload it for another attempt.

With an aircraft reporting a different model, such as C172,
use an input/recording configuration valid for that aircraft and arm with
`[initialisation]` enabled. Verify the console reports initialisation skipped,
playback starts on that frame at time zero, and normal telemetry is created in
`/work`. Verify another arm detects the aircraft again. An unreadable model takes
the same path; host tests exercise that failure without requiring MSFS.
Also check an A20N aircraft without `A32NX_IS_READY`: initialisation must be
skipped and the existence check must leave that variable absent. A failed lookup
has the same skip behavior, covered by host tests.

Successful loading, actual-aircraft convergence, timeout and control release
require manual validation after implementing verified A32NX initialisation mappings.
Host tests and WASM compilation do not establish that simulator compatibility.

## MVP simulator mappings

The default configuration stores the following low-level simulator identifiers
in each injection's `variable` field. These interfaces are part of TestPilot's
adapter configuration and must be validated against the selected simulator and
aircraft versions.

| Logical signal | Direction | Simulator interface | Native unit/conversion |
| --- | --- | --- | --- |
| `sidestick_pitch_position` | inject | `K:AXIS_ELEVATOR_SET` | configured raw axis value in `[-16383, 16384]`, written directly to the event |
| `sidestick_roll_position` | inject | `K:AXIS_AILERONS_SET` | configured raw axis value in `[-16383, 16384]`, written directly to the event |
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
