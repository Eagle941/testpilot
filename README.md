# Replay

`replay` is a Rust WebAssembly library for automated flight testing in
Microsoft Flight Simulator 2020, initially targeting the FlyByWire A32NX.
It replays timestamped flight-control inputs at simulator frame rate and
streams the configured aircraft response to a telemetry file.

The project is currently at the specification stage. The configuration and
file formats below define the intended MVP contract; their parsers are not yet
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

Only these logical names are accepted by the MVP. Configuration files cannot
supply arbitrary simulator variables or events.

## MVP configuration

The MVP configuration format is TOML:

```toml
format_version = 1
aircraft_target = "flybywire-a32nx"
input_file = "scenario.csv"

[inject.0]
name = "sidestick_pitch_position"
time_column = "sidestick_pitch_position.time"
value_column = "sidestick_pitch_position.value"
source_unit = "percent"
source_range = [-100.0, 100.0]
simulator_unit = "normalized"
simulator_range = [-1.0, 1.0]

[inject.1]
name = "sidestick_roll_position"
time_column = "sidestick_roll_position.time"
value_column = "sidestick_roll_position.value"
source_unit = "percent"
source_range = [-100.0, 100.0]
simulator_unit = "normalized"
simulator_range = [-1.0, 1.0]

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

`inject` and `record` section indexes are zero-based, contiguous, and define
stable processing and output-column order. Missing indexes, duplicate signal
names, and reused time or value columns are invalid.

For the MVP, the module reads `/work/replay/config.toml`. Relative
`input_file` paths are resolved from `/work/replay/`. This hardcoded location
is provisional and must be validated in MSFS with the selected `msfs-rs`
revision before compatibility is claimed. `format_version` governs both the
TOML configuration and its scenario CSV contract.

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
it to `0` and remains idle. Setting it to `1` loads and validates the configured
scenario, then starts the run. Setting it back to `0` while running requests an
abort. The module rejects an overlapping run and resets the variable to `0`
when the run ends.

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
its upper endpoint. The simulator range must remain within the safe range for
the supported logical signal. Invalid or out-of-range values are rejected, not
clamped. Interpolation is performed before conversion so scenario values and
validation remain in the documented source unit. Logical source signs are
consistent with the corresponding MSFS/A32NX output signs; low-level event sign
or axis conversions are handled only by the simulator adapter.

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

## A32NX MVP mappings

These adapter mappings are based on the YourControls FS2020 A32NX definition,
version `0.12.3`, from repository tree
`4e32af561a82f1f998fbe4b0b0db0efe2642cdf2`. The logical configuration never
contains these low-level identifiers.

| Logical signal | Direction | MSFS/A32NX interface | Native unit/conversion |
| --- | --- | --- | --- |
| `sidestick_pitch_position` | inject | `K:AXIS_ELEVATOR_SET`; readback `L:A32NX_SIDESTICK_POSITION_Y` | normalized `[-1, 1]`; event value = converted simulator value × `-16384` |
| `sidestick_roll_position` | inject | `K:AXIS_AILERONS_SET`; readback `L:A32NX_SIDESTICK_POSITION_X` | normalized `[-1, 1]`; event value = converted simulator value × `-16384` |
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
