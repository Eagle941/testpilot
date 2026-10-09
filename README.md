# TestPilot

Rust library and WebAssembly gauge for automated flight testing in Microsoft
Flight Simulator 2020, targeting the FlyByWire A32NX. It streams timestamped
control inputs into the simulator and records aircraft responses to CSV.

The MVP injects sidestick pitch and roll and records pitch, roll, elevator and
aileron position. Logical signal names remain configurable strings. Input
interception and operator-requested aborts are not implemented; autopilot setup
is the operator's responsibility. Playback assumes an unpaused simulator at `1x`.

## Run in MSFS

Close MSFS and install into the A32NX Community package:

```powershell
.\scripts\install.ps1 "C:\path\to\flybywire-aircraft-a320-neo"
```

The installer requires Docker and Python on `PATH`. It builds the gauge, copies
[the example configuration](example/replayer_config.toml) and
[scenario](example/scenario.csv), and updates the aircraft panel and layout.
It overwrites files without backups. Supply `-WorkPath "C:\path\to\work"` when
the default Microsoft Store work directory is unsuitable.

Configuration is loaded from `/work/replayer_config.toml` on every arm; relative
input paths resolve from `/work`. On Microsoft Store installations, that mount is:

```text
%LOCALAPPDATA%\Packages\Microsoft.FlightSimulator_8wekyb3d8bbwe\LocalState\packages\flybywire-aircraft-a320-neo\work
```

Load the aircraft, then set `L:REPLAYER_ARMED` from `0` to `1`. Loading the gauge
alone never starts injection. Every run passes through initialisation; each
update processes its starting phase, and playback time zero is established on
the first running update. Time advances with the simulator clock.

Optional aircraft setup is skipped for unsupported or unidentified aircraft.
For supported aircraft, all requested targets must become ready within 30
simulator seconds of arming or the run fails. Inclusive tolerances are 100 kg for
ZFW/GW, 0.01 percentage points for GWCG and 0.01 degrees for THS; timeout takes
precedence over readiness. Waiting produces no replay inputs or telemetry.

Arming changes during a run are ignored. Completion, failure and shutdown stop
injection, flush telemetry and reset arming to `0`. Failures retain partial output
and return to idle; a failed arming reset is retried before accepting another run.

## Configuration

The [example TOML](example/replayer_config.toml) is the starting point.
`format_version = 1` governs both configuration and scenario CSV.

- `input_file` selects the scenario.
- `inject.N` requires `name`, prefixed simulator `variable`, `source_range` and
  `simulator_range`. At least one injection is required.
- Optional `record.N` entries require `name` and prefixed `variable`. `A:` variables
  require a `unit`; other prefixes reject it. A positive `max_sampling_rate` in Hz
  may limit sampling; otherwise sampling occurs every simulator frame.

Indexes start at zero, are contiguous and determine processing/output order.
Logical names must be unique; simulator identifiers come from configuration,
never from CSV data. All range endpoints must be finite and strictly ordered.
Source samples outside their configured range are rejected without clamping.
Simulator ranges must respect the selected interface's valid limits; the MVP
axis events use raw values in `[-16383, 16384]`.

Optional `[initialisation]` accepts `zfw` and `gw` in kilograms and `gwcg` in percent
MAC, all three together or all absent. Values must be finite, masses positive and
`gw >= zfw`. Independently optional `ths` is in degrees, finite and within
`[-4, 13.5]`. Unknown fields and partial mass/balance groups are rejected. An empty
table behaves as an omitted section. Only requested groups are commanded and
checked; actual loading limits are enforced by the aircraft integration.

## Scenario CSV

Use one header with adjacent `<name>.time,<name>.value` columns for each signal,
as in [the example](example/scenario.csv). Each pair represents explicit samples
in scenario-relative seconds, starting at zero with finite, strictly increasing
timestamps. Signals may have different timestamps and lengths on the same row.
Pairs must be populated from the first row, with only trailing empty pairs; a
time and value must both be present or both empty. Values must be finite and
within `source_range`.

Continuous inputs are linearly interpolated using their actual timestamps, then
converted from source range `[x, y]` to simulator range `[a, b]`:

```text
simulator_value = a + (value - x) * (b - a) / (y - x)
```

Late frames consume samples until they bracket the current scenario time. Shorter
series hold their last value while other series continue. Playback ends after all
series finish. Input and output are streamed with bounded memory; no duration,
row-count or file-size limit is imposed by the application.

## Telemetry

Output is written to `/work/telemetry_YYYYMMDDTHHMMSS.csv`, using host UTC captured
when entering playback, before the first running update. Existing files are never
overwritten. Host tests write beside their scenario.

Columns are adjacent `<name>.time,<name>.value` pairs: recordings first, then
injections, each in numeric configuration order. Samples use scenario-relative
seconds and are collected after input injection. Injection columns contain the
converted simulator values actually applied.

With sampling limits, rows are emitted when at least one recording is due;
non-due recordings have empty pairs. Injection pairs appear on every emitted row.
With no recordings, output contains injection pairs on every playback frame.
Numeric formatting is deterministic and buffering is bounded. Write and flush
failures are reported; partial files keep their normal timestamped names.

## Development

Keep configuration, streaming, interpolation and serialization usable on the
host. Isolate generic simulator I/O from aircraft-specific setup, and keep
lifecycle orchestration separate from phase work. See [AGENTS.md](AGENTS.md) for
contributor rules and [CHALLENGES.md](CHALLENGES.md) for known engineering limits.

```sh
cargo fmt --check
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo test --locked
```

Build the MSFS module with the repository's Docker environment:

```powershell
.\scripts\dev-env\run.cmd ./scripts/build-wasm.sh
```

On POSIX shells, use `sh scripts/dev-env/run.sh ./scripts/build-wasm.sh`.
The deployable artifact is `target/wasm32-wasip1/release/testpilot-msfs.wasm`.
Host tests and WASM compilation do not establish simulator compatibility. Manual
validation must record the MSFS build, A32NX version, locked `msfs-rs` revision,
scenario and output location, and check playback, initialisation and failure/reload
cleanup.

Interface references: [FlyByWire aircraft](https://github.com/flybywiresim/aircraft),
[msfs-rs](https://github.com/flybywiresim/msfs-rs) and
[YourControls](https://github.com/Sequal32/yourcontrols).
Future work includes selectable replay configurations.

Keep documentation focused on usage, public file contracts and enduring rules.
Internal renames and refactors do not require documentation updates.
