# Project Instructions

## Purpose

This repository contains a Rust WebAssembly library for automated flight testing in Microsoft Flight Simulator 2020 (MSFS 2020), with primary compatibility targeting the FlyByWire Simulations A32NX.

The library must:

1. Read a configuration that identifies which parameters are injected from the input time series and which aircraft-response parameters are recorded.
2. Read a text file containing timestamped time-series data. Each parameter sample is a `(time, value)` tuple, and sampling intervals may vary.
3. Replay the complete time series in real time. Do not impose an application-defined duration or sample-count limit; supported run length is limited by available storage.
4. At each MSFS frame, interpolate continuous input parameters to the current scenario time and inject them into MSFS/A32NX.
5. Sample the configured aircraft-response parameters while the scenario runs.
6. Write the timestamped response data incrementally to another text file for later analysis.

The primary integration references are:

- FlyByWire aircraft repository: <https://github.com/flybywiresim/aircraft>
- `msfs-rs`: <https://github.com/flybywiresim/msfs-rs>

A secondary research reference is:

- `yourcontrols`: <https://github.com/Sequal32/yourcontrols>

Use `msfs-rs` for interaction with MSFS. Treat the current FlyByWire A32NX implementation as the source of truth for A32NX-specific variable names, events, units, and behavior.

Study `yourcontrols` to understand how its shared-cockpit implementation synchronizes control state, applies remote values, and bypasses or arbitrates local simulator inputs. Treat its A32NX mapping file as an accepted baseline for straightforward parameters, including the MVP signals. For complex mappings or conflicts with current behavior, prefer the current A32NX source and MSFS SDK. Always verify that the required interface is available from an `msfs-rs` WASM module; do not assume every external SimConnect technique is usable in the WASM sandbox.

## MVP Signal Scope

For the MVP, support these continuous input signals from the time series:

- sidestick pitch position;
- sidestick roll position.

Record these aircraft-response signals:

- pitch;
- roll;
- elevator position;
- aileron position.

These are logical scenario and telemetry names, not arbitrary simulator variable names. Before implementing the simulator adapter, determine and document each signal's precise semantics, A32NX/MSFS interface, engineering unit, sign convention, valid range, and update behavior. The `yourcontrols` A32NX mapping is an accepted source for these straightforward MVP mappings; cross-check current FlyByWire A32NX source when the mapping is ambiguous, unavailable through `msfs-rs`, or contradicted by current behavior.

The MVP configuration must select from this supported signal set and reject unsupported names. Keep the core data model extensible, but do not add speculative simulator mappings before this end-to-end path works.

## Scope and Compatibility

- Target MSFS 2020 and the FlyByWire A32NX first.
- Build the simulator module for the WebAssembly target supported by the selected `msfs-rs` revision, normally `wasm32-wasip1` for current upstream examples.
- Keep dependency versions and APIs compatible with the selected `msfs-rs` and A32NX revisions. Pin Git revisions when reproducibility matters.
- Do not assume stock-aircraft simulation variables and events behave identically in the A32NX. The A32NX may use local variables, named variables, custom events, or custom system logic.
- Verify A32NX interfaces against upstream source or documentation before implementing them. Do not invent variable names, event names, units, ranges, or update semantics.
- This library will be open-sourced and GPLv3 compatibility with FlyByWire and `yourcontrols` code is accepted. Their GPL-licensed code may be reused when it materially helps the implementation, provided the repository uses a compatible license and preserves all required copyright notices, attribution, source availability, license text, and modification notices. Verify the license of non-code assets and bundled third-party components separately before reuse.

## Architecture

Keep simulator-independent logic separate from MSFS bindings. Prefer modules with responsibilities similar to:

- replay configuration parsing and validation;
- time-series parsing and validation;
- playback scheduling and interpolation;
- simulator input injection;
- telemetry sampling;
- output serialization;
- lifecycle and error reporting.

The configuration parser, time-series parser, scheduler, interpolation, and serialization code should run and be testable on the host without MSFS. Keep direct `msfs-rs` calls behind a small adapter or boundary so tests can use a fake simulator implementation.

Run-time memory use must not grow with the full scenario or telemetry duration. Stream or process input and output in bounded chunks with only the lookahead required for interpolation. Do not load an arbitrarily long time series or retain all recorded samples in memory.

Prefer a library/WASM entry point appropriate for `msfs-rs` rather than a conventional long-running native `main` loop. Follow the current `msfs-rs` gauge/module lifecycle and examples when establishing entry points and Cargo crate settings.

## Real-Time Execution

- Drive playback from MSFS update events or another supported non-blocking simulator callback. Never block the simulator thread with sleeps, busy loops, or synchronous waits for the next sample.
- Drive scenario-relative playback time from elapsed simulator-clock time since the run started. Do not use wall-clock time or callback counts as the playback clock.
- For the MVP, assume MSFS is not paused and simulation rate remains `1x` throughout a run. Pause, rate changes, and simulator-time discontinuities are outside validated MVP behavior.
- Use scenario time rather than callback counts to locate the two source samples surrounding each MSFS frame time.
- Every source data point is an explicit `(time, value)` tuple. Never derive a sample timestamp from its index or assume a constant interval between samples.
- At every MSFS update, linearly interpolate each continuous injected parameter between the two timestamped source samples surrounding the current scenario time. Compute the interpolation factor from their actual timestamps so irregularly sampled input is handled correctly. Injection follows the simulator frame rate independently of source sample timing.
- Never interpolate discrete or enumerated controls; apply an explicitly documented hold/transition policy for them.
- Define exact behavior at sample boundaries, before the first sample, after the final sample, and when a frame crosses one or more source intervals. The final sample marks the end of playback unless the format explicitly defines otherwise.
- Preserve deterministic processing when frames are late or source intervals are skipped. Do not silently omit required discrete transitions.
- Keep work per simulator update bounded. Avoid allocations, repeated parsing, and file access in the hot path where practical.

## Replay Configuration

Use an explicit, documented configuration format. The configuration must define:

- the input time-series file;
- the parameters and paired time/value columns to inject;
- the aircraft-response parameters to record;
- each injected parameter's source engineering unit/range and simulator engineering unit/range;
- each recorded parameter's engineering unit and supported range;
- optional metadata needed to reproduce the test.

Do not add configuration fields for behavior fixed by the format or supported signal catalog. In particular, the MVP configuration does not contain a time unit, time origin, telemetry section, parameter type, or interpolation parameter. Document fixed time-unit, time-origin, signal-type, interpolation, telemetry sampling, and sampling-order semantics in the format specification.

For the MVP, load configuration from the hardcoded `/work/replay/config.toml` path and resolve relative input paths from `/work/replay/`. Treat this location as provisional until validated in the MSFS WASM sandbox with the selected `msfs-rs` revision. The configuration `format_version` governs both the TOML and scenario CSV contract.

Reject duplicate, unknown, unsupported, or conflicting parameter selections. Do not require every time-series column to be injected or every supported aircraft parameter to be recorded.

## Scenario Input

Use an explicit, documented text format such as CSV unless the existing code establishes another format. A scenario format must define:

- format version;
- fixed time unit and time origin;
- signal/parameter names and column order;
- value types;
- engineering units;
- how each parameter's `(time, value)` tuples are represented and associated with that parameter;
- optional metadata needed to reproduce the test.

Every data point must carry an explicit time value in scenario-relative seconds. The MVP supports no other timestamp unit. Do not require or assume constant-rate sampling.

For independently sampled series, use a standard rectangular CSV with adjacent `<signal>.time` and `<signal>.value` columns for each parameter. Values in different pairs on the same row are associated by sample ordinal only and need not have the same timestamp. Keep each pair densely populated from the first data row and permit only trailing empty pairs when series lengths differ. Require both fields of a pair to be present or both empty. Do not introduce custom blocks or mixed row types that make ordinary batch reads difficult.

Do not impose a fixed duration, row-count, or file-size limit. Design parsing and playback so scenarios can use the available disk capacity without requiring proportional RAM. A streaming preflight validation pass followed by a streaming playback pass is acceptable when supported by the MSFS WASM file APIs.

Validation must detect and report:

- malformed rows or unsupported format versions;
- missing, duplicate, reused, half-populated, or internally sparse time/value columns;
- non-finite numeric values;
- non-finite, negative, out-of-range, duplicate, or non-increasing timestamps within a parameter's series;
- arithmetic overflow when parsing timestamps or calculating durations;
- unknown parameters;
- invalid units;
- values outside safe or supported ranges.

Errors should include useful context such as the file, line, column, timestamp, or signal name. Do not silently clamp, skip, or reinterpret invalid data unless that behavior is explicitly part of the format and is reported.

## Input Injection and Safety

- Represent supported signals through an explicit mapping from scenario names to MSFS/A32NX interfaces, value types, units, and valid ranges.
- Keep conversions at the simulator boundary and test them independently.
- For each continuous injected signal, linearly interpolate in source units, then apply the configured affine range conversion from source range `[x, y]` to simulator range `[a, b]`: `a + (value - x) * (b - a) / (y - x)`.
- Require finite, strictly ordered range endpoints. Reject source values outside the configured source range and simulator ranges outside the signal catalog's safe range; do not silently clamp them.
- Do not inject arbitrary variable names directly from an untrusted scenario file.
- Before playback, verify that required simulator/A32NX interfaces can be resolved where the API permits it.
- Stop or fail safely when a required injection fails. Do not continue a test while presenting it as valid.
- Arm and start a run by setting the library-owned `L:REPLAYER_ARMED` local variable to `1`. Setting it to `0` while running requests an abort. Initialize and reset it to `0` while idle and after any terminal run state. Reject overlapping runs.
- While running, replay commands must override local pilot controls using a verified A32NX-compatible input-bypass mechanism. Do not rely on racing competing input events.
- Treat autopilot configuration as a precondition established by the operator before arming. The MVP does not engage, disengage, change, or restore autopilot modes; scenarios requiring autopilot arbitration are outside scope.
- On completion, abort, or failure, stop injection and interception, remove replay overrides, reset `L:REPLAYER_ARMED` to `0`, and give control back to the user. Do not restore prior positions or autopilot modes unless a verified restoration mechanism is deliberately added later.
- Flight-test automation can command the aircraft unexpectedly. Do not automatically start control injection merely because the module loaded.

## Telemetry Recording

Output must be a machine-readable CSV saved in the same directory as the input scenario. Generate its name from the host UTC date and time at replay start using `telemetry_YYYYMMDDTHHMMSS.csv`; do not configure or overwrite a fixed telemetry name. If the generated path already exists, fail rather than overwrite it.

- Use `time` as the first column, followed by each selected `record.N` parameter in numeric configuration order.
- For the complete MVP selection, use `time,pitch,roll,elevator_position,aileron_position`.
- Record scenario-relative simulator-clock time in seconds.
- Treat pitch and roll as aggregate MSFS aircraft attitudes and elevator/aileron positions as aggregate MSFS control-surface positions, not individual A32NX surfaces.
- Sample telemetry every MSFS frame after that frame's input injection.
- Preserve the native MSFS sign convention for all logical signals. Keep low-level event sign conversions inside the simulator adapter.
- Use stable column order and deterministic numeric formatting.
- Stream telemetry incrementally with bounded buffering so recording length is limited by available output storage rather than RAM.
- Flush at safe lifecycle points so failures do not lose the entire run, and detect/report disk-full and partial-write failures.
- Retain partial telemetry after aborts or failures under its normal timestamped name; the MVP does not mark partial files in their name or schema.
- File access in MSFS WASM may be sandboxed or restricted. Use only file-system locations and APIs supported by the MSFS SDK and `msfs-rs`; surface access failures clearly.
- Never claim a recording is complete if output creation, serialization, write, or finalization failed.

## Reliability and Error Handling

- Do not use `unwrap`, `expect`, or panics for recoverable runtime failures in simulator-facing code.
- Propagate errors with actionable context and report them through the logging/error facilities available in `msfs-rs`.
- On a terminal failure, perform best-effort control release, flush and close but do not delete partial telemetry, then return from the WASM event loop/module entry point without panicking.
- Model the run lifecycle explicitly, for example: idle, loading, ready, running, stopping, completed, aborted, and failed.
- Make start, stop, and cleanup operations idempotent where practical.
- Reject overlapping runs unless concurrency is deliberately implemented and tested.
- Favor deterministic behavior and reproducible output over implicit convenience behavior.

## Testing

Add host-side unit and integration tests for simulator-independent code. Cover at least:

- valid and malformed configuration and scenario files;
- irregular timestamp intervals, timestamp ordering, duplicates, and boundary timestamps;
- exact sample times and frame times between irregularly spaced samples;
- frames that cross multiple source intervals;
- linear interpolation of continuous values and hold/transition behavior of discrete values;
- configured injection and recording parameter selection;
- bounded-memory processing of long scenarios and telemetry streams;
- simulator-clock scheduling under the MVP `1x`, unpaused assumption;
- arming-variable start/abort transitions and release of replay control;
- unit and range conversion;
- output schema, host-UTC timestamped file naming, and numeric formatting;
- write failures and partial-run status;
- failure cleanup and panic-free module termination;
- abort and lifecycle transitions.

Use a fake simulator adapter and a controllable clock for scheduling tests. Do not require a running copy of MSFS for ordinary parser, scheduler, or serializer tests.

For simulator integration changes, document the manual validation setup: MSFS version, A32NX channel/version or commit, `msfs-rs` revision, scenario, expected behavior, and generated output location.

## Build and Validation

Use the commands supported by the repository once configured. Typical checks are:

```sh
cargo fmt --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test
sh scripts/build-wasm.sh
```

Do not report simulator or A32NX compatibility based only on a successful host build. WASM compilation verifies the target build; actual compatibility requires an in-simulator test against the stated A32NX version.

## Change Guidelines

- Keep changes focused and avoid unrelated refactoring.
- Prefer small, explicit data types over loosely typed maps in scheduling and simulator-facing code.
- Document public file formats and behavior that affects test reproducibility.
- Update tests and format documentation when changing parsing, scheduling, injection, or output behavior.
- Treat changes to timing, units, interpolation, signal mapping, sampling order, and output columns as behavior changes and call them out clearly.
- Do not add dependencies unless they work on the required WASM target and materially simplify the implementation.
