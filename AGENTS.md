# Project Instructions

## Scope and references

Build a Rust library/WASM gauge for automated flight testing in MSFS 2020,
initially targeting the FlyByWire A32NX. Preserve the usage and file contracts in
[README.md](README.md); use [example/](example/) as the configuration starting point.
The MVP injects continuous sidestick pitch/roll and records aircraft attitudes
and control-surface positions. Logical signal names remain strings.

Use [msfs-rs](https://github.com/flybywiresim/msfs-rs) for simulator interaction.
The current [A32NX source](https://github.com/flybywiresim/aircraft) is authoritative
for aircraft-specific interfaces. [YourControls](https://github.com/Sequal32/yourcontrols)
is an accepted baseline for straightforward mappings; resolve conflicts against
A32NX and the MSFS SDK. Verify names, units, signs, ranges and WASM API availability
before implementing mappings. Do not invent interfaces or assume external
SimConnect techniques work inside WASM. Preserve required licenses and attribution
when reusing code; GPLv3-compatible reuse is accepted.

## Architecture and execution

- Keep parsing, scheduling, interpolation and serialization simulator-independent
  and testable on the host. Keep generic simulator I/O and the clock behind a small
  adapter; put aircraft detection, commands and readback in a separate component.
- The runtime owns arming, phase transitions and cleanup, and delegates phase work
  to its contexts. Read simulator time once per frame. Process only the phase active
  at the start of an update; newly entered phases advance on the following update.
- Stream scenarios and telemetry with bounded memory and no application-defined
  duration/sample limit. Do not load entire files or retain complete recordings.
  Keep frame work non-blocking and avoid hot-path allocation where practical.
- Drive playback from simulator callbacks and elapsed simulator time, using explicit
  timestamps and irregular-interval interpolation. Never use wall-clock time or
  callback counts. Never interpolate discrete controls.
- Read simulator identifiers only from trusted configuration. Reject malformed,
  non-finite or out-of-range data instead of silently clamping. Keep physical
  aircraft limits at the aircraft boundary and validate interfaces where possible.
- Preserve explicit arming and the idle/initialising/running lifecycle. Check arming
  edges only while idle; active runs ignore arming changes. All runs pass through
  initialisation before playback. Supported aircraft must satisfy all configured
  targets before the deadline; unsupported aircraft skip setup. Only configured
  mass/balance or trim groups may be commanded and read back.
- Stop injection on completion, failure or shutdown. Attempt all cleanup, flush and
  retain partial telemetry, reset arming and return to idle even if cleanup fails.
  Retry failed arming resets before accepting another start. Keep the gauge alive
  after run failures and await event-stream closure before returning on shutdown
  or setup failure. Make cleanup idempotent where practical.
- Input override requires a verified A32NX-compatible bypass; competing input events
  do not establish control ownership. Autopilot setup remains an operator precondition.
  Do not change autopilot modes or restore prior control positions.

## Errors and validation

- Return concrete `thiserror` enums with useful file, signal, line, column and
  operation context. Use `#[from]` and `?` for direct conversions. Use `anyhow::Error`
  only at orchestration boundaries; do not use `anyhow!`, `bail!`, `Context` or
  `with_context`, or wrap an error in the same enum merely to add context.
- Do not use `unwrap`, `expect`, `todo!()` or panics for recoverable runtime failures.
  Unimplemented integration returns typed errors with TODO comments. Report errors
  through the simulator's available logging facilities.
- Use host tests with a fake simulator and controllable clock for parsing, irregular
  interpolation and boundaries, conversions, bounded streaming, telemetry, lifecycle
  and failure cleanup. Run checks appropriate to the change using the README commands.
- Simulator changes require a WASM build and documented manual validation against
  stated MSFS/A32NX versions. Host tests or compilation alone do not prove compatibility.
- Keep changes focused. Add dependencies only when they materially simplify the code
  and work on the required WASM target; pin integration revisions for reproducibility.

## Documentation

README covers purpose, setup and public data/behavior contracts. AGENTS covers
enduring development rules. Code and tests describe implementation details.
Update documentation only when those contracts, setup commands or rules change;
internal refactors and renames require no Markdown updates. Do not add inventories
of private types, fields, methods or per-change implementation history.

@RTK.md
