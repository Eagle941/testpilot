# Repeatable airborne test initialisation

## Objective

Return the A32NX to a configured airborne condition before each replay, allowing
time for preparation and verifying stable flight before the test starts.

Repositioning does not restore all internal aircraft-system state. The aim is
comparable, measured starting conditions.

## Agreed scope

- Target steady, wings-level flight.
- Read the target condition from configuration.
- Freeze position, altitude and attitude during preparation.
- Hold the configured indicated airspeed (IAS) throughout the frozen period.
- Release automatically after a configured delay.
- Verify stability after release before starting replay.
- Leave fuel, weight, centre of gravity, aircraft configuration, weather, thrust,
  trim and automation modes to the operator.
- Do not implement automatic stabilisation or captured-state restoration yet.

## Proposed sequence

Arm → freeze → reposition → hold → release → verify → replay at time zero.

Preparation runs through non-blocking simulator callbacks using simulator time.
Scenario playback and its telemetry begin only after verification succeeds.

The aircraft may travel during verification. The configured position is the
reposition point; log the actual condition at replay start.

## Simulator interfaces

Use the explicit events:

- `FREEZE_LATITUDE_LONGITUDE_SET`
- `FREEZE_ALTITUDE_SET`
- `FREEZE_ATTITUDE_SET`

Confirm their state through the corresponding `IS … FREEZE ON` variables.

Use `msfs-rs` SimConnect support to send a single `SIMCONNECT_DATA_INITPOSITION`
with airborne state. Preserve speed in that structure and set `AIRSPEED INDICATED`
separately, avoiding its undocumented IAS/TAS distinction.

Do not repeatedly reposition during the hold. Stop IAS writes before verifying
unfrozen flight.

## Why freezing needs validation

Freezing can provide preparation time, but a motionless aircraft is not proof
of aerodynamic equilibrium. Controllers may accumulate corrections while motion
is constrained, producing a transient after release.

Pause and slew are unsuitable substitutes without further investigation:
A32NX treats them specially, including an autopilot-disconnect input after slew.

## Configuration and initial defaults

An optional initialisation section specifies latitude, longitude, altitude above
mean sea level, true heading, initial pitch, IAS and hold duration. Bank is zero.
Existing configurations retain their current behaviour.

Proposed configurable verification defaults:

| Measurement | Acceptance |
|---|---|
| Altitude | Within 50 ft of target |
| IAS | Within 2 kt of target |
| Heading | Within 2° of target |
| Bank | Within 1° of wings level |
| Vertical speed | Within 100 ft/min of zero |
| Attitude-change rates | Below 0.5°/s |

Require all conditions continuously for 3 simulator seconds, with a 60-second
verification timeout. These are engineering starting points requiring validation.

## Cleanup and failures

Reject preparation if freezes are already active. Track and release freezes owned
by the module on failure, cancellation or shutdown, even if another cleanup fails.

Clearing `L:REPLAYER_ARMED` during preparation cancels it. Running-replay abort
behaviour remains unchanged. Report timeouts with the failed conditions and
measured values.

## Validation

Host tests should cover configuration, phase ordering, timeouts, continuous
stability checks, heading wraparound, cancellation, cleanup failures and replay
starting at time zero.

Simulator tests must establish IAS retention, continued system updates during
the hold, release transients, reliable unfreezing and repeatable starting
conditions. Include repeated runs and aircraft reload during a hold.

Record the MSFS, A32NX and `msfs-rs` versions. Host tests and WASM compilation alone
do not establish simulator compatibility.

## References

- [MSFS freeze events](https://docs.flightsimulator.com/html/Programming_Tools/Event_IDs/Miscellaneous_Events.htm)
- [Initial-position structure](https://docs.flightsimulator.com/html/Programming_Tools/SimConnect/API_Reference/Structures_And_Enumerations/SIMCONNECT_DATA_INITPOSITION.htm)
- [A32NX integration](https://github.com/flybywiresim/aircraft/blob/master/fbw-a32nx/src/wasm/fbw_a320/src/FlyByWireInterface.cpp)

Status: proposed design; not implemented or validated in MSFS.
