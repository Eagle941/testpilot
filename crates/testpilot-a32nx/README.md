# A32NX initialisation

Add an optional `[initialisation]` table to `/work/replayer_config.toml`.
See the [project README](../../README.md) for installation, arming and file formats.

```toml
[initialisation]
zfw = 60000.0
gw = 65000.0
gwcg = 25.0
ths = 1.0
flaps = 2
landing_gear = "down"
pressure_altitude = 10000.0
calibrated_airspeed = 250.0
vertical_speed = 0.0
pitch = 3.0
roll = 0.0
```

## Configuration

| Field | Meaning | Accepted values |
|---|---|---|
| `zfw` | Zero-fuel weight, kg | 42,500–64,300 |
| `gw` | Gross weight, kg | At least `zfw`, at most 79,000 |
| `gwcg` | Gross-weight CG, percent MAC | Achievable by the loading model |
| `ths` | Initial stabilizer position, degrees | −4 to 13.5 |
| `flaps` | Lever detent | Integer 0 (UP) to 4 (FULL) |
| `landing_gear` | Gear demand | `"up"` or `"down"` |
| `pressure_altitude` | Pressure altitude at playback start, ft | Strictly positive |
| `calibrated_airspeed` | Calibrated airspeed, kt | Strictly positive |
| `vertical_speed` | Vertical speed, ft/min, positive climb | Any finite value; defaults to 0 for flight setup |
| `pitch` | Initial pitch, degrees, positive nose up | −90 to 90 |
| `roll` | Roll, degrees, positive right wing down | −180 to 180 |

All numeric values must be finite; bounds are inclusive. Supply `zfw`, `gw` and
`gwcg` together or omit all three. Other fields are independently optional.
Altitude, airspeed or vertical-speed setup defaults omitted pitch/roll to 0 degrees.
Pitch and THS are reposition seeds during flight setup; roll remains a bank target.
Otherwise omitted axes receive no commands. Unknown fields and invalid values are rejected.
An empty table behaves as an omitted section.

## Setup behaviour

Setup requests instant passenger, cargo and fuel loading; unreachable combinations
fail. It waits for all configured groups to be ready before playback. The deadline
includes planned altitude/speed travel and a settling allowance, measured from
arming. Timeout errors list the parameters still pending.

Altitude and speed approach their targets gradually. Once the ramps finish, direct
altitude, attitude, angular-rate, IAS and trim writes stop. Automatic sidestick and
thrust feedback establish the requested vertical speed, bank and airspeed while
the flight controls trim naturally. Playback requires three seconds of stability
with Normal Law, settled trim and nearly neutral stick. Setup commands stop before
playback; flight conditions are not held during playback.

A climb/descent with a playback altitude starts 60 seconds of vertical travel
before that altitude; the lead-in altitude must remain positive. Playback waits
for arrival within 10 ft or a crossing; passing it before settling fails the run.

`L:A32NX_IS_READY` is set to `0` before setup commands and to `1` when the altitude
and speed ramps finish, before stability verification. Stop, failure and timeout
also restore `1`; failed restoration is retried before another run.

Flap readiness confirms stopped flap/slat motion, not the requested physical
configuration. Lever 1 permits automatic CONF 1/1+F selection. Gear readiness
requires the requested locks and closed doors. Retraction also requires airborne
readback and IAS/Mach within 220 kt/M0.54; extension requires 250 kt/M0.60 or less.
With speed setup, its intermediate goal is at most 200 kt while gear is pending.
Gear-down targets above 280 kt are rejected. See the
[published A32NX limits](https://docs.flybywiresim.com/pilots-corner/a32nx/a32nx-briefing/limitations/).

## Prerequisites and validation

Provide electrical/hydraulic power, running engines and the required autopilot
setup. Initialisation supplies sidestick and thrust commands automatically; avoid
competing hardware or cockpit axis inputs. Speed setup uses the standard A32NX
idle mapping; custom idle calibrations are unsupported. Autopilot modes and
autothrust state are neither checked nor changed.

Manual MSFS 2020 validation and controller tuning remain pending. Record the MSFS
build, A32NX version and configuration; verify repositioning, speed recovery,
attitude/gear behaviour and cleanup on completion, failure and shutdown. Host tests
and WASM compilation alone do not establish simulator compatibility.
