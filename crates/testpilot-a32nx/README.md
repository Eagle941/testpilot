# A32NX initialisation

TestPilot can configure the FlyByWire A32NX before replay starts. Add an optional
`[initialisation]` table to `/work/replayer_config.toml`. See the
[project README](../../README.md) for installation, arming, scenario and telemetry
contracts.

```toml
[initialisation]
zfw = 60000.0
gw = 65000.0
gwcg = 25.0
ths = 1.0
flaps = 2
landing_gear = "down"
```

## Options

| Field | Meaning | Accepted values |
|---|---|---|
| `zfw` | Actual zero-fuel weight in kilograms | Finite, 42,500–64,300 kg |
| `gw` | Actual gross weight in kilograms | Finite, at least `zfw`, at most 79,000 kg |
| `gwcg` | Actual gross-weight centre of gravity in percent MAC | Finite and achievable by the aircraft loading model |
| `ths` | Trimmable horizontal stabilizer position in degrees | Finite, −4 to 13.5 inclusive |
| `flaps` | Flap lever detent | Integer `0` = UP, `1`, `2`, `3`, or `4` = FULL |
| `landing_gear` | Landing gear lever demand | Exactly `"up"` or `"down"` |

Supply `zfw`, `gw` and `gwcg` together or omit all three. Each of `ths`, `flaps`
and `landing_gear` is independently optional. Only supplied groups are commanded
and read back. An empty table behaves as an omitted section; unknown fields and
invalid values are rejected. Configuration remains `format_version = 1`.

For example, flap and gear setup alone is valid:

```toml
[initialisation]
flaps = 1
landing_gear = "up"
```

## Mass, balance and trim

Mass/balance setup requests instant passenger, cargo and fuel loading to achieve
the actual airframe targets, rather than changing flight-management entries.
`gw - zfw` determines the requested fuel mass. Targets must fit fuel capacity
and an achievable passenger/cargo distribution; a combination can be rejected
even when its individual masses are within the ranges above.

The loading model uses 84 kg per passenger and at least 20 kg of baggage per
passenger. Readiness requires actual ZFW and GW within 100 kg of their targets
and actual GWCG within 0.01 percentage points, all inclusive.

Trim setup sends the manual trim-axis demand and checks the actual A32NX trim
wheel position, converted to THS degrees. Its inclusive readiness tolerance is
0.01 degrees.

## Flaps and landing gear

Flap detents map to `K:FLAPS_UP`, `K:FLAPS_1`, `K:FLAPS_2`, `K:FLAPS_3` and
`K:FLAPS_DOWN`, respectively. Lever 1 allows A32NX to select CONF 1 or CONF 1+F
automatically.

Flap readiness checks only `L:A32NX_IS_FLAPS_MOVING` and
`L:A32NX_IS_SLATS_MOVING`: both must be `0`. It can pass on the first readback,
without a submission delay or a requirement to observe movement first. This
confirms stopped motion, not command acceptance or attainment of a particular
physical configuration. The flags must be exactly `0` or `1`.

Gear setup sends `K:GEAR_UP` or `K:GEAR_DOWN`. Readiness requires either
`L:A32NX_LGCIU_1_DISCRETE_WORD_1` or `L:A32NX_LGCIU_2_DISCRETE_WORD_1`, with
Normal Operation status, to confirm the requested selection, all three gear
locks in that position and closed, locked doors. Other valid statuses keep setup
waiting. Indications from different computers are not combined.

## Execution and prerequisites

Setup applies only to a detected A32NX; unsupported or unidentified aircraft skip
it and proceed to replay. On a supported aircraft, missing required interfaces,
malformed readback and simulator I/O failures fail the run.

Configured demands are resubmitted on each initialisation update. All configured
groups must be ready together before the 30 simulator-second deadline measured
from arming; timeout takes precedence at the deadline. Waiting produces no replay
inputs or telemetry. Playback advances on the following update after readiness.

Provide the electrical and hydraulic conditions needed for the requested
movement. Autopilot setup remains the operator's responsibility. Completion,
failure and shutdown stop setup commands and reset arming; prior lever or trim
positions are not restored. Run failures return the gauge to idle for another run.
