//! Automatic sidestick feedback after releasing repositioning and manual trim.
//!
//! Interfaces verified against A32NX ce46d9dbc7a90bd9afddc75d61c64316a6f78d0d:
//! SimConnectInterface.cpp normalises both axis events by 16384; FlyByWireInterface.cpp
//! negates them for the ELAC sidestick inputs. FCDC word 1 bits 11/16 indicate
//! pitch/roll Normal Law. ARINC packing follows Arinc429Utils::toSimVar.

use std::time::Duration;

use testpilot_core::config::InitialisationConfig;
use testpilot_core::error::InitialisationError;
use testpilot_core::simulator::SimulatorAdapter;

use crate::airspeed::VERTICAL_SPEED;
use crate::altitude::ALTITUDE_TOLERANCE;
use crate::attitude::A32nxAttitude;
use crate::configuration::{decode_discrete_word, read_native, read_simvar, validate_interface};
use crate::landing_gear::SIM_ON_GROUND;
use crate::ths::{TRIM_POSITION, read_trim_degrees};

/// Same sidestick events consumed by the A32NX ELAC input path.
pub(super) const STICK_EVENTS: [&str; 2] = ["K:AXIS_ELEVATOR_SET", "K:AXIS_AILERONS_SET"];
/// Physical flight-control law status from the two FCDCs.
pub(super) const FCDC_VARIABLES: [&str; 2] = [
    "L:A32NX_FCDC_1_DISCRETE_WORD_1",
    "L:A32NX_FCDC_2_DISCRETE_WORD_1",
];
/// Inclusive vertical-speed tolerance in ft/min.
pub(super) const VS_TOLERANCE: f64 = 50.0;
/// Time reserved before the requested altitude for establishing a steady climb/descent.
pub(super) const ALTITUDE_LEAD_SECONDS: f64 = 60.0;
/// Required interval with neutral stick and settled trim.
const STABLE_TIME: Duration = Duration::from_secs(3);
/// Maximum automatic sidestick deflection; the output is a physical actuator demand.
const MAX_STICK: f64 = 0.2;
/// Limit changes to automatic sidestick deflection per simulator second.
const STICK_RATE: f64 = 0.2;
/// Centre the flight path before reaching the wider altitude acceptance boundary.
const ALTITUDE_CONTROL_BAND: f64 = 5.0;

/// Flight-path observations and controller history for one stabilisation phase.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct A32nxFlightPath {
    /// Requested steady climb/descent rate in ft/min.
    pub(super) target: f64,
    /// Latest measured vertical speed in ft/min.
    pub(super) observed: f64,
    /// Latest bank observation, positive right wing down.
    pub(super) bank: f64,
    /// Latest stabilizer position in degrees.
    trim: f64,
    /// Measured trim movement in degrees per second; absent on the first sample.
    pub(super) trim_rate: Option<f64>,
    /// One valid FCDC confirms both Normal Laws.
    pub(super) normal_law: bool,
    /// Latest normalised nose-up/right-bank sidestick requests.
    sticks: [f64; 2],
    /// Observation timestamp.
    time: Duration,
    /// Start of the unforced verification interval.
    stable_since: Option<Duration>,
    /// Pressure altitude where natural flight started, for climb/descent tracking.
    reference_altitude: Option<f64>,
    /// Timestamp of the pressure-altitude reference.
    reference_time: Duration,
    /// Latest pressure altitude in feet.
    pub(super) altitude: Option<f64>,
    /// Target altitude was crossed on this update in the demanded direction.
    crossed_altitude: bool,
}

impl A32nxFlightPath {
    /// Validates additional interfaces before any initialisation command.
    pub(super) fn validate(
        simulator: &mut dyn SimulatorAdapter,
        target: f64,
    ) -> Result<(), InitialisationError> {
        if !target.is_finite() {
            return Err(InitialisationError::InvalidFlightPath {
                reason: "vertical speed must be finite",
            });
        }
        for variable in FCDC_VARIABLES.into_iter().chain([TRIM_POSITION]) {
            validate_interface(simulator, variable)?;
        }
        simulator
            .validate_read(VERTICAL_SPEED, Some("feet/minute"))
            .map_err(InitialisationError::Readback)
    }

    /// Chooses the reposition height on the approach side of the playback altitude.
    pub(super) fn reposition_altitude(
        target: f64,
        vertical_speed: f64,
    ) -> Result<f64, InitialisationError> {
        let start = target - vertical_speed * ALTITUDE_LEAD_SECONDS / 60.0;
        if !start.is_finite() || start <= 0.0 {
            return Err(InitialisationError::InvalidFlightPath {
                reason: "climb/descent lead-in must have a finite positive pressure altitude",
            });
        }
        Ok(start)
    }

    /// Reads physical flight controls and prepares bounded sidestick feedback using simulator time.
    pub(super) fn prepare(
        simulator: &mut dyn SimulatorAdapter,
        targets: InitialisationConfig,
        attitude: &A32nxAttitude,
        vertical_speed: Option<f64>,
        altitude: Option<f64>,
        now: Duration,
        previous: Option<Self>,
    ) -> Result<Self, InitialisationError> {
        let target = targets.vertical_speed.unwrap_or(0.0);
        let observed = match vertical_speed {
            Some(value) => value,
            None => read_simvar(simulator, VERTICAL_SPEED, "feet/minute")?,
        };
        let (bank, pitch_rate, roll_rate) = attitude.flight_path_observation()?;
        let trim = read_trim_degrees(simulator)?;
        let mut normal_law = false;
        for variable in FCDC_VARIABLES {
            if let Some(word) = decode_discrete_word(variable, read_native(simulator, variable)?)? {
                normal_law |= word & ((1 << 10) | (1 << 15)) == ((1 << 10) | (1 << 15));
            }
        }
        let ground = read_simvar(simulator, SIM_ON_GROUND, "bool")?;
        if ground != 0.0 && ground != 1.0 {
            return Err(InitialisationError::InvalidReadback {
                variable: SIM_ON_GROUND,
                value: ground,
                reason: "ground state must be exactly 0 or 1",
            });
        }
        normal_law &= ground == 0.0;
        let elapsed = previous.map_or(Ok(Duration::ZERO), |previous| {
            now.checked_sub(previous.time)
                .ok_or(InitialisationError::ClockMovedBackwards {
                    previous: previous.time,
                    current: now,
                })
        })?;
        let reference_altitude = previous.and_then(|p| p.reference_altitude).or(altitude);
        let reference_time = previous.map_or(now, |p| p.reference_time);
        let reference = if target == 0.0 {
            targets.pressure_altitude
        } else {
            reference_altitude
                .map(|height| height + target * (now - reference_time).as_secs_f64() / 60.0)
        };
        let path_error = reference
            .zip(altitude)
            .map_or(0.0, |(reference, altitude)| reference - altitude);
        let demanded_vs = target + (path_error * 6.0).clamp(-500.0, 500.0);
        let bank_target = targets.roll.unwrap_or(0.0);
        let bank_error = (bank_target - bank + 180.0).rem_euclid(360.0) - 180.0;
        let trim_rate = previous.and_then(|p| {
            if elapsed.is_zero() {
                if trim == p.trim { p.trim_rate } else { None }
            } else {
                Some((trim - p.trim) / elapsed.as_secs_f64())
            }
        });
        let near_path = (observed - target).abs() <= VS_TOLERANCE
            && path_error.abs() <= ALTITUDE_TOLERANCE
            && bank_error.abs() <= 0.5;
        let needs_correction = !near_path || path_error.abs() > ALTITUDE_CONTROL_BAND;
        let desired = if normal_law && needs_correction {
            [
                ((demanded_vs - observed) * 0.00008 - pitch_rate * 0.04)
                    .clamp(-MAX_STICK, MAX_STICK),
                (bank_error * 0.025 - roll_rate * 0.04).clamp(-MAX_STICK, MAX_STICK),
            ]
        } else {
            [0.0; 2]
        };
        let old_sticks = previous.map_or([0.0; 2], |p| p.sticks);
        let sticks = if !normal_law {
            [0.0; 2]
        } else {
            std::array::from_fn(|i| {
                old_sticks[i]
                    + (desired[i] - old_sticks[i]).clamp(
                        -STICK_RATE * elapsed.as_secs_f64(),
                        STICK_RATE * elapsed.as_secs_f64(),
                    )
            })
        };
        let steady = normal_law
            && near_path
            && sticks.iter().all(|v| v.abs() <= 0.005)
            && trim_rate.is_some_and(|rate| rate.abs() <= 0.02);
        let stable_since = if steady {
            Some(previous.and_then(|p| p.stable_since).unwrap_or(now))
        } else {
            None
        };
        let crossed_altitude = targets
            .pressure_altitude
            .zip(previous.and_then(|p| p.altitude))
            .zip(altitude)
            .is_some_and(|((goal, old), current)| {
                (old - goal) * target < 0.0 && (current - goal) * target >= 0.0
            });
        Ok(Self {
            target,
            observed,
            bank,
            trim,
            trim_rate,
            normal_law,
            sticks,
            time: now,
            stable_since,
            reference_altitude,
            reference_time,
            altitude,
            crossed_altitude,
        })
    }

    /// Issues only the verified sidestick events; no attitude, velocity or trim setters.
    pub(super) fn submit(
        &self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<(), InitialisationError> {
        for (event, stick) in STICK_EVENTS.into_iter().zip(self.sticks) {
            simulator
                .write(event, (-stick * 16384.0).round(), None)
                .map_err(InitialisationError::Submit)?;
        }
        Ok(())
    }

    /// All flight-path checks have held with nearly neutral stick and settled trim for three seconds.
    pub(super) fn is_ready(&self) -> bool {
        self.stable_since
            .is_some_and(|since| self.time - since >= STABLE_TIME)
    }

    /// Level flight captures a fixed altitude; a climb/descent gates playback on arrival/crossing.
    pub(super) fn altitude_ready(&self, target: f64) -> bool {
        self.altitude.is_some_and(|current| {
            (current - target).abs() <= ALTITUDE_TOLERANCE || self.crossed_altitude
        })
    }

    /// Detects an altitude crossing that cannot be accepted after the current update.
    pub(super) fn missed_altitude(&self, target: f64) -> bool {
        self.target != 0.0
            && !self.crossed_altitude
            && self.altitude.is_some_and(|current| {
                (current - target) * self.target.signum() > ALTITUDE_TOLERANCE
            })
    }
}
