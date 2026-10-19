//! Pressure-altitude feedback through the simulator's geometric-altitude setter.

use std::time::Duration;

use testpilot_core::error::InitialisationError;
use testpilot_core::simulator::SimulatorAdapter;

use crate::configuration::{ramp_towards, read_simvar, validate_flight_condition_target};

/// Actual geometric altitude, distinct from pressure altitude in current weather.
pub(super) const PLANE_ALTITUDE: &str = "A:PLANE ALTITUDE";
/// Read-only standard-pressure altitude; explicitly request conversion to feet.
pub(super) const PRESSURE_ALTITUDE: &str = "A:PRESSURE ALTITUDE";

/// Pressure-altitude setpoint rate, in feet per simulator second (30,000 ft/min).
const ALTITUDE_RATE: f64 = 500.0;
/// Inclusive pressure-altitude acceptance tolerance in feet.
pub(super) const ALTITUDE_TOLERANCE: f64 = 10.0;

/// Per-run pressure-altitude ramp and the corresponding geometric correction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct A32nxPressureAltitude {
    /// Final configured pressure altitude in feet.
    target: f64,
    /// Intermediate pressure-altitude setpoint in feet.
    ramped_target: f64,
    /// Timestamp of the preceding ramp update.
    time: Duration,
    /// Finite geometric altitude demand in feet for this update.
    demanded: f64,
    /// Pressure altitude in feet observed before any setup writes in this update.
    pub(super) observed: f64,
}

impl A32nxPressureAltitude {
    /// The ramp and actual pressure altitude have reached the repositioning seed.
    pub(super) fn ready_for_release(&self) -> bool {
        self.ramp_seconds() == 0.0 && self.is_ready(self.target)
    }
    /// Validates and observes the target's interfaces, without issuing a command.
    pub(super) fn prepare(
        simulator: &mut dyn SimulatorAdapter,
        target: f64,
        now: Duration,
        previous: Option<Self>,
    ) -> Result<Self, InitialisationError> {
        validate_flight_condition_target("pressure_altitude", target)?;
        let observed = read_simvar(simulator, PRESSURE_ALTITUDE, "feet")?;
        let plane_altitude = read_simvar(simulator, PLANE_ALTITUDE, "feet")?;
        let ramped_target = if let Some(previous) = previous.filter(|p| p.target == target) {
            let elapsed =
                now.checked_sub(previous.time)
                    .ok_or(InitialisationError::ClockMovedBackwards {
                        previous: previous.time,
                        current: now,
                    })?;
            ramp_towards(
                previous.ramped_target,
                target,
                ALTITUDE_RATE * elapsed.as_secs_f64(),
            )
        } else {
            observed
        };
        let demanded = plane_altitude + (ramped_target - observed);
        if !demanded.is_finite() {
            return Err(InitialisationError::InvalidAltitudeCorrection {
                plane_altitude,
                pressure_altitude: observed,
                target,
                value: demanded,
            });
        }
        Ok(Self {
            target,
            ramped_target,
            time: now,
            demanded,
            observed,
        })
    }

    /// Travel allowance from the initial observed altitude, in simulator seconds.
    pub(super) fn ramp_seconds(&self) -> f64 {
        (self.target - self.ramped_target).abs() / ALTITUDE_RATE
    }

    /// Confirms altitude and its setpoint using the pre-write observations.
    pub(super) fn is_ready(&self, target: f64) -> bool {
        self.target == target
            && self.observed >= target - ALTITUDE_TOLERANCE
            && self.observed <= target + ALTITUDE_TOLERANCE
            && self.ramped_target >= target - ALTITUDE_TOLERANCE
            && self.ramped_target <= target + ALTITUDE_TOLERANCE
    }

    /// Applies one altitude correction; the next update obtains fresh feedback.
    pub(super) fn submit(
        &self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<(), InitialisationError> {
        simulator
            .write(PLANE_ALTITUDE, self.demanded, Some("feet"))
            .map_err(InitialisationError::Submit)
    }
}
