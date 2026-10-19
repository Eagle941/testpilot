//! Initial pitch/roll targets using the A32NX's aeronautical sign convention.
//!
//! MSFS 2020 lists the angle and body rotation velocity SimVars as settable.
//! A32NX revision ce46d9dbc7a90bd9afddc75d61c64316a6f78d0d negates simulator
//! pitch and bank in navigation/adirs.rs to obtain positive nose-up/right-bank
//! angles. a320_systems_wasm/src/lib.rs requests degrees and degree per second.

use testpilot_core::error::InitialisationError;
use testpilot_core::simulator::SimulatorAdapter;

use crate::configuration::read_simvar;

/// Pitch/bank and corresponding body-rate interfaces, units and readiness tolerances.
pub(super) const ATTITUDE_VARIABLES: [(&str, &str, f64); 4] = [
    ("A:PLANE PITCH DEGREES", "degrees", 0.5),
    ("A:PLANE BANK DEGREES", "degrees", 0.5),
    ("A:ROTATION VELOCITY BODY X", "degree per second", 0.1),
    ("A:ROTATION VELOCITY BODY Z", "degree per second", 0.1),
];

/// Optional pitch/roll demands and readiness captured before setup writes.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(super) struct A32nxAttitude {
    /// Nose-up and right-bank demands in degrees; omitted axes receive no I/O.
    targets: [Option<f64>; 2],
    /// Angle and rate acceptance for each axis, including omitted axes.
    ready: [bool; 2],
    /// Aeronautical pitch/bank observations retained for flight-path feedback.
    observed: [Option<f64>; 2],
    /// Aeronautical pitch/roll rates, with MSFS body X/Z signs reversed.
    rates: [Option<f64>; 2],
}

impl A32nxAttitude {
    /// Validates both targets before observing the configured angles and rates.
    pub(super) fn prepare(
        simulator: &mut dyn SimulatorAdapter,
        pitch: Option<f64>,
        roll: Option<f64>,
    ) -> Result<Self, InitialisationError> {
        let targets = [pitch, roll];
        for (field, target, maximum) in [("pitch", pitch, 90.0), ("roll", roll, 180.0)] {
            if let Some(degrees) = target
                && (!degrees.is_finite() || !(-maximum..=maximum).contains(&degrees))
            {
                return Err(InitialisationError::InvalidAttitudeTarget {
                    field,
                    degrees,
                    minimum: -maximum,
                    maximum,
                });
            }
        }
        let mut ready = [true; 2];
        let mut observed = [None; 2];
        let mut rates = [None; 2];
        // Read angles before rates, retaining one consistent pre-write observation.
        for (index, target) in targets.into_iter().enumerate() {
            if let Some(target) = target {
                let (variable, unit, tolerance) = ATTITUDE_VARIABLES[index];
                let mut actual = -read_simvar(simulator, variable, unit)?;
                if index == 1 {
                    // Bank readback can wrap between -180 and +180 degrees.
                    actual = target + (actual - target + 180.0).rem_euclid(360.0) - 180.0;
                }
                ready[index] = actual >= target - tolerance && actual <= target + tolerance;
                observed[index] = Some(actual);
            }
        }
        for (index, target) in targets.into_iter().enumerate() {
            if target.is_some() {
                let (variable, unit, tolerance) = ATTITUDE_VARIABLES[index + 2];
                let rate = -read_simvar(simulator, variable, unit)?;
                rates[index] = Some(rate);
                ready[index] &= rate.abs() <= tolerance;
            }
        }
        Ok(Self {
            targets,
            ready,
            observed,
            rates,
        })
    }

    /// Returns bank and body rates used by flight-path feedback.
    pub(super) fn flight_path_observation(&self) -> Result<(f64, f64, f64), InitialisationError> {
        match (self.observed[1], self.rates[0], self.rates[1]) {
            (Some(bank), Some(pitch_rate), Some(roll_rate)) => Ok((bank, pitch_rate, roll_rate)),
            _ => Err(InitialisationError::MissingReadback {
                field: "flight-path attitude",
            }),
        }
    }

    /// Whether every requested angle and rotation rate is ready.
    pub(super) fn is_ready(&self) -> bool {
        self.targets
            .iter()
            .zip(self.ready)
            .all(|(target, ready)| target.is_none() || ready)
    }

    /// Checks an explicit pitch target; omission adds no separate readiness requirement.
    pub(super) fn pitch_ready(&self, target: Option<f64>) -> bool {
        target.is_none_or(|target| self.targets[0] == Some(target) && self.ready[0])
    }

    /// Checks an explicit roll target; omission adds no separate readiness requirement.
    pub(super) fn roll_ready(&self, target: Option<f64>) -> bool {
        target.is_none_or(|target| self.targets[1] == Some(target) && self.ready[1])
    }

    /// Applies the configured angles, then clears their pitch/roll rotation rates.
    pub(super) fn submit(
        &self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<(), InitialisationError> {
        for (index, target) in self.targets.into_iter().enumerate() {
            if let Some(target) = target {
                let (variable, unit, _) = ATTITUDE_VARIABLES[index];
                simulator
                    .write(variable, -target, Some(unit))
                    .map_err(InitialisationError::Submit)?;
            }
        }
        for (index, target) in self.targets.into_iter().enumerate() {
            if target.is_some() {
                let (variable, unit, _) = ATTITUDE_VARIABLES[index + 2];
                simulator
                    .write(variable, 0.0, Some(unit))
                    .map_err(InitialisationError::Submit)?;
            }
        }
        Ok(())
    }
}
