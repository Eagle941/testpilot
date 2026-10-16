//! A32NX THS target preparation, command submission and actual-state readback.

use testpilot_core::error::InitialisationError;
use testpilot_core::simulator::SimulatorAdapter;

use crate::configuration::read_native;

/// Trim-wheel position published by A32NX in percent.
const TRIM_POSITION: &str = "L:A32NX_HYD_TRIM_WHEEL_PERCENT";
/// Transient manual trim demand consumed each simulator tick.
const TRIM_EVENT: &str = "K:AXIS_ELEV_TRIM_SET";

/// Validated manual trim-axis demand, prepared before any setup writes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct A32nxThs {
    /// Integer event value corresponding to the requested THS degrees.
    axis: f64,
}

impl A32nxThs {
    /// Validates the target and its readback interface without writing commands.
    pub(super) fn prepare(
        simulator: &mut dyn SimulatorAdapter,
        degrees: f64,
    ) -> Result<Self, InitialisationError> {
        let axis = Self::axis_demand(degrees)?;
        if !simulator
            .local_variable_exists(TRIM_POSITION)
            .map_err(InitialisationError::Readback)?
        {
            return Err(InitialisationError::MissingThsInterface);
        }
        simulator
            .validate_read(TRIM_POSITION, None)
            .map_err(InitialisationError::Readback)?;
        Ok(Self { axis })
    }

    /// Submits the prepared manual trim demand.
    pub(super) fn submit(
        &self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<(), InitialisationError> {
        simulator
            .write(TRIM_EVENT, self.axis)
            .map_err(InitialisationError::Submit)
    }

    /// Converts the native trim-wheel percentage to THS degrees.
    pub(super) fn readback(
        &self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<f64, InitialisationError> {
        read_native(simulator, TRIM_POSITION).map(|percent| -4.0 + percent * 17.5 / 100.0)
    }

    /// Maps THS degrees to the SDK's integer event range without clamping.
    fn axis_demand(degrees: f64) -> Result<f64, InitialisationError> {
        if !degrees.is_finite() || !(-4.0..=13.5).contains(&degrees) {
            return Err(InitialisationError::InvalidThsTarget { degrees });
        }
        Ok((-16383.0 + (degrees + 4.0) * 32767.0 / 17.5).round())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axis_conversion_preserves_endpoints_and_rounds_to_integer_events() {
        for (degrees, axis) in [
            (-4.0, -16383.0),
            (0.0, -8893.0),
            (4.75, 1.0),
            (13.5, 16384.0),
        ] {
            assert_eq!(A32nxThs::axis_demand(degrees).unwrap(), axis);
        }
        for value in [
            -4.000001,
            13.500001,
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
        ] {
            assert!(matches!(
                A32nxThs::axis_demand(value),
                Err(InitialisationError::InvalidThsTarget { .. })
            ));
        }
    }
}
