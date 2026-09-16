//! Opt-in access to the production frame runtime for host benchmarks.
//!
//! This facade is intended for measurement, not as a stable application API.

use std::ffi::{CStr, CString, NulError};
use std::path::PathBuf;

use crate::error::{GaugeError, SimulatorError};
use crate::replayer::Replayer;
use crate::simulator::{SIMULATION_TIME_CODE, build_calculator_code, build_read_calculator_code};

pub use crate::gauge_runtime::GaugeRuntime;
pub use crate::simulator::ReadCommandCache;
pub use crate::simulator::SimulatorAdapter;

/// Builds the real frame runtime with a benchmark config and simulator adapter.
///
/// Construction resets the adapter's arming variable. The caller must then arm
/// and process the initial frame before timing steady-state updates.
pub fn new_runtime<S: SimulatorAdapter>(
    config_path: impl Into<PathBuf>,
    simulator: S,
) -> Result<GaugeRuntime<S>, GaugeError> {
    GaugeRuntime::new(Replayer::with_config_path(config_path), simulator)
}

/// Failure while preparing an MSFS calculator command on the host.
#[derive(Debug, thiserror::Error)]
pub enum PreparationError {
    /// The production formatter rejected a variable, unit, or value.
    #[error(transparent)]
    Simulator(#[from] SimulatorError),
    /// A calculator command could not be represented as a C string.
    #[error("calculator code contains an interior NUL: {0}")]
    InteriorNul(#[from] NulError),
}

/// Formats a write using the production adapter's reusable scratch buffer path.
pub fn format_write_code(
    scratch: &mut String,
    variable: &str,
    value: f64,
) -> Result<(), SimulatorError> {
    build_calculator_code(scratch, variable, value)
}

/// Prepares a write, including the CString conversion used by msfs-rs.
///
/// Does not call MSFS. Dropping the returned string releases the call's buffer.
pub fn prepare_write_code(
    scratch: &mut String,
    variable: &str,
    value: f64,
) -> Result<CString, PreparationError> {
    build_calculator_code(scratch, variable, value)?;
    Ok(CString::new(scratch.as_str())?)
}

/// Measures the original uncached recording/arming preparation path.
pub fn prepare_read_code(
    scratch: &mut String,
    variable: &str,
    unit: Option<&str>,
) -> Result<CString, PreparationError> {
    build_read_calculator_code(scratch, variable, unit)?;
    Ok(CString::new(scratch.as_str())?)
}

/// Measures the original per-frame CString allocation for the clock command.
pub fn prepare_clock_code() -> Result<CString, PreparationError> {
    Ok(CString::new(SIMULATION_TIME_CODE.to_bytes())?)
}

/// Borrows the static clock command used by the optimised production adapter.
pub fn cached_clock_code() -> &'static CStr {
    SIMULATION_TIME_CODE
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prepares_exact_null_terminated_production_commands() {
        let mut scratch = String::new();
        assert_eq!(
            prepare_write_code(&mut scratch, "K:AXIS_ELEVATOR_SET", -123.5)
                .unwrap()
                .as_bytes_with_nul(),
            b"-123.5 (>K:AXIS_ELEVATOR_SET)\0"
        );
        assert_eq!(
            prepare_read_code(&mut scratch, "A:PLANE PITCH DEGREES", Some("degrees"))
                .unwrap()
                .as_bytes_with_nul(),
            b"(A:PLANE PITCH DEGREES, degrees)\0"
        );
        assert_eq!(
            prepare_read_code(&mut scratch, "L:REPLAYER_ARMED", None)
                .unwrap()
                .as_bytes_with_nul(),
            b"(L:REPLAYER_ARMED)\0"
        );
        assert_eq!(
            prepare_clock_code().unwrap().as_bytes_with_nul(),
            b"(E:SIMULATION TIME, seconds)\0"
        );
    }

    #[test]
    fn propagates_invalid_boundary_arguments() {
        let mut scratch = String::new();
        assert!(prepare_write_code(&mut scratch, "K:TEST", f64::NAN).is_err());
        assert!(prepare_read_code(&mut scratch, "A:TEST", None).is_err());
        assert!(prepare_read_code(&mut scratch, "L:TEST", Some("number")).is_err());
        assert!(prepare_write_code(&mut scratch, "K:BAD\0NAME", 0.0).is_err());
    }
}
