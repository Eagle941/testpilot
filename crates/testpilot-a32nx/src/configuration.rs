//! Shared validation and native numeric readback for A32NX initialisation.

use testpilot_core::error::InitialisationError;
use testpilot_core::simulator::SimulatorAdapter;

/// Ensures a native LVAR exists and is readable before any setup writes.
pub(super) fn validate_interface(
    simulator: &mut dyn SimulatorAdapter,
    variable: &'static str,
) -> Result<(), InitialisationError> {
    if !simulator
        .local_variable_exists(variable)
        .map_err(InitialisationError::Readback)?
    {
        return Err(InitialisationError::MissingInterface { variable });
    }
    simulator
        .validate_read(variable, None)
        .map_err(InitialisationError::Readback)
}

/// Reads a native numeric LVAR without applying an SDK unit conversion.
pub(super) fn read_native(
    simulator: &mut dyn SimulatorAdapter,
    variable: &'static str,
) -> Result<f64, InitialisationError> {
    simulator
        .read(variable, None)
        .map_err(InitialisationError::Readback)
}
