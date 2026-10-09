//! Simulator I/O and clock contracts used by replay and aircraft integration.

use std::time::Duration;

use crate::error::SimulatorError;

/// Simulator operations required by replay injection.
pub trait SimulatorAdapter {
    /// Checks whether an `L:` variable exists, without registering it or reading its value.
    fn local_variable_exists(&mut self, variable: &str) -> Result<bool, SimulatorError>;

    /// Reads an aircraft (`A:`) string variable using the SDK's string unit.
    fn read_string(&mut self, variable: &str) -> Result<String, SimulatorError>;

    /// Returns the current simulator-clock time.
    fn simulation_time(&self) -> Result<Duration, SimulatorError>;

    /// Writes a value to a prefixed simulator destination.
    fn write(&mut self, variable: &str, value: f64) -> Result<(), SimulatorError>;

    /// Validates that a prefixed simulator source can be read with the given unit.
    fn validate_read(&mut self, variable: &str, unit: Option<&str>) -> Result<(), SimulatorError>;

    /// Reads a finite value from a prefixed simulator source.
    fn read(&mut self, variable: &str, unit: Option<&str>) -> Result<f64, SimulatorError>;
}
