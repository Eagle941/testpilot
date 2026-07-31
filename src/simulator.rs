//! MSFS simulator-variable writes through legacy calculator code.

use std::fmt::Write;
use std::time::Duration;

use crate::error::SimulatorError;

#[cfg(target_arch = "wasm32")]
const SIMULATION_TIME_CODE: &str = "(E:SIMULATION TIME, seconds)";

/// Simulator operations required by replay injection.
pub(crate) trait SimulatorAdapter {
    /// Returns the current simulator-clock time.
    fn simulation_time(&self) -> Result<Duration, SimulatorError>;

    /// Writes a value to a prefixed simulator destination.
    fn write(&mut self, variable: &str, value: f64) -> Result<(), SimulatorError>;
}

/// MSFS implementation backed by legacy calculator code.
pub(crate) struct MsfsSimulator {
    calculator_code: String,
}

impl MsfsSimulator {
    /// Creates an adapter with a reusable calculator-code buffer.
    pub(crate) const fn new() -> MsfsSimulator {
        MsfsSimulator {
            calculator_code: String::new(),
        }
    }
}

#[cfg(target_arch = "wasm32")]
impl SimulatorAdapter for MsfsSimulator {
    fn simulation_time(&self) -> Result<Duration, SimulatorError> {
        let value = msfs::legacy::execute_calculator_code::<f64>(SIMULATION_TIME_CODE)
            .ok_or(SimulatorError::SimulationTimeUnavailable)?;
        Duration::try_from_secs_f64(value)
            .map_err(|_| SimulatorError::InvalidSimulationTime { value })
    }

    fn write(&mut self, variable: &str, value: f64) -> Result<(), SimulatorError> {
        build_calculator_code(&mut self.calculator_code, variable, value)?;
        msfs::legacy::execute_calculator_code::<()>(&self.calculator_code).ok_or_else(|| {
            SimulatorError::CalculatorCodeWriteFailed {
                variable: variable.to_owned(),
                value,
            }
        })
    }
}

/// Formats one finite value write for a prefixed `K:` event or `L:` variable.
///
/// The output buffer is cleared and reused. Invalid destinations and non-finite
/// values are rejected before any calculator code is produced.
fn build_calculator_code(
    output: &mut String,
    variable: &str,
    value: f64,
) -> Result<(), SimulatorError> {
    if !value.is_finite() {
        return Err(SimulatorError::NonFiniteWrite {
            variable: variable.to_owned(),
            value,
        });
    }
    if !matches!(variable.as_bytes(), [b'K' | b'L', b':', _, ..])
        || variable.as_bytes().contains(&0)
    {
        return Err(SimulatorError::UnsupportedVariable {
            variable: variable.to_owned(),
        });
    }

    output.clear();
    write!(output, "{value} (>{variable})").map_err(|source| {
        SimulatorError::CalculatorCodeFormatting {
            variable: variable.to_owned(),
            source,
        }
    })
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use crate::error::SimulatorError;

    use super::{MsfsSimulator, SimulatorAdapter, build_calculator_code};

    struct FakeSimulator {
        time: Duration,
        writes: Vec<(String, f64)>,
    }

    impl SimulatorAdapter for FakeSimulator {
        fn simulation_time(&self) -> Result<Duration, SimulatorError> {
            Ok(self.time)
        }

        fn write(&mut self, variable: &str, value: f64) -> Result<(), SimulatorError> {
            self.writes.push((variable.to_owned(), value));
            Ok(())
        }
    }

    #[test]
    fn supports_fake_simulator_adapters() {
        let mut simulator = FakeSimulator {
            time: Duration::from_secs(42),
            writes: Vec::new(),
        };

        assert_eq!(
            simulator.simulation_time().unwrap(),
            Duration::from_secs(42)
        );
        simulator.write("L:TEST", 1.0).unwrap();
        assert_eq!(simulator.writes, vec![("L:TEST".to_owned(), 1.0)]);
    }

    #[test]
    fn builds_key_event_and_local_variable_writes() {
        let mut output = String::with_capacity(64);

        build_calculator_code(&mut output, "K:AXIS_ELEVATOR_SET", -8192.5).unwrap();
        assert_eq!(output, "-8192.5 (>K:AXIS_ELEVATOR_SET)");

        build_calculator_code(&mut output, "L:A32NX_EXAMPLE", 1.0).unwrap();
        assert_eq!(output, "1 (>L:A32NX_EXAMPLE)");
    }

    #[test]
    fn reuses_and_clears_the_output_buffer() {
        let mut simulator = MsfsSimulator::new();
        simulator.calculator_code.reserve(128);
        let capacity = simulator.calculator_code.capacity();

        build_calculator_code(
            &mut simulator.calculator_code,
            "K:AXIS_AILERONS_SET",
            16384.0,
        )
        .unwrap();
        build_calculator_code(&mut simulator.calculator_code, "L:X", 0.0).unwrap();

        assert_eq!(simulator.calculator_code, "0 (>L:X)");
        assert_eq!(simulator.calculator_code.capacity(), capacity);
    }

    #[test]
    fn rejects_invalid_destinations_and_values() {
        let mut output = String::new();

        for variable in [
            "AXIS_ELEVATOR_SET",
            "K:",
            "A:ELEVATOR POSITION",
            "L:BAD\0NAME",
        ] {
            assert!(matches!(
                build_calculator_code(&mut output, variable, 0.0),
                Err(SimulatorError::UnsupportedVariable { .. })
            ));
        }
        assert!(matches!(
            build_calculator_code(&mut output, "L:TEST", f64::NAN),
            Err(SimulatorError::NonFiniteWrite { .. })
        ));
    }
}
