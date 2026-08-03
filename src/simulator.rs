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

    /// Validates that a prefixed simulator source can be read with the given unit.
    fn validate_read(&mut self, variable: &str, unit: Option<&str>) -> Result<(), SimulatorError>;

    /// Reads a finite value from a prefixed simulator source.
    fn read(&mut self, variable: &str, unit: Option<&str>) -> Result<f64, SimulatorError>;
}

/// MSFS implementation backed by legacy calculator code.
pub(crate) struct MsfsSimulator {
    calculator_code_buffer: String,
}

impl MsfsSimulator {
    /// Creates an adapter with a reusable calculator-code buffer.
    pub(crate) const fn new() -> MsfsSimulator {
        MsfsSimulator {
            calculator_code_buffer: String::new(),
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
        build_calculator_code(&mut self.calculator_code_buffer, variable, value)?;
        msfs::legacy::execute_calculator_code::<()>(&self.calculator_code_buffer).ok_or_else(|| {
            SimulatorError::CalculatorCodeWriteFailed {
                variable: variable.to_owned(),
                value,
            }
        })
    }

    fn validate_read(&mut self, variable: &str, unit: Option<&str>) -> Result<(), SimulatorError> {
        build_read_calculator_code(&mut self.calculator_code_buffer, variable, unit)
    }

    fn read(&mut self, variable: &str, unit: Option<&str>) -> Result<f64, SimulatorError> {
        self.validate_read(variable, unit)?;
        let value = msfs::legacy::execute_calculator_code::<f64>(&self.calculator_code_buffer)
            .ok_or_else(|| SimulatorError::CalculatorCodeReadFailed {
                variable: variable.to_owned(),
            })?;
        if !value.is_finite() {
            return Err(SimulatorError::NonFiniteRead {
                variable: variable.to_owned(),
                value,
            });
        }
        Ok(value)
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

/// Formats a calculator-code read for an `A:` or `L:` simulator variable.
fn build_read_calculator_code(
    output: &mut String,
    variable: &str,
    unit: Option<&str>,
) -> Result<(), SimulatorError> {
    if variable.as_bytes().contains(&0) {
        return Err(SimulatorError::UnsupportedReadVariable {
            variable: variable.to_owned(),
        });
    }

    output.clear();
    match variable.as_bytes() {
        [b'A', b':', _, ..] => {
            let unit = unit
                .filter(|unit| !unit.is_empty() && !unit.as_bytes().contains(&0))
                .ok_or_else(|| SimulatorError::MissingReadUnit {
                    variable: variable.to_owned(),
                })?;
            write!(output, "({variable}, {unit})")
        }
        [b'L', b':', _, ..] if unit.is_none() => write!(output, "({variable})"),
        [b'L', b':', _, ..] => {
            return Err(SimulatorError::UnexpectedReadUnit {
                variable: variable.to_owned(),
            });
        }
        _ => {
            return Err(SimulatorError::UnsupportedReadVariable {
                variable: variable.to_owned(),
            });
        }
    }
    .map_err(|source| SimulatorError::CalculatorCodeFormatting {
        variable: variable.to_owned(),
        source,
    })
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use crate::error::SimulatorError;

    use super::{
        MsfsSimulator, SimulatorAdapter, build_calculator_code, build_read_calculator_code,
    };

    struct FakeSimulator {
        time: Duration,
        read_value: f64,
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

        fn validate_read(
            &mut self,
            variable: &str,
            unit: Option<&str>,
        ) -> Result<(), SimulatorError> {
            let mut output = String::new();
            build_read_calculator_code(&mut output, variable, unit)
        }

        fn read(&mut self, _variable: &str, _unit: Option<&str>) -> Result<f64, SimulatorError> {
            Ok(self.read_value)
        }
    }

    #[test]
    fn supports_fake_simulator_adapters() {
        let mut simulator = FakeSimulator {
            time: Duration::from_secs(42),
            read_value: 2.5,
            writes: Vec::new(),
        };

        assert_eq!(
            simulator.simulation_time().unwrap(),
            Duration::from_secs(42)
        );
        simulator.write("L:TEST", 1.0).unwrap();
        simulator.validate_read("A:TEST", Some("number")).unwrap();
        assert_eq!(simulator.read("A:TEST", Some("number")).unwrap(), 2.5);
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
    fn builds_aircraft_and_local_variable_reads() {
        let mut output = String::new();

        build_read_calculator_code(&mut output, "A:PLANE PITCH DEGREES", Some("radians")).unwrap();
        assert_eq!(output, "(A:PLANE PITCH DEGREES, radians)");

        build_read_calculator_code(&mut output, "L:EXAMPLE", None).unwrap();
        assert_eq!(output, "(L:EXAMPLE)");

        assert!(matches!(
            build_read_calculator_code(&mut output, "A:TEST", None),
            Err(SimulatorError::MissingReadUnit { .. })
        ));
        assert!(matches!(
            build_read_calculator_code(&mut output, "L:TEST", Some("number")),
            Err(SimulatorError::UnexpectedReadUnit { .. })
        ));
        assert!(matches!(
            build_read_calculator_code(&mut output, "K:EVENT", None),
            Err(SimulatorError::UnsupportedReadVariable { .. })
        ));
    }

    #[test]
    fn reuses_and_clears_the_output_buffer() {
        let mut simulator = MsfsSimulator::new();
        simulator.calculator_code_buffer.reserve(128);
        let capacity = simulator.calculator_code_buffer.capacity();

        build_calculator_code(
            &mut simulator.calculator_code_buffer,
            "K:AXIS_AILERONS_SET",
            16384.0,
        )
        .unwrap();
        build_calculator_code(&mut simulator.calculator_code_buffer, "L:X", 0.0).unwrap();

        assert_eq!(simulator.calculator_code_buffer, "0 (>L:X)");
        assert_eq!(simulator.calculator_code_buffer.capacity(), capacity);
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
