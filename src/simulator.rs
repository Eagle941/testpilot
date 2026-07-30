//! MSFS simulator-variable writes through legacy calculator code.

use std::fmt::Write;

use crate::error::SimulatorError;

/// Reusable calculator-code buffer for per-frame simulator writes.
pub(crate) struct VariableWriter {
    calculator_code: String,
}

impl VariableWriter {
    /// Creates an empty writer whose buffer is retained between writes.
    pub(crate) const fn new() -> Self {
        Self {
            calculator_code: String::new(),
        }
    }

    /// Writes a finite value to a prefixed `K:` event or `L:` variable.
    #[cfg(target_arch = "wasm32")]
    pub(crate) fn write(&mut self, variable: &str, value: f64) -> Result<(), SimulatorError> {
        build_calculator_code(&mut self.calculator_code, variable, value)?;
        msfs::legacy::execute_calculator_code::<()>(&self.calculator_code).ok_or_else(|| {
            SimulatorError::CalculatorCodeWriteFailed {
                variable: variable.to_owned(),
                value,
            }
        })
    }
}

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
    use crate::error::SimulatorError;

    use super::{build_calculator_code, VariableWriter};

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
        let mut writer = VariableWriter::new();
        writer.calculator_code.reserve(128);
        let capacity = writer.calculator_code.capacity();

        build_calculator_code(&mut writer.calculator_code, "K:AXIS_AILERONS_SET", 16384.0).unwrap();
        build_calculator_code(&mut writer.calculator_code, "L:X", 0.0).unwrap();

        assert_eq!(writer.calculator_code, "0 (>L:X)");
        assert_eq!(writer.calculator_code.capacity(), capacity);
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
