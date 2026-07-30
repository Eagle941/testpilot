//! MSFS simulator-variable writes through legacy calculator code.

use std::fmt::Write;

use anyhow::{Context, bail};

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
    pub(crate) fn write(&mut self, variable: &str, value: f64) -> anyhow::Result<()> {
        build_calculator_code(&mut self.calculator_code, variable, value)?;
        msfs::legacy::execute_calculator_code::<()>(&self.calculator_code).ok_or_else(|| {
            anyhow::anyhow!("calculator code failed while writing {value} to `{variable}`")
        })
    }
}

fn build_calculator_code(output: &mut String, variable: &str, value: f64) -> anyhow::Result<()> {
    if !value.is_finite() {
        bail!("cannot write non-finite value {value} to `{variable}`");
    }
    if !matches!(variable.as_bytes(), [b'K' | b'L', b':', _, ..])
        || variable.as_bytes().contains(&0)
    {
        bail!("unsupported simulator variable `{variable}`; expected a non-empty K: or L: prefix");
    }

    output.clear();
    write!(output, "{value} (>{variable})")
        .with_context(|| format!("failed to build calculator code for `{variable}`"))
}

#[cfg(test)]
mod tests {
    use super::{VariableWriter, build_calculator_code};

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

        assert!(build_calculator_code(&mut output, "AXIS_ELEVATOR_SET", 0.0).is_err());
        assert!(build_calculator_code(&mut output, "K:", 0.0).is_err());
        assert!(build_calculator_code(&mut output, "A:ELEVATOR POSITION", 0.0).is_err());
        assert!(build_calculator_code(&mut output, "L:TEST", f64::NAN).is_err());
        assert!(build_calculator_code(&mut output, "L:BAD\0NAME", 0.0).is_err());
    }
}
