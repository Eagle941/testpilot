//! MSFS simulator-variable writes through legacy calculator code.

use std::fmt::Write;
use std::time::Duration;

use crate::error::SimulatorError;

/// Packed calculator code used to query simulation time.
#[cfg(target_arch = "wasm32")]
const SIMULATION_TIME_CODE: &str = "(E:SIMULATION TIME, seconds)";

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

/// MSFS implementation backed by legacy calculator code.
pub struct MsfsSimulator {
    /// Reusable calculator-code scratch buffer for write/read commands.
    calculator_code_buffer: String,
}

impl MsfsSimulator {
    /// Creates an adapter with a reusable calculator-code buffer.
    pub const fn new() -> MsfsSimulator {
        MsfsSimulator {
            calculator_code_buffer: String::new(),
        }
    }
}

#[cfg(target_arch = "wasm32")]
impl SimulatorAdapter for MsfsSimulator {
    fn local_variable_exists(&mut self, variable: &str) -> Result<bool, SimulatorError> {
        let name = local_variable_name(variable)?;
        // SAFETY: name is NUL-terminated and remains alive throughout the SDK call.
        // Unlike register_named_variable, this lookup never creates a variable.
        Ok(unsafe { msfs::sys::check_named_variable(name.as_ptr()) } != -1)
    }

    fn read_string(&mut self, variable: &str) -> Result<String, SimulatorError> {
        self.validate_read(variable, Some("string"))?;
        let code = std::ffi::CString::new(self.calculator_code_buffer.as_str()).map_err(|_| {
            SimulatorError::UnsupportedReadVariable {
                variable: variable.to_owned(),
            }
        })?;
        let mut value = std::ptr::null();
        // SAFETY: code is NUL-terminated and lives through the synchronous SDK call;
        // value is a valid output pointer. The SDK owns the returned string.
        let success = unsafe {
            msfs::sys::execute_calculator_code(
                code.as_ptr(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                &mut value,
            )
        };
        let value = if success != 0 && !value.is_null() {
            // SAFETY: on success the SDK supplies a NUL-terminated string. Copy it
            // before another SDK call. Check null and UTF-8 instead of using the
            // pinned legacy String wrapper, which assumes both are valid.
            Some(unsafe { std::ffi::CStr::from_ptr(value) })
        } else {
            None
        };
        decode_string_read(value, variable)
    }

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

/// Converts a prefixed local variable into the SDK's unprefixed, NUL-terminated name.
fn local_variable_name(variable: &str) -> Result<std::ffi::CString, SimulatorError> {
    variable
        .strip_prefix("L:")
        .filter(|name| !name.is_empty())
        .and_then(|name| std::ffi::CString::new(name).ok())
        .ok_or_else(|| SimulatorError::UnsupportedReadVariable {
            variable: variable.to_owned(),
        })
}

/// Converts an SDK string result, reporting missing or invalid UTF-8 values without panicking.
fn decode_string_read(
    value: Option<&std::ffi::CStr>,
    variable: &str,
) -> Result<String, SimulatorError> {
    value
        .and_then(|value| value.to_str().ok())
        .map(str::to_owned)
        .ok_or_else(|| SimulatorError::CalculatorCodeReadFailed {
            variable: variable.to_owned(),
        })
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
    match variable.as_bytes() {
        [b'K' | b'L', b':', _, ..] => {}
        _ => {
            return Err(SimulatorError::UnsupportedVariable {
                variable: variable.to_owned(),
            });
        }
    }
    if variable.as_bytes().contains(&0) {
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
        fn local_variable_exists(&mut self, _variable: &str) -> Result<bool, SimulatorError> {
            Ok(false)
        }

        fn read_string(&mut self, variable: &str) -> Result<String, SimulatorError> {
            Err(SimulatorError::UnsupportedReadVariable {
                variable: variable.to_owned(),
            })
        }

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
    fn validates_local_variable_names_for_non_registering_lookup() {
        assert_eq!(
            super::local_variable_name("L:A32NX_IS_READY")
                .unwrap()
                .as_c_str(),
            c"A32NX_IS_READY"
        );
        for variable in ["", "L:", "A:ATC MODEL", "A32NX_IS_READY", "L:NAME\0SUFFIX"] {
            assert!(matches!(
                super::local_variable_name(variable),
                Err(SimulatorError::UnsupportedReadVariable { .. })
            ));
        }
    }

    #[test]
    fn string_reads_use_the_sdk_string_unit_and_reject_unavailable_or_invalid_results() {
        let mut code = String::new();
        build_read_calculator_code(&mut code, "A:ATC MODEL", Some("string")).unwrap();
        assert_eq!(code, "(A:ATC MODEL, string)");
        assert_eq!(
            super::decode_string_read(Some(c"A20N"), "A:ATC MODEL").unwrap(),
            "A20N"
        );
        assert_eq!(
            super::decode_string_read(Some(c""), "A:ATC MODEL").unwrap(),
            ""
        );
        let invalid = c"\xff";
        for value in [None, Some(invalid)] {
            assert_eq!(
                super::decode_string_read(value, "A:ATC MODEL"),
                Err(SimulatorError::CalculatorCodeReadFailed {
                    variable: "A:ATC MODEL".to_owned()
                })
            );
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

        assert_eq!(
            build_read_calculator_code(&mut output, "A:TEST", None),
            Err(SimulatorError::MissingReadUnit {
                variable: "A:TEST".to_owned(),
            })
        );
        assert_eq!(
            build_read_calculator_code(&mut output, "L:TEST", Some("number")),
            Err(SimulatorError::UnexpectedReadUnit {
                variable: "L:TEST".to_owned(),
            })
        );
        assert_eq!(
            build_read_calculator_code(&mut output, "K:EVENT", None),
            Err(SimulatorError::UnsupportedReadVariable {
                variable: "K:EVENT".to_owned(),
            })
        );
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
            assert_eq!(
                build_calculator_code(&mut output, variable, 0.0),
                Err(SimulatorError::UnsupportedVariable {
                    variable: variable.to_owned(),
                })
            );
        }
        match build_calculator_code(&mut output, "L:TEST", f64::NAN) {
            Err(SimulatorError::NonFiniteWrite { variable, value })
                if variable == "L:TEST" && value.is_nan() => {}
            unexpected => panic!("expected non-finite write error, got: {unexpected:?}"),
        }
    }
}
