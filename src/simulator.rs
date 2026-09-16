//! MSFS simulator-variable writes through legacy calculator code.

use std::ffi::{CStr, CString};
#[cfg(any(target_arch = "wasm32", test, feature = "bench-support"))]
use std::fmt::Write;
use std::time::Duration;

use crate::error::SimulatorError;

/// Static, NUL-terminated simulator-clock command; no per-frame conversion.
pub(crate) const SIMULATION_TIME_CODE: &CStr = c"(E:SIMULATION TIME, seconds)";

/// One validated read, keyed by both variable and unit to preserve unit semantics.
struct ReadCommand {
    /// Configured simulator source.
    variable: String,
    /// Configured aircraft-variable unit, if any.
    unit: Option<String>,
    /// Owned command bytes passed unchanged to msfs-rs on each read.
    code: CString,
}

/// Owned read commands reused for one replay configuration.
///
/// Entries contain text only, so dropping or clearing the cache needs no simulator
/// calls. The runtime clears it between runs to discard previous configurations.
pub struct ReadCommandCache {
    /// Small configured recording set, searched without allocating lookup keys.
    /// A Vec avoids hashing overhead for the usual four recordings. Host benchmarks
    /// with mixed and shared-prefix names favored Vec at 4, 8 and 16 entries;
    /// HashMap won at 32. Revisit if larger recording sets become typical; the
    /// crossover depends on key lengths and the target platform.
    commands: Vec<ReadCommand>,
    /// Scratch space used only when preparing a previously unseen read.
    scratch: String,
}

impl ReadCommandCache {
    /// Creates an empty cache without allocating.
    pub const fn new() -> Self {
        Self {
            commands: Vec::new(),
            scratch: String::new(),
        }
    }

    /// Releases cached commands while retaining the vector's capacity.
    pub fn clear(&mut self) {
        self.commands.clear();
    }

    /// Returns validated NUL-terminated code, preparing it once on a cache miss.
    pub fn get(&mut self, variable: &str, unit: Option<&str>) -> Result<&CStr, SimulatorError> {
        if variable == "L:REPLAYER_ARMED" && unit.is_none() {
            return Ok(c"(L:REPLAYER_ARMED)");
        }
        let index = match self
            .commands
            .iter()
            .position(|command| command.variable == variable && command.unit.as_deref() == unit)
        {
            Some(index) => index,
            None => {
                build_read_calculator_code(&mut self.scratch, variable, unit)?;
                let code = CString::new(self.scratch.as_str()).map_err(|source| {
                    SimulatorError::CalculatorCodeNul {
                        variable: variable.to_owned(),
                        source,
                    }
                })?;
                let index = self.commands.len();
                self.commands.push(ReadCommand {
                    variable: variable.to_owned(),
                    unit: unit.map(str::to_owned),
                    code,
                });
                index
            }
        };
        Ok(self.commands[index].code.as_c_str())
    }
}

impl Default for ReadCommandCache {
    fn default() -> Self {
        Self::new()
    }
}

/// Simulator operations required by replay injection.
pub trait SimulatorAdapter {
    /// Returns the current simulator-clock time.
    fn simulation_time(&self) -> Result<Duration, SimulatorError>;

    /// Writes a value to a prefixed simulator destination.
    fn write(&mut self, variable: &str, value: f64) -> Result<(), SimulatorError>;

    /// Validates that a prefixed simulator source can be read with the given unit.
    fn validate_read(&mut self, variable: &str, unit: Option<&str>) -> Result<(), SimulatorError>;

    /// Reads a finite value from a prefixed simulator source.
    fn read(&mut self, variable: &str, unit: Option<&str>) -> Result<f64, SimulatorError>;

    /// Discards commands belonging to an earlier run or configuration.
    fn clear_read_cache(&mut self) {}
}

/// MSFS implementation backed by legacy calculator code.
pub struct MsfsSimulator {
    /// Reusable calculator-code scratch buffer for dynamic write commands.
    #[cfg(any(target_arch = "wasm32", test))]
    calculator_code_buffer: String,
    /// Validated recording commands owned until the run ends.
    #[cfg(target_arch = "wasm32")]
    read_commands: ReadCommandCache,
}

impl MsfsSimulator {
    /// Creates an adapter with reusable write storage and a recording-command cache.
    pub const fn new() -> MsfsSimulator {
        MsfsSimulator {
            #[cfg(any(target_arch = "wasm32", test))]
            calculator_code_buffer: String::new(),
            #[cfg(target_arch = "wasm32")]
            read_commands: ReadCommandCache::new(),
        }
    }
}

#[cfg(target_arch = "wasm32")]
impl SimulatorAdapter for MsfsSimulator {
    fn simulation_time(&self) -> Result<Duration, SimulatorError> {
        let value = execute_read_code(SIMULATION_TIME_CODE)
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
        self.read_commands.get(variable, unit)?;
        Ok(())
    }

    fn read(&mut self, variable: &str, unit: Option<&str>) -> Result<f64, SimulatorError> {
        let code = self.read_commands.get(variable, unit)?;
        let value =
            execute_read_code(code).ok_or_else(|| SimulatorError::CalculatorCodeReadFailed {
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

    fn clear_read_cache(&mut self) {
        self.read_commands.clear();
    }
}

/// Calls the same msfs-rs implementation as execute_calculator_code, using
/// already-owned command bytes instead of allocating a CString on every read.
/// This doc-hidden public trait is available in the revision pinned by Cargo.lock.
#[cfg(target_arch = "wasm32")]
fn execute_read_code(code: &CStr) -> Option<f64> {
    <f64 as msfs::legacy::ExecuteCalculatorCodeImpl>::execute(code)
}

/// Formats one finite value write for a prefixed `K:` event or `L:` variable.
///
/// The output buffer is cleared and reused. Invalid destinations and non-finite
/// values are rejected before any calculator code is produced.
#[cfg(any(target_arch = "wasm32", test, feature = "bench-support"))]
pub(crate) fn build_calculator_code(
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
#[cfg(any(target_arch = "wasm32", test, feature = "bench-support"))]
pub(crate) fn build_read_calculator_code(
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
        MsfsSimulator, ReadCommandCache, SIMULATION_TIME_CODE, SimulatorAdapter,
        build_calculator_code, build_read_calculator_code,
    };

    #[test]
    fn caches_commands_by_variable_and_unit_and_releases_old_configuration() {
        let mut cache = ReadCommandCache::new();
        let original = cache
            .get("A:PLANE PITCH DEGREES", Some("degrees"))
            .unwrap()
            .as_ptr();
        assert_eq!(
            cache.get("A:PLANE PITCH DEGREES", Some("radians")).unwrap(),
            c"(A:PLANE PITCH DEGREES, radians)"
        );
        assert_eq!(
            cache
                .get("A:PLANE PITCH DEGREES", Some("degrees"))
                .unwrap()
                .as_ptr(),
            original
        );
        assert_eq!(cache.get("L:TEST", None).unwrap(), c"(L:TEST)");
        assert_eq!(cache.commands.len(), 3);
        cache.clear();
        assert!(cache.commands.is_empty());
        assert_eq!(
            cache.get("A:NEW", Some("number")).unwrap(),
            c"(A:NEW, number)"
        );
        assert_eq!(cache.commands.len(), 1);
    }

    #[test]
    fn cached_reads_preserve_validation_and_static_commands() {
        let mut cache = ReadCommandCache::new();
        assert_eq!(SIMULATION_TIME_CODE, c"(E:SIMULATION TIME, seconds)");
        assert_eq!(
            cache.get("L:REPLAYER_ARMED", None).unwrap(),
            c"(L:REPLAYER_ARMED)"
        );
        assert!(cache.commands.is_empty());
        cache.get("A:TEST", Some("number")).unwrap();
        for (variable, unit) in [
            ("A:TEST", None),
            ("A:TEST", Some("")),
            ("A:TEST", Some("nu\0mber")),
            ("L:REPLAYER_ARMED", Some("number")),
            ("L:BAD\0NAME", None),
            ("K:EVENT", None),
        ] {
            assert!(cache.get(variable, unit).is_err());
        }
        assert_eq!(cache.commands.len(), 1);
        assert_eq!(
            cache.get("A:TEST", Some("number")).unwrap(),
            c"(A:TEST, number)"
        );
    }

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
