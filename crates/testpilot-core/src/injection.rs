//! Conversion and simulator injection of interpolated scenario inputs.

use crate::cursor::ReplayInput;
use crate::error::{InjectionError, InterpolationError};
use crate::simulator::SimulatorAdapter;

/// Injects one frame in configuration order and retains its converted values.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct InputInjector {
    /// One reusable converted-value slot per configured input.
    values: Vec<f64>,
}

impl InputInjector {
    /// Allocates the fixed buffer for one prepared scenario's input count.
    pub(crate) fn new(input_count: usize) -> Self {
        Self {
            values: vec![0.0; input_count],
        }
    }

    /// Converts and writes each input as it is consumed, stopping at the first error.
    ///
    /// The input iterator and buffer describe the same configured scenario. Values
    /// are available for telemetry only after this entire operation succeeds.
    pub(crate) fn apply<'a>(
        &mut self,
        inputs: impl Iterator<Item = Result<ReplayInput<'a>, InterpolationError>>,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<(), InjectionError> {
        for (slot, input) in self.values.iter_mut().zip(inputs) {
            let input = input?;
            let value = input.conversion.convert(input.value).map_err(|source| {
                InjectionError::ConvertSignal {
                    signal: input.signal.to_owned(),
                    source,
                }
            })?;
            simulator.write(input.variable, value).map_err(|source| {
                InjectionError::InjectSignal {
                    signal: input.signal.to_owned(),
                    source,
                }
            })?;
            *slot = value;
        }
        Ok(())
    }

    /// Converted values for telemetry after a successful apply, in column order.
    pub(crate) fn values(&self) -> &[f64] {
        &self.values
    }
}

#[cfg(test)]
mod tests {
    use std::cell::Cell;
    use std::time::Duration;

    use crate::error::{PlaybackError, SimulatorError};
    use crate::playback::AffineRange;

    use super::*;

    #[derive(Debug, Clone, Default, PartialEq)]
    struct FakeSimulator {
        writes: Vec<(String, f64)>,
        fail_on: Option<&'static str>,
    }

    impl SimulatorAdapter for FakeSimulator {
        fn local_variable_exists(&mut self, _variable: &str) -> Result<bool, SimulatorError> {
            unreachable!("injection does not detect aircraft")
        }

        fn read_string(&mut self, _variable: &str) -> Result<String, SimulatorError> {
            unreachable!("injection does not read strings")
        }

        fn simulation_time(&self) -> Result<Duration, SimulatorError> {
            unreachable!("injection receives inputs already evaluated at scenario time")
        }

        fn write(&mut self, variable: &str, value: f64) -> Result<(), SimulatorError> {
            self.writes.push((variable.to_owned(), value));
            if self.fail_on == Some(variable) {
                return Err(SimulatorError::CalculatorCodeWriteFailed {
                    variable: variable.to_owned(),
                    value,
                });
            }
            Ok(())
        }

        fn validate_read(
            &mut self,
            _variable: &str,
            _unit: Option<&str>,
        ) -> Result<(), SimulatorError> {
            unreachable!("injection does not validate recording sources")
        }

        fn read(&mut self, _variable: &str, _unit: Option<&str>) -> Result<f64, SimulatorError> {
            unreachable!("injection does not sample telemetry")
        }
    }

    fn input(signal: &'static str, variable: &'static str, value: f64) -> ReplayInput<'static> {
        ReplayInput {
            signal,
            variable,
            value,
            conversion: AffineRange::new([-100.0, 100.0], [-1.0, 1.0]).unwrap(),
        }
    }

    #[test]
    fn converts_in_order_retains_values_and_preserves_clamping_with_a_fixed_buffer() {
        let mut injector = InputInjector::new(2);
        let buffer = injector.values().as_ptr();
        let mut simulator = FakeSimulator::default();

        for (pitch, roll) in [(50.0, -100.0), (250.0, -250.0)] {
            injector
                .apply(
                    [
                        Ok(input("pitch", "K:PITCH", pitch)),
                        Ok(input("roll", "K:ROLL", roll)),
                    ]
                    .into_iter(),
                    &mut simulator,
                )
                .unwrap();
            assert_eq!(injector.values().len(), 2);
            assert_eq!(injector.values().as_ptr(), buffer);
        }

        assert_eq!(
            simulator.writes,
            vec![
                ("K:PITCH".to_owned(), 0.5),
                ("K:ROLL".to_owned(), -1.0),
                ("K:PITCH".to_owned(), 1.0),
                ("K:ROLL".to_owned(), -1.0),
            ]
        );
        assert_eq!(injector.values(), &[1.0, -1.0]);
    }

    #[test]
    fn input_errors_retain_signal_context_and_stop_consuming_and_writing_later_inputs() {
        for failure in 0..3 {
            let mut injector = InputInjector::new(3);
            let mut simulator = FakeSimulator::default();
            let mut second = input("second", "L:SECOND", 0.0);
            let second = match failure {
                0 => Err(InterpolationError::InterpolateSignal {
                    signal: "second".to_owned(),
                    source: PlaybackError::ArithmeticOverflow,
                }),
                1 => {
                    second.conversion =
                        AffineRange::new([-1.0, 1.0], [-f64::MAX, f64::MAX]).unwrap();
                    Ok(second)
                }
                _ => {
                    simulator.fail_on = Some("L:SECOND");
                    Ok(second)
                }
            };
            let consumed = Cell::new(0);
            let inputs = [
                Ok(input("first", "L:FIRST", 50.0)),
                second,
                Ok(input("third", "L:THIRD", 100.0)),
            ]
            .into_iter()
            .inspect(|_| consumed.set(consumed.get() + 1));

            let error = injector.apply(inputs, &mut simulator).unwrap_err();
            assert!(error.to_string().contains("second"));
            assert!(matches!(
                (failure, error),
                (
                    0,
                    InjectionError::Interpolate(InterpolationError::InterpolateSignal {
                        source: PlaybackError::ArithmeticOverflow,
                        ..
                    })
                ) | (
                    1,
                    InjectionError::ConvertSignal {
                        source: PlaybackError::ArithmeticOverflow,
                        ..
                    }
                ) | (2, InjectionError::InjectSignal { .. })
            ));
            assert_eq!(consumed.get(), 2);
            let mut expected = vec![("L:FIRST".to_owned(), 0.5)];
            if failure == 2 {
                expected.push(("L:SECOND".to_owned(), 0.0));
            }
            assert_eq!(simulator.writes, expected);
        }
    }
}
