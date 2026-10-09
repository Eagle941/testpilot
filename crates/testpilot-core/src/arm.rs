use crate::error::SimulatorError;
use crate::simulator::SimulatorAdapter;

/// Tracks and applies the configured arming variable for simulator replay state.
#[derive(Debug, Clone, PartialEq)]
pub struct ArmingMonitor {
    /// Simulator variable name to read/write for arming.
    variable: String,
    /// Previously sampled arming value, used for zero-to-one edge detection.
    previous: f64,
    /// A failed reset must succeed before another arming value is accepted.
    reset_pending: bool,
}

impl ArmingMonitor {
    /// Creates a new monitor bound to the given local simulator variable.
    pub fn new(variable: impl Into<String>) -> Self {
        Self {
            variable: variable.into(),
            previous: 0.0,
            reset_pending: false,
        }
    }

    /// Reads the arming value, updates transition tracking, and reports whether
    /// the run should start from this frame.
    pub fn trigger_initialise(
        &mut self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<bool, SimulatorError> {
        if self.reset_pending {
            self.reset(simulator)?;
            return Ok(false);
        }
        let armed = simulator.read(&self.variable, None)?;
        let start = self.previous == 0.0 && armed == 1.0;
        self.previous = armed;
        Ok(start)
    }

    /// Resets arming to zero; failures are retried before accepting another start.
    pub fn reset(&mut self, simulator: &mut dyn SimulatorAdapter) -> Result<(), SimulatorError> {
        self.reset_pending = true;
        simulator.write(&self.variable, 0.0)?;
        self.previous = 0.0;
        self.reset_pending = false;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::simulator::SimulatorAdapter;

    use super::ArmingMonitor;
    use crate::error::SimulatorError;

    #[derive(Debug, Clone, PartialEq)]
    struct FakeSimulator {
        values: Vec<f64>,
        writes: Vec<f64>,
        fail_write: bool,
    }

    impl FakeSimulator {
        fn new() -> FakeSimulator {
            Self {
                values: vec![0.0],
                writes: Vec::new(),
                fail_write: false,
            }
        }
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

        fn simulation_time(&self) -> Result<std::time::Duration, SimulatorError> {
            unreachable!()
        }

        fn write(&mut self, variable: &str, value: f64) -> Result<(), SimulatorError> {
            self.writes.push(value);
            if self.fail_write {
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
            Ok(())
        }

        fn read(&mut self, _variable: &str, _unit: Option<&str>) -> Result<f64, SimulatorError> {
            self.values
                .pop()
                .ok_or_else(|| SimulatorError::NonFiniteRead {
                    variable: "L:REPLAYER_ARMED".to_owned(),
                    value: f64::NAN,
                })
        }
    }

    #[test]
    fn starts_when_armed_changes_from_zero_to_one() {
        let mut simulator = FakeSimulator::new();
        let mut monitor = ArmingMonitor::new("L:REPLAYER_ARMED");
        simulator.values.push(1.0);

        assert!(monitor.trigger_initialise(&mut simulator).unwrap());
    }

    #[test]
    fn does_not_start_without_a_zero_to_one_transition() {
        let mut simulator = FakeSimulator::new();
        let mut monitor = ArmingMonitor::new("L:REPLAYER_ARMED");

        for (value, start) in [
            (0.0, false),
            (1.0, true),
            (1.0, false),
            (0.0, false),
            (0.5, false),
            (1.0, false),
        ] {
            simulator.values.push(value);
            assert_eq!(monitor.trigger_initialise(&mut simulator).unwrap(), start);
        }
    }

    #[test]
    fn reads_and_tracks_arming_transitions() {
        let mut simulator = FakeSimulator::new();
        let mut monitor = ArmingMonitor::new("L:REPLAYER_ARMED");

        assert!(!monitor.trigger_initialise(&mut simulator).unwrap());
        simulator.values.push(1.0);
        assert!(monitor.trigger_initialise(&mut simulator).unwrap());
        simulator.values.push(1.0);
        assert!(!monitor.trigger_initialise(&mut simulator).unwrap());
        simulator.values.push(0.0);
        assert!(!monitor.trigger_initialise(&mut simulator).unwrap());
    }

    #[test]
    fn resetting_sets_the_arming_variable_to_zero() {
        let mut simulator = FakeSimulator::new();
        let mut monitor = ArmingMonitor::new("L:REPLAYER_ARMED");

        monitor.reset(&mut simulator).unwrap();
        assert_eq!(simulator.writes, vec![0.0]);
    }

    #[test]
    fn reset_allows_rearming_before_another_idle_frame() {
        let mut simulator = FakeSimulator::new();
        let mut monitor = ArmingMonitor::new("L:REPLAYER_ARMED");
        simulator.values.push(1.0);
        assert!(monitor.trigger_initialise(&mut simulator).unwrap());

        monitor.reset(&mut simulator).unwrap();
        simulator.values.push(1.0);
        assert!(monitor.trigger_initialise(&mut simulator).unwrap());
    }

    #[test]
    fn failed_reset_is_retried_before_accepting_an_unobserved_arming_edge() {
        let mut simulator = FakeSimulator::new();
        let mut monitor = ArmingMonitor::new("L:REPLAYER_ARMED");
        simulator.values = vec![1.0];
        simulator.fail_write = true;

        assert!(monitor.reset(&mut simulator).is_err());
        assert!(monitor.trigger_initialise(&mut simulator).is_err());
        assert_eq!(simulator.values, vec![1.0]);

        simulator.fail_write = false;
        assert!(!monitor.trigger_initialise(&mut simulator).unwrap());
        assert_eq!(simulator.writes, vec![0.0; 3]);
        // The successful reset leaves the actual simulator variable at zero.
        simulator.values = vec![0.0];
        assert!(!monitor.trigger_initialise(&mut simulator).unwrap());
        simulator.values.push(1.0);
        assert!(monitor.trigger_initialise(&mut simulator).unwrap());
    }
}
