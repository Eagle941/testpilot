use crate::error::SimulatorError;
use crate::simulator::SimulatorAdapter;

/// Tracks sampled arming values and identifies a zero-to-one transition.
#[derive(Debug, Default)]
pub struct PositiveTrigger {
    /// Previously sampled arming value, used for edge detection.
    previous: f64,
}

impl PositiveTrigger {
    /// Updates the sampled value and reports whether replay should start.
    pub fn start(&mut self, current: f64) -> bool {
        let start = self.previous == 0.0 && current == 1.0;
        self.previous = current;
        start
    }
}

/// Tracks and applies the configured arming variable for simulator replay state.
#[derive(Debug)]
pub struct ArmingMonitor {
    /// Simulator variable name to read/write for arming.
    variable: String,
    /// Most recently sampled arming value.
    armed_value: f64,
    /// Transition detector for a 0->1 arming edge.
    trigger: PositiveTrigger,
}

impl ArmingMonitor {
    /// Creates a new monitor bound to the given local simulator variable.
    pub fn new(variable: impl Into<String>) -> Self {
        Self {
            variable: variable.into(),
            armed_value: 0.0,
            trigger: PositiveTrigger::default(),
        }
    }

    /// Reads the arming value, updates transition tracking, and reports whether
    /// the run should start from this frame.
    pub fn ready_to_start<S: SimulatorAdapter>(
        &mut self,
        simulator: &mut S,
    ) -> Result<bool, SimulatorError> {
        let armed = simulator.read(&self.variable, None)?;
        let starting = self.trigger.start(armed);
        self.armed_value = armed;
        Ok(starting)
    }

    /// Resets the arming variable to `0.0`.
    pub fn reset<S: SimulatorAdapter>(&self, simulator: &mut S) -> Result<(), SimulatorError> {
        simulator.write(&self.variable, 0.0)
    }
}

#[cfg(test)]
mod tests {
    use crate::simulator::SimulatorAdapter;

    use super::{ArmingMonitor, PositiveTrigger};
    use crate::error::SimulatorError;

    struct FakeSimulator {
        values: Vec<f64>,
        writes: Vec<f64>,
    }

    impl FakeSimulator {
        fn new() -> FakeSimulator {
            Self {
                values: vec![0.0],
                writes: Vec::new(),
            }
        }
    }

    impl SimulatorAdapter for FakeSimulator {
        fn simulation_time(&self) -> Result<std::time::Duration, SimulatorError> {
            unreachable!()
        }

        fn write(&mut self, _variable: &str, value: f64) -> Result<(), SimulatorError> {
            self.writes.push(value);
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
        let mut state = PositiveTrigger::default();

        assert!(state.start(1.0));
    }

    #[test]
    fn does_not_start_without_a_zero_to_one_transition() {
        let mut state = PositiveTrigger::default();

        assert!(!state.start(0.0));
        assert!(state.start(1.0));
        assert!(!state.start(1.0));
        assert!(!state.start(0.0));
        assert!(!state.start(0.5));
        assert!(!state.start(1.0));
    }

    #[test]
    fn reads_and_tracks_arming_transitions() {
        let mut simulator = FakeSimulator::new();
        let mut monitor = ArmingMonitor::new("L:REPLAYER_ARMED");

        assert!(!monitor.ready_to_start(&mut simulator).unwrap());
        assert_eq!(monitor.armed_value, 0.0);
        simulator.values.push(1.0);
        assert!(monitor.ready_to_start(&mut simulator).unwrap());
        assert_eq!(monitor.armed_value, 1.0);
        simulator.values.push(1.0);
        assert!(!monitor.ready_to_start(&mut simulator).unwrap());
        assert_eq!(monitor.armed_value, 1.0);
        simulator.values.push(0.0);
        assert!(!monitor.ready_to_start(&mut simulator).unwrap());
        assert_eq!(monitor.armed_value, 0.0);
    }

    #[test]
    fn resetting_sets_the_arming_variable_to_zero() {
        let mut simulator = FakeSimulator::new();
        let monitor = ArmingMonitor::new("L:REPLAYER_ARMED");

        monitor.reset(&mut simulator).unwrap();
        assert_eq!(simulator.writes, vec![0.0]);
    }
}
