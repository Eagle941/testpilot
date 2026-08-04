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

#[cfg(test)]
mod tests {
    use super::PositiveTrigger;

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
}
