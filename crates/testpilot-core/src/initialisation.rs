//! Simulator-independent initialisation targets and deadlines.

use std::time::Duration;

use crate::config::InitialisationConfig;
use crate::error::InitialisationError;

/// Bounded readiness state for one armed run.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Initialisation {
    /// Demanded aircraft values.
    targets: InitialisationConfig,
    /// Simulator timestamp on the arm frame.
    armed_at: Duration,
    /// Most recent checked timestamp, used to reject backwards time.
    previous_time: Duration,
}

impl Initialisation {
    /// Targets to resubmit while the readiness gate is waiting.
    pub const fn targets(&self) -> InitialisationConfig {
        self.targets
    }

    /// Begins a 30-second simulator-time deadline at the arm frame.
    pub const fn new(targets: InitialisationConfig, armed_at: Duration) -> Self {
        Self {
            targets,
            armed_at,
            previous_time: armed_at,
        }
    }

    /// Checks time before readback so timeout takes precedence at exactly 30 seconds.
    pub fn timed_out(&mut self, now: Duration) -> Result<(), InitialisationError> {
        if now < self.previous_time {
            return Err(InitialisationError::ClockMovedBackwards {
                previous: self.previous_time,
                current: now,
            });
        }
        self.previous_time = now;
        if now - self.armed_at >= Duration::from_secs(30) {
            return Err(InitialisationError::Timeout {
                targets: Box::new(self.targets),
            });
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TARGETS: InitialisationConfig = InitialisationConfig {
        zfw: Some(60000.0),
        gw: Some(65000.0),
        gwcg: Some(25.0),
        ths: None,
        flaps: None,
        landing_gear: None,
    };

    #[test]
    fn deadline_uses_elapsed_simulator_time_and_reports_targets() {
        for elapsed in [Duration::from_secs(30), Duration::from_secs(35)] {
            let mut gate = Initialisation::new(TARGETS, Duration::from_secs(100));
            gate.timed_out(Duration::from_millis(129999)).unwrap();
            assert!(matches!(
                gate.timed_out(Duration::from_secs(100) + elapsed),
                Err(InitialisationError::Timeout { targets }) if *targets == TARGETS
            ));
        }
    }

    #[test]
    fn rejects_clock_reversal_even_after_the_arm_timestamp() {
        let mut gate = Initialisation::new(TARGETS, Duration::from_secs(100));
        gate.timed_out(Duration::from_secs(110)).unwrap();
        assert!(matches!(gate.timed_out(Duration::from_secs(109)),
            Err(InitialisationError::ClockMovedBackwards { previous, current })
                if previous == Duration::from_secs(110) && current == Duration::from_secs(109)));
    }
}
