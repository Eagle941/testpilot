//! Simulator-independent initialisation targets and deadlines.

use std::time::Duration;

use crate::aircraft_initialisation::INITIALISATION_TIMEOUT;
use crate::config::InitialisationConfig;
use crate::error::InitialisationError;

/// Bounded readiness state for one armed run.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Initialisation {
    /// Demanded aircraft values.
    targets: InitialisationConfig,
    /// Configured parameters not ready at the latest completed observation.
    pending: InitialisationConfig,
    /// Simulator timestamp on the arm frame.
    armed_at: Duration,
    /// Most recent checked timestamp, used to reject backwards time.
    previous_time: Duration,
    /// Fixed per-run budget; absent until the first successful setup observation.
    timeout: Option<Duration>,
}

impl Initialisation {
    /// Targets to resubmit while the readiness gate is waiting.
    pub const fn targets(&self) -> InitialisationConfig {
        self.targets
    }

    /// Starts the deadline at arming, using the base allowance until setup is observed.
    pub const fn new(targets: InitialisationConfig, armed_at: Duration) -> Self {
        Self {
            targets,
            pending: targets,
            armed_at,
            previous_time: armed_at,
            timeout: None,
        }
    }

    /// Retains the latest pending subset without sampling or commanding the simulator.
    pub fn update_pending(&mut self, pending: InitialisationConfig) {
        self.pending = pending;
    }

    /// Accepts the aircraft's planned travel budget once, without extending it on progress.
    pub fn set_timeout(&mut self, timeout: Duration) {
        self.timeout.get_or_insert(timeout);
    }

    /// Checks time before readback so timeout takes precedence at the fixed deadline.
    pub fn timed_out(&mut self, now: Duration) -> Result<(), InitialisationError> {
        if now < self.previous_time {
            return Err(InitialisationError::ClockMovedBackwards {
                previous: self.previous_time,
                current: now,
            });
        }
        self.previous_time = now;
        let timeout = self.timeout.unwrap_or(INITIALISATION_TIMEOUT);
        if now - self.armed_at >= timeout {
            return Err(InitialisationError::Timeout {
                timeout,
                targets: Box::new(self.targets),
                pending: [
                    ("zfw", self.pending.zfw.is_some()),
                    ("gw", self.pending.gw.is_some()),
                    ("gwcg", self.pending.gwcg.is_some()),
                    ("ths", self.pending.ths.is_some()),
                    ("flaps", self.pending.flaps.is_some()),
                    ("landing_gear", self.pending.landing_gear.is_some()),
                    (
                        "pressure_altitude",
                        self.pending.pressure_altitude.is_some(),
                    ),
                    (
                        "calibrated_airspeed",
                        self.pending.calibrated_airspeed.is_some(),
                    ),
                    ("pitch", self.pending.pitch.is_some()),
                    ("vertical_speed", self.pending.vertical_speed.is_some()),
                    ("roll", self.pending.roll.is_some()),
                ]
                .into_iter()
                .filter_map(|(name, pending)| pending.then_some(name))
                .collect(),
                details: String::new(),
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
        pressure_altitude: None,
        calibrated_airspeed: None,
        vertical_speed: None,
        pitch: None,
        roll: None,
    };

    #[test]
    fn deadline_uses_elapsed_simulator_time_and_reports_targets() {
        for elapsed in [Duration::from_secs(60), Duration::from_secs(65)] {
            let mut gate = Initialisation::new(TARGETS, Duration::from_secs(100));
            gate.timed_out(Duration::from_millis(159999)).unwrap();
            assert!(matches!(
                gate.timed_out(Duration::from_secs(100) + elapsed),
                Err(InitialisationError::Timeout { targets, pending, .. })
                    if *targets == TARGETS && pending == ["zfw", "gw", "gwcg"]
            ));
        }
    }

    #[test]
    fn timeout_reports_the_latest_pending_subset_without_changing_requested_targets() {
        let mut gate = Initialisation::new(TARGETS, Duration::from_secs(100));
        gate.update_pending(InitialisationConfig {
            zfw: TARGETS.zfw,
            gw: TARGETS.gw,
            ..InitialisationConfig::default()
        });
        gate.timed_out(Duration::from_secs(110)).unwrap();
        gate.update_pending(InitialisationConfig {
            gwcg: TARGETS.gwcg,
            ..InitialisationConfig::default()
        });
        assert_eq!(gate.targets(), TARGETS);
        let error = gate.timed_out(Duration::from_secs(160)).unwrap_err();
        assert!(
            matches!(&error, InitialisationError::Timeout { targets, pending, .. }
            if **targets == TARGETS && pending == &["gwcg"])
        );
        assert_eq!(
            error.to_string(),
            "aircraft initialisation timed out after 60 simulator seconds; pending parameters at the last check: [\"gwcg\"]"
        );
    }

    #[test]
    fn timeout_lists_explicit_pitch_and_roll_targets() {
        let targets = InitialisationConfig {
            pitch: Some(3.0),
            roll: Some(-15.0),
            ..InitialisationConfig::default()
        };
        let mut gate = Initialisation::new(targets, Duration::ZERO);
        assert!(matches!(gate.timed_out(Duration::from_secs(60)),
            Err(InitialisationError::Timeout { pending, .. }) if pending == ["pitch", "roll"]));
    }

    #[test]
    fn planned_travel_extends_the_budget_once_without_a_sliding_deadline() {
        let mut gate = Initialisation::new(TARGETS, Duration::from_secs(100));
        gate.set_timeout(Duration::from_secs(90));
        gate.timed_out(Duration::from_secs(160)).unwrap();
        gate.set_timeout(Duration::from_secs(500));
        gate.timed_out(Duration::from_millis(189999)).unwrap();
        let error = gate.timed_out(Duration::from_secs(190)).unwrap_err();
        assert!(matches!(error, InitialisationError::Timeout { timeout, .. }
            if timeout == Duration::from_secs(90)));
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
