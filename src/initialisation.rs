//! Simulator-independent aircraft initialisation readiness and deadline checks.

use std::time::Duration;

use crate::config::InitialisationConfig;
use crate::error::InitialisationError;

/// Actual aircraft mass and balance returned by the simulator, not FMS entries.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AircraftMassBalance {
    /// Actual zero-fuel weight in kilograms.
    pub zfw: f64,
    /// Actual gross weight in kilograms.
    pub gw: f64,
    /// Actual gross-weight centre of gravity in percent MAC.
    pub gwcg: f64,
}

/// Bounded readiness state for one armed run.
pub struct Initialisation {
    /// Demanded aircraft values.
    targets: InitialisationConfig,
    /// Simulator timestamp on the arm frame.
    armed_at: Duration,
    /// Most recent checked timestamp, used to reject backwards time.
    previous_time: Duration,
    /// Latest valid snapshot, included in timeout diagnostics.
    latest: Option<AircraftMassBalance>,
}

impl Initialisation {
    /// Begins a 30-second simulator-time deadline at the arm frame.
    pub const fn new(targets: InitialisationConfig, armed_at: Duration) -> Self {
        Self {
            targets,
            armed_at,
            previous_time: armed_at,
            latest: None,
        }
    }

    /// Checks time before readback so timeout takes precedence at exactly 30 seconds.
    pub fn check_deadline(&mut self, now: Duration) -> Result<(), InitialisationError> {
        if now < self.previous_time {
            return Err(InitialisationError::ClockMovedBackwards {
                previous: self.previous_time,
                current: now,
            });
        }
        self.previous_time = now;
        if now - self.armed_at >= Duration::from_secs(30) {
            return Err(InitialisationError::Timeout {
                targets: self.targets,
                latest: self.latest,
            });
        }
        Ok(())
    }

    /// Checks all three values on one frame, with inclusive absolute tolerances.
    pub fn observe(&mut self, actual: AircraftMassBalance) -> Result<bool, InitialisationError> {
        for (field, value) in [
            ("zfw", actual.zfw),
            ("gw", actual.gw),
            ("gwcg", actual.gwcg),
        ] {
            if !value.is_finite() {
                return Err(InitialisationError::NonFiniteReadback { field, value });
            }
        }
        self.latest = Some(actual);
        Ok(Self::within(actual.zfw, self.targets.zfw, 100.0)
            && Self::within(actual.gw, self.targets.gw, 100.0)
            && Self::within(actual.gwcg, self.targets.gwcg, 0.01))
    }

    /// Compares inclusive endpoints directly to preserve decimal boundary rounding.
    fn within(actual: f64, target: f64, tolerance: f64) -> bool {
        actual >= target - tolerance && actual <= target + tolerance
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TARGETS: InitialisationConfig = InitialisationConfig {
        zfw: 60000.0,
        gw: 65000.0,
        gwcg: 25.0,
    };
    const ACTUAL: AircraftMassBalance = AircraftMassBalance {
        zfw: 60000.0,
        gw: 65000.0,
        gwcg: 25.0,
    };

    #[test]
    fn accepts_inclusive_tolerances_and_rejects_values_just_outside() {
        let mut gate = Initialisation::new(TARGETS, Duration::ZERO);
        for zfw in [59900.0, 60100.0] {
            for gw in [64900.0, 65100.0] {
                for gwcg in [24.99, 25.01] {
                    assert!(gate.observe(AircraftMassBalance { zfw, gw, gwcg }).unwrap());
                }
            }
        }
        for actual in [
            AircraftMassBalance {
                zfw: 59899.999,
                ..ACTUAL
            },
            AircraftMassBalance {
                zfw: 60100.001,
                ..ACTUAL
            },
            AircraftMassBalance {
                gw: 64899.999,
                ..ACTUAL
            },
            AircraftMassBalance {
                gw: 65100.001,
                ..ACTUAL
            },
            AircraftMassBalance {
                gwcg: 24.989999,
                ..ACTUAL
            },
            AircraftMassBalance {
                gwcg: 25.010001,
                ..ACTUAL
            },
        ] {
            assert!(!gate.observe(actual).unwrap(), "accepted {actual:?}");
        }
    }

    #[test]
    fn requires_simultaneous_readiness_and_finite_readback() {
        let mut gate = Initialisation::new(TARGETS, Duration::ZERO);
        assert!(
            !gate
                .observe(AircraftMassBalance {
                    zfw: 61000.0,
                    ..ACTUAL
                })
                .unwrap()
        );
        assert!(
            !gate
                .observe(AircraftMassBalance {
                    gw: 66000.0,
                    ..ACTUAL
                })
                .unwrap()
        );
        assert!(
            !gate
                .observe(AircraftMassBalance {
                    gwcg: 26.0,
                    ..ACTUAL
                })
                .unwrap()
        );
        assert!(gate.observe(ACTUAL).unwrap());
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for actual in [
                AircraftMassBalance {
                    zfw: value,
                    ..ACTUAL
                },
                AircraftMassBalance {
                    gw: value,
                    ..ACTUAL
                },
                AircraftMassBalance {
                    gwcg: value,
                    ..ACTUAL
                },
            ] {
                assert!(matches!(
                    gate.observe(actual),
                    Err(InitialisationError::NonFiniteReadback { .. })
                ));
            }
        }
    }

    #[test]
    fn deadline_uses_elapsed_simulator_time_and_reports_latest_snapshot() {
        for elapsed in [Duration::from_secs(30), Duration::from_secs(35)] {
            let mut gate = Initialisation::new(TARGETS, Duration::from_secs(100));
            gate.check_deadline(Duration::from_millis(129999)).unwrap();
            gate.observe(ACTUAL).unwrap();
            assert!(matches!(
                gate.check_deadline(Duration::from_secs(100) + elapsed),
                Err(InitialisationError::Timeout {
                    targets: TARGETS,
                    latest: Some(ACTUAL)
                })
            ));
        }
        let mut gate = Initialisation::new(TARGETS, Duration::from_secs(100));
        assert!(matches!(
            gate.check_deadline(Duration::from_secs(130)),
            Err(InitialisationError::Timeout { latest: None, .. })
        ));
    }

    #[test]
    fn rejects_clock_reversal_even_after_the_arm_timestamp() {
        let mut gate = Initialisation::new(TARGETS, Duration::from_secs(100));
        gate.check_deadline(Duration::from_secs(110)).unwrap();
        assert!(matches!(gate.check_deadline(Duration::from_secs(109)),
            Err(InitialisationError::ClockMovedBackwards { previous, current })
                if previous == Duration::from_secs(110) && current == Duration::from_secs(109)));
    }
}
