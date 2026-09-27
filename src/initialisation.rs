//! Simulator-independent aircraft initialisation readiness and deadline checks.

use std::time::Duration;

use crate::config::InitialisationConfig;
use crate::error::InitialisationError;

/// Inclusive THS readiness tolerance in degrees, including event quantisation.
pub(crate) fn ths_ready(target: f64, value: f64) -> Result<bool, InitialisationError> {
    if !value.is_finite() {
        return Err(InitialisationError::NonFiniteReadback {
            field: "THS",
            value,
        });
    }
    Ok(Initialisation::within(value, target, 0.01))
}

/// Actual aircraft mass, balance and optional THS returned by the simulator, not FMS entries.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AircraftInitialisationState {
    /// Actual zero-fuel weight in kilograms, present for configured mass/balance.
    pub zfw: Option<f64>,
    /// Actual gross weight in kilograms, present for configured mass/balance.
    pub gw: Option<f64>,
    /// Actual gross-weight centre of gravity in percent MAC, present for configured mass/balance.
    pub gwcg: Option<f64>,
    /// Actual THS in degrees, present when trim initialisation is requested.
    pub ths: Option<f64>,
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
    latest: Option<AircraftInitialisationState>,
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
                targets: Box::new(self.targets),
                latest: self.latest,
            });
        }
        Ok(())
    }

    /// Checks all requested values on one frame, with inclusive absolute tolerances.
    pub fn observe(
        &mut self,
        actual: AircraftInitialisationState,
    ) -> Result<bool, InitialisationError> {
        let mut mass_ready = true;
        for (field, value, target, tolerance) in [
            ("zfw", actual.zfw, self.targets.zfw, 100.0),
            ("gw", actual.gw, self.targets.gw, 100.0),
            ("gwcg", actual.gwcg, self.targets.gwcg, 0.01),
        ] {
            let Some(target) = target else {
                continue;
            };
            let value = value.ok_or(InitialisationError::MissingMassBalanceReadback { field })?;
            if !value.is_finite() {
                return Err(InitialisationError::NonFiniteReadback { field, value });
            }
            mass_ready &= Self::within(value, target, tolerance);
        }
        let trim_ready = match (self.targets.ths, actual.ths) {
            (Some(_), None) => return Err(InitialisationError::MissingThsReadback),
            (Some(target), Some(value)) => ths_ready(target, value)?,
            (None, _) => true,
        };
        self.latest = Some(actual);
        Ok(trim_ready && mass_ready)
    }

    /// Compares inclusive endpoints directly to preserve decimal boundary rounding.
    fn within(actual: f64, target: f64, tolerance: f64) -> bool {
        actual >= target - tolerance && actual <= target + tolerance
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ths_only_readiness_does_not_require_mass_readback() {
        let targets = InitialisationConfig {
            zfw: None,
            gw: None,
            gwcg: None,
            ths: Some(1.0),
        };
        let mut gate = Initialisation::new(targets, Duration::ZERO);
        let actual = AircraftInitialisationState {
            zfw: None,
            gw: None,
            gwcg: None,
            ths: Some(0.0),
        };
        assert!(!gate.observe(actual).unwrap());
        assert!(
            gate.observe(AircraftInitialisationState {
                ths: Some(1.0),
                ..actual
            })
            .unwrap()
        );
        assert!(matches!(
            Initialisation::new(TARGETS, Duration::ZERO).observe(actual),
            Err(InitialisationError::MissingMassBalanceReadback { field: "zfw" })
        ));
    }

    #[test]
    fn ths_must_be_present_finite_and_simultaneously_within_tolerance() {
        let mut gate = Initialisation::new(
            InitialisationConfig {
                ths: Some(1.0),
                ..TARGETS
            },
            Duration::ZERO,
        );
        assert!(matches!(
            gate.observe(ACTUAL),
            Err(InitialisationError::MissingThsReadback)
        ));
        for value in [0.99, 1.0, 1.01] {
            assert!(
                gate.observe(AircraftInitialisationState {
                    ths: Some(value),
                    ..ACTUAL
                })
                .unwrap()
            );
        }
        for value in [0.989999, 1.010001] {
            assert!(
                !gate
                    .observe(AircraftInitialisationState {
                        ths: Some(value),
                        ..ACTUAL
                    })
                    .unwrap()
            );
        }
        assert!(
            !gate
                .observe(AircraftInitialisationState {
                    ths: Some(1.0),
                    gw: Some(66000.0),
                    ..ACTUAL
                })
                .unwrap()
        );
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(matches!(
                gate.observe(AircraftInitialisationState {
                    ths: Some(value),
                    ..ACTUAL
                }),
                Err(InitialisationError::NonFiniteReadback { field: "THS", .. })
            ));
        }
    }

    const TARGETS: InitialisationConfig = InitialisationConfig {
        zfw: Some(60000.0),
        gw: Some(65000.0),
        gwcg: Some(25.0),
        ths: None,
    };
    const ACTUAL: AircraftInitialisationState = AircraftInitialisationState {
        zfw: Some(60000.0),
        gw: Some(65000.0),
        gwcg: Some(25.0),
        ths: None,
    };

    #[test]
    fn accepts_inclusive_tolerances_and_rejects_values_just_outside() {
        let mut gate = Initialisation::new(TARGETS, Duration::ZERO);
        for zfw in [59900.0, 60100.0] {
            for gw in [64900.0, 65100.0] {
                for gwcg in [24.99, 25.01] {
                    assert!(
                        gate.observe(AircraftInitialisationState {
                            zfw: Some(zfw),
                            gw: Some(gw),
                            gwcg: Some(gwcg),
                            ths: None
                        })
                        .unwrap()
                    );
                }
            }
        }
        for actual in [
            AircraftInitialisationState {
                zfw: Some(59899.999),
                ..ACTUAL
            },
            AircraftInitialisationState {
                zfw: Some(60100.001),
                ..ACTUAL
            },
            AircraftInitialisationState {
                gw: Some(64899.999),
                ..ACTUAL
            },
            AircraftInitialisationState {
                gw: Some(65100.001),
                ..ACTUAL
            },
            AircraftInitialisationState {
                gwcg: Some(24.989999),
                ..ACTUAL
            },
            AircraftInitialisationState {
                gwcg: Some(25.010001),
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
                .observe(AircraftInitialisationState {
                    zfw: Some(61000.0),
                    ..ACTUAL
                })
                .unwrap()
        );
        assert!(
            !gate
                .observe(AircraftInitialisationState {
                    gw: Some(66000.0),
                    ..ACTUAL
                })
                .unwrap()
        );
        assert!(
            !gate
                .observe(AircraftInitialisationState {
                    gwcg: Some(26.0),
                    ..ACTUAL
                })
                .unwrap()
        );
        assert!(gate.observe(ACTUAL).unwrap());
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for actual in [
                AircraftInitialisationState {
                    zfw: Some(value),
                    ..ACTUAL
                },
                AircraftInitialisationState {
                    gw: Some(value),
                    ..ACTUAL
                },
                AircraftInitialisationState {
                    gwcg: Some(value),
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
                    targets,
                    latest: Some(ACTUAL)
                }) if *targets == TARGETS
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
