//! Aircraft-specific setup operations and readiness checks, separate from generic simulator I/O.

use std::time::Duration;

use crate::config::{InitialisationConfig, LandingGear};
use crate::error::{InitialisationError, SimulatorError};
use crate::simulator::SimulatorAdapter;

/// Base readiness allowance; aircraft may add planned repositioning time.
pub const INITIALISATION_TIMEOUT: Duration = Duration::from_secs(60);

/// Aircraft setup and readiness used by the runtime; tests can supply an independent component.
pub trait AircraftInitialiser {
    /// Checks this initialiser's model and interface requirements.
    /// Unavailable identifiers or interfaces must return false.
    fn supported(&mut self, simulator: &mut dyn SimulatorAdapter) -> bool;

    /// Drops per-run controller history without restoring prior aircraft settings.
    fn reset(&mut self) {}

    /// Releases temporary setup inhibitions before dropping per-run history.
    /// Failed cleanup must retain enough state for a later retry.
    fn cleanup(&mut self, _simulator: &mut dyn SimulatorAdapter) -> Result<(), SimulatorError> {
        self.reset();
        Ok(())
    }

    /// Readiness budget including planned travel, sampled once after the first setup observation.
    fn initialisation_timeout(&self) -> Result<Duration, InitialisationError> {
        Ok(INITIALISATION_TIMEOUT)
    }

    /// Describes out-of-tolerance readbacks at timeout without commanding the aircraft.
    /// Diagnostic read failures must be included in the text rather than replacing the timeout.
    fn timeout_diagnostics(
        &self,
        _simulator: &mut dyn SimulatorAdapter,
        _targets: InitialisationConfig,
    ) -> String {
        String::new()
    }

    /// Advances configured setup once using the runtime's simulator timestamp.
    fn submit(
        &mut self,
        simulator: &mut dyn SimulatorAdapter,
        targets: InitialisationConfig,
        now: Duration,
    ) -> Result<(), InitialisationError>;

    /// Reads and checks zfw, returning true when the target is omitted.
    fn check_zfw(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError>;

    /// Reads and checks gw, returning true when the target is omitted.
    fn check_gw(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError>;

    /// Reads and checks gwcg, returning true when the target is omitted.
    fn check_gwcg(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError>;

    /// Reads and checks ths, returning true when the target is omitted.
    fn check_ths(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError>;

    /// Reads and checks flaps, returning true when the target is omitted.
    fn check_flaps(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<u8>,
    ) -> Result<bool, InitialisationError>;

    /// Reads and checks landing_gear, returning true when the target is omitted.
    fn check_landing_gear(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<LandingGear>,
    ) -> Result<bool, InitialisationError>;

    /// Checks the pre-submission pressure-altitude observation; omitted targets are ready.
    fn check_pressure_altitude(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError>;

    /// Checks the pre-submission calibrated-airspeed observation; omitted targets are ready.
    fn check_calibrated_airspeed(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError>;

    /// Checks the pre-submission pitch observation; omitted targets are ready.
    fn check_pitch(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError>;

    /// Checks an explicit vertical-speed target; aircraft implementations supply its feedback.
    fn check_vertical_speed(
        &self,
        _simulator: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError> {
        if target.is_some() {
            return Err(InitialisationError::MissingReadback {
                field: "vertical_speed",
            });
        }
        Ok(true)
    }

    /// Checks the pre-submission roll observation; omitted targets are ready.
    fn check_roll(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError>;

    /// Whether all configured targets are ready after observing every group.
    fn is_ready(
        &mut self,
        simulator: &mut dyn SimulatorAdapter,
        targets: InitialisationConfig,
    ) -> Result<bool, InitialisationError> {
        Ok(!self.pending_targets(simulator, targets)?.has_targets())
    }

    /// Observes every group and returns the subset not ready for timeout diagnostics.
    ///
    /// This subset is diagnostic only and must never be submitted as setup targets.
    fn pending_targets(
        &mut self,
        simulator: &mut dyn SimulatorAdapter,
        targets: InitialisationConfig,
    ) -> Result<InitialisationConfig, InitialisationError> {
        let zfw = self.check_zfw(simulator, targets.zfw)?;
        let gw = self.check_gw(simulator, targets.gw)?;
        let gwcg = self.check_gwcg(simulator, targets.gwcg)?;
        let ths = self.check_ths(simulator, targets.ths)?;
        let flaps = self.check_flaps(simulator, targets.flaps)?;
        let landing_gear = self.check_landing_gear(simulator, targets.landing_gear)?;
        let pressure_altitude =
            self.check_pressure_altitude(simulator, targets.pressure_altitude)?;
        let calibrated_airspeed =
            self.check_calibrated_airspeed(simulator, targets.calibrated_airspeed)?;
        let pitch = self.check_pitch(simulator, targets.pitch)?;
        let vertical_speed = self.check_vertical_speed(simulator, targets.vertical_speed)?;
        let roll = self.check_roll(simulator, targets.roll)?;
        Ok(InitialisationConfig {
            zfw: targets.zfw.filter(|_| !zfw),
            gw: targets.gw.filter(|_| !gw),
            gwcg: targets.gwcg.filter(|_| !gwcg),
            ths: targets.ths.filter(|_| !ths),
            flaps: targets.flaps.filter(|_| !flaps),
            landing_gear: targets.landing_gear.filter(|_| !landing_gear),
            pressure_altitude: targets.pressure_altitude.filter(|_| !pressure_altitude),
            calibrated_airspeed: targets.calibrated_airspeed.filter(|_| !calibrated_airspeed),
            vertical_speed: targets.vertical_speed.filter(|_| !vertical_speed),
            pitch: targets.pitch.filter(|_| !pitch),
            roll: targets.roll.filter(|_| !roll),
        })
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use std::cell::RefCell;
    use std::time::Duration;

    use super::*;
    use crate::error::SimulatorError;

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub(crate) enum Check {
        Zfw(Option<f64>),
        Gw(Option<f64>),
        Gwcg(Option<f64>),
        Ths(Option<f64>),
        Flaps(Option<u8>),
        LandingGear(Option<LandingGear>),
        PressureAltitude(Option<f64>),
        CalibratedAirspeed(Option<f64>),
        Pitch(Option<f64>),
        Roll(Option<f64>),
    }

    #[derive(Debug, Clone, PartialEq)]
    pub(crate) struct FakeAircraftInitialiser {
        pub(crate) outcomes: [Result<bool, InitialisationError>; 10],
        pub(crate) calls: RefCell<Vec<Check>>,
    }

    impl Default for FakeAircraftInitialiser {
        fn default() -> Self {
            Self {
                outcomes: std::array::from_fn(|_| Ok(true)),
                calls: RefCell::default(),
            }
        }
    }

    impl FakeAircraftInitialiser {
        fn check(
            &self,
            index: usize,
            check: Check,
            configured: bool,
        ) -> Result<bool, InitialisationError> {
            self.calls.borrow_mut().push(check);
            if configured {
                self.outcomes[index].clone()
            } else {
                Ok(true)
            }
        }
    }

    impl AircraftInitialiser for FakeAircraftInitialiser {
        fn supported(&mut self, _: &mut dyn SimulatorAdapter) -> bool {
            true
        }

        fn submit(
            &mut self,
            _: &mut dyn SimulatorAdapter,
            _: InitialisationConfig,
            _: Duration,
        ) -> Result<(), InitialisationError> {
            Ok(())
        }

        fn check_zfw(
            &self,
            _: &mut dyn SimulatorAdapter,
            target: Option<f64>,
        ) -> Result<bool, InitialisationError> {
            self.check(0, Check::Zfw(target), target.is_some())
        }

        fn check_gw(
            &self,
            _: &mut dyn SimulatorAdapter,
            target: Option<f64>,
        ) -> Result<bool, InitialisationError> {
            self.check(1, Check::Gw(target), target.is_some())
        }

        fn check_gwcg(
            &self,
            _: &mut dyn SimulatorAdapter,
            target: Option<f64>,
        ) -> Result<bool, InitialisationError> {
            self.check(2, Check::Gwcg(target), target.is_some())
        }

        fn check_ths(
            &self,
            _: &mut dyn SimulatorAdapter,
            target: Option<f64>,
        ) -> Result<bool, InitialisationError> {
            self.check(3, Check::Ths(target), target.is_some())
        }

        fn check_flaps(
            &self,
            _: &mut dyn SimulatorAdapter,
            target: Option<u8>,
        ) -> Result<bool, InitialisationError> {
            self.check(4, Check::Flaps(target), target.is_some())
        }

        fn check_landing_gear(
            &self,
            _: &mut dyn SimulatorAdapter,
            target: Option<LandingGear>,
        ) -> Result<bool, InitialisationError> {
            self.check(5, Check::LandingGear(target), target.is_some())
        }

        fn check_pressure_altitude(
            &self,
            _: &mut dyn SimulatorAdapter,
            target: Option<f64>,
        ) -> Result<bool, InitialisationError> {
            self.check(6, Check::PressureAltitude(target), target.is_some())
        }

        fn check_calibrated_airspeed(
            &self,
            _: &mut dyn SimulatorAdapter,
            target: Option<f64>,
        ) -> Result<bool, InitialisationError> {
            self.check(7, Check::CalibratedAirspeed(target), target.is_some())
        }

        fn check_pitch(
            &self,
            _: &mut dyn SimulatorAdapter,
            target: Option<f64>,
        ) -> Result<bool, InitialisationError> {
            self.check(8, Check::Pitch(target), target.is_some())
        }

        fn check_roll(
            &self,
            _: &mut dyn SimulatorAdapter,
            target: Option<f64>,
        ) -> Result<bool, InitialisationError> {
            self.check(9, Check::Roll(target), target.is_some())
        }
    }

    struct NoIoSimulator;

    impl SimulatorAdapter for NoIoSimulator {
        fn local_variable_exists(&mut self, _: &str) -> Result<bool, SimulatorError> {
            unreachable!("fake readiness checks do not access simulator interfaces")
        }

        fn read_string(&mut self, _: &str) -> Result<String, SimulatorError> {
            unreachable!("fake readiness checks do not read strings")
        }

        fn simulation_time(&self) -> Result<Duration, SimulatorError> {
            unreachable!("readiness checks do not read the clock")
        }

        fn write(&mut self, _: &str, _: f64, _unit: Option<&str>) -> Result<(), SimulatorError> {
            unreachable!("readiness checks do not submit commands")
        }

        fn validate_read(&mut self, _: &str, _: Option<&str>) -> Result<(), SimulatorError> {
            unreachable!("fake readiness checks do not validate interfaces")
        }

        fn read(&mut self, _: &str, _: Option<&str>) -> Result<f64, SimulatorError> {
            unreachable!("fake readiness checks return predetermined outcomes")
        }
    }

    const TARGETS: InitialisationConfig = InitialisationConfig {
        zfw: Some(60000.0),
        gw: Some(65000.0),
        gwcg: Some(25.0),
        ths: Some(1.0),
        flaps: Some(2),
        landing_gear: Some(LandingGear::Down),
        pressure_altitude: Some(10000.0),
        calibrated_airspeed: Some(250.0),
        vertical_speed: None,
        pitch: Some(3.0),
        roll: Some(0.0),
    };

    #[test]
    fn aggregate_checks_every_group_even_after_false() {
        let mut aircraft = FakeAircraftInitialiser::default();
        aircraft.outcomes[0] = Ok(false);
        assert!(!aircraft.is_ready(&mut NoIoSimulator, TARGETS).unwrap());
        assert_eq!(
            *aircraft.calls.borrow(),
            [
                Check::Zfw(TARGETS.zfw),
                Check::Gw(TARGETS.gw),
                Check::Gwcg(TARGETS.gwcg),
                Check::Ths(TARGETS.ths),
                Check::Flaps(TARGETS.flaps),
                Check::LandingGear(TARGETS.landing_gear),
                Check::PressureAltitude(TARGETS.pressure_altitude),
                Check::CalibratedAirspeed(TARGETS.calibrated_airspeed),
                Check::Pitch(TARGETS.pitch),
                Check::Roll(TARGETS.roll),
            ]
        );
    }

    #[test]
    fn aggregate_propagates_later_errors_after_an_unsettled_group() {
        let mut aircraft = FakeAircraftInitialiser::default();
        aircraft.outcomes[0] = Ok(false);
        aircraft.outcomes[4] = Err(InitialisationError::MissingReadback { field: "flaps" });
        assert!(matches!(
            aircraft.is_ready(&mut NoIoSimulator, TARGETS),
            Err(InitialisationError::MissingReadback { field: "flaps" })
        ));
        assert_eq!(aircraft.calls.borrow().len(), 5);
    }

    #[test]
    fn aggregate_passes_omitted_targets_to_each_check() {
        let mut aircraft = FakeAircraftInitialiser {
            outcomes: std::array::from_fn(|_| Err(InitialisationError::MissingThsReadback)),
            ..Default::default()
        };
        assert!(
            aircraft
                .is_ready(&mut NoIoSimulator, InitialisationConfig::default())
                .unwrap()
        );
        assert_eq!(
            *aircraft.calls.borrow(),
            [
                Check::Zfw(None),
                Check::Gw(None),
                Check::Gwcg(None),
                Check::Ths(None),
                Check::Flaps(None),
                Check::LandingGear(None),
                Check::PressureAltitude(None),
                Check::CalibratedAirspeed(None),
                Check::Pitch(None),
                Check::Roll(None),
            ]
        );
    }
}
