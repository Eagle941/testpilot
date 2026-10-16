//! Aircraft-specific setup operations and readiness checks, separate from generic simulator I/O.

use crate::config::{InitialisationConfig, LandingGear};
use crate::error::InitialisationError;
use crate::simulator::SimulatorAdapter;

/// Aircraft setup and readiness used by the runtime; tests can supply an independent component.
pub trait AircraftInitialiser {
    /// Checks this initialiser's model and interface requirements.
    /// Unavailable identifiers or interfaces must return false.
    fn supported(&mut self, simulator: &mut dyn SimulatorAdapter) -> bool;

    /// Submits all configured targets on each initialisation frame until ready.
    fn submit(
        &mut self,
        simulator: &mut dyn SimulatorAdapter,
        targets: InitialisationConfig,
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

    /// Reads and checks every group unless a simulator or validation error occurs.
    ///
    /// False means valid but unsettled readback. A false check does not skip later groups.
    fn is_ready(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        targets: InitialisationConfig,
    ) -> Result<bool, InitialisationError> {
        let zfw = self.check_zfw(simulator, targets.zfw)?;
        let gw = self.check_gw(simulator, targets.gw)?;
        let gwcg = self.check_gwcg(simulator, targets.gwcg)?;
        let ths = self.check_ths(simulator, targets.ths)?;
        let flaps = self.check_flaps(simulator, targets.flaps)?;
        let landing_gear = self.check_landing_gear(simulator, targets.landing_gear)?;
        Ok(zfw && gw && gwcg && ths && flaps && landing_gear)
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
    }

    #[derive(Debug, Clone, PartialEq)]
    pub(crate) struct FakeAircraftInitialiser {
        pub(crate) outcomes: [Result<bool, InitialisationError>; 6],
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

        fn write(&mut self, _: &str, _: f64) -> Result<(), SimulatorError> {
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
        let aircraft = FakeAircraftInitialiser {
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
            ]
        );
    }
}
