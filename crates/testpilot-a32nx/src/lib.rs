//! A32NX detection, initialisation submission and actual-state readback.

#![cfg_attr(
    not(test),
    deny(
        clippy::expect_used,
        clippy::missing_docs_in_private_items,
        clippy::panic,
        clippy::todo,
        clippy::unimplemented,
        clippy::unreachable,
        clippy::unwrap_used
    )
)]

use testpilot_core::aircraft_initialisation::AircraftInitialiser;
use testpilot_core::config::{InitialisationConfig, LandingGear};
use testpilot_core::error::InitialisationError;
use testpilot_core::simulator::SimulatorAdapter;

mod configuration;
mod flaps;
mod landing_gear;
mod loading;
mod ths;

#[cfg(test)]
mod runtime_tests;

use flaps::A32nxFlaps;
use landing_gear::A32nxLandingGear;
use loading::A32nxLoading;
use ths::A32nxThs;

/// A32NX detection, loading submission and configured actual-state readback.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct A32nxInitialiser {
    /// Prepared mass/balance commands from the latest successful submission.
    mass_balance: Option<A32nxLoading>,
    /// Prepared THS command from the latest successful submission.
    ths: Option<A32nxThs>,
    /// Prepared flap/slat command from the latest successful submission.
    flaps: Option<A32nxFlaps>,
    /// Prepared landing gear command from the latest successful submission.
    landing_gear: Option<A32nxLandingGear>,
}

impl A32nxInitialiser {
    /// Prepares all configured groups and validates their interfaces without writing commands.
    fn new(
        targets: InitialisationConfig,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<Self, InitialisationError> {
        Ok(Self {
            mass_balance: targets
                .has_mass_balance()
                .then(|| A32nxLoading::prepare(targets))
                .transpose()?,
            ths: targets
                .ths
                .map(|target| A32nxThs::prepare(simulator, target))
                .transpose()?,
            flaps: targets
                .flaps
                .map(|target| A32nxFlaps::prepare(simulator, target))
                .transpose()?,
            landing_gear: targets
                .landing_gear
                .map(|target| A32nxLandingGear::prepare(simulator, target))
                .transpose()?,
        })
    }

    /// Reads a configured mass/balance value and checks its inclusive absolute tolerance.
    fn mass_balance_ready(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        field: &'static str,
        variable: &'static str,
        target: Option<f64>,
        tolerance: f64,
    ) -> Result<bool, InitialisationError> {
        let Some(target) = target else {
            return Ok(true);
        };
        self.mass_balance
            .as_ref()
            .ok_or(InitialisationError::MissingMassBalanceReadback { field })?;
        let actual = configuration::read_native(simulator, variable)?;
        Self::within(field, actual, target, tolerance)
    }

    /// Rejects non-finite readback and compares inclusive endpoints to preserve boundary rounding.
    fn within(
        field: &'static str,
        actual: f64,
        target: f64,
        tolerance: f64,
    ) -> Result<bool, InitialisationError> {
        if !actual.is_finite() {
            return Err(InitialisationError::NonFiniteReadback {
                field,
                value: actual,
            });
        }
        Ok(actual >= target - tolerance && actual <= target + tolerance)
    }
}

impl AircraftInitialiser for A32nxInitialiser {
    /// Matches the A20N model code and its configured ATC localisation key (see README).
    /// Requires the FlyByWire readiness variable to exist, without reading its value.
    fn supported(&mut self, simulator: &mut dyn SimulatorAdapter) -> bool {
        let Ok(model) = simulator.read_string("A:ATC MODEL") else {
            return false;
        };
        let model = model.trim();
        if !model.eq_ignore_ascii_case("A20N")
            && !model.eq_ignore_ascii_case("TT:ATCCOM.AC_MODEL_A20N.0.text")
        {
            return false;
        }
        simulator
            .local_variable_exists("L:A32NX_IS_READY")
            .unwrap_or(false)
    }

    fn submit(
        &mut self,
        simulator: &mut dyn SimulatorAdapter,
        targets: InitialisationConfig,
    ) -> Result<(), InitialisationError> {
        let next = Self::new(targets, simulator)?;
        if let Some(mass_balance) = &next.mass_balance {
            mass_balance.submit(simulator)?;
        }
        if let Some(ths) = &next.ths {
            ths.submit(simulator)?;
        }
        if let Some(flaps) = &next.flaps {
            flaps.submit(simulator)?;
        }
        if let Some(landing_gear) = &next.landing_gear {
            landing_gear.submit(simulator)?;
        }
        *self = next;
        Ok(())
    }

    fn check_zfw(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError> {
        self.mass_balance_ready(simulator, "zfw", "L:A32NX_AIRFRAME_ZFW", target, 100.0)
    }

    fn check_gw(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError> {
        self.mass_balance_ready(simulator, "gw", "L:A32NX_AIRFRAME_GW", target, 100.0)
    }

    fn check_gwcg(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError> {
        self.mass_balance_ready(
            simulator,
            "gwcg",
            "L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC",
            target,
            0.01,
        )
    }

    fn check_ths(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError> {
        let Some(target) = target else {
            return Ok(true);
        };
        let ths = self
            .ths
            .as_ref()
            .ok_or(InitialisationError::MissingThsReadback)?;
        Self::within("THS", ths.readback(simulator)?, target, 0.01)
    }

    fn check_flaps(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<u8>,
    ) -> Result<bool, InitialisationError> {
        if target.is_none() {
            return Ok(true);
        }
        let flaps = self
            .flaps
            .as_ref()
            .ok_or(InitialisationError::MissingReadback { field: "flaps" })?;
        let (flaps_moving, slats_moving) = flaps.readback(simulator)?;
        Ok(!flaps_moving && !slats_moving)
    }

    fn check_landing_gear(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<LandingGear>,
    ) -> Result<bool, InitialisationError> {
        let Some(target) = target else {
            return Ok(true);
        };
        let gear = self
            .landing_gear
            .as_ref()
            .ok_or(InitialisationError::MissingReadback {
                field: "landing_gear",
            })?;
        A32nxLandingGear::is_ready(gear.readback(simulator)?, target)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    use testpilot_core::error::SimulatorError;

    #[derive(Debug, Clone, Default, PartialEq)]
    struct LoadingSimulator {
        writes: Vec<(String, f64)>,
        fail_at: Option<usize>,
    }

    impl SimulatorAdapter for LoadingSimulator {
        fn write(&mut self, variable: &str, value: f64) -> Result<(), SimulatorError> {
            self.writes.push((variable.to_owned(), value));
            if self.fail_at == Some(self.writes.len() - 1) {
                return Err(SimulatorError::CalculatorCodeWriteFailed {
                    variable: variable.to_owned(),
                    value,
                });
            }
            Ok(())
        }

        fn local_variable_exists(&mut self, _: &str) -> Result<bool, SimulatorError> {
            unreachable!("submission only writes loading variables")
        }

        fn read_string(&mut self, _: &str) -> Result<String, SimulatorError> {
            unreachable!("submission only writes loading variables")
        }

        fn simulation_time(&self) -> Result<Duration, SimulatorError> {
            unreachable!("submission only writes loading variables")
        }

        fn validate_read(&mut self, _: &str, _: Option<&str>) -> Result<(), SimulatorError> {
            unreachable!("submission only writes loading variables")
        }

        fn read(&mut self, _: &str, _: Option<&str>) -> Result<f64, SimulatorError> {
            unreachable!("submission only writes loading variables")
        }
    }

    fn loading() -> A32nxLoading {
        A32nxLoading {
            pax_a: 1.0,
            pax_b: 2.0,
            pax_c: 3.0,
            pax_d: 4.0,
            cargo_fwd_baggage_container: 5.0,
            cargo_aft_container: 6.0,
            cargo_aft_baggage: 7.0,
            cargo_aft_bulk_loose: 8.0,
            fuel_left_aux: 9.0,
            fuel_right_aux: 10.0,
            fuel_left_main: 11.0,
            fuel_right_main: 12.0,
            fuel_center: 13.0,
            fuel_total: 55.0,
            fuel_percent: 0.88,
        }
    }

    const EXPECTED: [(&str, f64); 21] = [
        ("L:A32NX_WB_PER_PAX_WEIGHT", 84.0),
        ("L:A32NX_WB_PER_BAG_WEIGHT", 20.0),
        ("L:A32NX_PAX_A_DESIRED", 1.0),
        ("L:A32NX_PAX_B_DESIRED", 2.0),
        ("L:A32NX_PAX_C_DESIRED", 3.0),
        ("L:A32NX_PAX_D_DESIRED", 4.0),
        ("L:A32NX_CARGO_FWD_BAGGAGE_CONTAINER_DESIRED", 5.0),
        ("L:A32NX_CARGO_AFT_CONTAINER_DESIRED", 6.0),
        ("L:A32NX_CARGO_AFT_BAGGAGE_DESIRED", 7.0),
        ("L:A32NX_CARGO_AFT_BULK_LOOSE_DESIRED", 8.0),
        ("L:A32NX_FUEL_LEFT_AUX_DESIRED", 9.0),
        ("L:A32NX_FUEL_RIGHT_AUX_DESIRED", 10.0),
        ("L:A32NX_FUEL_LEFT_MAIN_DESIRED", 11.0),
        ("L:A32NX_FUEL_RIGHT_MAIN_DESIRED", 12.0),
        ("L:A32NX_FUEL_CENTER_DESIRED", 13.0),
        ("L:A32NX_FUEL_TOTAL_DESIRED", 55.0),
        ("L:A32NX_FUEL_DESIRED_PERCENT", 0.88),
        ("L:A32NX_BOARDING_RATE", 0.0),
        ("L:A32NX_EFB_REFUEL_RATE_SETTING", 2.0),
        ("L:A32NX_BOARDING_STARTED_BY_USR", 1.0),
        ("L:A32NX_REFUEL_STARTED_BY_USR", 1.0),
    ];

    #[test]
    fn submits_all_desired_values_before_rates_and_start_requests() {
        let mut simulator = LoadingSimulator::default();
        loading().submit(&mut simulator).unwrap();
        let writes: Vec<_> = simulator
            .writes
            .iter()
            .map(|(v, n)| (v.as_str(), *n))
            .collect();
        assert_eq!(writes, EXPECTED);
    }

    #[test]
    fn each_write_failure_stops_submission_and_preserves_variable_context() {
        for (index, (expected_variable, expected_value)) in EXPECTED.iter().enumerate() {
            let mut simulator = LoadingSimulator {
                fail_at: Some(index),
                ..Default::default()
            };
            let error = loading().submit(&mut simulator).unwrap_err();
            match error {
                InitialisationError::Submit(SimulatorError::CalculatorCodeWriteFailed {
                    variable,
                    value,
                }) => {
                    assert_eq!(variable, *expected_variable);
                    assert_eq!(value, *expected_value);
                }
                error => panic!("unexpected error: {error}"),
            }
            assert_eq!(simulator.writes.len(), index + 1);
        }
    }

    #[test]
    fn unreachable_loading_performs_no_loading_writes() {
        let mut simulator = LoadingSimulator::default();
        let error = A32nxInitialiser::default()
            .submit(
                &mut simulator,
                InitialisationConfig {
                    zfw: Some(60000.0),
                    gw: Some(65000.0),
                    gwcg: Some(99.0),
                    ths: None,
                    flaps: None,
                    landing_gear: None,
                },
            )
            .unwrap_err();
        assert!(matches!(
            error,
            InitialisationError::UnreachableLoading { .. }
        ));
        assert!(simulator.writes.is_empty());
    }

    #[test]
    fn readiness_tolerances_are_inclusive() {
        for (field, target, tolerance) in [
            ("zfw", 60000.0, 100.0),
            ("gw", 65000.0, 100.0),
            ("gwcg", 25.0, 0.01),
            ("THS", 1.0, 0.01),
        ] {
            for actual in [target - tolerance, target, target + tolerance] {
                assert!(A32nxInitialiser::within(field, actual, target, tolerance).unwrap());
            }
            for actual in [target - tolerance - 0.000001, target + tolerance + 0.000001] {
                assert!(!A32nxInitialiser::within(field, actual, target, tolerance).unwrap());
            }
            for actual in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                assert!(matches!(
                    A32nxInitialiser::within(field, actual, target, tolerance),
                    Err(InitialisationError::NonFiniteReadback { field: missing, .. })
                        if missing == field
                ));
            }
        }
    }
}
