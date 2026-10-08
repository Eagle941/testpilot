//! A32NX detection, aircraft loading, trim initialisation and actual-state readback.

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
use testpilot_core::config::InitialisationConfig;
use testpilot_core::error::InitialisationError;
use testpilot_core::initialisation::AircraftInitialisationState;
use testpilot_core::simulator::SimulatorAdapter;

mod loading;

#[cfg(test)]
mod runtime_tests;

use loading::{A32nxLoading, BAG_WEIGHT_KG, PAX_WEIGHT_KG};

impl A32nxLoading {
    /// Submits prepared loading targets before setting rates and requesting instant loading.
    fn submit(self, simulator: &mut dyn SimulatorAdapter) -> Result<(), InitialisationError> {
        // Desired values first, then instant loading rates, then start requests.
        // Stop at the first failed write; never start loading after a target failure.
        for (variable, value) in [
            ("L:A32NX_WB_PER_PAX_WEIGHT", PAX_WEIGHT_KG),
            ("L:A32NX_WB_PER_BAG_WEIGHT", BAG_WEIGHT_KG),
            ("L:A32NX_PAX_A_DESIRED", self.pax_a),
            ("L:A32NX_PAX_B_DESIRED", self.pax_b),
            ("L:A32NX_PAX_C_DESIRED", self.pax_c),
            ("L:A32NX_PAX_D_DESIRED", self.pax_d),
            (
                "L:A32NX_CARGO_FWD_BAGGAGE_CONTAINER_DESIRED",
                self.cargo_fwd_baggage_container,
            ),
            (
                "L:A32NX_CARGO_AFT_CONTAINER_DESIRED",
                self.cargo_aft_container,
            ),
            ("L:A32NX_CARGO_AFT_BAGGAGE_DESIRED", self.cargo_aft_baggage),
            (
                "L:A32NX_CARGO_AFT_BULK_LOOSE_DESIRED",
                self.cargo_aft_bulk_loose,
            ),
            ("L:A32NX_FUEL_LEFT_AUX_DESIRED", self.fuel_left_aux),
            ("L:A32NX_FUEL_RIGHT_AUX_DESIRED", self.fuel_right_aux),
            ("L:A32NX_FUEL_LEFT_MAIN_DESIRED", self.fuel_left_main),
            ("L:A32NX_FUEL_RIGHT_MAIN_DESIRED", self.fuel_right_main),
            ("L:A32NX_FUEL_CENTER_DESIRED", self.fuel_center),
            ("L:A32NX_FUEL_TOTAL_DESIRED", self.fuel_total),
            ("L:A32NX_FUEL_DESIRED_PERCENT", self.fuel_percent),
            ("L:A32NX_BOARDING_RATE", 0.0),
            ("L:A32NX_EFB_REFUEL_RATE_SETTING", 2.0),
            ("L:A32NX_BOARDING_STARTED_BY_USR", 1.0),
            ("L:A32NX_REFUEL_STARTED_BY_USR", 1.0),
        ] {
            simulator
                .write(variable, value)
                .map_err(InitialisationError::Submit)?;
        }
        Ok(())
    }
}

/// A32NX detection, loading submission and configured actual-state readback.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct A32nxInitialiser {
    /// Whether the latest submitted configuration requests THS readback.
    read_ths: bool,
    /// Whether the latest submission omitted the mass/balance group.
    skip_mass_balance: bool,
}

impl A32nxInitialiser {
    /// Trim-wheel position published by A32NX in percent.
    const TRIM_POSITION: &str = "L:A32NX_HYD_TRIM_WHEEL_PERCENT";
    /// Transient manual trim demand consumed each simulator tick.
    const TRIM_EVENT: &str = "K:AXIS_ELEV_TRIM_SET";

    /// Maps THS degrees to the SDK's integer event range without clamping.
    fn trim_axis_demand(degrees: f64) -> Result<f64, InitialisationError> {
        if !degrees.is_finite() || !(-4.0..=13.5).contains(&degrees) {
            return Err(InitialisationError::InvalidThsTarget { degrees });
        }
        Ok((-16383.0 + (degrees + 4.0) * 32767.0 / 17.5).round())
    }
}

impl AircraftInitialiser for A32nxInitialiser {
    /// Matches the A20N model code and its configured ATC localisation key (see README).
    /// Requires the FlyByWire readiness variable to exist, without reading its value.
    fn supported_model(&mut self, simulator: &mut dyn SimulatorAdapter) -> bool {
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
        self.read_ths = false;
        self.skip_mass_balance = true;
        // Prepare every target before performing any writes.
        let loading = targets
            .has_mass_balance()
            .then(|| A32nxLoading::from_targets(targets))
            .transpose()?;
        let trim = targets
            .ths
            .map(|target| {
                let axis = Self::trim_axis_demand(target)?;
                if !simulator
                    .local_variable_exists(Self::TRIM_POSITION)
                    .map_err(InitialisationError::Readback)?
                {
                    return Err(InitialisationError::MissingThsInterface);
                }
                simulator
                    .validate_read(Self::TRIM_POSITION, None)
                    .map_err(InitialisationError::Readback)?;
                Ok(axis)
            })
            .transpose()?;
        if let Some(loading) = loading {
            loading.submit(simulator)?;
        }
        if let Some(axis) = trim {
            simulator
                .write(Self::TRIM_EVENT, axis)
                .map_err(InitialisationError::Submit)?;
        }
        self.skip_mass_balance = !targets.has_mass_balance();
        self.read_ths = targets.ths.is_some();
        Ok(())
    }

    fn readback(
        &mut self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<AircraftInitialisationState, InitialisationError> {
        // A32NX publishes actual airframe masses in kg and CG in percent MAC.
        // L: reads use their native numeric scale without an SDK unit conversion.
        Ok(AircraftInitialisationState {
            zfw: (!self.skip_mass_balance)
                .then(|| {
                    simulator
                        .read("L:A32NX_AIRFRAME_ZFW", None)
                        .map_err(InitialisationError::Readback)
                })
                .transpose()?,
            gw: (!self.skip_mass_balance)
                .then(|| {
                    simulator
                        .read("L:A32NX_AIRFRAME_GW", None)
                        .map_err(InitialisationError::Readback)
                })
                .transpose()?,
            gwcg: (!self.skip_mass_balance)
                .then(|| {
                    simulator
                        .read("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", None)
                        .map_err(InitialisationError::Readback)
                })
                .transpose()?,
            ths: self
                .read_ths
                .then(|| {
                    simulator
                        .read(Self::TRIM_POSITION, None)
                        .map(|percent| -4.0 + percent * 17.5 / 100.0)
                        .map_err(InitialisationError::Readback)
                })
                .transpose()?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    use testpilot_core::error::SimulatorError;

    #[test]
    fn axis_conversion_preserves_endpoints_and_rounds_to_integer_events() {
        for (degrees, axis) in [
            (-4.0, -16383.0),
            (0.0, -8893.0),
            (4.75, 1.0),
            (13.5, 16384.0),
        ] {
            assert_eq!(A32nxInitialiser::trim_axis_demand(degrees).unwrap(), axis);
        }
        for value in [
            -4.000001,
            13.500001,
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
        ] {
            assert!(matches!(
                A32nxInitialiser::trim_axis_demand(value),
                Err(InitialisationError::InvalidThsTarget { .. })
            ));
        }
    }

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
                },
            )
            .unwrap_err();
        assert!(matches!(
            error,
            InitialisationError::UnreachableLoading { .. }
        ));
        assert!(simulator.writes.is_empty());
    }
}
