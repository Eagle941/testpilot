//! Aircraft-specific loading operations, separate from generic simulator I/O and readiness checks.

use crate::config::InitialisationConfig;
use crate::error::InitialisationError;
use crate::initialisation::AircraftMassBalance;
use crate::simulator::SimulatorAdapter;

/// Detection is deliberately non-failing: unreadable or unknown aircraft are unsupported.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AircraftSupport {
    /// The aircraft matches this initialiser's model and interface requirements.
    Supported,
    /// The aircraft does not match; skip initialisation and continue replay.
    Unsupported,
}

/// Loading operations used by the runtime; tests can supply an independent aircraft component.
pub trait AircraftInitialiser {
    /// Checks this initialiser's model and interface requirements.
    /// Unavailable identifiers or interfaces must return false.
    fn supported_model(&mut self, simulator: &mut dyn SimulatorAdapter) -> bool;

    /// Detects support once per armed run that requests initialisation.
    fn detect(&mut self, simulator: &mut dyn SimulatorAdapter) -> AircraftSupport {
        if self.supported_model(simulator) {
            AircraftSupport::Supported
        } else {
            AircraftSupport::Unsupported
        }
    }

    /// Submits actual aircraft loading targets once on arming.
    fn submit(
        &mut self,
        simulator: &mut dyn SimulatorAdapter,
        targets: InitialisationConfig,
    ) -> Result<(), InitialisationError>;

    /// Reads actual mass and balance, independently of configured telemetry signals.
    fn readback(
        &mut self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<AircraftMassBalance, InitialisationError>;
}

/// A32NX detection, loading submission and actual mass/balance readback.
/// Loading-value calculation remains unimplemented.
pub struct A32nxInitialiser;

/// Native LVAR values, calculated before any loading writes are performed.
struct A32nxLoading {
    pax_a: f64,
    pax_b: f64,
    pax_c: f64,
    pax_d: f64,
    cargo_fwd_baggage_container: f64,
    cargo_aft_container: f64,
    cargo_aft_baggage: f64,
    cargo_aft_bulk_loose: f64,
    fuel_left_aux: f64,
    fuel_right_aux: f64,
    fuel_left_main: f64,
    fuel_right_main: f64,
    fuel_center: f64,
    fuel_total: f64,
    fuel_percent: f64,
}

impl A32nxLoading {
    fn from_targets(_targets: InitialisationConfig) -> Result<Self, InitialisationError> {
        // TODO: Calculate passenger seat flags, cargo and fuel distribution from
        // ZFW, GW and GWCG. Do not submit placeholder loading values.
        Err(InitialisationError::NotImplemented {
            operation: "loading-value calculation",
        })
    }

    fn submit(self, simulator: &mut dyn SimulatorAdapter) -> Result<(), InitialisationError> {
        // Desired values first, then instant loading rates, then start requests.
        // Stop at the first failed write; never start loading after a target failure.
        for (variable, value) in [
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
        A32nxLoading::from_targets(targets)?.submit(simulator)
    }

    fn readback(
        &mut self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<AircraftMassBalance, InitialisationError> {
        // A32NX publishes actual airframe masses in kg and CG in percent MAC.
        // L: reads use their native numeric scale without an SDK unit conversion.
        Ok(AircraftMassBalance {
            zfw: simulator
                .read("L:A32NX_AIRFRAME_ZFW", None)
                .map_err(InitialisationError::Readback)?,
            gw: simulator
                .read("L:A32NX_AIRFRAME_GW", None)
                .map_err(InitialisationError::Readback)?,
            gwcg: simulator
                .read("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", None)
                .map_err(InitialisationError::Readback)?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::SimulatorError;
    use std::time::Duration;

    #[derive(Default)]
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

    const EXPECTED: [(&str, f64); 19] = [
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
    fn pending_calculation_performs_no_loading_writes() {
        let mut simulator = LoadingSimulator::default();
        let error = A32nxInitialiser
            .submit(
                &mut simulator,
                InitialisationConfig {
                    zfw: 60000.0,
                    gw: 65000.0,
                    gwcg: 25.0,
                },
            )
            .unwrap_err();
        assert!(matches!(
            error,
            InitialisationError::NotImplemented {
                operation: "loading-value calculation"
            }
        ));
        assert!(simulator.writes.is_empty());
    }
}
