//! A32NX detection, aircraft loading, trim initialisation and actual-state readback.

use super::AircraftInitialiser;
use crate::config::InitialisationConfig;
use crate::error::InitialisationError;
use crate::initialisation::AircraftInitialisationState;
use crate::simulator::SimulatorAdapter;

const PAX_WEIGHT_KG: f64 = 84.0;
const BAG_WEIGHT_KG: f64 = 20.0;
const EMPTY_KG: f64 = 42500.0;
const EMPTY_ARM_FT: f64 = -9.42;
const LEMAC_FT: f64 = -5.383;
const MAC_FT: f64 = 13.464;
const MAX_ZFW_KG: f64 = 64300.0;
const MAX_GW_KG: f64 = 79000.0;
const KG_PER_GALLON: f64 = 3.039075693483925;
const MAX_FUEL_GALLONS: f64 = 6267.0;
const MAX_PAX: u32 = 174;
const MAX_CARGO_KG: f64 = 9435.0;
const EPS: f64 = 1e-7;
const MOMENT_EPS: f64 = 1e-6;
const PAX_CAPACITIES: [u32; 4] = [36, 42, 48, 48];
const PAX_ARMS: [f64; 4] = [20.5, 1.5, -16.6, -35.6];
const CARGO_CAPACITIES: [f64; 4] = [3402.0, 2426.0, 2110.0, 1497.0];
const CARGO_ARMS: [f64; 4] = [17.3, -24.1, -34.1, -42.4];
const FUEL_ARMS: [f64; 5] = [-16.9, -16.9, -8.0, -8.0, -4.5];

/// Native LVAR values, calculated before any loading writes are performed.
#[derive(Debug, PartialEq)]
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
    /// Reduced `calculate`: GW fixes fuel; its moment fixes the required payload moment.
    fn from_targets(targets: InitialisationConfig) -> Result<Self, InitialisationError> {
        let invalid = |reason| InitialisationError::InvalidLoadingTargets { targets, reason };
        let (Some(zfw), Some(gw), Some(gwcg)) = (targets.zfw, targets.gw, targets.gwcg) else {
            return Err(invalid(
                "ZFW, GW and GWCG must all be supplied for aircraft loading",
            ));
        };
        if ![zfw, gw, gwcg].into_iter().all(f64::is_finite) {
            return Err(invalid("all inputs must be finite"));
        }
        if !(EMPTY_KG..=MAX_ZFW_KG).contains(&zfw) {
            return Err(invalid("ZFW must be between 42500 and 64300 kg"));
        }
        if !(zfw..=MAX_GW_KG).contains(&gw) {
            return Err(invalid("GW must be at least ZFW and at most 79000 kg"));
        }
        if gw - zfw > MAX_FUEL_GALLONS * KG_PER_GALLON + EPS {
            return Err(invalid("GW - ZFW exceeds total fuel capacity"));
        }

        let fuel = Self::fuel_distribution(gw - zfw);
        let gross_moment = gw * (LEMAC_FT - MAC_FT * gwcg / 100.0);
        let payload_moment = gross_moment
            - Self::moment(&fuel, &FUEL_ARMS) * KG_PER_GALLON
            - EMPTY_KG * EMPTY_ARM_FT;
        if !payload_moment.is_finite() {
            return Err(invalid(
                "CG is outside the numeric range of the aircraft model",
            ));
        }
        let (counts, cargo) = Self::solve_payload(zfw - EMPTY_KG, payload_moment)
            .ok_or(InitialisationError::UnreachableLoading { targets })?;
        let [pax_a, pax_b, pax_c, pax_d] = counts.map(Self::seat_mask);
        let [
            cargo_fwd_baggage_container,
            cargo_aft_container,
            cargo_aft_baggage,
            cargo_aft_bulk_loose,
        ] = cargo;
        let [
            fuel_left_aux,
            fuel_right_aux,
            fuel_left_main,
            fuel_right_main,
            fuel_center,
        ] = fuel;
        let fuel_total = fuel.iter().sum();
        Ok(Self {
            pax_a,
            pax_b,
            pax_c,
            pax_d,
            cargo_fwd_baggage_container,
            cargo_aft_container,
            cargo_aft_baggage,
            cargo_aft_bulk_loose,
            fuel_left_aux,
            fuel_right_aux,
            fuel_left_main,
            fuel_right_main,
            fuel_center,
            fuel_total,
            fuel_percent: fuel_total / MAX_FUEL_GALLONS * 100.0,
        })
    }

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

    fn fuel_distribution(fuel_kg: f64) -> [f64; 5] {
        let mut remaining = fuel_kg / KG_PER_GALLON;
        let auxiliary = remaining.min(456.0);
        remaining -= auxiliary;
        let main = remaining.min(3632.0);
        remaining -= main;
        [
            auxiliary / 2.0,
            auxiliary / 2.0,
            main / 2.0,
            main / 2.0,
            remaining,
        ]
    }

    fn moment<const N: usize>(loads: &[f64; N], arms: &[f64; N]) -> f64 {
        loads.iter().zip(arms).map(|(load, arm)| load * arm).sum()
    }

    /// Both station arrays are ordered from the most forward to the most aft arm.
    fn extreme_load(mut total: f64, capacities: &[f64; 4], forward: bool) -> [f64; 4] {
        let mut loads = [0.0; 4];
        for step in 0..4 {
            let i = if forward { step } else { 3 - step };
            loads[i] = total.min(capacities[i]);
            total -= loads[i];
        }
        loads
    }

    fn efb_passengers(total: u32) -> [u32; 4] {
        let mut counts = [0; 4];
        let mut remaining = total;
        for i in (1..4).rev() {
            let percent = (PAX_CAPACITIES[i] * 100).div_ceil(MAX_PAX);
            counts[i] = (total * percent / 100).min(PAX_CAPACITIES[i]);
            remaining -= counts[i];
        }
        counts[0] = remaining;
        counts
    }

    /// Solves payload mass (kg) and longitudinal moment (kg ft) using integer passengers
    /// and continuously divisible cargo. Returns A/B/C/D counts and cargo station kg.
    ///
    /// Passenger total is the primary preference; cabin distribution is secondary.
    /// For each total, enumerate seats and let cargo supply the remaining moment.
    /// The aircraft's fixed capacities bound the search and its storage requirements.
    fn solve_payload(payload: f64, target_moment: f64) -> Option<([u32; 4], [f64; 4])> {
        // Start near the EFB estimate: one passenger plus their baggage per 104 kg.
        // Try every other total by distance from that estimate, lower totals first
        // on ties. The first feasible total wins even if another seats people more evenly.
        let preferred = (payload / (PAX_WEIGHT_KG + BAG_WEIGHT_KG))
            .round()
            .min(MAX_PAX as f64) as u32;
        let mut totals = std::array::from_fn::<_, 175, _>(|i| i as u32);
        totals.sort_unstable_by_key(|&n| (n.abs_diff(preferred), n));
        for total in totals {
            // Passenger mass fixes the remaining cargo mass, which already includes
            // baggage. Reject totals that overfill the holds or cannot carry all bags.
            let cargo_mass = payload - total as f64 * PAX_WEIGHT_KG;
            if !(-EPS..=MAX_CARGO_KG + EPS).contains(&cargo_mass)
                || cargo_mass + EPS < total as f64 * BAG_WEIGHT_KG
            {
                continue;
            }
            // Only absorb floating-point error at physical boundaries.
            let cargo_mass = cargo_mass.clamp(0.0, MAX_CARGO_KG);
            // Greedily filling aftmost/forwardmost holds gives the minimum/maximum
            // cargo moment for this mass. Every moment between them can be achieved
            // by blending the two loadings because cargo weights are continuous.
            let aft = Self::extreme_load(cargo_mass, &CARGO_CAPACITIES, false);
            let forward = Self::extreme_load(cargo_mass, &CARGO_CAPACITIES, true);
            let min_cargo = Self::moment(&aft, &CARGO_ARMS);
            let max_cargo = Self::moment(&forward, &CARGO_ARMS);
            // Combine the cargo limits with extreme cabin loadings to cheaply reject
            // impossible totals before enumerating seats. Passing this check is only
            // necessary: integer seating can still leave gaps in reachable moments.
            let pax_capacities = PAX_CAPACITIES.map(f64::from);
            let min_moment = PAX_WEIGHT_KG
                * Self::moment(
                    &Self::extreme_load(total as f64, &pax_capacities, false),
                    &PAX_ARMS,
                )
                + min_cargo;
            let max_moment = PAX_WEIGHT_KG
                * Self::moment(
                    &Self::extreme_load(total as f64, &pax_capacities, true),
                    &PAX_ARMS,
                )
                + max_cargo;
            if !(min_moment - MOMENT_EPS..=max_moment + MOMENT_EPS).contains(&target_moment) {
                continue;
            }

            let desired = Self::efb_passengers(total);
            let mut best: Option<(u32, [u32; 4], f64)> = None;
            // Enumerate A/B/C; D is the remainder. Lower bounds reserve enough seats
            // in later sections (B+C+D = 138, C+D = 96, D = 48); upper bounds respect
            // each section's capacity. This visits at most 37 * 43 * 49 combinations.
            for a in total.saturating_sub(138)..=36.min(total) {
                for b in (total - a).saturating_sub(96)..=42.min(total - a) {
                    for c in (total - a - b).saturating_sub(48)..=48.min(total - a - b) {
                        let counts = [a, b, c, total - a - b - c];
                        // Once the seats are fixed, cargo must supply exactly the
                        // target moment minus the passengers' contribution.
                        let cargo_moment = target_moment
                            - PAX_WEIGHT_KG * Self::moment(&counts.map(f64::from), &PAX_ARMS);
                        if !(min_cargo - MOMENT_EPS..=max_cargo + MOMENT_EPS)
                            .contains(&cargo_moment)
                        {
                            continue;
                        }
                        // Among feasible seatings at this total, favour the smallest
                        // sum of squared deviations from the EFB cabin distribution.
                        let score = counts
                            .iter()
                            .zip(desired)
                            .map(|(n, ideal)| n.abs_diff(ideal).pow(2))
                            .sum();
                        // Ascending enumeration retains the lexicographically first tie.
                        if best
                            .as_ref()
                            .is_none_or(|(previous, _, _)| score < *previous)
                        {
                            best = Some((score, counts, cargo_moment));
                        }
                    }
                }
            }
            if let Some((_, counts, cargo_moment)) = best {
                // Use the same blend fraction in every hold: total mass stays fixed,
                // each hold remains within capacity, and moment varies linearly.
                // A negligible span needs no blend; endpoint clamping absorbs only
                // the rounding error allowed by the moment feasibility checks above.
                let span = max_cargo - min_cargo;
                let fraction = if span <= MOMENT_EPS {
                    0.0
                } else {
                    (cargo_moment - min_cargo) / span
                }
                .clamp(0.0, 1.0);
                let cargo = std::array::from_fn(|i| aft[i] + fraction * (forward[i] - aft[i]));
                return Some((counts, cargo));
            }
        }
        // No passenger total and seating can meet both the mass and moment constraints.
        None
    }

    /// EFB BitFlags uses 31-bit words with a 32-bit stride; all masks fit exactly in f64.
    fn seat_mask(count: u32) -> f64 {
        (0..count)
            .map(|seat| 1_u64 << (seat + seat / 31))
            .sum::<u64>() as f64
    }
}

/// A32NX detection, loading submission and configured actual-state readback.
#[derive(Default)]
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
    use crate::error::SimulatorError;
    use std::time::Duration;

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

    // Forward fixtures independently reconstruct mass and CG from physical loads.
    fn targets(counts: [u32; 4], cargo: [f64; 4], fuel: [f64; 5]) -> InitialisationConfig {
        let zfw = 42500.0 + counts.iter().sum::<u32>() as f64 * 84.0 + cargo.iter().sum::<f64>();
        let payload_moment = counts
            .iter()
            .zip([20.5, 1.5, -16.6, -35.6])
            .map(|(&n, arm)| n as f64 * 84.0 * arm)
            .sum::<f64>()
            + cargo
                .iter()
                .zip([17.3, -24.1, -34.1, -42.4])
                .map(|(m, arm)| m * arm)
                .sum::<f64>();
        let gw = zfw + fuel.iter().sum::<f64>() * 3.039075693483925;
        let fuel_moment = fuel
            .iter()
            .zip([-16.9, -16.9, -8.0, -8.0, -4.5])
            .map(|(volume, arm)| volume * 3.039075693483925 * arm)
            .sum::<f64>();
        let gwcg = (-5.383 - (-400350.0 + payload_moment + fuel_moment) / gw) * 100.0 / 13.464;
        InitialisationConfig {
            zfw: Some(zfw),
            gw: Some(gw),
            gwcg: Some(gwcg),
            ths: None,
        }
    }

    fn counts(load: &A32nxLoading) -> [u32; 4] {
        [load.pax_a, load.pax_b, load.pax_c, load.pax_d].map(|mask| (mask as u64).count_ones())
    }

    fn cargo(load: &A32nxLoading) -> [f64; 4] {
        [
            load.cargo_fwd_baggage_container,
            load.cargo_aft_container,
            load.cargo_aft_baggage,
            load.cargo_aft_bulk_loose,
        ]
    }

    fn fuel(load: &A32nxLoading) -> [f64; 5] {
        [
            load.fuel_left_aux,
            load.fuel_right_aux,
            load.fuel_left_main,
            load.fuel_right_main,
            load.fuel_center,
        ]
    }

    fn close(actual: f64, expected: f64) {
        assert!((actual - expected).abs() < 1e-7, "{actual} != {expected}");
    }

    fn verify(load: &A32nxLoading, requested: InitialisationConfig) {
        let counts = counts(load);
        let cargo = cargo(load);
        let fuel = fuel(load);
        let actual = targets(counts, cargo, fuel);
        close(actual.zfw.unwrap(), requested.zfw.unwrap());
        close(actual.gw.unwrap(), requested.gw.unwrap());
        close(actual.gwcg.unwrap(), requested.gwcg.unwrap());
        assert!(cargo.iter().sum::<f64>() + 1e-7 >= counts.iter().sum::<u32>() as f64 * 20.0);
        for (n, capacity) in counts.into_iter().zip([36, 42, 48, 48]) {
            assert!(n <= capacity);
        }
        for (value, capacity) in cargo
            .into_iter()
            .zip([3402.0, 2426.0, 2110.0, 1497.0])
            .chain(fuel.into_iter().zip([228.0, 228.0, 1816.0, 1816.0, 2179.0]))
        {
            assert!(value.is_finite() && (0.0..=capacity + 1e-7).contains(&value));
        }
        close(load.fuel_total, fuel.iter().sum());
        close(load.fuel_percent, load.fuel_total / 6267.0 * 100.0);
    }

    #[test]
    fn matches_python_calculator_examples_and_is_deterministic() {
        for (requested, expected_counts, expected_cargo, main) in [
            (
                InitialisationConfig {
                    zfw: Some(50000.0),
                    gw: Some(56000.0),
                    gwcg: Some(30.5),
                    ths: None,
                },
                [14, 18, 20, 20],
                [873.6258759536895, 0.0, 0.0, 578.3741240463105],
                759.1422440817427,
            ),
            (
                InitialisationConfig {
                    zfw: Some(60000.0),
                    gw: Some(65000.0),
                    gwcg: Some(25.0),
                    ths: None,
                },
                [32, 42, 47, 47],
                [
                    2862.752337259579,
                    0.0,
                    293.16509157087853,
                    232.0825711695427,
                ],
                594.6185367347856,
            ),
        ] {
            let load = A32nxLoading::from_targets(requested).unwrap();
            assert_eq!(counts(&load), expected_counts);
            for (actual, expected) in cargo(&load).into_iter().zip(expected_cargo) {
                close(actual, expected);
            }
            for (actual, expected) in fuel(&load).into_iter().zip([228.0, 228.0, main, main, 0.0]) {
                close(actual, expected);
            }
            verify(&load, requested);
            assert_eq!(load, A32nxLoading::from_targets(requested).unwrap());
        }
    }

    #[test]
    fn conserves_mass_and_cg_across_all_fuel_stages_and_boundaries() {
        for expected_fuel in [
            [0.0; 5],
            [100.0, 100.0, 0.0, 0.0, 0.0],
            [227.9995, 227.9995, 0.0, 0.0, 0.0],
            [228.0, 228.0, 0.0, 0.0, 0.0],
            [228.0, 228.0, 0.0005, 0.0005, 0.0],
            [228.0, 228.0, 1000.0, 1000.0, 0.0],
            [228.0, 228.0, 1815.9995, 1815.9995, 0.0],
            [228.0, 228.0, 1816.0, 1816.0, 0.0],
            [228.0, 228.0, 1816.0, 1816.0, 0.001],
            [228.0, 228.0, 1816.0, 1816.0, 1500.0],
            [228.0, 228.0, 1816.0, 1816.0, 2179.0],
        ] {
            let requested = targets(
                [15, 18, 20, 20],
                [400.0, 500.0, 300.0, 300.0],
                expected_fuel,
            );
            let load = A32nxLoading::from_targets(requested).unwrap();
            verify(&load, requested);
            for (actual, expected) in fuel(&load).into_iter().zip(expected_fuel) {
                close(actual, expected);
            }
        }
    }

    #[test]
    fn handles_empty_aircraft_baggage_constraints_cg_constraints_and_full_cabin() {
        for (pax, cargo, expected_counts) in [
            ([0; 4], [0.0; 4], [0; 4]),
            // EFB rounding prefers one passenger, but the payload cannot fit one.
            ([0; 4], [60.0, 0.0, 0.0, 0.0], [0; 4]),
            // CG forces a lower count than the preferred 72 passengers.
            ([36, 14, 0, 0], [3300.0, 0.0, 0.0, 0.0], [36, 14, 0, 0]),
            (
                [36, 42, 48, 48],
                [2400.0, 2400.0, 1500.0, 884.0],
                [36, 42, 48, 48],
            ),
        ] {
            let requested = targets(pax, cargo, [0.0; 5]);
            let load = A32nxLoading::from_targets(requested).unwrap();
            verify(&load, requested);
            assert_eq!(counts(&load), expected_counts);
        }
        let mut requested = targets([36, 42, 48, 48], [2400.0, 2400.0, 1500.0, 884.0], [0.0; 5]);
        let fuel = A32nxLoading::fuel_distribution(79000.0 - requested.zfw.unwrap());
        requested = targets([36, 42, 48, 48], [2400.0, 2400.0, 1500.0, 884.0], fuel);
        close(requested.gw.unwrap(), 79000.0);
        verify(&A32nxLoading::from_targets(requested).unwrap(), requested);
    }

    #[test]
    fn seat_masks_skip_bit_31_and_remain_exact_as_f64() {
        for (count, expected) in [
            (0, 0),
            (1, 1),
            (31, (1_u64 << 31) - 1),
            (32, (1_u64 << 32) + (1 << 31) - 1),
            (48, (1_u64 << 49) - (1 << 31) - 1),
        ] {
            let mask = A32nxLoading::seat_mask(count);
            assert_eq!(mask as u64, expected);
            assert_eq!((mask as u64).count_ones(), count);
            assert_eq!((mask as u64) & (1 << 31), 0);
        }
    }

    #[test]
    fn rejects_invalid_and_unreachable_targets() {
        let base = InitialisationConfig {
            zfw: Some(50000.0),
            gw: Some(56000.0),
            gwcg: Some(30.5),
            ths: None,
        };
        for requested in [
            InitialisationConfig {
                zfw: Some(42499.0),
                ..base
            },
            InitialisationConfig {
                zfw: Some(64301.0),
                gw: Some(65000.0),
                ..base
            },
            InitialisationConfig {
                gw: Some(49999.0),
                ..base
            },
            InitialisationConfig {
                gw: Some(79001.0),
                ..base
            },
            InitialisationConfig {
                gw: Some(70000.0),
                ..base
            },
            InitialisationConfig {
                gwcg: Some(f64::MAX),
                ..base
            },
        ]
        .into_iter()
        .chain(
            [f64::NAN, f64::INFINITY, f64::NEG_INFINITY]
                .into_iter()
                .flat_map(|value| {
                    [
                        InitialisationConfig {
                            zfw: Some(value),
                            ..base
                        },
                        InitialisationConfig {
                            gw: Some(value),
                            ..base
                        },
                        InitialisationConfig {
                            gwcg: Some(value),
                            ..base
                        },
                    ]
                }),
        ) {
            assert!(matches!(
                A32nxLoading::from_targets(requested),
                Err(InitialisationError::InvalidLoadingTargets { .. })
            ));
        }
        for requested in [
            InitialisationConfig {
                gwcg: Some(99.0),
                ..base
            },
            InitialisationConfig {
                gwcg: Some(-99.0),
                ..base
            },
            InitialisationConfig {
                zfw: Some(42500.0),
                gw: Some(42500.0),
                gwcg: Some(30.0),
                ths: None,
            },
        ] {
            assert!(matches!(
                A32nxLoading::from_targets(requested),
                Err(InitialisationError::UnreachableLoading { .. })
            ));
        }
    }
}
