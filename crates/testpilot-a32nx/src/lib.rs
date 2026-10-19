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

use std::time::Duration;
use testpilot_core::aircraft_initialisation::{AircraftInitialiser, INITIALISATION_TIMEOUT};
use testpilot_core::config::{InitialisationConfig, LandingGear};
use testpilot_core::error::{InitialisationError, SimulatorError};
use testpilot_core::simulator::SimulatorAdapter;

mod airspeed;
mod altitude;
mod attitude;
mod configuration;
mod flaps;
mod flight_path;
mod landing_gear;
mod loading;
mod throttle;
mod ths;

#[cfg(test)]
mod runtime_tests;

use airspeed::A32nxCalibratedAirspeed;
use altitude::A32nxPressureAltitude;
use attitude::A32nxAttitude;
use flaps::A32nxFlaps;
use flight_path::A32nxFlightPath;
use landing_gear::A32nxLandingGear;
use loading::A32nxLoading;
use ths::A32nxThs;

/// A32NX startup signal gating flight-control and FADEC updates in aircraft revision
/// ce46d9dbc7a90bd9afddc75d61c64316a6f78d0d (FlyByWireInterface.cpp and ReadySignal.ts).
const AIRCRAFT_READY: &str = "L:A32NX_IS_READY";

/// A32NX revision ce46d9dbc7a90bd9afddc75d61c64316a6f78d0d reads this in
/// MsfsHandler.cpp; EngineControlA32NX.cpp skips engine fuel burn when it is 2.
const DEVELOPER_STATE: &str = "L:A32NX_DEVELOPER_STATE";

/// Inclusive ZFW and GW acceptance tolerance in kilograms.
const MASS_TOLERANCE: f64 = 100.0;
/// Inclusive gross-weight CG acceptance tolerance in percent MAC.
const CG_TOLERANCE: f64 = 0.01;
/// Inclusive THS acceptance tolerance in degrees.
const THS_TOLERANCE: f64 = 0.01;

/// Ordered loading, repositioning and physical stabilisation for one run.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
enum SetupStage {
    /// No setup command has been issued for this run.
    #[default]
    NotStarted,
    /// Passenger, cargo and fuel demands are waiting for actual mass/balance readback.
    Loading,
    /// Aircraft systems are inhibited while the configured ramps advance.
    Repositioning,
    /// Aircraft systems have been released for throttle settling and readiness checks.
    Stabilising,
}

/// A32NX detection, loading submission and configured actual-state readback.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct A32nxInitialiser {
    /// Tracks ownership of the temporary aircraft-system inhibition.
    stage: SetupStage,
    /// Original developer state, retained until the temporary fuel-burn inhibit is released.
    previous_developer_state: Option<f64>,
    /// Prepared mass/balance commands from the latest successful submission.
    mass_balance: Option<A32nxLoading>,
    /// All requested mass/balance values were confirmed on a loading update.
    loading_complete: bool,
    /// Prepared THS command from the latest successful submission.
    ths: Option<A32nxThs>,
    /// Prepared flap/slat command from the latest successful submission.
    flaps: Option<A32nxFlaps>,
    /// Prepared landing gear command from the latest successful submission.
    landing_gear: Option<A32nxLandingGear>,
    /// Current update's pressure-altitude observation and correction.
    pressure_altitude: Option<A32nxPressureAltitude>,
    /// Prepared pitch/roll targets and their pre-write readiness observations.
    attitude: A32nxAttitude,
    /// Persistent airspeed and throttle controller for the current run.
    calibrated_airspeed: Option<A32nxCalibratedAirspeed>,
    /// Natural-flight feedback, present only after the repositioning writes stop.
    flight_path: Option<A32nxFlightPath>,
    /// Each temporary sidestick channel remains owned until its neutral release succeeds.
    stick_channels: [bool; 2],
    /// Fixed allowance for natural climb/descent before reaching the playback altitude.
    flight_path_lead: f64,
    /// Effective vertical-speed target, including the zero default for flight setup.
    flight_vertical_speed: Option<f64>,
    /// Gear selection confirmed by the latest completed readiness observation.
    ready_gear: Option<LandingGear>,
}

impl A32nxInitialiser {
    /// Prepares all configured groups and validates their interfaces without writing commands.
    fn prepare(
        targets: InitialisationConfig,
        simulator: &mut dyn SimulatorAdapter,
        now: Duration,
        previous: &Self,
    ) -> Result<Self, InitialisationError> {
        if targets.landing_gear == Some(LandingGear::Down)
            && let Some(target) = targets.calibrated_airspeed
            && target > 280.0
        {
            return Err(InitialisationError::GearExtendedSpeedLimit {
                target,
                maximum: 280.0,
            });
        }
        let gear_pending =
            targets.landing_gear.is_some() && targets.landing_gear != previous.ready_gear;
        let flight_setup = targets.has_flight_conditions();
        if flight_setup && previous.stage == SetupStage::NotStarted {
            A32nxFlightPath::validate(simulator, targets.vertical_speed.unwrap_or(0.0))?;
        }
        let reposition_altitude = targets
            .pressure_altitude
            .map(|target| {
                A32nxFlightPath::reposition_altitude(target, targets.vertical_speed.unwrap_or(0.0))
            })
            .transpose()?;
        let mut next = Self {
            stage: previous.stage,
            previous_developer_state: previous.previous_developer_state,
            mass_balance: if targets.has_mass_balance() {
                Some(
                    match previous
                        .mass_balance
                        .filter(|plan| plan.matches_targets(targets))
                    {
                        Some(plan) => plan,
                        None => A32nxLoading::prepare(targets)?,
                    },
                )
            } else {
                None
            },
            loading_complete: previous.loading_complete,
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
            attitude: A32nxAttitude::prepare(
                simulator,
                targets.pitch.or(flight_setup.then_some(0.0)),
                targets.roll.or(flight_setup.then_some(0.0)),
            )?,
            pressure_altitude: reposition_altitude
                .map(|target| {
                    A32nxPressureAltitude::prepare(
                        simulator,
                        target,
                        now,
                        previous
                            .pressure_altitude
                            .filter(|_| previous.stage != SetupStage::Loading),
                    )
                })
                .transpose()?,
            flight_path: previous.flight_path,
            stick_channels: previous.stick_channels,
            flight_path_lead: if targets.pressure_altitude.is_some()
                && targets.vertical_speed.unwrap_or(0.0) != 0.0
            {
                flight_path::ALTITUDE_LEAD_SECONDS
            } else {
                0.0
            },
            flight_vertical_speed: flight_setup.then_some(targets.vertical_speed.unwrap_or(0.0)),
            calibrated_airspeed: targets
                .calibrated_airspeed
                .map(|target| {
                    A32nxCalibratedAirspeed::prepare(
                        simulator,
                        target,
                        now,
                        previous
                            .calibrated_airspeed
                            .filter(|_| previous.stage != SetupStage::Loading),
                        gear_pending,
                    )
                })
                .transpose()?,
            ready_gear: previous
                .ready_gear
                .filter(|gear| Some(*gear) == targets.landing_gear),
        };
        if previous.stage == SetupStage::Stabilising && flight_setup {
            next.flight_path = Some(A32nxFlightPath::prepare(
                simulator,
                targets,
                &next.attitude,
                next.calibrated_airspeed.map(|speed| speed.vertical_speed),
                next.pressure_altitude.map(|altitude| altitude.observed),
                now,
                previous.flight_path,
            )?);
        }
        if targets.has_mass_balance() && next.previous_developer_state.is_none() {
            configuration::validate_interface(simulator, DEVELOPER_STATE)?;
            let value = configuration::read_native(simulator, DEVELOPER_STATE)?;
            if !value.is_finite() {
                return Err(InitialisationError::InvalidReadback {
                    variable: DEVELOPER_STATE,
                    value,
                    reason: "must be finite",
                });
            }
            next.previous_developer_state = Some(value);
        }
        next.initialisation_timeout()?;
        Ok(next)
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
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn cleanup(&mut self, simulator: &mut dyn SimulatorAdapter) -> Result<(), SimulatorError> {
        // Attempt every release even when one fails, retaining only failed work for retry.
        let ready = if matches!(self.stage, SetupStage::Loading | SetupStage::Repositioning) {
            simulator.write(AIRCRAFT_READY, 1.0, None).map(|()| {
                self.stage = SetupStage::Stabilising;
                println!("TESTPILOT: {AIRCRAFT_READY}=1; initialisation stopped");
            })
        } else {
            Ok(())
        };
        let fuel = if let Some(previous) = self.previous_developer_state {
            simulator.write(DEVELOPER_STATE, previous, None).map(|()| {
                self.previous_developer_state = None;
                println!(
                    "TESTPILOT: {DEVELOPER_STATE}={previous}; previous developer state restored"
                );
            })
        } else {
            Ok(())
        };
        let mut sticks = Ok(());
        for (index, event) in flight_path::STICK_EVENTS.into_iter().enumerate() {
            if self.stick_channels[index] {
                match simulator.write(event, 0.0, None) {
                    Ok(()) => self.stick_channels[index] = false,
                    Err(error) => {
                        if sticks.is_ok() {
                            sticks = Err(error);
                        }
                    }
                }
            }
        }
        ready.and(fuel).and(sticks)?;
        self.reset();
        Ok(())
    }

    fn initialisation_timeout(&self) -> Result<Duration, InitialisationError> {
        let altitude = self
            .pressure_altitude
            .map_or(0.0, |altitude| altitude.ramp_seconds());
        let gear_pending = self.landing_gear.is_some() && self.ready_gear.is_none();
        let speed = self
            .calibrated_airspeed
            .map_or(0.0, |speed| speed.ramp_seconds(gear_pending));
        let seconds =
            INITIALISATION_TIMEOUT.as_secs_f64() + altitude.max(speed) + self.flight_path_lead;
        Duration::try_from_secs_f64(seconds)
            .map_err(|_| InitialisationError::InvalidInitialisationDuration { seconds })
    }

    fn timeout_diagnostics(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        targets: InitialisationConfig,
    ) -> String {
        use configuration::{describe_deviation, read_native, read_simvar};

        let mut output = String::new();
        for (field, target, variable, tolerance, unit) in [
            (
                "zfw",
                targets.zfw,
                "L:A32NX_AIRFRAME_ZFW",
                MASS_TOLERANCE,
                "kg",
            ),
            (
                "gw",
                targets.gw,
                "L:A32NX_AIRFRAME_GW",
                MASS_TOLERANCE,
                "kg",
            ),
            (
                "gwcg",
                targets.gwcg,
                "L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC",
                CG_TOLERANCE,
                "% MAC",
            ),
        ] {
            if let Some(target) = target {
                describe_deviation(
                    &mut output,
                    field,
                    target,
                    read_native(simulator, variable),
                    tolerance,
                    unit,
                );
            }
        }
        let natural_flight =
            targets.has_flight_conditions() && self.stage == SetupStage::Stabilising;
        if let Some(target) = targets.ths
            && !natural_flight
        {
            let current = self
                .ths
                .as_ref()
                .ok_or(InitialisationError::MissingThsReadback)
                .and_then(|ths| ths.readback(simulator));
            describe_deviation(&mut output, "ths", target, current, THS_TOLERANCE, "deg");
        }
        for (field, target, variable, tolerance, unit) in [
            (
                "pressure_altitude",
                targets.pressure_altitude,
                altitude::PRESSURE_ALTITUDE,
                altitude::ALTITUDE_TOLERANCE,
                "feet",
            ),
            (
                "calibrated_airspeed",
                targets.calibrated_airspeed,
                airspeed::CALIBRATED_AIRSPEED,
                airspeed::SPEED_TOLERANCE,
                "knots",
            ),
        ] {
            if let Some(target) = target {
                describe_deviation(
                    &mut output,
                    field,
                    target,
                    read_simvar(simulator, variable, unit),
                    tolerance,
                    unit,
                );
            }
        }
        if targets.calibrated_airspeed.is_some()
            && let Some(airspeed) = self.calibrated_airspeed
        {
            airspeed.describe_verification(&mut output);
        }
        for (index, field, target) in [
            (
                0,
                "pitch",
                targets
                    .pitch
                    .or(targets.has_flight_conditions().then_some(0.0)),
            ),
            (
                1,
                "roll",
                targets
                    .roll
                    .or(targets.has_flight_conditions().then_some(0.0)),
            ),
        ] {
            if let Some(target) = target {
                if field == "pitch" && natural_flight {
                    continue;
                }
                let (variable, unit, tolerance) = attitude::ATTITUDE_VARIABLES[index];
                let current = read_simvar(simulator, variable, unit).map(|value| {
                    let value = -value;
                    if index == 1 {
                        target + (value - target + 180.0).rem_euclid(360.0) - 180.0
                    } else {
                        value
                    }
                });
                describe_deviation(&mut output, field, target, current, tolerance, "deg");
                if !natural_flight {
                    let (variable, unit, tolerance) = attitude::ATTITUDE_VARIABLES[index + 2];
                    describe_deviation(
                        &mut output,
                        &format!("{field}_rate"),
                        0.0,
                        read_simvar(simulator, variable, unit).map(|value| -value),
                        tolerance,
                        "deg/s",
                    );
                }
            }
        }
        if targets.has_flight_conditions() {
            describe_deviation(
                &mut output,
                "vertical_speed",
                targets.vertical_speed.unwrap_or(0.0),
                read_simvar(simulator, airspeed::VERTICAL_SPEED, "feet/minute"),
                flight_path::VS_TOLERANCE,
                "ft/min",
            );
            if let Some(path) = self.flight_path {
                output.push_str(&format!(
                    "\n  flight_path: normal_law={} settled={} trim_rate={:?} deg/s",
                    path.normal_law,
                    path.is_ready(),
                    path.trim_rate
                ));
            }
        }
        output
    }

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
            .local_variable_exists(AIRCRAFT_READY)
            .unwrap_or(false)
    }

    fn submit(
        &mut self,
        simulator: &mut dyn SimulatorAdapter,
        targets: InitialisationConfig,
        now: Duration,
    ) -> Result<(), InitialisationError> {
        let mut next = Self::prepare(targets, simulator, now, self)?;
        if self.stage == SetupStage::NotStarted && targets.has_targets() {
            // Retain ownership even if the SDK reports failure after applying the write.
            self.stage = if targets.has_mass_balance() {
                SetupStage::Loading
            } else {
                SetupStage::Repositioning
            };
            simulator
                .write(AIRCRAFT_READY, 0.0, None)
                .map_err(InitialisationError::Submit)?;
            println!("TESTPILOT: {AIRCRAFT_READY}=0; initialising");
        }
        next.stage = self.stage;
        if self.previous_developer_state.is_none()
            && let Some(previous) = next.previous_developer_state
        {
            // Retain the original before writing so cleanup also handles a partial SDK failure.
            self.previous_developer_state = Some(previous);
            simulator
                .write(DEVELOPER_STATE, 2.0, None)
                .map_err(InitialisationError::Submit)?;
            println!("TESTPILOT: {DEVELOPER_STATE}=2; fuel burn inhibited during initialisation");
        }
        if next.stage == SetupStage::Loading {
            if !next.loading_complete && targets.has_mass_balance() {
                // A successful submission is retained while boarding/refuelling runs.
                // Leave its demands and start flags untouched while waiting.
                if self.mass_balance.is_none()
                    && let Some(mass_balance) = &next.mass_balance
                {
                    mass_balance.submit(simulator)?;
                }
                *self = next;
                return Ok(());
            }
            next.stage = SetupStage::Repositioning;
            println!("TESTPILOT: loading complete; repositioning");
        }
        let natural_flight =
            self.stage == SetupStage::Stabilising && targets.has_flight_conditions();
        if !natural_flight && let Some(ths) = &next.ths {
            ths.submit(simulator)?;
        }
        if let Some(flaps) = &next.flaps {
            flaps.submit(simulator)?;
        }
        if !natural_flight {
            next.attitude.submit(simulator)?;
        }
        if !natural_flight && let Some(pressure_altitude) = &next.pressure_altitude {
            pressure_altitude.submit(simulator)?;
        }
        if let Some(landing_gear) = &next.landing_gear {
            landing_gear.submit(simulator)?;
        }
        if let Some(calibrated_airspeed) = &next.calibrated_airspeed {
            calibrated_airspeed.submit(simulator)?;
        }
        if let Some(path) = &next.flight_path {
            // Preserve ownership before a potentially partial pair of event writes.
            self.stick_channels = [true; 2];
            next.stick_channels = [true; 2];
            path.submit(simulator)?;
        }
        *self = next;
        if self.stage == SetupStage::Repositioning
            && self
                .pressure_altitude
                .is_none_or(|altitude| altitude.ready_for_release())
            && self
                .calibrated_airspeed
                .is_none_or(|airspeed| airspeed.ramp_finished())
        {
            simulator
                .write(AIRCRAFT_READY, 1.0, None)
                .map_err(InitialisationError::Submit)?;
            self.stage = SetupStage::Stabilising;
            println!("TESTPILOT: {AIRCRAFT_READY}=1; stabilising");
            if let Some(airspeed) = &mut self.calibrated_airspeed {
                airspeed.begin_stabilising(now);
            }
        }
        Ok(())
    }

    fn pending_targets(
        &mut self,
        simulator: &mut dyn SimulatorAdapter,
        targets: InitialisationConfig,
    ) -> Result<InitialisationConfig, InitialisationError> {
        let zfw = self.check_zfw(simulator, targets.zfw)?;
        let gw = self.check_gw(simulator, targets.gw)?;
        let gwcg = self.check_gwcg(simulator, targets.gwcg)?;
        if self.stage == SetupStage::Loading {
            self.loading_complete = zfw && gw && gwcg;
            return Ok(InitialisationConfig {
                zfw: targets.zfw.filter(|_| !zfw),
                gw: targets.gw.filter(|_| !gw),
                gwcg: targets.gwcg.filter(|_| !gwcg),
                ..targets
            });
        }
        let ths = self.check_ths(simulator, targets.ths)?;
        let flaps = self.check_flaps(simulator, targets.flaps)?;
        let gear = self.check_landing_gear(simulator, targets.landing_gear)?;
        self.ready_gear = targets.landing_gear.filter(|_| gear);
        let stabilising = self.stage == SetupStage::Stabilising;
        let flight_setup = targets.has_flight_conditions();
        let altitude = self.check_pressure_altitude(simulator, targets.pressure_altitude)?
            && (targets.pressure_altitude.is_none() || stabilising);
        let speed = self.check_calibrated_airspeed(simulator, targets.calibrated_airspeed)?;
        let pitch = self.check_pitch(simulator, targets.pitch)?;
        let roll = self.check_roll(simulator, targets.roll)?;
        let path_ready = !flight_setup || self.flight_path.is_some_and(|path| path.is_ready());
        // Speed and flight path each own their dwell interval. A transient path
        // correction must not discard an otherwise steady airspeed history.
        let configuration_ready = zfw && gw && gwcg && ths && flaps && gear;
        if (!configuration_ready || !stabilising)
            && let Some(airspeed) = &mut self.calibrated_airspeed
        {
            airspeed.invalidate_stability();
        }
        if let (Some(path), Some(target)) = (self.flight_path, targets.pressure_altitude)
            && path.missed_altitude(target)
        {
            return Err(InitialisationError::MissedPlaybackAltitude {
                target,
                current: path.altitude.unwrap_or(target),
                vertical_speed: path.target,
            });
        }
        Ok(InitialisationConfig {
            zfw: targets.zfw.filter(|_| !zfw),
            gw: targets.gw.filter(|_| !gw),
            gwcg: targets.gwcg.filter(|_| !gwcg),
            ths: targets.ths.filter(|_| !ths),
            flaps: targets.flaps.filter(|_| !flaps),
            landing_gear: targets.landing_gear.filter(|_| !gear),
            pressure_altitude: targets.pressure_altitude.filter(|_| !altitude),
            calibrated_airspeed: targets
                .calibrated_airspeed
                .filter(|_| !configuration_ready || !speed),
            vertical_speed: if flight_setup && !path_ready {
                Some(targets.vertical_speed.unwrap_or(0.0))
            } else {
                None
            },
            pitch: targets.pitch.filter(|_| !pitch),
            roll: targets.roll.filter(|_| !roll),
        })
    }

    fn check_zfw(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError> {
        self.mass_balance_ready(
            simulator,
            "zfw",
            "L:A32NX_AIRFRAME_ZFW",
            target,
            MASS_TOLERANCE,
        )
    }

    fn check_gw(
        &self,
        simulator: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError> {
        self.mass_balance_ready(
            simulator,
            "gw",
            "L:A32NX_AIRFRAME_GW",
            target,
            MASS_TOLERANCE,
        )
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
            CG_TOLERANCE,
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
        if self.flight_vertical_speed.is_some() {
            return Ok(true);
        }
        let ths = self
            .ths
            .as_ref()
            .ok_or(InitialisationError::MissingThsReadback)?;
        Self::within("THS", ths.readback(simulator)?, target, THS_TOLERANCE)
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
        Ok(gear.is_ready(gear.readback(simulator)?, target))
    }

    fn check_pressure_altitude(
        &self,
        _: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError> {
        let Some(target) = target else {
            return Ok(true);
        };
        if let Some(path) = self.flight_path {
            return Ok(path.altitude_ready(target));
        }
        let altitude =
            self.pressure_altitude
                .as_ref()
                .ok_or(InitialisationError::MissingReadback {
                    field: "pressure_altitude",
                })?;
        Ok(altitude.is_ready(target) && self.attitude.is_ready())
    }

    fn check_calibrated_airspeed(
        &self,
        _: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError> {
        let Some(target) = target else {
            return Ok(true);
        };
        let airspeed =
            self.calibrated_airspeed
                .as_ref()
                .ok_or(InitialisationError::MissingReadback {
                    field: "calibrated_airspeed",
                })?;
        Ok(airspeed.is_ready(target))
    }

    fn check_pitch(
        &self,
        _: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError> {
        Ok(self.flight_vertical_speed.is_some() || self.attitude.pitch_ready(target))
    }

    fn check_roll(
        &self,
        _: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError> {
        Ok(self
            .flight_path
            .is_some_and(|path| (path.bank - target.unwrap_or(0.0)).abs() <= 0.5)
            || self.flight_path.is_none() && self.attitude.roll_ready(target))
    }

    fn check_vertical_speed(
        &self,
        _: &mut dyn SimulatorAdapter,
        target: Option<f64>,
    ) -> Result<bool, InitialisationError> {
        Ok(target.is_none() || self.flight_path.is_some_and(|path| path.is_ready()))
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
        fn write(
            &mut self,
            variable: &str,
            value: f64,
            _unit: Option<&str>,
        ) -> Result<(), SimulatorError> {
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
            targets: [60000.0, 65000.0, 25.0],
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
                    pressure_altitude: None,
                    calibrated_airspeed: None,
                    vertical_speed: None,
                    pitch: None,
                    roll: None,
                },
                Duration::ZERO,
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
