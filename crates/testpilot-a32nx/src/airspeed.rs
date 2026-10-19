//! A32NX calibrated-airspeed setup with throttle settling before playback.
//!
//! MSFS 2020 documents AIRSPEED INDICATED in knots. The A32NX ADR maps its
//! computed airspeed to UpdateContext::indicated_airspeed in the common
//! navigation/adirs.rs implementation at aircraft revision
//! 76db0aef132fd9698fd4013e2d7239c517dd284e. Readiness uses observations from before
//! submission, since successful calculator evaluation does not prove acceptance.

use std::time::Duration;

use testpilot_core::error::InitialisationError;
use testpilot_core::simulator::SimulatorAdapter;

use crate::configuration::{ramp_towards, read_simvar, validate_flight_condition_target};
use crate::throttle::A32nxThrottles;

/// MSFS indicated airspeed used as computed CAS by the A32NX ADR.
pub(super) const CALIBRATED_AIRSPEED: &str = "A:AIRSPEED INDICATED";
/// Vertical-speed readback shared with flight-path feedback and timeout diagnostics.
pub(super) const VERTICAL_SPEED: &str = "A:VERTICAL SPEED";

/// Existing inclusive speed acceptance tolerance in knots.
pub(super) const SPEED_TOLERANCE: f64 = 1.0;
/// Maximum accepted pre-correction or unforced speed drift, in knots per second.
const DRIFT_TOLERANCE: f64 = 0.1;
/// Observation interval during the commanded IAS ramp.
const OBSERVATION_DURATION: Duration = Duration::from_millis(500);
/// Average small readback oscillations without delaying recovery for two seconds.
const STABILISING_OBSERVATION_DURATION: Duration = Duration::from_secs(1);
/// Discard delayed setter readback before capturing a new drift baseline.
const SETTLING_DURATION: Duration = Duration::from_millis(500);
/// Sustained verification with fixed levers and no IAS writes.
const STABLE_DURATION: Duration = Duration::from_secs(3);
/// Maximum raw EX1 reduction rate, reached at a feedback magnitude of 1 kt/s.
const THROTTLE_RATE: f64 = 4096.0;
/// Recover promptly from the idle seed after releasing inhibited computers.
const THROTTLE_INCREASE_RATE: f64 = 8192.0;
/// Gentle corrections near trim, including thrust increases while overspeed.
const FINE_THROTTLE_RATE: f64 = 2048.0;
/// Convert speed error into desired acceleration during throttle-only settling.
const SPEED_RECOVERY_SECONDS: f64 = 16.0;
/// Avoid holding maximum thrust merely to chase an unnecessarily high recovery rate.
const MAX_RECOVERY_ACCELERATION: f64 = 0.5;
/// IAS setpoint rate in knots per simulator second.
const AIRSPEED_RATE: f64 = 5.0;
/// Intermediate IAS ceiling while configured landing gear is not yet ready.
pub(super) const GEAR_TRANSIT_SPEED: f64 = 200.0;

/// Per-run airspeed target, actuator demand and bounded feedback history.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct A32nxCalibratedAirspeed {
    /// Strictly positive finite demand in knots.
    target: f64,
    /// Latest pre-write vertical speed, shared with flight-path feedback.
    pub(super) vertical_speed: f64,
    /// Intermediate IAS setpoint, starting at the initial observed speed.
    ramped_target: f64,
    /// Timestamp of the preceding setpoint update, independent of drift sampling.
    ramp_time: Duration,
    /// The IAS setpoint has reached its final target, allowing release once altitude is ready.
    ramp_finished: bool,
    /// Feedback is enabled only after the coordinator releases the aircraft computers.
    stabilising: bool,
    /// Simulator timestamp at the start of the current unforced observation interval.
    time: Duration,
    /// Timestamp of the latest IAS correction, including its readback settling interval.
    corrected_at: Duration,
    /// Actual baseline; absent until a later simulator update observes an IAS correction.
    reference_speed: Option<f64>,
    /// IAS setter demand, corrected from actual readback independently of throttle feedback.
    demanded: f64,
    /// Common raw demand for both throttle axes.
    throttles: A32nxThrottles,
    /// Most recent complete unforced drift sample, for verification diagnostics.
    last_drift: Option<f64>,
    /// Beginning of the current stable interval with IAS writes stopped.
    stable_since: Option<Duration>,
    /// Only a complete observation interval may advance the lever commands.
    write_throttles: bool,
    /// Whether this update should command IAS while the computers remain inhibited.
    write_airspeed: bool,
}

impl A32nxCalibratedAirspeed {
    /// Validates the demand and captures readback before issuing any command.
    pub(super) fn prepare(
        simulator: &mut dyn SimulatorAdapter,
        target: f64,
        now: Duration,
        previous: Option<Self>,
        gear_pending: bool,
    ) -> Result<Self, InitialisationError> {
        validate_flight_condition_target("calibrated_airspeed", target)?;
        let observed = read_simvar(simulator, CALIBRATED_AIRSPEED, "knots")?;
        if observed < 0.0 {
            return Err(InitialisationError::InvalidReadback {
                variable: CALIBRATED_AIRSPEED,
                value: observed,
                reason: "airspeed must be non-negative",
            });
        }
        let vertical_speed = read_simvar(simulator, VERTICAL_SPEED, "feet/minute")?;
        let goal = if gear_pending {
            target.min(GEAR_TRANSIT_SPEED)
        } else {
            target
        };
        if let Some(mut control) = previous.filter(|control| control.target == target) {
            control.vertical_speed = vertical_speed;
            if now < control.ramp_time {
                return Err(InitialisationError::ClockMovedBackwards {
                    previous: control.ramp_time,
                    current: now,
                });
            }
            if !control.ramp_finished {
                control.ramped_target = ramp_towards(
                    control.ramped_target,
                    goal,
                    AIRSPEED_RATE * (now - control.ramp_time).as_secs_f64(),
                );
            }
            control.ramp_time = now;
            let ramp_complete = goal == target && control.ramped_target == target;
            control.advance(observed, now, ramp_complete)?;
            return Ok(control);
        }
        Ok(Self {
            target,
            vertical_speed,
            ramped_target: observed,
            ramp_time: now,
            ramp_finished: goal == target && observed == target,
            stabilising: false,
            time: now,
            corrected_at: now,
            reference_speed: None,
            demanded: observed,
            throttles: A32nxThrottles::new(),
            last_drift: None,
            stable_since: None,
            write_throttles: true,
            write_airspeed: true,
        })
    }

    /// Advances feedback once using elapsed simulator time and pre-write observations.
    fn advance(
        &mut self,
        observed: f64,
        now: Duration,
        ramp_complete: bool,
    ) -> Result<(), InitialisationError> {
        let elapsed = now - self.time;
        self.write_throttles = false;
        self.write_airspeed = false;
        let Some(reference_speed) = self.reference_speed else {
            // Initial readback may echo the command before physics applies it.
            // Wait for it to settle, then start a separate observation interval.
            if elapsed >= SETTLING_DURATION {
                self.reference_speed = Some(observed);
                // Feedback integrates only the interval with a valid baseline.
                self.time = now;
            }
            return Ok(());
        };
        if elapsed < self.observation_duration() {
            // Check speed on every frame, but measure drift only over complete
            // observation intervals. Short frames amplify small readback steps
            // and can otherwise keep resetting a genuinely stable speed window.
            if observed < self.target - SPEED_TOLERANCE || observed > self.target + SPEED_TOLERANCE
            {
                self.invalidate_stability();
            }
            return Ok(());
        }
        self.write_throttles = true;
        let seconds = elapsed.as_secs_f64();
        let correction_age = (now - self.corrected_at).as_secs_f64();
        let drift = (observed - reference_speed) / seconds;
        self.last_drift = Some(drift);
        let stable_speed = self.stable_sample(observed);
        let speed_error = self.target - observed;
        let desired_drift = (speed_error / SPEED_RECOVERY_SECONDS)
            .clamp(-MAX_RECOVERY_ACCELERATION, MAX_RECOVERY_ACCELERATION);
        let acceleration_error = desired_drift - drift;
        let throttle_rate = if acceleration_error < 0.0
            && (speed_error.abs() > 2.0 * SPEED_TOLERANCE || drift > 2.0 * DRIFT_TOLERANCE)
        {
            // Reduce promptly as recovery acceleration falls, including near target.
            THROTTLE_RATE
        } else if speed_error > 2.0 * SPEED_TOLERANCE {
            THROTTLE_INCREASE_RATE
        } else {
            // Deceleration while overspeed is already moving in the right direction.
            // A large increase here reverses the engine response before it can settle.
            FINE_THROTTLE_RATE
        };
        // Inhibited computers cannot respond to thrust commands. Keep the idle
        // seed until release, rather than integrating artificial reposition drift.
        let increment = if !self.stabilising || stable_speed {
            0.0
        } else {
            acceleration_error.clamp(-1.0, 1.0) * throttle_rate * seconds
        };
        let fixed_levers = self.throttles.adjust(increment);
        if stable_speed && fixed_levers {
            self.stable_since.get_or_insert(now);
        } else {
            self.invalidate_stability();
        }
        self.write_airspeed = !self.stabilising;
        if self.write_airspeed {
            let maximum_change = AIRSPEED_RATE * correction_age;
            let demanded = self.demanded
                + (self.ramped_target - observed).clamp(-maximum_change, maximum_change);
            if !demanded.is_finite() || demanded < 0.0 {
                return Err(InitialisationError::InvalidAirspeedCorrection {
                    previous_demand: self.demanded,
                    observed,
                    target: self.ramped_target,
                    value: demanded,
                });
            }
            self.demanded = demanded;
        }
        // Reaching the speed target permits release, but altitude may still be travelling.
        // Keep commanding IAS until release: inhibited computers cannot provide thrust feedback.
        // The temporary gear-transit goal must not permit early release.
        self.ramp_finished |= ramp_complete;
        self.time = now;
        if self.write_airspeed {
            self.corrected_at = now;
            self.last_drift = None;
        }
        self.reference_speed = if self.write_airspeed {
            None
        } else {
            Some(observed)
        };
        Ok(())
    }

    /// Checks inclusive speed/drift bounds over a complete observation interval.
    fn stable_sample(&self, observed: f64) -> bool {
        self.stabilising
            && self.ramp_finished
            && observed >= self.target - SPEED_TOLERANCE
            && observed <= self.target + SPEED_TOLERANCE
            && self
                .last_drift
                .is_some_and(|drift| drift.abs() <= DRIFT_TOLERANCE)
    }

    /// Drift samples use only natural flight after release of the computers.
    const fn observation_duration(&self) -> Duration {
        if self.stabilising {
            STABILISING_OBSERVATION_DURATION
        } else {
            OBSERVATION_DURATION
        }
    }

    /// Restarts free-speed verification when another configured group is unsettled.
    pub(super) fn invalidate_stability(&mut self) {
        self.stable_since = None;
    }

    /// Whether the terminal IAS ramp command has been prepared.
    pub(super) const fn ramp_finished(&self) -> bool {
        self.ramp_finished
    }

    /// Discards observations made while aircraft systems were inhibited.
    pub(super) fn begin_stabilising(&mut self, now: Duration) {
        self.stabilising = true;
        self.throttles = A32nxThrottles::new();
        self.invalidate_stability();
        self.reference_speed = None;
        self.last_drift = None;
        self.time = now;
    }

    /// Initial speed travel allowance, including a detour through the gear transit speed.
    pub(super) fn ramp_seconds(&self, gear_pending: bool) -> f64 {
        let intermediate = if gear_pending {
            self.target.min(GEAR_TRANSIT_SPEED)
        } else {
            self.target
        };
        ((self.ramped_target - intermediate).abs() + (self.target - intermediate).abs())
            / AIRSPEED_RATE
    }

    /// Readiness is cached; checking it never commands or samples the simulator.
    pub(super) fn is_ready(&self, target: f64) -> bool {
        self.target == target
            && self
                .stable_since
                .is_some_and(|since| self.time - since >= STABLE_DURATION)
    }

    /// Appends cached verification state without additional simulator reads or commands.
    pub(super) fn describe_verification(&self, output: &mut String) {
        let stable_seconds = self
            .stable_since
            .map_or(0.0, |since| (self.time - since).as_secs_f64());
        let drift = self
            .last_drift
            .map_or_else(|| "pending".to_owned(), |drift| format!("{drift:.4} kt/s"));
        output.push_str(&format!(
            "\n  calibrated_airspeed: ready={} stable_for={stable_seconds:.3}/{:.3} s drift={drift} drift_tolerance=+/-{DRIFT_TOLERANCE:.4} kt/s sample_age={:.3} s observation_interval={:.3} s ramp_finished={} throttle_feedback={}",
            self.is_ready(self.target),
            STABLE_DURATION.as_secs_f64(),
            (self.ramp_time - self.time).as_secs_f64(),
            self.observation_duration().as_secs_f64(),
            self.ramp_finished,
            self.stabilising,
        ));
    }

    /// Submits both lever demands and, only before releasing the computers, the demanded IAS.
    pub(super) fn submit(
        &self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<(), InitialisationError> {
        if self.write_throttles {
            self.throttles.submit(simulator)?;
        }
        if self.write_airspeed {
            simulator
                .write(CALIBRATED_AIRSPEED, self.demanded, Some("knots"))
                .map_err(InitialisationError::Submit)?;
        }
        Ok(())
    }
}
