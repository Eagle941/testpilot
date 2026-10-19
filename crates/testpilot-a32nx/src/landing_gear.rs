//! A32NX landing gear target preparation, command submission and lock readback.
//!
//! Mappings verified against FlyByWire aircraft revision
//! ce46d9dbc7a90bd9afddc75d61c64316a6f78d0d: systems_wasm gear aspect,
//! common landing_gear and shared/arinc429 implementations.

use testpilot_core::config::LandingGear;
use testpilot_core::error::InitialisationError;
use testpilot_core::simulator::SimulatorAdapter;

use crate::airspeed::CALIBRATED_AIRSPEED;
use crate::configuration::{decode_discrete_word, read_native, read_simvar, validate_interface};

/// Simulator contact state; gear-up commands wait for an airborne observation.
pub(super) const SIM_ON_GROUND: &str = "A:SIM ON GROUND";
/// SDK Mach readback for the published A32NX gear operating limits.
pub(super) const AIRSPEED_MACH: &str = "A:AIRSPEED MACH";

/// Packed discrete word 1 from each computer, including gear and door locks.
pub(super) const GEAR_VARIABLES: [&str; 2] = [
    "L:A32NX_LGCIU_1_DISCRETE_WORD_1",
    "L:A32NX_LGCIU_2_DISCRETE_WORD_1",
];

/// Discrete bit 29: gear selected down.
const GEAR_SELECTED_DOWN: u32 = 1 << 28;
/// Discrete bits 17-19: gear doors not closed and locked.
const DOORS_NOT_CLOSED: u32 = 0b111 << 16;
/// Discrete bits 11-13: gears not uplocked while up is selected.
const GEARS_NOT_UPLOCKED: u32 = 0b111 << 10;
/// Discrete bits 23-25: individual gears downlocked.
const GEARS_DOWNLOCKED: u32 = 0b111 << 22;

/// Validated absolute landing gear event, prepared before any setup writes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct A32nxLandingGear {
    /// Absolute event, withheld outside operating limits or for gear-up on the ground.
    event: Option<&'static str>,
    /// Ground inhibition is separate from the speed gate and physical lock readiness.
    on_ground: bool,
}

impl A32nxLandingGear {
    /// Validates both computer interfaces and prepares the absolute gear event without writes.
    pub(super) fn prepare(
        simulator: &mut dyn SimulatorAdapter,
        target: LandingGear,
    ) -> Result<Self, InitialisationError> {
        for variable in GEAR_VARIABLES {
            validate_interface(simulator, variable)?;
        }
        let on_ground = if target == LandingGear::Up {
            match read_simvar(simulator, SIM_ON_GROUND, "bool")? {
                0.0 => false,
                1.0 => true,
                value => {
                    return Err(InitialisationError::InvalidReadback {
                        variable: SIM_ON_GROUND,
                        value,
                        reason: "ground state must be exactly 0 or 1",
                    });
                }
            }
        } else {
            false
        };
        let airspeed = read_simvar(simulator, CALIBRATED_AIRSPEED, "knots")?;
        let mach = read_simvar(simulator, AIRSPEED_MACH, "mach")?;
        for (variable, value) in [(CALIBRATED_AIRSPEED, airspeed), (AIRSPEED_MACH, mach)] {
            if value < 0.0 {
                return Err(InitialisationError::InvalidReadback {
                    variable,
                    value,
                    reason: "gear operating speed must be non-negative",
                });
            }
        }
        // Published A32NX VLO: retraction 220 kt/M0.54, extension 250 kt/M0.60.
        let (maximum_airspeed, maximum_mach) = match target {
            LandingGear::Up => (220.0, 0.54),
            LandingGear::Down => (250.0, 0.60),
        };
        let event = (!on_ground && airspeed <= maximum_airspeed && mach <= maximum_mach)
            .then_some(gear_event(target));
        Ok(Self { event, on_ground })
    }

    /// Submits the prepared absolute landing gear event.
    pub(super) fn submit(
        &self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<(), InitialisationError> {
        if let Some(event) = self.event {
            simulator
                .write(event, 0.0, None)
                .map_err(InitialisationError::Submit)?;
        }
        Ok(())
    }

    /// Reads and validates both computer words independently of the requested selection.
    pub(super) fn readback(
        &self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<[Option<u32>; 2], InitialisationError> {
        let mut lgciu_words = [None; 2];
        for (raw_word, variable) in lgciu_words.iter_mut().zip(GEAR_VARIABLES) {
            *raw_word = decode_discrete_word(variable, read_native(simulator, variable)?)?;
        }
        Ok(lgciu_words)
    }

    /// Accepts the requested locks and closed doors confirmed by one valid computer.
    pub(super) fn is_ready(&self, actual: [Option<u32>; 2], target: LandingGear) -> bool {
        !self.on_ground
            && actual
                .into_iter()
                .flatten()
                .any(|word| locks_ready(word, target))
    }
}

/// Selects the absolute gear event rather than toggling an existing demand.
const fn gear_event(target: LandingGear) -> &'static str {
    match target {
        LandingGear::Up => "K:GEAR_UP",
        LandingGear::Down => "K:GEAR_DOWN",
    }
}

/// Checks one-based discrete bits against selection, physical locks and doors.
fn locks_ready(word: u32, target: LandingGear) -> bool {
    if word & DOORS_NOT_CLOSED != 0 {
        return false;
    }
    let selected_down = word & GEAR_SELECTED_DOWN != 0;
    match target {
        LandingGear::Up => !selected_down && word & GEARS_NOT_UPLOCKED == 0,
        LandingGear::Down => selected_down && word & GEARS_DOWNLOCKED == GEARS_DOWNLOCKED,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Packs a known discrete payload using the aircraft's f32 numeric representation.
    fn packed(word: u32, status: u64) -> f64 {
        (u64::from((word as f32).to_bits()) | (status << 32)) as f64
    }

    #[test]
    fn absolute_events_cover_both_gear_selections() {
        assert_eq!(gear_event(LandingGear::Up), "K:GEAR_UP");
        assert_eq!(gear_event(LandingGear::Down), "K:GEAR_DOWN");
    }

    #[test]
    fn decoder_reads_numeric_f32_payload_and_checks_status() {
        let word = (1 << 28) | (0b111 << 22) | (1 << 16);
        assert_eq!(
            decode_discrete_word(GEAR_VARIABLES[0], packed(word, 3)).unwrap(),
            Some(word)
        );
        for status in 0..3 {
            assert_eq!(
                decode_discrete_word(GEAR_VARIABLES[0], packed(word, status)).unwrap(),
                None
            );
        }
        for value in [
            -1.0,
            0.5,
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            0x4_0000_0000_u64 as f64,
            u64::from(f32::NAN.to_bits()) as f64,
            u64::from(f32::INFINITY.to_bits()) as f64,
            u64::from((-1.0_f32).to_bits()) as f64,
            u64::from(0.5_f32.to_bits()) as f64,
            u64::from(4294967296.0_f32.to_bits()) as f64,
        ] {
            assert!(matches!(decode_discrete_word(GEAR_VARIABLES[0], value),
                Err(InitialisationError::InvalidReadback { variable, value: actual, .. })
                if variable == GEAR_VARIABLES[0] && (actual == value || (actual.is_nan() && value.is_nan()))));
        }
    }

    #[test]
    fn gear_completion_requires_selection_each_lock_and_closed_doors() {
        let down = (1 << 28) | (0b111 << 22);
        assert!(locks_ready(down, LandingGear::Down));
        assert!(!locks_ready(down, LandingGear::Up));
        assert!(locks_ready(0, LandingGear::Up));
        assert!(!locks_ready(0, LandingGear::Down));
        assert!(!locks_ready(down & !(1 << 28), LandingGear::Down));
        for bit in 22..25 {
            assert!(!locks_ready(down & !(1 << bit), LandingGear::Down));
        }
        for bit in 10..13 {
            assert!(!locks_ready(1 << bit, LandingGear::Up));
        }
        for bit in 16..19 {
            assert!(!locks_ready(down | (1 << bit), LandingGear::Down));
            assert!(!locks_ready(1 << bit, LandingGear::Up));
        }
    }
}
