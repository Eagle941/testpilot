//! A32NX landing gear target preparation, command submission and lock readback.
//!
//! Mappings verified against FlyByWire aircraft revision
//! ce46d9dbc7a90bd9afddc75d61c64316a6f78d0d: systems_wasm gear aspect,
//! common landing_gear and shared/arinc429 implementations.

use testpilot_core::config::LandingGear;
use testpilot_core::error::InitialisationError;
use testpilot_core::simulator::SimulatorAdapter;

use crate::configuration::{read_native, validate_interface};

/// Packed discrete word 1 from each computer, including gear and door locks.
pub(super) const GEAR_VARIABLES: [&str; 2] = [
    "L:A32NX_LGCIU_1_DISCRETE_WORD_1",
    "L:A32NX_LGCIU_2_DISCRETE_WORD_1",
];

/// Maximum packed LGCIU value: a 32-bit payload and a two-bit status.
const MAX_PACKED_LGCIU: f64 = 0x3_ffff_ffff_u64 as f64;
/// ARINC 429 status that permits physical lock confirmation.
const NORMAL_OPERATION_STATUS: u64 = 3;
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
    /// Absolute event for the configured landing gear selection.
    event: &'static str,
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
        Ok(Self {
            event: gear_event(target),
        })
    }

    /// Submits the prepared absolute landing gear event.
    pub(super) fn submit(
        &self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<(), InitialisationError> {
        simulator
            .write(self.event, 0.0)
            .map_err(InitialisationError::Submit)
    }

    /// Reads and validates both computer words independently of the requested selection.
    pub(super) fn readback(
        &self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<[f64; 2], InitialisationError> {
        let mut lgciu_words = [0.0; 2];
        for (raw_word, variable) in lgciu_words.iter_mut().zip(GEAR_VARIABLES) {
            let value = read_native(simulator, variable)?;
            decode_lgciu(variable, value)?;
            *raw_word = value;
        }
        Ok(lgciu_words)
    }

    /// Accepts the requested locks and closed doors confirmed by one valid computer.
    pub(super) fn is_ready(
        actual: [f64; 2],
        target: LandingGear,
    ) -> Result<bool, InitialisationError> {
        let mut ready = false;
        for (value, variable) in actual.into_iter().zip(GEAR_VARIABLES) {
            if let Some(word) = decode_lgciu(variable, value)? {
                ready |= locks_ready(word, target);
            }
        }
        Ok(ready)
    }
}

/// Selects the absolute gear event rather than toggling an existing demand.
const fn gear_event(target: LandingGear) -> &'static str {
    match target {
        LandingGear::Up => "K:GEAR_UP",
        LandingGear::Down => "K:GEAR_DOWN",
    }
}

/// Decodes A32NX's numeric packing: f32 payload bits below a two-bit SSM.
fn decode_lgciu(variable: &'static str, value: f64) -> Result<Option<u32>, InitialisationError> {
    if !value.is_finite() || !(0.0..=MAX_PACKED_LGCIU).contains(&value) || value.fract() != 0.0 {
        return Err(InitialisationError::InvalidReadback {
            variable,
            value,
            reason: "packed LGCIU word must be an integer with a two-bit status and 32-bit payload",
        });
    }
    let packed = value as u64;
    let payload = f64::from(f32::from_bits(packed as u32));
    if !payload.is_finite()
        || !(0.0..=f64::from(u32::MAX)).contains(&payload)
        || payload.fract() != 0.0
    {
        return Err(InitialisationError::InvalidReadback {
            variable,
            value,
            reason: "LGCIU payload must encode a finite non-negative u32 integer as f32",
        });
    }
    // Failure warning, no computed data and functional test cannot confirm locks.
    let status = packed >> u32::BITS;
    Ok((status == NORMAL_OPERATION_STATUS).then_some(payload as u32))
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
            decode_lgciu(GEAR_VARIABLES[0], packed(word, 3)).unwrap(),
            Some(word)
        );
        for status in 0..3 {
            assert_eq!(
                decode_lgciu(GEAR_VARIABLES[0], packed(word, status)).unwrap(),
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
            assert!(matches!(decode_lgciu(GEAR_VARIABLES[0], value),
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
