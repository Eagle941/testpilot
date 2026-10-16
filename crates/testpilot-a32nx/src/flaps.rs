//! A32NX flap/slat target preparation, command submission and motion readback.
//!
//! Mappings verified against FlyByWire aircraft revision
//! ce46d9dbc7a90bd9afddc75d61c64316a6f78d0d: systems_wasm flaps aspect and
//! common hydraulic/flap_slat implementation.

use testpilot_core::error::InitialisationError;
use testpilot_core::simulator::SimulatorAdapter;

use crate::configuration::{read_native, validate_interface};

/// Native motion flags published from the flap/slat assembly's speed.
pub(super) const MOVEMENT_VARIABLES: [&str; 2] =
    ["L:A32NX_IS_FLAPS_MOVING", "L:A32NX_IS_SLATS_MOVING"];

/// Validated absolute flap event, prepared before any setup writes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct A32nxFlaps {
    /// Absolute event for the configured flap lever detent.
    event: &'static str,
}

impl A32nxFlaps {
    /// Validates the lever demand and both motion interfaces without writing commands.
    pub(super) fn prepare(
        simulator: &mut dyn SimulatorAdapter,
        detent: u8,
    ) -> Result<Self, InitialisationError> {
        let event = flap_event(detent)?;
        for variable in MOVEMENT_VARIABLES {
            validate_interface(simulator, variable)?;
        }
        Ok(Self { event })
    }

    /// Submits the prepared absolute flap lever demand.
    pub(super) fn submit(
        &self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<(), InitialisationError> {
        simulator
            .write(self.event, 0.0)
            .map_err(InitialisationError::Submit)
    }

    /// Reads actual flap and slat motion flags, in that order.
    pub(super) fn readback(
        &self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<(bool, bool), InitialisationError> {
        Ok((
            read_moving(simulator, MOVEMENT_VARIABLES[0])?,
            read_moving(simulator, MOVEMENT_VARIABLES[1])?,
        ))
    }
}

/// Validates a flap lever demand and selects its absolute event.
fn flap_event(detent: u8) -> Result<&'static str, InitialisationError> {
    match detent {
        0 => Ok("K:FLAPS_UP"),
        1 => Ok("K:FLAPS_1"),
        2 => Ok("K:FLAPS_2"),
        3 => Ok("K:FLAPS_3"),
        4 => Ok("K:FLAPS_DOWN"),
        _ => Err(InitialisationError::InvalidFlapsTarget { detent }),
    }
}

/// Reads an exact boolean motion flag without assuming nonzero means true.
fn read_moving(
    simulator: &mut dyn SimulatorAdapter,
    variable: &'static str,
) -> Result<bool, InitialisationError> {
    let value = read_native(simulator, variable)?;
    match value {
        0.0 => Ok(false),
        1.0 => Ok(true),
        _ => Err(InitialisationError::InvalidReadback {
            variable,
            value,
            reason: "movement flag must be exactly 0 or 1",
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn absolute_events_cover_every_lever_setting() {
        for (detent, event) in [
            "K:FLAPS_UP",
            "K:FLAPS_1",
            "K:FLAPS_2",
            "K:FLAPS_3",
            "K:FLAPS_DOWN",
        ]
        .into_iter()
        .enumerate()
        {
            assert_eq!(flap_event(detent as u8).unwrap(), event);
        }
        assert!(matches!(
            flap_event(5),
            Err(InitialisationError::InvalidFlapsTarget { detent: 5 })
        ));
    }
}
