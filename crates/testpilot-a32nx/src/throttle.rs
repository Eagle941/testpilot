//! Direct A32NX throttle-axis commands, independent of mapping LVARs.
//!
//! Verified against aircraft revision ce46d9dbc7a90bd9afddc75d61c64316a6f78d0d:
//! ThrottleAxisMapping divides EX1 event values by 16384, then applies each
//! engine's calibration internally. Its default idle detent spans -0.55..-0.45;
//! -8192 selects the centre (-0.5). The MVP uses that fixed idle floor without
//! querying throttle mappings or automation state.

use testpilot_core::error::InitialisationError;
use testpilot_core::simulator::SimulatorAdapter;

/// Absolute throttle events accepted by the A32NX WASM event handler.
pub(super) const THROTTLE_EVENTS: [&str; 2] =
    ["K:THROTTLE1_AXIS_SET_EX1", "K:THROTTLE2_AXIS_SET_EX1"];
/// Fixed idle floor for the standard A32NX axis mapping (-0.5 * 16384).
pub(super) const IDLE_AXIS: f64 = -8192.0;
/// Common raw EX1 demand for both thrust levers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct A32nxThrottles {
    /// Continuous raw axis value; quantise only when submitting events.
    axis: f64,
}

impl A32nxThrottles {
    /// Starts the ramp at idle instead of commanding the reverse range.
    pub(super) const fn new() -> Self {
        Self { axis: IDLE_AXIS }
    }

    /// Applies a raw increment between idle and maximum, returning whether it stayed fixed.
    pub(super) fn adjust(&mut self, change: f64) -> bool {
        let previous = self.axis;
        self.axis = (self.axis + change).clamp(IDLE_AXIS, 16384.0);
        self.axis == previous
    }

    /// Submits the current demands through the same EX1 events used by CSV injection.
    pub(super) fn submit(
        &self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<(), InitialisationError> {
        for event in THROTTLE_EVENTS {
            let command = self.axis.round();
            simulator
                .write(event, command, None)
                .map_err(InitialisationError::Submit)?;
        }
        Ok(())
    }
}
