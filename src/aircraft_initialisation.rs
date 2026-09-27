//! Aircraft-specific loading operations, separate from generic simulator I/O and readiness checks.

mod a32nx;

pub use a32nx::A32nxInitialiser;

use crate::config::InitialisationConfig;
use crate::error::InitialisationError;
use crate::initialisation::AircraftInitialisationState;
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

    /// Submits all configured targets on each initialisation frame until ready.
    fn submit(
        &mut self,
        simulator: &mut dyn SimulatorAdapter,
        targets: InitialisationConfig,
    ) -> Result<(), InitialisationError>;

    /// Reads back aircraft configuration independently of telemetry signals.
    fn readback(
        &mut self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<AircraftInitialisationState, InitialisationError>;
}
