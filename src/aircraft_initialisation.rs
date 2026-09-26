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
pub trait AircraftInitialiser<S: SimulatorAdapter> {
    /// Checks this initialiser's model and interface requirements.
    /// Unavailable identifiers or interfaces must return false.
    fn supported_model(&mut self, simulator: &mut S) -> bool;

    /// Detects support once per armed run that requests initialisation.
    fn detect(&mut self, simulator: &mut S) -> AircraftSupport {
        if self.supported_model(simulator) {
            AircraftSupport::Supported
        } else {
            AircraftSupport::Unsupported
        }
    }

    /// Submits actual aircraft loading targets once on arming.
    fn submit(
        &mut self,
        simulator: &mut S,
        targets: InitialisationConfig,
    ) -> Result<(), InitialisationError>;

    /// Reads actual mass and balance, independently of configured telemetry signals.
    fn readback(&mut self, simulator: &mut S) -> Result<AircraftMassBalance, InitialisationError>;
}

/// A32NX loading component. Simulator mappings remain deliberately unimplemented.
pub struct A32nxInitialiser;

impl<S: SimulatorAdapter> AircraftInitialiser<S> for A32nxInitialiser {
    /// Matches the A20N model code and its configured ATC localisation key (see README).
    /// Requires the FlyByWire readiness variable to exist, without reading its value.
    fn supported_model(&mut self, simulator: &mut S) -> bool {
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
        _simulator: &mut S,
        _targets: InitialisationConfig,
    ) -> Result<(), InitialisationError> {
        // TODO: Verify and implement actual A32NX payload/fuel/balance injection through msfs-rs.
        Err(InitialisationError::NotImplemented {
            operation: "submission",
        })
    }

    fn readback(&mut self, _simulator: &mut S) -> Result<AircraftMassBalance, InitialisationError> {
        // TODO: Verify and implement actual A32NX ZFW/GW/GWCG readback in kg and percent MAC.
        Err(InitialisationError::NotImplemented {
            operation: "readback",
        })
    }
}
