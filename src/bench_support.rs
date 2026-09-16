//! Opt-in access to the production frame runtime for host benchmarks.
//!
//! This facade is intended for measurement, not as a stable application API.

use std::path::PathBuf;

use crate::error::GaugeError;
use crate::replayer::Replayer;

pub use crate::gauge_runtime::GaugeRuntime;
pub use crate::simulator::SimulatorAdapter;

/// Builds the real frame runtime with a benchmark config and simulator adapter.
///
/// Construction resets the adapter's arming variable. The caller must then arm
/// and process the initial frame before timing steady-state updates.
pub fn new_runtime<S: SimulatorAdapter>(
    config_path: impl Into<PathBuf>,
    simulator: S,
) -> Result<GaugeRuntime<S>, GaugeError> {
    GaugeRuntime::new(Replayer::with_config_path(config_path), simulator)
}
