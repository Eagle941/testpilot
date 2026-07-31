//! Incremental playback cursors and optional streaming validation for the
//! independently sampled scenario CSV format.
//!
//! Playback opens one read-only file cursor per injection and retains two
//! samples per signal, so memory use does not grow with scenario duration.

mod cursor;
mod validation;

pub use crate::error::ScenarioError;
pub use cursor::{InterpolationRows, ScenarioPlayback};
pub use validation::{validate_scenario, ScenarioSummary, SignalSummary};

#[cfg(test)]
mod tests;
