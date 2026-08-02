//! Incremental playback cursors and optional streaming validation for the
//! independently sampled scenario CSV format.
//!
//! Playback opens one read-only file cursor per injection and retains two
//! samples per signal, so memory use does not grow with scenario duration.

mod cursor;
mod validation;

pub use crate::error::ScenarioError;
pub use cursor::{InterpolationRows, ScenarioPlayback};
pub use validation::{ScenarioSummary, SignalSummary, validate_scenario};

#[cfg(test)]
mod tests;
