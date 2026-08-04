//! Incremental playback cursors for independently sampled scenario CSV inputs.
//!
//! Playback opens one read-only file cursor per injection and retains two
//! samples per signal, so memory use does not grow with scenario duration.

mod cursor;

pub use crate::error::ScenarioError;
pub use cursor::{Frame, Scenario};

#[cfg(test)]
mod tests;
