//! Replay configuration, streaming, playback and lifecycle orchestration.
//!
//! Simulator and aircraft services are supplied through host-testable traits.

#![cfg_attr(
    not(test),
    deny(
        clippy::expect_used,
        clippy::missing_docs_in_private_items,
        clippy::panic,
        clippy::todo,
        clippy::unimplemented,
        clippy::unreachable,
        clippy::unwrap_used
    )
)]

/// Arming state and transition tracking used by runtime start/stop control.
mod arm;

pub mod aircraft_initialisation;
pub mod config;
/// Scenario cursor abstraction, readers, and row interpolation sources.
pub mod cursor;
pub mod error;
pub mod initialisation;
mod injection;
pub mod playback;
pub mod recording;
pub mod runtime;
pub mod simulator;
mod telemetry;

#[cfg(test)]
mod tests {
    mod playback;
    mod shared;
    mod validation;
}
