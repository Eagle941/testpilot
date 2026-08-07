//! Simulator-independent replay parsing and playback primitives plus the MSFS
//! WASM gauge entry point.
//!
//! The public modules can be tested on the host. Direct `msfs-rs` integration
//! is compiled only for `wasm32` targets.

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

#[cfg(any(target_arch = "wasm32", test))]
/// Arming state and transition tracking used by runtime start/stop control.
mod arm;

#[cfg(target_arch = "wasm32")]
/// MSFS gauge entrypoint and event loop.
mod gauge;

#[cfg(target_arch = "wasm32")]
mod gauge_runtime;

#[cfg(any(target_arch = "wasm32", test))]
mod replayer;

#[cfg(any(target_arch = "wasm32", test))]
mod simulator;

pub mod config;
/// Scenario cursor abstraction, readers, and row interpolation sources.
pub mod cursor;
pub mod error;
pub mod playback;
pub mod recording;

#[cfg(test)]
mod tests {
    mod playback;
    mod shared;
    mod validation;
}
