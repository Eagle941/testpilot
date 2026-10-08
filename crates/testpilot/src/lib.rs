//! Replay parsing and playback interfaces plus the MSFS WASM gauge entry point.
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

#[cfg(target_arch = "wasm32")]
/// MSFS gauge entrypoint and event loop.
mod gauge;

/// Replay interfaces retained at their original `testpilot` module paths.
pub use testpilot_core::{config, cursor, error, initialisation, playback, recording};
