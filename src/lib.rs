//! Simulator-independent replay parsing and playback primitives plus the MSFS
//! WASM gauge entry point.
//!
//! The public modules can be tested on the host. Direct `msfs-rs` integration
//! is compiled only for `wasm32` targets.

#![cfg_attr(
    not(test),
    deny(
        clippy::expect_used,
        clippy::panic,
        clippy::todo,
        clippy::unimplemented,
        clippy::unreachable,
        clippy::unwrap_used
    )
)]

#[cfg(any(target_arch = "wasm32", test))]
mod arm;

#[cfg(target_arch = "wasm32")]
mod gauge;

#[cfg(target_arch = "wasm32")]
mod gauge_runtime;

#[cfg(any(target_arch = "wasm32", test))]
mod replayer;

#[cfg(any(target_arch = "wasm32", test))]
mod simulator;

pub mod config;
pub mod error;
pub mod playback;
pub mod scenario;
