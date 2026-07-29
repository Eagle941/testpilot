//! Simulator-independent replay parsing and playback primitives plus the MSFS
//! WASM gauge entry point.
//!
//! The public modules can be tested on the host. Direct `msfs-rs` integration
//! is compiled only for `wasm32` targets.

#[cfg(any(target_arch = "wasm32", test))]
mod arm;

#[cfg(target_arch = "wasm32")]
mod gauge;

pub mod config;
pub mod error;
pub mod playback;
pub mod scenario;
