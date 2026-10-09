//! MSFS simulator I/O backed by the WASM SDK and legacy calculator code.

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
mod simulator;

#[cfg(any(target_arch = "wasm32", test))]
pub use simulator::MsfsSimulator;
