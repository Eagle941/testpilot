#!/usr/bin/env sh
set -eu

raw_artifact="target/wasm32-wasip1/release/testpilot.wasm"
msfs_artifact="target/wasm32-wasip1/release/testpilot-msfs.wasm"

if ! command -v wasm-opt >/dev/null 2>&1; then
    echo "error: wasm-opt is required to package the MSFS module" >&2
    exit 1
fi

# Calling rustc instead of `cargo build` to force `cdylib`
cargo rustc --locked --release --target wasm32-wasip1 --lib --crate-type cdylib

wasm-opt \
    -O1 \
    --signext-lowering \
    --enable-bulk-memory \
    --enable-nontrapping-float-to-int \
    -o "$msfs_artifact" \
    "$raw_artifact"

printf 'MSFS WASM artifact: %s\n' "$msfs_artifact"
