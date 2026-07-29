#!/usr/bin/env sh
set -eu

raw_artifact="target/wasm32-wasip1/release/replay.wasm"
msfs_artifact="target/wasm32-wasip1/release/replay-msfs.wasm"

if ! command -v wasm-opt >/dev/null 2>&1; then
    echo "error: wasm-opt is required to package the MSFS module" >&2
    exit 1
fi

cargo build --release --target wasm32-wasip1

wasm-opt \
    -O1 \
    --signext-lowering \
    --enable-bulk-memory \
    --enable-nontrapping-float-to-int \
    -o "$msfs_artifact" \
    "$raw_artifact"

printf 'MSFS WASM artifact: %s\n' "$msfs_artifact"
