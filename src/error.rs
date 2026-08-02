//! Typed domain errors produced by configuration, scenario, and playback logic.
//!
//! File-system and simulator-facing variants retain underlying failures as
//! inspectable error sources.

use std::io;
use std::num::ParseFloatError;
use std::path::PathBuf;
use std::time::Duration;

use thiserror::Error;

/// Validation failures for replay TOML contents.
#[derive(Debug, Error)]
pub enum ConfigError {
    #[error(transparent)]
    FileIo(#[from] io::Error),

    #[error("invalid TOML configuration: {0}")]
    Toml(#[from] toml::de::Error),

    #[error("unsupported format_version {found}; expected {expected}")]
    UnsupportedFormatVersion { found: u32, expected: u32 },

    #[error("unsupported aircraft_target `{found}`; expected `{expected}")]
    UnsupportedAircraftTarget {
        found: String,
        expected: &'static str,
    },

    #[error("{section} section must contain at least one entry")]
    EmptySection { section: &'static str },

    #[error("invalid {section} index `{index}`: indexes must be canonical non-negative integers")]
    InvalidIndex {
        section: &'static str,
        index: String,
    },

    #[error("non-contiguous {section} indexes: expected {expected}, found {found}")]
    NonContiguousIndex {
        section: &'static str,
        expected: usize,
        found: usize,
    },

    #[error("invalid {field} at inject.{index}: {reason}")]
    InvalidInjectionRange {
        index: usize,
        field: &'static str,
        reason: &'static str,
    },

    #[error("simulator_range at inject.{index} must remain within [-16383, 16384]")]
    UnsafeSimulatorRange { index: usize },

    #[error("duplicate injection signal `{name}` at inject.{index}")]
    DuplicateInjectionSignal { index: usize, name: String },

    #[error("column `{column}` is reused at inject.{index}; time and value columns must be unique")]
    ReusedInjectionColumn { index: usize, column: String },

    #[error("{field} at inject.{index} must not be empty")]
    EmptyInjectionColumn { index: usize, field: &'static str },

    #[error("name at record.{index} must not be empty")]
    EmptyRecordingName { index: usize },

    #[error("variable at record.{index} must not be empty")]
    EmptyRecordingVariable { index: usize },

    #[error("unit at record.{index} is required for A: variables")]
    MissingRecordingUnit { index: usize },

    #[error("unit at record.{index} must not be empty")]
    EmptyRecordingUnit { index: usize },

    #[error("unit at record.{index} is only valid for A: variables, got `{variable}`")]
    UnexpectedRecordingUnit { index: usize, variable: String },

    #[error("duplicate recording signal `{name}` at record.{index}")]
    DuplicateRecordingSignal { index: usize, name: String },
}

/// Replay lifecycle and simulator-clock failures.
#[derive(Debug, Error)]
pub enum ReplayerError {
    #[error("a replay scenario is already loaded")]
    ScenarioAlreadyLoaded,

    #[error("configuration path `{path}` has no parent directory")]
    ConfigPathWithoutParent { path: PathBuf },

    #[error("scenario update requested while idle")]
    UpdateWhileIdle,

    #[error("simulation time moved backwards from {started_at:?} to {current:?}")]
    SimulationTimeMovedBackwards {
        started_at: Duration,
        current: Duration,
    },
}

/// MSFS calculator-code and simulator-variable failures.
#[derive(Debug, Error)]
pub enum SimulatorError {
    #[error("failed to read simulator time")]
    SimulationTimeUnavailable,

    #[error("invalid simulator time {value}")]
    InvalidSimulationTime { value: f64 },

    #[error("cannot write non-finite value {value} to `{variable}`")]
    NonFiniteWrite { variable: String, value: f64 },

    #[error("unsupported simulator variable `{variable}`; expected a non-empty K: or L: prefix")]
    UnsupportedVariable { variable: String },

    #[error("failed to build calculator code for `{variable}`: {source}")]
    CalculatorCodeFormatting {
        variable: String,
        #[source]
        source: std::fmt::Error,
    },

    #[error("calculator code failed while writing {value} to `{variable}`")]
    CalculatorCodeWriteFailed { variable: String, value: f64 },
}

/// Structural and numeric validation failures for scenario CSV input.
#[derive(Debug, Error)]
pub enum ScenarioError {
    #[error(transparent)]
    ParseInvalid(#[from] ParseFloatError),

    #[error(transparent)]
    Csv(#[from] csv::Error),

    #[error(transparent)]
    Playback(#[from] PlaybackError),

    #[error("duplicate CSV header `{column}` at indexes {first_index} and {duplicate_index}")]
    DuplicateHeader {
        column: String,
        first_index: usize,
        duplicate_index: usize,
    },

    #[error("missing column `{column}` for signal `{signal}`")]
    MissingColumn { signal: String, column: String },

    #[error(
        "time column `{time_column}` and value column `{value_column}` for signal `{signal}` must be adjacent and ordered time then value"
    )]
    NonAdjacentColumns {
        signal: String,
        time_column: String,
        value_column: String,
    },

    #[error("half-populated time/value pair for signal `{signal}`{line_suffix}", line_suffix = format_line(*line))]
    HalfPopulatedPair { signal: String, line: Option<u64> },

    #[error("internally sparse samples for signal `{signal}`{line_suffix}", line_suffix = format_line(*line))]
    SparseSeries { signal: String, line: Option<u64> },

    #[error("non-finite timestamp for signal `{signal}`{line_suffix}", line_suffix = format_line(*line))]
    NonFiniteTime { signal: String, line: Option<u64> },

    #[error("negative timestamp {time_seconds} for signal `{signal}`{line_suffix}", line_suffix = format_line(*line))]
    NegativeTime {
        signal: String,
        time_seconds: f64,
        line: Option<u64>,
    },

    #[error("timestamp {time_seconds} for signal `{signal}` exceeds Duration's range{line_suffix}", line_suffix = format_line(*line))]
    TimeOutOfRange {
        signal: String,
        time_seconds: f64,
        line: Option<u64>,
    },

    #[error("non-finite value for signal `{signal}`{line_suffix}", line_suffix = format_line(*line))]
    NonFiniteValue { signal: String, line: Option<u64> },

    #[error("first timestamp for signal `{signal}` must be 0 seconds, got {time:?}{line_suffix}", line_suffix = format_line(*line))]
    FirstTimestampNotZero {
        signal: String,
        time: Duration,
        line: Option<u64>,
    },

    #[error("timestamp {time:?} for signal `{signal}` must be greater than {previous:?}{line_suffix}", line_suffix = format_line(*line))]
    NonIncreasingTime {
        signal: String,
        previous: Duration,
        time: Duration,
        line: Option<u64>,
    },

    #[error("value {value} for signal `{signal}` is outside [{minimum}, {maximum}]{line_suffix}", line_suffix = format_line(*line))]
    ValueOutsideSourceRange {
        signal: String,
        value: f64,
        minimum: f64,
        maximum: f64,
        line: Option<u64>,
    },

    #[error("signal `{signal}` contains no samples")]
    MissingSamples { signal: String },
}

/// Per-frame gauge orchestration and injection failures.
#[derive(Debug, Error)]
pub enum GaugeError {
    #[error("failed to interpolate signal `{signal}`: {source}")]
    InterpolateSignal {
        signal: String,
        #[source]
        source: PlaybackError,
    },

    #[error("failed to convert signal `{signal}`: {source}")]
    ConvertSignal {
        signal: String,
        #[source]
        source: PlaybackError,
    },

    #[error("failed to inject signal `{signal}`: {source}")]
    InjectSignal {
        signal: String,
        #[source]
        source: SimulatorError,
    },
}

/// Invalid samples, interpolation requests, and range conversions.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum PlaybackError {
    #[error("sample value must be finite, got {value}")]
    NonFiniteValue { value: f64 },

    #[error("segment timestamps must increase, got {start:?} then {end:?}")]
    NonIncreasingSegment { start: Duration, end: Duration },

    #[error("time {time:?} is outside interpolation segment [{start:?}, {end:?}]")]
    TimeOutsideSegment {
        time: Duration,
        start: Duration,
        end: Duration,
    },

    #[error("invalid {name} range: {reason}")]
    InvalidRange {
        name: &'static str,
        reason: &'static str,
    },

    #[error("value {value} is outside source range [{minimum}, {maximum}]")]
    ValueOutsideSourceRange {
        value: f64,
        minimum: f64,
        maximum: f64,
    },

    #[error("floating-point arithmetic overflowed")]
    ArithmeticOverflow,
}

fn format_line(line: Option<u64>) -> String {
    line.map_or_else(String::new, |line| format!(" at CSV line {line}"))
}
