//! Replay configuration data types, parsing, and file loading.
//!
//! The parser accepts the versioned TOML contract documented in the crate
//! README and converts supported signal names and units into strongly typed
//! values used by the simulator-independent replay core.

use std::collections::{BTreeMap, HashSet};
use std::fs;
use std::path::{Path, PathBuf};

use anyhow::Context;
use serde::Deserialize;

pub use crate::error::ConfigError;

/// Configuration and scenario format version supported by this crate.
pub const FORMAT_VERSION: u32 = 1;

/// Aircraft target identifier accepted by the MVP configuration parser.
pub const AIRCRAFT_TARGET: &str = "flybywire-a32nx";

/// Package-relative path read by the MSFS WASM gauge when it is armed.
pub const CONFIG_PATH: &str = "SimObjects/AirPlanes/FlyByWire_A320_NEO/replayer_config.toml";

/// Validated replay configuration in deterministic processing order.
#[derive(Debug, Clone, PartialEq)]
pub struct ReplayConfig {
    /// Parsed format version. This is always [`FORMAT_VERSION`].
    pub format_version: u32,
    /// Parsed aircraft target. This is always [`AIRCRAFT_TARGET`].
    pub aircraft_target: String,
    /// Scenario CSV path exactly as specified by `input_file`.
    pub input_file: PathBuf,
    /// Injection definitions ordered by their numeric `inject.N` indexes.
    pub inject: Vec<InjectionConfig>,
    /// Recording definitions ordered by their numeric `record.N` indexes.
    pub record: Vec<RecordingConfig>,
}

/// Configuration for one continuous scenario input.
#[derive(Debug, Clone, PartialEq)]
pub struct InjectionConfig {
    /// Logical input signal name from the configuration.
    pub name: String,
    /// CSV column containing scenario-relative timestamps in seconds.
    pub time_column: String,
    /// CSV column containing source values.
    pub value_column: String,
    /// Engineering unit used by the source values.
    pub source_unit: SourceUnit,
    /// Inclusive, strictly increasing valid range for source values.
    pub source_range: [f64; 2],
    /// Engineering unit expected at the simulator boundary.
    pub simulator_unit: SimulatorUnit,
    /// Inclusive affine-conversion target range within the signal's safe range.
    pub simulator_range: [f64; 2],
}

/// Configuration for one aircraft-response signal recorded each frame.
#[derive(Debug, Clone, PartialEq)]
pub struct RecordingConfig {
    /// Supported logical response signal.
    pub name: RecordingSignal,
    /// Native engineering unit written to telemetry.
    pub unit: RecordingUnit,
    /// Exact supported range for the selected signal.
    pub range: [f64; 2],
}

/// Aircraft-response signals supported by the MVP.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RecordingSignal {
    /// Aggregate aircraft pitch attitude.
    Pitch,
    /// Aggregate aircraft roll attitude.
    Roll,
    /// Aggregate elevator position.
    ElevatorPosition,
    /// Aggregate aileron position.
    AileronPosition,
}

impl RecordingSignal {
    /// Returns the stable configuration and telemetry column name.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Pitch => "pitch",
            Self::Roll => "roll",
            Self::ElevatorPosition => "elevator_position",
            Self::AileronPosition => "aileron_position",
        }
    }
}

/// Engineering units accepted for scenario input values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceUnit {
    /// Percentage values, typically in `[-100, 100]`.
    Percent,
    /// Dimensionless normalized values, typically in `[-1, 1]`.
    Normalized,
}

impl SourceUnit {
    /// Returns the stable TOML unit name.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Percent => "percent",
            Self::Normalized => "normalized",
        }
    }
}

/// Engineering units supported at the simulator input boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimulatorUnit {
    /// Dimensionless normalized control position in `[-1, 1]`.
    Normalized,
}

impl SimulatorUnit {
    /// Returns the stable TOML unit name.
    pub const fn as_str(self) -> &'static str {
        "normalized"
    }
}

/// Native units supported for recorded response signals.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecordingUnit {
    /// Angular value in degrees.
    Degrees,
    /// MSFS `Position 16k` control-surface value.
    Position16k,
}

impl RecordingUnit {
    /// Returns the stable TOML unit name.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Degrees => "degrees",
            Self::Position16k => "position_16k",
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawReplayConfig {
    format_version: u32,
    aircraft_target: String,
    input_file: String,
    #[serde(default)]
    inject: BTreeMap<String, RawInjectionConfig>,
    #[serde(default)]
    record: BTreeMap<String, RawRecordingConfig>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawInjectionConfig {
    name: String,
    time_column: String,
    value_column: String,
    source_unit: String,
    source_range: [f64; 2],
    simulator_unit: String,
    simulator_range: [f64; 2],
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawRecordingConfig {
    name: String,
    unit: String,
    range: [f64; 2],
}

/// Reads and parses a replay configuration file.
///
/// The returned [`anyhow::Error`] adds the requested file path as context while
/// preserving the underlying I/O or [`ConfigError`] source.
pub fn read_config_file(path: impl AsRef<Path>) -> anyhow::Result<ReplayConfig> {
    read_config_file_with_contents(path).map(|(_, config)| config)
}

pub(crate) fn read_config_file_with_contents(
    path: impl AsRef<Path>,
) -> anyhow::Result<(String, ReplayConfig)> {
    let path = path.as_ref();
    let contents = fs::read_to_string(path)
        .with_context(|| format!("failed to read replay configuration `{}`", path.display()))?;
    let config = parse_config(&contents)
        .with_context(|| format!("invalid replay configuration `{}`", path.display()))?;

    Ok((contents, config))
}

/// Parses and validates the versioned replay TOML document.
///
/// Numeric `inject.N` and `record.N` tables must be contiguous from zero. Their
/// indexes define the order of the returned vectors.
pub fn parse_config(contents: &str) -> Result<ReplayConfig, ConfigError> {
    let raw: RawReplayConfig = toml::from_str(contents)?;

    if raw.format_version != FORMAT_VERSION {
        return Err(ConfigError::UnsupportedFormatVersion {
            found: raw.format_version,
            expected: FORMAT_VERSION,
        });
    }
    if raw.aircraft_target != AIRCRAFT_TARGET {
        return Err(ConfigError::UnsupportedAircraftTarget {
            found: raw.aircraft_target,
            expected: AIRCRAFT_TARGET,
        });
    }

    let inject = parse_injections(raw.inject)?;
    let record = parse_recordings(raw.record)?;

    Ok(ReplayConfig {
        format_version: raw.format_version,
        aircraft_target: raw.aircraft_target,
        input_file: PathBuf::from(raw.input_file),
        inject,
        record,
    })
}

impl ReplayConfig {
    /// Parses a replay configuration from TOML text.
    pub fn parse(contents: &str) -> Result<Self, ConfigError> {
        parse_config(contents)
    }
}

fn parse_injections(
    entries: BTreeMap<String, RawInjectionConfig>,
) -> Result<Vec<InjectionConfig>, ConfigError> {
    let entries = ordered_entries("inject", entries)?;
    let mut signals = HashSet::with_capacity(entries.len());
    let mut columns = HashSet::with_capacity(entries.len().saturating_mul(2));
    let mut result = Vec::with_capacity(entries.len());

    for (index, raw) in entries {
        if !signals.insert(raw.name.clone()) {
            return Err(ConfigError::DuplicateInjectionSignal {
                index,
                name: raw.name,
            });
        }

        validate_column(index, "time_column", &raw.time_column, &mut columns)?;
        validate_column(index, "value_column", &raw.value_column, &mut columns)?;

        let source_unit = match raw.source_unit.as_str() {
            "percent" => SourceUnit::Percent,
            "normalized" => SourceUnit::Normalized,
            _ => {
                return Err(ConfigError::UnsupportedSourceUnit {
                    index,
                    unit: raw.source_unit,
                });
            }
        };
        let simulator_unit = match raw.simulator_unit.as_str() {
            "normalized" => SimulatorUnit::Normalized,
            _ => {
                return Err(ConfigError::UnsupportedSimulatorUnit {
                    index,
                    unit: raw.simulator_unit,
                });
            }
        };

        validate_increasing_range(index, "source_range", raw.source_range)?;
        validate_increasing_range(index, "simulator_range", raw.simulator_range)?;
        if raw.simulator_range[0] < -1.0 || raw.simulator_range[1] > 1.0 {
            return Err(ConfigError::UnsafeSimulatorRange { index });
        }

        result.push(InjectionConfig {
            name: raw.name,
            time_column: raw.time_column,
            value_column: raw.value_column,
            source_unit,
            source_range: raw.source_range,
            simulator_unit,
            simulator_range: raw.simulator_range,
        });
    }

    Ok(result)
}

fn parse_recordings(
    entries: BTreeMap<String, RawRecordingConfig>,
) -> Result<Vec<RecordingConfig>, ConfigError> {
    let entries = ordered_entries("record", entries)?;
    let mut signals = HashSet::with_capacity(entries.len());
    let mut result = Vec::with_capacity(entries.len());

    for (index, raw) in entries {
        let name = parse_recording_signal(index, &raw.name)?;
        if !signals.insert(name) {
            return Err(ConfigError::DuplicateRecordingSignal {
                index,
                name: raw.name,
            });
        }

        let (unit, expected_range) = recording_catalog(name);
        if raw.unit != unit.as_str() {
            return Err(ConfigError::UnsupportedRecordingUnit {
                index,
                signal: name.as_str(),
                unit: raw.unit,
                expected: unit.as_str(),
            });
        }
        if !raw.range.iter().all(|endpoint| endpoint.is_finite()) || raw.range != expected_range {
            return Err(ConfigError::InvalidRecordingRange {
                index,
                signal: name.as_str(),
                expected_min: expected_range[0],
                expected_max: expected_range[1],
            });
        }

        result.push(RecordingConfig {
            name,
            unit,
            range: raw.range,
        });
    }

    Ok(result)
}

fn ordered_entries<T>(
    section: &'static str,
    entries: BTreeMap<String, T>,
) -> Result<Vec<(usize, T)>, ConfigError> {
    if entries.is_empty() {
        return Err(ConfigError::EmptySection { section });
    }

    let mut indexed = Vec::with_capacity(entries.len());
    for (key, value) in entries {
        let Ok(index) = key.parse::<usize>() else {
            return Err(ConfigError::InvalidIndex {
                section,
                index: key,
            });
        };
        if index.to_string() != key {
            return Err(ConfigError::InvalidIndex {
                section,
                index: key,
            });
        }
        indexed.push((index, value));
    }
    indexed.sort_unstable_by_key(|(index, _)| *index);

    for (expected, (found, _)) in indexed.iter().enumerate() {
        if expected != *found {
            return Err(ConfigError::NonContiguousIndex {
                section,
                expected,
                found: *found,
            });
        }
    }

    Ok(indexed)
}

fn parse_recording_signal(index: usize, name: &str) -> Result<RecordingSignal, ConfigError> {
    match name {
        "pitch" => Ok(RecordingSignal::Pitch),
        "roll" => Ok(RecordingSignal::Roll),
        "elevator_position" => Ok(RecordingSignal::ElevatorPosition),
        "aileron_position" => Ok(RecordingSignal::AileronPosition),
        _ => Err(ConfigError::UnsupportedRecordingSignal {
            index,
            name: name.to_owned(),
        }),
    }
}

fn validate_column(
    index: usize,
    field: &'static str,
    column: &str,
    columns: &mut HashSet<String>,
) -> Result<(), ConfigError> {
    if column.is_empty() {
        return Err(ConfigError::EmptyInjectionColumn { index, field });
    }
    if !columns.insert(column.to_owned()) {
        return Err(ConfigError::ReusedInjectionColumn {
            index,
            column: column.to_owned(),
        });
    }
    Ok(())
}

fn validate_increasing_range(
    index: usize,
    field: &'static str,
    range: [f64; 2],
) -> Result<(), ConfigError> {
    if !range.iter().all(|endpoint| endpoint.is_finite()) {
        return Err(ConfigError::InvalidInjectionRange {
            index,
            field,
            reason: "both endpoints must be finite",
        });
    }
    if range[0] >= range[1] {
        return Err(ConfigError::InvalidInjectionRange {
            index,
            field,
            reason: "lower endpoint must be less than upper endpoint",
        });
    }
    Ok(())
}

const fn recording_catalog(signal: RecordingSignal) -> (RecordingUnit, [f64; 2]) {
    match signal {
        RecordingSignal::Pitch | RecordingSignal::Roll => (RecordingUnit::Degrees, [-180.0, 180.0]),
        RecordingSignal::ElevatorPosition | RecordingSignal::AileronPosition => {
            (RecordingUnit::Position16k, [-16_384.0, 16_384.0])
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const VALID_CONFIG: &str = r#"
format_version = 1
aircraft_target = "flybywire-a32nx"
input_file = "scenario.csv"

[inject.0]
name = "sidestick_pitch_position"
time_column = "sidestick_pitch_position.time"
value_column = "sidestick_pitch_position.value"
source_unit = "percent"
source_range = [-100.0, 100.0]
simulator_unit = "normalized"
simulator_range = [-1.0, 1.0]

[inject.1]
name = "sidestick_roll_position"
time_column = "sidestick_roll_position.time"
value_column = "sidestick_roll_position.value"
source_unit = "percent"
source_range = [-100.0, 100.0]
simulator_unit = "normalized"
simulator_range = [-1.0, 1.0]

[record.0]
name = "pitch"
unit = "degrees"
range = [-180.0, 180.0]

[record.1]
name = "roll"
unit = "degrees"
range = [-180.0, 180.0]

[record.2]
name = "elevator_position"
unit = "position_16k"
range = [-16384.0, 16384.0]

[record.3]
name = "aileron_position"
unit = "position_16k"
range = [-16384.0, 16384.0]
"#;

    fn assert_error(config: &str, predicate: impl FnOnce(&ConfigError) -> bool) {
        match parse_config(config) {
            Ok(parsed) => panic!("configuration unexpectedly parsed: {parsed:?}"),
            Err(error) => assert!(predicate(&error), "unexpected error: {error}"),
        }
    }

    #[test]
    fn parses_default_configuration_file() {
        match parse_config(include_str!("../replayer_config.toml")) {
            Ok(config) => {
                assert_eq!(config.inject.len(), 2);
                assert_eq!(config.record.len(), 4);
            }
            Err(error) => panic!("default configuration should parse: {error}"),
        }
    }

    #[test]
    fn parses_readme_configuration() {
        let config = match parse_config(VALID_CONFIG) {
            Ok(config) => config,
            Err(error) => panic!("README configuration should parse: {error}"),
        };

        assert_eq!(config.format_version, FORMAT_VERSION);
        assert_eq!(config.aircraft_target, AIRCRAFT_TARGET);
        assert_eq!(config.input_file, PathBuf::from("scenario.csv"));
        assert_eq!(config.inject.len(), 2);
        assert_eq!(config.inject[0].name, "sidestick_pitch_position");
        assert_eq!(config.inject[1].name, "sidestick_roll_position");
        assert_eq!(config.record.len(), 4);
        assert_eq!(config.record[0].name, RecordingSignal::Pitch);
        assert_eq!(config.record[3].name, RecordingSignal::AileronPosition);
    }

    #[test]
    fn reads_configuration_file() {
        let path =
            std::env::temp_dir().join(format!("replay-valid-config-{}.toml", std::process::id()));
        if let Err(error) = std::fs::write(&path, VALID_CONFIG) {
            panic!("failed to create test configuration: {error}");
        }

        let result = read_config_file(&path);
        let _ = std::fs::remove_file(&path);

        match result {
            Ok(config) => assert_eq!(config.inject.len(), 2),
            Err(error) => panic!("configuration file should load: {error}"),
        }
    }

    #[test]
    fn reports_configuration_file_read_path() {
        let path =
            std::env::temp_dir().join(format!("replay-missing-config-{}.toml", std::process::id()));
        let _ = std::fs::remove_file(&path);

        match read_config_file(&path) {
            Err(error) => assert!(
                error.to_string().contains(&path.display().to_string()),
                "read error should contain path: {error:#}"
            ),
            Ok(config) => panic!("missing configuration unexpectedly loaded: {config:?}"),
        }
    }

    #[test]
    fn rejects_unsupported_version_and_target() {
        assert_error(
            &VALID_CONFIG.replacen("format_version = 1", "format_version = 2", 1),
            |error| matches!(error, ConfigError::UnsupportedFormatVersion { .. }),
        );
        assert_error(
            &VALID_CONFIG.replacen("flybywire-a32nx", "other-aircraft", 1),
            |error| matches!(error, ConfigError::UnsupportedAircraftTarget { .. }),
        );
    }

    #[test]
    fn accepts_arbitrary_injection_names_and_rejects_unsupported_recordings() {
        let arbitrary = VALID_CONFIG.replacen("sidestick_pitch_position\"", "custom_input\"", 1);
        match parse_config(&arbitrary) {
            Ok(config) => assert_eq!(config.inject[0].name, "custom_input"),
            Err(error) => panic!("arbitrary injection name should parse: {error}"),
        }

        assert_error(
            &VALID_CONFIG.replacen("name = \"pitch\"", "name = \"yaw\"", 1),
            |error| matches!(error, ConfigError::UnsupportedRecordingSignal { .. }),
        );
    }

    #[test]
    fn rejects_unsupported_units() {
        assert_error(
            &VALID_CONFIG.replacen("source_unit = \"percent\"", "source_unit = \"radians\"", 1),
            |error| matches!(error, ConfigError::UnsupportedSourceUnit { .. }),
        );
        assert_error(
            &VALID_CONFIG.replacen(
                "simulator_unit = \"normalized\"",
                "simulator_unit = \"percent\"",
                1,
            ),
            |error| matches!(error, ConfigError::UnsupportedSimulatorUnit { .. }),
        );
        assert_error(
            &VALID_CONFIG.replacen("unit = \"degrees\"", "unit = \"radians\"", 1),
            |error| matches!(error, ConfigError::UnsupportedRecordingUnit { .. }),
        );
    }

    #[test]
    fn rejects_non_numeric_and_non_contiguous_indexes() {
        assert_error(
            &VALID_CONFIG.replacen("[inject.0]", "[inject.first]", 1),
            |error| {
                matches!(
                    error,
                    ConfigError::InvalidIndex {
                        section: "inject",
                        ..
                    }
                )
            },
        );
        assert_error(
            &VALID_CONFIG.replacen("[record.1]", "[record.4]", 1),
            |error| {
                matches!(
                    error,
                    ConfigError::NonContiguousIndex {
                        section: "record",
                        ..
                    }
                )
            },
        );
    }

    #[test]
    fn rejects_duplicate_signals_and_columns() {
        assert_error(
            &VALID_CONFIG.replacen(
                "name = \"sidestick_roll_position\"",
                "name = \"sidestick_pitch_position\"",
                1,
            ),
            |error| matches!(error, ConfigError::DuplicateInjectionSignal { .. }),
        );
        assert_error(
            &VALID_CONFIG.replacen("name = \"roll\"", "name = \"pitch\"", 1),
            |error| matches!(error, ConfigError::DuplicateRecordingSignal { .. }),
        );
        assert_error(
            &VALID_CONFIG.replacen(
                "value_column = \"sidestick_pitch_position.value\"",
                "value_column = \"sidestick_pitch_position.time\"",
                1,
            ),
            |error| matches!(error, ConfigError::ReusedInjectionColumn { .. }),
        );
        assert_error(
            &VALID_CONFIG.replacen(
                "time_column = \"sidestick_roll_position.time\"",
                "time_column = \"sidestick_pitch_position.value\"",
                1,
            ),
            |error| matches!(error, ConfigError::ReusedInjectionColumn { .. }),
        );
    }

    #[test]
    fn rejects_invalid_injection_ranges() {
        for replacement in [
            "source_range = [nan, 100.0]",
            "source_range = [100.0, -100.0]",
            "source_range = [1.0, 1.0]",
        ] {
            assert_error(
                &VALID_CONFIG.replacen("source_range = [-100.0, 100.0]", replacement, 1),
                |error| matches!(error, ConfigError::InvalidInjectionRange { .. }),
            );
        }
        assert_error(
            &VALID_CONFIG.replacen(
                "simulator_range = [-1.0, 1.0]",
                "simulator_range = [-1.1, 1.0]",
                1,
            ),
            |error| matches!(error, ConfigError::UnsafeSimulatorRange { .. }),
        );
    }

    #[test]
    fn rejects_invalid_recording_ranges() {
        for replacement in [
            "range = [nan, 180.0]",
            "range = [180.0, -180.0]",
            "range = [-90.0, 90.0]",
        ] {
            assert_error(
                &VALID_CONFIG.replacen("range = [-180.0, 180.0]", replacement, 1),
                |error| matches!(error, ConfigError::InvalidRecordingRange { .. }),
            );
        }
    }

    #[test]
    fn rejects_unknown_fields() {
        assert_error(
            &VALID_CONFIG.replacen(
                "input_file = \"scenario.csv\"",
                "input_file = \"scenario.csv\"\nunexpected = true",
                1,
            ),
            |error| matches!(error, ConfigError::Toml(_)),
        );
        assert_error(
            &VALID_CONFIG.replacen(
                "source_unit = \"percent\"",
                "source_unit = \"percent\"\ninterpolation = \"linear\"",
                1,
            ),
            |error| matches!(error, ConfigError::Toml(_)),
        );
    }

    #[test]
    fn preserves_numeric_section_order() {
        let reordered = VALID_CONFIG
            .replace("[inject.0]", "[inject.9]")
            .replace("[inject.1]", "[inject.0]")
            .replace("[inject.9]", "[inject.1]")
            .replace("[record.0]", "[record.9]")
            .replace("[record.1]", "[record.0]")
            .replace("[record.9]", "[record.1]");

        let config = match parse_config(&reordered) {
            Ok(config) => config,
            Err(error) => panic!("reordered configuration should parse: {error}"),
        };
        assert_eq!(
            config
                .inject
                .iter()
                .map(|item| item.name.as_str())
                .collect::<Vec<_>>(),
            vec!["sidestick_roll_position", "sidestick_pitch_position"]
        );
        assert_eq!(config.record[0].name, RecordingSignal::Roll);
        assert_eq!(config.record[1].name, RecordingSignal::Pitch);
    }
}
