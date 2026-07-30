//! Replay configuration data types, parsing, and file loading.
//!
//! The parser accepts the versioned TOML contract documented in the crate
//! README and validates signal selections for the simulator-independent replay
//! core.

use std::collections::{BTreeMap, HashSet};
use std::fs;
use std::path::{Path, PathBuf};

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
    /// Prefixed simulator destination, such as `K:AXIS_ELEVATOR_SET`.
    pub variable: String,
    /// CSV column containing scenario-relative timestamps in seconds.
    pub time_column: String,
    /// CSV column containing source values.
    pub value_column: String,
    /// Inclusive, strictly increasing valid range for source values.
    pub source_range: [f64; 2],
    /// Inclusive affine-conversion target range within the signal's safe range.
    pub simulator_range: [f64; 2],
}

/// Configuration for one aircraft-response signal recorded each frame.
#[derive(Debug, Clone, PartialEq)]
pub struct RecordingConfig {
    /// Logical telemetry column name from the configuration.
    pub name: String,
    /// Prefixed simulator source, such as `A:PLANE PITCH DEGREES`.
    pub variable: String,
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
    variable: String,
    time_column: String,
    value_column: String,
    source_range: [f64; 2],
    simulator_range: [f64; 2],
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawRecordingConfig {
    name: String,
    variable: String,
}

/// Reads and parses a replay configuration file.
///
/// File-system and parse failures retain their typed source error.
pub fn read_config_file(path: impl AsRef<Path>) -> Result<ReplayConfig, ConfigError> {
    let contents = fs::read_to_string(path)?;
    parse_config(&contents)
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

        validate_increasing_range(index, "source_range", raw.source_range)?;
        validate_increasing_range(index, "simulator_range", raw.simulator_range)?;
        if raw.simulator_range[0] < -16_383.0 || raw.simulator_range[1] > 16_384.0 {
            return Err(ConfigError::UnsafeSimulatorRange { index });
        }

        result.push(InjectionConfig {
            name: raw.name,
            variable: raw.variable,
            time_column: raw.time_column,
            value_column: raw.value_column,
            source_range: raw.source_range,
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
        if raw.name.is_empty() {
            return Err(ConfigError::EmptyRecordingName { index });
        }
        if !signals.insert(raw.name.clone()) {
            return Err(ConfigError::DuplicateRecordingSignal {
                index,
                name: raw.name,
            });
        }

        if raw.variable.is_empty() {
            return Err(ConfigError::EmptyRecordingVariable { index });
        }

        result.push(RecordingConfig {
            name: raw.name,
            variable: raw.variable,
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

#[cfg(test)]
mod tests {
    use super::*;

    const VALID_CONFIG: &str = r#"
format_version = 1
aircraft_target = "flybywire-a32nx"
input_file = "scenario.csv"

[inject.0]
name = "sidestick_pitch_position"
variable = "K:AXIS_ELEVATOR_SET"
time_column = "sidestick_pitch_position.time"
value_column = "sidestick_pitch_position.value"
source_range = [-100.0, 100.0]
simulator_range = [-1.0, 1.0]

[inject.1]
name = "sidestick_roll_position"
variable = "K:AXIS_AILERONS_SET"
time_column = "sidestick_roll_position.time"
value_column = "sidestick_roll_position.value"
source_range = [-100.0, 100.0]
simulator_range = [-1.0, 1.0]

[record.0]
name = "pitch"
variable = "A:PLANE PITCH DEGREES"

[record.1]
name = "roll"
variable = "A:PLANE BANK DEGREES"

[record.2]
name = "elevator_position"
variable = "A:ELEVATOR POSITION"

[record.3]
name = "aileron_position"
variable = "A:AILERON POSITION"
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
        assert_eq!(config.inject[0].variable, "K:AXIS_ELEVATOR_SET");
        assert_eq!(config.inject[1].name, "sidestick_roll_position");
        assert_eq!(config.inject[1].variable, "K:AXIS_AILERONS_SET");
        assert_eq!(config.record.len(), 4);
        assert_eq!(config.record[0].name, "pitch");
        assert_eq!(config.record[0].variable, "A:PLANE PITCH DEGREES");
        assert_eq!(config.record[3].name, "aileron_position");
        assert_eq!(config.record[3].variable, "A:AILERON POSITION");
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
    fn reports_configuration_file_read_error() {
        let path =
            std::env::temp_dir().join(format!("replay-missing-config-{}.toml", std::process::id()));
        let _ = std::fs::remove_file(&path);

        assert!(matches!(
            read_config_file(&path),
            Err(ConfigError::FileIo(_))
        ));
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
    fn accepts_arbitrary_injection_names_and_variables() {
        let arbitrary = VALID_CONFIG
            .replacen("sidestick_pitch_position\"", "custom_input\"", 1)
            .replacen("K:AXIS_ELEVATOR_SET", "L:CUSTOM_INPUT", 1);
        match parse_config(&arbitrary) {
            Ok(config) => {
                assert_eq!(config.inject[0].name, "custom_input");
                assert_eq!(config.inject[0].variable, "L:CUSTOM_INPUT");
            }
            Err(error) => panic!("arbitrary injection should parse: {error}"),
        }
    }

    #[test]
    fn requires_injection_variable() {
        assert_error(
            &VALID_CONFIG.replacen("variable = \"K:AXIS_ELEVATOR_SET\"\n", "", 1),
            |error| matches!(error, ConfigError::Toml(_)),
        );
    }

    #[test]
    fn accepts_arbitrary_recording_names_and_variables() {
        let arbitrary = VALID_CONFIG
            .replacen("name = \"pitch\"", "name = \"custom_response\"", 1)
            .replacen("A:PLANE PITCH DEGREES", "L:CUSTOM_RESPONSE", 1);
        match parse_config(&arbitrary) {
            Ok(config) => {
                assert_eq!(config.record[0].name, "custom_response");
                assert_eq!(config.record[0].variable, "L:CUSTOM_RESPONSE");
            }
            Err(error) => panic!("arbitrary recording should parse: {error}"),
        }
    }

    #[test]
    fn requires_non_empty_recording_name() {
        assert_error(
            &VALID_CONFIG.replacen("name = \"pitch\"", "name = \"\"", 1),
            |error| matches!(error, ConfigError::EmptyRecordingName { .. }),
        );
    }

    #[test]
    fn requires_non_empty_recording_variable() {
        assert_error(
            &VALID_CONFIG.replacen("variable = \"A:PLANE PITCH DEGREES\"\n", "", 1),
            |error| matches!(error, ConfigError::Toml(_)),
        );
        assert_error(
            &VALID_CONFIG.replacen("variable = \"A:PLANE PITCH DEGREES\"", "variable = \"\"", 1),
            |error| matches!(error, ConfigError::EmptyRecordingVariable { .. }),
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
                "simulator_range = [-16384.0, 16384.0]",
                1,
            ),
            |error| matches!(error, ConfigError::UnsafeSimulatorRange { .. }),
        );
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
                "source_range = [-100.0, 100.0]",
                "source_range = [-100.0, 100.0]\ninterpolation = \"linear\"",
                1,
            ),
            |error| matches!(error, ConfigError::Toml(_)),
        );
        for removed_field in ["unit = \"degrees\"", "range = [-180.0, 180.0]"] {
            assert_error(
                &VALID_CONFIG.replacen(
                    "variable = \"A:PLANE PITCH DEGREES\"",
                    &format!("variable = \"A:PLANE PITCH DEGREES\"\n{removed_field}"),
                    1,
                ),
                |error| matches!(error, ConfigError::Toml(_)),
            );
        }
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
        assert_eq!(config.record[0].name, "roll");
        assert_eq!(config.record[1].name, "pitch");
    }
}
