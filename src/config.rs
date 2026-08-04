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

/// Configuration path in the package-specific writable MSFS work mount.
pub const CONFIG_PATH: &str = "/work/replayer_config.toml";

/// Validated replay configuration in deterministic processing order.
#[derive(Debug, Clone, PartialEq)]
pub struct ReplayConfig {
    /// Scenario CSV path exactly as specified by `input_file`.
    pub input_file: PathBuf,
    /// Injection definitions ordered by their numeric `inject.N` indexes.
    pub inject: Vec<InjectionConfig>,
    /// Recording definitions ordered by their numeric `record.N` indexes.
    pub record: Vec<RecordingConfig>,
}

impl ReplayConfig {
    /// Creates a replay configuration from TOML text.
    pub fn new(contents: &str) -> Result<ReplayConfig, ConfigError> {
        let raw = toml::from_str(contents)?;
        Self::parse_raw(raw)
    }

    /// Reads and parses a replay configuration file.
    pub fn read_config_file(path: impl AsRef<Path>) -> Result<ReplayConfig, ConfigError> {
        let contents = fs::read_to_string(path)?;
        Self::new(&contents)
    }

    fn parse_raw(raw: RawReplayConfig) -> Result<ReplayConfig, ConfigError> {
        if raw.format_version != FORMAT_VERSION {
            return Err(ConfigError::UnsupportedFormatVersion {
                found: raw.format_version,
                expected: FORMAT_VERSION,
            });
        }

        let inject = Self::parse_injections(raw.inject)?;
        let record = Self::parse_recordings(raw.record)?;

        Ok(ReplayConfig {
            input_file: PathBuf::from(raw.input_file),
            inject,
            record,
        })
    }

    fn parse_injections(
        entries: BTreeMap<String, RawInjectionConfig>,
    ) -> Result<Vec<InjectionConfig>, ConfigError> {
        let entries = Self::ordered_entries("inject", entries)?;
        let mut signals = HashSet::with_capacity(entries.len());
        let mut result = Vec::with_capacity(entries.len());

        for (index, raw) in entries {
            result.push(InjectionConfig::new(index, raw, &mut signals)?);
        }

        Ok(result)
    }

    fn parse_recordings(
        entries: BTreeMap<String, RawRecordingConfig>,
    ) -> Result<Vec<RecordingConfig>, ConfigError> {
        let entries = Self::ordered_entries("record", entries)?;
        let mut signals = HashSet::with_capacity(entries.len());
        let mut result = Vec::with_capacity(entries.len());

        for (index, raw) in entries {
            result.push(RecordingConfig::new(index, raw, &mut signals)?);
        }

        Ok(result)
    }

    /// Converts an indexed TOML section into deterministic numeric order.
    ///
    /// Section keys must be canonical non-negative integers and must form a
    /// contiguous sequence starting at zero. This keeps processing order stable
    /// and rejects missing or malformed `inject.N`/`record.N` entries.
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

        let highest_index = indexed.last().map_or(0, |(index, _)| *index);
        let required_length = highest_index.saturating_add(1);
        if indexed.len() != required_length {
            return Err(ConfigError::NonContiguousIndex {
                section,
                expected: indexed.len(),
                found: required_length,
            });
        }

        Ok(indexed)
    }
}

/// Configuration for one continuous scenario input.
#[derive(Debug, Clone, PartialEq)]
pub struct InjectionConfig {
    /// Logical input signal name from the configuration.
    pub name: String,
    /// Prefixed simulator destination, such as `K:AXIS_ELEVATOR_SET`.
    pub variable: String,
    /// Inclusive, strictly increasing valid range for source values.
    pub source_range: [f64; 2],
    /// Inclusive affine-conversion target range within the signal's safe range.
    pub simulator_range: [f64; 2],
}

impl InjectionConfig {
    fn new(
        index: usize,
        raw: RawInjectionConfig,
        signals: &mut HashSet<String>,
    ) -> Result<InjectionConfig, ConfigError> {
        if raw.name.is_empty() {
            return Err(ConfigError::EmptyInjectionName { index });
        }
        if !signals.insert(raw.name.clone()) {
            return Err(ConfigError::DuplicateInjectionSignal {
                index,
                name: raw.name,
            });
        }

        Self::validate_increasing_range(index, "source_range", raw.source_range)?;
        Self::validate_increasing_range(index, "simulator_range", raw.simulator_range)?;
        if raw.simulator_range[0] < -16_383.0 || raw.simulator_range[1] > 16_384.0 {
            return Err(ConfigError::UnsafeSimulatorRange { index });
        }

        Ok(InjectionConfig {
            name: raw.name,
            variable: raw.variable,
            source_range: raw.source_range,
            simulator_range: raw.simulator_range,
        })
    }

    /// Validates that a configured range has finite, strictly increasing endpoints.
    ///
    /// The field name and injection index are retained in any returned error so
    /// invalid source and simulator ranges can be distinguished.
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
}

/// Configuration for one aircraft-response signal recorded each frame.
#[derive(Debug, Clone, PartialEq)]
pub struct RecordingConfig {
    /// Logical telemetry column name from the configuration.
    pub name: String,
    /// Prefixed simulator source, such as `A:PLANE PITCH DEGREES`.
    pub variable: String,
    /// MSFS read unit required for `A:` variables and absent for other prefixes.
    pub unit: Option<String>,
    /// Optional maximum sampling frequency in Hz.
    pub max_sampling_rate: Option<f64>,
}

impl RecordingConfig {
    fn new(
        index: usize,
        raw: RawRecordingConfig,
        signals: &mut HashSet<String>,
    ) -> Result<RecordingConfig, ConfigError> {
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
        if raw.variable.starts_with("A:") {
            match raw.unit.as_deref() {
                None => return Err(ConfigError::MissingRecordingUnit { index }),
                Some("") => return Err(ConfigError::EmptyRecordingUnit { index }),
                Some(_) => {}
            }
        } else if raw.unit.is_some() {
            return Err(ConfigError::UnexpectedRecordingUnit {
                index,
                variable: raw.variable,
            });
        }
        Ok(RecordingConfig {
            name: raw.name,
            variable: raw.variable,
            unit: raw.unit,
            max_sampling_rate: match raw.max_sampling_rate {
                Some(rate) => Some(Self::validate_sampling_rate(index, rate)?),
                None => None,
            },
        })
    }

    fn validate_sampling_rate(index: usize, rate: f64) -> Result<f64, ConfigError> {
        if !rate.is_finite() {
            return Err(ConfigError::InvalidRecordingSamplingRate {
                index,
                value: rate,
                reason: "must be finite",
            });
        }
        if rate <= 0.0 {
            return Err(ConfigError::InvalidRecordingSamplingRate {
                index,
                value: rate,
                reason: "must be greater than 0",
            });
        }
        Ok(rate)
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawReplayConfig {
    format_version: u32,
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
    source_range: [f64; 2],
    simulator_range: [f64; 2],
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawRecordingConfig {
    name: String,
    variable: String,
    unit: Option<String>,
    max_sampling_rate: Option<f64>,
}

#[cfg(test)]
mod tests {
    use super::*;

    const VALID_CONFIG: &str = r#"
format_version = 1
input_file = "scenario.csv"

[inject.0]
name = "sidestick_pitch_position"
variable = "K:AXIS_ELEVATOR_SET"
source_range = [-100.0, 100.0]
simulator_range = [-1.0, 1.0]

[inject.1]
name = "sidestick_roll_position"
variable = "K:AXIS_AILERONS_SET"
source_range = [-100.0, 100.0]
simulator_range = [-1.0, 1.0]

[record.0]
name = "pitch"
variable = "A:PLANE PITCH DEGREES"
unit = "radians"

[record.1]
name = "roll"
variable = "A:PLANE BANK DEGREES"
unit = "radians"

[record.2]
name = "elevator_position"
variable = "A:ELEVATOR POSITION"
unit = "position"

[record.3]
name = "aileron_position"
variable = "A:AILERON POSITION"
unit = "position"
"#;

    fn assert_error(config: &str, predicate: impl FnOnce(&ConfigError) -> bool) {
        match ReplayConfig::new(config) {
            Ok(parsed) => panic!("configuration unexpectedly parsed: {parsed:?}"),
            Err(error) => assert!(predicate(&error), "unexpected error: {error}"),
        }
    }

    #[test]
    fn parses_default_configuration_file() {
        match ReplayConfig::new(include_str!("../example/replayer_config.toml")) {
            Ok(config) => {
                assert_eq!(config.inject.len(), 2);
                assert_eq!(config.record.len(), 4);
            }
            Err(error) => panic!("default configuration should parse: {error}"),
        }
    }

    #[test]
    fn parses_readme_configuration() {
        let config = match ReplayConfig::new(VALID_CONFIG) {
            Ok(config) => config,
            Err(error) => panic!("README configuration should parse: {error}"),
        };

        assert_eq!(config.input_file, PathBuf::from("scenario.csv"));
        assert_eq!(config.inject.len(), 2);
        assert_eq!(config.inject[0].name, "sidestick_pitch_position");
        assert_eq!(config.inject[0].variable, "K:AXIS_ELEVATOR_SET");
        assert_eq!(config.inject[1].name, "sidestick_roll_position");
        assert_eq!(config.inject[1].variable, "K:AXIS_AILERONS_SET");
        assert_eq!(config.record.len(), 4);
        assert_eq!(config.record[0].name, "pitch");
        assert_eq!(config.record[0].variable, "A:PLANE PITCH DEGREES");
        assert_eq!(config.record[0].unit.as_deref(), Some("radians"));
        assert_eq!(config.record[3].name, "aileron_position");
        assert_eq!(config.record[3].variable, "A:AILERON POSITION");
        assert_eq!(config.record[3].unit.as_deref(), Some("position"));
    }

    #[test]
    fn reads_configuration_file() {
        let path =
            std::env::temp_dir().join(format!("replay-valid-config-{}.toml", std::process::id()));
        if let Err(error) = std::fs::write(&path, VALID_CONFIG) {
            panic!("failed to create test configuration: {error}");
        }

        let result = ReplayConfig::read_config_file(&path);
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
            ReplayConfig::read_config_file(&path),
            Err(ConfigError::FileIo(_))
        ));
    }

    #[test]
    fn rejects_unsupported_version() {
        assert_error(
            &VALID_CONFIG.replacen("format_version = 1", "format_version = 2", 1),
            |error| matches!(error, ConfigError::UnsupportedFormatVersion { .. }),
        );
    }

    #[test]
    fn accepts_arbitrary_injection_names_and_variables() {
        let arbitrary = VALID_CONFIG
            .replacen("sidestick_pitch_position\"", "custom_input\"", 1)
            .replacen("K:AXIS_ELEVATOR_SET", "L:CUSTOM_INPUT", 1);
        match ReplayConfig::new(&arbitrary) {
            Ok(config) => {
                assert_eq!(config.inject[0].name, "custom_input");
                assert_eq!(config.inject[0].variable, "L:CUSTOM_INPUT");
            }
            Err(error) => panic!("arbitrary injection should parse: {error}"),
        }
    }

    #[test]
    fn requires_non_empty_injection_name() {
        assert_error(
            &VALID_CONFIG.replacen("name = \"sidestick_pitch_position\"", "name = \"\"", 1),
            |error| matches!(error, ConfigError::EmptyInjectionName { .. }),
        );
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
            .replacen("A:PLANE PITCH DEGREES", "L:CUSTOM_RESPONSE", 1)
            .replacen("unit = \"radians\"\n", "", 1);
        match ReplayConfig::new(&arbitrary) {
            Ok(config) => {
                assert_eq!(config.record[0].name, "custom_response");
                assert_eq!(config.record[0].variable, "L:CUSTOM_RESPONSE");
                assert_eq!(config.record[0].unit, None);
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
    fn validates_recording_units() {
        assert_error(
            &VALID_CONFIG.replacen("unit = \"radians\"\n", "", 1),
            |error| matches!(error, ConfigError::MissingRecordingUnit { .. }),
        );
        assert_error(
            &VALID_CONFIG.replacen("unit = \"radians\"", "unit = \"\"", 1),
            |error| matches!(error, ConfigError::EmptyRecordingUnit { .. }),
        );
        assert_error(
            &VALID_CONFIG.replacen("A:PLANE PITCH DEGREES", "L:CUSTOM_RESPONSE", 1),
            |error| matches!(error, ConfigError::UnexpectedRecordingUnit { .. }),
        );
    }

    #[test]
    fn accepts_optional_recording_sampling_rate() {
        let config = ReplayConfig::new(&VALID_CONFIG.replacen(
            "unit = \"radians\"\n",
            "unit = \"radians\"\nmax_sampling_rate = 1.0\n",
            1,
        ))
        .unwrap_or_else(|error| panic!("valid sampling-rate config rejected: {error}"));
        assert_eq!(config.record[0].max_sampling_rate, Some(1.0));
        assert_eq!(config.record[1].max_sampling_rate, None);
    }

    #[test]
    fn rejects_invalid_recording_sampling_rate() {
        for value in ["0", "-1", "nan", "inf", "-inf"] {
            assert_error(
                &VALID_CONFIG.replacen(
                    "unit = \"radians\"\n",
                    &format!("unit = \"radians\"\nmax_sampling_rate = {value}\n"),
                    1,
                ),
                |error| matches!(error, ConfigError::InvalidRecordingSamplingRate { .. }),
            );
        }
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
    fn rejects_duplicate_signals() {
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
        for unknown_field in [
            "time_column = \"custom.time\"",
            "value_column = \"custom.value\"",
            "interpolation = \"linear\"",
        ] {
            assert_error(
                &VALID_CONFIG.replacen(
                    "source_range = [-100.0, 100.0]",
                    &format!("source_range = [-100.0, 100.0]\n{unknown_field}"),
                    1,
                ),
                |error| matches!(error, ConfigError::Toml(_)),
            );
        }
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

        let config = match ReplayConfig::new(&reordered) {
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
