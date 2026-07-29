use std::collections::{BTreeMap, HashSet};
use std::path::{Component, Path, PathBuf};

use serde::Deserialize;
use thiserror::Error;

pub const FORMAT_VERSION: u32 = 1;
pub const AIRCRAFT_TARGET: &str = "flybywire-a32nx";
pub const CONFIG_PATH: &str = "/work/replay/config.toml";

#[derive(Debug, Clone, PartialEq)]
pub struct ReplayConfig {
    pub format_version: u32,
    pub aircraft_target: String,
    pub input_file: PathBuf,
    pub inject: Vec<InjectionConfig>,
    pub record: Vec<RecordingConfig>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct InjectionConfig {
    pub name: InjectionSignal,
    pub time_column: String,
    pub value_column: String,
    pub source_unit: SourceUnit,
    pub source_range: [f64; 2],
    pub simulator_unit: SimulatorUnit,
    pub simulator_range: [f64; 2],
}

#[derive(Debug, Clone, PartialEq)]
pub struct RecordingConfig {
    pub name: RecordingSignal,
    pub unit: RecordingUnit,
    pub range: [f64; 2],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InjectionSignal {
    SidestickPitchPosition,
    SidestickRollPosition,
}

impl InjectionSignal {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::SidestickPitchPosition => "sidestick_pitch_position",
            Self::SidestickRollPosition => "sidestick_roll_position",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RecordingSignal {
    Pitch,
    Roll,
    ElevatorPosition,
    AileronPosition,
}

impl RecordingSignal {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Pitch => "pitch",
            Self::Roll => "roll",
            Self::ElevatorPosition => "elevator_position",
            Self::AileronPosition => "aileron_position",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceUnit {
    Percent,
    Normalized,
}

impl SourceUnit {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Percent => "percent",
            Self::Normalized => "normalized",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimulatorUnit {
    Normalized,
}

impl SimulatorUnit {
    pub const fn as_str(self) -> &'static str {
        "normalized"
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecordingUnit {
    Degrees,
    Position16k,
}

impl RecordingUnit {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Degrees => "degrees",
            Self::Position16k => "position_16k",
        }
    }
}

#[derive(Debug, Error)]
pub enum ConfigError {
    #[error("invalid TOML configuration: {0}")]
    Toml(#[from] toml::de::Error),

    #[error("unsupported format_version {found}; expected {expected}")]
    UnsupportedFormatVersion { found: u32, expected: u32 },

    #[error("unsupported aircraft_target `{found}`; expected `{expected}`")]
    UnsupportedAircraftTarget {
        found: String,
        expected: &'static str,
    },

    #[error("invalid input_file `{path}`: {reason}")]
    InvalidInputFile { path: String, reason: &'static str },

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

    #[error("unsupported injection signal `{name}` at inject.{index}")]
    UnsupportedInjectionSignal { index: usize, name: String },

    #[error("unsupported recording signal `{name}` at record.{index}")]
    UnsupportedRecordingSignal { index: usize, name: String },

    #[error(
        "unsupported source unit `{unit}` at inject.{index}; expected `percent` or `normalized`"
    )]
    UnsupportedSourceUnit { index: usize, unit: String },

    #[error("unsupported simulator unit `{unit}` at inject.{index}; expected `normalized`")]
    UnsupportedSimulatorUnit { index: usize, unit: String },

    #[error("invalid {field} at inject.{index}: {reason}")]
    InvalidInjectionRange {
        index: usize,
        field: &'static str,
        reason: &'static str,
    },

    #[error("simulator_range at inject.{index} must remain within [-1, 1]")]
    UnsafeSimulatorRange { index: usize },

    #[error("duplicate injection signal `{name}` at inject.{index}")]
    DuplicateInjectionSignal { index: usize, name: String },

    #[error("column `{column}` is reused at inject.{index}; time and value columns must be unique")]
    ReusedInjectionColumn { index: usize, column: String },

    #[error("{field} at inject.{index} must not be empty")]
    EmptyInjectionColumn { index: usize, field: &'static str },

    #[error(
        "unsupported recording unit `{unit}` at record.{index} for `{signal}`; expected `{expected}`"
    )]
    UnsupportedRecordingUnit {
        index: usize,
        signal: &'static str,
        unit: String,
        expected: &'static str,
    },

    #[error(
        "invalid range at record.{index} for `{signal}`; expected [{expected_min}, {expected_max}]"
    )]
    InvalidRecordingRange {
        index: usize,
        signal: &'static str,
        expected_min: f64,
        expected_max: f64,
    },

    #[error("duplicate recording signal `{name}` at record.{index}")]
    DuplicateRecordingSignal { index: usize, name: String },
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

    validate_input_file(&raw.input_file)?;
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
    pub fn parse(contents: &str) -> Result<Self, ConfigError> {
        parse_config(contents)
    }
}

fn validate_input_file(input_file: &str) -> Result<(), ConfigError> {
    let invalid = |reason| ConfigError::InvalidInputFile {
        path: input_file.to_owned(),
        reason,
    };

    if input_file.trim().is_empty() {
        return Err(invalid("path must not be empty"));
    }

    let path = Path::new(input_file);
    if path.is_absolute() {
        return Err(invalid("path must be relative"));
    }

    let mut has_normal_component = false;
    for component in path.components() {
        match component {
            Component::Normal(_) => has_normal_component = true,
            Component::ParentDir => {
                return Err(invalid("parent components are not allowed"));
            }
            Component::RootDir => return Err(invalid("root components are not allowed")),
            Component::Prefix(_) => return Err(invalid("path prefixes are not allowed")),
            Component::CurDir => {
                return Err(invalid("current-directory components are not allowed"));
            }
        }
    }

    if !has_normal_component {
        return Err(invalid("path must contain a file name"));
    }

    Ok(())
}

fn parse_injections(
    entries: BTreeMap<String, RawInjectionConfig>,
) -> Result<Vec<InjectionConfig>, ConfigError> {
    let entries = ordered_entries("inject", entries)?;
    let mut signals = HashSet::with_capacity(entries.len());
    let mut columns = HashSet::with_capacity(entries.len().saturating_mul(2));
    let mut result = Vec::with_capacity(entries.len());

    for (index, raw) in entries {
        let name = parse_injection_signal(index, &raw.name)?;
        if !signals.insert(name) {
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
            name,
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

fn parse_injection_signal(index: usize, name: &str) -> Result<InjectionSignal, ConfigError> {
    match name {
        "sidestick_pitch_position" => Ok(InjectionSignal::SidestickPitchPosition),
        "sidestick_roll_position" => Ok(InjectionSignal::SidestickRollPosition),
        _ => Err(ConfigError::UnsupportedInjectionSignal {
            index,
            name: name.to_owned(),
        }),
    }
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
    fn parses_readme_configuration() {
        let config = match parse_config(VALID_CONFIG) {
            Ok(config) => config,
            Err(error) => panic!("README configuration should parse: {error}"),
        };

        assert_eq!(config.format_version, FORMAT_VERSION);
        assert_eq!(config.aircraft_target, AIRCRAFT_TARGET);
        assert_eq!(config.input_file, PathBuf::from("scenario.csv"));
        assert_eq!(config.inject.len(), 2);
        assert_eq!(
            config.inject[0].name,
            InjectionSignal::SidestickPitchPosition
        );
        assert_eq!(
            config.inject[1].name,
            InjectionSignal::SidestickRollPosition
        );
        assert_eq!(config.record.len(), 4);
        assert_eq!(config.record[0].name, RecordingSignal::Pitch);
        assert_eq!(config.record[3].name, RecordingSignal::AileronPosition);
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
    fn rejects_unsupported_signals() {
        assert_error(
            &VALID_CONFIG.replacen("sidestick_pitch_position\"", "throttle\"", 1),
            |error| matches!(error, ConfigError::UnsupportedInjectionSignal { .. }),
        );
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
    fn rejects_unsafe_input_paths() {
        for path in ["", "../scenario.csv", "/scenario.csv", "C:\\\\scenario.csv"] {
            let config = VALID_CONFIG.replacen(
                "input_file = \"scenario.csv\"",
                &format!("input_file = \"{path}\""),
                1,
            );
            assert_error(&config, |error| {
                matches!(
                    error,
                    ConfigError::InvalidInputFile { .. } | ConfigError::Toml(_)
                )
            });
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
                .map(|item| item.name)
                .collect::<Vec<_>>(),
            vec![
                InjectionSignal::SidestickRollPosition,
                InjectionSignal::SidestickPitchPosition,
            ]
        );
        assert_eq!(config.record[0].name, RecordingSignal::Roll);
        assert_eq!(config.record[1].name, RecordingSignal::Pitch);
    }
}
