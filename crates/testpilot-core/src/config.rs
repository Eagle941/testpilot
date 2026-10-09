//! Replay configuration data types, parsing, and file loading.
//!
//! The parser accepts the versioned TOML contract documented in the crate
//! README and validates signal selections for the simulator-independent replay
//! core.

use std::collections::{BTreeMap, HashSet};
use std::fs;
use std::path::{Path, PathBuf};

use serde::Deserialize;
use serde::de::Error as SerdeError;
use toml::Value;
use toml::value::Table;

pub use crate::error::{ConfigError, ConfigFileError};

/// Configuration and scenario format version supported by this crate.
pub const FORMAT_VERSION: u32 = 1;

/// Configuration path in the package-specific writable MSFS work mount.
pub const CONFIG_PATH: &str = "/work/replayer_config.toml";

/// Allowed top-level TOML fields in replay configuration.
const ROOT_FIELDS: [&str; 5] = [
    "format_version",
    "input_file",
    "initialisation",
    "inject",
    "record",
];
/// Allowed fields for each `[inject.N]` section.
const INJECT_SECTION_FIELDS: [&str; 4] = ["name", "variable", "source_range", "simulator_range"];
/// Allowed fields for each `[record.N]` section.
const RECORD_SECTION_FIELDS: [&str; 4] = ["name", "variable", "unit", "max_sampling_rate"];

/// Demanded actual aircraft mass, balance and optional THS, independent of flight-management entries.
#[derive(Debug, Clone, Copy, Default, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InitialisationConfig {
    /// Optional zero-fuel weight in kilograms; required with gw and gwcg.
    pub zfw: Option<f64>,
    /// Optional gross weight in kilograms; required with zfw and gwcg.
    pub gw: Option<f64>,
    /// Optional gross-weight centre of gravity in percent MAC; required with zfw and gw.
    pub gwcg: Option<f64>,
    /// Optional trimmable horizontal stabilizer demand in degrees, from -4 to 13.5.
    pub ths: Option<f64>,
}

impl InitialisationConfig {
    /// Whether any mass/balance target is configured; validation requires the complete group.
    pub const fn has_mass_balance(&self) -> bool {
        self.zfw.is_some() || self.gw.is_some() || self.gwcg.is_some()
    }

    /// Whether the section requests any initialisation work.
    pub const fn has_targets(&self) -> bool {
        self.has_mass_balance() || self.ths.is_some()
    }

    /// Checks finite targets and basic mass consistency; aircraft limits belong to the adapter.
    fn validate(&self) -> Result<(), ConfigError> {
        if self.has_mass_balance()
            && (self.zfw.is_none() || self.gw.is_none() || self.gwcg.is_none())
        {
            return Err(ConfigError::IncompleteInitialisationMassBalance);
        }
        for (field, value) in [("zfw", self.zfw), ("gw", self.gw), ("gwcg", self.gwcg)] {
            let Some(value) = value else {
                continue;
            };
            if !value.is_finite() {
                return Err(ConfigError::InvalidInitialisation {
                    field,
                    reason: "must be finite",
                });
            }
            if field != "gwcg" && value <= 0.0 {
                return Err(ConfigError::InvalidInitialisation {
                    field,
                    reason: "must be positive",
                });
            }
        }
        if let (Some(gw), Some(zfw)) = (self.gw, self.zfw)
            && gw < zfw
        {
            return Err(ConfigError::InvalidInitialisation {
                field: "gw",
                reason: "must be at least zfw",
            });
        }
        if let Some(ths) = self.ths
            && (!ths.is_finite() || !(-4.0..=13.5).contains(&ths))
        {
            return Err(ConfigError::InvalidInitialisation {
                field: "ths",
                reason: "must be finite and between -4 and 13.5 degrees inclusive",
            });
        }
        Ok(())
    }
}

/// Validated replay configuration in deterministic processing order.
#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    /// Optional demanded aircraft mass, balance and THS before playback.
    pub initialisation: Option<InitialisationConfig>,
    /// Scenario CSV path exactly as specified by `input_file`.
    pub input_file: PathBuf,
    /// Injection definitions ordered by their numeric `inject.N` indexes.
    pub inject: Vec<InjectionConfig>,
    /// Recording definitions ordered by their numeric `record.N` indexes; may be empty.
    pub record: Vec<RecordingConfig>,
}

impl Config {
    /// Creates a replay configuration from TOML text.
    pub fn new(contents: &str) -> Result<Config, ConfigError> {
        let value: Value = toml::from_str(contents)?;
        let root = value.as_table().ok_or_else(|| {
            ConfigError::Toml(toml::de::Error::custom(
                "configuration root must be a table",
            ))
        })?;
        Self::reject_unknown_fields("root", root, &ROOT_FIELDS)?;
        if let Some(section) = root.get("initialisation").and_then(Value::as_table) {
            Self::reject_unknown_fields("initialisation", section, &["zfw", "gw", "gwcg", "ths"])?;
        }

        let raw: RawReplayConfig = value.try_into().map_err(ConfigError::Toml)?;
        Self::parse_raw(raw)
    }

    /// Reads and parses a replay configuration file.
    pub fn read_config_file(path: impl AsRef<Path>) -> Result<Config, ConfigFileError> {
        let path = path.as_ref();
        let contents = fs::read_to_string(path).map_err(|source| ConfigFileError::Read {
            path: path.to_path_buf(),
            source,
        })?;
        Self::new(&contents).map_err(|source| ConfigFileError::Parse {
            path: path.to_path_buf(),
            source: Box::new(source),
        })
    }

    /// Builds a validated config from raw deserialized TOML data.
    fn parse_raw(raw: RawReplayConfig) -> Result<Config, ConfigError> {
        if raw.format_version != FORMAT_VERSION {
            return Err(ConfigError::UnsupportedFormatVersion {
                found: raw.format_version,
                expected: FORMAT_VERSION,
            });
        }

        let inject = Self::parse_injections(raw.inject)?;
        let record = Self::parse_recordings(raw.record)?;
        Self::validate_signal_names(&inject, &record)?;
        if let Some(initialisation) = &raw.initialisation {
            initialisation.validate()?;
        }

        Ok(Config {
            initialisation: raw.initialisation.filter(InitialisationConfig::has_targets),
            input_file: PathBuf::from(raw.input_file),
            inject,
            record,
        })
    }

    /// Parses and validates all injection entries.
    fn parse_injections(
        entries: BTreeMap<String, Value>,
    ) -> Result<Vec<InjectionConfig>, ConfigError> {
        let entries = Self::ordered_entries("inject", entries)?;
        let mut signals = HashSet::with_capacity(entries.len());
        let mut result = Vec::with_capacity(entries.len());

        for (index, raw) in entries {
            let section_name = format!("inject.{index}");
            let injection: InjectionConfig =
                Self::parse_indexed_section_entry(&section_name, raw, &INJECT_SECTION_FIELDS)?;
            result.push(injection.validate(index, &mut signals)?);
        }

        Ok(result)
    }

    /// Parses and validates recording entries, allowing an omitted or empty section.
    fn parse_recordings(
        entries: BTreeMap<String, Value>,
    ) -> Result<Vec<RecordingConfig>, ConfigError> {
        if entries.is_empty() {
            return Ok(Vec::new());
        }
        let entries = Self::ordered_entries("record", entries)?;
        let mut signals = HashSet::with_capacity(entries.len());
        let mut result = Vec::with_capacity(entries.len());

        for (index, raw) in entries {
            let section_name = format!("record.{index}");
            let recording: RecordingConfig =
                Self::parse_indexed_section_entry(&section_name, raw, &RECORD_SECTION_FIELDS)?;
            result.push(recording.validate(index, &mut signals)?);
        }

        Ok(result)
    }

    /// Rejects names that appear in both inject and record sections.
    fn validate_signal_names(
        inject: &[InjectionConfig],
        record: &[RecordingConfig],
    ) -> Result<(), ConfigError> {
        let injection_names = inject
            .iter()
            .map(|injection| injection.name.as_str())
            .collect::<HashSet<_>>();
        for recording in record {
            if injection_names.contains(recording.name.as_str()) {
                return Err(ConfigError::DuplicateSignalAcrossSections {
                    name: recording.name.clone(),
                });
            }
        }
        Ok(())
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

    /// Rejects fields not listed in the expected field set for a configuration section.
    fn reject_unknown_fields(
        section: &str,
        fields: &Table,
        expected: &[&str],
    ) -> Result<(), ConfigError> {
        for field in fields.keys() {
            if !expected.contains(&field.as_str()) {
                return Err(ConfigError::UnexpectedField {
                    section: section.to_owned(),
                    field: field.to_owned(),
                });
            }
        }
        Ok(())
    }

    /// Parses and validates one index-based configuration entry table.
    fn parse_indexed_section_entry<T>(
        section_name: &str,
        raw: Value,
        expected: &[&str],
    ) -> Result<T, ConfigError>
    where
        T: for<'de> serde::Deserialize<'de>,
    {
        let table = Self::as_table(section_name, raw)?;
        Self::reject_unknown_fields(section_name, &table, expected)?;
        table.try_into().map_err(ConfigError::Toml)
    }

    /// Converts a TOML value into a table used by section parsing.
    fn as_table(section: &str, value: Value) -> Result<Table, ConfigError> {
        match value {
            Value::Table(table) => Ok(table),
            _ => Err(ConfigError::Toml(toml::de::Error::custom(format!(
                "`{section}` must be a TOML table"
            )))),
        }
    }
}

/// Configuration for one continuous scenario input.
/// Deserialization parses fields; `Config::new` validates their semantics.
#[derive(Debug, Clone, PartialEq, Deserialize)]
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
    /// Validates a deserialized injection config before accepting it.
    fn validate(self, index: usize, signals: &mut HashSet<String>) -> Result<Self, ConfigError> {
        if self.name.is_empty() {
            return Err(ConfigError::EmptyInjectionName { index });
        }
        if !signals.insert(self.name.clone()) {
            return Err(ConfigError::DuplicateInjectionSignal {
                index,
                name: self.name,
            });
        }

        Self::validate_increasing_range(index, "source_range", self.source_range)?;
        Self::validate_increasing_range(index, "simulator_range", self.simulator_range)?;
        if self.simulator_range[0] < -16_383.0 || self.simulator_range[1] > 16_384.0 {
            return Err(ConfigError::UnsafeSimulatorRange { index });
        }

        Ok(self)
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
/// Deserialization parses fields; `Config::new` validates their semantics.
#[derive(Debug, Clone, PartialEq, Deserialize)]
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
    /// Validates a deserialized recording config before accepting it.
    fn validate(self, index: usize, signals: &mut HashSet<String>) -> Result<Self, ConfigError> {
        if self.name.is_empty() {
            return Err(ConfigError::EmptyRecordingName { index });
        }
        if !signals.insert(self.name.clone()) {
            return Err(ConfigError::DuplicateRecordingSignal {
                index,
                name: self.name,
            });
        }
        if self.variable.is_empty() {
            return Err(ConfigError::EmptyRecordingVariable { index });
        }
        if self.variable.starts_with("A:") {
            match self.unit.as_deref() {
                None => return Err(ConfigError::MissingRecordingUnit { index }),
                Some("") => return Err(ConfigError::EmptyRecordingUnit { index }),
                Some(_) => {}
            }
        } else if self.unit.is_some() {
            return Err(ConfigError::UnexpectedRecordingUnit {
                index,
                variable: self.variable,
            });
        }
        if let Some(rate) = self.max_sampling_rate {
            Self::validate_sampling_rate(index, rate)?;
        }
        Ok(self)
    }

    /// Validates an optional max sampling rate (must be finite and > 0).
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

#[derive(Debug, Clone, PartialEq, Deserialize)]
/// Internal raw configuration as deserialized from TOML.
struct RawReplayConfig {
    /// Optional aircraft initialisation targets.
    initialisation: Option<InitialisationConfig>,
    /// Declared format version.
    format_version: u32,
    /// Raw input filename provided in config.
    input_file: String,
    #[serde(default)]
    /// Raw injection map section.
    inject: BTreeMap<String, Value>,
    #[serde(default)]
    /// Raw recording map section.
    record: BTreeMap<String, Value>,
}

#[cfg(test)]
mod tests {
    #[test]
    fn validates_optional_ths_degrees() {
        let parse = |value: &str| {
            Config::new(&format!(
                "{VALID_CONFIG}\n[initialisation]\nzfw = 60000\ngw = 65000\ngwcg = 25\nths = {value}\n"
            ))
        };
        for value in [-4.0, 0.0, 4.75, 13.5] {
            assert_eq!(
                parse(&value.to_string())
                    .unwrap()
                    .initialisation
                    .unwrap()
                    .ths,
                Some(value)
            );
        }
        for value in ["-4.00001", "13.50001", "nan", "inf", "-inf"] {
            assert!(matches!(
                parse(value),
                Err(ConfigError::InvalidInitialisation { field: "ths", .. })
            ));
        }
        for value in ["'1'", "true", "[]", "1\nths = 2"] {
            assert!(parse(value).is_err());
        }
        assert!(matches!(
            parse("1\nTHS = 2"),
            Err(ConfigError::UnexpectedField { section, field })
                if section == "initialisation" && field == "THS"
        ));
    }

    use super::*;

    #[test]
    fn initialisation_is_optional_and_requires_complete_finite_targets() {
        assert_eq!(Config::new(VALID_CONFIG).unwrap().initialisation, None);
        let parse =
            |body: &str| Config::new(&format!("{VALID_CONFIG}\n[initialisation]\n{body}\n"));
        assert_eq!(
            parse("zfw = 60000\ngw = 65000.0\ngwcg = 25.0")
                .unwrap()
                .initialisation,
            Some(InitialisationConfig {
                zfw: Some(60000.0),
                gw: Some(65000.0),
                gwcg: Some(25.0),
                ths: None,
            })
        );
        assert!(parse("zfw = 60000\ngw = 60000\ngwcg = 25").is_ok());
        assert_eq!(parse("").unwrap().initialisation, None);
        for body in [
            "zfw = 60000",
            "zfw = 60000\ngw = 65000",
            "gw = 65000\ngwcg = 25",
            "zfw = 60000\ngwcg = 25",
            "zfw = '60000'\ngw = 65000\ngwcg = 25",
            "zfw = 60000\ngw = 65000\ngwcg = 25\nextra = 1",
            "zfw = 60000\nzfw = 60000\ngw = 65000\ngwcg = 25",
        ] {
            assert!(parse(body).is_err(), "accepted {body}");
        }
        for field in ["zfw", "gw", "gwcg"] {
            for invalid in ["nan", "inf", "-inf"] {
                let body = [("zfw", "60000"), ("gw", "65000"), ("gwcg", "25")]
                    .map(|(name, value)| {
                        format!("{name} = {}", if name == field { invalid } else { value })
                    })
                    .join("\n");
                assert!(
                    matches!(parse(&body), Err(ConfigError::InvalidInitialisation { field: actual, .. }) if actual == field)
                );
            }
        }
        for body in [
            "zfw = 0\ngw = 65000\ngwcg = 25",
            "zfw = -1\ngw = 65000\ngwcg = 25",
            "zfw = 60000\ngw = 0\ngwcg = 25",
            "zfw = 60000\ngw = -1\ngwcg = 25",
            "zfw = 60000\ngw = 59999\ngwcg = 25",
        ] {
            assert!(matches!(
                parse(body),
                Err(ConfigError::InvalidInitialisation { .. })
            ));
        }
        assert!(
            matches!(parse("zfw = 60000\ngw = 65000\ngwcg = 25\ntimeout = 30"),
            Err(ConfigError::UnexpectedField { section, field }) if section == "initialisation" && field == "timeout")
        );
        assert!(Config::new(&format!("initialisation = 1\n{VALID_CONFIG}")).is_err());
    }

    #[test]
    fn initialisation_mass_group_is_all_or_none_independently_of_ths() {
        for include_ths in [false, true] {
            for mask in 0..8 {
                let mut body = String::new();
                for (index, (name, value)) in [("zfw", 60000), ("gw", 65000), ("gwcg", 25)]
                    .into_iter()
                    .enumerate()
                {
                    if mask & (1 << index) != 0 {
                        body.push_str(&format!("{name} = {value}\n"));
                    }
                }
                if include_ths {
                    body.push_str("ths = 1.0\n");
                }
                let parsed = Config::new(&format!("{VALID_CONFIG}\n[initialisation]\n{body}"));
                if mask != 0 && mask != 7 {
                    assert!(
                        matches!(
                            parsed,
                            Err(ConfigError::IncompleteInitialisationMassBalance)
                        ),
                        "accepted {body}"
                    );
                } else {
                    let targets = parsed.unwrap().initialisation;
                    assert_eq!(targets.is_some(), mask == 7 || include_ths);
                    if let Some(targets) = targets {
                        assert_eq!(targets.has_mass_balance(), mask == 7);
                        assert_eq!(targets.ths, include_ths.then_some(1.0));
                    }
                }
            }
        }
        for invalid in ["nan", "inf", "-4.01", "13.51"] {
            assert!(matches!(
                Config::new(&format!(
                    "{VALID_CONFIG}\n[initialisation]\nths = {invalid}"
                )),
                Err(ConfigError::InvalidInitialisation { field: "ths", .. })
            ));
        }
    }

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

    fn assert_error(config: &str, assertion: impl FnOnce(&ConfigError)) {
        match Config::new(config) {
            Ok(parsed) => panic!("configuration unexpectedly parsed: {parsed:?}"),
            Err(error) => assertion(&error),
        }
    }

    #[test]
    fn parses_default_configuration_file() {
        match Config::new(include_str!("../../../example/replayer_config.toml")) {
            Ok(config) => {
                assert_eq!(config.inject.len(), 2);
                assert_eq!(config.record.len(), 4);
            }
            Err(error) => panic!("default configuration should parse: {error}"),
        }
    }

    #[test]
    fn parses_readme_configuration() {
        let config = match Config::new(VALID_CONFIG) {
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
    fn accepts_omitted_and_empty_recordings() {
        let (injections_only, _) = VALID_CONFIG.split_once("[record.0]").unwrap();
        for suffix in ["", "[record]\n"] {
            let config = Config::new(&format!("{injections_only}{suffix}")).unwrap();
            assert_eq!(config.inject.len(), 2);
            assert!(config.record.is_empty());
        }
    }

    #[test]
    fn rejects_omitted_and_empty_injections() {
        for suffix in ["", "[inject]\n"] {
            let contents = format!("format_version = 1\ninput_file = \"scenario.csv\"\n{suffix}");
            assert_error(&contents, |error| {
                assert!(matches!(
                    error,
                    ConfigError::EmptySection { section: "inject" }
                ));
            });
        }
    }

    #[test]
    fn reads_configuration_file() {
        let path =
            std::env::temp_dir().join(format!("replay-valid-config-{}.toml", std::process::id()));
        if let Err(error) = std::fs::write(&path, VALID_CONFIG) {
            panic!("failed to create test configuration: {error}");
        }

        let result = Config::read_config_file(&path);
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

        match Config::read_config_file(&path) {
            Err(ConfigFileError::Read { .. }) => {}
            unexpected => panic!("expected file I/O error, got: {unexpected:?}"),
        }
    }

    #[test]
    fn rejects_unsupported_version() {
        assert_error(
            &VALID_CONFIG.replacen("format_version = 1", "format_version = 2", 1),
            |error| match error {
                ConfigError::UnsupportedFormatVersion { .. } => {}
                _ => panic!("unexpected error: {error:?}"),
            },
        );
    }

    #[test]
    fn accepts_arbitrary_injection_names_and_variables() {
        let arbitrary = VALID_CONFIG
            .replacen("sidestick_pitch_position\"", "custom_input\"", 1)
            .replacen("K:AXIS_ELEVATOR_SET", "L:CUSTOM_INPUT", 1);
        match Config::new(&arbitrary) {
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
            |error| match error {
                ConfigError::EmptyInjectionName { .. } => {}
                _ => panic!("unexpected error: {error:?}"),
            },
        );
    }

    #[test]
    fn requires_injection_variable() {
        assert_error(
            &VALID_CONFIG.replacen("variable = \"K:AXIS_ELEVATOR_SET\"\n", "", 1),
            |error| match error {
                ConfigError::Toml(_) => {}
                _ => panic!("unexpected error: {error:?}"),
            },
        );
    }

    #[test]
    fn accepts_arbitrary_recording_names_and_variables() {
        let arbitrary = VALID_CONFIG
            .replacen("name = \"pitch\"", "name = \"custom_response\"", 1)
            .replacen("A:PLANE PITCH DEGREES", "L:CUSTOM_RESPONSE", 1)
            .replacen("unit = \"radians\"\n", "", 1);
        match Config::new(&arbitrary) {
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
            |error| match error {
                ConfigError::EmptyRecordingName { .. } => {}
                _ => panic!("unexpected error: {error:?}"),
            },
        );
    }

    #[test]
    fn requires_non_empty_recording_variable() {
        assert_error(
            &VALID_CONFIG.replacen("variable = \"A:PLANE PITCH DEGREES\"\n", "", 1),
            |error| match error {
                ConfigError::Toml(_) => {}
                _ => panic!("unexpected error: {error:?}"),
            },
        );
        assert_error(
            &VALID_CONFIG.replacen("variable = \"A:PLANE PITCH DEGREES\"", "variable = \"\"", 1),
            |error| match error {
                ConfigError::EmptyRecordingVariable { .. } => {}
                _ => panic!("unexpected error: {error:?}"),
            },
        );
    }

    #[test]
    fn validates_recording_units() {
        assert_error(
            &VALID_CONFIG.replacen("unit = \"radians\"\n", "", 1),
            |error| match error {
                ConfigError::MissingRecordingUnit { .. } => {}
                _ => panic!("unexpected error: {error:?}"),
            },
        );
        assert_error(
            &VALID_CONFIG.replacen("unit = \"radians\"", "unit = \"\"", 1),
            |error| match error {
                ConfigError::EmptyRecordingUnit { .. } => {}
                _ => panic!("unexpected error: {error:?}"),
            },
        );
        assert_error(
            &VALID_CONFIG.replacen("A:PLANE PITCH DEGREES", "L:CUSTOM_RESPONSE", 1),
            |error| match error {
                ConfigError::UnexpectedRecordingUnit { .. } => {}
                _ => panic!("unexpected error: {error:?}"),
            },
        );
    }

    #[test]
    fn accepts_optional_recording_sampling_rate() {
        let config = Config::new(&VALID_CONFIG.replacen(
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
                |error| match error {
                    ConfigError::InvalidRecordingSamplingRate { .. } => {}
                    _ => panic!("unexpected error: {error:?}"),
                },
            );
        }
    }

    #[test]
    fn rejects_non_numeric_and_non_contiguous_indexes() {
        assert_error(
            &VALID_CONFIG.replacen("[inject.0]", "[inject.first]", 1),
            |error| match error {
                ConfigError::InvalidIndex {
                    section: "inject", ..
                } => {}
                _ => panic!("unexpected error: {error:?}"),
            },
        );
        assert_error(
            &VALID_CONFIG.replacen("[record.1]", "[record.4]", 1),
            |error| match error {
                ConfigError::NonContiguousIndex {
                    section: "record", ..
                } => {}
                _ => panic!("unexpected error: {error:?}"),
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
            |error| match error {
                ConfigError::DuplicateInjectionSignal { .. } => {}
                _ => panic!("unexpected error: {error:?}"),
            },
        );
        assert_error(
            &VALID_CONFIG.replacen("name = \"roll\"", "name = \"pitch\"", 1),
            |error| match error {
                ConfigError::DuplicateRecordingSignal { .. } => {}
                _ => panic!("unexpected error: {error:?}"),
            },
        );
    }

    #[test]
    fn rejects_signals_used_in_both_inject_and_record_sections() {
        assert_error(
            &VALID_CONFIG.replacen("name = \"pitch\"", "name = \"sidestick_pitch_position\"", 1),
            |error| match error {
                ConfigError::DuplicateSignalAcrossSections { .. } => {}
                _ => panic!("unexpected error: {error:?}"),
            },
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
                |error| match error {
                    ConfigError::InvalidInjectionRange { .. } => {}
                    _ => panic!("unexpected error: {error:?}"),
                },
            );
        }
        assert_error(
            &VALID_CONFIG.replacen(
                "simulator_range = [-1.0, 1.0]",
                "simulator_range = [-16384.0, 16384.0]",
                1,
            ),
            |error| match error {
                ConfigError::UnsafeSimulatorRange { .. } => {}
                _ => panic!("unexpected error: {error:?}"),
            },
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
            |error| match error {
                ConfigError::UnexpectedField { section, field }
                    if section == "root" && field == "unexpected" => {}
                _ => panic!("unexpected error: {error:?}"),
            },
        );
        for (unknown_field, source_text) in [
            ("time_column", "time_column = \"custom.time\""),
            ("value_column", "value_column = \"custom.value\""),
            ("interpolation", "interpolation = \"linear\""),
        ] {
            assert_error(
                &VALID_CONFIG.replacen(
                    "source_range = [-100.0, 100.0]",
                    &format!("source_range = [-100.0, 100.0]\n{source_text}"),
                    1,
                ),
                |error| match error {
                    ConfigError::UnexpectedField { section, field }
                        if section == "inject.0" && field == unknown_field => {}
                    _ => panic!("unexpected error: {error:?}"),
                },
            );
        }
        let removed_field = "range = [-180.0, 180.0]";
        assert_error(
            &VALID_CONFIG.replacen(
                "variable = \"A:PLANE PITCH DEGREES\"",
                &format!("variable = \"A:PLANE PITCH DEGREES\"\n{removed_field}"),
                1,
            ),
            |error| match error {
                ConfigError::UnexpectedField { section, .. } if section == "record.0" => {}
                _ => panic!("unexpected error: {error:?}"),
            },
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

        let config = match Config::new(&reordered) {
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
