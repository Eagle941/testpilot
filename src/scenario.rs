//! Streaming validation for the independently sampled scenario CSV format.
//!
//! Validation retains only per-signal counters and the previous timestamp, so
//! memory use does not grow with scenario duration.

use std::collections::HashMap;
use std::io::Read;

use crate::config::{InjectionConfig, ReplayConfig};

pub use crate::error::ScenarioError;

/// Preflight summary for one configured input signal.
#[derive(Debug, Clone, PartialEq)]
pub struct SignalSummary {
    /// Logical signal name represented by the paired CSV columns.
    pub signal: String,
    /// Number of validated `(time, value)` samples.
    pub sample_count: u64,
    /// Timestamp of the final sample in scenario-relative seconds.
    pub final_time_seconds: f64,
}

/// Result of a successful streaming scenario preflight pass.
#[derive(Debug, Clone, PartialEq)]
pub struct ScenarioSummary {
    /// Common final timestamp of all configured input series.
    pub duration_seconds: f64,
    /// Per-signal summaries in configuration injection order.
    pub signals: Vec<SignalSummary>,
}

#[derive(Debug, Clone, Copy)]
struct ColumnPair {
    time: usize,
    value: usize,
}

#[derive(Debug, Default)]
struct ValidationState {
    sample_count: u64,
    last_time: Option<f64>,
    ended: bool,
}

/// Validates a scenario CSV stream against a replay configuration.
///
/// The function checks paired-column structure, finite values, source ranges,
/// strictly increasing timestamps, dense series, and a common final timestamp.
/// It consumes the reader without loading the complete CSV into memory.
pub fn validate_scenario<R: Read>(
    reader: R,
    config: &ReplayConfig,
) -> Result<ScenarioSummary, ScenarioError> {
    let mut csv = csv::ReaderBuilder::new()
        .trim(csv::Trim::All)
        .from_reader(reader);
    let headers = csv.headers().map_err(ScenarioError::Csv)?.clone();
    validate_unique_headers(&headers)?;

    let columns = config
        .inject
        .iter()
        .map(|injection| find_columns(&headers, injection))
        .collect::<Result<Vec<_>, _>>()?;
    let mut states = (0..config.inject.len())
        .map(|_| ValidationState::default())
        .collect::<Vec<_>>();

    for record in csv.records() {
        let record = record.map_err(ScenarioError::Csv)?;
        let line = record.position().map(csv::Position::line);

        for (index, injection) in config.inject.iter().enumerate() {
            validate_pair(&record, columns[index], injection, &mut states[index], line)?;
        }
    }

    summarize(config, states)
}

fn validate_unique_headers(headers: &csv::StringRecord) -> Result<(), ScenarioError> {
    let mut first_indexes = HashMap::with_capacity(headers.len());
    for (index, header) in headers.iter().enumerate() {
        if let Some(first_index) = first_indexes.insert(header, index) {
            return Err(ScenarioError::DuplicateHeader {
                column: header.to_owned(),
                first_index,
                duplicate_index: index,
            });
        }
    }
    Ok(())
}

fn find_columns(
    headers: &csv::StringRecord,
    injection: &InjectionConfig,
) -> Result<ColumnPair, ScenarioError> {
    let time = headers
        .iter()
        .position(|header| header == injection.time_column)
        .ok_or_else(|| ScenarioError::MissingColumn {
            signal: injection.name.clone(),
            column: injection.time_column.clone(),
        })?;
    let value = headers
        .iter()
        .position(|header| header == injection.value_column)
        .ok_or_else(|| ScenarioError::MissingColumn {
            signal: injection.name.clone(),
            column: injection.value_column.clone(),
        })?;

    if time.checked_add(1) != Some(value) {
        return Err(ScenarioError::NonAdjacentColumns {
            signal: injection.name.clone(),
            time_column: injection.time_column.clone(),
            value_column: injection.value_column.clone(),
        });
    }

    Ok(ColumnPair { time, value })
}

fn validate_pair(
    record: &csv::StringRecord,
    columns: ColumnPair,
    injection: &InjectionConfig,
    state: &mut ValidationState,
    line: Option<u64>,
) -> Result<(), ScenarioError> {
    let time_text = record.get(columns.time).unwrap_or_default();
    let value_text = record.get(columns.value).unwrap_or_default();
    let time_empty = time_text.is_empty();
    let value_empty = value_text.is_empty();

    if time_empty != value_empty {
        return Err(ScenarioError::HalfPopulatedPair {
            signal: injection.name.clone(),
            line,
        });
    }
    if time_empty {
        state.ended = true;
        return Ok(());
    }
    if state.ended {
        return Err(ScenarioError::SparseSeries {
            signal: injection.name.clone(),
            line,
        });
    }

    let time = parse_number(time_text, &injection.name, &injection.time_column, line)?;
    let value = parse_number(value_text, &injection.name, &injection.value_column, line)?;

    if !time.is_finite() {
        return Err(ScenarioError::NonFiniteTime {
            signal: injection.name.clone(),
            line,
        });
    }
    if time < 0.0 {
        return Err(ScenarioError::NegativeTime {
            signal: injection.name.clone(),
            time_seconds: time,
            line,
        });
    }
    if !value.is_finite() {
        return Err(ScenarioError::NonFiniteValue {
            signal: injection.name.clone(),
            line,
        });
    }
    if state.sample_count == 0 && time != 0.0 {
        return Err(ScenarioError::FirstTimestampNotZero {
            signal: injection.name.clone(),
            time_seconds: time,
            line,
        });
    }
    if let Some(previous) = state.last_time
        && time <= previous
    {
        return Err(ScenarioError::NonIncreasingTime {
            signal: injection.name.clone(),
            previous_seconds: previous,
            time_seconds: time,
            line,
        });
    }
    if value < injection.source_range[0] || value > injection.source_range[1] {
        return Err(ScenarioError::ValueOutsideSourceRange {
            signal: injection.name.clone(),
            value,
            minimum: injection.source_range[0],
            maximum: injection.source_range[1],
            line,
        });
    }

    state.sample_count =
        state
            .sample_count
            .checked_add(1)
            .ok_or_else(|| ScenarioError::SampleCountOverflow {
                signal: injection.name.clone(),
            })?;
    state.last_time = Some(time);
    Ok(())
}

fn parse_number(
    text: &str,
    signal: &str,
    column: &str,
    line: Option<u64>,
) -> Result<f64, ScenarioError> {
    text.parse::<f64>()
        .map_err(|source| ScenarioError::InvalidNumber {
            signal: signal.to_owned(),
            column: column.to_owned(),
            value: text.to_owned(),
            line,
            source,
        })
}

fn summarize(
    config: &ReplayConfig,
    states: Vec<ValidationState>,
) -> Result<ScenarioSummary, ScenarioError> {
    let mut duration = None;
    let mut signals = Vec::with_capacity(states.len());

    for (injection, state) in config.inject.iter().zip(states) {
        let final_time = state
            .last_time
            .ok_or_else(|| ScenarioError::MissingSamples {
                signal: injection.name.clone(),
            })?;
        if let Some(expected) = duration
            && final_time != expected
        {
            return Err(ScenarioError::FinalTimestampMismatch {
                signal: injection.name.clone(),
                expected_seconds: expected,
                actual_seconds: final_time,
            });
        }
        duration = Some(final_time);
        signals.push(SignalSummary {
            signal: injection.name.clone(),
            sample_count: state.sample_count,
            final_time_seconds: final_time,
        });
    }

    Ok(ScenarioSummary {
        duration_seconds: duration.unwrap_or(0.0),
        signals,
    })
}

#[cfg(test)]
mod tests {
    use crate::config::parse_config;

    use super::*;

    const CONFIG: &str = r#"
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
"#;

    const HEADER: &str = "sidestick_pitch_position.time,sidestick_pitch_position.value,sidestick_roll_position.time,sidestick_roll_position.value\n";

    fn config() -> ReplayConfig {
        parse_config(CONFIG).unwrap_or_else(|error| panic!("valid test config rejected: {error}"))
    }

    fn validate(body: &str) -> Result<ScenarioSummary, ScenarioError> {
        validate_scenario(format!("{HEADER}{body}").as_bytes(), &config())
    }

    #[test]
    fn validates_irregular_independent_series() {
        let summary = validate(
            "0,0,0,0\n\
             0.15,10,0.2,5\n\
             0.425,-5,0.7,0\n\
             0.7,0,,\n",
        )
        .unwrap_or_else(|error| panic!("valid scenario rejected: {error}"));

        assert_eq!(summary.duration_seconds, 0.7);
        assert_eq!(summary.signals[0].sample_count, 4);
        assert_eq!(summary.signals[1].sample_count, 3);
    }

    #[test]
    fn rejects_missing_duplicate_and_non_adjacent_headers() {
        let missing = "sidestick_pitch_position.time,other,sidestick_roll_position.time,sidestick_roll_position.value\n0,0,0,0\n";
        assert!(matches!(
            validate_scenario(missing.as_bytes(), &config()),
            Err(ScenarioError::MissingColumn { .. })
        ));

        let duplicate = "sidestick_pitch_position.time,sidestick_pitch_position.value,sidestick_pitch_position.value,sidestick_roll_position.time,sidestick_roll_position.value\n0,0,0,0,0\n";
        assert!(matches!(
            validate_scenario(duplicate.as_bytes(), &config()),
            Err(ScenarioError::DuplicateHeader { .. })
        ));

        let non_adjacent = "sidestick_pitch_position.time,other,sidestick_pitch_position.value,sidestick_roll_position.time,sidestick_roll_position.value\n0,0,0,0,0\n";
        assert!(matches!(
            validate_scenario(non_adjacent.as_bytes(), &config()),
            Err(ScenarioError::NonAdjacentColumns { .. })
        ));
    }

    #[test]
    fn rejects_half_populated_and_sparse_pairs() {
        assert!(matches!(
            validate("0,0,0,0\n0.5,,0.5,0\n"),
            Err(ScenarioError::HalfPopulatedPair { .. })
        ));
        assert!(matches!(
            validate("0,0,0,0\n,,0.5,0\n0.5,0,1,0\n"),
            Err(ScenarioError::SparseSeries { .. })
        ));
    }

    #[test]
    fn rejects_invalid_timestamps() {
        for (body, predicate) in [
            ("0.1,0,0,0\n1,0,1,0\n", "first"),
            ("0,0,0,0\n0,0,1,0\n", "order"),
            ("0,0,0,0\n-1,0,1,0\n", "negative"),
            ("0,0,0,0\nNaN,0,1,0\n", "finite"),
        ] {
            let error = validate(body).expect_err("invalid timestamp accepted");
            match predicate {
                "first" => assert!(matches!(error, ScenarioError::FirstTimestampNotZero { .. })),
                "order" => assert!(matches!(error, ScenarioError::NonIncreasingTime { .. })),
                "negative" => assert!(matches!(error, ScenarioError::NegativeTime { .. })),
                "finite" => assert!(matches!(error, ScenarioError::NonFiniteTime { .. })),
                _ => panic!("unknown test predicate"),
            }
        }
    }

    #[test]
    fn rejects_invalid_and_out_of_range_values() {
        assert!(matches!(
            validate("0,nope,0,0\n1,0,1,0\n"),
            Err(ScenarioError::InvalidNumber { .. })
        ));
        assert!(matches!(
            validate("0,NaN,0,0\n1,0,1,0\n"),
            Err(ScenarioError::NonFiniteValue { .. })
        ));
        assert!(matches!(
            validate("0,101,0,0\n1,0,1,0\n"),
            Err(ScenarioError::ValueOutsideSourceRange { .. })
        ));
    }

    #[test]
    fn rejects_missing_samples_and_mismatched_final_times() {
        assert!(matches!(
            validate(",,0,0\n,,1,0\n"),
            Err(ScenarioError::MissingSamples { .. })
        ));
        assert!(matches!(
            validate("0,0,0,0\n1,0,2,0\n"),
            Err(ScenarioError::FinalTimestampMismatch { .. })
        ));
    }

    #[test]
    fn reports_csv_structure_errors() {
        assert!(matches!(validate("0,0,0\n"), Err(ScenarioError::Csv(_))));
    }

    #[test]
    fn validates_long_inputs_with_constant_state_size() {
        let mut csv = String::from(HEADER);
        for index in 0..20_000 {
            csv.push_str(&format!("{index},0,{index},0\n"));
        }

        let summary = validate_scenario(csv.as_bytes(), &config())
            .unwrap_or_else(|error| panic!("long scenario rejected: {error}"));
        assert_eq!(summary.signals[0].sample_count, 20_000);
        assert_eq!(summary.signals[1].sample_count, 20_000);
    }
}
