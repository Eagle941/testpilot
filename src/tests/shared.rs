use std::collections::HashMap;
use std::io::Read;
use std::time::Duration;

use csv::{Position, ReaderBuilder, StringRecord, Trim};

use crate::config::ReplayConfig;
use crate::error::ScenarioError;

pub(super) const CONFIG: &str = r#"
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
"#;

pub(super) const HEADER: &str = "sidestick_pitch_position.time,sidestick_pitch_position.value,sidestick_roll_position.time,sidestick_roll_position.value\n";

pub(super) fn time(seconds: f64) -> Duration {
    Duration::try_from_secs_f64(seconds).unwrap()
}

#[derive(Debug, Clone, Copy)]
pub(super) struct ColumnPair {
    pub(super) time_idx: usize,
    pub(super) value_idx: usize,
}

#[derive(Debug, Default, Clone)]
pub(super) struct ValidationState {
    pub(super) sample_count: u64,
    pub(super) last_time: Option<Duration>,
    pub(super) ended: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub(super) struct SignalSummary {
    /// Logical signal name represented by the paired CSV columns.
    pub(super) signal: String,
    /// Number of validated `(time, value)` samples.
    pub(super) sample_count: u64,
    /// Scenario-relative time of the final sample.
    pub(super) final_time: Duration,
}

#[derive(Debug, Clone, PartialEq)]
pub(super) struct ScenarioSummary {
    /// Latest final timestamp across all configured input series.
    pub(super) duration: Duration,
    /// Per-signal summaries in configuration injection order.
    pub(super) signals: Vec<SignalSummary>,
}

pub(super) fn config() -> ReplayConfig {
    ReplayConfig::new(CONFIG).unwrap_or_else(|error| panic!("valid test config rejected: {error}"))
}

pub(super) fn parse_time(
    text: &str,
    signal: &str,
    line: Option<u64>,
) -> Result<Duration, ScenarioError> {
    let time_seconds = text.parse::<f64>()?;
    if !time_seconds.is_finite() {
        return Err(ScenarioError::NonFiniteTime {
            signal: signal.to_owned(),
            line,
        });
    }
    if time_seconds < 0.0 {
        return Err(ScenarioError::NegativeTime {
            signal: signal.to_owned(),
            time_seconds,
            line,
        });
    }
    Duration::try_from_secs_f64(time_seconds).map_err(|_| ScenarioError::TimeOutOfRange {
        signal: signal.to_owned(),
        time_seconds,
        line,
    })
}

pub(super) fn validate_unique_headers(headers: &StringRecord) -> Result<(), ScenarioError> {
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

pub(super) fn find_column_indices(
    headers: &StringRecord,
    signal: &str,
) -> Result<ColumnPair, ScenarioError> {
    let time_column = format!("{signal}.time");
    let value_column = format!("{signal}.value");
    let time_idx = headers
        .iter()
        .position(|header| header == time_column)
        .ok_or_else(|| ScenarioError::MissingColumn {
            signal: signal.to_owned(),
            column: time_column.clone(),
        })?;
    let value_idx = headers
        .iter()
        .position(|header| header == value_column)
        .ok_or_else(|| ScenarioError::MissingColumn {
            signal: signal.to_owned(),
            column: value_column.clone(),
        })?;

    if time_idx.checked_add(1) != Some(value_idx) {
        return Err(ScenarioError::NonAdjacentColumns {
            signal: signal.to_owned(),
            time_column,
            value_column,
        });
    }

    Ok(ColumnPair {
        time_idx,
        value_idx,
    })
}

pub(super) fn validate_pair(
    record: &StringRecord,
    columns: ColumnPair,
    injection_name: &str,
    source_range: [f64; 2],
    state: &mut ValidationState,
    line: Option<u64>,
) -> Result<(), ScenarioError> {
    let time_text = record.get(columns.time_idx).unwrap_or_default();
    let value_text = record.get(columns.value_idx).unwrap_or_default();
    let time_empty = time_text.is_empty();
    let value_empty = value_text.is_empty();

    if time_empty != value_empty {
        return Err(ScenarioError::HalfPopulatedPair {
            signal: injection_name.to_owned(),
            line,
        });
    }
    if time_empty {
        state.ended = true;
        return Ok(());
    }
    if state.ended {
        return Err(ScenarioError::SparseSeries {
            signal: injection_name.to_owned(),
            line,
        });
    }

    let time = parse_time(time_text, injection_name, line)?;
    let value = value_text.parse::<f64>()?;

    if !value.is_finite() {
        return Err(ScenarioError::NonFiniteValue {
            signal: injection_name.to_owned(),
            line,
        });
    }
    if state.sample_count == 0 && !time.is_zero() {
        return Err(ScenarioError::FirstTimestampNotZero {
            signal: injection_name.to_owned(),
            time,
            line,
        });
    }
    if let Some(previous) = state.last_time
        && time <= previous
    {
        return Err(ScenarioError::NonIncreasingTime {
            signal: injection_name.to_owned(),
            previous,
            time,
            line,
        });
    }
    if value < source_range[0] || value > source_range[1] {
        return Err(ScenarioError::ValueOutsideSourceRange {
            signal: injection_name.to_owned(),
            value,
            minimum: source_range[0],
            maximum: source_range[1],
            line,
        });
    }

    state.sample_count += 1;
    state.last_time = Some(time);
    Ok(())
}

pub(super) fn summarize(
    config: &ReplayConfig,
    states: Vec<ValidationState>,
) -> Result<ScenarioSummary, ScenarioError> {
    let mut duration = Duration::ZERO;
    let mut signals = Vec::with_capacity(states.len());

    for (injection, state) in config.inject.iter().zip(states) {
        let final_time = state
            .last_time
            .ok_or_else(|| ScenarioError::MissingSamples {
                signal: injection.name.clone(),
            })?;
        duration = duration.max(final_time);
        signals.push(SignalSummary {
            signal: injection.name.clone(),
            sample_count: state.sample_count,
            final_time,
        });
    }

    Ok(ScenarioSummary { duration, signals })
}

pub(super) fn validate(body: &str) -> Result<ScenarioSummary, ScenarioError> {
    validate_scenario(format!("{HEADER}{body}").as_bytes(), &config())
}

pub(super) fn validate_scenario<R: Read>(
    reader: R,
    config: &ReplayConfig,
) -> Result<ScenarioSummary, ScenarioError> {
    let mut csv = ReaderBuilder::new().trim(Trim::All).from_reader(reader);
    let headers = csv.headers()?.clone();
    validate_unique_headers(&headers)?;

    let columns = config
        .inject
        .iter()
        .map(|injection| find_column_indices(&headers, &injection.name))
        .collect::<Result<Vec<_>, _>>()?;
    let mut states = vec![ValidationState::default(); config.inject.len()];

    for record in csv.records() {
        let record = record?;
        let line = record.position().map(Position::line);
        for ((injection, columns), state) in config.inject.iter().zip(&columns).zip(&mut states) {
            validate_pair(
                &record,
                *columns,
                &injection.name,
                injection.source_range,
                state,
                line,
            )?;
        }
    }

    summarize(config, states)
}
