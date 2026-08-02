use std::collections::HashMap;
use std::io::Read;
use std::time::Duration;

use csv::{Position, ReaderBuilder, StringRecord, Trim};

use crate::config::{InjectionConfig, ReplayConfig};

use super::ScenarioError;
use super::cursor::{ColumnPair, find_column_indices, parse_time};

/// Preflight summary for one configured input signal.
#[derive(Debug, Clone, PartialEq)]
pub struct SignalSummary {
    /// Logical signal name represented by the paired CSV columns.
    pub signal: String,
    /// Number of validated `(time, value)` samples.
    pub sample_count: u64,
    /// Scenario-relative time of the final sample.
    pub final_time: Duration,
}

/// Result of a successful streaming scenario preflight pass.
#[derive(Debug, Clone, PartialEq)]
pub struct ScenarioSummary {
    /// Latest final timestamp across all configured input series.
    pub duration: Duration,
    /// Per-signal summaries in configuration injection order.
    pub signals: Vec<SignalSummary>,
}

#[derive(Debug, Default, Clone)]
struct ValidationState {
    sample_count: u64,
    last_time: Option<Duration>,
    ended: bool,
}

/// Validates a scenario CSV stream against a replay configuration.
///
/// The function checks paired-column structure, finite values, source ranges,
/// strictly increasing timestamps, and dense series. It consumes the reader
/// without loading the complete CSV into memory.
pub fn validate_scenario<R: Read>(
    reader: R,
    config: &ReplayConfig,
) -> Result<ScenarioSummary, ScenarioError> {
    let mut csv = ReaderBuilder::new().trim(Trim::All).from_reader(reader);
    let headers = csv.headers()?.clone();
    validate_unique_headers(&headers)?;

    let columns = config
        .inject
        .iter()
        .map(|injection| find_column_indices(&headers, injection))
        .collect::<Result<Vec<_>, _>>()?;
    let mut states = vec![ValidationState::default(); config.inject.len()];

    for record in csv.records() {
        let record = record?;
        let line = record.position().map(Position::line);

        for ((injection, columns), state) in config.inject.iter().zip(&columns).zip(&mut states) {
            validate_pair(&record, *columns, injection, state, line)?;
        }
    }

    summarize(config, states)
}

fn validate_unique_headers(headers: &StringRecord) -> Result<(), ScenarioError> {
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

fn validate_pair(
    record: &StringRecord,
    columns: ColumnPair,
    injection: &InjectionConfig,
    state: &mut ValidationState,
    line: Option<u64>,
) -> Result<(), ScenarioError> {
    let time_text = record.get(columns.time_idx).unwrap_or_default();
    let value_text = record.get(columns.value_idx).unwrap_or_default();
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

    let time = parse_time(time_text, &injection.name, line)?;
    let value = value_text.parse::<f64>()?;

    if !value.is_finite() {
        return Err(ScenarioError::NonFiniteValue {
            signal: injection.name.clone(),
            line,
        });
    }
    if state.sample_count == 0 && !time.is_zero() {
        return Err(ScenarioError::FirstTimestampNotZero {
            signal: injection.name.clone(),
            time,
            line,
        });
    }
    if let Some(previous) = state.last_time
        && time <= previous
    {
        return Err(ScenarioError::NonIncreasingTime {
            signal: injection.name.clone(),
            previous,
            time,
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

    state.sample_count += 1;
    state.last_time = Some(time);
    Ok(())
}

fn summarize(
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
