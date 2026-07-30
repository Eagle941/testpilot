use std::collections::HashMap;
use std::io::Read;

use csv::{Position, ReaderBuilder, StringRecord, Trim};

use crate::config::{InjectionConfig, ReplayConfig};

use super::ScenarioError;

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
    /// Latest final timestamp across all configured input series.
    pub duration_seconds: f64,
    /// Per-signal summaries in configuration injection order.
    pub signals: Vec<SignalSummary>,
}

#[derive(Debug, Clone, Copy)]
pub(super) struct ColumnPair {
    pub(super) time_idx: usize,
    pub(super) value_idx: usize,
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
/// strictly increasing timestamps, and dense series. It consumes the reader
/// without loading the complete CSV into memory.
pub fn validate_scenario<R: Read>(
    reader: R,
    config: &ReplayConfig,
) -> Result<ScenarioSummary, ScenarioError> {
    let mut csv = ReaderBuilder::new().trim(Trim::All).from_reader(reader);
    let headers = csv.headers().map_err(ScenarioError::Csv)?.clone();
    validate_unique_headers(&headers)?;

    let columns = config
        .inject
        .iter()
        .map(|injection| find_column_indices(&headers, injection))
        .collect::<Result<Vec<_>, _>>()?;
    let mut states = (0..config.inject.len())
        .map(|_| ValidationState::default())
        .collect::<Vec<_>>();

    for record in csv.records() {
        let record = record.map_err(ScenarioError::Csv)?;
        let line = record.position().map(Position::line);

        for (index, injection) in config.inject.iter().enumerate() {
            validate_pair(&record, columns[index], injection, &mut states[index], line)?;
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

/// Returns the CSV-header indexes for one configured injection's columns.
///
/// The returned [`ColumnPair`] contains the zero-based indexes of the time and
/// value columns named by `injection.time_column` and `injection.value_column`.
/// The scenario CSV format requires every `<signal>.time` column to be
/// immediately followed by its matching `<signal>.value` column. Missing
/// columns return [`ScenarioError::MissingColumn`]; present but non-adjacent
/// columns return [`ScenarioError::NonAdjacentColumns`].
pub(super) fn find_column_indices(
    headers: &StringRecord,
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

    Ok(ColumnPair {
        time_idx: time,
        value_idx: value,
    })
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

    let time = time_text.parse::<f64>()?;
    let value = value_text.parse::<f64>()?;

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

fn summarize(
    config: &ReplayConfig,
    states: Vec<ValidationState>,
) -> Result<ScenarioSummary, ScenarioError> {
    let mut duration_seconds: f64 = 0.0;
    let mut signals = Vec::with_capacity(states.len());

    for (injection, state) in config.inject.iter().zip(states) {
        let final_time = state
            .last_time
            .ok_or_else(|| ScenarioError::MissingSamples {
                signal: injection.name.clone(),
            })?;
        duration_seconds = duration_seconds.max(final_time);
        signals.push(SignalSummary {
            signal: injection.name.clone(),
            sample_count: state.sample_count,
            final_time_seconds: final_time,
        });
    }

    Ok(ScenarioSummary {
        duration_seconds,
        signals,
    })
}
