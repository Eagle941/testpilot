use std::fs::File;
use std::path::Path;
use std::time::Duration;

use csv::{Position, Reader, ReaderBuilder, StringRecord, Trim};

use crate::config::{InjectionConfig, ReplayConfig};
use crate::playback::{AffineRange, LinearSegment, PlaybackError, Sample};

use super::ScenarioError;

#[derive(Debug, Clone, Copy)]
pub(super) struct ColumnPair {
    pub(super) time_idx: usize,
    pub(super) value_idx: usize,
}

/// Parses scenario-relative seconds into a duration.
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

/// Returns the CSV-header indexes for one configured injection's columns.
///
/// The returned [`ColumnPair`] contains the zero-based indexes of the time and
/// value columns named by `injection.time_column` and `injection.value_column`.
/// The time column must immediately precede its matching value column.
pub(super) fn find_column_indices(
    headers: &StringRecord,
    injection: &InjectionConfig,
) -> Result<ColumnPair, ScenarioError> {
    let time_idx = headers
        .iter()
        .position(|header| header == injection.time_column)
        .ok_or_else(|| ScenarioError::MissingColumn {
            signal: injection.name.clone(),
            column: injection.time_column.clone(),
        })?;
    let value_idx = headers
        .iter()
        .position(|header| header == injection.value_column)
        .ok_or_else(|| ScenarioError::MissingColumn {
            signal: injection.name.clone(),
            column: injection.value_column.clone(),
        })?;

    if time_idx.checked_add(1) != Some(value_idx) {
        return Err(ScenarioError::NonAdjacentColumns {
            signal: injection.name.clone(),
            time_column: injection.time_column.clone(),
            value_column: injection.value_column.clone(),
        });
    }

    Ok(ColumnPair {
        time_idx,
        value_idx,
    })
}

/// Scenario data points used to calculate one injection value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InterpolationRows<'a> {
    /// Logical injection name.
    pub signal: &'a str,
    /// Prefixed simulator destination from the replay configuration.
    pub variable: &'a str,
    /// Earlier sample in the interpolation interval.
    pub previous: Sample,
    /// Later sample in the interpolation interval, or `None` after the series ends.
    pub next: Option<Sample>,
    /// Configured conversion from source scale to simulator scale.
    pub conversion: AffineRange,
}

impl InterpolationRows<'_> {
    /// Interpolates between two samples or holds the final sample after EOF.
    pub fn value_at(&self, elapsed: Duration) -> Result<f64, PlaybackError> {
        match self.next {
            Some(next) => LinearSegment::new(self.previous, next)?.value_at(elapsed),
            None => Ok(self.previous.value),
        }
    }
}

/// Incremental, read-only scenario loader with one file cursor per injection.
pub struct ScenarioPlayback {
    cursors: Vec<SignalCursor>,
}

impl ScenarioPlayback {
    /// Opens the scenario independently for every configured injection.
    ///
    /// Initialization reads each CSV header and the first two samples needed
    /// for interpolation. The scenario file is never opened for writing.
    pub fn new(
        path: impl AsRef<Path>,
        config: &ReplayConfig,
    ) -> Result<ScenarioPlayback, ScenarioError> {
        let path = path.as_ref();
        let cursors = config
            .inject
            .iter()
            .map(|injection| SignalCursor::new(path, injection))
            .collect::<Result<Vec<_>, _>>()?;

        Ok(ScenarioPlayback { cursors })
    }

    /// Advances every signal cursor for the current elapsed scenario time.
    ///
    /// Each cursor consumes at most one data row after initialization and
    /// advances when `elapsed` passes its next row.
    pub fn advance(&mut self, elapsed: Duration) -> Result<(), ScenarioError> {
        for cursor in &mut self.cursors {
            cursor.advance(elapsed)?;
        }
        Ok(())
    }

    /// Returns whether every signal cursor has passed its final sample.
    pub fn completed(&self) -> bool {
        self.cursors.iter().all(|cursor| cursor.ended)
    }

    /// Returns one bounding data-point pair per configured injection.
    pub fn interpolation_rows(&self) -> impl Iterator<Item = InterpolationRows<'_>> {
        self.cursors.iter().filter_map(|cursor| {
            Some(InterpolationRows {
                signal: &cursor.signal,
                variable: &cursor.variable,
                previous: cursor.previous?,
                next: cursor.next,
                conversion: cursor.conversion,
            })
        })
    }

    /// Returns the number of independently opened signal cursors.
    pub fn signal_count(&self) -> usize {
        self.cursors.len()
    }
}

/// Stateful reader for one independently sampled injection signal.
///
/// Each cursor owns its own read-only CSV reader and therefore its own file
/// position. It retains only the previous and next samples needed to
/// interpolate the current simulator frame, keeping memory use independent of
/// scenario duration. Once the reader reaches the final sample, it holds that
/// value until every configured cursor has ended.
struct SignalCursor {
    signal: String,
    variable: String,
    columns: ColumnPair,
    reader: Reader<File>,
    previous: Option<Sample>,
    next: Option<Sample>,
    conversion: AffineRange,
    ended: bool,
}

impl SignalCursor {
    fn new(path: &Path, injection: &InjectionConfig) -> Result<SignalCursor, ScenarioError> {
        let mut reader = ReaderBuilder::new().trim(Trim::All).from_path(path)?;
        let columns = find_column_indices(reader.headers()?, injection)?;
        let conversion = AffineRange::new(injection.source_range, injection.simulator_range)?;

        let mut cursor = SignalCursor {
            signal: injection.name.clone(),
            variable: injection.variable.clone(),
            columns,
            reader,
            previous: None,
            next: None,
            conversion,
            ended: false,
        };
        cursor.previous = Some(cursor.required_sample()?);
        cursor.next = Some(cursor.required_sample()?);
        Ok(cursor)
    }

    fn advance(&mut self, elapsed: Duration) -> Result<(), ScenarioError> {
        if self.ended {
            return Ok(());
        }

        if let Some(next) = self.next
            && elapsed > next.time
        {
            self.previous = Some(next);
            self.next = self.read_sample()?;
            self.ended = self.next.is_none();
        }
        Ok(())
    }

    fn required_sample(&mut self) -> Result<Sample, ScenarioError> {
        self.read_sample()?
            .ok_or_else(|| ScenarioError::MissingSamples {
                signal: self.signal.clone(),
            })
    }

    fn read_sample(&mut self) -> Result<Option<Sample>, ScenarioError> {
        let mut record = StringRecord::new();
        let has_record = self.reader.read_record(&mut record)?;
        if !has_record {
            return Ok(None);
        }

        let line = record.position().map(Position::line);
        let time_text = record.get(self.columns.time_idx).unwrap_or_default();
        let value_text = record.get(self.columns.value_idx).unwrap_or_default();
        if time_text.is_empty() != value_text.is_empty() {
            return Err(ScenarioError::HalfPopulatedPair {
                signal: self.signal.clone(),
                line,
            });
        }
        if time_text.is_empty() {
            return Ok(None);
        }

        let time = parse_time(time_text, &self.signal, line)?;
        let value = value_text.parse::<f64>()?;
        Ok(Some(Sample::new(time, value)?))
    }
}
