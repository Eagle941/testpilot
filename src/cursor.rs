use std::fs::File;
use std::path::Path;
use std::time::Duration;

use csv::{Position, Reader, ReaderBuilder, StringRecord, Trim};

use crate::config::{InjectionConfig, ReplayConfig};
use crate::playback::{AffineRange, LinearSegment, PlaybackError, Sample};

use crate::error::ScenarioError;

#[derive(Debug, Clone, Copy)]
/// Pair of CSV column indexes used for one configured input signal.
///
/// `time_idx` points to `<signal>.time` and `value_idx` points to the adjacent
/// `<signal>.value`.
pub struct ColumnPair {
    /// Zero-based column index of `<signal>.time`.
    pub time_idx: usize,
    /// Zero-based column index of `<signal>.value`.
    pub value_idx: usize,
}

/// Scenario data points used to calculate one injection value.
///
/// This is a frame-scoped, read-only view of one configured injection derived
/// from cursor state. It exposes only the data needed for interpolation and
/// simulator injection:
/// - logical input signal name,
/// - destination simulator variable,
/// - bracketing samples,
/// - and affine conversion configuration.
///
/// The iterator over these rows intentionally hides cursor internals (for example,
/// CSV column indexes and readers), so playback mechanics and file-state management
/// remain encapsulated and frame processing only depends on interpolation inputs.
///
/// If `next` is `Some`, `value_at` interpolates within the interval
/// `[previous.time, next.time]`; if `next` is `None`, the signal has reached EOF
/// and the last `previous.value` is held.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frame<'a> {
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

impl Frame<'_> {
    /// Computes this injection’s value for the requested frame time.
    ///
    /// Returns an interpolated value when a forward sample exists.
    /// If `next` is `None`, the cursor has reached EOF and the last `previous`
    /// value is held.
    ///
    /// The call is expected to target times within the current cursor bounds.
    /// Out-of-range times are converted into `PlaybackError::TimeOutsideSegment`.
    pub fn value_at(&self, elapsed: Duration) -> Result<f64, PlaybackError> {
        match self.next {
            Some(next) => LinearSegment::new(self.previous, next)?.value_at(elapsed),
            None => Ok(self.previous.value),
        }
    }
}

/// Incremental, read-only scenario loader with one file cursor per injection.
pub struct Scenario {
    /// Active per-signal cursor set.
    cursors: Vec<Cursor>,
}

impl Scenario {
    /// Opens the scenario independently for every configured injection.
    ///
    /// Initialization reads each CSV header and the first two samples needed
    /// for interpolation. The scenario file is never opened for writing.
    pub fn new(path: impl AsRef<Path>, config: &ReplayConfig) -> Result<Scenario, ScenarioError> {
        let path = path.as_ref();
        let cursors = config
            .inject
            .iter()
            .map(|injection| Cursor::new(path, injection))
            .collect::<Result<Vec<_>, _>>()?;

        Ok(Scenario { cursors })
    }

    /// Advances every signal cursor for the current elapsed scenario time.
    ///
    /// Each cursor reads forward until its samples bracket `elapsed`, or until
    /// it reaches the end of its series.
    pub fn advance(&mut self, elapsed: Duration) -> Result<(), ScenarioError> {
        for cursor in &mut self.cursors {
            cursor.advance(elapsed)?;
        }
        Ok(())
    }

    /// Returns whether every signal cursor has passed its final sample.
    pub fn completed(&self) -> bool {
        self.cursors.iter().all(|cursor| cursor.next.is_none())
    }

    /// Returns one bounding data-point pair per configured injection.
    pub fn interpolation_rows(&self) -> impl Iterator<Item = Frame<'_>> {
        self.cursors.iter().map(|cursor| Frame {
            signal: &cursor.signal,
            variable: &cursor.variable,
            previous: cursor.previous,
            next: cursor.next,
            conversion: cursor.conversion,
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
/// value until every configured cursor has completed.
pub struct Cursor {
    /// Logical input signal name from configuration.
    signal: String,
    /// Prefixed simulator destination for this injection.
    variable: String,
    /// Zero-based indexes of time/value CSV columns.
    columns: ColumnPair,
    /// Open reader for this signal's scenario stream.
    reader: Reader<File>,
    /// Lower bracket sample for interpolation.
    previous: Sample,
    /// Upper bracket sample, or `None` after EOF.
    next: Option<Sample>,
    /// Per-signal affine conversion from source to simulator units.
    conversion: AffineRange,
}

impl Cursor {
    /// Builds a cursor for one configured injection signal.
    ///
    /// The cursor owns its own CSV reader and keeps only the two samples needed
    /// for the current interpolation interval (`previous` and `next`).
    /// It reads and validates the first pair of samples during construction so
    /// playback can fail fast on empty or malformed columns.
    fn new(path: &Path, injection: &InjectionConfig) -> Result<Cursor, ScenarioError> {
        let mut reader = ReaderBuilder::new().trim(Trim::All).from_path(path)?;
        let columns = Cursor::find_column_indices(reader.headers()?, injection)?;
        let conversion = AffineRange::new(injection.source_range, injection.simulator_range)?;
        let mut cursor = Cursor {
            signal: injection.name.clone(),
            variable: injection.variable.clone(),
            columns,
            reader,
            previous: Sample::new(Duration::ZERO, 0.0)?,
            next: None,
            conversion,
        };
        cursor.previous = cursor.required_sample()?;
        cursor.next = Some(cursor.required_sample()?);
        Ok(cursor)
    }

    /// Parses scenario-relative seconds into a duration.
    pub fn parse_time(
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
    /// The returned [`ColumnPair`] contains the zero-based indexes of the derived
    /// `<signal>.time` and `<signal>.value` columns. The time column must
    /// immediately precede its matching value column.
    pub fn find_column_indices(
        headers: &StringRecord,
        injection: &InjectionConfig,
    ) -> Result<ColumnPair, ScenarioError> {
        let time_column = format!("{}.time", injection.name);
        let value_column = format!("{}.value", injection.name);
        let time_idx = headers
            .iter()
            .position(|header| header == time_column)
            .ok_or_else(|| ScenarioError::MissingColumn {
                signal: injection.name.clone(),
                column: time_column.clone(),
            })?;
        let value_idx = headers
            .iter()
            .position(|header| header == value_column)
            .ok_or_else(|| ScenarioError::MissingColumn {
                signal: injection.name.clone(),
                column: value_column.clone(),
            })?;

        if time_idx.checked_add(1) != Some(value_idx) {
            return Err(ScenarioError::NonAdjacentColumns {
                signal: injection.name.clone(),
                time_column,
                value_column,
            });
        }

        Ok(ColumnPair {
            time_idx,
            value_idx,
        })
    }

    /// Advances this cursor to bracket the provided elapsed scenario time.
    ///
    /// When frames are delayed, multiple rows may be consumed so that
    /// `previous.time <= elapsed <= next.time` (or `next` becomes `None` at EOF).
    fn advance(&mut self, elapsed: Duration) -> Result<(), ScenarioError> {
        while let Some(next) = self.next {
            if elapsed <= next.time {
                break;
            }
            self.previous = next;
            self.next = self.read_sample()?;
        }
        Ok(())
    }

    /// Reads the next sample and requires it to exist.
    ///
    /// This is used during cursor initialization to guarantee each signal has at
    /// least one concrete sample pair before playback starts.
    fn required_sample(&mut self) -> Result<Sample, ScenarioError> {
        self.read_sample()?
            .ok_or_else(|| ScenarioError::MissingSamples {
                signal: self.signal.clone(),
            })
    }

    /// Reads one scenario row and returns the next concrete sample, if any.
    ///
    /// Empty cells in both time/value columns are treated as end-of-stream.
    /// If only one side of the pair is populated, parsing fails with
    /// [`ScenarioError::HalfPopulatedPair`].
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

        let time = Cursor::parse_time(time_text, &self.signal, line)?;
        let value = value_text.parse::<f64>()?;
        Ok(Some(Sample::new(time, value)?))
    }
}
