use std::fs::File;
use std::path::Path;

use csv::{Position, Reader, ReaderBuilder, StringRecord, Trim};

use crate::config::{InjectionConfig, ReplayConfig};
use crate::playback::{AffineRange, LinearSegment, PlaybackError, Sample};

use super::ScenarioError;
use super::validation::{ColumnPair, find_column_indices};

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
    pub fn value_at(&self, elapsed_seconds: f64) -> Result<f64, PlaybackError> {
        match self.next {
            Some(next) => LinearSegment::new(self.previous, next)?.value_at(elapsed_seconds),
            None => Ok(self.previous.value),
        }
    }
}

/// Progress reported by one incremental scenario update.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScenarioProgress {
    /// At least one cursor does not yet have two samples.
    Loading,
    /// Every cursor can interpolate or hold its final sample.
    Running,
    /// Every cursor has advanced beyond its final row.
    Completed,
}

/// Result of advancing every scenario cursor for one simulator frame.
pub struct ScenarioStep<'a> {
    progress: ScenarioProgress,
    cursors: &'a [SignalCursor],
}

impl ScenarioStep<'_> {
    /// Returns the aggregate cursor progress for this frame.
    pub const fn progress(&self) -> ScenarioProgress {
        self.progress
    }

    /// Returns the rows available for interpolation on this frame.
    pub fn interpolation_rows(&self) -> impl Iterator<Item = InterpolationRows<'_>> {
        self.cursors.iter().filter_map(|cursor| {
            if self.progress == ScenarioProgress::Completed {
                return None;
            }
            Some(InterpolationRows {
                signal: &cursor.signal,
                variable: &cursor.variable,
                previous: cursor.previous?,
                next: cursor.next,
                conversion: cursor.conversion,
            })
        })
    }
}

/// Incremental, read-only scenario loader with one file cursor per injection.
pub struct ScenarioPlayback {
    cursors: Vec<SignalCursor>,
}

impl ScenarioPlayback {
    /// Opens the scenario independently for every configured injection.
    ///
    /// This reads the CSV header from each cursor but does not consume any data
    /// rows. The scenario file is never opened for writing.
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
    /// Each cursor consumes at most one data row. Before playback starts,
    /// repeated calls with `0` prime the two interpolation rows. During
    /// playback, a cursor advances when `elapsed_seconds` passes its next row.
    pub fn next(&mut self, elapsed_seconds: f64) -> Result<ScenarioStep<'_>, ScenarioError> {
        for cursor in &mut self.cursors {
            cursor.next(elapsed_seconds)?;
        }

        let progress = if self.cursors.iter().all(|cursor| cursor.ended) {
            ScenarioProgress::Completed
        } else if self.cursors.iter().all(SignalCursor::is_ready) {
            ScenarioProgress::Running
        } else {
            ScenarioProgress::Loading
        };
        Ok(ScenarioStep {
            progress,
            cursors: &self.cursors,
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

        Ok(SignalCursor {
            signal: injection.name.clone(),
            variable: injection.variable.clone(),
            columns,
            reader,
            previous: None,
            next: None,
            conversion,
            ended: false,
        })
    }

    fn next(&mut self, elapsed_seconds: f64) -> Result<(), ScenarioError> {
        if self.ended {
            return Ok(());
        }
        if self.previous.is_none() {
            self.previous = Some(self.required_sample()?);
            return Ok(());
        }
        if self.next.is_none() {
            self.next = Some(self.required_sample()?);
            return Ok(());
        }

        let Some(next) = self.next else {
            return Ok(());
        };
        if elapsed_seconds <= next.time_seconds {
            return Ok(());
        }

        self.previous = Some(next);
        match self.read_sample()? {
            Some(sample) => self.next = Some(sample),
            None => {
                self.next = None;
                self.ended = true;
            }
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

        let time = time_text.parse::<f64>()?;
        let value = value_text.parse::<f64>()?;
        Ok(Some(Sample::new(time, value)?))
    }

    fn is_ready(&self) -> bool {
        self.previous.is_some() && (self.next.is_some() || self.ended)
    }
}
