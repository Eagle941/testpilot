use std::fs::File;
use std::path::Path;

use anyhow::Context;

use crate::config::ReplayConfig;
use crate::playback::{AffineRange, LinearSegment, Sample};

use super::ScenarioError;
use super::validation::{ColumnPair, find_columns, parse_number};

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
    pub fn value_at(&self, elapsed_seconds: f64) -> anyhow::Result<f64> {
        match self.next {
            Some(next) => Ok(LinearSegment::new(self.previous, next)?.value_at(elapsed_seconds)?),
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

struct SignalCursor {
    signal: String,
    variable: String,
    time_column: String,
    value_column: String,
    columns: ColumnPair,
    reader: csv::Reader<File>,
    previous: Option<Sample>,
    next: Option<Sample>,
    conversion: AffineRange,
    ended: bool,
}

impl ScenarioPlayback {
    /// Opens the scenario independently for every configured injection.
    ///
    /// This reads the CSV header from each cursor but does not consume any data
    /// rows. The scenario file is never opened for writing.
    pub fn open(path: impl AsRef<Path>, config: &ReplayConfig) -> anyhow::Result<Self> {
        let path = path.as_ref();
        let mut cursors = Vec::with_capacity(config.inject.len());

        for injection in &config.inject {
            let file = File::open(path).with_context(|| {
                format!(
                    "failed to open scenario `{}` for signal `{}`",
                    path.display(),
                    injection.name
                )
            })?;
            let mut reader = csv::ReaderBuilder::new()
                .trim(csv::Trim::All)
                .from_reader(file);
            let headers = reader
                .headers()
                .map_err(ScenarioError::Csv)
                .with_context(|| {
                    format!(
                        "failed to read scenario header `{}` for signal `{}`",
                        path.display(),
                        injection.name
                    )
                })?
                .clone();
            let columns = find_columns(&headers, injection).with_context(|| {
                format!(
                    "invalid scenario header `{}` for signal `{}`",
                    path.display(),
                    injection.name
                )
            })?;

            cursors.push(SignalCursor {
                signal: injection.name.clone(),
                variable: injection.variable.clone(),
                time_column: injection.time_column.clone(),
                value_column: injection.value_column.clone(),
                columns,
                reader,
                previous: None,
                next: None,
                conversion: AffineRange::new(injection.source_range, injection.simulator_range)
                    .with_context(|| {
                        format!("invalid range conversion for signal `{}`", injection.name)
                    })?,
                ended: false,
            });
        }

        Ok(Self { cursors })
    }

    /// Advances every signal cursor for the current elapsed scenario time.
    ///
    /// Each cursor consumes at most one data row. Before playback starts,
    /// repeated calls with `0` prime the two interpolation rows. During
    /// playback, a cursor advances when `elapsed_seconds` passes its next row.
    pub fn next(&mut self, elapsed_seconds: f64) -> anyhow::Result<ScenarioStep<'_>> {
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

impl SignalCursor {
    fn next(&mut self, elapsed_seconds: f64) -> anyhow::Result<()> {
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

        let next = self
            .next
            .ok_or_else(|| ScenarioError::MissingNextInterpolationRow {
                signal: self.signal.clone(),
            })?;
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

    fn required_sample(&mut self) -> anyhow::Result<Sample> {
        self.read_sample()?
            .ok_or_else(|| ScenarioError::MissingSamples {
                signal: self.signal.clone(),
            })
            .map_err(Into::into)
    }

    fn read_sample(&mut self) -> anyhow::Result<Option<Sample>> {
        let mut record = csv::StringRecord::new();
        let has_record = self
            .reader
            .read_record(&mut record)
            .map_err(ScenarioError::Csv)
            .with_context(|| format!("failed to read scenario row for signal `{}`", self.signal))?;
        if !has_record {
            return Ok(None);
        }

        let line = record.position().map(csv::Position::line);
        let time_text = record.get(self.columns.time).unwrap_or_default();
        let value_text = record.get(self.columns.value).unwrap_or_default();
        if time_text.is_empty() != value_text.is_empty() {
            return Err(ScenarioError::HalfPopulatedPair {
                signal: self.signal.clone(),
                line,
            })
            .with_context(|| format!("failed to read signal `{}`", self.signal));
        }
        if time_text.is_empty() {
            return Ok(None);
        }

        let time = parse_number(time_text, &self.signal, &self.time_column, line)?;
        let value = parse_number(value_text, &self.signal, &self.value_column, line)?;
        Sample::new(time, value)
            .map(Some)
            .with_context(|| format!("invalid sample for signal `{}`", self.signal))
    }

    fn is_ready(&self) -> bool {
        self.previous.is_some() && (self.next.is_some() || self.ended)
    }
}
