use std::fs::File;
use std::path::{Path, PathBuf};
use std::time::Duration;

use csv::{Position, Reader, ReaderBuilder, StringRecord, Trim};

use crate::config::{Config, InjectionConfig};
use crate::playback::{AffineRange, LinearSegment, PlaybackError, Sample};

use crate::error::{InterpolationError, ScenarioError};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

/// One interpolated input in source units, with its configured simulator destination.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReplayInput<'a> {
    /// Logical signal name used in diagnostics.
    pub signal: &'a str,
    /// Prefixed simulator destination from trusted configuration.
    pub variable: &'a str,
    /// Interpolated value before affine conversion.
    pub value: f64,
    /// Conversion applied at the simulator I/O boundary.
    pub conversion: AffineRange,
}

/// Read-only input values for a single scenario timestamp.
#[derive(Debug, Clone, Copy)]
pub struct ReplayInputs<'a> {
    /// Cursors already advanced to bracket this frame.
    scenario: &'a Scenario,
    /// Scenario-relative simulator elapsed time.
    elapsed: Duration,
}

impl ReplayInputs<'_> {
    /// Interpolates inputs in configuration order, without allocating a frame buffer.
    ///
    /// Values are evaluated as consumed so an input failure retains the established
    /// injection order. No telemetry or scheduling state is borrowed by this view.
    pub fn iter(&self) -> impl Iterator<Item = Result<ReplayInput<'_>, InterpolationError>> {
        self.scenario.interpolation_rows().map(|points| {
            let value = points.value_at(self.elapsed).map_err(|source| {
                InterpolationError::InterpolateSignal {
                    signal: points.signal.to_owned(),
                    source,
                }
            })?;
            Ok(ReplayInput {
                signal: points.signal,
                variable: points.variable,
                value,
                conversion: points.conversion,
            })
        })
    }
}

/// Streams and interpolates one scenario through independent, read-only signal cursors.
#[derive(Debug)]
pub struct Scenario {
    /// Active per-signal cursor set.
    cursors: Vec<Cursor>,
}

impl Scenario {
    /// Opens the scenario independently for every configured injection.
    ///
    /// Initialization reads each CSV header and the first two samples needed
    /// for interpolation. The scenario file is never opened for writing.
    pub fn new(path: impl AsRef<Path>, config: &Config) -> Result<Scenario, ScenarioError> {
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
    /// it reaches the end of its series. Returns a borrowed input view for this
    /// frame, or `None` when every signal has passed its final sample.
    pub fn advance(
        &mut self,
        elapsed: Duration,
    ) -> Result<Option<ReplayInputs<'_>>, ScenarioError> {
        for cursor in &mut self.cursors {
            cursor.advance(elapsed)?;
        }
        Ok((!self.completed()).then_some(ReplayInputs {
            scenario: self,
            elapsed,
        }))
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
#[derive(Debug)]
pub struct Cursor {
    /// Scenario filename retained for streaming error diagnostics.
    path: PathBuf,
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
    /// Reusable CSV record buffer for the currently consumed row.
    row: StringRecord,
}

impl Cursor {
    /// Builds a cursor for one configured injection signal.
    ///
    /// The cursor owns its own CSV reader and keeps only the two samples needed
    /// for the current interpolation interval (`previous` and `next`).
    /// It reads and validates the first pair of samples during construction so
    /// playback can fail fast on empty or malformed columns.
    fn new(path: &Path, injection: &InjectionConfig) -> Result<Cursor, ScenarioError> {
        let csv_error = |operation, source| ScenarioError::Csv {
            path: path.to_path_buf(),
            signal: injection.name.clone(),
            operation,
            source,
        };
        let mut reader = ReaderBuilder::new()
            .trim(Trim::All)
            .from_path(path)
            .map_err(|source| csv_error("open", source))?;
        let headers = reader
            .headers()
            .map_err(|source| csv_error("read header of", source))?;
        let columns = Cursor::find_column_indices(headers, injection)?;
        let conversion = AffineRange::new(injection.source_range, injection.simulator_range)?;
        let mut cursor = Cursor {
            path: path.to_path_buf(),
            signal: injection.name.clone(),
            variable: injection.variable.clone(),
            columns,
            reader,
            previous: Sample::new(Duration::ZERO, 0.0)?,
            next: None,
            conversion,
            row: StringRecord::new(),
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
        Self::parse_time_in_file(text, signal, line, Path::new("<scenario>"))
    }

    /// Parses a timestamp with file context when called by a streaming cursor.
    fn parse_time_in_file(
        text: &str,
        signal: &str,
        line: Option<u64>,
        path: &Path,
    ) -> Result<Duration, ScenarioError> {
        let time_seconds = text
            .parse::<f64>()
            .map_err(|source| ScenarioError::ParseNumber {
                path: path.to_path_buf(),
                signal: signal.to_owned(),
                column: "time",
                line,
                source,
            })?;
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
        self.row.clear();
        let has_record =
            self.reader
                .read_record(&mut self.row)
                .map_err(|source| ScenarioError::Csv {
                    path: self.path.clone(),
                    signal: self.signal.clone(),
                    operation: "read row of",
                    source,
                })?;
        if !has_record {
            return Ok(None);
        }

        let line = self.row.position().map(Position::line);
        let time_text = self.row.get(self.columns.time_idx).unwrap_or_default();
        let value_text = self.row.get(self.columns.value_idx).unwrap_or_default();
        if time_text.is_empty() != value_text.is_empty() {
            return Err(ScenarioError::HalfPopulatedPair {
                signal: self.signal.clone(),
                line,
            });
        }
        if time_text.is_empty() {
            return Ok(None);
        }

        let time = Cursor::parse_time_in_file(time_text, &self.signal, line, &self.path)?;
        let value = value_text
            .parse::<f64>()
            .map_err(|source| ScenarioError::ParseNumber {
                path: self.path.clone(),
                signal: self.signal.clone(),
                column: "value",
                line,
                source,
            })?;
        let sample = Sample::new(time, value).map_err(|source| ScenarioError::InvalidSample {
            path: self.path.clone(),
            signal: self.signal.clone(),
            line,
            source,
        })?;
        Ok(Some(sample))
    }
}

#[cfg(test)]
mod tests {
    use std::fs;
    use std::path::PathBuf;
    use std::time::Duration;

    use super::Scenario;
    use crate::config::Config;

    fn time(seconds: f64) -> Duration {
        Duration::try_from_secs_f64(seconds).unwrap()
    }

    #[derive(Debug)]
    struct Fixture {
        directory: PathBuf,
        config_path: PathBuf,
    }

    impl Fixture {
        fn new() -> Fixture {
            let directory = std::env::temp_dir().join(format!(
                "replay-gauge-{}-{:?}",
                std::process::id(),
                std::thread::current().id()
            ));
            fs::create_dir_all(&directory)
                .unwrap_or_else(|error| panic!("failed to create fixture directory: {error}"));
            let config_path = directory.join("replayer_config.toml");
            fs::write(
                &config_path,
                r#"format_version = 1
input_file = "scenario.csv"

[inject.0]
name = "sidestick_pitch_position"
variable = "K:AXIS_ELEVATOR_SET"
source_range = [-100.0, 100.0]
simulator_range = [-1.0, 1.0]

[record.0]
name = "pitch"
variable = "A:PLANE PITCH DEGREES"
unit = "radians"
"#,
            )
            .unwrap_or_else(|error| panic!("failed to write fixture config: {error}"));
            fs::write(
                directory.join("scenario.csv"),
                "sidestick_pitch_position.time,sidestick_pitch_position.value\n0,0\n0.1,10\n",
            )
            .unwrap_or_else(|error| panic!("failed to write fixture scenario: {error}"));

            Fixture {
                directory,
                config_path,
            }
        }
    }

    impl Drop for Fixture {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.directory);
        }
    }

    #[test]
    fn preparation_opens_inputs_without_output_or_a_clock() {
        let fixture = Fixture::new();
        let config = Config::read_config_file(&fixture.config_path).unwrap();
        let mut scenario =
            Scenario::new(fixture.directory.join(&config.input_file), &config).unwrap();
        assert_eq!(fs::read_dir(&fixture.directory).unwrap().count(), 2);
        let inputs = scenario.advance(Duration::ZERO).unwrap().unwrap();
        let input = inputs.iter().next().unwrap().unwrap();
        assert_eq!(input.signal, "sidestick_pitch_position");
        assert_eq!(input.variable, "K:AXIS_ELEVATOR_SET");
        assert_eq!(input.value, 0.0);
        assert!((input.conversion.convert(10.0).unwrap() - 0.1).abs() < 1e-12);
    }

    #[test]
    fn interpolates_irregular_series_and_catches_up_across_multiple_intervals() {
        let fixture = Fixture::new();
        let config = Config::read_config_file(&fixture.config_path).unwrap();
        let path = fixture.directory.join(&config.input_file);
        fs::write(&path, "sidestick_pitch_position.time,sidestick_pitch_position.value\n0,0\n0.2,20\n0.5,50\n2,80\n").unwrap();
        let mut scenario = Scenario::new(&path, &config).unwrap();
        for (elapsed, expected) in [
            (0.0, 0.0),
            (0.1, 10.0),
            (0.2, 20.0),
            (1.25, 65.0),
            (2.0, 80.0),
        ] {
            let inputs = scenario.advance(time(elapsed)).unwrap().unwrap();
            assert_eq!(inputs.iter().next().unwrap().unwrap().value, expected);
        }
        assert!(scenario.advance(time(2.1)).unwrap().is_none());
        assert!(scenario.advance(time(3.0)).unwrap().is_none());
        assert_eq!(fs::read_dir(&fixture.directory).unwrap().count(), 2);
    }

    #[test]
    fn holds_shorter_series_until_the_longest_series_finishes() {
        let fixture = Fixture::new();
        let mut config = Config::read_config_file(&fixture.config_path).unwrap();
        let mut second = config.inject[0].clone();
        second.name = "roll".to_owned();
        second.variable = "K:AXIS_AILERONS_SET".to_owned();
        config.inject.push(second);
        let path = fixture.directory.join(&config.input_file);
        fs::write(&path, "sidestick_pitch_position.time,sidestick_pitch_position.value,roll.time,roll.value\n0,0,0,0\n0.1,10,1,100\n").unwrap();
        let mut scenario = Scenario::new(&path, &config).unwrap();
        let inputs = scenario.advance(time(0.5)).unwrap().unwrap();
        let values: Vec<_> = inputs.iter().map(|input| input.unwrap().value).collect();
        assert_eq!(values, [10.0, 50.0]);
        assert!(scenario.advance(time(1.1)).unwrap().is_none());
    }

    #[test]
    fn streaming_parse_failures_retain_location_and_signal() {
        let fixture = Fixture::new();
        let config = Config::read_config_file(&fixture.config_path).unwrap();
        let path = fixture.directory.join(&config.input_file);
        fs::write(
            &path,
            "sidestick_pitch_position.time,sidestick_pitch_position.value\n0,0\n1,10\n2,invalid\n",
        )
        .unwrap();
        let mut scenario = Scenario::new(&path, &config).unwrap();
        let error = scenario.advance(time(1.5)).err().unwrap();
        let message = error.to_string();
        assert!(message.contains("scenario.csv"));
        assert!(message.contains("sidestick_pitch_position"));
        assert!(message.contains("line 4"));
        assert!(message.contains("value"));
    }
}
