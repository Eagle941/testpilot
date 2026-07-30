//! Incremental playback cursors and optional streaming validation for the
//! independently sampled scenario CSV format.
//!
//! Playback opens one read-only file cursor per injection and retains two
//! samples per signal, so memory use does not grow with scenario duration.

use std::collections::HashMap;
use std::fs::File;
use std::io::Read;
use std::path::Path;

use anyhow::Context;

use crate::config::{InjectionConfig, ReplayConfig};
use crate::playback::{AffineRange, LinearSegment, Sample};

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
    /// Latest final timestamp across all configured input series.
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

        let next = self.next.ok_or_else(|| {
            anyhow::anyhow!("signal `{}` has no next interpolation row", self.signal)
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
variable = "K:AXIS_ELEVATOR_SET"
time_column = "sidestick_pitch_position.time"
value_column = "sidestick_pitch_position.value"
source_range = [-100.0, 100.0]
simulator_range = [-1.0, 1.0]

[inject.1]
name = "sidestick_roll_position"
variable = "K:AXIS_AILERONS_SET"
time_column = "sidestick_roll_position.time"
value_column = "sidestick_roll_position.value"
source_range = [-100.0, 100.0]
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
    fn validates_default_unequal_length_scenario() {
        let summary = validate_scenario(include_bytes!("../scenario.csv").as_slice(), &config())
            .unwrap_or_else(|error| panic!("default scenario rejected: {error}"));

        assert_eq!(summary.duration_seconds, 40.0);
        assert_eq!(summary.signals[0].sample_count, 4);
        assert_eq!(summary.signals[0].final_time_seconds, 20.0);
        assert_eq!(summary.signals[1].sample_count, 5);
        assert_eq!(summary.signals[1].final_time_seconds, 40.0);
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
    fn rejects_missing_samples() {
        assert!(matches!(
            validate(",,0,0\n,,1,0\n"),
            Err(ScenarioError::MissingSamples { .. })
        ));
    }

    #[test]
    fn primes_independent_cursors_one_row_per_frame() {
        let path = std::env::temp_dir().join(format!(
            "replay-incremental-scenario-{}.csv",
            std::process::id()
        ));
        let contents = format!("{HEADER}0,1,0,2\n0.5,3,0.75,4\n1,5,1.5,6\n");
        if let Err(error) = std::fs::write(&path, contents) {
            panic!("failed to create scenario fixture: {error}");
        }

        let result = ScenarioPlayback::open(&path, &config());
        let mut playback = match result {
            Ok(playback) => playback,
            Err(error) => panic!("failed to open scenario fixture: {error:#}"),
        };
        assert_eq!(playback.signal_count(), 2);

        let step = playback
            .next(0.0)
            .unwrap_or_else(|error| panic!("first incremental read failed: {error:#}"));
        assert_eq!(step.progress(), ScenarioProgress::Loading);
        assert!(
            playback
                .cursors
                .iter()
                .all(|cursor| cursor.previous.is_some() && cursor.next.is_none())
        );

        let step = playback
            .next(0.0)
            .unwrap_or_else(|error| panic!("second incremental read failed: {error:#}"));
        assert_eq!(step.progress(), ScenarioProgress::Running);
        let rows = step.interpolation_rows().collect::<Vec<_>>();
        assert_eq!(rows.len(), 2);
        assert_eq!(rows[0].signal, "sidestick_pitch_position");
        assert_eq!(rows[0].variable, "K:AXIS_ELEVATOR_SET");
        assert_eq!(rows[0].previous, Sample::new(0.0, 1.0).unwrap());
        assert_eq!(rows[0].next, Some(Sample::new(0.5, 3.0).unwrap()));
        assert_eq!(rows[1].signal, "sidestick_roll_position");
        assert_eq!(rows[1].variable, "K:AXIS_AILERONS_SET");
        assert_eq!(rows[1].previous, Sample::new(0.0, 2.0).unwrap());
        assert_eq!(rows[1].next, Some(Sample::new(0.75, 4.0).unwrap()));

        let step = playback
            .next(0.0)
            .unwrap_or_else(|error| panic!("ready cursor read failed: {error:#}"));
        assert_eq!(step.progress(), ScenarioProgress::Running);

        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn advances_and_holds_unequal_length_series() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("scenario.csv");
        let mut playback = ScenarioPlayback::open(path, &config())
            .unwrap_or_else(|error| panic!("failed to open default scenario: {error:#}"));
        let step = playback
            .next(0.0)
            .unwrap_or_else(|error| panic!("first prime frame failed: {error:#}"));
        assert_eq!(step.progress(), ScenarioProgress::Loading);
        let step = playback
            .next(0.0)
            .unwrap_or_else(|error| panic!("second prime frame failed: {error:#}"));
        assert_eq!(step.progress(), ScenarioProgress::Running);

        let mut held_pitch = false;
        for frame in 0..=1200 {
            let elapsed_seconds = f64::from(frame) / 30.0;
            let step = playback
                .next(elapsed_seconds)
                .unwrap_or_else(|error| panic!("advance failed at {elapsed_seconds}: {error:#}"));
            assert_eq!(step.progress(), ScenarioProgress::Running);
            for rows in step.interpolation_rows() {
                assert!(rows.previous.time_seconds <= elapsed_seconds);
                match rows.next {
                    Some(next) => assert!(elapsed_seconds <= next.time_seconds),
                    None => {
                        assert_eq!(rows.value_at(elapsed_seconds).unwrap(), rows.previous.value);
                        if rows.signal == "sidestick_pitch_position" {
                            held_pitch = true;
                            assert_eq!(rows.previous.time_seconds, 20.0);
                            assert_eq!(rows.previous.value, 0.0);
                        }
                    }
                }
                rows.value_at(elapsed_seconds).unwrap_or_else(|error| {
                    panic!("interpolation failed at {elapsed_seconds}: {error}")
                });
            }
        }
        assert!(held_pitch);

        let step = playback
            .next(40.1)
            .unwrap_or_else(|error| panic!("completion advance failed: {error:#}"));
        assert_eq!(step.progress(), ScenarioProgress::Completed);
        assert_eq!(step.interpolation_rows().count(), 0);
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
