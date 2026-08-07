//! Bounded-memory telemetry CSV creation and streaming serialization.

use std::fs::{File, OpenOptions};
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};

use csv::{StringRecord, Writer, WriterBuilder};
use time::OffsetDateTime;

pub use crate::error::RecordingError;

/// Streaming telemetry writer with one adjacent time/value pair per signal.
pub struct TelemetryRecorder {
    /// Absolute path to the output CSV file.
    filename: PathBuf,
    /// CSV writer bound to the telemetry file.
    writer: Writer<File>,
    /// Reused row buffer to avoid per-frame allocations.
    row_buffer: StringRecord,
    /// Recording column labels in deterministic output order.
    recording_signal_names: Vec<String>,
    /// Injection column labels in deterministic output order.
    injected_signal_names: Vec<String>,
}

impl TelemetryRecorder {
    /// Creates a timestamped telemetry file without overwriting an existing file.
    pub fn new(
        directory: impl AsRef<Path>,
        recording_names: &[String],
        injected_names: &[String],
        started_at: SystemTime,
    ) -> Result<TelemetryRecorder, RecordingError> {
        let path = TelemetryRecorder::path_for(directory, started_at);
        let file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
            .map_err(|source| RecordingError::CreateFile {
                path: path.clone(),
                source,
            })?;
        let mut recorder = TelemetryRecorder {
            filename: path,
            writer: WriterBuilder::new().has_headers(false).from_writer(file),
            row_buffer: StringRecord::new(),
            recording_signal_names: recording_names.to_vec(),
            injected_signal_names: injected_names.to_vec(),
        };
        recorder.write_header()?;
        recorder.flush()?;
        Ok(recorder)
    }

    /// Returns the generated telemetry path.
    pub fn path(&self) -> &Path {
        &self.filename
    }

    /// Computes the telemetry path from the UTC run start time and target directory.
    fn path_for(directory: impl AsRef<Path>, started_at: SystemTime) -> PathBuf {
        // `SystemTime` values for telemetry filenames come from `SystemTime::now()` in normal
        // execution, so they are expected to be well within supported UTC timestamp ranges.
        // Keeping the conversion infallible here is acceptable for this low-likelihood edge case.
        let started_at: OffsetDateTime = started_at.into();
        let filename = format!(
            "telemetry_{:0>4}{:0>2}{:0>2}T{:0>2}{:0>2}{:0>2}.csv",
            started_at.year(),
            started_at.month() as u8,
            started_at.day(),
            started_at.hour(),
            started_at.minute(),
            started_at.second()
        );
        directory.as_ref().join(filename)
    }

    /// Appends one frame using the same elapsed time for every configured value.
    pub fn write_frame(
        &mut self,
        elapsed: Duration,
        recording_values: &[Option<f64>],
        injected_values: &[f64],
    ) -> Result<(), RecordingError> {
        let expected_count = self.recording_signal_names.len() + self.injected_signal_names.len();
        if recording_values.len() != self.recording_signal_names.len()
            || injected_values.len() != self.injected_signal_names.len()
        {
            return Err(RecordingError::ValueCount {
                expected: expected_count,
                found: recording_values.len() + injected_values.len(),
            });
        }

        self.row_buffer.clear();
        let time = elapsed.as_secs_f64().to_string();

        for (recording_name, value) in self.recording_signal_names.iter().zip(recording_values) {
            match value {
                Some(value) => {
                    self.row_buffer.push_field(&time);
                    if !value.is_finite() {
                        return Err(RecordingError::NonFiniteValue {
                            signal: recording_name.clone(),
                            value: *value,
                        });
                    }
                    self.row_buffer.push_field(&value.to_string());
                }
                None => {
                    self.row_buffer.push_field("");
                    self.row_buffer.push_field("");
                }
            }
        }
        for (injected_name, value) in self.injected_signal_names.iter().zip(injected_values) {
            self.row_buffer.push_field(&time);
            if !value.is_finite() {
                return Err(RecordingError::NonFiniteValue {
                    signal: injected_name.clone(),
                    value: *value,
                });
            }
            self.row_buffer.push_field(&value.to_string());
        }
        self.writer
            .write_record(&self.row_buffer)
            .map_err(|source| RecordingError::WriteCsv {
                path: self.filename.clone(),
                source,
            })
    }

    /// Flushes buffered telemetry to the output file.
    pub fn flush(&mut self) -> Result<(), RecordingError> {
        self.writer
            .flush()
            .map_err(|source| RecordingError::FlushFile {
                path: self.filename.clone(),
                source,
            })
    }

    /// Writes the paired `<signal>.time`/`<signal>.value` CSV header columns.
    fn write_header(&mut self) -> Result<(), RecordingError> {
        self.row_buffer.clear();
        for name in &self.recording_signal_names {
            self.row_buffer.push_field(&format!("{}.time", name));
            self.row_buffer.push_field(&format!("{}.value", name));
        }
        for name in &self.injected_signal_names {
            self.row_buffer.push_field(&format!("{}.time", name));
            self.row_buffer.push_field(&format!("{}.value", name));
        }
        self.writer
            .write_record(&self.row_buffer)
            .map_err(|source| RecordingError::WriteCsv {
                path: self.filename.clone(),
                source,
            })
    }
}

#[cfg(test)]
mod tests {
    use std::fs;
    use std::time::UNIX_EPOCH;

    use super::*;

    fn fixture_directory() -> PathBuf {
        std::env::temp_dir().join(format!(
            "testpilot-recording-{}-{:?}",
            std::process::id(),
            std::thread::current().id()
        ))
    }

    #[test]
    fn formats_host_utc_timestamped_paths() {
        assert_eq!(
            TelemetryRecorder::path_for(Path::new("output"), UNIX_EPOCH),
            Path::new("output/telemetry_19700101T000000.csv")
        );
        assert_eq!(
            TelemetryRecorder::path_for(
                Path::new("output"),
                UNIX_EPOCH + Duration::from_secs(946_684_800)
            ),
            Path::new("output/telemetry_20000101T000000.csv")
        );
    }

    #[test]
    fn formats_gregorian_leap_dates() {
        let feb_2000 = Duration::from_secs(11_016 * 86_400);
        assert_eq!(
            TelemetryRecorder::path_for(Path::new("output"), UNIX_EPOCH + feb_2000),
            Path::new("output/telemetry_20000229T000000.csv")
        );

        let feb_2024 = Duration::from_secs(19_782 * 86_400);
        assert_eq!(
            TelemetryRecorder::path_for(Path::new("output"), UNIX_EPOCH + feb_2024),
            Path::new("output/telemetry_20240229T000000.csv")
        );
    }

    #[test]
    fn writes_paired_columns_and_rejects_overwrite() {
        let directory = fixture_directory();
        fs::create_dir_all(&directory).unwrap();
        let recording_names = vec!["pitch".to_owned(), "roll".to_owned()];
        let injected_names = vec!["sidestick_pitch_position".to_owned()];
        let started_at = UNIX_EPOCH + Duration::from_secs(946_684_800);
        let mut recorder =
            TelemetryRecorder::new(&directory, &recording_names, &injected_names, started_at)
                .unwrap();
        recorder
            .write_frame(
                Duration::from_millis(1_500),
                &[Some(1.25), Some(-0.5)],
                &[1.0],
            )
            .unwrap();
        recorder.flush().unwrap();

        let contents = fs::read_to_string(recorder.path()).unwrap();
        assert_eq!(
            contents,
            "pitch.time,pitch.value,roll.time,roll.value,sidestick_pitch_position.time,sidestick_pitch_position.value\n\
             1.5,1.25,1.5,-0.5,1.5,1\n"
        );

        match TelemetryRecorder::new(&directory, &recording_names, &injected_names, started_at) {
            Err(RecordingError::CreateFile { .. }) => {}
            _ => panic!("expected create-file failure"),
        }
        drop(recorder);
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn writes_blank_columns_when_values_are_not_sampled() {
        let directory = fixture_directory();
        fs::create_dir_all(&directory).unwrap();
        let recording_names = vec!["pitch".to_owned(), "roll".to_owned()];
        let injected_names = vec!["sidestick_roll_position".to_owned()];
        let mut recorder = TelemetryRecorder::new(
            &directory,
            &recording_names,
            &injected_names,
            UNIX_EPOCH + Duration::from_secs(42),
        )
        .unwrap();
        recorder
            .write_frame(
                Duration::from_secs_f64(0.0),
                &[Some(1.0), Some(2.0)],
                &[0.25],
            )
            .unwrap();
        recorder
            .write_frame(Duration::from_secs_f64(0.5), &[None, Some(3.0)], &[0.5])
            .unwrap();
        recorder.flush().unwrap();

        let contents = fs::read_to_string(recorder.path()).unwrap();
        assert_eq!(
            contents,
            "pitch.time,pitch.value,roll.time,roll.value,sidestick_roll_position.time,sidestick_roll_position.value\n\
             0,1,0,2,0,0.25\n\
             ,,0.5,3,0.5,0.5\n"
        );
        drop(recorder);
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn rejects_invalid_frames() {
        let directory = fixture_directory();
        fs::create_dir_all(&directory).unwrap();
        let recording_names = vec!["pitch".to_owned()];
        let injected_names = vec!["sidestick_pitch_position".to_owned()];
        let mut recorder =
            TelemetryRecorder::new(&directory, &recording_names, &injected_names, UNIX_EPOCH)
                .unwrap();

        match recorder.write_frame(Duration::ZERO, &[Some(0.0)], &[]) {
            Err(RecordingError::ValueCount { .. }) => {}
            unexpected => panic!("expected frame value count failure, got: {unexpected:?}"),
        }
        match recorder.write_frame(Duration::ZERO, &[Some(f64::NAN)], &[0.0]) {
            Err(RecordingError::NonFiniteValue { .. }) => {}
            unexpected => panic!("expected non-finite value failure, got: {unexpected:?}"),
        }
        drop(recorder);
        fs::remove_dir_all(directory).unwrap();
    }
}
