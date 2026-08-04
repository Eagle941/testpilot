//! Bounded-memory telemetry CSV creation and streaming serialization.

use std::fs::{File, OpenOptions};
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use csv::{StringRecord, Writer, WriterBuilder};

use crate::config::RecordingConfig;
pub use crate::error::RecordingError;

/// Streaming telemetry writer with one adjacent time/value pair per signal.
pub struct TelemetryRecorder {
    path: PathBuf,
    writer: Writer<File>,
    row_buffer: StringRecord,
}

impl TelemetryRecorder {
    /// Creates a timestamped telemetry file without overwriting an existing file.
    pub fn new(
        directory: impl AsRef<Path>,
        recordings: &[RecordingConfig],
        started_at: SystemTime,
    ) -> Result<TelemetryRecorder, RecordingError> {
        let path = timestamped_path(directory.as_ref(), started_at)?;
        let file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
            .map_err(|source| RecordingError::CreateFile {
                path: path.clone(),
                source,
            })?;
        let mut recorder = TelemetryRecorder {
            path,
            writer: WriterBuilder::new().has_headers(false).from_writer(file),
            row_buffer: StringRecord::new(),
        };
        recorder.write_header(recordings)?;
        recorder.flush()?;
        Ok(recorder)
    }

    /// Returns the generated telemetry path.
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Appends one frame using the same elapsed time for every configured value.
    pub fn write_frame(
        &mut self,
        elapsed: Duration,
        recordings: &[RecordingConfig],
        values: &[Option<f64>],
    ) -> Result<(), RecordingError> {
        if recordings.len() != values.len() {
            return Err(RecordingError::ValueCount {
                expected: recordings.len(),
                found: values.len(),
            });
        }

        self.row_buffer.clear();
        let time = elapsed.as_secs_f64().to_string();
        for (recording, value) in recordings.iter().zip(values) {
            match value {
                Some(value) => {
                    self.row_buffer.push_field(&time);
                    if !value.is_finite() {
                        return Err(RecordingError::NonFiniteValue {
                            signal: recording.name.clone(),
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
        self.writer
            .write_record(&self.row_buffer)
            .map_err(|source| RecordingError::WriteCsv {
                path: self.path.clone(),
                source,
            })
    }

    /// Flushes buffered telemetry to the output file.
    pub fn flush(&mut self) -> Result<(), RecordingError> {
        self.writer
            .flush()
            .map_err(|source| RecordingError::FlushFile {
                path: self.path.clone(),
                source,
            })
    }

    fn write_header(&mut self, recordings: &[RecordingConfig]) -> Result<(), RecordingError> {
        self.row_buffer.clear();
        for recording in recordings {
            self.row_buffer
                .push_field(&format!("{}.time", recording.name));
            self.row_buffer
                .push_field(&format!("{}.value", recording.name));
        }
        self.writer
            .write_record(&self.row_buffer)
            .map_err(|source| RecordingError::WriteCsv {
                path: self.path.clone(),
                source,
            })
    }
}

/// Builds `telemetry_YYYYMMDDTHHMMSS.csv` from a host UTC system time.
pub fn timestamped_path(
    directory: &Path,
    started_at: SystemTime,
) -> Result<PathBuf, RecordingError> {
    let elapsed = started_at
        .duration_since(UNIX_EPOCH)
        .map_err(|_| RecordingError::ClockBeforeUnixEpoch)?;
    let seconds = elapsed.as_secs();
    let days = i64::try_from(seconds / 86_400).map_err(|_| RecordingError::TimestampOutOfRange)?;
    let seconds_of_day = seconds % 86_400;
    let (year, month, day) = civil_date(days)?;
    let hour = seconds_of_day / 3_600;
    let minute = seconds_of_day % 3_600 / 60;
    let second = seconds_of_day % 60;
    let filename =
        format!("telemetry_{year:04}{month:02}{day:02}T{hour:02}{minute:02}{second:02}.csv");
    Ok(directory.join(filename))
}

/// Converts a signed count of days since 1970-01-01 into a UTC civil date.
///
/// The conversion follows Gregorian leap-year rules and supports four-digit
/// years from 0000 through 9999. Dates outside that range are rejected because
/// the telemetry filename reserves exactly four digits for the year.
fn civil_date(days_since_epoch: i64) -> Result<(i64, i64, i64), RecordingError> {
    let shifted = days_since_epoch
        .checked_add(719_468)
        .ok_or(RecordingError::TimestampOutOfRange)?;
    let era = shifted.div_euclid(146_097);
    let day_of_era = shifted.rem_euclid(146_097);
    let year_of_era =
        (day_of_era - day_of_era / 1_460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
    let mut year = year_of_era + era * 400;
    let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
    let month_prime = (5 * day_of_year + 2) / 153;
    let day = day_of_year - (153 * month_prime + 2) / 5 + 1;
    let month = month_prime + if month_prime < 10 { 3 } else { -9 };
    if month <= 2 {
        year += 1;
    }
    if !(0..=9_999).contains(&year) {
        return Err(RecordingError::TimestampOutOfRange);
    }
    Ok((year, month, day))
}

#[cfg(test)]
mod tests {
    use std::fs;

    use super::*;

    fn recording(name: &str) -> RecordingConfig {
        RecordingConfig {
            name: name.to_owned(),
            variable: format!("L:{name}"),
            unit: None,
            max_sampling_rate: None,
        }
    }

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
            timestamped_path(Path::new("output"), UNIX_EPOCH).unwrap(),
            Path::new("output/telemetry_19700101T000000.csv")
        );
        assert_eq!(
            timestamped_path(
                Path::new("output"),
                UNIX_EPOCH + Duration::from_secs(946_684_800)
            )
            .unwrap(),
            Path::new("output/telemetry_20000101T000000.csv")
        );
    }

    #[test]
    fn converts_epoch_and_pre_epoch_civil_dates() {
        assert_eq!(civil_date(0).unwrap(), (1970, 1, 1));
        assert_eq!(civil_date(-1).unwrap(), (1969, 12, 31));
    }

    #[test]
    fn converts_gregorian_leap_days() {
        assert_eq!(civil_date(11_016).unwrap(), (2000, 2, 29));
        assert_eq!(civil_date(19_782).unwrap(), (2024, 2, 29));
    }

    #[test]
    fn rejects_civil_dates_after_four_digit_years() {
        assert_eq!(civil_date(2_932_896).unwrap(), (9999, 12, 31));
        assert!(matches!(
            civil_date(2_932_897),
            Err(RecordingError::TimestampOutOfRange)
        ));
    }

    #[test]
    fn writes_paired_columns_and_rejects_overwrite() {
        let directory = fixture_directory();
        fs::create_dir_all(&directory).unwrap();
        let recordings = [recording("pitch"), recording("roll")];
        let started_at = UNIX_EPOCH + Duration::from_secs(946_684_800);
        let mut recorder = TelemetryRecorder::new(&directory, &recordings, started_at).unwrap();
        recorder
            .write_frame(
                Duration::from_millis(1_500),
                &recordings,
                &[Some(1.25), Some(-0.5)],
            )
            .unwrap();
        recorder.flush().unwrap();

        let contents = fs::read_to_string(recorder.path()).unwrap();
        assert_eq!(
            contents,
            "pitch.time,pitch.value,roll.time,roll.value\n1.5,1.25,1.5,-0.5\n"
        );
        assert!(matches!(
            TelemetryRecorder::new(&directory, &recordings, started_at),
            Err(RecordingError::CreateFile { .. })
        ));
        drop(recorder);
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn writes_blank_columns_when_values_are_not_sampled() {
        let directory = fixture_directory();
        fs::create_dir_all(&directory).unwrap();
        let recordings = [recording("pitch"), recording("roll")];
        let mut recorder = TelemetryRecorder::new(
            &directory,
            &recordings,
            UNIX_EPOCH + Duration::from_secs(42),
        )
        .unwrap();
        recorder
            .write_frame(
                Duration::from_secs_f64(0.0),
                &recordings,
                &[Some(1.0), Some(2.0)],
            )
            .unwrap();
        recorder
            .write_frame(
                Duration::from_secs_f64(0.5),
                &recordings,
                &[None, Some(3.0)],
            )
            .unwrap();
        recorder.flush().unwrap();

        let contents = fs::read_to_string(recorder.path()).unwrap();
        assert_eq!(
            contents,
            "pitch.time,pitch.value,roll.time,roll.value\n0,1,0,2\n,,0.5,3\n"
        );
        drop(recorder);
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn rejects_invalid_frames() {
        let directory = fixture_directory();
        fs::create_dir_all(&directory).unwrap();
        let recordings = [recording("pitch")];
        let mut recorder = TelemetryRecorder::new(&directory, &recordings, UNIX_EPOCH).unwrap();

        assert!(matches!(
            recorder.write_frame(Duration::ZERO, &recordings, &[]),
            Err(RecordingError::ValueCount { .. })
        ));
        assert!(matches!(
            recorder.write_frame(Duration::ZERO, &recordings, &[Some(f64::NAN)]),
            Err(RecordingError::NonFiniteValue { .. })
        ));
        drop(recorder);
        fs::remove_dir_all(directory).unwrap();
    }
}
