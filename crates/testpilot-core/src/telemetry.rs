//! Recording interface validation, sampling schedules and incremental telemetry output.

use std::path::Path;
use std::time::{Duration, SystemTime};

use crate::config::RecordingConfig;
use crate::error::{RecordingError, TelemetryError};
use crate::recording::TelemetryRecorder;
use crate::simulator::SimulatorAdapter;

/// Owns one run's recording definitions, bounded sampling state and CSV output.
#[derive(Debug)]
pub(crate) struct Telemetry {
    /// Recording sources in numeric configuration order.
    recordings: Vec<RecordingConfig>,
    /// Per-signal sampling deadlines in scenario-relative time.
    schedules: Vec<RecordingSchedule>,
    /// Reused sample slots; absent values represent signals not due on this frame.
    values: Vec<Option<f64>>,
    /// Incremental file writer, created when the runtime enters its running phase.
    recorder: TelemetryRecorder,
}

impl Telemetry {
    /// Creates output and fixed-size buffers from validated recording definitions.
    pub(crate) fn new(
        directory: &Path,
        recordings: Vec<RecordingConfig>,
        injected_names: &[String],
        started_at: SystemTime,
    ) -> Result<Self, RecordingError> {
        let recording_names: Vec<_> = recordings
            .iter()
            .map(|recording| recording.name.clone())
            .collect();
        let recorder =
            TelemetryRecorder::new(directory, &recording_names, injected_names, started_at)?;
        let schedules = recordings
            .iter()
            .map(|recording| RecordingSchedule::new(recording.max_sampling_rate))
            .collect();
        let values = vec![None; recordings.len()];
        Ok(Self {
            recordings,
            schedules,
            values,
            recorder,
        })
    }

    /// Validates sources once after the runtime has installed the running context.
    pub(crate) fn validate(
        &self,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<(), TelemetryError> {
        for recording in &self.recordings {
            simulator
                .validate_read(&recording.variable, recording.unit.as_deref())
                .map_err(|source| TelemetryError::ValidateRecordingSignal {
                    signal: recording.name.clone(),
                    source,
                })?;
        }
        Ok(())
    }

    /// Samples due outputs after injection, preserving the existing sparse-row policy.
    pub(crate) fn record_frame(
        &mut self,
        elapsed: Duration,
        injected_values: &[f64],
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<(), TelemetryError> {
        self.values.fill(None);
        let mut any_due = false;
        for ((recording, schedule), slot) in self
            .recordings
            .iter()
            .zip(&mut self.schedules)
            .zip(&mut self.values)
        {
            if !schedule.should_sample(elapsed) {
                continue;
            }
            any_due = true;
            *slot = Some(
                simulator
                    .read(&recording.variable, recording.unit.as_deref())
                    .map_err(|source| TelemetryError::SampleSignal {
                        signal: recording.name.clone(),
                        source,
                    })?,
            );
        }
        if any_due || self.recordings.is_empty() {
            self.recorder
                .write_frame(elapsed, &self.values, injected_values)?;
        }
        Ok(())
    }

    /// Output path used by runtime diagnostics.
    pub(crate) fn path(&self) -> &Path {
        self.recorder.path()
    }

    /// Flushes the current file before the owning context is released.
    pub(crate) fn flush(&mut self) -> Result<(), RecordingError> {
        self.recorder.flush()
    }

    #[cfg(test)]
    /// Makes the real output unwritable so lifecycle tests can exercise flush failures.
    pub(crate) fn make_output_read_only(&mut self) {
        self.recorder.make_output_read_only();
    }
}

/// Controls when each recording signal is sampled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RecordingSchedule {
    /// Sample on every frame.
    EveryFrame,
    /// Sample no more often than the configured interval.
    Limited {
        /// Minimum interval between samples.
        period: Duration,
        /// Next due scenario-relative timestamp.
        next_due: Duration,
    },
}

impl RecordingSchedule {
    /// Builds the existing maximum-rate policy from validated configuration.
    fn new(max_sampling_rate: Option<f64>) -> Self {
        match max_sampling_rate {
            Some(rate) => Self::Limited {
                period: Duration::from_secs_f64(1.0 / rate),
                next_due: Duration::ZERO,
            },
            None => Self::EveryFrame,
        }
    }

    /// Advances the sampling deadline only when this frame is due.
    fn should_sample(&mut self, elapsed: Duration) -> bool {
        match self {
            Self::EveryFrame => true,
            Self::Limited { period, next_due } => {
                if elapsed < *next_due {
                    return false;
                }
                *next_due = elapsed.saturating_add(*period);
                true
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::{HashMap, VecDeque};
    use std::fs;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::time::UNIX_EPOCH;

    use crate::error::SimulatorError;

    use super::*;

    #[test]
    fn schedules_sample_rate_limits_are_advanced_with_frame_elapsed_time() {
        let mut schedule = super::RecordingSchedule::new(Some(1.0));

        assert!(schedule.should_sample(Duration::ZERO));
        assert!(!schedule.should_sample(Duration::from_millis(400)));
        assert!(schedule.should_sample(Duration::from_secs(1)));
        assert!(!schedule.should_sample(Duration::from_millis(1_600)));
        assert!(schedule.should_sample(Duration::from_secs(2)));
    }

    #[test]
    fn schedules_without_max_sampling_rate_sample_every_frame() {
        let mut schedule = super::RecordingSchedule::new(None);

        assert!(schedule.should_sample(Duration::ZERO));
        assert!(schedule.should_sample(Duration::from_millis(100)));
        assert!(schedule.should_sample(Duration::from_millis(200)));
    }

    #[derive(Debug, Clone, Default, PartialEq)]
    struct FakeSimulator {
        values: HashMap<String, VecDeque<f64>>,
        validations: Vec<(String, Option<String>)>,
        reads: Vec<(String, Option<String>)>,
        fail_validation: Option<&'static str>,
        fail_read: Option<&'static str>,
    }

    impl FakeSimulator {
        fn queue(&mut self, variable: &str, values: impl IntoIterator<Item = f64>) {
            self.values
                .insert(variable.to_owned(), values.into_iter().collect());
        }
    }

    impl SimulatorAdapter for FakeSimulator {
        fn local_variable_exists(&mut self, _variable: &str) -> Result<bool, SimulatorError> {
            unreachable!("telemetry does not detect aircraft")
        }

        fn read_string(&mut self, _variable: &str) -> Result<String, SimulatorError> {
            unreachable!("telemetry records numeric values")
        }

        fn simulation_time(&self) -> Result<Duration, SimulatorError> {
            unreachable!("telemetry receives the current scenario time")
        }

        fn write(&mut self, _variable: &str, _value: f64) -> Result<(), SimulatorError> {
            unreachable!("telemetry never injects simulator inputs")
        }

        fn validate_read(
            &mut self,
            variable: &str,
            unit: Option<&str>,
        ) -> Result<(), SimulatorError> {
            self.validations
                .push((variable.to_owned(), unit.map(ToOwned::to_owned)));
            if self.fail_validation == Some(variable) {
                return Err(SimulatorError::UnsupportedReadVariable {
                    variable: variable.to_owned(),
                });
            }
            Ok(())
        }

        fn read(&mut self, variable: &str, unit: Option<&str>) -> Result<f64, SimulatorError> {
            self.reads
                .push((variable.to_owned(), unit.map(ToOwned::to_owned)));
            if self.fail_read == Some(variable) {
                return Err(SimulatorError::CalculatorCodeReadFailed {
                    variable: variable.to_owned(),
                });
            }
            Ok(self.values.get_mut(variable).unwrap().pop_front().unwrap())
        }
    }

    static NEXT_FIXTURE_ID: AtomicU64 = AtomicU64::new(0);

    #[derive(Debug)]
    struct Fixture {
        directory: PathBuf,
    }

    impl Fixture {
        fn new() -> Self {
            let id = NEXT_FIXTURE_ID.fetch_add(1, Ordering::Relaxed);
            let directory =
                std::env::temp_dir().join(format!("replay-telemetry-{}-{id}", std::process::id()));
            fs::create_dir_all(&directory).unwrap();
            Self { directory }
        }

        fn telemetry(&self, recordings: Vec<RecordingConfig>) -> Telemetry {
            Telemetry::new(
                &self.directory,
                recordings,
                &["input".to_owned()],
                UNIX_EPOCH,
            )
            .unwrap()
        }
    }

    impl Drop for Fixture {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.directory);
        }
    }

    fn recordings() -> Vec<RecordingConfig> {
        vec![
            RecordingConfig {
                name: "pitch".to_owned(),
                variable: "A:PLANE PITCH DEGREES".to_owned(),
                unit: Some("radians".to_owned()),
                max_sampling_rate: None,
            },
            RecordingConfig {
                name: "elevator_position".to_owned(),
                variable: "L:ELEVATOR_POSITION".to_owned(),
                unit: None,
                max_sampling_rate: None,
            },
        ]
    }

    const HEADER: &str = "pitch.time,pitch.value,elevator_position.time,elevator_position.value,input.time,input.value\n";

    #[test]
    fn mixed_sampling_rates_preserve_sparse_rows_units_and_bounded_buffers() {
        let fixture = Fixture::new();
        let mut definitions = recordings();
        definitions[0].max_sampling_rate = Some(2.0);
        definitions[1].max_sampling_rate = Some(1.0);
        let mut telemetry = fixture.telemetry(definitions);
        let mut simulator = FakeSimulator::default();
        simulator.queue("A:PLANE PITCH DEGREES", [10.0, 20.0, 30.0]);
        simulator.queue("L:ELEVATOR_POSITION", [40.0, 50.0]);
        telemetry.validate(&mut simulator).unwrap();
        let values_buffer = telemetry.values.as_ptr();
        let schedules_buffer = telemetry.schedules.as_ptr();

        for millis in [0, 250, 500, 1000] {
            let elapsed = Duration::from_millis(millis);
            telemetry
                .record_frame(elapsed, &[elapsed.as_secs_f64()], &mut simulator)
                .unwrap();
            assert_eq!(telemetry.values.len(), 2);
            assert_eq!(telemetry.values.as_ptr(), values_buffer);
            assert_eq!(telemetry.schedules.len(), 2);
            assert_eq!(telemetry.schedules.as_ptr(), schedules_buffer);
        }
        telemetry.flush().unwrap();

        let pitch = (
            "A:PLANE PITCH DEGREES".to_owned(),
            Some("radians".to_owned()),
        );
        let elevator = ("L:ELEVATOR_POSITION".to_owned(), None);
        assert_eq!(simulator.validations, vec![pitch.clone(), elevator.clone()]);
        assert_eq!(
            simulator.reads,
            vec![
                pitch.clone(),
                elevator.clone(),
                pitch.clone(),
                pitch,
                elevator
            ]
        );
        assert_eq!(
            fs::read_to_string(telemetry.path()).unwrap(),
            format!("{HEADER}0,10,0,40,0,0\n0.5,20,,,0.5,0.5\n1,30,1,50,1,1\n")
        );
    }

    #[test]
    fn validation_errors_identify_the_source_and_stop_before_later_signals() {
        for (index, variable, signal) in [
            (0, "A:PLANE PITCH DEGREES", "pitch"),
            (1, "L:ELEVATOR_POSITION", "elevator_position"),
        ] {
            let fixture = Fixture::new();
            let mut telemetry = fixture.telemetry(recordings());
            let mut simulator = FakeSimulator {
                fail_validation: Some(variable),
                ..FakeSimulator::default()
            };
            let error = telemetry.validate(&mut simulator).unwrap_err();
            assert!(matches!(
                error,
                TelemetryError::ValidateRecordingSignal {
                    signal: actual,
                    source: SimulatorError::UnsupportedReadVariable { variable: source },
                } if actual == signal && source == variable
            ));
            assert_eq!(simulator.validations.len(), index + 1);
            assert!(simulator.reads.is_empty());
            telemetry.flush().unwrap();
            assert_eq!(fs::read_to_string(telemetry.path()).unwrap(), HEADER);
        }
    }

    #[test]
    fn sampling_failure_preserves_previous_rows_without_serializing_a_partial_frame() {
        let fixture = Fixture::new();
        let mut telemetry = fixture.telemetry(recordings());
        let mut simulator = FakeSimulator::default();
        simulator.queue("A:PLANE PITCH DEGREES", [10.0, 20.0]);
        simulator.queue("L:ELEVATOR_POSITION", [40.0]);
        telemetry
            .record_frame(Duration::ZERO, &[0.0], &mut simulator)
            .unwrap();
        simulator.fail_read = Some("L:ELEVATOR_POSITION");

        let error = telemetry
            .record_frame(Duration::from_millis(500), &[0.5], &mut simulator)
            .unwrap_err();
        assert!(matches!(
            error,
            TelemetryError::SampleSignal {
                signal,
                source: SimulatorError::CalculatorCodeReadFailed { variable },
            } if signal == "elevator_position" && variable == "L:ELEVATOR_POSITION"
        ));
        telemetry.flush().unwrap();
        assert_eq!(
            fs::read_to_string(telemetry.path()).unwrap(),
            format!("{HEADER}0,10,0,40,0,0\n")
        );
    }
}
