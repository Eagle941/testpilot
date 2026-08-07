//! Replay lifecycle orchestration independent of the MSFS gauge API.

use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};

use crate::config::{CONFIG_PATH, RecordingConfig, ReplayConfig};
use crate::cursor::{Frame, Scenario};
use crate::error::{RecordingError, ReplayerError};
use crate::recording::TelemetryRecorder;

/// Data available while processing one running simulator frame.
pub struct InterpolationFrame<'a> {
    /// Elapsed scenario time since playback start for the current frame.
    elapsed: Duration,
    /// Borrowed playback cursor set used for interpolation.
    playback: &'a Scenario,
    /// Configured telemetry signals in deterministic order.
    recordings: &'a [RecordingConfig],
    /// Mutable schedules that gate each recording signal by its sampling policy.
    recording_schedules: &'a mut [RecordingSchedule],
    /// Open telemetry writer for the active scenario.
    recorder: &'a mut TelemetryRecorder,
}

impl InterpolationFrame<'_> {
    /// Returns elapsed scenario-relative simulator time.
    pub const fn elapsed(&self) -> Duration {
        self.elapsed
    }

    /// Returns one bounding data-point pair per configured injection.
    pub fn data_points(&self) -> impl Iterator<Item = Frame<'_>> {
        self.playback.interpolation_rows()
    }

    /// Returns configured telemetry signals in deterministic numeric order.
    pub const fn recordings(&self) -> &[RecordingConfig] {
        self.recordings
    }

    /// Returns recordings together with mutable schedules for this frame.
    pub fn recordings_and_schedules(&mut self) -> (&[RecordingConfig], &mut [RecordingSchedule]) {
        (self.recordings, self.recording_schedules)
    }

    /// Appends the sampled telemetry values for this simulator frame.
    pub fn record(&mut self, values: &[Option<f64>]) -> Result<(), RecordingError> {
        self.recorder
            .write_frame(self.elapsed, self.recordings, values)
    }
}

/// Action produced by processing one simulator update.
pub enum ReplayerUpdate<'a> {
    /// Every cursor has data points available for interpolation after start-up.
    Running {
        frame: InterpolationFrame<'a>,
        started_now: bool,
    },
    /// Every scenario cursor reached the end of its input series.
    Completed,
}

/// Scenario playback and its simulator-clock origin.
struct ActiveScenario {
    playback: Scenario,
    recordings: Vec<RecordingConfig>,
    recorder: TelemetryRecorder,
    recording_schedules: Vec<RecordingSchedule>,
    started_at: Duration,
}

pub enum RecordingSchedule {
    EveryFrame,
    Limited {
        period: Duration,
        next_due: Duration,
    },
}

impl RecordingSchedule {
    fn new(max_sampling_rate: Option<f64>) -> RecordingSchedule {
        match max_sampling_rate {
            Some(rate) => {
                let period = Duration::from_secs_f64(1.0 / rate);
                RecordingSchedule::Limited {
                    period,
                    next_due: Duration::ZERO,
                }
            }
            None => RecordingSchedule::EveryFrame,
        }
    }

    pub fn should_sample(&mut self, elapsed: Duration) -> bool {
        match self {
            RecordingSchedule::EveryFrame => true,
            RecordingSchedule::Limited { period, next_due } => {
                if elapsed < *next_due {
                    return false;
                }

                *next_due = elapsed.saturating_add(*period);
                true
            }
        }
    }
}

/// Owns arming, scenario streaming, and simulator-clock scheduling state.
pub struct Replayer {
    /// Filesystem path from which replay configuration is loaded.
    config_path: PathBuf,
    /// Active replay state, if a scenario is currently loaded.
    active: Option<ActiveScenario>,
}

impl Replayer {
    /// Creates a replayer using the configuration in the writable MSFS work mount.
    pub fn new() -> Replayer {
        Replayer::with_config_path(CONFIG_PATH)
    }

    fn with_config_path(config_path: impl Into<PathBuf>) -> Replayer {
        Replayer {
            config_path: config_path.into(),
            active: None,
        }
    }

    /// Processes one gauge update.
    ///
    /// The arming frame opens and initializes the configured cursor set.
    /// Simulator time is used once a scenario is loaded.
    pub fn pre_update(
        &mut self,
        init: bool,
        simulation_time: Duration,
    ) -> anyhow::Result<Option<ReplayerUpdate<'_>>> {
        if init {
            self.start_scenario(simulation_time)?;
        }
        if self.active.is_none() {
            return Ok(None);
        }

        let update = self.update_scenario(simulation_time)?;
        Ok(Some(match update {
            ReplayerUpdate::Running {
                frame,
                started_now: false,
            } if init => ReplayerUpdate::Running {
                frame,
                started_now: true,
            },
            update => update,
        }))
    }

    /// Flushes telemetry and releases all replay state.
    ///
    /// Calling this repeatedly is safe. Replay state is released even when the
    /// final telemetry flush fails.
    pub fn reset(&mut self) -> Result<(), RecordingError> {
        let Some(mut active) = self.active.take() else {
            return Ok(());
        };
        active.recorder.flush()
    }

    fn start_scenario(&mut self, started_at: Duration) -> anyhow::Result<()> {
        if self.active.is_some() {
            return Err(ReplayerError::ScenarioAlreadyLoaded.into());
        }

        let config = ReplayConfig::read_config_file(&self.config_path)?;
        // Paths need to be joined because the scenario file is in the same
        // directory as the config file.
        let config_directory =
            self.config_path
                .parent()
                .ok_or_else(|| ReplayerError::ConfigPathWithoutParent {
                    path: self.config_path.clone(),
                })?;
        let scenario_path = config_directory.join(&config.input_file);
        let playback = Scenario::new(&scenario_path, &config)?;
        let telemetry_directory = self.telemetry_directory(&scenario_path)?;
        println!("TESTPILOT: reading host UTC timestamp");
        let recording_started_at = SystemTime::now();
        println!(
            "TESTPILOT: creating telemetry file in {}",
            telemetry_directory.display()
        );
        let recorder =
            TelemetryRecorder::new(telemetry_directory, &config.record, recording_started_at)?;

        println!(
            "TESTPILOT: opened {} with {} signal cursors",
            scenario_path.display(),
            playback.signal_count()
        );
        println!(
            "TESTPILOT: recording telemetry to {}",
            recorder.path().display()
        );
        let recordings = config.record;
        let recording_schedules = recordings
            .iter()
            .map(|recording| RecordingSchedule::new(recording.max_sampling_rate))
            .collect();
        self.active = Some(ActiveScenario {
            playback,
            recordings,
            recorder,
            recording_schedules,
            started_at,
        });
        println!("TESTPILOT: scenario cursors ready");
        Ok(())
    }

    #[cfg(target_arch = "wasm32")]
    fn telemetry_directory(&self, _scenario_path: &Path) -> Result<PathBuf, ReplayerError> {
        Ok(Path::new("/work").to_path_buf())
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn telemetry_directory(&self, scenario_path: &Path) -> Result<PathBuf, ReplayerError> {
        scenario_path
            .parent()
            .map(ToOwned::to_owned)
            .ok_or_else(|| ReplayerError::ScenarioPathWithoutParent {
                path: scenario_path.to_path_buf(),
            })
    }

    fn update_scenario(&mut self, simulation_time: Duration) -> anyhow::Result<ReplayerUpdate<'_>> {
        let active = self.active.as_mut().ok_or(ReplayerError::UpdateWhileIdle)?;
        let elapsed = simulation_time.checked_sub(active.started_at).ok_or(
            ReplayerError::SimulationTimeMovedBackwards {
                started_at: active.started_at,
                current: simulation_time,
            },
        )?;

        active.playback.advance(elapsed)?;
        if active.playback.completed() {
            return Ok(ReplayerUpdate::Completed);
        }

        Ok(ReplayerUpdate::Running {
            frame: InterpolationFrame {
                elapsed,
                playback: &active.playback,
                recordings: &active.recordings,
                recording_schedules: &mut active.recording_schedules,
                recorder: &mut active.recorder,
            },
            started_now: false,
        })
    }
}

#[cfg(test)]
mod tests {

    use std::fs;
    use std::path::PathBuf;
    use std::time::Duration;

    use crate::error::ReplayerError;

    use super::{Replayer, ReplayerUpdate};

    fn time(seconds: f64) -> Duration {
        Duration::try_from_secs_f64(seconds).unwrap()
    }

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
    fn uses_the_work_configuration_path_by_default() {
        let replayer = Replayer::new();

        assert_eq!(
            replayer.config_path,
            PathBuf::from(crate::config::CONFIG_PATH)
        );
    }

    #[test]
    fn remains_idle_when_disarmed() {
        let fixture = Fixture::new();
        let mut replayer = Replayer::with_config_path(fixture.config_path.clone());

        let update = replayer
            .pre_update(false, time(42.0))
            .unwrap_or_else(|error| panic!("idle update failed: {error:#}"));

        assert!(update.is_none());
    }

    #[test]
    fn start_scenario_loads_the_configured_playback() {
        let fixture = Fixture::new();
        let mut replayer = Replayer::with_config_path(fixture.config_path.clone());

        replayer
            .start_scenario(time(100.0))
            .unwrap_or_else(|error| panic!("failed to start scenario: {error:#}"));

        let active = replayer.active.as_ref().expect("scenario was not started");
        assert_eq!(active.started_at, time(100.0));
    }

    #[test]
    fn start_scenario_rejects_an_overlapping_replay() {
        let fixture = Fixture::new();
        let mut replayer = Replayer::with_config_path(fixture.config_path.clone());
        replayer.start_scenario(time(100.0)).unwrap();

        let error = replayer
            .start_scenario(time(101.0))
            .expect_err("overlapping replay should fail");

        assert!(matches!(
            error.downcast_ref::<ReplayerError>(),
            Some(ReplayerError::ScenarioAlreadyLoaded)
        ));
        assert!(replayer.active.is_some());
    }

    #[test]
    fn update_scenario_starts_and_completes_playback() {
        let fixture = Fixture::new();
        let mut replayer = Replayer::with_config_path(fixture.config_path.clone());
        replayer.start_scenario(time(100.0)).unwrap();

        let update = replayer.update_scenario(time(100.0)).unwrap();
        let ReplayerUpdate::Running { frame, .. } = update else {
            panic!("running update did not return interpolation data");
        };
        assert_eq!(frame.elapsed(), Duration::ZERO);
        let data_points = frame.data_points().collect::<Vec<_>>();
        assert_eq!(data_points.len(), 1);
        assert_eq!(data_points[0].signal, "sidestick_pitch_position");
        assert_eq!(data_points[0].previous.time, Duration::ZERO);
        assert_eq!(data_points[0].previous.value, 0.0);
        assert_eq!(
            data_points[0].next,
            Some(crate::playback::Sample::new(time(0.1), 10.0).unwrap())
        );
        assert_eq!(
            replayer.active.as_ref().map(|active| active.started_at),
            Some(time(100.0))
        );

        let update = replayer.update_scenario(time(100.0625)).unwrap();
        let ReplayerUpdate::Running { frame, .. } = update else {
            panic!("running update did not return interpolation data");
        };
        assert_eq!(frame.elapsed(), time(0.0625));

        let update = replayer.update_scenario(time(100.2)).unwrap();
        assert!(matches!(update, ReplayerUpdate::Completed));
        assert!(replayer.active.is_some());
        replayer.reset().unwrap();
        assert!(replayer.active.is_none());
    }

    #[test]
    fn records_running_frames_beside_the_scenario() {
        let fixture = Fixture::new();
        let mut replayer = Replayer::with_config_path(fixture.config_path.clone());
        replayer.start_scenario(time(100.0)).unwrap();

        let ReplayerUpdate::Running { mut frame, .. } =
            replayer.update_scenario(time(100.05)).unwrap()
        else {
            panic!("running update did not return interpolation data");
        };
        assert_eq!(frame.recordings().len(), 1);
        assert_eq!(frame.recordings()[0].name, "pitch");
        frame.record(&[Some(0.125)]).unwrap();
        replayer.reset().unwrap();

        let telemetry_path = fs::read_dir(&fixture.directory)
            .unwrap()
            .filter_map(Result::ok)
            .map(|entry| entry.path())
            .find(|path| {
                path.file_name()
                    .and_then(|name| name.to_str())
                    .is_some_and(|name| name.starts_with("telemetry_") && name.ends_with(".csv"))
            })
            .expect("telemetry file was not created beside the scenario");
        assert_eq!(
            fs::read_to_string(telemetry_path).unwrap(),
            "pitch.time,pitch.value\n0.05,0.125\n"
        );
    }

    #[test]
    fn writes_sparse_rows_for_sample_limited_recordings() {
        let fixture = Fixture::new();
        fs::write(
            &fixture.config_path,
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
max_sampling_rate = 1.0

[record.1]
name = "roll"
variable = "A:PLANE BANK DEGREES"
unit = "radians"
"#,
        )
        .unwrap_or_else(|error| panic!("failed to rewrite fixture config: {error}"));
        fs::write(
            fixture.directory.join("scenario.csv"),
            "sidestick_pitch_position.time,sidestick_pitch_position.value\n0,0\n1,10\n2,20\n",
        )
        .unwrap_or_else(|error| panic!("failed to write fixture scenario: {error}"));

        let mut replayer = Replayer::with_config_path(fixture.config_path.clone());
        replayer.start_scenario(time(10.0)).unwrap();

        let ReplayerUpdate::Running { mut frame, .. } =
            replayer.update_scenario(time(10.0)).unwrap()
        else {
            panic!("running update did not return interpolation data");
        };
        assert_eq!(frame.recordings().len(), 2);
        assert_eq!(frame.recordings_and_schedules().1.len(), 2);
        frame.record(&[Some(0.1), Some(10.0)]).unwrap();

        let ReplayerUpdate::Running { mut frame, .. } =
            replayer.update_scenario(time(10.5)).unwrap()
        else {
            panic!("running update did not return interpolation data");
        };
        frame.record(&[None, Some(20.0)]).unwrap();

        let ReplayerUpdate::Running { mut frame, .. } =
            replayer.update_scenario(time(11.0)).unwrap()
        else {
            panic!("running update did not return interpolation data");
        };
        frame.record(&[Some(0.2), Some(30.0)]).unwrap();
        replayer.reset().unwrap();

        let telemetry_path = fs::read_dir(&fixture.directory)
            .unwrap()
            .filter_map(Result::ok)
            .map(|entry| entry.path())
            .find(|path| {
                path.file_name()
                    .and_then(|name| name.to_str())
                    .is_some_and(|name| name.starts_with("telemetry_") && name.ends_with(".csv"))
            })
            .expect("telemetry file was not created beside the scenario");
        assert_eq!(
            fs::read_to_string(telemetry_path).unwrap(),
            "pitch.time,pitch.value,roll.time,roll.value\n0,0.1,0,10\n,,0.5,20\n1,0.2,1,30\n"
        );
    }

    #[test]
    fn does_not_emit_rows_when_no_recordings_are_due() {
        let fixture = Fixture::new();
        fs::write(
            &fixture.config_path,
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
max_sampling_rate = 1.0

[record.1]
name = "roll"
variable = "A:PLANE BANK DEGREES"
unit = "radians"
max_sampling_rate = 1.0
"#,
        )
        .unwrap_or_else(|error| panic!("failed to rewrite fixture config: {error}"));
        fs::write(
            fixture.directory.join("scenario.csv"),
            "sidestick_pitch_position.time,sidestick_pitch_position.value\n0,0\n1,10\n2,20\n",
        )
        .unwrap_or_else(|error| panic!("failed to write fixture scenario: {error}"));

        let mut replayer = Replayer::with_config_path(fixture.config_path.clone());
        replayer.start_scenario(time(10.0)).unwrap();

        let ReplayerUpdate::Running { mut frame, .. } =
            replayer.update_scenario(time(10.0)).unwrap()
        else {
            panic!("running update did not return interpolation data");
        };
        frame.record(&[Some(0.1), Some(10.0)]).unwrap();

        let ReplayerUpdate::Running { .. } = replayer.update_scenario(time(10.5)).unwrap() else {
            panic!("running update did not return interpolation data");
        };

        let ReplayerUpdate::Running { mut frame, .. } =
            replayer.update_scenario(time(11.0)).unwrap()
        else {
            panic!("running update did not return interpolation data");
        };
        frame.record(&[Some(0.2), Some(20.0)]).unwrap();
        replayer.reset().unwrap();

        let telemetry_path = fs::read_dir(&fixture.directory)
            .unwrap()
            .filter_map(Result::ok)
            .map(|entry| entry.path())
            .find(|path| {
                path.file_name()
                    .and_then(|name| name.to_str())
                    .is_some_and(|name| name.starts_with("telemetry_") && name.ends_with(".csv"))
            })
            .expect("telemetry file was not created beside the scenario");
        assert_eq!(
            fs::read_to_string(telemetry_path).unwrap(),
            "pitch.time,pitch.value,roll.time,roll.value\n0,0.1,0,10\n1,0.2,1,20\n"
        );
    }

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

    #[test]
    fn update_scenario_rejects_an_idle_replayer() {
        let fixture = Fixture::new();
        let mut replayer = Replayer::with_config_path(fixture.config_path.clone());

        let error = match replayer.update_scenario(time(1.0)) {
            Ok(_) => panic!("idle scenario update should fail"),
            Err(error) => error,
        };

        assert!(matches!(
            error.downcast_ref::<ReplayerError>(),
            Some(ReplayerError::UpdateWhileIdle)
        ));
    }

    #[test]
    fn arms_loads_and_completes_a_scenario() {
        let fixture = Fixture::new();
        let mut replayer = Replayer::with_config_path(fixture.config_path.clone());

        assert!(matches!(
            replayer.pre_update(true, time(99.0)).unwrap(),
            Some(ReplayerUpdate::Running {
                started_now: true,
                ..
            })
        ));
        assert_eq!(
            replayer.active.as_ref().map(|active| active.started_at),
            Some(time(99.0))
        );
        assert!(matches!(
            replayer.pre_update(false, time(99.05)).unwrap(),
            Some(ReplayerUpdate::Running {
                started_now: false,
                ..
            })
        ));
        assert!(matches!(
            replayer.pre_update(false, time(99.2)).unwrap(),
            Some(ReplayerUpdate::Completed)
        ));
        assert!(replayer.active.is_some());
    }

    #[test]
    fn update_scenario_rejects_simulator_time_moving_backwards() {
        let fixture = Fixture::new();
        let mut replayer = Replayer::with_config_path(fixture.config_path.clone());

        replayer.start_scenario(time(10.0)).unwrap();
        replayer.update_scenario(time(10.0)).unwrap();
        let error = match replayer.update_scenario(time(9.5)) {
            Ok(_) => panic!("backwards simulator time should fail"),
            Err(error) => error,
        };

        assert!(matches!(
            error.downcast_ref::<ReplayerError>(),
            Some(ReplayerError::SimulationTimeMovedBackwards {
                started_at,
                current,
            }) if *started_at == time(10.0) && *current == time(9.5)
        ));
    }

    #[test]
    fn reset_is_idempotent() {
        let fixture = Fixture::new();
        let mut replayer = Replayer::with_config_path(fixture.config_path.clone());
        replayer.pre_update(true, Duration::ZERO).unwrap();

        replayer.reset().unwrap();
        replayer.reset().unwrap();

        assert!(replayer.active.is_none());
    }
}
