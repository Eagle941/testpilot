//! Replay lifecycle orchestration independent of the MSFS gauge API.

use std::path::PathBuf;
use std::time::{Duration, SystemTime};

use crate::arm::ArmState;
use crate::config::{CONFIG_PATH, RecordingConfig, read_config_file};
use crate::error::{RecordingError, ReplayerError};
use crate::recording::TelemetryRecorder;
use crate::scenario::{InterpolationRows, ScenarioPlayback};

/// Data available while processing one running simulator frame.
pub(crate) struct InterpolationFrame<'a> {
    elapsed: Duration,
    playback: &'a ScenarioPlayback,
    recordings: &'a [RecordingConfig],
    recorder: &'a mut TelemetryRecorder,
}

impl InterpolationFrame<'_> {
    /// Returns elapsed scenario-relative simulator time.
    pub(crate) const fn elapsed(&self) -> Duration {
        self.elapsed
    }

    /// Returns one bounding data-point pair per configured injection.
    pub(crate) fn data_points(&self) -> impl Iterator<Item = InterpolationRows<'_>> {
        self.playback.interpolation_rows()
    }

    /// Returns configured telemetry signals in deterministic numeric order.
    pub(crate) const fn recordings(&self) -> &[RecordingConfig] {
        self.recordings
    }

    /// Appends the sampled telemetry values for this simulator frame.
    pub(crate) fn record(&mut self, values: &[f64]) -> Result<(), RecordingError> {
        self.recorder
            .write_frame(self.elapsed, self.recordings, values)
    }
}

/// Action produced by processing one simulator update.
pub(crate) enum ReplayerUpdate<'a> {
    /// Every cursor has data points available for interpolation.
    Running(InterpolationFrame<'a>),
    /// Every scenario cursor reached the end of its input series.
    Completed,
}

/// Scenario playback and its simulator-clock origin.
struct ActiveScenario {
    playback: ScenarioPlayback,
    recordings: Vec<RecordingConfig>,
    recorder: TelemetryRecorder,
    started_at: Duration,
}

/// Owns arming, scenario streaming, and simulator-clock scheduling state.
pub(crate) struct Replayer {
    config_path: PathBuf,
    arm_state: ArmState,
    active: Option<ActiveScenario>,
}

impl Replayer {
    /// Creates a replayer using the configuration in the writable MSFS work mount.
    pub(crate) fn new() -> Replayer {
        Replayer::with_config_path(CONFIG_PATH)
    }

    fn with_config_path(config_path: impl Into<PathBuf>) -> Replayer {
        Replayer {
            config_path: config_path.into(),
            arm_state: ArmState::default(),
            active: None,
        }
    }

    /// Processes one gauge update.
    ///
    /// The arming frame opens and initializes the configured cursor set.
    /// Simulator time is used once a scenario is loaded.
    pub(crate) fn pre_update(
        &mut self,
        armed_value: f64,
        simulation_time: Duration,
    ) -> anyhow::Result<Option<ReplayerUpdate<'_>>> {
        if self.arm_state.start(armed_value) {
            self.start_scenario(simulation_time)?;
        }
        if self.active.is_none() {
            return Ok(None);
        }

        Ok(Some(self.update_scenario(simulation_time)?))
    }

    /// Flushes telemetry and releases all replay state.
    ///
    /// Calling this repeatedly is safe. Replay state is released even when the
    /// final telemetry flush fails.
    pub(crate) fn reset(&mut self) -> Result<(), RecordingError> {
        self.arm_state = ArmState::default();
        let Some(mut active) = self.active.take() else {
            return Ok(());
        };
        active.recorder.flush()
    }

    fn start_scenario(&mut self, started_at: Duration) -> anyhow::Result<()> {
        if self.active.is_some() {
            return Err(ReplayerError::ScenarioAlreadyLoaded.into());
        }

        let config = read_config_file(&self.config_path)?;
        // Paths need to be joined because the scenario file is in the same
        // directory as the config file.
        let config_directory =
            self.config_path
                .parent()
                .ok_or_else(|| ReplayerError::ConfigPathWithoutParent {
                    path: self.config_path.clone(),
                })?;
        let scenario_path = config_directory.join(&config.input_file);
        let playback = ScenarioPlayback::new(&scenario_path, &config)?;
        #[cfg(target_arch = "wasm32")]
        let telemetry_directory = std::path::Path::new("/work");
        #[cfg(not(target_arch = "wasm32"))]
        let telemetry_directory =
            scenario_path
                .parent()
                .ok_or_else(|| ReplayerError::ScenarioPathWithoutParent {
                    path: scenario_path.clone(),
                })?;
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
        self.active = Some(ActiveScenario {
            playback,
            recordings: config.record,
            recorder,
            started_at,
        });
        println!("TESTPILOT: scenario cursors ready");
        Ok(())
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

        Ok(ReplayerUpdate::Running(InterpolationFrame {
            elapsed,
            playback: &active.playback,
            recordings: &active.recordings,
            recorder: &mut active.recorder,
        }))
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
            .pre_update(0.0, time(42.0))
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
        let ReplayerUpdate::Running(frame) = update else {
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
        let ReplayerUpdate::Running(frame) = update else {
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

        let ReplayerUpdate::Running(mut frame) = replayer.update_scenario(time(100.05)).unwrap()
        else {
            panic!("running update did not return interpolation data");
        };
        assert_eq!(frame.recordings().len(), 1);
        assert_eq!(frame.recordings()[0].name, "pitch");
        frame.record(&[0.125]).unwrap();
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
            replayer.pre_update(1.0, time(99.0)).unwrap(),
            Some(ReplayerUpdate::Running(_))
        ));
        assert_eq!(
            replayer.active.as_ref().map(|active| active.started_at),
            Some(time(99.0))
        );
        assert!(matches!(
            replayer.pre_update(1.0, time(99.05)).unwrap(),
            Some(ReplayerUpdate::Running(_))
        ));
        assert!(matches!(
            replayer.pre_update(1.0, time(99.2)).unwrap(),
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
        replayer.pre_update(1.0, Duration::ZERO).unwrap();

        replayer.reset().unwrap();
        replayer.reset().unwrap();

        assert!(replayer.active.is_none());
    }
}
