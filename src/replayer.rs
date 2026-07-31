//! Replay lifecycle orchestration independent of the MSFS gauge API.

use std::path::PathBuf;
use std::time::Duration;

use crate::arm::ArmState;
use crate::config::{CONFIG_PATH, read_config_file};
use crate::error::ReplayerError;
use crate::scenario::{InterpolationRows, ScenarioPlayback};

/// Data available while processing one running simulator frame.
pub(crate) struct InterpolationFrame<'a> {
    elapsed: Duration,
    playback: &'a ScenarioPlayback,
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
}

/// Action produced by processing one simulator update.
pub(crate) enum ReplayerUpdate<'a> {
    /// Every cursor has data points available for interpolation.
    Running(InterpolationFrame<'a>),
    /// Every scenario cursor reached the end of its input series.
    Completed,
}

/// Owns arming, scenario streaming, and simulator-clock scheduling state.
pub(crate) struct ReplayerGauge {
    config_path: PathBuf,
    arm_state: ArmState,
    scenario: Option<ScenarioPlayback>,
    started_at: Option<Duration>,
}

impl ReplayerGauge {
    /// Creates a replayer using the package-relative MVP configuration path.
    pub(crate) fn new() -> ReplayerGauge {
        ReplayerGauge::with_config_path(CONFIG_PATH)
    }

    fn with_config_path(config_path: impl Into<PathBuf>) -> ReplayerGauge {
        ReplayerGauge {
            config_path: config_path.into(),
            arm_state: ArmState::default(),
            scenario: None,
            started_at: None,
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
            self.init_scenario()?;
        }
        if self.scenario.is_none() {
            return Ok(None);
        }

        Ok(Some(self.update_scenario(simulation_time)?))
    }

    /// Releases all replay state. Calling this repeatedly is safe.
    pub(crate) fn reset(&mut self) {
        self.scenario = None;
        self.started_at = None;
    }

    fn init_scenario(&mut self) -> anyhow::Result<()> {
        if self.scenario.is_some() {
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

        println!(
            "TESTPILOT: opened {} with {} signal cursors",
            scenario_path.display(),
            playback.signal_count()
        );
        self.scenario = Some(playback);
        self.started_at = None;
        Ok(())
    }

    fn update_scenario(&mut self, simulation_time: Duration) -> anyhow::Result<ReplayerUpdate<'_>> {
        let playback = self
            .scenario
            .as_mut()
            .ok_or(ReplayerError::UpdateWhileIdle)?;
        let started_at = match self.started_at {
            Some(started_at) => started_at,
            None => {
                self.started_at = Some(simulation_time);
                println!("TESTPILOT: scenario cursors ready");
                simulation_time
            }
        };
        let elapsed = simulation_time.checked_sub(started_at).ok_or(
            ReplayerError::SimulationTimeMovedBackwards {
                started_at,
                current: simulation_time,
            },
        )?;

        playback.advance(elapsed)?;
        if playback.completed() {
            return Ok(ReplayerUpdate::Completed);
        }

        Ok(ReplayerUpdate::Running(InterpolationFrame {
            elapsed,
            playback,
        }))
    }
}

#[cfg(test)]
mod tests {

    use std::fs;
    use std::path::PathBuf;
    use std::time::Duration;

    use crate::error::ReplayerError;

    use super::{ReplayerGauge, ReplayerUpdate};

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
aircraft_target = "flybywire-a32nx"
input_file = "scenario.csv"

[inject.0]
name = "sidestick_pitch_position"
variable = "K:AXIS_ELEVATOR_SET"
time_column = "sidestick_pitch_position.time"
value_column = "sidestick_pitch_position.value"
source_range = [-100.0, 100.0]
simulator_range = [-1.0, 1.0]

[record.0]
name = "pitch"
variable = "A:PLANE PITCH DEGREES"
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
    fn uses_the_package_relative_configuration_path_by_default() {
        let replayer = ReplayerGauge::new();

        assert_eq!(
            replayer.config_path,
            PathBuf::from(crate::config::CONFIG_PATH)
        );
    }

    #[test]
    fn remains_idle_when_disarmed() {
        let fixture = Fixture::new();
        let mut replayer = ReplayerGauge::with_config_path(fixture.config_path.clone());

        let update = replayer
            .pre_update(0.0, time(42.0))
            .unwrap_or_else(|error| panic!("idle update failed: {error:#}"));

        assert!(update.is_none());
    }

    #[test]
    fn begin_scenario_loads_the_configured_playback() {
        let fixture = Fixture::new();
        let mut replayer = ReplayerGauge::with_config_path(fixture.config_path.clone());

        replayer
            .init_scenario()
            .unwrap_or_else(|error| panic!("failed to begin scenario: {error:#}"));

        assert!(replayer.scenario.is_some());
        assert_eq!(replayer.started_at, None);
    }

    #[test]
    fn begin_scenario_rejects_an_overlapping_replay() {
        let fixture = Fixture::new();
        let mut replayer = ReplayerGauge::with_config_path(fixture.config_path.clone());
        replayer.init_scenario().unwrap();

        let error = replayer
            .init_scenario()
            .expect_err("overlapping replay should fail");

        assert!(matches!(
            error.downcast_ref::<ReplayerError>(),
            Some(ReplayerError::ScenarioAlreadyLoaded)
        ));
        assert!(replayer.scenario.is_some());
    }

    #[test]
    fn update_scenario_starts_and_completes_playback() {
        let fixture = Fixture::new();
        let mut replayer = ReplayerGauge::with_config_path(fixture.config_path.clone());
        replayer.init_scenario().unwrap();

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
        assert_eq!(replayer.started_at, Some(time(100.0)));

        let update = replayer.update_scenario(time(100.0625)).unwrap();
        let ReplayerUpdate::Running(frame) = update else {
            panic!("running update did not return interpolation data");
        };
        assert_eq!(frame.elapsed(), time(0.0625));

        let update = replayer.update_scenario(time(100.2)).unwrap();
        assert!(matches!(update, ReplayerUpdate::Completed));
        assert!(replayer.scenario.is_some());
        replayer.reset();
        assert_eq!(replayer.started_at, None);
    }

    #[test]
    fn update_scenario_rejects_an_idle_replayer() {
        let fixture = Fixture::new();
        let mut replayer = ReplayerGauge::with_config_path(fixture.config_path.clone());

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
        let mut replayer = ReplayerGauge::with_config_path(fixture.config_path.clone());

        assert!(matches!(
            replayer.pre_update(1.0, time(99.0)).unwrap(),
            Some(ReplayerUpdate::Running(_))
        ));
        assert_eq!(replayer.started_at, Some(time(99.0)));
        assert!(matches!(
            replayer.pre_update(1.0, time(99.05)).unwrap(),
            Some(ReplayerUpdate::Running(_))
        ));
        assert!(matches!(
            replayer.pre_update(1.0, time(99.2)).unwrap(),
            Some(ReplayerUpdate::Completed)
        ));
        assert!(replayer.scenario.is_some());
    }

    #[test]
    fn update_scenario_rejects_simulator_time_moving_backwards() {
        let fixture = Fixture::new();
        let mut replayer = ReplayerGauge::with_config_path(fixture.config_path.clone());

        replayer.init_scenario().unwrap();
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
        let mut replayer = ReplayerGauge::with_config_path(fixture.config_path.clone());
        replayer.pre_update(1.0, Duration::ZERO).unwrap();

        replayer.reset();
        replayer.reset();

        assert!(replayer.scenario.is_none());
        assert_eq!(replayer.started_at, None);
    }
}
