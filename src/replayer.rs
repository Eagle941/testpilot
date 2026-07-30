//! Replay lifecycle orchestration independent of the MSFS gauge API.

use std::path::PathBuf;

use anyhow::Context;

use crate::arm::ArmState;
use crate::config::{CONFIG_PATH, read_config_file};
use crate::error::ReplayerError;
use crate::scenario::{InterpolationRows, ScenarioPlayback, ScenarioProgress, ScenarioStep};

/// Result of processing one gauge update.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ReplayerEvent {
    /// No scenario is currently loaded.
    Idle,
    /// A rising arming edge loaded a scenario.
    Started,
    /// Scenario cursors do not yet all contain two samples.
    Loading,
    /// Every cursor has data points available for interpolation.
    Running,
    /// Every scenario cursor reached the end of its input series.
    Completed,
}

/// Data points returned for interpolation on one simulator frame.
pub(crate) struct InterpolationFrame<'a> {
    elapsed_seconds: f64,
    step: ScenarioStep<'a>,
}

impl InterpolationFrame<'_> {
    /// Returns scenario-relative simulator time in seconds.
    pub(crate) const fn elapsed_seconds(&self) -> f64 {
        self.elapsed_seconds
    }

    /// Returns one bounding data-point pair per configured injection.
    pub(crate) fn data_points(&self) -> impl Iterator<Item = InterpolationRows<'_>> {
        self.step.interpolation_rows()
    }
}

/// Result of processing one gauge update.
pub(crate) struct ReplayerUpdate<'a> {
    event: ReplayerEvent,
    interpolation: Option<InterpolationFrame<'a>>,
}

impl ReplayerUpdate<'_> {
    pub(crate) const fn event(&self) -> ReplayerEvent {
        self.event
    }

    pub(crate) const fn interpolation(&self) -> Option<&InterpolationFrame<'_>> {
        self.interpolation.as_ref()
    }

    const fn without_interpolation(event: ReplayerEvent) -> Self {
        Self {
            event,
            interpolation: None,
        }
    }
}

/// Owns arming, scenario streaming, and simulator-clock scheduling state.
pub(crate) struct ReplayerGauge {
    config_path: PathBuf,
    arm_state: ArmState,
    scenario: Option<ScenarioPlayback>,
    started_at_seconds: Option<f64>,
}

impl ReplayerGauge {
    /// Creates a replayer using the package-relative MVP configuration path.
    pub(crate) fn new() -> Self {
        Self::with_config_path(CONFIG_PATH)
    }

    fn with_config_path(config_path: impl Into<PathBuf>) -> Self {
        Self {
            config_path: config_path.into(),
            arm_state: ArmState::default(),
            scenario: None,
            started_at_seconds: None,
        }
    }

    /// Processes one gauge update.
    ///
    /// The arming frame opens the configured cursor set but consumes no
    /// scenario data rows. Simulator time is used once a scenario is loaded.
    pub(crate) fn pre_update(
        &mut self,
        armed_value: f64,
        simulation_time_seconds: f64,
    ) -> anyhow::Result<ReplayerUpdate<'_>> {
        if self.arm_state.start(armed_value) {
            self.begin_scenario()?;
            return Ok(ReplayerUpdate::without_interpolation(
                ReplayerEvent::Started,
            ));
        }
        if self.scenario.is_none() {
            return Ok(ReplayerUpdate::without_interpolation(ReplayerEvent::Idle));
        }

        self.update_scenario(simulation_time_seconds)
    }

    /// Releases all replay state. Calling this repeatedly is safe.
    pub(crate) fn reset(&mut self) {
        self.scenario = None;
        self.started_at_seconds = None;
    }

    fn begin_scenario(&mut self) -> anyhow::Result<()> {
        if self.scenario.is_some() {
            return Err(ReplayerError::ScenarioAlreadyLoaded.into());
        }

        let config = read_config_file(&self.config_path)?;
        let config_directory =
            self.config_path
                .parent()
                .ok_or_else(|| ReplayerError::ConfigPathWithoutParent {
                    path: self.config_path.clone(),
                })?;
        let scenario_path = config_directory.join(&config.input_file);
        let playback = ScenarioPlayback::open(&scenario_path, &config)
            .with_context(|| format!("failed to open scenario `{}`", scenario_path.display()))?;

        println!(
            "REPLAYER: opened {} with {} signal cursors",
            scenario_path.display(),
            playback.signal_count()
        );
        self.scenario = Some(playback);
        self.started_at_seconds = None;
        Ok(())
    }

    fn update_scenario(
        &mut self,
        simulation_time_seconds: f64,
    ) -> anyhow::Result<ReplayerUpdate<'_>> {
        if !simulation_time_seconds.is_finite() {
            return Err(ReplayerError::InvalidSimulationTime {
                value: simulation_time_seconds,
            }
            .into());
        }

        let elapsed_seconds = self
            .started_at_seconds
            .map_or(0.0, |started| simulation_time_seconds - started);
        if !elapsed_seconds.is_finite() || elapsed_seconds < 0.0 {
            return Err(ReplayerError::InvalidElapsedSimulationTime {
                value: elapsed_seconds,
            }
            .into());
        }

        let playback = self
            .scenario
            .as_mut()
            .ok_or(ReplayerError::UpdateWhileIdle)?;
        let step = playback.next(elapsed_seconds)?;
        match step.progress() {
            ScenarioProgress::Loading => Ok(ReplayerUpdate::without_interpolation(
                ReplayerEvent::Loading,
            )),
            ScenarioProgress::Completed => Ok(ReplayerUpdate::without_interpolation(
                ReplayerEvent::Completed,
            )),
            ScenarioProgress::Running => {
                let elapsed_seconds = match self.started_at_seconds {
                    Some(started) => simulation_time_seconds - started,
                    None => {
                        self.started_at_seconds = Some(simulation_time_seconds);
                        println!("REPLAYER: scenario cursors ready");
                        0.0
                    }
                };

                Ok(ReplayerUpdate {
                    event: ReplayerEvent::Running,
                    interpolation: Some(InterpolationFrame {
                        elapsed_seconds,
                        step,
                    }),
                })
            }
        }
    }
}

#[cfg(test)]
mod tests {

    use std::fs;
    use std::path::PathBuf;

    use crate::error::ReplayerError;

    use super::{ReplayerEvent, ReplayerGauge};

    struct Fixture {
        directory: PathBuf,
        config_path: PathBuf,
    }

    impl Fixture {
        fn new() -> Self {
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

            Self {
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

        let event = replayer
            .pre_update(0.0, 42.0)
            .unwrap_or_else(|error| panic!("idle update failed: {error:#}"));

        assert_eq!(event.event(), ReplayerEvent::Idle);
    }

    #[test]
    fn begin_scenario_loads_the_configured_playback() {
        let fixture = Fixture::new();
        let mut replayer = ReplayerGauge::with_config_path(fixture.config_path.clone());

        replayer
            .begin_scenario()
            .unwrap_or_else(|error| panic!("failed to begin scenario: {error:#}"));

        assert!(replayer.scenario.is_some());
        assert_eq!(replayer.started_at_seconds, None);
    }

    #[test]
    fn begin_scenario_rejects_an_overlapping_replay() {
        let fixture = Fixture::new();
        let mut replayer = ReplayerGauge::with_config_path(fixture.config_path.clone());
        replayer.begin_scenario().unwrap();

        let error = replayer
            .begin_scenario()
            .expect_err("overlapping replay should fail");

        assert!(matches!(
            error.downcast_ref::<ReplayerError>(),
            Some(ReplayerError::ScenarioAlreadyLoaded)
        ));
        assert!(replayer.scenario.is_some());
    }

    #[test]
    fn update_scenario_loads_starts_and_completes_playback() {
        let fixture = Fixture::new();
        let mut replayer = ReplayerGauge::with_config_path(fixture.config_path.clone());
        replayer.begin_scenario().unwrap();

        let update = replayer.update_scenario(100.0).unwrap();
        assert_eq!(update.event(), ReplayerEvent::Loading);
        assert!(update.interpolation().is_none());
        assert_eq!(replayer.started_at_seconds, None);

        let update = replayer.update_scenario(101.0).unwrap();
        assert_eq!(update.event(), ReplayerEvent::Running);
        let frame = update
            .interpolation()
            .unwrap_or_else(|| panic!("running update did not return interpolation data"));
        assert_eq!(frame.elapsed_seconds(), 0.0);
        let data_points = frame.data_points().collect::<Vec<_>>();
        assert_eq!(data_points.len(), 1);
        assert_eq!(data_points[0].signal, "sidestick_pitch_position");
        assert_eq!(data_points[0].previous.time_seconds, 0.0);
        assert_eq!(data_points[0].previous.value, 0.0);
        assert_eq!(
            data_points[0].next,
            Some(crate::playback::Sample::new(0.1, 10.0).unwrap())
        );
        assert_eq!(replayer.started_at_seconds, Some(101.0));

        let update = replayer.update_scenario(101.0625).unwrap();
        assert_eq!(update.event(), ReplayerEvent::Running);
        assert_eq!(
            update.interpolation().map(|frame| frame.elapsed_seconds()),
            Some(0.0625)
        );

        let update = replayer.update_scenario(101.2).unwrap();
        assert_eq!(update.event(), ReplayerEvent::Completed);
        assert!(update.interpolation().is_none());
        assert!(replayer.scenario.is_some());
        replayer.reset();
        assert_eq!(replayer.started_at_seconds, None);
    }

    #[test]
    fn update_scenario_rejects_an_idle_replayer() {
        let fixture = Fixture::new();
        let mut replayer = ReplayerGauge::with_config_path(fixture.config_path.clone());

        let error = match replayer.update_scenario(1.0) {
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

        assert_eq!(
            replayer.pre_update(1.0, 99.0).unwrap().event(),
            ReplayerEvent::Started
        );
        assert_eq!(
            replayer.pre_update(1.0, 100.0).unwrap().event(),
            ReplayerEvent::Loading
        );
        assert_eq!(replayer.started_at_seconds, None);
        assert_eq!(
            replayer.pre_update(1.0, 101.0).unwrap().event(),
            ReplayerEvent::Running
        );
        assert_eq!(replayer.started_at_seconds, Some(101.0));
        assert_eq!(
            replayer.pre_update(1.0, 101.2).unwrap().event(),
            ReplayerEvent::Completed
        );
        assert!(replayer.scenario.is_some());
    }

    #[test]
    fn update_scenario_rejects_simulator_time_moving_backwards() {
        let fixture = Fixture::new();
        let mut replayer = ReplayerGauge::with_config_path(fixture.config_path.clone());

        replayer.begin_scenario().unwrap();
        replayer.update_scenario(10.0).unwrap();
        replayer.update_scenario(11.0).unwrap();
        let error = match replayer.update_scenario(10.5) {
            Ok(_) => panic!("backwards simulator time should fail"),
            Err(error) => error,
        };

        assert!(matches!(
            error.downcast_ref::<ReplayerError>(),
            Some(ReplayerError::InvalidElapsedSimulationTime { value }) if *value == -0.5
        ));
    }

    #[test]
    fn reset_is_idempotent() {
        let fixture = Fixture::new();
        let mut replayer = ReplayerGauge::with_config_path(fixture.config_path.clone());
        replayer.pre_update(1.0, 0.0).unwrap();

        replayer.reset();
        replayer.reset();

        assert!(replayer.scenario.is_none());
        assert_eq!(replayer.started_at_seconds, None);
    }
}
