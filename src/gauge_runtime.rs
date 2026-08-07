//! MSFS-specific replay runtime used by the gauge entry point.

use crate::arm::ArmingMonitor;
use crate::error::GaugeError;
use crate::replayer::{InterpolationFrame, Replayer, ReplayerUpdate};
use crate::simulator::{MsfsSimulator, SimulatorAdapter};

/// Local simulator variable that arms replay start when it transitions to `1`.
const ARMED_VARIABLE: &str = "L:REPLAYER_ARMED";

/// Owns the MSFS variables and replay state used by the gauge event loop.
pub struct GaugeRuntime<S = MsfsSimulator> {
    /// Replay orchestrator.
    replayer: Replayer,
    /// Tracks arming transitions and writes reset state.
    arming: ArmingMonitor,
    /// Adapter around msfs-rs legacy calculator code.
    simulator: S,
}

impl<S: SimulatorAdapter> GaugeRuntime<S> {
    /// Creates a runtime from explicit replay and simulator components.
    pub fn new(replayer: Replayer, simulator: S) -> Result<Self, GaugeError> {
        let mut runtime = Self {
            arming: ArmingMonitor::new(ARMED_VARIABLE),
            replayer,
            simulator,
        };
        runtime.arming.reset(&mut runtime.simulator)?;

        Ok(runtime)
    }

    /// Handles one `MSFSEvent::PreUpdate` cycle.
    ///
    /// Reads current simulation time, evaluates arming transitions, and applies the
    /// resulting replay update if the scenario is active.
    ///
    /// A start transition is a `0 -> 1` armed value transition; the first running
    /// frame after that transition is marked with `started_now = true` so one-time
    /// initialization logic can run exactly once per run.
    pub fn pre_update(&mut self) -> anyhow::Result<()> {
        let simulation_time = self.simulator.simulation_time()?;
        let init_now = self.arming.ready_to_start(&mut self.simulator)?;

        match self.replayer.pre_update(init_now, simulation_time)? {
            Some(ReplayerUpdate::Running { frame, started_now }) => {
                Self::handle_running_frame(frame, started_now, &mut self.simulator)?;
            }
            Some(ReplayerUpdate::Completed) => self.stop()?,
            None => {}
        }

        Ok(())
    }

    /// Stops the active replay, flushing telemetry and resetting arming state.
    ///
    /// This method is idempotent from the perspective of runtime state; if no
    /// scenario is active, it still resets arming state and returns `Ok(())`.
    pub fn stop(&mut self) -> Result<(), GaugeError> {
        self.arming.reset(&mut self.simulator)?;
        Ok(self.replayer.reset()?)
    }

    /// Processes one running frame from the replay engine.
    ///
    /// If the frame is the first after a start transition, recording variables are
    /// validated before input/record operations are executed.
    ///
    /// `started_now` is true only for the first frame of a newly started run, and
    /// allows one-time per-run work to execute exactly once.
    ///
    /// These helper functions are defined without `&self` (static-style) because
    /// `Replayer::pre_update` returns an `InterpolationFrame` tied to mutable state
    /// inside `self.replayer`; passing the simulator explicitly avoids creating a
    /// second overlapping `&mut self` borrow in this hot path.
    fn handle_running_frame(
        mut frame: InterpolationFrame<'_>,
        started_now: bool,
        simulator: &mut S,
    ) -> Result<(), GaugeError> {
        if started_now {
            Self::validate_recordings(simulator, &frame)?;
        }

        Self::inject_inputs(simulator, &frame)?;
        Self::record_outputs(simulator, &mut frame)?;

        Ok(())
    }

    /// Validates configured recordings before a run starts.
    ///
    /// This verifies every requested recording signal is readable by the simulator:
    /// variable prefix/format is supported, required read units are present for
    /// `A:` variables, `L:` variables do not provide units, and calculator read
    /// code can be generated.
    ///
    /// Validation runs only on the first running frame after a start transition so
    /// expensive per-run checks are separated from hot per-frame logic.
    fn validate_recordings(
        simulator: &mut S,
        frame: &InterpolationFrame<'_>,
    ) -> Result<(), GaugeError> {
        for recording in frame.recordings() {
            simulator
                .validate_read(&recording.variable, recording.unit.as_deref())
                .map_err(|source| GaugeError::ValidateRecordingSignal {
                    signal: recording.name.clone(),
                    source,
                })?;
        }

        Ok(())
    }

    /// Interpolates and writes all configured input signals for this frame.
    ///
    /// Interpolation and conversion failures are surfaced as gauge-level errors.
    fn inject_inputs(simulator: &mut S, frame: &InterpolationFrame<'_>) -> Result<(), GaugeError> {
        let elapsed = frame.elapsed();
        for data_points in frame.data_points() {
            let source_value =
                data_points
                    .value_at(elapsed)
                    .map_err(|source| GaugeError::InterpolateSignal {
                        signal: data_points.signal.to_owned(),
                        source,
                    })?;

            let simulator_value =
                data_points
                    .conversion
                    .convert(source_value)
                    .map_err(|source| GaugeError::ConvertSignal {
                        signal: data_points.signal.to_owned(),
                        source,
                    })?;

            simulator
                .write(data_points.variable, simulator_value)
                .map_err(|source| GaugeError::InjectSignal {
                    signal: data_points.signal.to_owned(),
                    source,
                })?;
        }

        Ok(())
    }

    /// Samples configured recordings when they are due and writes a telemetry row.
    ///
    /// Rows are written only when at least one recording is due on this frame.
    fn record_outputs(
        simulator: &mut S,
        frame: &mut InterpolationFrame<'_>,
    ) -> Result<(), GaugeError> {
        let elapsed = frame.elapsed();
        let mut sampled_values = Vec::with_capacity(frame.recordings().len());
        let mut any_due = false;
        let (recordings, schedules) = frame.recordings_and_schedules();

        for (recording, schedule) in recordings.iter().zip(schedules.iter_mut()) {
            if !schedule.should_sample(elapsed) {
                sampled_values.push(None);
                continue;
            }

            any_due = true;
            let value = simulator
                .read(&recording.variable, recording.unit.as_deref())
                .map_err(|source| GaugeError::SampleSignal {
                    signal: recording.name.clone(),
                    source,
                })?;
            sampled_values.push(Some(value));
        }

        if any_due {
            frame.record(&sampled_values)?;
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::collections::{HashMap, VecDeque};
    use std::fs;
    use std::path::{Path, PathBuf};
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::time::Duration;

    use crate::error::{GaugeError, ReplayerError, SimulatorError};
    use crate::replayer::Replayer;
    use crate::simulator::SimulatorAdapter;

    use super::{GaugeRuntime, ARMED_VARIABLE};

    const CONFIG: &str = r#"format_version = 1
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

[record.1]
name = "elevator_position"
variable = "L:ELEVATOR_POSITION"
"#;

    const RATE_LIMITED_CONFIG: &str = r#"format_version = 1
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
name = "elevator_position"
variable = "L:ELEVATOR_POSITION"
max_sampling_rate = 1.0
"#;

    const SCENARIO: &str =
        "sidestick_pitch_position.time,sidestick_pitch_position.value\n0,0\n1,100\n2,0\n";

    static NEXT_FIXTURE_ID: AtomicU64 = AtomicU64::new(0);

    #[derive(Debug, PartialEq)]
    enum Operation {
        Write {
            variable: String,
            value: f64,
        },
        ValidateRead {
            variable: String,
            unit: Option<String>,
        },
        Read {
            variable: String,
            unit: Option<String>,
        },
    }

    #[derive(Debug)]
    enum Failure {
        SimulationTime,
        Write(String),
        ValidateRead(String),
        Read(String),
    }

    struct FakeSimulator {
        time: Duration,
        reads: HashMap<String, VecDeque<f64>>,
        operations: Vec<Operation>,
        failure: Option<Failure>,
    }

    impl FakeSimulator {
        fn new(time: Duration) -> Self {
            Self {
                time,
                reads: HashMap::new(),
                operations: Vec::new(),
                failure: None,
            }
        }

        fn queue_reads(&mut self, variable: &str, values: impl IntoIterator<Item = f64>) {
            self.reads
                .entry(variable.to_owned())
                .or_default()
                .extend(values);
        }

        fn clear_operations(&mut self) {
            self.operations.clear();
        }

        fn should_fail(&self, operation: &Failure) -> bool {
            match (&self.failure, operation) {
                (Some(Failure::SimulationTime), Failure::SimulationTime) => true,
                (Some(Failure::Write(configured)), Failure::Write(actual))
                | (Some(Failure::ValidateRead(configured)), Failure::ValidateRead(actual))
                | (Some(Failure::Read(configured)), Failure::Read(actual)) => configured == actual,
                _ => false,
            }
        }
    }

    impl SimulatorAdapter for FakeSimulator {
        fn simulation_time(&self) -> Result<Duration, SimulatorError> {
            if self.should_fail(&Failure::SimulationTime) {
                return Err(SimulatorError::SimulationTimeUnavailable);
            }
            Ok(self.time)
        }

        fn write(&mut self, variable: &str, value: f64) -> Result<(), SimulatorError> {
            self.operations.push(Operation::Write {
                variable: variable.to_owned(),
                value,
            });
            if self.should_fail(&Failure::Write(variable.to_owned())) {
                return Err(SimulatorError::CalculatorCodeWriteFailed {
                    variable: variable.to_owned(),
                    value,
                });
            }
            Ok(())
        }

        fn validate_read(
            &mut self,
            variable: &str,
            unit: Option<&str>,
        ) -> Result<(), SimulatorError> {
            self.operations.push(Operation::ValidateRead {
                variable: variable.to_owned(),
                unit: unit.map(ToOwned::to_owned),
            });
            if self.should_fail(&Failure::ValidateRead(variable.to_owned())) {
                return Err(SimulatorError::UnsupportedReadVariable {
                    variable: variable.to_owned(),
                });
            }
            Ok(())
        }

        fn read(&mut self, variable: &str, unit: Option<&str>) -> Result<f64, SimulatorError> {
            self.operations.push(Operation::Read {
                variable: variable.to_owned(),
                unit: unit.map(ToOwned::to_owned),
            });
            if self.should_fail(&Failure::Read(variable.to_owned())) {
                return Err(SimulatorError::CalculatorCodeReadFailed {
                    variable: variable.to_owned(),
                });
            }
            self.reads
                .get_mut(variable)
                .and_then(VecDeque::pop_front)
                .ok_or_else(|| SimulatorError::CalculatorCodeReadFailed {
                    variable: variable.to_owned(),
                })
        }
    }

    struct Fixture {
        directory: PathBuf,
        config_path: PathBuf,
    }

    impl Fixture {
        fn new(config: &str) -> Self {
            let id = NEXT_FIXTURE_ID.fetch_add(1, Ordering::Relaxed);
            let directory = std::env::temp_dir()
                .join(format!("replay-gauge-runtime-{}-{id}", std::process::id()));
            fs::create_dir_all(&directory)
                .unwrap_or_else(|error| panic!("failed to create fixture directory: {error}"));
            let config_path = directory.join("replayer_config.toml");
            fs::write(&config_path, config)
                .unwrap_or_else(|error| panic!("failed to write fixture config: {error}"));
            fs::write(directory.join("scenario.csv"), SCENARIO)
                .unwrap_or_else(|error| panic!("failed to write fixture scenario: {error}"));

            Self {
                directory,
                config_path,
            }
        }

        fn telemetry_contents(&self) -> String {
            let path = telemetry_path(&self.directory);
            fs::read_to_string(path)
                .unwrap_or_else(|error| panic!("failed to read telemetry fixture: {error}"))
        }
    }

    impl Drop for Fixture {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.directory);
        }
    }

    fn telemetry_path(directory: &Path) -> PathBuf {
        fs::read_dir(directory)
            .unwrap_or_else(|error| panic!("failed to list fixture directory: {error}"))
            .filter_map(Result::ok)
            .map(|entry| entry.path())
            .find(|path| {
                path.file_name()
                    .and_then(|name| name.to_str())
                    .is_some_and(|name| name.starts_with("telemetry_") && name.ends_with(".csv"))
            })
            .unwrap_or_else(|| panic!("telemetry file was not created"))
    }

    fn runtime(fixture: &Fixture, simulator: FakeSimulator) -> GaugeRuntime<FakeSimulator> {
        let replayer = Replayer::with_config_path(fixture.config_path.clone());
        GaugeRuntime::new(replayer, simulator)
            .unwrap_or_else(|error| panic!("failed to construct gauge runtime: {error}"))
    }

    fn duration(seconds: f64) -> Duration {
        Duration::try_from_secs_f64(seconds)
            .unwrap_or_else(|error| panic!("invalid test duration: {error}"))
    }

    #[test]
    fn construction_resets_arming_and_propagates_reset_failures() {
        let fixture = Fixture::new(CONFIG);
        let runtime = runtime(&fixture, FakeSimulator::new(Duration::ZERO));
        assert_eq!(
            runtime.simulator.operations,
            vec![Operation::Write {
                variable: ARMED_VARIABLE.to_owned(),
                value: 0.0,
            }]
        );

        let mut simulator = FakeSimulator::new(Duration::ZERO);
        simulator.failure = Some(Failure::Write(ARMED_VARIABLE.to_owned()));
        let result = GaugeRuntime::new(
            Replayer::with_config_path(fixture.config_path.clone()),
            simulator,
        );
        assert!(matches!(
            result,
            Err(GaugeError::Simulator(
                SimulatorError::CalculatorCodeWriteFailed { variable, value }
            )) if variable == ARMED_VARIABLE && value == 0.0
        ));
    }

    #[test]
    fn idle_frames_only_read_time_and_arming_state() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(42.0));
        simulator.queue_reads(ARMED_VARIABLE, [0.0]);
        let mut runtime = runtime(&fixture, simulator);
        runtime.simulator.clear_operations();

        runtime
            .pre_update()
            .unwrap_or_else(|error| panic!("idle update failed: {error:#}"));

        assert_eq!(
            runtime.simulator.operations,
            vec![Operation::Read {
                variable: ARMED_VARIABLE.to_owned(),
                unit: None,
            },]
        );
    }

    #[test]
    fn running_frames_validate_once_inject_before_sampling_and_ignore_disarming() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0, 0.0]);
        simulator.queue_reads("A:PLANE PITCH DEGREES", [0.25, 0.5]);
        simulator.queue_reads("L:ELEVATOR_POSITION", [0.75, 1.0]);
        let mut runtime = runtime(&fixture, simulator);
        runtime.simulator.clear_operations();

        runtime
            .pre_update()
            .unwrap_or_else(|error| panic!("arming update failed: {error:#}"));
        assert_eq!(
            runtime.simulator.operations,
            vec![
                Operation::Read {
                    variable: ARMED_VARIABLE.to_owned(),
                    unit: None,
                },
                Operation::ValidateRead {
                    variable: "A:PLANE PITCH DEGREES".to_owned(),
                    unit: Some("radians".to_owned()),
                },
                Operation::ValidateRead {
                    variable: "L:ELEVATOR_POSITION".to_owned(),
                    unit: None,
                },
                Operation::Write {
                    variable: "K:AXIS_ELEVATOR_SET".to_owned(),
                    value: 0.0,
                },
                Operation::Read {
                    variable: "A:PLANE PITCH DEGREES".to_owned(),
                    unit: Some("radians".to_owned()),
                },
                Operation::Read {
                    variable: "L:ELEVATOR_POSITION".to_owned(),
                    unit: None,
                },
            ]
        );

        runtime.simulator.clear_operations();
        runtime.simulator.time = duration(100.5);
        runtime
            .pre_update()
            .unwrap_or_else(|error| panic!("running update failed: {error:#}"));
        assert_eq!(
            runtime.simulator.operations,
            vec![
                Operation::Read {
                    variable: ARMED_VARIABLE.to_owned(),
                    unit: None,
                },
                Operation::Write {
                    variable: "K:AXIS_ELEVATOR_SET".to_owned(),
                    value: 0.5,
                },
                Operation::Read {
                    variable: "A:PLANE PITCH DEGREES".to_owned(),
                    unit: Some("radians".to_owned()),
                },
                Operation::Read {
                    variable: "L:ELEVATOR_POSITION".to_owned(),
                    unit: None,
                },
            ]
        );

        runtime.stop().unwrap();
        assert_eq!(
            fixture.telemetry_contents(),
            "pitch.time,pitch.value,elevator_position.time,elevator_position.value\n\
             0,0.25,0,0.75\n\
             0.5,0.5,0.5,1\n"
        );
    }

    #[test]
    fn rate_limited_frames_skip_sampling_and_empty_rows() {
        let fixture = Fixture::new(RATE_LIMITED_CONFIG);
        let mut simulator = FakeSimulator::new(duration(10.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0, 1.0, 1.0]);
        simulator.queue_reads("A:PLANE PITCH DEGREES", [0.1, 0.2]);
        simulator.queue_reads("L:ELEVATOR_POSITION", [0.3, 0.4]);
        let mut runtime = runtime(&fixture, simulator);

        runtime.pre_update().unwrap();
        runtime.simulator.clear_operations();
        runtime.simulator.time = duration(10.5);
        runtime.pre_update().unwrap();
        assert_eq!(
            runtime.simulator.operations,
            vec![
                Operation::Read {
                    variable: ARMED_VARIABLE.to_owned(),
                    unit: None,
                },
                Operation::Write {
                    variable: "K:AXIS_ELEVATOR_SET".to_owned(),
                    value: 0.5,
                },
            ]
        );

        runtime.simulator.time = duration(11.0);
        runtime.pre_update().unwrap();
        runtime.stop().unwrap();
        assert_eq!(
            fixture.telemetry_contents(),
            "pitch.time,pitch.value,elevator_position.time,elevator_position.value\n\
             0,0.1,0,0.3\n\
             1,0.2,1,0.4\n"
        );
    }

    #[test]
    fn completion_stops_and_repeated_stop_calls_remain_safe() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(20.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0, 1.0]);
        simulator.queue_reads("A:PLANE PITCH DEGREES", [0.1]);
        simulator.queue_reads("L:ELEVATOR_POSITION", [0.2]);
        let mut runtime = runtime(&fixture, simulator);
        runtime.pre_update().unwrap();
        runtime.simulator.clear_operations();

        runtime.simulator.time = duration(22.1);
        runtime.pre_update().unwrap();

        assert_eq!(
            runtime.simulator.operations,
            vec![
                Operation::Read {
                    variable: ARMED_VARIABLE.to_owned(),
                    unit: None,
                },
                Operation::Write {
                    variable: ARMED_VARIABLE.to_owned(),
                    value: 0.0,
                },
            ]
        );
        runtime.stop().unwrap();
        assert_eq!(
            runtime.simulator.operations.last(),
            Some(&Operation::Write {
                variable: ARMED_VARIABLE.to_owned(),
                value: 0.0,
            })
        );
        assert_eq!(
            fixture.telemetry_contents(),
            "pitch.time,pitch.value,elevator_position.time,elevator_position.value\n\
             0,0.1,0,0.2\n"
        );
    }

    #[test]
    fn rejects_a_second_arming_edge_while_running() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(30.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0, 0.0, 1.0]);
        simulator.queue_reads("A:PLANE PITCH DEGREES", [0.1, 0.2]);
        simulator.queue_reads("L:ELEVATOR_POSITION", [0.3, 0.4]);
        let mut runtime = runtime(&fixture, simulator);
        runtime.pre_update().unwrap();
        runtime.simulator.time = duration(30.25);
        runtime.pre_update().unwrap();
        runtime.simulator.clear_operations();
        runtime.simulator.time = duration(30.5);

        let error = runtime
            .pre_update()
            .expect_err("overlapping replay should fail");

        assert!(matches!(
            error.downcast_ref::<ReplayerError>(),
            Some(ReplayerError::ScenarioAlreadyLoaded)
        ));
        assert_eq!(
            runtime.simulator.operations,
            vec![Operation::Read {
                variable: ARMED_VARIABLE.to_owned(),
                unit: None,
            }]
        );
        runtime.stop().unwrap();
    }

    #[test]
    fn recording_validation_failures_include_the_signal_and_prevent_injection() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(40.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        simulator.failure = Some(Failure::ValidateRead("A:PLANE PITCH DEGREES".to_owned()));
        let mut runtime = runtime(&fixture, simulator);
        runtime.simulator.clear_operations();

        let error = runtime
            .pre_update()
            .expect_err("recording validation should fail");

        assert!(matches!(
            error.downcast_ref::<GaugeError>(),
            Some(GaugeError::ValidateRecordingSignal { signal, .. }) if signal == "pitch"
        ));
        assert_eq!(
            runtime.simulator.operations,
            vec![
                Operation::Read {
                    variable: ARMED_VARIABLE.to_owned(),
                    unit: None,
                },
                Operation::ValidateRead {
                    variable: "A:PLANE PITCH DEGREES".to_owned(),
                    unit: Some("radians".to_owned()),
                },
            ]
        );
        runtime.simulator.failure = None;
        runtime.stop().unwrap();
        assert_eq!(
            fixture.telemetry_contents(),
            "pitch.time,pitch.value,elevator_position.time,elevator_position.value\n"
        );
    }

    #[test]
    fn injection_failures_include_the_signal_and_prevent_sampling() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(50.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        simulator.failure = Some(Failure::Write("K:AXIS_ELEVATOR_SET".to_owned()));
        let mut runtime = runtime(&fixture, simulator);
        runtime.simulator.clear_operations();

        let error = runtime
            .pre_update()
            .expect_err("input injection should fail");

        assert!(matches!(
            error.downcast_ref::<GaugeError>(),
            Some(GaugeError::InjectSignal { signal, .. })
                if signal == "sidestick_pitch_position"
        ));
        assert!(runtime.simulator.operations.iter().all(|operation| {
            !matches!(
                operation,
                Operation::Read { variable, .. }
                    if variable == "A:PLANE PITCH DEGREES"
                        || variable == "L:ELEVATOR_POSITION"
            )
        }));
        runtime.simulator.failure = None;
        runtime.stop().unwrap();
    }

    #[test]
    fn sampling_failures_include_the_signal_after_input_injection() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(60.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        simulator.failure = Some(Failure::Read("A:PLANE PITCH DEGREES".to_owned()));
        let mut runtime = runtime(&fixture, simulator);
        runtime.simulator.clear_operations();

        let error = runtime
            .pre_update()
            .expect_err("telemetry sampling should fail");

        assert!(matches!(
            error.downcast_ref::<GaugeError>(),
            Some(GaugeError::SampleSignal { signal, .. }) if signal == "pitch"
        ));
        let injection_index = runtime
            .simulator
            .operations
            .iter()
            .position(|operation| matches!(operation, Operation::Write { variable, .. } if variable == "K:AXIS_ELEVATOR_SET"))
            .expect("input was not injected");
        let sampling_index = runtime
            .simulator
            .operations
            .iter()
            .position(|operation| matches!(operation, Operation::Read { variable, .. } if variable == "A:PLANE PITCH DEGREES"))
            .expect("recording was not sampled");
        assert!(injection_index < sampling_index);
        runtime.simulator.failure = None;
        runtime.stop().unwrap();
    }
}
