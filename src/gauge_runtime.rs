//! MSFS-specific replay runtime used by the gauge entry point.

use crate::aircraft_initialisation::{AircraftInitialiser, AircraftSupport};
use crate::arm::ArmingMonitor;
use crate::cursor::Frame;
use crate::error::GaugeError;
use crate::initialisation::Initialisation;
use crate::replayer::{InterpolationFrame, Replayer, ReplayerUpdate};
use crate::simulator::SimulatorAdapter;

/// Local simulator variable that arms replay start when it transitions to `1`.
const ARMED_VARIABLE: &str = "L:REPLAYER_ARMED";

/// Owns the MSFS variables and replay state used by the gauge event loop.
pub struct GaugeRuntime {
    /// Replay orchestrator.
    replayer: Replayer,
    /// Tracks arming transitions and writes reset state.
    arming: ArmingMonitor,
    /// Adapter around msfs-rs legacy calculator code.
    simulator: Box<dyn SimulatorAdapter>,
    /// Aircraft loading operations, independent of generic simulator I/O.
    aircraft_initialiser: Box<dyn AircraftInitialiser>,
    /// Readiness and timeout state while the prepared scenario waits to start.
    initialisation: Option<Initialisation>,
    /// Reusable converted injection values for the current frame.
    injected_values: Vec<f64>,
    /// Reusable sampled telemetry values for the current frame.
    recorded_values: Vec<Option<f64>>,
}

impl GaugeRuntime {
    /// Creates a runtime from explicit replay, simulator and aircraft components.
    ///
    /// # Arguments
    ///
    /// * `replayer` - Parsed replay state machine driving scenario playback.
    /// * `simulator` - MSFS adapter used for all per-frame I/O.
    /// * `aircraft_initialiser` - Aircraft loading operations, substitutable for tests.
    pub fn new(
        replayer: Replayer,
        simulator: Box<dyn SimulatorAdapter>,
        aircraft_initialiser: Box<dyn AircraftInitialiser>,
    ) -> Result<Self, GaugeError> {
        let mut runtime = Self {
            arming: ArmingMonitor::new(ARMED_VARIABLE),
            replayer,
            simulator,
            aircraft_initialiser,
            initialisation: None,
            injected_values: Vec::new(),
            recorded_values: Vec::new(),
        };
        runtime.arming.reset(runtime.simulator.as_mut())?;

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
        let init_now = self.arming.ready_to_start(self.simulator.as_mut())?;

        if init_now {
            if let Some(targets) = self.replayer.prepare_scenario()? {
                match self.aircraft_initialiser.detect(self.simulator.as_mut()) {
                    AircraftSupport::Supported => {
                        self.initialisation = Some(Initialisation::new(targets, simulation_time));
                        self.aircraft_initialiser
                            .submit(self.simulator.as_mut(), targets)?;
                    }
                    AircraftSupport::Unsupported => {
                        println!(
                            "TESTPILOT: initialisation skipped: unsupported or unidentified aircraft"
                        );
                        self.replayer.start_prepared(simulation_time)?;
                    }
                }
            } else {
                self.replayer.start_prepared(simulation_time)?;
            }
        }
        if let Some(gate) = &mut self.initialisation {
            gate.check_deadline(simulation_time)?;
            let actual = self
                .aircraft_initialiser
                .readback(self.simulator.as_mut())?;
            if !gate.observe(actual)? {
                return Ok(());
            }
            self.initialisation = None;
            self.replayer.start_prepared(simulation_time)?;
        }

        match self.replayer.pre_update(simulation_time)? {
            Some(ReplayerUpdate::Running { frame, started_now }) => {
                let simulator = self.simulator.as_mut();
                let injected_values = &mut self.injected_values;
                let recorded_values = &mut self.recorded_values;
                Self::handle_running_frame(
                    simulator,
                    injected_values,
                    recorded_values,
                    frame,
                    started_now,
                )?;
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
        self.initialisation = None;
        let replay_result = self.replayer.reset();
        let arming_result = self.arming.reset(self.simulator.as_mut());
        if let Err(error) = &arming_result {
            println!("TESTPILOT ERROR: arming reset failed: {error}");
        }
        replay_result?;
        arming_result?;
        Ok(())
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
    ///
    /// # Arguments
    ///
    /// * `simulator` - Mutable simulator adapter used for this frame.
    /// * `injected_values` - Reusable buffer where converted injection values for this
    ///   frame are written.
    /// * `recorded_values` - Reusable buffer of optional recorded values for this frame.
    /// * `frame` - Active interpolation/sampling context.
    /// * `started_now` - `true` only on the first frame of a newly started run.
    fn handle_running_frame(
        simulator: &mut dyn SimulatorAdapter,
        injected_values: &mut Vec<f64>,
        recorded_values: &mut Vec<Option<f64>>,
        mut frame: InterpolationFrame<'_>,
        started_now: bool,
    ) -> Result<(), GaugeError> {
        if started_now {
            Self::validate_recordings(simulator, &frame)?;
        }

        Self::inject_inputs(injected_values, simulator, &frame)?;
        Self::record_outputs(recorded_values, simulator, injected_values, &mut frame)?;

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
    ///
    /// # Arguments
    ///
    /// * `simulator` - Mutable simulator adapter used to validate each recording
    ///   variable contract.
    /// * `frame` - Frame context exposing configured recordings and telemetry signal
    ///   metadata.
    fn validate_recordings(
        simulator: &mut dyn SimulatorAdapter,
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
    ///
    /// # Arguments
    ///
    /// * `injected_values` - Reusable mutable buffer filled with per-signal simulator
    ///   values after interpolation and conversion.
    /// * `simulator` - Mutable simulator adapter that receives each converted write.
    /// * `frame` - Interpolation context providing elapsed time and input data.
    fn inject_inputs(
        injected_values: &mut Vec<f64>,
        simulator: &mut dyn SimulatorAdapter,
        frame: &InterpolationFrame<'_>,
    ) -> Result<(), GaugeError> {
        let elapsed = frame.elapsed();
        let injection_count = frame.injection_count();
        if injected_values.len() != injection_count {
            injected_values.resize(injection_count, 0.0);
        }
        injected_values
            .iter_mut()
            .zip(frame.data_points())
            .try_for_each(|(slot, data_points): (&mut f64, Frame<'_>)| {
                let source_value = data_points.value_at(elapsed).map_err(|source| {
                    GaugeError::InterpolateSignal {
                        signal: data_points.signal.to_owned(),
                        source,
                    }
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
                *slot = simulator_value;
                Ok::<(), GaugeError>(())
            })?;

        injected_values.truncate(injection_count);
        Ok(())
    }

    /// Samples configured recordings when they are due and writes a telemetry row.
    ///
    /// Rows are written when at least one recording is due on this frame, or every
    /// frame when no recordings are configured, to retain the injected values.
    ///
    /// # Arguments
    ///
    /// * `recorded_values` - Reusable mutable buffer of sampled values for this frame.
    /// * `simulator` - Mutable simulator adapter used for telemetry reads.
    /// * `injected_values` - Per-frame converted input values serialized with the same
    ///   telemetry row.
    /// * `frame` - Mutable frame context containing recording schedules and recorder.
    fn record_outputs(
        recorded_values: &mut Vec<Option<f64>>,
        simulator: &mut dyn SimulatorAdapter,
        injected_values: &[f64],
        frame: &mut InterpolationFrame<'_>,
    ) -> Result<(), GaugeError> {
        let elapsed = frame.elapsed();
        let mut any_due = false;
        let (recordings, schedules) = frame.recordings_and_schedules();
        let recording_count = recordings.len();
        if recorded_values.len() != recording_count {
            recorded_values.resize(recording_count, None);
        } else {
            recorded_values.fill(None);
        }

        for (index, (recording, schedule)) in
            recordings.iter().zip(schedules.iter_mut()).enumerate()
        {
            if !schedule.should_sample(elapsed) {
                continue;
            }

            any_due = true;
            let value = simulator
                .read(&recording.variable, recording.unit.as_deref())
                .map_err(|source| GaugeError::SampleSignal {
                    signal: recording.name.clone(),
                    source,
                })?;
            recorded_values[index] = Some(value);
        }

        if any_due || recording_count == 0 {
            frame.record(recorded_values, injected_values)?;
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::collections::{HashMap, VecDeque};
    use std::fs;
    use std::path::{Path, PathBuf};
    use std::rc::Rc;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::time::Duration;

    use crate::aircraft_initialisation::{A32nxInitialiser, AircraftSupport};
    use crate::config::InitialisationConfig;
    use crate::error::InitialisationError;
    use crate::error::{GaugeError, ReplayerError, SimulatorError};
    use crate::initialisation::AircraftMassBalance;
    use crate::replayer::Replayer;
    use crate::simulator::SimulatorAdapter;

    use super::{ARMED_VARIABLE, GaugeRuntime};

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

    const INJECTED_VALUES_ONLY_CONFIG: &str = r#"format_version = 1
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
"#;

    const SCENARIO: &str =
        "sidestick_pitch_position.time,sidestick_pitch_position.value\n0,0\n1,100\n2,0\n";

    static NEXT_FIXTURE_ID: AtomicU64 = AtomicU64::new(0);

    #[derive(Debug, PartialEq)]
    enum Operation {
        DetectAircraft,
        ReadString(String),
        LocalVariableExists(String),
        Initialise(InitialisationConfig),
        ReadMassBalance,
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
        LocalVariableExists(String),
    }

    struct FakeSimulator {
        time: Duration,
        reads: HashMap<String, VecDeque<f64>>,
        operations: Vec<Operation>,
        failure: Option<Failure>,
        string_reads: VecDeque<Result<String, SimulatorError>>,
    }

    impl FakeSimulator {
        fn new(time: Duration) -> Self {
            Self {
                time,
                reads: HashMap::new(),
                operations: Vec::new(),
                failure: None,
                string_reads: VecDeque::new(),
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
                | (
                    Some(Failure::LocalVariableExists(configured)),
                    Failure::LocalVariableExists(actual),
                )
                | (Some(Failure::Read(configured)), Failure::Read(actual)) => configured == actual,
                _ => false,
            }
        }
    }

    #[derive(Default)]
    struct FakeAircraftInitialiser {
        mass_balance: VecDeque<Result<AircraftMassBalance, InitialisationError>>,
        fail_initialisation: bool,
        unsupported: bool,
    }

    type SimulatorHandle = Rc<RefCell<FakeSimulator>>;
    type InitialiserHandle = Rc<RefCell<FakeAircraftInitialiser>>;

    struct SharedAircraftInitialiser {
        state: InitialiserHandle,
        simulator: SimulatorHandle,
    }

    impl crate::aircraft_initialisation::AircraftInitialiser for SharedAircraftInitialiser {
        fn supported_model(&mut self, _simulator: &mut dyn SimulatorAdapter) -> bool {
            self.simulator
                .borrow_mut()
                .operations
                .push(Operation::DetectAircraft);
            !self.state.borrow().unsupported
        }

        fn submit(
            &mut self,
            _simulator: &mut dyn SimulatorAdapter,
            targets: InitialisationConfig,
        ) -> Result<(), InitialisationError> {
            self.simulator
                .borrow_mut()
                .operations
                .push(Operation::Initialise(targets));
            if self.state.borrow().fail_initialisation {
                return Err(InitialisationError::Submit(
                    SimulatorError::CalculatorCodeWriteFailed {
                        variable: "L:TEST_LOADING".to_owned(),
                        value: targets.zfw,
                    },
                ));
            }
            Ok(())
        }

        fn readback(
            &mut self,
            _simulator: &mut dyn SimulatorAdapter,
        ) -> Result<AircraftMassBalance, InitialisationError> {
            self.simulator
                .borrow_mut()
                .operations
                .push(Operation::ReadMassBalance);
            self.state
                .borrow_mut()
                .mass_balance
                .pop_front()
                .expect("missing queued mass/balance readback")
        }
    }

    impl SimulatorAdapter for Rc<RefCell<FakeSimulator>> {
        fn local_variable_exists(&mut self, variable: &str) -> Result<bool, SimulatorError> {
            self.borrow_mut().local_variable_exists(variable)
        }

        fn read_string(&mut self, variable: &str) -> Result<String, SimulatorError> {
            self.borrow_mut().read_string(variable)
        }

        fn simulation_time(&self) -> Result<Duration, SimulatorError> {
            self.borrow().simulation_time()
        }

        fn write(&mut self, variable: &str, value: f64) -> Result<(), SimulatorError> {
            self.borrow_mut().write(variable, value)
        }

        fn validate_read(
            &mut self,
            variable: &str,
            unit: Option<&str>,
        ) -> Result<(), SimulatorError> {
            self.borrow_mut().validate_read(variable, unit)
        }

        fn read(&mut self, variable: &str, unit: Option<&str>) -> Result<f64, SimulatorError> {
            self.borrow_mut().read(variable, unit)
        }
    }

    impl SimulatorAdapter for FakeSimulator {
        fn local_variable_exists(&mut self, variable: &str) -> Result<bool, SimulatorError> {
            self.operations
                .push(Operation::LocalVariableExists(variable.to_owned()));
            if self.should_fail(&Failure::LocalVariableExists(variable.to_owned())) {
                return Err(SimulatorError::UnsupportedReadVariable {
                    variable: variable.to_owned(),
                });
            }
            Ok(self.reads.contains_key(variable))
        }

        fn read_string(&mut self, variable: &str) -> Result<String, SimulatorError> {
            self.operations
                .push(Operation::ReadString(variable.to_owned()));
            self.string_reads.pop_front().unwrap_or_else(|| {
                Err(SimulatorError::CalculatorCodeReadFailed {
                    variable: variable.to_owned(),
                })
            })
        }

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
            let value = self
                .reads
                .get_mut(variable)
                .and_then(VecDeque::pop_front)
                .ok_or_else(|| SimulatorError::CalculatorCodeReadFailed {
                    variable: variable.to_owned(),
                })?;
            if !value.is_finite() {
                return Err(SimulatorError::NonFiniteRead {
                    variable: variable.to_owned(),
                    value,
                });
            }
            Ok(value)
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

        fn clear_telemetry_files(&self) {
            for entry in fs::read_dir(&self.directory)
                .unwrap_or_else(|error| panic!("failed to list fixture directory: {error}"))
                .filter_map(Result::ok)
            {
                let path = entry.path();
                if path
                    .file_name()
                    .and_then(|name| name.to_str())
                    .is_some_and(|name| name.starts_with("telemetry_") && name.ends_with(".csv"))
                {
                    fs::remove_file(&path).unwrap_or_else(|error| {
                        panic!("failed to remove old telemetry file {:?}: {error}", path)
                    });
                }
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

    fn runtime(fixture: &Fixture, simulator: FakeSimulator) -> (GaugeRuntime, SimulatorHandle) {
        let (runtime, simulator, _) =
            runtime_with_initialiser(fixture, simulator, FakeAircraftInitialiser::default());
        (runtime, simulator)
    }

    fn runtime_with_initialiser(
        fixture: &Fixture,
        simulator: FakeSimulator,
        initialiser: FakeAircraftInitialiser,
    ) -> (GaugeRuntime, SimulatorHandle, InitialiserHandle) {
        let replayer = Replayer::with_config_path(fixture.config_path.clone());
        let simulator = Rc::new(RefCell::new(simulator));
        let initialiser = Rc::new(RefCell::new(initialiser));
        let runtime = GaugeRuntime::new(
            replayer,
            Box::new(Rc::clone(&simulator)),
            Box::new(SharedAircraftInitialiser {
                state: Rc::clone(&initialiser),
                simulator: Rc::clone(&simulator),
            }),
        )
        .unwrap_or_else(|error| panic!("failed to construct gauge runtime: {error}"));
        (runtime, simulator, initialiser)
    }

    fn duration(seconds: f64) -> Duration {
        Duration::try_from_secs_f64(seconds)
            .unwrap_or_else(|error| panic!("invalid test duration: {error}"))
    }

    const MATCHED: AircraftMassBalance = AircraftMassBalance {
        zfw: 60000.0,
        gw: 65000.0,
        gwcg: 25.0,
    };
    const UNMATCHED: AircraftMassBalance = AircraftMassBalance {
        gwcg: 26.0,
        ..MATCHED
    };

    fn initialisation_fixture() -> Fixture {
        let (input_config, _) = CONFIG.split_once("[record.0]").unwrap();
        Fixture::new(&format!(
            "{input_config}\n[initialisation]\nzfw = 60000\ngw = 65000\ngwcg = 25\n"
        ))
    }

    fn assert_no_telemetry(fixture: &Fixture) {
        assert!(!fs::read_dir(&fixture.directory).unwrap().any(|entry| {
            entry
                .unwrap()
                .file_name()
                .to_string_lossy()
                .starts_with("telemetry_")
        }));
    }

    fn assert_no_replay_writes(simulator: &FakeSimulator) {
        assert!(
            !simulator
                .operations
                .iter()
                .any(|operation| matches!(operation,
            Operation::Write { variable, .. } if variable != ARMED_VARIABLE))
        );
    }

    #[test]
    fn detects_aircraft_model_and_variable_presence_without_reading_readiness() {
        use crate::aircraft_initialisation::AircraftInitialiser;
        for (model, expected) in [
            ("A20N", AircraftSupport::Supported),
            ("TT:ATCCOM.AC_MODEL_A20N.0.text", AircraftSupport::Supported),
            ("  a20n  ", AircraftSupport::Supported),
            (
                " tt:atccom.ac_model_a20n.0.TEXT ",
                AircraftSupport::Supported,
            ),
            ("A320", AircraftSupport::Unsupported),
            ("A388", AircraftSupport::Unsupported),
            ("C172", AircraftSupport::Unsupported),
            ("A20NX", AircraftSupport::Unsupported),
            (
                "TT:ATCCOM.AC_MODEL_A388.0.text",
                AircraftSupport::Unsupported,
            ),
            ("Airbus A320 Neo FlyByWire", AircraftSupport::Unsupported),
            ("", AircraftSupport::Unsupported),
            (" ", AircraftSupport::Unsupported),
        ] {
            let mut simulator = FakeSimulator::new(Duration::ZERO);
            simulator.string_reads.push_back(Ok(model.to_owned()));
            simulator.queue_reads("L:A32NX_IS_READY", [0.0]);
            assert_eq!(
                A32nxInitialiser.detect(&mut simulator),
                expected,
                "model {model:?}"
            );
            let mut operations = vec![Operation::ReadString("A:ATC MODEL".to_owned())];
            if expected == AircraftSupport::Supported {
                operations.push(Operation::LocalVariableExists(
                    "L:A32NX_IS_READY".to_owned(),
                ));
            }
            assert_eq!(simulator.operations, operations);
            assert_eq!(simulator.reads["L:A32NX_IS_READY"], [0.0]);
        }
        let mut simulator = FakeSimulator::new(Duration::ZERO);
        assert_eq!(
            A32nxInitialiser.detect(&mut simulator),
            AircraftSupport::Unsupported
        );
    }

    #[test]
    fn a20n_without_ready_variable_or_with_failed_lookup_is_unsupported() {
        use crate::aircraft_initialisation::AircraftInitialiser;
        for fail in [false, true] {
            let mut simulator = FakeSimulator::new(Duration::ZERO);
            simulator.string_reads.push_back(Ok("A20N".to_owned()));
            if fail {
                simulator.queue_reads("L:A32NX_IS_READY", [1.0]);
                simulator.failure =
                    Some(Failure::LocalVariableExists("L:A32NX_IS_READY".to_owned()));
            }
            assert_eq!(
                A32nxInitialiser.detect(&mut simulator),
                AircraftSupport::Unsupported
            );
            assert_eq!(
                simulator.operations,
                [
                    Operation::ReadString("A:ATC MODEL".to_owned()),
                    Operation::LocalVariableExists("L:A32NX_IS_READY".to_owned()),
                ]
            );
        }
    }

    #[test]
    fn unsupported_or_unidentified_aircraft_start_replay_immediately() {
        for model in [Some("C172"), Some("A20N"), Some(""), None] {
            let fixture = initialisation_fixture();
            let mut simulator = FakeSimulator::new(duration(100.0));
            simulator.queue_reads(ARMED_VARIABLE, [1.0, 1.0]);
            if let Some(model) = model {
                simulator.string_reads.push_back(Ok(model.to_owned()));
            }
            let replayer = Replayer::with_config_path(fixture.config_path.clone());
            let simulator = Rc::new(RefCell::new(simulator));
            let mut runtime = GaugeRuntime::new(
                replayer,
                Box::new(Rc::clone(&simulator)),
                Box::new(A32nxInitialiser),
            )
            .unwrap();
            runtime.pre_update().unwrap(); // Unsupported aircraft must skip loading operations.
            assert!(runtime.initialisation.is_none());
            simulator.borrow_mut().time = duration(100.5);
            runtime.pre_update().unwrap();
            assert_eq!(
                simulator
                    .borrow_mut()
                    .operations
                    .iter()
                    .filter(|op| matches!(op, Operation::ReadString(_)))
                    .count(),
                1
            );
            runtime.stop().unwrap();
            assert_eq!(
                fixture.telemetry_contents(),
                "sidestick_pitch_position.time,sidestick_pitch_position.value\n0,0\n0.5,0.5\n"
            );
        }
    }

    #[test]
    fn support_is_checked_again_when_a_new_run_is_armed() {
        let fixture = initialisation_fixture();
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0, 0.0, 1.0]);
        simulator
            .string_reads
            .extend([Ok("unknown".to_owned()), Ok("A20N".to_owned())]);
        simulator.queue_reads("L:A32NX_IS_READY", [0.0]);
        let replayer = Replayer::with_config_path(fixture.config_path.clone());
        let simulator = Rc::new(RefCell::new(simulator));
        let mut runtime = GaugeRuntime::new(
            replayer,
            Box::new(Rc::clone(&simulator)),
            Box::new(A32nxInitialiser),
        )
        .unwrap();
        runtime.pre_update().unwrap();
        runtime.stop().unwrap();
        fixture.clear_telemetry_files();
        runtime.pre_update().unwrap();
        let error = runtime.pre_update().unwrap_err();
        assert!(matches!(
            error.downcast_ref::<InitialisationError>(),
            Some(InitialisationError::NotImplemented {
                operation: "submission"
            })
        ));
        assert_eq!(
            simulator
                .borrow_mut()
                .operations
                .iter()
                .filter(|op| matches!(op, Operation::ReadString(_)))
                .count(),
            2
        );
        runtime.stop().unwrap();
        assert_no_telemetry(&fixture);
    }

    #[test]
    fn unsupported_detection_does_not_call_loading_operations() {
        let fixture = initialisation_fixture();
        let initialiser = FakeAircraftInitialiser {
            unsupported: true,
            fail_initialisation: true,
            ..Default::default()
        };
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        let (mut runtime, simulator, _) =
            runtime_with_initialiser(&fixture, simulator, initialiser);
        runtime.pre_update().unwrap();
        assert!(
            !simulator
                .borrow_mut()
                .operations
                .iter()
                .any(|op| matches!(op, Operation::Initialise(_) | Operation::ReadMassBalance))
        );
        runtime.stop().unwrap();
        assert!(fixture.telemetry_contents().ends_with("\n0,0\n"));
    }

    #[test]
    fn a32nx_readback_reads_fresh_actual_values_in_native_units_without_writes() {
        use crate::aircraft_initialisation::AircraftInitialiser;

        let mut simulator = FakeSimulator::new(Duration::ZERO);
        simulator.queue_reads("L:A32NX_AIRFRAME_ZFW", [60000.0, 60100.0]);
        simulator.queue_reads("L:A32NX_AIRFRAME_GW", [65000.0, 65100.0]);
        simulator.queue_reads("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", [25.0, 25.01]);
        for expected in [
            MATCHED,
            AircraftMassBalance {
                zfw: 60100.0,
                gw: 65100.0,
                gwcg: 25.01,
            },
        ] {
            simulator.clear_operations();
            assert_eq!(A32nxInitialiser.readback(&mut simulator).unwrap(), expected);
            assert_eq!(
                simulator.operations,
                [
                    "L:A32NX_AIRFRAME_ZFW",
                    "L:A32NX_AIRFRAME_GW",
                    "L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC",
                ]
                .map(|variable| Operation::Read {
                    variable: variable.to_owned(),
                    unit: None,
                })
            );
        }
    }

    #[test]
    fn a32nx_readback_reports_each_failed_variable_and_stops_reading() {
        use crate::aircraft_initialisation::AircraftInitialiser;

        let variables = [
            "L:A32NX_AIRFRAME_ZFW",
            "L:A32NX_AIRFRAME_GW",
            "L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC",
        ];
        for (failed_index, failed_variable) in variables.iter().enumerate() {
            // None exercises an SDK read failure; non-finite values exercise
            // the SimulatorAdapter finite-value contract used by MsfsSimulator.
            for invalid in [
                None,
                Some(f64::NAN),
                Some(f64::INFINITY),
                Some(f64::NEG_INFINITY),
            ] {
                let mut simulator = FakeSimulator::new(Duration::ZERO);
                for (variable, actual) in variables.into_iter().zip([60000.0, 65000.0, 25.0]) {
                    let value = if variable == *failed_variable {
                        invalid.unwrap_or(actual)
                    } else {
                        actual
                    };
                    simulator.queue_reads(variable, [value]);
                }
                if invalid.is_none() {
                    simulator.failure = Some(Failure::Read((*failed_variable).to_owned()));
                }
                let error = A32nxInitialiser.readback(&mut simulator).unwrap_err();
                match (invalid, error) {
                    (
                        None,
                        InitialisationError::Readback(SimulatorError::CalculatorCodeReadFailed {
                            variable,
                        }),
                    )
                    | (
                        Some(_),
                        InitialisationError::Readback(SimulatorError::NonFiniteRead {
                            variable,
                            ..
                        }),
                    ) => {
                        assert_eq!(variable, *failed_variable);
                    }
                    unexpected => panic!("unexpected readback error: {unexpected:?}"),
                }
                assert_eq!(simulator.operations.len(), failed_index + 1);
            }
        }
    }

    #[test]
    fn a32nx_submission_stub_fails_safely_without_replay_or_telemetry() {
        let fixture = initialisation_fixture();
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        simulator
            .string_reads
            .push_back(Ok("TT:ATCCOM.AC_MODEL_A20N.0.text".to_owned()));
        simulator.queue_reads("L:A32NX_IS_READY", [1.0]);
        let replayer = Replayer::with_config_path(fixture.config_path.clone());
        let simulator = Rc::new(RefCell::new(simulator));
        let mut runtime = GaugeRuntime::new(
            replayer,
            Box::new(Rc::clone(&simulator)),
            Box::new(A32nxInitialiser),
        )
        .unwrap();
        let error = runtime.pre_update().unwrap_err();
        assert!(matches!(
            error.downcast_ref::<InitialisationError>(),
            Some(InitialisationError::NotImplemented {
                operation: "submission"
            })
        ));
        runtime.stop().unwrap();
        assert!(runtime.initialisation.is_none());
        assert_no_replay_writes(&simulator.borrow());
        assert_no_telemetry(&fixture);
    }

    #[test]
    fn initialisation_waits_without_output_and_starts_at_zero_after_simultaneous_readiness() {
        let fixture = initialisation_fixture();
        let mut initialiser = FakeAircraftInitialiser::default();
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0; 5]);
        initialiser.mass_balance.extend([
            Ok(AircraftMassBalance {
                zfw: 61000.0,
                ..MATCHED
            }),
            Ok(AircraftMassBalance {
                gw: 66000.0,
                ..MATCHED
            }),
            Ok(UNMATCHED),
            Ok(MATCHED),
        ]);
        let (mut runtime, simulator, _) =
            runtime_with_initialiser(&fixture, simulator, initialiser);
        for now in [100.0, 110.0, 120.0] {
            simulator.borrow_mut().time = duration(now);
            runtime.pre_update().unwrap();
            assert_no_telemetry(&fixture);
            assert_no_replay_writes(&simulator.borrow());
        }
        simulator.borrow_mut().time = duration(129.999);
        runtime.pre_update().unwrap();
        simulator.borrow_mut().time = duration(130.499);
        runtime.pre_update().unwrap();
        assert_eq!(
            simulator
                .borrow_mut()
                .operations
                .iter()
                .filter(|op| matches!(op, Operation::Initialise(_)))
                .count(),
            1
        );
        assert_eq!(
            simulator
                .borrow_mut()
                .operations
                .iter()
                .filter(|op| matches!(op, Operation::ReadMassBalance))
                .count(),
            4
        );
        assert!(
            simulator
                .borrow_mut()
                .operations
                .contains(&Operation::Initialise(InitialisationConfig {
                    zfw: 60000.0,
                    gw: 65000.0,
                    gwcg: 25.0
                }))
        );
        runtime.stop().unwrap();
        assert_eq!(
            fixture.telemetry_contents(),
            "sidestick_pitch_position.time,sidestick_pitch_position.value\n0,0\n0.5,0.5\n"
        );
    }

    #[test]
    fn initialisation_can_start_on_the_arm_frame() {
        let fixture = initialisation_fixture();
        let mut initialiser = FakeAircraftInitialiser::default();
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        initialiser.mass_balance.push_back(Ok(MATCHED));
        let (mut runtime, simulator, _) =
            runtime_with_initialiser(&fixture, simulator, initialiser);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert!(matches!(
            simulator.borrow().operations.as_slice(),
            [
                Operation::Read { .. },
                Operation::DetectAircraft,
                Operation::Initialise(_),
                Operation::ReadMassBalance,
                Operation::Write { value: 0.0, .. }
            ]
        ));
        runtime.stop().unwrap();
        assert!(fixture.telemetry_contents().ends_with("\n0,0\n"));
    }

    #[test]
    fn initialisation_timeout_precedes_readiness_at_and_after_deadline() {
        for now in [130.0, 135.0] {
            let fixture = initialisation_fixture();
            let mut initialiser = FakeAircraftInitialiser::default();
            let mut simulator = FakeSimulator::new(duration(100.0));
            simulator.queue_reads(ARMED_VARIABLE, [1.0, 1.0]);
            initialiser
                .mass_balance
                .extend([Ok(UNMATCHED), Ok(MATCHED)]);
            let (mut runtime, simulator, initialiser) =
                runtime_with_initialiser(&fixture, simulator, initialiser);
            runtime.pre_update().unwrap();
            simulator.borrow_mut().time = duration(now);
            let error = runtime.pre_update().unwrap_err();
            assert!(matches!(
                error.downcast_ref::<InitialisationError>(),
                Some(InitialisationError::Timeout {
                    latest: Some(UNMATCHED),
                    ..
                })
            ));
            assert_eq!(
                initialiser.borrow().mass_balance.len(),
                1,
                "deadline must be checked before readback"
            );
            assert_no_replay_writes(&simulator.borrow());
            runtime.stop().unwrap();
            runtime.stop().unwrap();
            assert_no_telemetry(&fixture);
            assert_eq!(
                simulator.borrow().operations.last(),
                Some(&Operation::Write {
                    variable: ARMED_VARIABLE.to_owned(),
                    value: 0.0
                })
            );
        }
    }

    #[test]
    fn initialisation_failures_are_typed_and_cleanup_creates_no_telemetry() {
        for case in 0..4 {
            let fixture = initialisation_fixture();
            let mut initialiser = FakeAircraftInitialiser::default();
            let mut simulator = FakeSimulator::new(duration(100.0));
            simulator.queue_reads(ARMED_VARIABLE, [1.0, 1.0]);
            match case {
                0 => initialiser.fail_initialisation = true,
                1 => initialiser
                    .mass_balance
                    .push_back(Err(InitialisationError::Readback(
                        SimulatorError::CalculatorCodeReadFailed {
                            variable: "L:TEST_LOADING".to_owned(),
                        },
                    ))),
                2 => initialiser.mass_balance.push_back(Ok(AircraftMassBalance {
                    gw: f64::NAN,
                    ..MATCHED
                })),
                _ => initialiser.mass_balance.push_back(Ok(UNMATCHED)),
            }
            let (mut runtime, simulator, _) =
                runtime_with_initialiser(&fixture, simulator, initialiser);
            if case == 3 {
                runtime.pre_update().unwrap();
                simulator.borrow_mut().time = duration(99.0);
            }
            let error = runtime.pre_update().unwrap_err();
            let error = error.downcast_ref::<InitialisationError>().unwrap();
            assert!(matches!(
                (case, error),
                (0, InitialisationError::Submit(_))
                    | (1, InitialisationError::Readback(_))
                    | (2, InitialisationError::NonFiniteReadback { .. })
                    | (3, InitialisationError::ClockMovedBackwards { .. })
            ));
            runtime.stop().unwrap();
            assert_no_replay_writes(&simulator.borrow());
            assert_no_telemetry(&fixture);
        }
    }

    #[test]
    fn initialisation_rejects_overlapping_arming_and_can_be_cleaned_up_while_waiting() {
        let fixture = initialisation_fixture();
        let mut initialiser = FakeAircraftInitialiser::default();
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0, 0.0, 1.0]);
        initialiser
            .mass_balance
            .extend([Ok(UNMATCHED), Ok(UNMATCHED)]);
        let (mut runtime, simulator, initialiser) =
            runtime_with_initialiser(&fixture, simulator, initialiser);
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap(); // Disarming is not an abort.
        let error = runtime.pre_update().unwrap_err();
        assert_eq!(
            error.downcast_ref::<ReplayerError>(),
            Some(&ReplayerError::ScenarioAlreadyLoaded)
        );
        assert_eq!(
            simulator
                .borrow_mut()
                .operations
                .iter()
                .filter(|op| matches!(op, Operation::Initialise(_)))
                .count(),
            1
        );
        runtime.stop().unwrap();
        runtime.stop().unwrap();
        assert_no_telemetry(&fixture);
        assert_no_replay_writes(&simulator.borrow());
        simulator
            .borrow_mut()
            .queue_reads(ARMED_VARIABLE, [0.0, 1.0]);
        initialiser.borrow_mut().mass_balance.push_back(Ok(MATCHED));
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        runtime.stop().unwrap();
        assert!(fixture.telemetry_contents().ends_with("\n0,0\n"));
    }

    #[test]
    fn construction_resets_arming_and_propagates_reset_failures() {
        let fixture = Fixture::new(CONFIG);
        let (_runtime, simulator) = runtime(&fixture, FakeSimulator::new(Duration::ZERO));
        assert_eq!(
            simulator.borrow().operations,
            vec![Operation::Write {
                variable: ARMED_VARIABLE.to_owned(),
                value: 0.0,
            }]
        );

        let mut simulator = FakeSimulator::new(Duration::ZERO);
        simulator.failure = Some(Failure::Write(ARMED_VARIABLE.to_owned()));
        let result = GaugeRuntime::new(
            Replayer::with_config_path(fixture.config_path.clone()),
            Box::new(simulator),
            Box::new(A32nxInitialiser),
        );
        match result {
            Err(GaugeError::Simulator(SimulatorError::CalculatorCodeWriteFailed {
                variable,
                value,
            })) if variable == ARMED_VARIABLE && value == 0.0 => {}
            _ => panic!("expected arming write failure"),
        }
    }

    #[test]
    fn idle_frames_only_read_time_and_arming_state() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(42.0));
        simulator.queue_reads(ARMED_VARIABLE, [0.0]);
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        simulator.borrow_mut().clear_operations();

        runtime
            .pre_update()
            .unwrap_or_else(|error| panic!("idle update failed: {error:#}"));

        assert_eq!(
            simulator.borrow().operations,
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
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        simulator.borrow_mut().clear_operations();

        runtime
            .pre_update()
            .unwrap_or_else(|error| panic!("arming update failed: {error:#}"));
        assert_eq!(
            simulator.borrow().operations,
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

        simulator.borrow_mut().clear_operations();
        simulator.borrow_mut().time = duration(100.5);
        runtime
            .pre_update()
            .unwrap_or_else(|error| panic!("running update failed: {error:#}"));
        assert_eq!(
            simulator.borrow().operations,
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
            "pitch.time,pitch.value,elevator_position.time,elevator_position.value,sidestick_pitch_position.time,sidestick_pitch_position.value\n\
             0,0.25,0,0.75,0,0\n\
             0.5,0.5,0.5,1,0.5,0.5\n"
        );
    }

    #[test]
    fn replays_without_recordings_and_logs_injected_values() {
        let (injections_only, _) = CONFIG.split_once("[record.0]").unwrap();
        for suffix in ["", "[record]\n"] {
            let fixture = Fixture::new(&format!("{injections_only}{suffix}"));
            let mut simulator = FakeSimulator::new(duration(100.0));
            simulator.queue_reads(ARMED_VARIABLE, [1.0, 1.0, 1.0, 1.0, 1.0, 0.0]);
            let (mut runtime, simulator) = runtime(&fixture, simulator);

            for (elapsed, value) in [(0.0, 0.0), (0.5, 0.5), (1.0, 1.0), (1.5, 0.5)] {
                simulator.borrow_mut().clear_operations();
                simulator.borrow_mut().time = duration(100.0 + elapsed);
                runtime.pre_update().unwrap();
                // Exact operations also exclude recording validation and sampling.
                assert_eq!(
                    simulator.borrow().operations,
                    vec![
                        Operation::Read {
                            variable: ARMED_VARIABLE.to_owned(),
                            unit: None,
                        },
                        Operation::Write {
                            variable: "K:AXIS_ELEVATOR_SET".to_owned(),
                            value,
                        },
                    ]
                );
            }

            simulator.borrow_mut().clear_operations();
            simulator.borrow_mut().time = duration(102.1);
            runtime.pre_update().unwrap();
            assert_eq!(
                simulator.borrow().operations,
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
            let contents = fixture.telemetry_contents();
            assert_eq!(
                contents,
                "sidestick_pitch_position.time,sidestick_pitch_position.value\n\
                 0,0\n0.5,0.5\n1,1\n1.5,0.5\n"
            );

            simulator.borrow_mut().clear_operations();
            simulator.borrow_mut().time = duration(103.0);
            runtime.pre_update().unwrap();
            assert_eq!(
                simulator.borrow().operations,
                vec![Operation::Read {
                    variable: ARMED_VARIABLE.to_owned(),
                    unit: None,
                }]
            );
            assert_eq!(fixture.telemetry_contents(), contents);
        }
    }

    #[test]
    fn rate_limited_frames_skip_sampling_and_empty_rows() {
        let fixture = Fixture::new(RATE_LIMITED_CONFIG);
        let mut simulator = FakeSimulator::new(duration(10.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0, 1.0, 1.0]);
        simulator.queue_reads("A:PLANE PITCH DEGREES", [0.1, 0.2]);
        simulator.queue_reads("L:ELEVATOR_POSITION", [0.3, 0.4]);
        let (mut runtime, simulator) = runtime(&fixture, simulator);

        runtime.pre_update().unwrap();
        simulator.borrow_mut().clear_operations();
        simulator.borrow_mut().time = duration(10.5);
        runtime.pre_update().unwrap();
        assert_eq!(
            simulator.borrow().operations,
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

        simulator.borrow_mut().time = duration(11.0);
        runtime.pre_update().unwrap();
        runtime.stop().unwrap();
        assert_eq!(
            fixture.telemetry_contents(),
            "pitch.time,pitch.value,elevator_position.time,elevator_position.value,sidestick_pitch_position.time,sidestick_pitch_position.value\n\
             0,0.1,0,0.3,0,0\n\
             1,0.2,1,0.4,1,1\n"
        );
    }

    #[test]
    fn injected_values_are_written_to_telemetry() {
        let fixture = Fixture::new(INJECTED_VALUES_ONLY_CONFIG);
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        simulator.queue_reads("A:PLANE PITCH DEGREES", [0.25]);
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        simulator.borrow_mut().clear_operations();

        runtime
            .pre_update()
            .unwrap_or_else(|error| panic!("arming update failed: {error:#}"));

        assert_eq!(
            simulator.borrow().operations,
            vec![
                Operation::Read {
                    variable: ARMED_VARIABLE.to_owned(),
                    unit: None,
                },
                Operation::ValidateRead {
                    variable: "A:PLANE PITCH DEGREES".to_owned(),
                    unit: Some("radians".to_owned()),
                },
                Operation::Write {
                    variable: "K:AXIS_ELEVATOR_SET".to_owned(),
                    value: 0.0,
                },
                Operation::Read {
                    variable: "A:PLANE PITCH DEGREES".to_owned(),
                    unit: Some("radians".to_owned()),
                },
            ]
        );

        runtime.stop().unwrap();
        assert_eq!(
            fixture.telemetry_contents(),
            "pitch.time,pitch.value,sidestick_pitch_position.time,sidestick_pitch_position.value\n\
             0,0.25,0,0\n"
        );
    }

    #[test]
    fn completion_stops_and_repeated_stop_calls_remain_safe() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(20.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0, 1.0]);
        simulator.queue_reads("A:PLANE PITCH DEGREES", [0.1]);
        simulator.queue_reads("L:ELEVATOR_POSITION", [0.2]);
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        runtime.pre_update().unwrap();
        simulator.borrow_mut().clear_operations();

        simulator.borrow_mut().time = duration(22.1);
        runtime.pre_update().unwrap();

        assert_eq!(
            simulator.borrow().operations,
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
            simulator.borrow().operations.last(),
            Some(&Operation::Write {
                variable: ARMED_VARIABLE.to_owned(),
                value: 0.0,
            })
        );
        assert_eq!(
            fixture.telemetry_contents(),
            "pitch.time,pitch.value,elevator_position.time,elevator_position.value,sidestick_pitch_position.time,sidestick_pitch_position.value\n\
             0,0.1,0,0.2,0,0\n"
        );
    }

    #[test]
    fn rejects_a_second_arming_edge_while_running() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(30.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0, 0.0, 1.0]);
        simulator.queue_reads("A:PLANE PITCH DEGREES", [0.1, 0.2]);
        simulator.queue_reads("L:ELEVATOR_POSITION", [0.3, 0.4]);
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        runtime.pre_update().unwrap();
        simulator.borrow_mut().time = duration(30.25);
        runtime.pre_update().unwrap();
        simulator.borrow_mut().clear_operations();
        simulator.borrow_mut().time = duration(30.5);

        let error = runtime
            .pre_update()
            .expect_err("overlapping replay should fail");

        match error.downcast_ref::<ReplayerError>() {
            Some(ReplayerError::ScenarioAlreadyLoaded) => {}
            unexpected => panic!("expected scenario already loaded error, got: {unexpected:?}"),
        }
        assert_eq!(
            simulator.borrow().operations,
            vec![Operation::Read {
                variable: ARMED_VARIABLE.to_owned(),
                unit: None,
            }]
        );
        runtime.stop().unwrap();
    }

    #[test]
    fn scenario_restart_reloads_the_updated_config() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(10.0));
        simulator.queue_reads(
            ARMED_VARIABLE,
            [
                1.0, // first start
                0.0, // clear previous edge
                1.0, // second start after stop
            ],
        );
        simulator.queue_reads("A:PLANE PITCH DEGREES", [0.25, 0.35, 0.45, 0.55]);
        simulator.queue_reads("L:ELEVATOR_POSITION", [0.5, 0.6, 0.7, 0.8]);
        let (mut runtime, simulator) = runtime(&fixture, simulator);

        runtime
            .pre_update()
            .unwrap_or_else(|error| panic!("first start failed: {error:#}"));
        assert_eq!(
            simulator.borrow()
                .operations
                .iter()
                .filter(|operation| matches!(operation, Operation::Write { variable, .. } if variable == "K:AXIS_ELEVATOR_SET"))
                .count(),
            1
        );

        runtime.stop().unwrap();
        simulator.borrow_mut().clear_operations();
        fixture.clear_telemetry_files();

        fs::write(
            &fixture.config_path,
            r#"format_version = 1
input_file = "scenario.csv"

[inject.0]
name = "sidestick_pitch_position"
variable = "K:AXIS_ELEVATOR_SET"
source_range = [-100.0, 100.0]
simulator_range = [-1.0, 1.0]

[inject.1]
name = "sidestick_roll_position"
variable = "K:AXIS_AILERONS_SET"
source_range = [-100.0, 100.0]
simulator_range = [-1.0, 1.0]

[record.0]
name = "pitch"
variable = "A:PLANE PITCH DEGREES"
unit = "radians"

[record.1]
name = "elevator_position"
variable = "L:ELEVATOR_POSITION"
"#,
        )
        .unwrap_or_else(|error| panic!("failed to rewrite fixture config: {error}"));
        fs::write(
            fixture.directory.join("scenario.csv"),
            "sidestick_pitch_position.time,sidestick_pitch_position.value,sidestick_roll_position.time,sidestick_roll_position.value\n0,0,0,0\n1,10,0.2,20\n",
        )
        .unwrap_or_else(|error| panic!("failed to rewrite fixture scenario: {error}"));

        runtime
            .pre_update()
            .unwrap_or_else(|error| panic!("first disarmed transition failed: {error:#}"));
        assert!(
            simulator
                .borrow_mut()
                .operations
                .iter()
                .all(|operation| !matches!(operation, Operation::Write { .. }))
        );

        runtime
            .pre_update()
            .unwrap_or_else(|error| panic!("second start failed: {error:#}"));
        assert!(
            simulator.borrow()
                .operations
                .iter()
                .any(|operation| matches!(operation, Operation::Write { variable, .. } if variable == "K:AXIS_ELEVATOR_SET"))
        );
        assert!(
            simulator.borrow()
                .operations
                .iter()
                .any(|operation| matches!(operation, Operation::Write { variable, .. } if variable == "K:AXIS_AILERONS_SET"))
        );
        runtime.stop().unwrap();
    }

    #[test]
    fn recording_validation_failures_include_the_signal_and_prevent_injection() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(40.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        simulator.failure = Some(Failure::ValidateRead("A:PLANE PITCH DEGREES".to_owned()));
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        simulator.borrow_mut().clear_operations();

        let error = runtime
            .pre_update()
            .expect_err("recording validation should fail");

        match error.downcast_ref::<GaugeError>() {
            Some(GaugeError::ValidateRecordingSignal { signal, .. }) if signal == "pitch" => {}
            unexpected => {
                panic!("expected recording-validation error for pitch, got: {unexpected:?}")
            }
        }
        assert_eq!(
            simulator.borrow().operations,
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
        simulator.borrow_mut().failure = None;
        runtime.stop().unwrap();
        assert_eq!(
            fixture.telemetry_contents(),
            "pitch.time,pitch.value,elevator_position.time,elevator_position.value,sidestick_pitch_position.time,sidestick_pitch_position.value\n"
        );
    }

    #[test]
    fn injection_failures_include_the_signal_and_prevent_sampling() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(50.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        simulator.failure = Some(Failure::Write("K:AXIS_ELEVATOR_SET".to_owned()));
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        simulator.borrow_mut().clear_operations();

        let error = runtime
            .pre_update()
            .expect_err("input injection should fail");

        match error.downcast_ref::<GaugeError>() {
            Some(GaugeError::InjectSignal { signal, .. })
                if signal == "sidestick_pitch_position" => {}
            unexpected => {
                panic!("expected injection error for sidestick_pitch_position, got: {unexpected:?}")
            }
        }
        assert!(simulator.borrow().operations.iter().all(|operation| {
            if let Operation::Read { variable, .. } = operation {
                !(variable == "A:PLANE PITCH DEGREES" || variable == "L:ELEVATOR_POSITION")
            } else {
                true
            }
        }));
        simulator.borrow_mut().failure = None;
        runtime.stop().unwrap();
    }

    #[test]
    fn sampling_failures_include_the_signal_after_input_injection() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(60.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        simulator.failure = Some(Failure::Read("A:PLANE PITCH DEGREES".to_owned()));
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        simulator.borrow_mut().clear_operations();

        let error = runtime
            .pre_update()
            .expect_err("telemetry sampling should fail");

        match error.downcast_ref::<GaugeError>() {
            Some(GaugeError::SampleSignal { signal, .. }) if signal == "pitch" => {}
            unexpected => panic!("expected sample error for pitch, got: {unexpected:?}"),
        }
        let injection_index = simulator
            .borrow_mut()
            .operations
            .iter()
            .position(|operation| {
                if let Operation::Write { variable, .. } = operation {
                    variable == "K:AXIS_ELEVATOR_SET"
                } else {
                    false
                }
            })
            .expect("input was not injected");
        let sampling_index = simulator
            .borrow_mut()
            .operations
            .iter()
            .position(|operation| {
                if let Operation::Read { variable, .. } = operation {
                    variable == "A:PLANE PITCH DEGREES"
                } else {
                    false
                }
            })
            .expect("recording was not sampled");
        assert!(injection_index < sampling_index);
        simulator.borrow_mut().failure = None;
        runtime.stop().unwrap();
    }

    #[test]
    fn clips_out_of_range_inputs_before_injection() {
        let fixture = Fixture::new(INJECTED_VALUES_ONLY_CONFIG);
        let mut simulator = FakeSimulator::new(duration(10.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        simulator.queue_reads("A:PLANE PITCH DEGREES", [0.0]);
        fs::write(
            fixture.directory.join("scenario.csv"),
            "sidestick_pitch_position.time,sidestick_pitch_position.value\n0,250\n1,-250\n2,0\n",
        )
        .unwrap_or_else(|error| panic!("failed to rewrite fixture scenario: {error}"));
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        simulator.borrow_mut().clear_operations();

        runtime
            .pre_update()
            .unwrap_or_else(|error| panic!("runtime should clamp out-of-range input: {error:#}"));

        let write_value = simulator
            .borrow_mut()
            .operations
            .iter()
            .find_map(|operation| match operation {
                Operation::Write {
                    variable, value, ..
                } if variable == "K:AXIS_ELEVATOR_SET" => Some(*value),
                _ => None,
            })
            .expect("sidestick injection was not written");

        assert_eq!(write_value, 1.0);
        runtime.stop().unwrap();
    }
}
