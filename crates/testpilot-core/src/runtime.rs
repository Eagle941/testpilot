//! Arming, phase transitions and cleanup coordinated from simulator frames.

use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};

use crate::aircraft_initialisation::{AircraftInitialiser, AircraftSupport};
use crate::arm::ArmingMonitor;
use crate::config::{Config, InitialisationConfig, RecordingConfig};
use crate::cursor::Scenario;
use crate::error::{
    CleanupError, InitialisationError, RecordingError, RuntimeError, SimulatorError,
    TelemetryError, TerminationError,
};
use crate::initialisation::Initialisation;
use crate::injection::InputInjector;
use crate::simulator::SimulatorAdapter;
use crate::telemetry::Telemetry;

/// Local simulator variable that arms replay on a zero-to-one transition.
const ARMED_VARIABLE: &str = "L:REPLAYER_ARMED";

// TODO: Remove the Boxed context in `RunState`.

/// The runtime is the only owner of run lifecycle transitions.
#[derive(Debug)]
enum RunState {
    /// Arming is monitored while no input files or telemetry are open.
    Idle(ArmingMonitor),
    /// Input cursors are prepared; optional aircraft readiness is checked before playback.
    Initialising(Box<InitContext>),
    /// Playback resources are ready; the first update establishes the clock.
    Running(Box<RunContext>),
}

impl Default for RunState {
    fn default() -> Self {
        Self::Idle(ArmingMonitor::new(ARMED_VARIABLE))
    }
}

/// Equality compares the lifecycle phase, regardless of its owned context.
impl PartialEq for RunState {
    fn eq(&self, other: &Self) -> bool {
        std::mem::discriminant(self) == std::mem::discriminant(other)
    }
}

/// Result of one aircraft initialisation update.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InitProgress {
    /// Configured targets have not all reached their tolerances.
    Waiting,
    /// All configured targets are ready; enter the running phase for the next frame.
    Ready,
}

/// Result of processing one simulator frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FrameOutcome {
    /// Wait for another simulator frame, whether idle, initialising or running.
    Continue,
    /// Every input series has completed.
    Complete,
}

/// Long-lived simulator services and the current phase's owned resources.
pub struct Runtime {
    /// Configuration is reloaded from this location on every arm.
    config_path: PathBuf,
    /// Generic simulator I/O and clock.
    simulator: Box<dyn SimulatorAdapter>,
    /// Aircraft detection, target submission and actual-state readback.
    aircraft_initialiser: Box<dyn AircraftInitialiser>,
    /// Exactly one idle, initialising or running phase.
    state: RunState,
}

impl Runtime {
    /// Creates an idle runtime and resets arming before accepting simulator events.
    pub fn new(
        config_path: impl Into<PathBuf>,
        simulator: Box<dyn SimulatorAdapter>,
        aircraft_initialiser: Box<dyn AircraftInitialiser>,
    ) -> Result<Self, SimulatorError> {
        let mut runtime = Self {
            config_path: config_path.into(),
            simulator,
            aircraft_initialiser,
            state: RunState::default(),
        };
        runtime.enter_idle()?;
        Ok(runtime)
    }

    /// Processes a frame, returning to idle after completion or failure.
    ///
    /// Arming is checked only while idle; changes during an active run are ignored.
    /// Errors end the current run; another run requires a new arming transition.
    pub fn pre_update(&mut self) -> anyhow::Result<()> {
        match self.advance_frame() {
            Ok(FrameOutcome::Complete) => self.stop().map_err(Into::into),
            Ok(FrameOutcome::Continue) => Ok(()),
            Err(primary) => match self.stop() {
                Ok(()) => Err(primary),
                Err(cleanup) => {
                    Err(TerminationError::CleanupAfterFailure { primary, cleanup }.into())
                }
            },
        }
    }

    /// Advances only the phase that was active at the start of this frame.
    fn advance_frame(&mut self) -> anyhow::Result<FrameOutcome> {
        let simulation_time = self.simulator.simulation_time()?;

        match &mut self.state {
            RunState::Idle(context) => {
                if context.trigger_initialise(self.simulator.as_mut())? {
                    self.start_initialising(simulation_time)?;
                }
            }
            RunState::Initialising(init_context) => {
                let progress = init_context.advance(
                    simulation_time,
                    self.aircraft_initialiser.as_mut(),
                    self.simulator.as_mut(),
                )?;
                match progress {
                    InitProgress::Waiting => {}
                    InitProgress::Ready => {
                        self.start_running()?;
                    }
                }
            }
            RunState::Running(context) => {
                return context.advance(simulation_time, self.simulator.as_mut());
            }
        }
        Ok(FrameOutcome::Continue)
    }

    /// Enters idle before resetting arming, retaining an existing context for reset retries.
    fn enter_idle(&mut self) -> Result<(), SimulatorError> {
        if !matches!(self.state, RunState::Idle(_)) {
            self.state = RunState::default();
        }
        // TODO: Refactor the function to remove the following if-statement because it will always be true.
        if let RunState::Idle(context) = &mut self.state {
            context.reset(self.simulator.as_mut())?;
        }
        Ok(())
    }

    /// Loads configuration and primes input cursors before any aircraft loading.
    fn start_initialising(&mut self, simulation_time: Duration) -> anyhow::Result<()> {
        let config = Config::read_config_file(&self.config_path)?;
        let config_directory =
            self.config_path
                .parent()
                .ok_or_else(|| RuntimeError::ConfigPathWithoutParent {
                    path: self.config_path.clone(),
                })?;
        let scenario_path = config_directory.join(&config.input_file);
        // Open and prime the scenario before aircraft setup so file, header or initial
        // sample errors fail before any mass or trim changes.
        let scenario = Scenario::new(&scenario_path, &config)?;
        let telemetry_directory = Self::telemetry_directory(&scenario_path)?;
        println!(
            "TESTPILOT: opened {} with {} signal cursors",
            scenario_path.display(),
            config.inject.len()
        );

        let init_context = InitContext::new(
            config,
            scenario,
            telemetry_directory,
            simulation_time,
            self.aircraft_initialiser.as_mut(),
            self.simulator.as_mut(),
        );
        self.state = RunState::Initialising(Box::new(init_context));
        Ok(())
    }

    /// Installs the running context before validating its recording interfaces.
    fn start_running(&mut self) -> anyhow::Result<()> {
        let state = std::mem::take(&mut self.state);
        let RunState::Initialising(init_context) = state else {
            self.state = state;
            return Err(RuntimeError::InitialisationNotActive.into());
        };
        let context: RunContext = (*init_context).try_into()?;
        println!(
            "TESTPILOT: recording telemetry to {}",
            context.telemetry.path().display()
        );
        self.state = RunState::Running(Box::new(context));
        // Install the context first so a validation failure follows the same flush/close path.
        if let RunState::Running(context) = &self.state {
            context.validate(self.simulator.as_mut())?;
        }
        Ok(())
    }

    /// Releases a run and resets arming, leaving the runtime idle even if cleanup fails.
    pub fn stop(&mut self) -> Result<(), CleanupError> {
        let flush = match &mut self.state {
            RunState::Running(context) => context.finish(),
            _ => Ok(()),
        };
        // Release all resources even when flushing failed, before resetting arming.
        let arming = self.enter_idle();
        match (flush, arming) {
            (Ok(()), Ok(())) => Ok(()),
            (Err(source), Ok(())) => Err(CleanupError::Telemetry(source)),
            (Ok(()), Err(source)) => Err(CleanupError::Arming(source)),
            (Err(telemetry), Err(arming)) => {
                Err(CleanupError::TelemetryAndArming { telemetry, arming })
            }
        }
    }

    // TODO: `telemetry_directory` should move out of `Runtime` and be taken from `SimulatorAdapter`
    // or somewhere else to avoid the `target_arch` configuration.

    #[cfg(target_arch = "wasm32")]
    /// The package-specific writable mount is the only simulator output location.
    fn telemetry_directory(_scenario_path: &Path) -> Result<PathBuf, RuntimeError> {
        Ok(PathBuf::from("/work"))
    }

    #[cfg(not(target_arch = "wasm32"))]
    /// Host runs write beside the input scenario.
    fn telemetry_directory(scenario_path: &Path) -> Result<PathBuf, RuntimeError> {
        scenario_path
            .parent()
            .map(ToOwned::to_owned)
            .ok_or_else(|| RuntimeError::ScenarioPathWithoutParent {
                path: scenario_path.to_path_buf(),
            })
    }
}

/// Prepared inputs and optional aircraft readiness checks before entering playback.
#[derive(Debug)]
struct InitContext {
    /// Validated recording definitions, consumed by playback once ready.
    record: Vec<RecordingConfig>,
    /// Primed input cursors that remain stationary while initialising.
    scenario: Scenario,
    /// Output location, without an open telemetry file.
    telemetry_directory: PathBuf,
    /// Pure readiness and deadline state, absent when no aircraft setup is required.
    initialisation: Option<Initialisation>,
}

impl InitContext {
    /// Retains prepared resources and selects optional aircraft readiness checks on arming.
    fn new(
        config: Config,
        scenario: Scenario,
        telemetry_directory: PathBuf,
        simulation_time: Duration,
        aircraft: &mut dyn AircraftInitialiser,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Self {
        let Config {
            initialisation: targets,
            record,
            ..
        } = config;
        let initialisation = targets.and_then(|targets| {
            Self::prepare_initialisation(targets, simulation_time, aircraft, simulator)
        });
        Self {
            record,
            scenario,
            telemetry_directory,
            initialisation,
        }
    }

    /// Creates readiness checks for configured targets when the aircraft is supported.
    fn prepare_initialisation(
        targets: InitialisationConfig,
        simulation_time: Duration,
        aircraft: &mut dyn AircraftInitialiser,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Option<Initialisation> {
        match aircraft.detect(simulator) {
            AircraftSupport::Supported => Some(Initialisation::new(targets, simulation_time)),
            AircraftSupport::Unsupported => {
                println!("TESTPILOT: initialisation skipped: unsupported or unidentified aircraft");
                None
            }
        }
    }

    /// Reports ready without setup, or checks the deadline before submitting and observing targets.
    fn advance(
        &mut self,
        simulation_time: Duration,
        aircraft: &mut dyn AircraftInitialiser,
        simulator: &mut dyn SimulatorAdapter,
    ) -> Result<InitProgress, InitialisationError> {
        let Some(initialisation) = &mut self.initialisation else {
            return Ok(InitProgress::Ready);
        };
        initialisation.timed_out(simulation_time)?;
        aircraft.submit(simulator, initialisation.targets())?;
        let actual = aircraft.readback(simulator)?;
        if initialisation.is_complete(actual)? {
            Ok(InitProgress::Ready)
        } else {
            Ok(InitProgress::Waiting)
        }
    }
}

/// Resources and operations that exist only while replay is running.
#[derive(Debug)]
struct RunContext {
    /// Streaming input cursors.
    scenario: Scenario,
    /// Conversion, injection and retained input values for telemetry.
    injector: InputInjector,
    /// Recording definitions, schedules, sample buffers and CSV writer.
    telemetry: Telemetry,
    /// Simulator time of the first playback update, set when that frame arrives.
    started_at: Option<Duration>,
}

impl TryFrom<InitContext> for RunContext {
    type Error = RecordingError;

    /// Moves prepared resources once after readiness, opening telemetry at the phase boundary.
    fn try_from(init: InitContext) -> Result<Self, Self::Error> {
        let input_count = init.scenario.signal_count();
        let injected_names: Vec<_> = init
            .scenario
            .interpolation_rows()
            .map(|input| input.signal.to_owned())
            .collect();
        let telemetry = Telemetry::new(
            &init.telemetry_directory,
            init.record,
            &injected_names,
            SystemTime::now(),
        )?;
        Ok(Self {
            scenario: init.scenario,
            injector: InputInjector::new(input_count),
            telemetry,
            started_at: None,
        })
    }
}

impl RunContext {
    /// Checks recording interfaces once, after this context is installed in the runtime.
    fn validate(&self, simulator: &mut dyn SimulatorAdapter) -> Result<(), TelemetryError> {
        self.telemetry.validate(simulator)
    }

    /// Coordinates a frame in order: stream, interpolate, inject, sample and record.
    fn advance(
        &mut self,
        simulation_time: Duration,
        simulator: &mut dyn SimulatorAdapter,
    ) -> anyhow::Result<FrameOutcome> {
        let started_at = *self.started_at.get_or_insert(simulation_time);
        let elapsed = simulation_time.checked_sub(started_at).ok_or(
            RuntimeError::SimulationTimeMovedBackwards {
                started_at,
                current: simulation_time,
            },
        )?;
        let Some(inputs) = self.scenario.advance(elapsed)? else {
            return Ok(FrameOutcome::Complete);
        };
        self.injector.apply(inputs.iter(), simulator)?;
        self.telemetry
            .record_frame(elapsed, self.injector.values(), simulator)?;
        Ok(FrameOutcome::Continue)
    }

    /// Flushes telemetry before the runtime drops this context and its resources.
    fn finish(&mut self) -> Result<(), RecordingError> {
        self.telemetry.flush()
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

    use crate::config::InitialisationConfig;
    use crate::error::InitialisationError;
    use crate::error::{InjectionError, RuntimeError, SimulatorError, TelemetryError};
    use crate::initialisation::AircraftInitialisationState;
    use crate::simulator::SimulatorAdapter;

    use super::{ARMED_VARIABLE, RunState, Runtime};

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

    #[derive(Debug, Clone, PartialEq)]
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

    #[derive(Debug, Clone, PartialEq, Eq)]
    enum Failure {
        SimulationTime,
        Write(String),
        ValidateRead(String),
        Read(String),
        LocalVariableExists(String),
    }

    #[derive(Debug, Clone, PartialEq)]
    struct FakeSimulator {
        time: Duration,
        reads: HashMap<String, VecDeque<f64>>,
        operations: Vec<Operation>,
        failure: Option<Failure>,
        fail_arming_reset: bool,
        string_reads: VecDeque<Result<String, SimulatorError>>,
    }

    impl FakeSimulator {
        fn new(time: Duration) -> Self {
            Self {
                time,
                reads: HashMap::new(),
                operations: Vec::new(),
                failure: None,
                fail_arming_reset: false,
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
            if self.fail_arming_reset
                && matches!(operation, Failure::Write(variable) if variable == ARMED_VARIABLE)
            {
                return true;
            }
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

    #[derive(Debug, Clone, Default, PartialEq)]
    struct FakeAircraftInitialiser {
        mass_balance: VecDeque<Result<AircraftInitialisationState, InitialisationError>>,
        fail_initialisation: bool,
        unsupported: bool,
    }

    type SimulatorHandle = Rc<RefCell<FakeSimulator>>;
    type InitialiserHandle = Rc<RefCell<FakeAircraftInitialiser>>;

    #[derive(Debug, Clone, PartialEq)]
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
                        value: targets.zfw.unwrap(),
                    },
                ));
            }
            Ok(())
        }

        fn readback(
            &mut self,
            _simulator: &mut dyn SimulatorAdapter,
        ) -> Result<AircraftInitialisationState, InitialisationError> {
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

    #[derive(Debug)]
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

    fn runtime(fixture: &Fixture, simulator: FakeSimulator) -> (Runtime, SimulatorHandle) {
        let (runtime, simulator, _) =
            runtime_with_initialiser(fixture, simulator, FakeAircraftInitialiser::default());
        (runtime, simulator)
    }

    fn runtime_with_initialiser(
        fixture: &Fixture,
        simulator: FakeSimulator,
        initialiser: FakeAircraftInitialiser,
    ) -> (Runtime, SimulatorHandle, InitialiserHandle) {
        let config_path = fixture.config_path.clone();
        let simulator = Rc::new(RefCell::new(simulator));
        let initialiser = Rc::new(RefCell::new(initialiser));
        let runtime = Runtime::new(
            config_path,
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

    const MATCHED: AircraftInitialisationState = AircraftInitialisationState {
        zfw: Some(60000.0),
        gw: Some(65000.0),
        gwcg: Some(25.0),
        ths: None,
    };
    const UNMATCHED: AircraftInitialisationState = AircraftInitialisationState {
        gwcg: Some(26.0),
        ..MATCHED
    };

    fn initialisation_fixture() -> Fixture {
        let (input_config, _) = CONFIG.split_once("[record.0]").unwrap();
        Fixture::new(&format!(
            "{input_config}\n[initialisation]\nzfw = 60000\ngw = 65000\ngwcg = 25\n"
        ))
    }

    #[test]
    fn absent_or_empty_initialisation_advances_through_each_phase_without_detection() {
        let (input_config, _) = CONFIG.split_once("[record.0]").unwrap();
        for suffix in ["", "\n[initialisation]\n"] {
            let fixture = Fixture::new(&format!("{input_config}{suffix}"));
            let mut simulator = FakeSimulator::new(duration(100.0));
            simulator.queue_reads(ARMED_VARIABLE, [1.0]);
            let (mut runtime, simulator) = runtime(&fixture, simulator);

            runtime.pre_update().unwrap();
            assert!(matches!(runtime.state, RunState::Initialising(_)));
            assert_no_replay_writes(&simulator.borrow());
            assert_no_telemetry(&fixture);

            // No aircraft setup means no initialisation deadline or arming read.
            simulator.borrow_mut().time = duration(135.0);
            simulator.borrow_mut().failure = Some(Failure::Read(ARMED_VARIABLE.to_owned()));
            simulator.borrow_mut().clear_operations();
            runtime.pre_update().unwrap();
            assert!(matches!(runtime.state, RunState::Running(_)));
            assert!(simulator.borrow().operations.is_empty());
            assert_eq!(fixture.telemetry_contents().lines().count(), 1);

            // Even a late first playback update must begin at scenario time zero.
            simulator.borrow_mut().time = duration(140.0);
            runtime.pre_update().unwrap();
            assert_eq!(
                simulator.borrow().operations,
                vec![Operation::Write {
                    variable: "K:AXIS_ELEVATOR_SET".to_owned(),
                    value: 0.0,
                }]
            );
            runtime.stop().unwrap();
            assert!(fixture.telemetry_contents().ends_with("\n0,0\n"));
        }
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
        runtime.pre_update().unwrap();
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
    fn initialisation_waits_without_output_and_starts_at_zero_after_simultaneous_readiness() {
        let fixture = initialisation_fixture();
        let mut initialiser = FakeAircraftInitialiser::default();
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0; 5]);
        initialiser.mass_balance.extend([
            Ok(AircraftInitialisationState {
                zfw: Some(61000.0),
                ..MATCHED
            }),
            Ok(AircraftInitialisationState {
                gw: Some(66000.0),
                ..MATCHED
            }),
            Ok(UNMATCHED),
            Ok(MATCHED),
        ]);
        let (mut runtime, simulator, _) =
            runtime_with_initialiser(&fixture, simulator, initialiser);
        runtime.pre_update().unwrap();
        for now in [100.1, 110.0, 120.0] {
            simulator.borrow_mut().time = duration(now);
            runtime.pre_update().unwrap();
            assert_no_telemetry(&fixture);
            assert_no_replay_writes(&simulator.borrow());
        }
        simulator.borrow_mut().time = duration(129.999);
        runtime.pre_update().unwrap();
        simulator.borrow_mut().time = duration(130.0);
        runtime.pre_update().unwrap();
        simulator.borrow_mut().time = duration(130.5);
        runtime.pre_update().unwrap();
        assert_eq!(
            simulator
                .borrow_mut()
                .operations
                .iter()
                .filter(|op| matches!(op, Operation::Initialise(_)))
                .count(),
            4
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
                    zfw: Some(60000.0),
                    gw: Some(65000.0),
                    gwcg: Some(25.0),
                    ths: None,
                }))
        );
        runtime.stop().unwrap();
        assert_eq!(
            fixture.telemetry_contents(),
            "sidestick_pitch_position.time,sidestick_pitch_position.value\n0,0\n0.5,0.5\n"
        );
    }

    #[test]
    fn initialisation_and_playback_begin_on_separate_updates() {
        let fixture = initialisation_fixture();
        let mut initialiser = FakeAircraftInitialiser::default();
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        initialiser.mass_balance.push_back(Ok(MATCHED));
        let (mut runtime, simulator, _) =
            runtime_with_initialiser(&fixture, simulator, initialiser);

        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Initialising(_)));
        assert!(matches!(
            simulator.borrow().operations.as_slice(),
            [Operation::Read { .. }, Operation::DetectAircraft]
        ));
        assert_no_telemetry(&fixture);

        simulator.borrow_mut().time = duration(101.0);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Running(_)));
        assert!(matches!(
            simulator.borrow().operations.as_slice(),
            [Operation::Initialise(_), Operation::ReadMassBalance]
        ));
        assert_eq!(fixture.telemetry_contents().lines().count(), 1);

        simulator.borrow_mut().time = duration(105.0);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert_eq!(
            simulator.borrow().operations,
            vec![Operation::Write {
                variable: "K:AXIS_ELEVATOR_SET".to_owned(),
                value: 0.0,
            }]
        );
        runtime.stop().unwrap();
        assert!(fixture.telemetry_contents().ends_with("\n0,0\n"));
    }

    #[test]
    fn initialisation_timeout_precedes_readiness_and_allows_a_new_run() {
        for now in [130.0, 135.0] {
            let fixture = initialisation_fixture();
            let mut initialiser = FakeAircraftInitialiser::default();
            let mut simulator = FakeSimulator::new(duration(100.0));
            simulator.queue_reads(ARMED_VARIABLE, [1.0]);
            initialiser
                .mass_balance
                .extend([Ok(UNMATCHED), Ok(MATCHED)]);
            let (mut runtime, simulator, initialiser) =
                runtime_with_initialiser(&fixture, simulator, initialiser);
            runtime.pre_update().unwrap();
            simulator.borrow_mut().time = duration(100.1);
            runtime.pre_update().unwrap();
            simulator.borrow_mut().time = duration(now);
            let error = runtime.pre_update().unwrap_err();
            assert!(matches!(
                error.downcast_ref::<InitialisationError>(),
                Some(InitialisationError::Timeout {
                    latest: Some(crate::initialisation::AircraftInitialisationState {
                        zfw: Some(60000.0),
                        gw: Some(65000.0),
                        gwcg: Some(26.0),
                        ths: None
                    }),
                    ..
                })
            ));
            assert_eq!(
                initialiser.borrow().mass_balance.len(),
                1,
                "deadline must be checked before readback"
            );
            assert!(matches!(runtime.state, RunState::Idle(_)));
            assert_no_replay_writes(&simulator.borrow());
            assert_no_telemetry(&fixture);
            assert_eq!(
                simulator.borrow().operations.last(),
                Some(&Operation::Write {
                    variable: ARMED_VARIABLE.to_owned(),
                    value: 0.0
                })
            );
            simulator
                .borrow_mut()
                .queue_reads(ARMED_VARIABLE, [0.0, 1.0]);
            runtime.pre_update().unwrap();
            assert!(matches!(runtime.state, RunState::Idle(_)));
            assert_no_telemetry(&fixture);
            runtime.pre_update().unwrap();
            assert!(matches!(runtime.state, RunState::Initialising(_)));
            runtime.pre_update().unwrap();
            assert!(matches!(runtime.state, RunState::Running(_)));
            runtime.pre_update().unwrap();
            runtime.stop().unwrap();
            assert!(fixture.telemetry_contents().ends_with("\n0,0\n"));
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
                2 => initialiser
                    .mass_balance
                    .push_back(Ok(AircraftInitialisationState {
                        gw: Some(f64::NAN),
                        ..MATCHED
                    })),
                _ => initialiser.mass_balance.push_back(Ok(UNMATCHED)),
            }
            let (mut runtime, simulator, _) =
                runtime_with_initialiser(&fixture, simulator, initialiser);
            runtime.pre_update().unwrap();
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
            assert!(matches!(runtime.state, RunState::Idle(_)));
            runtime.stop().unwrap();
            assert_no_replay_writes(&simulator.borrow());
            assert_no_telemetry(&fixture);
        }
    }

    #[test]
    fn initialisation_ignores_arming_changes_and_starts_when_ready() {
        let fixture = initialisation_fixture();
        let mut initialiser = FakeAircraftInitialiser::default();
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        initialiser
            .mass_balance
            .extend([Ok(UNMATCHED), Ok(UNMATCHED), Ok(MATCHED)]);
        let (mut runtime, simulator, _) =
            runtime_with_initialiser(&fixture, simulator, initialiser);
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Initialising(_)));

        // Active phases must not depend on arming reads, even if they would fail.
        simulator.borrow_mut().failure = Some(Failure::Read(ARMED_VARIABLE.to_owned()));
        for (now, armed) in [(100.5, 0.0), (101.0, 1.0)] {
            simulator.borrow_mut().time = duration(now);
            simulator
                .borrow_mut()
                .reads
                .insert(ARMED_VARIABLE.to_owned(), VecDeque::from([armed]));
            simulator.borrow_mut().clear_operations();

            runtime.pre_update().unwrap();

            assert!(matches!(runtime.state, RunState::Initialising(_)));
            assert!(matches!(
                simulator.borrow().operations.as_slice(),
                [Operation::Initialise(_), Operation::ReadMassBalance]
            ));
            assert_no_telemetry(&fixture);
        }

        simulator.borrow_mut().time = duration(101.5);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Running(_)));
        assert!(matches!(
            simulator.borrow().operations.as_slice(),
            [Operation::Initialise(_), Operation::ReadMassBalance]
        ));
        simulator.borrow_mut().time = duration(102.0);
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
        let simulator = Rc::new(RefCell::new(simulator));
        let result = Runtime::new(
            fixture.config_path.clone(),
            Box::new(Rc::clone(&simulator)),
            Box::new(SharedAircraftInitialiser {
                state: Rc::new(RefCell::new(FakeAircraftInitialiser::default())),
                simulator: Rc::clone(&simulator),
            }),
        );
        match result {
            Err(SimulatorError::CalculatorCodeWriteFailed { variable, value })
                if variable == ARMED_VARIABLE && value == 0.0 => {}
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
    fn idle_frames_preserve_arming_history_until_a_zero_to_one_transition() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(42.0));
        simulator.queue_reads(ARMED_VARIABLE, [2.0, 1.0, 0.0, 1.0]);
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        simulator.borrow_mut().clear_operations();

        for _ in 0..3 {
            runtime.pre_update().unwrap();
            assert!(matches!(runtime.state, RunState::Idle(_)));
        }
        assert_no_telemetry(&fixture);
        assert_eq!(
            simulator.borrow().operations,
            vec![
                Operation::Read {
                    variable: ARMED_VARIABLE.to_owned(),
                    unit: None,
                };
                3
            ]
        );

        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Initialising(_)));
        assert_no_telemetry(&fixture);
        runtime.stop().unwrap();
    }

    #[test]
    fn running_frames_validate_once_and_inject_before_sampling() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        simulator.queue_reads("A:PLANE PITCH DEGREES", [0.25, 0.5]);
        simulator.queue_reads("L:ELEVATOR_POSITION", [0.75, 1.0]);
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        simulator.borrow_mut().clear_operations();

        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Initialising(_)));
        assert_no_telemetry(&fixture);
        assert_no_replay_writes(&simulator.borrow());
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Running(_)));
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
            ]
        );
        assert_eq!(fixture.telemetry_contents().lines().count(), 1);

        for (now, value) in [(101.0, 0.0), (101.5, 0.5)] {
            simulator.borrow_mut().clear_operations();
            simulator.borrow_mut().time = duration(now);
            runtime.pre_update().unwrap();
            assert_eq!(
                simulator.borrow().operations,
                vec![
                    Operation::Write {
                        variable: "K:AXIS_ELEVATOR_SET".to_owned(),
                        value,
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
        }

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
            simulator.queue_reads(ARMED_VARIABLE, [1.0, 0.0]);
            let (mut runtime, simulator) = runtime(&fixture, simulator);

            runtime.pre_update().unwrap();
            runtime.pre_update().unwrap();
            assert_no_replay_writes(&simulator.borrow());

            for (elapsed, value) in [(0.0, 0.0), (0.5, 0.5), (1.0, 1.0), (1.5, 0.5)] {
                simulator.borrow_mut().clear_operations();
                simulator.borrow_mut().time = duration(100.0 + elapsed);
                runtime.pre_update().unwrap();
                assert_eq!(
                    simulator.borrow().operations,
                    vec![Operation::Write {
                        variable: "K:AXIS_ELEVATOR_SET".to_owned(),
                        value,
                    }]
                );
            }

            simulator.borrow_mut().clear_operations();
            simulator.borrow_mut().time = duration(102.1);
            runtime.pre_update().unwrap();
            assert_eq!(
                simulator.borrow().operations,
                vec![Operation::Write {
                    variable: ARMED_VARIABLE.to_owned(),
                    value: 0.0,
                },]
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
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        simulator.queue_reads("A:PLANE PITCH DEGREES", [0.1, 0.2]);
        simulator.queue_reads("L:ELEVATOR_POSITION", [0.3, 0.4]);
        let (mut runtime, simulator) = runtime(&fixture, simulator);

        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        simulator.borrow_mut().clear_operations();
        simulator.borrow_mut().time = duration(10.5);
        runtime.pre_update().unwrap();
        assert_eq!(
            simulator.borrow().operations,
            vec![Operation::Write {
                variable: "K:AXIS_ELEVATOR_SET".to_owned(),
                value: 0.5,
            },]
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
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();

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
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        simulator.queue_reads("A:PLANE PITCH DEGREES", [0.1]);
        simulator.queue_reads("L:ELEVATOR_POSITION", [0.2]);
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        simulator.borrow_mut().clear_operations();

        simulator.borrow_mut().time = duration(22.1);
        runtime.pre_update().unwrap();

        assert_eq!(
            simulator.borrow().operations,
            vec![Operation::Write {
                variable: ARMED_VARIABLE.to_owned(),
                value: 0.0,
            },]
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
    fn running_ignores_arming_changes_until_completion_and_requires_a_fresh_start() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(30.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        simulator.queue_reads("A:PLANE PITCH DEGREES", [0.1, 0.2, 0.3]);
        simulator.queue_reads("L:ELEVATOR_POSITION", [0.3, 0.4, 0.5]);
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();

        simulator.borrow_mut().failure = Some(Failure::Read(ARMED_VARIABLE.to_owned()));
        for (elapsed, armed) in [(0.25, 0.0), (0.5, 1.0)] {
            simulator.borrow_mut().time = duration(30.0 + elapsed);
            simulator
                .borrow_mut()
                .reads
                .insert(ARMED_VARIABLE.to_owned(), VecDeque::from([armed]));
            simulator.borrow_mut().clear_operations();

            runtime.pre_update().unwrap();

            assert!(matches!(runtime.state, RunState::Running(_)));
            assert_eq!(
                simulator.borrow().operations,
                vec![
                    Operation::Write {
                        variable: "K:AXIS_ELEVATOR_SET".to_owned(),
                        value: elapsed,
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
        }

        simulator.borrow_mut().time = duration(32.1);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Idle(_)));
        assert_eq!(
            simulator.borrow().operations,
            vec![Operation::Write {
                variable: ARMED_VARIABLE.to_owned(),
                value: 0.0,
            }]
        );
        let completed_telemetry = fixture.telemetry_contents();
        assert!(completed_telemetry.ends_with(
            "\n0,0.1,0,0.3,0,0\n0.25,0.2,0.25,0.4,0.25,0.25\n0.5,0.3,0.5,0.5,0.5,0.5\n"
        ));

        // Reflect the successful reset in the fake's read queue.
        simulator.borrow_mut().failure = None;
        simulator
            .borrow_mut()
            .reads
            .insert(ARMED_VARIABLE.to_owned(), VecDeque::from([0.0]));
        simulator.borrow_mut().time = duration(33.0);
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Idle(_)));
        assert_eq!(fixture.telemetry_contents(), completed_telemetry);

        // A new directory avoids a same-second telemetry filename collision.
        let restart = Fixture::new(CONFIG);
        runtime.config_path = restart.config_path.clone();
        simulator.borrow_mut().queue_reads(ARMED_VARIABLE, [1.0]);
        simulator
            .borrow_mut()
            .queue_reads("A:PLANE PITCH DEGREES", [0.6]);
        simulator
            .borrow_mut()
            .queue_reads("L:ELEVATOR_POSITION", [0.7]);
        simulator.borrow_mut().time = duration(34.0);
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Initialising(_)));
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Running(_)));
        runtime.pre_update().unwrap();
        runtime.stop().unwrap();
        assert!(
            restart
                .telemetry_contents()
                .ends_with("\n0,0.6,0,0.7,0,0\n")
        );
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
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
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
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
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
    fn telemetry_creation_failure_during_conversion_resets_arming_and_allows_rearming() {
        let (input_config, _) = CONFIG.split_once("[record.0]").unwrap();
        let fixture = Fixture::new(input_config);
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        runtime.pre_update().unwrap();
        let RunState::Initialising(context) = &mut runtime.state else {
            panic!("not initialising")
        };
        // A regular file cannot serve as the output directory.
        context.telemetry_directory = fixture.directory.join("scenario.csv");
        simulator.borrow_mut().clear_operations();

        let error = runtime.pre_update().unwrap_err();

        assert!(matches!(
            error.downcast_ref::<crate::error::RecordingError>(),
            Some(crate::error::RecordingError::CreateFile { .. })
        ));
        assert!(matches!(runtime.state, RunState::Idle(_)));
        assert_no_telemetry(&fixture);
        assert_eq!(
            simulator.borrow().operations,
            vec![Operation::Write {
                variable: ARMED_VARIABLE.to_owned(),
                value: 0.0,
            }]
        );

        simulator
            .borrow_mut()
            .queue_reads(ARMED_VARIABLE, [0.0, 1.0]);
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Idle(_)));
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Initialising(_)));
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Running(_)));
        runtime.pre_update().unwrap();
        runtime.stop().unwrap();
        assert!(fixture.telemetry_contents().ends_with("\n0,0\n"));
    }

    #[test]
    fn recording_validation_failures_include_the_signal_and_prevent_injection() {
        let fixture = Fixture::new(CONFIG);
        let mut simulator = FakeSimulator::new(duration(40.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        simulator.failure = Some(Failure::ValidateRead("A:PLANE PITCH DEGREES".to_owned()));
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        simulator.borrow_mut().clear_operations();

        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Initialising(_)));
        assert_no_telemetry(&fixture);
        let error = runtime
            .pre_update()
            .expect_err("recording validation should fail");

        match error.downcast_ref::<TelemetryError>() {
            Some(TelemetryError::ValidateRecordingSignal { signal, .. }) if signal == "pitch" => {}
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
                Operation::Write {
                    variable: ARMED_VARIABLE.to_owned(),
                    value: 0.0
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

        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        let error = runtime
            .pre_update()
            .expect_err("input injection should fail");

        match error.downcast_ref::<InjectionError>() {
            Some(InjectionError::InjectSignal { signal, .. })
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

        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        let error = runtime
            .pre_update()
            .expect_err("telemetry sampling should fail");

        match error.downcast_ref::<TelemetryError>() {
            Some(TelemetryError::SampleSignal { signal, .. }) if signal == "pitch" => {}
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
    fn invalid_config_or_initial_input_can_be_corrected_and_rearmed() {
        for invalid_config in [true, false] {
            let fixture = initialisation_fixture();
            let valid_config = fs::read_to_string(&fixture.config_path).unwrap();
            if invalid_config {
                fs::write(&fixture.config_path, "format_version =").unwrap();
            } else {
                fs::write(fixture.directory.join("scenario.csv"), "sidestick_pitch_position.time,sidestick_pitch_position.value\n0,invalid\n1,1\n").unwrap();
            }
            let mut simulator = FakeSimulator::new(duration(100.0));
            simulator.queue_reads(ARMED_VARIABLE, [1.0]);
            let (mut runtime, simulator, initialiser) =
                runtime_with_initialiser(&fixture, simulator, FakeAircraftInitialiser::default());
            simulator.borrow_mut().clear_operations();
            let error = runtime.pre_update().unwrap_err();
            assert!(format!("{error:#}").contains(if invalid_config {
                "replayer_config.toml"
            } else {
                "scenario.csv"
            }));
            assert!(matches!(runtime.state, RunState::Idle(_)));
            assert!(!simulator.borrow().operations.iter().any(|op| matches!(
                op,
                Operation::DetectAircraft | Operation::Initialise(_) | Operation::ReadMassBalance
            )));
            assert_eq!(
                simulator.borrow().operations.last(),
                Some(&Operation::Write {
                    variable: ARMED_VARIABLE.to_owned(),
                    value: 0.0,
                })
            );
            assert_no_telemetry(&fixture);
            fs::write(&fixture.config_path, valid_config).unwrap();
            fs::write(fixture.directory.join("scenario.csv"), SCENARIO).unwrap();
            initialiser.borrow_mut().mass_balance.push_back(Ok(MATCHED));
            simulator
                .borrow_mut()
                .queue_reads(ARMED_VARIABLE, [0.0, 1.0]);
            runtime.pre_update().unwrap();
            assert!(matches!(runtime.state, RunState::Idle(_)));
            runtime.pre_update().unwrap();
            assert!(matches!(runtime.state, RunState::Initialising(_)));
            runtime.pre_update().unwrap();
            assert!(matches!(runtime.state, RunState::Running(_)));
            runtime.pre_update().unwrap();
            runtime.stop().unwrap();
            assert!(fixture.telemetry_contents().ends_with("\n0,0\n"));
        }
    }

    #[test]
    fn stopping_initialisation_allows_a_fresh_run() {
        let fixture = initialisation_fixture();
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0, 1.0]);
        let mut initialiser = FakeAircraftInitialiser::default();
        initialiser
            .mass_balance
            .extend([Ok(UNMATCHED), Ok(MATCHED)]);
        let (mut runtime, simulator, _) =
            runtime_with_initialiser(&fixture, simulator, initialiser);
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Initialising(_)));
        runtime.stop().unwrap();
        assert!(matches!(runtime.state, RunState::Idle(_)));
        assert_no_telemetry(&fixture);
        simulator.borrow_mut().time = duration(150.0);
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Initialising(_)));
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Running(_)));
        runtime.pre_update().unwrap();
        runtime.stop().unwrap();
        assert!(fixture.telemetry_contents().ends_with("\n0,0\n"));
    }

    #[test]
    fn backwards_playback_clock_preserves_output_and_allows_rearming() {
        let fixture = Fixture::new(INJECTED_VALUES_ONLY_CONFIG);
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        simulator.queue_reads("A:PLANE PITCH DEGREES", [0.25]);
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        simulator.borrow_mut().time = duration(99.0);
        let error = runtime.pre_update().unwrap_err();
        assert!(matches!(
            error.downcast_ref::<RuntimeError>(),
            Some(RuntimeError::SimulationTimeMovedBackwards { .. })
        ));
        assert!(matches!(runtime.state, RunState::Idle(_)));
        assert!(fixture.telemetry_contents().ends_with("\n0,0.25,0,0\n"));
        let partial_telemetry = fixture.telemetry_contents();
        // A separate output directory avoids same-second filename collisions
        // while preserving the failed run's telemetry throughout the retry.
        let retry = Fixture::new(INJECTED_VALUES_ONLY_CONFIG);
        runtime.config_path = retry.config_path.clone();
        simulator.borrow_mut().time = duration(150.0);
        simulator
            .borrow_mut()
            .queue_reads(ARMED_VARIABLE, [0.0, 1.0]);
        simulator
            .borrow_mut()
            .queue_reads("A:PLANE PITCH DEGREES", [0.75, 1.0]);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Idle(_)));
        assert_no_replay_writes(&simulator.borrow());
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Initialising(_)));
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Running(_)));
        runtime.pre_update().unwrap();
        simulator.borrow_mut().time = duration(150.5);
        runtime.pre_update().unwrap();
        runtime.stop().unwrap();
        assert!(
            retry
                .telemetry_contents()
                .ends_with("\n0,0.75,0,0\n0.5,1,0.5,0.5\n")
        );
        assert_eq!(fixture.telemetry_contents(), partial_telemetry);
    }

    #[test]
    fn completion_flush_failure_resets_arming_and_allows_a_fresh_run() {
        let (input_config, _) = CONFIG.split_once("[record.0]").unwrap();
        let fixture = Fixture::new(input_config);
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        let RunState::Running(context) = &mut runtime.state else {
            panic!("not running")
        };
        context.telemetry.make_output_read_only();
        simulator.borrow_mut().time = duration(100.5);
        runtime.pre_update().unwrap();
        simulator.borrow_mut().clear_operations();
        simulator.borrow_mut().time = duration(103.0);
        let error = runtime.pre_update().unwrap_err();
        assert!(matches!(
            error.downcast_ref::<crate::error::CleanupError>(),
            Some(crate::error::CleanupError::Telemetry(_))
        ));
        assert!(matches!(runtime.state, RunState::Idle(_)));
        assert_eq!(
            simulator.borrow().operations.last(),
            Some(&Operation::Write {
                variable: ARMED_VARIABLE.to_owned(),
                value: 0.0,
            })
        );
        assert!(fixture.telemetry_contents().ends_with("\n0,0\n"));
        let partial_telemetry = fixture.telemetry_contents();
        let retry = Fixture::new(input_config);
        runtime.config_path = retry.config_path.clone();
        simulator
            .borrow_mut()
            .queue_reads(ARMED_VARIABLE, [0.0, 1.0]);
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Idle(_)));
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Initialising(_)));
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Running(_)));
        runtime.pre_update().unwrap();
        runtime.stop().unwrap();
        assert!(matches!(runtime.state, RunState::Idle(_)));
        assert!(retry.telemetry_contents().ends_with("\n0,0\n"));
        assert_eq!(fixture.telemetry_contents(), partial_telemetry);
    }

    #[test]
    fn primary_and_cleanup_failures_are_retained_and_reset_is_retried_before_rearming() {
        let (input_config, _) = CONFIG.split_once("[record.0]").unwrap();
        let fixture = Fixture::new(input_config);
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0]);
        let (mut runtime, simulator) = runtime(&fixture, simulator);
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        let RunState::Running(context) = &mut runtime.state else {
            panic!("not running")
        };
        context.telemetry.make_output_read_only();
        simulator.borrow_mut().time = duration(100.5);
        runtime.pre_update().unwrap();
        simulator.borrow_mut().failure = Some(Failure::SimulationTime);
        simulator.borrow_mut().fail_arming_reset = true;
        simulator.borrow_mut().clear_operations();
        let error = runtime.pre_update().unwrap_err();
        let failure = error
            .downcast_ref::<crate::error::TerminationError>()
            .unwrap();
        let crate::error::TerminationError::CleanupAfterFailure { primary, cleanup } = failure;
        assert!(matches!(
            primary.downcast_ref::<SimulatorError>(),
            Some(SimulatorError::SimulationTimeUnavailable)
        ));
        assert!(matches!(
            cleanup,
            crate::error::CleanupError::TelemetryAndArming { .. }
        ));
        assert!(error.to_string().contains("telemetry cleanup failed"));
        assert!(error.to_string().contains("arming reset also failed"));
        assert_eq!(
            simulator.borrow().operations,
            vec![Operation::Write {
                variable: ARMED_VARIABLE.to_owned(),
                value: 0.0,
            }]
        );
        assert!(matches!(runtime.state, RunState::Idle(_)));
        assert!(fixture.telemetry_contents().ends_with("\n0,0\n"));
        let partial_telemetry = fixture.telemetry_contents();
        simulator.borrow_mut().failure = None;
        // A stale high value must not start another run while reset is failing.
        simulator.borrow_mut().queue_reads(ARMED_VARIABLE, [1.0]);
        simulator.borrow_mut().clear_operations();
        assert!(runtime.pre_update().is_err());
        assert!(matches!(runtime.state, RunState::Idle(_)));
        assert!(simulator.borrow().operations.iter().all(|operation| {
            *operation
                == Operation::Write {
                    variable: ARMED_VARIABLE.to_owned(),
                    value: 0.0,
                }
        }));
        simulator.borrow_mut().fail_arming_reset = false;
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert_eq!(
            simulator.borrow().operations,
            vec![Operation::Write {
                variable: ARMED_VARIABLE.to_owned(),
                value: 0.0,
            }]
        );
        assert!(matches!(runtime.state, RunState::Idle(_)));
        let retry = Fixture::new(input_config);
        runtime.config_path = retry.config_path.clone();
        simulator
            .borrow_mut()
            .reads
            .insert(ARMED_VARIABLE.to_owned(), VecDeque::from([0.0, 1.0]));
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Idle(_)));
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Initialising(_)));
        runtime.pre_update().unwrap();
        assert!(matches!(runtime.state, RunState::Running(_)));
        runtime.pre_update().unwrap();
        runtime.stop().unwrap();
        assert!(retry.telemetry_contents().ends_with("\n0,0\n"));
        assert_eq!(fixture.telemetry_contents(), partial_telemetry);
    }
}
