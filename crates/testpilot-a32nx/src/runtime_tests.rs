//! A32NX commands and lifecycle behavior exercised through the public core interfaces.

use std::cell::RefCell;
use std::collections::{HashMap, VecDeque};
use std::fs;
use std::path::{Path, PathBuf};
use std::rc::Rc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use testpilot_core::aircraft_initialisation::AircraftInitialiser;
use testpilot_core::config::{InitialisationConfig, LandingGear};
use testpilot_core::error::{InitialisationError, SimulatorError};
use testpilot_core::runtime::Runtime;
use testpilot_core::simulator::SimulatorAdapter;

use super::A32nxInitialiser;

use crate::flaps::MOVEMENT_VARIABLES;
use crate::landing_gear::GEAR_VARIABLES;

const ARMED_VARIABLE: &str = "L:REPLAYER_ARMED";

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

const SCENARIO: &str =
    "sidestick_pitch_position.time,sidestick_pitch_position.value\n0,0\n1,100\n2,0\n";

static NEXT_FIXTURE_ID: AtomicU64 = AtomicU64::new(0);

#[derive(Debug, Clone, PartialEq)]
enum Operation {
    ReadString(String),
    LocalVariableExists(String),
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

#[derive(Debug, Clone, PartialEq)]
struct SharedSimulator(Rc<RefCell<FakeSimulator>>);

type SimulatorHandle = Rc<RefCell<FakeSimulator>>;

impl SimulatorAdapter for SharedSimulator {
    fn local_variable_exists(&mut self, variable: &str) -> Result<bool, SimulatorError> {
        self.0.borrow_mut().local_variable_exists(variable)
    }

    fn read_string(&mut self, variable: &str) -> Result<String, SimulatorError> {
        self.0.borrow_mut().read_string(variable)
    }

    fn simulation_time(&self) -> Result<Duration, SimulatorError> {
        self.0.borrow().simulation_time()
    }

    fn write(&mut self, variable: &str, value: f64) -> Result<(), SimulatorError> {
        self.0.borrow_mut().write(variable, value)
    }

    fn validate_read(&mut self, variable: &str, unit: Option<&str>) -> Result<(), SimulatorError> {
        self.0.borrow_mut().validate_read(variable, unit)
    }

    fn read(&mut self, variable: &str, unit: Option<&str>) -> Result<f64, SimulatorError> {
        self.0.borrow_mut().read(variable, unit)
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

    fn validate_read(&mut self, variable: &str, unit: Option<&str>) -> Result<(), SimulatorError> {
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
        let directory =
            std::env::temp_dir().join(format!("replay-gauge-runtime-{}-{id}", std::process::id()));
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

fn duration(seconds: f64) -> Duration {
    Duration::try_from_secs_f64(seconds)
        .unwrap_or_else(|error| panic!("invalid test duration: {error}"))
}

const TARGETS: InitialisationConfig = InitialisationConfig {
    zfw: Some(60000.0),
    gw: Some(65000.0),
    gwcg: Some(25.0),
    ths: None,
    flaps: None,
    landing_gear: None,
};

fn initialisation_fixture() -> Fixture {
    let (input_config, _) = CONFIG.split_once("[record.0]").unwrap();
    Fixture::new(&format!(
        "{input_config}\n[initialisation]\nzfw = 60000\ngw = 65000\ngwcg = 25\n"
    ))
}

fn trim_runtime() -> (Fixture, Runtime, SimulatorHandle) {
    let fixture = initialisation_fixture();
    let config = fs::read_to_string(&fixture.config_path).unwrap();
    fs::write(&fixture.config_path, format!("{config}ths = 4.75\n")).unwrap();
    let mut simulator = FakeSimulator::new(duration(100.0));
    simulator.string_reads.push_back(Ok("A20N".to_owned()));
    simulator.queue_reads("L:A32NX_IS_READY", [0.0]);
    simulator.queue_reads(ARMED_VARIABLE, [1.0; 4]);
    simulator.queue_reads("L:A32NX_AIRFRAME_ZFW", [60000.0; 4]);
    simulator.queue_reads("L:A32NX_AIRFRAME_GW", [65000.0; 4]);
    simulator.queue_reads("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", [25.0; 4]);
    simulator.queue_reads("L:A32NX_HYD_TRIM_WHEEL_PERCENT", [0.0, 25.0, 50.0]);
    let simulator = Rc::new(RefCell::new(simulator));
    let runtime = Runtime::new(
        fixture.config_path.clone(),
        Box::new(SharedSimulator(Rc::clone(&simulator))),
        Box::new(A32nxInitialiser::default()),
    )
    .unwrap();
    (fixture, runtime, simulator)
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
fn ths_only_initialisation_does_not_load_or_read_mass_balance() {
    let (fixture, mut runtime, simulator) = trim_runtime();
    let config = fs::read_to_string(&fixture.config_path).unwrap();
    fs::write(
        &fixture.config_path,
        config.replace("zfw = 60000\ngw = 65000\ngwcg = 25\n", ""),
    )
    .unwrap();
    for variable in [
        "L:A32NX_AIRFRAME_ZFW",
        "L:A32NX_AIRFRAME_GW",
        "L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC",
    ] {
        simulator.borrow_mut().reads.remove(variable);
    }
    runtime.pre_update().unwrap();
    for now in [100.5, 101.0] {
        simulator.borrow_mut().time = duration(now);
        runtime.pre_update().unwrap();
        assert_no_telemetry(&fixture);
    }
    assert!(simulator.borrow().operations.iter().all(|op| match op {
        Operation::Write { variable, .. } =>
            variable == ARMED_VARIABLE || variable == "K:AXIS_ELEV_TRIM_SET",
        Operation::Read { variable, .. } => !variable.starts_with("L:A32NX_AIRFRAME_"),
        _ => true,
    }));
    simulator.borrow_mut().time = duration(102.0);
    runtime.pre_update().unwrap();
    simulator.borrow_mut().time = duration(102.5);
    runtime.pre_update().unwrap();
    runtime.stop().unwrap();
    assert!(fixture.telemetry_contents().ends_with("\n0,0\n"));
}

#[test]
fn trim_delays_replay_resends_demand_and_releases_on_readiness() {
    let (fixture, mut runtime, simulator) = trim_runtime();
    runtime.pre_update().unwrap();
    for now in [100.5, 101.0] {
        simulator.borrow_mut().time = duration(now);
        runtime.pre_update().unwrap();
        assert_no_telemetry(&fixture);
        assert!(!simulator.borrow().operations.iter().any(|op| matches!(op, Operation::Write { variable, .. } if variable == "K:AXIS_ELEVATOR_SET")));
    }
    assert_eq!(simulator.borrow().operations.iter().filter(|op|
        matches!(op, Operation::Write { variable, value: 1.0 } if variable == "K:AXIS_ELEV_TRIM_SET")
    ).count(), 2);
    // Every waiting frame resubmits the same loading and trim targets.
    assert_eq!(simulator.borrow().operations.iter().filter(|op|
        matches!(op, Operation::Write { variable, .. } if variable == "L:A32NX_BOARDING_STARTED_BY_USR")
    ).count(), 2);
    let writes: Vec<_> = simulator
        .borrow()
        .operations
        .iter()
        .filter_map(|op| match op {
            Operation::Write { variable, value } if variable != ARMED_VARIABLE => {
                Some((variable.clone(), *value))
            }
            _ => None,
        })
        .collect();
    let (first, repeated) = writes.split_at(writes.len() / 2);
    assert_eq!(
        first, repeated,
        "resubmission must preserve every loading and trim value"
    );
    simulator.borrow_mut().clear_operations();
    simulator.borrow_mut().time = duration(102.0);
    runtime.pre_update().unwrap();
    simulator.borrow_mut().clear_operations();
    simulator.borrow_mut().time = duration(102.5);
    runtime.pre_update().unwrap();
    runtime.stop().unwrap();
    assert!(fixture.telemetry_contents().contains("\n0,0\n"));
    assert!(!simulator.borrow().operations.iter().any(|op|
        matches!(op, Operation::Write { variable, .. } if variable == "K:AXIS_ELEV_TRIM_SET" || variable.starts_with("L:A32NX_"))));
}

#[test]
fn trim_failures_and_timeout_stop_without_telemetry() {
    for failure in [
        Some(Failure::Read("L:A32NX_HYD_TRIM_WHEEL_PERCENT".to_owned())),
        Some(Failure::Write("K:AXIS_ELEV_TRIM_SET".to_owned())),
        Some(Failure::Write("L:A32NX_PAX_A_DESIRED".to_owned())),
        None,
    ] {
        let (fixture, mut runtime, simulator) = trim_runtime();
        runtime.pre_update().unwrap();
        let timeout = failure.is_none();
        simulator.borrow_mut().failure = failure;
        simulator.borrow_mut().time = duration(if timeout { 130.0 } else { 101.0 });
        simulator.borrow_mut().clear_operations();
        let error = runtime.pre_update().unwrap_err();
        assert!(error.downcast_ref::<InitialisationError>().is_some());
        if timeout {
            assert!(matches!(
                error.downcast_ref::<InitialisationError>(),
                Some(InitialisationError::Timeout { .. })
            ));
            assert!(
                !simulator
                    .borrow()
                    .operations
                    .iter()
                    .any(|op| matches!(op, Operation::Write { variable, .. } if variable != ARMED_VARIABLE)),
                "timeout must precede all loading and trim submissions"
            );
        }
        runtime.stop().unwrap();
        assert_no_telemetry(&fixture);
        assert!(!simulator.borrow().operations.iter().any(|op| matches!(op, Operation::Write { variable, .. } if variable == "K:AXIS_ELEVATOR_SET")));
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
fn missing_trim_interface_fails_before_loading_writes() {
    let (fixture, mut runtime, simulator) = trim_runtime();
    runtime.pre_update().unwrap();
    simulator
        .borrow_mut()
        .reads
        .remove("L:A32NX_HYD_TRIM_WHEEL_PERCENT");
    simulator.borrow_mut().clear_operations();
    let error = runtime.pre_update().unwrap_err();
    assert!(matches!(
        error.downcast_ref::<InitialisationError>(),
        Some(InitialisationError::MissingThsInterface)
    ));
    assert!(
        !simulator.borrow().operations.iter().any(
            |op| matches!(op, Operation::Write { variable, .. } if variable != ARMED_VARIABLE)
        )
    );
    runtime.stop().unwrap();
    assert_no_telemetry(&fixture);
}

#[test]
fn a32nx_readiness_reads_ths_only_when_latest_submission_requests_it() {
    use testpilot_core::aircraft_initialisation::AircraftInitialiser;

    let mut simulator = FakeSimulator::new(Duration::ZERO);
    let mut initialiser = A32nxInitialiser::default();
    let targets = InitialisationConfig {
        zfw: Some(60000.0),
        gw: Some(65000.0),
        gwcg: Some(25.0),
        ths: Some(1.0),
        flaps: None,
        landing_gear: None,
    };
    simulator.queue_reads("L:A32NX_AIRFRAME_ZFW", [60000.0; 4]);
    simulator.queue_reads("L:A32NX_AIRFRAME_GW", [65000.0; 4]);
    simulator.queue_reads("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", [25.0; 4]);
    simulator.queue_reads("L:A32NX_HYD_TRIM_WHEEL_PERCENT", [0.0, 50.0, 100.0]);
    initialiser.submit(&mut simulator, targets).unwrap();
    for expected in [-4.0, 4.75, 13.5] {
        simulator.clear_operations();
        assert!(
            initialiser
                .is_ready(
                    &mut simulator,
                    InitialisationConfig {
                        ths: Some(expected),
                        ..targets
                    }
                )
                .unwrap()
        );
        assert_eq!(simulator.operations.len(), 4);
        assert!(
            simulator
                .operations
                .iter()
                .all(|op| matches!(op, Operation::Read { .. }))
        );
        assert_eq!(
            simulator.operations.last(),
            Some(&Operation::Read {
                variable: "L:A32NX_HYD_TRIM_WHEEL_PERCENT".to_owned(),
                unit: None,
            })
        );
    }
    initialiser
        .submit(
            &mut simulator,
            InitialisationConfig {
                ths: None,
                ..targets
            },
        )
        .unwrap();
    simulator.clear_operations();
    assert!(initialiser.is_ready(&mut simulator, TARGETS).unwrap());
    assert_eq!(simulator.operations.len(), 3);
    assert!(
        simulator
            .operations
            .iter()
            .all(|op| matches!(op, Operation::Read { variable, .. }
        if variable != "L:A32NX_HYD_TRIM_WHEEL_PERCENT"))
    );
}

#[test]
fn a32nx_readiness_propagates_configured_ths_read_failures() {
    use testpilot_core::aircraft_initialisation::AircraftInitialiser;

    for invalid in [
        None,
        Some(f64::NAN),
        Some(f64::INFINITY),
        Some(f64::NEG_INFINITY),
    ] {
        let mut simulator = FakeSimulator::new(Duration::ZERO);
        let mut initialiser = A32nxInitialiser::default();
        // Register the interface without queuing a value, so None tests a failed read.
        simulator.queue_reads("L:A32NX_HYD_TRIM_WHEEL_PERCENT", []);
        initialiser
            .submit(
                &mut simulator,
                InitialisationConfig {
                    zfw: Some(60000.0),
                    gw: Some(65000.0),
                    gwcg: Some(25.0),
                    ths: Some(1.0),
                    flaps: None,
                    landing_gear: None,
                },
            )
            .unwrap();
        simulator.queue_reads("L:A32NX_AIRFRAME_ZFW", [60000.0]);
        simulator.queue_reads("L:A32NX_AIRFRAME_GW", [65000.0]);
        simulator.queue_reads("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", [25.0]);
        if let Some(value) = invalid {
            simulator.queue_reads("L:A32NX_HYD_TRIM_WHEEL_PERCENT", [value]);
        }
        simulator.clear_operations();
        assert!(
            matches!(initialiser.is_ready(&mut simulator, InitialisationConfig {
            ths: Some(1.0), ..TARGETS
        }),
            Err(InitialisationError::Readback(SimulatorError::CalculatorCodeReadFailed { variable }))
            | Err(InitialisationError::Readback(SimulatorError::NonFiniteRead { variable, .. }))
            if variable == "L:A32NX_HYD_TRIM_WHEEL_PERCENT")
        );
        assert_eq!(simulator.operations.len(), 4);
        assert!(
            simulator
                .operations
                .iter()
                .all(|op| matches!(op, Operation::Read { .. }))
        );
    }
}

#[test]
fn unsupported_aircraft_skip_trim() {
    let (fixture, mut runtime, simulator) = trim_runtime();
    simulator.borrow_mut().string_reads = VecDeque::from([Ok("C172".to_owned())]);
    simulator.borrow_mut().clear_operations();
    runtime.pre_update().unwrap();
    runtime.pre_update().unwrap();
    runtime.pre_update().unwrap();
    assert!(!simulator.borrow().operations.iter().any(|op| match op {
        Operation::Read { variable, .. }
        | Operation::Write { variable, .. }
        | Operation::ValidateRead { variable, .. }
        | Operation::LocalVariableExists(variable) => variable.contains("TRIM"),
        _ => false,
    }));
    runtime.stop().unwrap();
    assert!(fixture.telemetry_contents().ends_with("\n0,0\n"));
}

#[test]
fn detects_aircraft_model_and_variable_presence_without_reading_readiness() {
    use testpilot_core::aircraft_initialisation::AircraftInitialiser;
    for (model, expected) in [
        ("A20N", true),
        ("TT:ATCCOM.AC_MODEL_A20N.0.text", true),
        ("  a20n  ", true),
        (" tt:atccom.ac_model_a20n.0.TEXT ", true),
        ("A320", false),
        ("A388", false),
        ("C172", false),
        ("A20NX", false),
        ("TT:ATCCOM.AC_MODEL_A388.0.text", false),
        ("Airbus A320 Neo FlyByWire", false),
        ("", false),
        (" ", false),
    ] {
        let mut simulator = FakeSimulator::new(Duration::ZERO);
        simulator.string_reads.push_back(Ok(model.to_owned()));
        simulator.queue_reads("L:A32NX_IS_READY", [0.0]);
        assert_eq!(
            A32nxInitialiser::default().supported(&mut simulator),
            expected,
            "model {model:?}"
        );
        let mut operations = vec![Operation::ReadString("A:ATC MODEL".to_owned())];
        if expected {
            operations.push(Operation::LocalVariableExists(
                "L:A32NX_IS_READY".to_owned(),
            ));
        }
        assert_eq!(simulator.operations, operations);
        assert_eq!(simulator.reads["L:A32NX_IS_READY"], [0.0]);
    }
    let mut simulator = FakeSimulator::new(Duration::ZERO);
    assert!(!A32nxInitialiser::default().supported(&mut simulator));
}

#[test]
fn a20n_without_ready_variable_or_with_failed_lookup_is_unsupported() {
    use testpilot_core::aircraft_initialisation::AircraftInitialiser;
    for fail in [false, true] {
        let mut simulator = FakeSimulator::new(Duration::ZERO);
        simulator.string_reads.push_back(Ok("A20N".to_owned()));
        if fail {
            simulator.queue_reads("L:A32NX_IS_READY", [1.0]);
            simulator.failure = Some(Failure::LocalVariableExists("L:A32NX_IS_READY".to_owned()));
        }
        assert!(!A32nxInitialiser::default().supported(&mut simulator));
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
fn unsupported_or_unidentified_aircraft_enter_running_through_initialising() {
    for model in [Some("C172"), Some("A20N"), Some(""), None] {
        let fixture = initialisation_fixture();
        let mut simulator = FakeSimulator::new(duration(100.0));
        simulator.queue_reads(ARMED_VARIABLE, [1.0, 1.0]);
        if let Some(model) = model {
            simulator.string_reads.push_back(Ok(model.to_owned()));
        }
        let config_path = fixture.config_path.clone();
        let simulator = Rc::new(RefCell::new(simulator));
        let mut runtime = Runtime::new(
            config_path,
            Box::new(SharedSimulator(Rc::clone(&simulator))),
            Box::new(A32nxInitialiser::default()),
        )
        .unwrap();
        runtime.pre_update().unwrap(); // Unsupported aircraft must skip loading operations.
        assert_no_replay_writes(&simulator.borrow());
        assert_no_telemetry(&fixture);
        simulator.borrow_mut().time = duration(100.25);
        simulator.borrow_mut().failure = Some(Failure::Read(ARMED_VARIABLE.to_owned()));
        runtime.pre_update().unwrap();
        assert_no_replay_writes(&simulator.borrow());
        assert_eq!(fixture.telemetry_contents().lines().count(), 1);
        simulator.borrow_mut().time = duration(100.75);
        runtime.pre_update().unwrap();
        simulator.borrow_mut().time = duration(101.25);
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
        assert!(!simulator.borrow().operations.iter().any(|operation| matches!(
            operation,
            Operation::Write { variable, .. } if variable != "K:AXIS_ELEVATOR_SET" && variable != ARMED_VARIABLE
        )));
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
    simulator.queue_reads("L:A32NX_AIRFRAME_ZFW", [59000.0]);
    simulator.queue_reads("L:A32NX_AIRFRAME_GW", [64000.0]);
    simulator.queue_reads("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", [25.0]);
    let config_path = fixture.config_path.clone();
    let simulator = Rc::new(RefCell::new(simulator));
    let mut runtime = Runtime::new(
        config_path,
        Box::new(SharedSimulator(Rc::clone(&simulator))),
        Box::new(A32nxInitialiser::default()),
    )
    .unwrap();
    runtime.pre_update().unwrap();
    runtime.stop().unwrap();
    fixture.clear_telemetry_files();
    runtime.pre_update().unwrap();
    runtime.pre_update().unwrap();
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
fn a32nx_readiness_reads_fresh_actual_values_in_native_units_without_writes() {
    use testpilot_core::aircraft_initialisation::AircraftInitialiser;
    let mut simulator = FakeSimulator::new(Duration::ZERO);
    simulator.queue_reads("L:A32NX_AIRFRAME_ZFW", [60000.0, 60100.0]);
    simulator.queue_reads("L:A32NX_AIRFRAME_GW", [65000.0, 65100.0]);
    simulator.queue_reads("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", [25.0, 25.01]);
    let mut initialiser = A32nxInitialiser::default();
    initialiser
        .submit(
            &mut simulator,
            InitialisationConfig {
                zfw: TARGETS.zfw,
                gw: TARGETS.gw,
                gwcg: TARGETS.gwcg,
                ..InitialisationConfig::default()
            },
        )
        .unwrap();
    for _ in 0..2 {
        simulator.clear_operations();
        assert!(initialiser.is_ready(&mut simulator, TARGETS).unwrap());
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
    use testpilot_core::aircraft_initialisation::AircraftInitialiser;
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
            let mut initialiser = A32nxInitialiser::default();
            initialiser
                .submit(
                    &mut simulator,
                    InitialisationConfig {
                        zfw: TARGETS.zfw,
                        gw: TARGETS.gw,
                        gwcg: TARGETS.gwcg,
                        ..InitialisationConfig::default()
                    },
                )
                .unwrap();
            simulator.clear_operations();
            let error = initialiser.is_ready(&mut simulator, TARGETS).unwrap_err();
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
                        variable, ..
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
fn readiness_checks_every_configured_group_even_while_waiting() {
    let targets = InitialisationConfig {
        ths: Some(1.0),
        flaps: Some(2),
        landing_gear: Some(LandingGear::Down),
        ..TARGETS
    };
    let ready_values = [
        ("L:A32NX_AIRFRAME_ZFW", 60000.0),
        ("L:A32NX_AIRFRAME_GW", 65000.0),
        ("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", 25.0),
        ("L:A32NX_HYD_TRIM_WHEEL_PERCENT", 5.0 * 100.0 / 17.5),
        (MOVEMENT_VARIABLES[0], 0.0),
        (MOVEMENT_VARIABLES[1], 0.0),
        (GEAR_VARIABLES[0], packed(DOWN, 3)),
        (GEAR_VARIABLES[1], packed(DOWN, 0)),
    ];
    for waiting in [
        None,
        Some(("L:A32NX_AIRFRAME_ZFW", 60101.0)),
        Some(("L:A32NX_AIRFRAME_GW", 65101.0)),
        Some(("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", 26.0)),
        Some(("L:A32NX_HYD_TRIM_WHEEL_PERCENT", 0.0)),
        Some((MOVEMENT_VARIABLES[0], 1.0)),
        Some((MOVEMENT_VARIABLES[1], 1.0)),
        Some((GEAR_VARIABLES[0], packed(DOWN, 0))),
    ] {
        let mut simulator = FakeSimulator::new(Duration::ZERO);
        for (variable, ready_value) in ready_values {
            let actual = match waiting {
                Some((source, value)) if source == variable => value,
                _ => ready_value,
            };
            simulator.queue_reads(variable, [actual]);
        }
        let mut initialiser = A32nxInitialiser::default();
        initialiser.submit(&mut simulator, targets).unwrap();
        simulator.clear_operations();
        assert_eq!(
            initialiser.is_ready(&mut simulator, targets).unwrap(),
            waiting.is_none(),
            "waiting group: {waiting:?}"
        );
        assert_eq!(
            simulator.operations,
            ready_values.map(|(variable, _)| Operation::Read {
                variable: variable.to_owned(),
                unit: None,
            })
        );
    }
}

#[test]
fn later_malformed_readback_fails_after_an_unsettled_mass() {
    let targets = InitialisationConfig {
        flaps: Some(2),
        ..TARGETS
    };
    let mut simulator = FakeSimulator::new(Duration::ZERO);
    for (variable, value) in [
        ("L:A32NX_AIRFRAME_ZFW", 61000.0),
        ("L:A32NX_AIRFRAME_GW", 65000.0),
        ("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", 25.0),
        (MOVEMENT_VARIABLES[0], 0.0),
        (MOVEMENT_VARIABLES[1], 2.0),
    ] {
        simulator.queue_reads(variable, [value]);
    }
    let mut initialiser = A32nxInitialiser::default();
    initialiser.submit(&mut simulator, targets).unwrap();
    simulator.clear_operations();
    assert!(matches!(
        initialiser.is_ready(&mut simulator, targets),
        Err(InitialisationError::InvalidReadback { variable, value: 2.0, .. })
            if variable == MOVEMENT_VARIABLES[1]
    ));
    assert_eq!(simulator.operations.len(), 5);
    assert!(
        simulator
            .operations
            .iter()
            .all(|operation| matches!(operation, Operation::Read { unit: None, .. }))
    );
}

#[test]
fn initialiser_construction_validates_interfaces_without_commands_or_readback() {
    for targets in [
        InitialisationConfig::default(),
        InitialisationConfig {
            zfw: TARGETS.zfw,
            gw: TARGETS.gw,
            gwcg: TARGETS.gwcg,
            ths: Some(1.0),
            flaps: Some(2),
            landing_gear: Some(LandingGear::Down),
        },
    ] {
        let mut simulator = FakeSimulator::new(Duration::ZERO);
        for variable in ["L:A32NX_HYD_TRIM_WHEEL_PERCENT"]
            .into_iter()
            .chain(MOVEMENT_VARIABLES)
            .chain(GEAR_VARIABLES)
        {
            simulator.queue_reads(variable, []);
        }
        let _initialiser = A32nxInitialiser::new(targets, &mut simulator).unwrap();
        if targets.has_targets() {
            assert!(!simulator.operations.is_empty());
        } else {
            assert!(simulator.operations.is_empty());
        }
        assert!(simulator.operations.iter().all(|operation| matches!(
            operation,
            Operation::LocalVariableExists(_) | Operation::ValidateRead { unit: None, .. }
        )));
    }
}

#[test]
fn successive_submissions_prepare_updated_targets_and_validate_each_time() {
    let mut simulator = FakeSimulator::new(Duration::ZERO);
    for variable in ["L:A32NX_HYD_TRIM_WHEEL_PERCENT"]
        .into_iter()
        .chain(MOVEMENT_VARIABLES)
        .chain(GEAR_VARIABLES)
    {
        simulator.queue_reads(variable, []);
    }
    let mut initialiser = A32nxInitialiser::default();
    for (ths, flaps, landing_gear, axis, flap_event, gear_event) in [
        (
            -4.0,
            0,
            LandingGear::Down,
            -16383.0,
            "K:FLAPS_UP",
            "K:GEAR_DOWN",
        ),
        (
            13.5,
            4,
            LandingGear::Up,
            16384.0,
            "K:FLAPS_DOWN",
            "K:GEAR_UP",
        ),
    ] {
        simulator.clear_operations();
        initialiser
            .submit(
                &mut simulator,
                InitialisationConfig {
                    ths: Some(ths),
                    flaps: Some(flaps),
                    landing_gear: Some(landing_gear),
                    ..InitialisationConfig::default()
                },
            )
            .unwrap();
        let writes: Vec<_> = simulator
            .operations
            .iter()
            .filter_map(|operation| match operation {
                Operation::Write { variable, value } => Some((variable.as_str(), *value)),
                _ => None,
            })
            .collect();
        assert_eq!(
            writes,
            [
                ("K:AXIS_ELEV_TRIM_SET", axis),
                (flap_event, 0.0),
                (gear_event, 0.0),
            ]
        );
        for variable in ["L:A32NX_HYD_TRIM_WHEEL_PERCENT"]
            .into_iter()
            .chain(MOVEMENT_VARIABLES)
            .chain(GEAR_VARIABLES)
        {
            assert!(simulator.operations.contains(&Operation::ValidateRead {
                variable: variable.to_owned(),
                unit: None,
            }));
        }
    }
}

#[test]
fn unsubmitted_initialiser_with_no_targets_performs_no_readback_io() {
    let mut simulator = FakeSimulator::new(Duration::ZERO);
    let initialiser = A32nxInitialiser::default();
    assert!(
        initialiser
            .is_ready(&mut simulator, InitialisationConfig::default())
            .unwrap()
    );
    assert!(simulator.operations.is_empty());
}

#[test]
fn resubmissions_replace_readiness_groups_only_after_success() {
    let targets = InitialisationConfig {
        zfw: TARGETS.zfw,
        gw: TARGETS.gw,
        gwcg: TARGETS.gwcg,
        ths: Some(1.0),
        flaps: Some(2),
        landing_gear: Some(LandingGear::Down),
    };
    let replacement = InitialisationConfig {
        ths: Some(13.5),
        flaps: Some(3),
        landing_gear: Some(LandingGear::Up),
        ..targets
    };
    for (next_targets, failure) in [
        (InitialisationConfig::default(), None),
        (
            InitialisationConfig {
                flaps: Some(5),
                ..targets
            },
            None,
        ),
        (
            InitialisationConfig {
                ths: Some(14.0),
                ..targets
            },
            None,
        ),
        (
            InitialisationConfig {
                gwcg: Some(99.0),
                ..targets
            },
            None,
        ),
        (
            replacement,
            Some(Failure::LocalVariableExists(
                MOVEMENT_VARIABLES[0].to_owned(),
            )),
        ),
        (
            replacement,
            Some(Failure::ValidateRead(GEAR_VARIABLES[1].to_owned())),
        ),
        (
            replacement,
            Some(Failure::Write("L:A32NX_PAX_A_DESIRED".to_owned())),
        ),
        (
            replacement,
            Some(Failure::Write("K:AXIS_ELEV_TRIM_SET".to_owned())),
        ),
        (replacement, Some(Failure::Write("K:FLAPS_3".to_owned()))),
        (replacement, Some(Failure::Write("K:GEAR_UP".to_owned()))),
    ] {
        let mut simulator = FakeSimulator::new(Duration::ZERO);
        simulator.queue_reads("L:A32NX_AIRFRAME_ZFW", [60000.0; 2]);
        simulator.queue_reads("L:A32NX_AIRFRAME_GW", [65000.0; 2]);
        simulator.queue_reads("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", [25.0; 2]);
        simulator.queue_reads("L:A32NX_HYD_TRIM_WHEEL_PERCENT", [50.0; 2]);
        for variable in MOVEMENT_VARIABLES {
            simulator.queue_reads(variable, [0.0; 2]);
        }
        for variable in GEAR_VARIABLES {
            simulator.queue_reads(variable, [packed(DOWN, 3); 2]);
        }
        let mut initialiser = A32nxInitialiser::default();
        initialiser.submit(&mut simulator, targets).unwrap();
        assert!(!initialiser.is_ready(&mut simulator, targets).unwrap());

        let previous = initialiser;
        simulator.failure = failure;
        let result = initialiser.submit(&mut simulator, next_targets);
        assert_eq!(result.is_ok(), !next_targets.has_targets());
        let readiness_targets = if result.is_ok() {
            InitialisationConfig::default()
        } else {
            assert_eq!(initialiser, previous);
            targets
        };
        simulator.clear_operations();
        assert_eq!(
            initialiser
                .is_ready(&mut simulator, readiness_targets)
                .unwrap(),
            result.is_ok()
        );
        assert_eq!(simulator.operations.is_empty(), result.is_ok());
    }
}

#[test]
fn unreachable_a32nx_loading_fails_safely_without_replay_or_telemetry() {
    let fixture = Fixture::new(&format!(
        "{CONFIG}\n[initialisation]\nzfw = 60000.0\ngw = 65000.0\ngwcg = 99.0\n"
    ));
    let mut simulator = FakeSimulator::new(duration(100.0));
    simulator.queue_reads(ARMED_VARIABLE, [1.0]);
    simulator
        .string_reads
        .push_back(Ok("TT:ATCCOM.AC_MODEL_A20N.0.text".to_owned()));
    simulator.queue_reads("L:A32NX_IS_READY", [1.0]);
    let config_path = fixture.config_path.clone();
    let simulator = Rc::new(RefCell::new(simulator));
    let mut runtime = Runtime::new(
        config_path,
        Box::new(SharedSimulator(Rc::clone(&simulator))),
        Box::new(A32nxInitialiser::default()),
    )
    .unwrap();
    runtime.pre_update().unwrap();
    let error = runtime.pre_update().unwrap_err();
    assert!(matches!(
        error.downcast_ref::<InitialisationError>(),
        Some(InitialisationError::UnreachableLoading { .. })
    ));
    runtime.stop().unwrap();
    assert_no_replay_writes(&simulator.borrow());
    assert_no_telemetry(&fixture);
}

/// Packs a discrete payload using A32NX's native numeric f32 transport.
fn packed(word: u32, status: u64) -> f64 {
    (u64::from((word as f32).to_bits()) | (status << 32)) as f64
}

/// Complete downlocked gear with doors closed and selection acknowledged.
const DOWN: u32 = (1 << 28) | (0b111 << 22);

/// Builds a run with only lever setup and no telemetry read dependencies.
fn lever_runtime(settings: &str) -> (Fixture, Runtime, SimulatorHandle) {
    let (input, _) = CONFIG.split_once("[record.0]").unwrap();
    let fixture = Fixture::new(&format!("{input}\n[initialisation]\n{settings}\n"));
    let mut simulator = FakeSimulator::new(duration(100.0));
    simulator
        .string_reads
        .extend((0..2).map(|_| Ok("A20N".to_owned())));
    simulator.queue_reads("L:A32NX_IS_READY", [0.0]);
    simulator.queue_reads(ARMED_VARIABLE, [1.0, 1.0]);
    for variable in MOVEMENT_VARIABLES {
        simulator.queue_reads(variable, [0.0; 16]);
    }
    for variable in GEAR_VARIABLES {
        simulator.queue_reads(variable, [packed(DOWN, 3); 16]);
    }
    let simulator = Rc::new(RefCell::new(simulator));
    let runtime = Runtime::new(
        fixture.config_path.clone(),
        Box::new(SharedSimulator(Rc::clone(&simulator))),
        Box::new(A32nxInitialiser::default()),
    )
    .unwrap();
    (fixture, runtime, simulator)
}

/// Replaces the queued observations for one native interface.
fn observations(
    simulator: &SimulatorHandle,
    variable: &str,
    values: impl IntoIterator<Item = f64>,
) {
    simulator
        .borrow_mut()
        .reads
        .insert(variable.to_owned(), values.into_iter().collect());
}

#[test]
fn each_lever_commands_only_its_group_and_stopped_flaps_are_immediately_ready() {
    for (settings, event, variables) in [
        ("flaps = 0", "K:FLAPS_UP", MOVEMENT_VARIABLES),
        ("flaps = 1", "K:FLAPS_1", MOVEMENT_VARIABLES),
        ("flaps = 2", "K:FLAPS_2", MOVEMENT_VARIABLES),
        ("flaps = 3", "K:FLAPS_3", MOVEMENT_VARIABLES),
        ("flaps = 4", "K:FLAPS_DOWN", MOVEMENT_VARIABLES),
        ("landing_gear = 'down'", "K:GEAR_DOWN", GEAR_VARIABLES),
        ("landing_gear = 'up'", "K:GEAR_UP", GEAR_VARIABLES),
    ] {
        let (fixture, mut runtime, simulator) = lever_runtime(settings);
        if event == "K:GEAR_UP" {
            for variable in GEAR_VARIABLES {
                observations(&simulator, variable, [packed(0, 3)]);
            }
        }
        runtime.pre_update().unwrap();
        assert_no_telemetry(&fixture);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap(); // Same timestamp; there is no submission-time gate.
        assert_eq!(fixture.telemetry_contents().lines().count(), 1);
        let operations = simulator.borrow().operations.clone();
        assert!(operations.iter().any(
            |op| matches!(op, Operation::Write { variable, value: 0.0 } if variable == event)
        ));
        assert!(operations.iter().all(|op| match op {
            Operation::Write { variable, value } => variable == event && *value == 0.0,
            Operation::Read { variable, unit } | Operation::ValidateRead { variable, unit } =>
                variables.contains(&variable.as_str()) && unit.is_none(),
            Operation::LocalVariableExists(variable) => variables.contains(&variable.as_str()),
            _ => false,
        }));
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert!(simulator.borrow().operations.iter().all(|op| matches!(op, Operation::Write { variable, .. } if variable == "K:AXIS_ELEVATOR_SET")));
        runtime.stop().unwrap();
        assert_eq!(fixture.telemetry_contents().lines().count(), 2);
    }
}

#[test]
fn both_motion_flags_and_all_gear_locks_gate_playback_and_commands_are_repeated() {
    let (fixture, mut runtime, simulator) = lever_runtime("flaps = 2\nlanding_gear = 'down'");
    observations(&simulator, MOVEMENT_VARIABLES[0], [1.0, 0.0, 0.0, 0.0]);
    observations(&simulator, MOVEMENT_VARIABLES[1], [0.0, 1.0, 0.0, 0.0]);
    for variable in GEAR_VARIABLES {
        observations(
            &simulator,
            variable,
            [
                packed(DOWN, 3),
                packed(DOWN, 3),
                packed(DOWN | (1 << 16), 3),
                packed(DOWN, 3),
            ],
        );
    }
    runtime.pre_update().unwrap();
    for now in [100.5, 101.0, 101.5] {
        simulator.borrow_mut().time = duration(now);
        runtime.pre_update().unwrap();
        assert_no_telemetry(&fixture);
        assert!(!simulator.borrow().operations.iter().any(|op| matches!(op, Operation::Write { variable, .. } if variable == "K:AXIS_ELEVATOR_SET")));
    }
    simulator.borrow_mut().time = duration(102.0);
    runtime.pre_update().unwrap();
    for event in ["K:FLAPS_2", "K:GEAR_DOWN"] {
        assert_eq!(simulator.borrow().operations.iter().filter(|op| matches!(op, Operation::Write { variable, value: 0.0 } if variable == event)).count(), 4);
    }
    assert_eq!(fixture.telemetry_contents().lines().count(), 1);
    simulator.borrow_mut().clear_operations();
    runtime.pre_update().unwrap();
    runtime.stop().unwrap();
    assert_eq!(fixture.telemetry_contents().lines().count(), 2);
}

#[test]
fn gear_accepts_one_valid_complete_computer_without_combining_partial_locks() {
    for (first, second, down_ready, up_ready) in [
        (packed(DOWN, 0), packed(DOWN, 3), true, false),
        (packed(DOWN, 3), packed(DOWN, 1), true, false),
        (packed(DOWN, 2), packed(DOWN, 0), false, false),
        (
            packed(DOWN & !(1 << 22), 3),
            packed(DOWN & !(1 << 23), 3),
            false,
            false,
        ),
        (packed(0, 3), packed(0, 3), false, true),
        (packed(0, 0), packed(0, 3), false, true),
        (packed(0, 2), packed(0, 0), false, false),
        (packed(1 << 10, 3), packed(1 << 11, 3), false, false),
    ] {
        let (_, _, simulator) = lever_runtime("landing_gear = 'down'");
        observations(&simulator, GEAR_VARIABLES[0], [first; 2]);
        observations(&simulator, GEAR_VARIABLES[1], [second; 2]);
        let mut initialiser = A32nxInitialiser::default();
        let mut simulator = simulator.borrow_mut();
        initialiser
            .submit(
                &mut *simulator,
                InitialisationConfig {
                    landing_gear: Some(LandingGear::Down),
                    ..InitialisationConfig::default()
                },
            )
            .unwrap();
        simulator.clear_operations();
        for (target, ready) in [(LandingGear::Down, down_ready), (LandingGear::Up, up_ready)] {
            assert_eq!(
                initialiser
                    .check_landing_gear(&mut *simulator, Some(target))
                    .unwrap(),
                ready
            );
        }
        assert_eq!(simulator.operations.len(), 4);
        assert!(
            simulator
                .operations
                .iter()
                .all(|operation| matches!(operation, Operation::Read { unit: None, .. }))
        );
    }
}

#[test]
fn missing_lever_interfaces_prevent_all_setup_writes() {
    for variable in MOVEMENT_VARIABLES.into_iter().chain(GEAR_VARIABLES) {
        let (fixture, mut runtime, simulator) =
            lever_runtime("flaps = 2\nlanding_gear = 'down'\nths = 1.0");
        simulator
            .borrow_mut()
            .queue_reads("L:A32NX_HYD_TRIM_WHEEL_PERCENT", [0.0]);
        simulator.borrow_mut().reads.remove(variable);
        runtime.pre_update().unwrap();
        simulator.borrow_mut().clear_operations();
        let error = runtime.pre_update().unwrap_err();
        assert!(
            matches!(error.downcast_ref::<InitialisationError>(), Some(InitialisationError::MissingInterface { variable: actual }) if *actual == variable)
        );
        assert_no_replay_writes(&simulator.borrow());
        assert_no_telemetry(&fixture);
        assert!(
            matches!(simulator.borrow().operations.last(), Some(Operation::Write { variable, value: 0.0 }) if variable == ARMED_VARIABLE)
        );
    }
}

#[test]
fn lever_failures_clean_up_and_allow_a_subsequent_run() {
    for failure in [
        Failure::Write("K:FLAPS_2".to_owned()),
        Failure::Write("K:GEAR_DOWN".to_owned()),
        Failure::Read(MOVEMENT_VARIABLES[0].to_owned()),
        Failure::Read(MOVEMENT_VARIABLES[1].to_owned()),
        Failure::Read(GEAR_VARIABLES[0].to_owned()),
        Failure::Read(GEAR_VARIABLES[1].to_owned()),
        Failure::ValidateRead(MOVEMENT_VARIABLES[0].to_owned()),
        Failure::ValidateRead(GEAR_VARIABLES[1].to_owned()),
        Failure::LocalVariableExists(GEAR_VARIABLES[0].to_owned()),
    ] {
        let (fixture, mut runtime, simulator) = lever_runtime("flaps = 2\nlanding_gear = 'down'");
        runtime.pre_update().unwrap();
        simulator.borrow_mut().failure = Some(failure);
        let error = runtime.pre_update().unwrap_err();
        assert!(error.downcast_ref::<InitialisationError>().is_some());
        assert_no_telemetry(&fixture);
        assert!(
            matches!(simulator.borrow().operations.last(), Some(Operation::Write { variable, value: 0.0 }) if variable == ARMED_VARIABLE)
        );
        simulator.borrow_mut().failure = None;
        runtime.pre_update().unwrap(); // Arming is reset internally; a new edge starts another run.
        runtime.pre_update().unwrap();
        assert_eq!(fixture.telemetry_contents().lines().count(), 1);
        runtime.stop().unwrap();
    }
}

#[test]
fn invalid_motion_or_packed_gear_values_fail_instead_of_becoming_ready() {
    for (variable, value) in [
        (MOVEMENT_VARIABLES[0], 2.0),
        (MOVEMENT_VARIABLES[1], -1.0),
        (GEAR_VARIABLES[0], 0.5),
        (GEAR_VARIABLES[1], -1.0),
    ] {
        let (fixture, mut runtime, simulator) = lever_runtime("flaps = 2\nlanding_gear = 'down'");
        observations(&simulator, variable, [value]);
        runtime.pre_update().unwrap();
        let error = runtime.pre_update().unwrap_err();
        assert!(
            matches!(error.downcast_ref::<InitialisationError>(), Some(InitialisationError::InvalidReadback { variable: actual, .. }) if *actual == variable)
        );
        assert_no_telemetry(&fixture);
        runtime.stop().unwrap();
    }
}

#[test]
fn stalled_movement_times_out_before_more_commands_and_reports_targets() {
    let (fixture, mut runtime, simulator) = lever_runtime("flaps = 2\nlanding_gear = 'down'");
    observations(&simulator, MOVEMENT_VARIABLES[1], [1.0]);
    observations(&simulator, GEAR_VARIABLES[0], [packed(DOWN, 0)]);
    runtime.pre_update().unwrap();
    runtime.pre_update().unwrap();
    simulator.borrow_mut().clear_operations();
    simulator.borrow_mut().time = duration(130.0);
    let error = runtime.pre_update().unwrap_err();
    assert!(matches!(
        error.downcast_ref::<InitialisationError>(),
        Some(InitialisationError::Timeout { targets })
            if targets.flaps == Some(2) && targets.landing_gear == Some(LandingGear::Down)
    ));
    assert!(simulator.borrow().operations.iter().all(|operation|
        matches!(operation, Operation::Write { variable, value: 0.0 } if variable == ARMED_VARIABLE)));
    assert_no_replay_writes(&simulator.borrow());
    assert_no_telemetry(&fixture);
    runtime.stop().unwrap();
}

#[test]
fn invalid_programmatic_detents_are_rejected_before_writes() {
    let (_, _, simulator) = lever_runtime("flaps = 2");
    let mut initialiser = A32nxInitialiser::default();
    let mut simulator = simulator.borrow_mut();
    simulator.clear_operations();
    assert!(matches!(
        initialiser.submit(
            &mut *simulator,
            InitialisationConfig {
                flaps: Some(5),
                ..InitialisationConfig::default()
            }
        ),
        Err(InitialisationError::InvalidFlapsTarget { detent: 5 })
    ));
    assert!(simulator.operations.is_empty());
}
