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
use testpilot_core::error::{CleanupError, InitialisationError, SimulatorError, TerminationError};
use testpilot_core::runtime::Runtime;
use testpilot_core::simulator::SimulatorAdapter;

use super::{A32nxInitialiser, AIRCRAFT_READY, DEVELOPER_STATE};

use crate::airspeed::{CALIBRATED_AIRSPEED, VERTICAL_SPEED};
use crate::altitude::{PLANE_ALTITUDE, PRESSURE_ALTITUDE};
use crate::attitude::ATTITUDE_VARIABLES;
use crate::flaps::MOVEMENT_VARIABLES;
use crate::flight_path::{FCDC_VARIABLES, STICK_EVENTS};
use crate::landing_gear::{AIRSPEED_MACH, GEAR_VARIABLES, SIM_ON_GROUND};
use crate::throttle::{IDLE_AXIS, THROTTLE_EVENTS};

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
    write_units: Vec<(String, Option<String>)>,
    echo_airspeed_write: bool,
    failure: Option<Failure>,
    fail_arming_reset: bool,
    string_reads: VecDeque<Result<String, SimulatorError>>,
}

impl FakeSimulator {
    fn new(time: Duration) -> Self {
        Self {
            time,
            reads: HashMap::from([
                (DEVELOPER_STATE.to_owned(), VecDeque::from([0.0])),
                (CALIBRATED_AIRSPEED.to_owned(), VecDeque::from([0.0; 64])),
                (AIRSPEED_MACH.to_owned(), VecDeque::from([0.0; 64])),
            ]),
            operations: Vec::new(),
            write_units: Vec::new(),
            echo_airspeed_write: false,
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
        self.write_units.clear();
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

    fn write(
        &mut self,
        variable: &str,
        value: f64,
        _unit: Option<&str>,
    ) -> Result<(), SimulatorError> {
        self.0.borrow_mut().write(variable, value, _unit)
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

    fn write(
        &mut self,
        variable: &str,
        value: f64,
        unit: Option<&str>,
    ) -> Result<(), SimulatorError> {
        self.operations.push(Operation::Write {
            variable: variable.to_owned(),
            value,
        });
        self.write_units
            .push((variable.to_owned(), unit.map(ToOwned::to_owned)));
        if self.should_fail(&Failure::Write(variable.to_owned())) {
            return Err(SimulatorError::CalculatorCodeWriteFailed {
                variable: variable.to_owned(),
                value,
            });
        }
        if self.echo_airspeed_write && variable == CALIBRATED_AIRSPEED {
            self.reads
                .insert(variable.to_owned(), VecDeque::from([value]));
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
    pressure_altitude: None,
    calibrated_airspeed: None,
    vertical_speed: None,
    pitch: None,
    roll: None,
};

/// Confirms loading before exercising later readiness checks, preserving queued observations.
fn finish_loading(
    initialiser: &mut A32nxInitialiser,
    simulator: &mut FakeSimulator,
    targets: InitialisationConfig,
) {
    for (variable, value) in [
        ("L:A32NX_AIRFRAME_ZFW", targets.zfw),
        ("L:A32NX_AIRFRAME_GW", targets.gw),
        ("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", targets.gwcg),
    ] {
        simulator
            .reads
            .entry(variable.to_owned())
            .or_default()
            .push_front(value.unwrap());
    }
    let pending = initialiser.pending_targets(simulator, targets).unwrap();
    assert!(!pending.has_mass_balance());
    initialiser
        .submit(simulator, targets, Duration::ZERO)
        .unwrap();
}

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
        Operation::Write { variable, .. } if variable != ARMED_VARIABLE && variable != AIRCRAFT_READY))
    );
}

fn write_operations(simulator: &FakeSimulator) -> Vec<Operation> {
    simulator
        .operations
        .iter()
        .filter(|op| matches!(op, Operation::Write { .. }))
        .cloned()
        .collect()
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
        DEVELOPER_STATE,
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
            variable == ARMED_VARIABLE
                || variable == AIRCRAFT_READY
                || variable == "K:AXIS_ELEV_TRIM_SET",
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
fn cached_loading_plan_does_not_bypass_validation_of_changed_mass_targets() {
    for invalid in [
        InitialisationConfig {
            zfw: Some(f64::NAN),
            ..TARGETS
        },
        InitialisationConfig {
            gw: Some(80000.0),
            ..TARGETS
        },
        InitialisationConfig {
            gwcg: None,
            ..TARGETS
        },
    ] {
        let mut simulator = FakeSimulator::new(Duration::ZERO);
        let mut aircraft = A32nxInitialiser::default();
        aircraft
            .submit(&mut simulator, TARGETS, Duration::ZERO)
            .unwrap();
        simulator.clear_operations();
        assert!(matches!(
            aircraft.submit(&mut simulator, invalid, duration(0.1)),
            Err(InitialisationError::InvalidLoadingTargets { .. })
        ));
        assert!(simulator.operations.is_empty());
    }
}

#[test]
fn loading_finishes_before_trim_setup_and_is_not_restarted_during_trim_verification() {
    let (fixture, mut runtime, simulator) = trim_runtime();
    runtime.pre_update().unwrap();
    for now in [100.5, 101.0, 102.0] {
        simulator.borrow_mut().time = duration(now);
        runtime.pre_update().unwrap();
        assert_no_telemetry(&fixture);
        assert!(!simulator.borrow().operations.iter().any(|op| matches!(op, Operation::Write { variable, .. } if variable == "K:AXIS_ELEVATOR_SET")));
    }
    assert_eq!(simulator.borrow().operations.iter().filter(|op|
        matches!(op, Operation::Write { variable, value: 1.0 } if variable == "K:AXIS_ELEV_TRIM_SET")
    ).count(), 2);
    // The first setup update loads; subsequent updates only retry trim.
    assert_eq!(simulator.borrow().operations.iter().filter(|op|
        matches!(op, Operation::Write { variable, .. } if variable == "L:A32NX_BOARDING_STARTED_BY_USR")
    ).count(), 1);
    simulator.borrow_mut().clear_operations();
    simulator.borrow_mut().time = duration(102.5);
    runtime.pre_update().unwrap();
    simulator.borrow_mut().clear_operations();
    simulator.borrow_mut().time = duration(103.0);
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
        if matches!(&failure, Some(Failure::Read(variable)) if variable == "L:A32NX_HYD_TRIM_WHEEL_PERCENT")
            || matches!(&failure, Some(Failure::Write(variable)) if variable == "K:AXIS_ELEV_TRIM_SET")
        {
            runtime.pre_update().unwrap();
        }
        let timeout = failure.is_none();
        simulator.borrow_mut().failure = failure;
        simulator.borrow_mut().time = duration(if timeout { 160.0 } else { 101.0 });
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
        pressure_altitude: None,
        calibrated_airspeed: None,
        vertical_speed: None,
        pitch: None,
        roll: None,
    };
    simulator.queue_reads("L:A32NX_AIRFRAME_ZFW", [60000.0; 4]);
    simulator.queue_reads("L:A32NX_AIRFRAME_GW", [65000.0; 4]);
    simulator.queue_reads("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", [25.0; 4]);
    simulator.queue_reads("L:A32NX_HYD_TRIM_WHEEL_PERCENT", [0.0, 50.0, 100.0]);
    initialiser
        .submit(&mut simulator, targets, Duration::ZERO)
        .unwrap();
    finish_loading(&mut initialiser, &mut simulator, targets);
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
            Duration::ZERO,
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
                    pressure_altitude: None,
                    calibrated_airspeed: None,
                    vertical_speed: None,
                    pitch: None,
                    roll: None,
                },
                Duration::ZERO,
            )
            .unwrap();
        finish_loading(
            &mut initialiser,
            &mut simulator,
            InitialisationConfig {
                ths: Some(1.0),
                ..TARGETS
            },
        );
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
fn ths_and_flight_path_reject_out_of_range_trim_percentages() {
    for flight_setup in [false, true] {
        for percent in [-0.01, 100.01] {
            let (_, _, simulator) = flight_runtime(if flight_setup {
                "vertical_speed = 0.0"
            } else {
                "ths = 1.0"
            });
            let targets = InitialisationConfig {
                ths: (!flight_setup).then_some(1.0),
                vertical_speed: flight_setup.then_some(0.0),
                ..InitialisationConfig::default()
            };
            let mut aircraft = A32nxInitialiser::default();
            aircraft
                .submit(&mut *simulator.borrow_mut(), targets, duration(100.0))
                .unwrap();
            observations(&simulator, crate::ths::TRIM_POSITION, [percent]);
            let result = if flight_setup {
                aircraft.submit(&mut *simulator.borrow_mut(), targets, duration(100.5))
            } else {
                aircraft
                    .is_ready(&mut *simulator.borrow_mut(), targets)
                    .map(|_| ())
            };
            assert!(
                matches!(result, Err(InitialisationError::InvalidReadback { variable: crate::ths::TRIM_POSITION, value, .. }) if value == percent)
            );
        }
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
            Duration::ZERO,
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
                    Duration::ZERO,
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
        initialiser
            .submit(&mut simulator, targets, Duration::ZERO)
            .unwrap();
        finish_loading(&mut initialiser, &mut simulator, targets);
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
    initialiser
        .submit(&mut simulator, targets, Duration::ZERO)
        .unwrap();
    finish_loading(&mut initialiser, &mut simulator, targets);
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
fn initialiser_construction_validates_configured_interfaces_before_commands() {
    for targets in [
        InitialisationConfig::default(),
        InitialisationConfig {
            zfw: TARGETS.zfw,
            gw: TARGETS.gw,
            gwcg: TARGETS.gwcg,
            ths: Some(1.0),
            flaps: Some(2),
            landing_gear: Some(LandingGear::Down),
            pressure_altitude: None,
            calibrated_airspeed: None,
            vertical_speed: None,
            pitch: None,
            roll: None,
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
        let _initialiser = A32nxInitialiser::prepare(
            targets,
            &mut simulator,
            Duration::ZERO,
            &A32nxInitialiser::default(),
        )
        .unwrap();
        if targets.has_targets() {
            assert!(!simulator.operations.is_empty());
        } else {
            assert!(simulator.operations.is_empty());
        }
        assert!(
            simulator
                .operations
                .iter()
                .all(|operation| match operation {
                    Operation::LocalVariableExists(_)
                    | Operation::ValidateRead { unit: None, .. } => true,
                    Operation::Read { variable, unit }
                    | Operation::ValidateRead { variable, unit } => {
                        (variable == CALIBRATED_AIRSPEED && unit.as_deref() == Some("knots"))
                            || (variable == AIRSPEED_MACH && unit.as_deref() == Some("mach"))
                            || (variable == DEVELOPER_STATE && unit.is_none())
                    }
                    _ => false,
                })
        );
    }
}

#[test]
fn successive_submissions_prepare_updated_targets_and_validate_each_time() {
    let mut simulator = FakeSimulator::new(Duration::ZERO);
    simulator.queue_reads(SIM_ON_GROUND, [0.0]);
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
                Duration::ZERO,
            )
            .unwrap();
        let writes: Vec<_> = simulator
            .operations
            .iter()
            .filter_map(|operation| match operation {
                Operation::Write { variable, value } if variable != AIRCRAFT_READY => {
                    Some((variable.as_str(), *value))
                }
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
    let mut initialiser = A32nxInitialiser::default();
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
        pressure_altitude: None,
        calibrated_airspeed: None,
        vertical_speed: None,
        pitch: None,
        roll: None,
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
            Some(Failure::Write("K:AXIS_ELEV_TRIM_SET".to_owned())),
        ),
        (replacement, Some(Failure::Write("K:FLAPS_3".to_owned()))),
        (replacement, Some(Failure::Write("K:GEAR_UP".to_owned()))),
    ] {
        let mut simulator = FakeSimulator::new(Duration::ZERO);
        simulator.queue_reads(SIM_ON_GROUND, [0.0]);
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
        initialiser
            .submit(&mut simulator, targets, Duration::ZERO)
            .unwrap();
        assert!(!initialiser.is_ready(&mut simulator, targets).unwrap());

        let previous = initialiser;
        simulator.failure = failure;
        let result = initialiser.submit(&mut simulator, next_targets, Duration::ZERO);
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
    simulator.queue_reads(SIM_ON_GROUND, [0.0; 32]);
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
            Operation::Write { variable, .. } if variable == AIRCRAFT_READY => true,
            Operation::Write { variable, value } => variable == event && *value == 0.0,
            Operation::Read { variable, unit } | Operation::ValidateRead { variable, unit } => {
                if variable == SIM_ON_GROUND {
                    event == "K:GEAR_UP" && unit.as_deref() == Some("bool")
                } else if variable == CALIBRATED_AIRSPEED || variable == AIRSPEED_MACH {
                    event.starts_with("K:GEAR_")
                        && unit.as_deref()
                            == Some(if variable == CALIBRATED_AIRSPEED {
                                "knots"
                            } else {
                                "mach"
                            })
                } else {
                    variables.contains(&variable.as_str()) && unit.is_none()
                }
            }
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
fn gear_commands_require_both_ias_and_mach_within_the_operating_limits() {
    for (gear, event, maximum_ias, maximum_mach) in [
        (LandingGear::Up, "K:GEAR_UP", 220.0, 0.54),
        (LandingGear::Down, "K:GEAR_DOWN", 250.0, 0.60),
    ] {
        for (ias, mach, allowed) in [
            (maximum_ias, maximum_mach, true),
            (maximum_ias + 0.001, maximum_mach, false),
            (maximum_ias, maximum_mach + 0.001, false),
        ] {
            let (_, _, simulator) = lever_runtime("landing_gear = 'up'");
            observations(&simulator, CALIBRATED_AIRSPEED, [ias]);
            observations(&simulator, AIRSPEED_MACH, [mach]);
            let mut aircraft = A32nxInitialiser::default();
            aircraft
                .submit(
                    &mut *simulator.borrow_mut(),
                    InitialisationConfig {
                        landing_gear: Some(gear),
                        ..InitialisationConfig::default()
                    },
                    Duration::ZERO,
                )
                .unwrap();
            assert_eq!(
                writes_to(&simulator, event),
                allowed,
                "{gear:?}: {ias} kt/M{mach}"
            );
            assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
        }
    }
}

#[test]
fn already_retracted_gear_can_be_ready_above_the_retraction_operating_limit() {
    let (fixture, mut runtime, simulator) = lever_runtime("landing_gear = 'up'");
    observations(&simulator, CALIBRATED_AIRSPEED, [287.0]);
    observations(&simulator, AIRSPEED_MACH, [0.7]);
    for variable in GEAR_VARIABLES {
        observations(&simulator, variable, [packed(0, 3)]);
    }
    runtime.pre_update().unwrap();
    simulator.borrow_mut().clear_operations();
    runtime.pre_update().unwrap();
    assert!(!writes_to(&simulator, "K:GEAR_UP"));
    assert_eq!(fixture.telemetry_contents().lines().count(), 1);
    runtime.stop().unwrap();
}

#[test]
fn speed_waits_at_200_knots_for_gear_locks_and_closed_doors() {
    for target in [200.0, 287.0] {
        let (_, _, simulator) = flight_runtime("calibrated_airspeed = 287.0\nlanding_gear = 'up'");
        let targets = InitialisationConfig {
            landing_gear: Some(LandingGear::Up),
            calibrated_airspeed: Some(target),
            vertical_speed: None,
            ..InitialisationConfig::default()
        };
        let mut aircraft = A32nxInitialiser::default();
        for (now, word) in [
            (100.0, DOWN),
            (100.25, DOWN),
            (101.0, 1 << 16), // Up selection and uplocks alone do not confirm closed doors.
            (101.25, 1 << 16),
            (102.0, 1 << 16),
            (103.0, 0),
            (104.0, 0),
        ] {
            observations(&simulator, CALIBRATED_AIRSPEED, [200.0; 2]);
            for variable in GEAR_VARIABLES {
                observations(&simulator, variable, [packed(word, 3)]);
            }
            simulator.borrow_mut().clear_operations();
            aircraft
                .submit(&mut *simulator.borrow_mut(), targets, duration(now))
                .unwrap();
            let pending = aircraft
                .pending_targets(&mut *simulator.borrow_mut(), targets)
                .unwrap();
            assert_eq!(pending.landing_gear.is_some(), word != 0);
            assert_eq!(pending.calibrated_airspeed, Some(target));
            if target == 200.0 {
                assert_eq!(writes_to(&simulator, CALIBRATED_AIRSPEED), now == 100.0);
            }
            if writes_to(&simulator, CALIBRATED_AIRSPEED) {
                let demand = last_write(&simulator.borrow(), CALIBRATED_AIRSPEED);
                assert_eq!(demand, if now == 104.0 { 205.0 } else { 200.0 });
            }
        }
        assert_eq!(writes_to(&simulator, CALIBRATED_AIRSPEED), target > 200.0);
    }
}

#[test]
fn gear_down_target_above_the_extended_speed_limit_fails_before_commands() {
    let (fixture, mut runtime, simulator) =
        flight_runtime("calibrated_airspeed = 280.001\nlanding_gear = 'down'");
    runtime.pre_update().unwrap();
    simulator.borrow_mut().clear_operations();
    let error = runtime.pre_update().unwrap_err();
    assert!(matches!(
        error.downcast_ref::<InitialisationError>(),
        Some(InitialisationError::GearExtendedSpeedLimit {
            target: 280.001,
            maximum: 280.0
        })
    ));
    assert_no_replay_writes(&simulator.borrow());
    assert_no_telemetry(&fixture);
    runtime.stop().unwrap();
}

#[test]
fn gear_up_waits_for_airborne_readback_while_altitude_reposition_continues() {
    let (fixture, mut runtime, simulator) =
        flight_runtime("pressure_altitude = 10000.0\nlanding_gear = 'up'");
    observations(&simulator, CALIBRATED_AIRSPEED, [200.0; 2]);
    observations(&simulator, PRESSURE_ALTITUDE, [10000.0; 2]);
    observations(&simulator, SIM_ON_GROUND, [1.0, 0.0, 0.0]);
    runtime.pre_update().unwrap();
    simulator.borrow_mut().clear_operations();
    runtime.pre_update().unwrap();
    assert!(writes_to(&simulator, PLANE_ALTITUDE));
    assert!(!writes_to(&simulator, "K:GEAR_UP"));
    assert_no_telemetry(&fixture);

    observations(&simulator, PRESSURE_ALTITUDE, [10000.0]);
    for variable in GEAR_VARIABLES {
        observations(&simulator, variable, [packed(0, 3)]);
    }
    simulator.borrow_mut().time = duration(100.5);
    simulator.borrow_mut().clear_operations();
    runtime.pre_update().unwrap();
    let writes: Vec<_> = simulator
        .borrow()
        .operations
        .iter()
        .filter_map(|operation| match operation {
            Operation::Write { variable, .. } => Some(variable.clone()),
            _ => None,
        })
        .collect();
    assert_eq!(writes, ["K:GEAR_UP", STICK_EVENTS[0], STICK_EVENTS[1]]);
    settle_250_knots(&fixture, &mut runtime, &simulator, 101.0);
    assert_eq!(fixture.telemetry_contents().lines().count(), 1);
    runtime.stop().unwrap();
}

#[test]
fn invalid_ground_state_fails_before_any_aircraft_commands() {
    for value in [-1.0, 0.5, 2.0, f64::NAN] {
        let (fixture, mut runtime, simulator) =
            flight_runtime("pressure_altitude = 10000.0\nlanding_gear = 'up'");
        observations(&simulator, SIM_ON_GROUND, [value]);
        runtime.pre_update().unwrap();
        simulator.borrow_mut().clear_operations();
        let error = runtime.pre_update().unwrap_err();
        assert!(matches!(
            error.downcast_ref::<InitialisationError>(),
            Some(InitialisationError::InvalidReadback {
                variable: SIM_ON_GROUND,
                ..
            }) | Some(InitialisationError::Readback(
                SimulatorError::NonFiniteRead { .. }
            ))
        ));
        assert!(!writes_to(&simulator, PLANE_ALTITUDE));
        assert!(!writes_to(&simulator, "K:GEAR_UP"));
        assert_no_replay_writes(&simulator.borrow());
        assert_no_telemetry(&fixture);
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
                Duration::ZERO,
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
    simulator.borrow_mut().time = duration(160.0);
    let error = runtime.pre_update().unwrap_err();
    assert!(matches!(
        error.downcast_ref::<InitialisationError>(),
        Some(InitialisationError::Timeout { targets, .. })
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
            },
            Duration::ZERO
        ),
        Err(InitialisationError::InvalidFlapsTarget { detent: 5 })
    ));
    assert!(simulator.operations.is_empty());
}

const FLIGHT_CONDITION_SETTINGS: &str = "pressure_altitude = 10000.0\ncalibrated_airspeed = 250.0";

/// Builds a supported run whose default observations keep both targets waiting.
fn flight_runtime(settings: &str) -> (Fixture, Runtime, SimulatorHandle) {
    let (fixture, runtime, simulator) = lever_runtime(settings);
    observations(&simulator, PRESSURE_ALTITUDE, [9500.0; 16]);
    observations(&simulator, PLANE_ALTITUDE, [10000.0; 16]);
    observations(&simulator, CALIBRATED_AIRSPEED, [240.0; 16]);
    observations(&simulator, VERTICAL_SPEED, [0.0; 2048]);
    observations(&simulator, SIM_ON_GROUND, [0.0; 2048]);
    observations(
        &simulator,
        crate::ths::TRIM_POSITION,
        [4.0 * 100.0 / 17.5; 2048],
    );
    for variable in FCDC_VARIABLES {
        observations(
            &simulator,
            variable,
            [packed((1 << 10) | (1 << 15), 3); 2048],
        );
    }
    for (variable, _, _) in ATTITUDE_VARIABLES {
        observations(&simulator, variable, [0.0; 2048]);
    }
    (fixture, runtime, simulator)
}

#[test]
fn loading_finishes_before_any_reposition_commands_and_ramps_start_from_fresh_observations() {
    let (fixture, mut runtime, simulator) = flight_runtime(&format!(
        "{FLIGHT_CONDITION_SETTINGS}\nzfw = 60000.0\ngw = 65000.0\ngwcg = 25.0\nths = 1.0\nflaps = 2\nlanding_gear = 'up'\npitch = 3.0\nroll = 15.0"
    ));
    observations(&simulator, "L:A32NX_HYD_TRIM_WHEEL_PERCENT", [0.0; 4]);
    runtime.pre_update().unwrap();
    for (now, zfw, gw, gwcg) in [
        (100.0, 59000.0, 65000.0, 25.0),
        (110.0, 60000.0, 64800.0, 25.0),
        (120.0, 60000.0, 65000.0, 26.0),
        (130.0, 60000.0, 65000.0, 25.0),
    ] {
        for (variable, value) in [
            ("L:A32NX_AIRFRAME_ZFW", zfw),
            ("L:A32NX_AIRFRAME_GW", gw),
            ("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", gwcg),
        ] {
            observations(&simulator, variable, [value]);
        }
        observations(&simulator, PRESSURE_ALTITUDE, [9000.0]);
        observations(&simulator, PLANE_ALTITUDE, [9500.0]);
        observations(&simulator, CALIBRATED_AIRSPEED, [190.0; 2]);
        simulator.borrow_mut().time = duration(now);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert_eq!(
            writes_to(&simulator, "L:A32NX_BOARDING_STARTED_BY_USR"),
            now == 100.0
        );
        assert_eq!(
            writes_to(&simulator, "L:A32NX_REFUEL_STARTED_BY_USR"),
            now == 100.0
        );
        if now != 100.0 {
            assert!(
                simulator
                    .borrow()
                    .operations
                    .iter()
                    .all(|op| !matches!(op, Operation::Write { .. })),
                "loading demands and start flags must remain untouched while waiting"
            );
        }
        for event in [
            PLANE_ALTITUDE,
            CALIBRATED_AIRSPEED,
            THROTTLE_EVENTS[0],
            THROTTLE_EVENTS[1],
            "K:AXIS_ELEV_TRIM_SET",
            "K:FLAPS_2",
            "K:GEAR_UP",
        ]
        .into_iter()
        .chain(ATTITUDE_VARIABLES.iter().map(|(name, _, _)| *name))
        {
            assert!(
                !writes_to(&simulator, event),
                "{event} written while loading at {now}"
            );
        }
        assert_eq!(writes_to(&simulator, AIRCRAFT_READY), now == 100.0);
        assert_no_telemetry(&fixture);
    }
    for now in [135.0, 135.5] {
        for (variable, value) in [
            ("L:A32NX_AIRFRAME_ZFW", 60000.0),
            ("L:A32NX_AIRFRAME_GW", 65000.0),
            ("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", 25.0),
        ] {
            observations(&simulator, variable, [value]);
        }
        observations(&simulator, PRESSURE_ALTITUDE, [9200.0]);
        observations(&simulator, PLANE_ALTITUDE, [9700.0]);
        observations(&simulator, CALIBRATED_AIRSPEED, [210.0; 2]);
        simulator.borrow_mut().time = duration(now);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert!(!writes_to(&simulator, "L:A32NX_BOARDING_STARTED_BY_USR"));
        assert!(!writes_to(&simulator, "L:A32NX_REFUEL_STARTED_BY_USR"));
        assert!(!writes_to(&simulator, "L:A32NX_FUEL_TOTAL_DESIRED"));
        assert_eq!(
            last_write(&simulator.borrow(), PLANE_ALTITUDE),
            if now == 135.0 { 9700.0 } else { 9950.0 }
        );
        assert_eq!(
            last_write(&simulator.borrow(), "A:PLANE PITCH DEGREES"),
            -3.0
        );
        assert_eq!(
            last_write(&simulator.borrow(), "A:PLANE BANK DEGREES"),
            -15.0
        );
        for event in ["K:AXIS_ELEV_TRIM_SET", "K:FLAPS_2", "K:GEAR_UP"] {
            assert!(writes_to(&simulator, event));
        }
        if now == 135.0 {
            assert_eq!(last_write(&simulator.borrow(), CALIBRATED_AIRSPEED), 210.0);
            assert_eq!(
                last_write(&simulator.borrow(), THROTTLE_EVENTS[0]),
                IDLE_AXIS
            );
        }
        assert_no_telemetry(&fixture);
    }
    runtime.stop().unwrap();
    assert_eq!(last_write(&simulator.borrow(), AIRCRAFT_READY), 1.0);
}

#[test]
fn loading_without_reposition_enters_playback_when_the_actual_masses_are_ready() {
    let (fixture, mut runtime, simulator) =
        flight_runtime("zfw = 60000.0\ngw = 65000.0\ngwcg = 25.0");
    for variable in [
        PRESSURE_ALTITUDE,
        PLANE_ALTITUDE,
        CALIBRATED_AIRSPEED,
        VERTICAL_SPEED,
    ]
    .into_iter()
    .chain(ATTITUDE_VARIABLES.iter().map(|(name, _, _)| *name))
    {
        simulator.borrow_mut().reads.remove(variable);
    }
    observations(&simulator, "L:A32NX_AIRFRAME_ZFW", [59700.0, 60000.0]);
    observations(&simulator, "L:A32NX_AIRFRAME_GW", [65000.0; 2]);
    observations(&simulator, "L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", [25.0; 2]);
    runtime.pre_update().unwrap();
    simulator.borrow_mut().clear_operations();
    runtime.pre_update().unwrap();
    assert_no_telemetry(&fixture);
    assert_eq!(last_write(&simulator.borrow(), AIRCRAFT_READY), 0.0);
    simulator.borrow_mut().time = duration(100.5);
    simulator.borrow_mut().clear_operations();
    runtime.pre_update().unwrap();
    assert_eq!(fixture.telemetry_contents().lines().count(), 1);
    assert_eq!(last_write(&simulator.borrow(), AIRCRAFT_READY), 1.0);
    assert_eq!(last_write(&simulator.borrow(), DEVELOPER_STATE), 0.0);
    assert_eq!(
        simulator
            .borrow()
            .operations
            .iter()
            .filter(|op| matches!(op, Operation::Write { .. }))
            .count(),
        2
    );
    assert!(simulator.borrow().operations.iter().all(|op| match op {
        Operation::Read { variable, .. }
        | Operation::ValidateRead { variable, .. }
        | Operation::Write { variable, .. } => variable.starts_with("L:"),
        _ => false,
    }));
    simulator.borrow_mut().clear_operations();
    runtime.pre_update().unwrap();
    assert!(writes_to(&simulator, "K:AXIS_ELEVATOR_SET"));
    runtime.stop().unwrap();
    assert_eq!(fixture.telemetry_contents().lines().count(), 2);
}

#[test]
fn loading_timeout_restores_ready_and_reports_the_unstarted_reposition_targets() {
    let (fixture, mut runtime, simulator) = flight_runtime(
        "zfw = 60000.0\ngw = 65000.0\ngwcg = 25.0\npressure_altitude = 30000.0\ncalibrated_airspeed = 250.0",
    );
    for (variable, value) in [
        ("L:A32NX_AIRFRAME_ZFW", 59000.0),
        ("L:A32NX_AIRFRAME_GW", 65000.0),
        ("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", 25.0),
        (PRESSURE_ALTITUDE, 0.0),
        (PLANE_ALTITUDE, 0.0),
        (CALIBRATED_AIRSPEED, 0.0),
    ] {
        observations(&simulator, variable, [value; 2]);
    }
    runtime.pre_update().unwrap();
    for now in [100.0, 170.0] {
        simulator.borrow_mut().time = duration(now);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert_eq!(
            writes_to(&simulator, "L:A32NX_BOARDING_STARTED_BY_USR"),
            now == 100.0
        );
        assert!(!writes_to(&simulator, PLANE_ALTITUDE));
        assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
        assert_no_telemetry(&fixture);
    }
    simulator.borrow_mut().time = duration(220.0);
    simulator.borrow_mut().clear_operations();
    let error = runtime.pre_update().unwrap_err();
    assert!(matches!(error.downcast_ref::<InitialisationError>(),
        Some(InitialisationError::Timeout { timeout, pending, .. })
        if *timeout == duration(120.0) && pending == &["zfw", "pressure_altitude", "calibrated_airspeed"]));
    assert_eq!(
        write_operations(&simulator.borrow()),
        [
            Operation::Write {
                variable: AIRCRAFT_READY.to_owned(),
                value: 1.0
            },
            Operation::Write {
                variable: DEVELOPER_STATE.to_owned(),
                value: 0.0
            },
            Operation::Write {
                variable: ARMED_VARIABLE.to_owned(),
                value: 0.0
            },
        ]
    );
    simulator.borrow_mut().clear_operations();
    runtime.stop().unwrap();
    assert!(!writes_to(&simulator, AIRCRAFT_READY));
    assert_no_telemetry(&fixture);
}

#[test]
fn fuel_burn_is_inhibited_once_through_stabilisation_and_restored_before_playback() {
    for previous in [0.0, 1.0, 2.0] {
        let (fixture, mut runtime, simulator) =
            flight_runtime("zfw = 60000.0\ngw = 65000.0\ngwcg = 25.0\ncalibrated_airspeed = 250.0");
        observations(&simulator, DEVELOPER_STATE, [previous]);
        runtime.pre_update().unwrap();
        let mut state = previous;
        let mut state_writes = Vec::new();
        let mut stabilising = false;
        let mut entered_playback = false;
        for tick in 0..=30 {
            observations(
                &simulator,
                "L:A32NX_AIRFRAME_ZFW",
                [if tick == 0 { 59700.0 } else { 60000.0 }],
            );
            observations(&simulator, "L:A32NX_AIRFRAME_GW", [65000.0]);
            observations(&simulator, "L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", [25.0]);
            observations(
                &simulator,
                CALIBRATED_AIRSPEED,
                [(240.0 + f64::from(tick) * 2.5).min(250.0)],
            );
            simulator.borrow_mut().time = duration(100.0 + f64::from(tick) * 0.5);
            simulator.borrow_mut().clear_operations();
            runtime.pre_update().unwrap();
            let ops = &simulator.borrow().operations;
            if tick == 0 {
                let freeze = ops
                    .iter()
                    .position(|op| {
                        matches!(op,
                    Operation::Write { variable, value: 2.0 } if variable == DEVELOPER_STATE)
                    })
                    .unwrap();
                let loading = ops
                    .iter()
                    .position(|op| {
                        matches!(op,
                    Operation::Write { variable, .. } if variable == "L:A32NX_PAX_A_DESIRED")
                    })
                    .unwrap();
                assert!(freeze < loading);
            }
            for op in ops {
                if let Operation::Write { variable, value } = op {
                    if variable == DEVELOPER_STATE {
                        state = *value;
                        state_writes.push(*value);
                    }
                    if variable == AIRCRAFT_READY && *value == 1.0 {
                        stabilising = true;
                    }
                }
            }
            if fixture.directory.read_dir().unwrap().any(|entry| {
                entry
                    .unwrap()
                    .file_name()
                    .to_string_lossy()
                    .starts_with("telemetry_")
            }) {
                entered_playback = true;
                break;
            }
            assert_eq!(state, 2.0);
            assert_eq!(state_writes, [2.0]);
        }
        assert!(stabilising && entered_playback);
        assert_eq!(state, previous);
        assert_eq!(state_writes, [2.0, previous]);
        assert_eq!(fixture.telemetry_contents().lines().count(), 1);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        runtime.stop().unwrap();
        assert!(!writes_to(&simulator, DEVELOPER_STATE));
    }
}

#[test]
fn setup_failure_timeout_and_shutdown_restore_fuel_burn() {
    for exit in ["setup", "timeout", "shutdown"] {
        let (fixture, mut runtime, simulator) =
            flight_runtime("zfw = 60000.0\ngw = 65000.0\ngwcg = 25.0");
        observations(&simulator, DEVELOPER_STATE, [1.0]);
        observations(&simulator, "L:A32NX_AIRFRAME_ZFW", [59000.0]);
        observations(&simulator, "L:A32NX_AIRFRAME_GW", [65000.0]);
        observations(&simulator, "L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", [25.0]);
        runtime.pre_update().unwrap();
        if exit == "setup" {
            simulator.borrow_mut().failure =
                Some(Failure::Write("L:A32NX_PAX_A_DESIRED".to_owned()));
            assert!(runtime.pre_update().is_err());
        } else {
            runtime.pre_update().unwrap();
            assert_eq!(last_write(&simulator.borrow(), DEVELOPER_STATE), 2.0);
            if exit == "timeout" {
                simulator.borrow_mut().time = duration(160.0);
                assert!(matches!(
                    runtime
                        .pre_update()
                        .unwrap_err()
                        .downcast_ref::<InitialisationError>(),
                    Some(InitialisationError::Timeout { .. })
                ));
            } else {
                runtime.stop().unwrap();
            }
        }
        assert_eq!(last_write(&simulator.borrow(), DEVELOPER_STATE), 1.0);
        assert_eq!(last_write(&simulator.borrow(), AIRCRAFT_READY), 1.0);
        assert_no_telemetry(&fixture);
        simulator.borrow_mut().clear_operations();
        runtime.stop().unwrap();
        assert!(!writes_to(&simulator, DEVELOPER_STATE));
        assert!(!writes_to(&simulator, AIRCRAFT_READY));
    }
}

#[test]
fn failed_fuel_or_ready_release_attempts_both_and_retries_only_the_failed_write() {
    for failed in [DEVELOPER_STATE, AIRCRAFT_READY] {
        let (_fixture, mut runtime, simulator) =
            flight_runtime("zfw = 60000.0\ngw = 65000.0\ngwcg = 25.0");
        observations(&simulator, "L:A32NX_AIRFRAME_ZFW", [59000.0]);
        observations(&simulator, "L:A32NX_AIRFRAME_GW", [65000.0]);
        observations(&simulator, "L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", [25.0]);
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        simulator.borrow_mut().failure = Some(Failure::Write(failed.to_owned()));
        simulator.borrow_mut().clear_operations();
        assert!(
            matches!(runtime.stop().unwrap_err(), CleanupError::Aircraft {
            source: SimulatorError::CalculatorCodeWriteFailed { variable, .. }, ..
        } if variable == failed)
        );
        assert!(writes_to(&simulator, AIRCRAFT_READY));
        assert!(writes_to(&simulator, DEVELOPER_STATE));
        simulator.borrow_mut().clear_operations();
        assert!(runtime.pre_update().is_err());
        assert!(simulator.borrow().operations.iter().all(|op|
            matches!(op, Operation::Write { variable, .. } if variable == failed || variable == ARMED_VARIABLE)));
        assert!(writes_to(&simulator, failed));
        simulator.borrow_mut().failure = None;
        observations(&simulator, ARMED_VARIABLE, [0.0]);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert!(writes_to(&simulator, failed));
        assert!(!writes_to(
            &simulator,
            if failed == DEVELOPER_STATE {
                AIRCRAFT_READY
            } else {
                DEVELOPER_STATE
            }
        ));
        simulator.borrow_mut().clear_operations();
        runtime.stop().unwrap();
        assert!(!writes_to(&simulator, DEVELOPER_STATE));
        assert!(!writes_to(&simulator, AIRCRAFT_READY));
    }
}

#[test]
fn unavailable_fuel_inhibit_state_fails_before_setup_writes() {
    for unavailable in ["missing", "read", "nonfinite"] {
        let (_fixture, mut runtime, simulator) =
            flight_runtime("zfw = 60000.0\ngw = 65000.0\ngwcg = 25.0");
        match unavailable {
            "missing" => {
                simulator.borrow_mut().reads.remove(DEVELOPER_STATE);
            }
            "read" => {
                simulator.borrow_mut().failure = Some(Failure::Read(DEVELOPER_STATE.to_owned()));
            }
            _ => observations(&simulator, DEVELOPER_STATE, [f64::NAN]),
        }
        runtime.pre_update().unwrap();
        simulator.borrow_mut().clear_operations();
        let error = runtime.pre_update().unwrap_err();
        assert!(matches!(
            error.downcast_ref::<InitialisationError>(),
            Some(InitialisationError::MissingInterface {
                variable: DEVELOPER_STATE
            }) | Some(InitialisationError::Readback(
                SimulatorError::CalculatorCodeReadFailed { .. }
                    | SimulatorError::NonFiniteRead { .. }
            ))
        ));
        assert!(simulator.borrow().operations.iter().all(
            |op| !matches!(op, Operation::Write { variable, .. } if variable != ARMED_VARIABLE)
        ));
    }
}

/// Supplies stable observations until the sustained speed gate enters playback.
fn settle_250_knots(
    fixture: &Fixture,
    runtime: &mut Runtime,
    simulator: &SimulatorHandle,
    first_time: f64,
) {
    let targets = testpilot_core::config::Config::read_config_file(&fixture.config_path)
        .unwrap()
        .initialisation
        .unwrap();
    for step in 0..32 {
        observations(
            simulator,
            PRESSURE_ALTITUDE,
            [targets.pressure_altitude.unwrap_or(10000.0)],
        );
        observations(
            simulator,
            PLANE_ALTITUDE,
            [targets.pressure_altitude.unwrap_or(10000.0) + 500.0],
        );
        observations(
            simulator,
            CALIBRATED_AIRSPEED,
            [targets.calibrated_airspeed.unwrap_or(250.0)],
        );
        observations(
            simulator,
            VERTICAL_SPEED,
            [targets.vertical_speed.unwrap_or(0.0)],
        );
        observations(simulator, SIM_ON_GROUND, [0.0; 2]);
        observations(simulator, "A:PLANE PITCH DEGREES", [-3.0]);
        observations(
            simulator,
            "A:PLANE BANK DEGREES",
            [-targets.roll.unwrap_or(0.0)],
        );
        for (variable, _, _) in &ATTITUDE_VARIABLES[2..] {
            observations(simulator, variable, [0.0]);
        }
        for variable in MOVEMENT_VARIABLES {
            observations(simulator, variable, [0.0]);
        }
        if let Some(gear) = targets.landing_gear {
            for variable in GEAR_VARIABLES {
                observations(
                    simulator,
                    variable,
                    [packed(if gear == LandingGear::Down { DOWN } else { 0 }, 3)],
                );
            }
        }
        for (variable, value) in [
            ("L:A32NX_AIRFRAME_ZFW", targets.zfw),
            ("L:A32NX_AIRFRAME_GW", targets.gw),
            ("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", targets.gwcg),
        ] {
            if let Some(value) = value {
                observations(simulator, variable, [value]);
            }
        }
        simulator.borrow_mut().time = duration(first_time + f64::from(step) * 0.5);
        runtime.pre_update().unwrap();
        if fixture.directory.read_dir().unwrap().any(|entry| {
            entry
                .unwrap()
                .file_name()
                .to_string_lossy()
                .starts_with("telemetry_")
        }) {
            return;
        }
    }
    panic!("flight conditions did not settle");
}

/// Most recent command to one interface, excluding unrelated groups and cleanup.
fn last_write(simulator: &FakeSimulator, expected: &str) -> f64 {
    simulator
        .operations
        .iter()
        .rev()
        .find_map(|op| match op {
            Operation::Write { variable, value } if variable == expected => Some(*value),
            _ => None,
        })
        .unwrap_or_else(|| panic!("missing write to {expected}"))
}

#[test]
fn level_climb_and_descent_settle_naturally_and_start_playback_at_requested_altitude() {
    for vertical_speed in [0.0, 1000.0, -1000.0] {
        let (fixture, mut runtime, simulator) = flight_runtime(&format!(
            "pressure_altitude = 10000.0\ncalibrated_airspeed = 250.0\nvertical_speed = {vertical_speed}\npitch = 1.9\nths = 1.0"
        ));
        let seed = 10000.0 - vertical_speed;
        observations(&simulator, PRESSURE_ALTITUDE, [seed]);
        observations(&simulator, PLANE_ALTITUDE, [seed + 500.0]);
        observations(&simulator, CALIBRATED_AIRSPEED, [250.0]);
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        assert_eq!(last_write(&simulator.borrow(), AIRCRAFT_READY), 1.0);
        let mut playback = false;
        for tick in 1..=130 {
            let elapsed = f64::from(tick) * 0.5;
            let height = seed + vertical_speed * elapsed / 60.0;
            observations(&simulator, PRESSURE_ALTITUDE, [height]);
            observations(&simulator, PLANE_ALTITUDE, [height + 500.0]);
            observations(&simulator, CALIBRATED_AIRSPEED, [250.0]);
            observations(&simulator, VERTICAL_SPEED, [vertical_speed]);
            // Natural pitch and trim differ from their repositioning seeds.
            observations(&simulator, "A:PLANE PITCH DEGREES", [-3.0]);
            observations(&simulator, crate::ths::TRIM_POSITION, [44.0]);
            simulator.borrow_mut().time = duration(100.0 + elapsed);
            simulator.borrow_mut().clear_operations();
            runtime.pre_update().unwrap();
            for variable in ATTITUDE_VARIABLES
                .map(|(name, _, _)| name)
                .into_iter()
                .chain([PLANE_ALTITUDE, CALIBRATED_AIRSPEED, "K:AXIS_ELEV_TRIM_SET"])
            {
                assert!(
                    !writes_to(&simulator, variable),
                    "{variable} forced during natural flight"
                );
            }
            if fixture.directory.read_dir().unwrap().any(|entry| {
                entry
                    .unwrap()
                    .file_name()
                    .to_string_lossy()
                    .starts_with("telemetry_")
            }) {
                assert!((height - 10000.0).abs() <= 10.0);
                assert!(elapsed >= 3.0);
                if vertical_speed != 0.0 {
                    assert!((119..=120).contains(&tick));
                }
                playback = true;
                break;
            }
        }
        assert!(playback);
        assert_eq!(fixture.telemetry_contents().lines().count(), 1);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        runtime.stop().unwrap();
        assert!(!writes_to(&simulator, STICK_EVENTS[1]));
        assert!(!writes_to(&simulator, THROTTLE_EVENTS[0]));
        assert_eq!(fixture.telemetry_contents().lines().count(), 2);
    }
}

#[test]
fn altitude_feedback_corrects_inside_the_acceptance_band_without_widening_it() {
    for error in [-10.26_f64, -8.0, -4.0, 4.0, 8.0, 10.26] {
        let (_, _, simulator) = flight_runtime("pressure_altitude = 31089.0");
        let targets = InitialisationConfig {
            pressure_altitude: Some(31089.0),
            ..InitialisationConfig::default()
        };
        let mut aircraft = A32nxInitialiser::default();
        observations(&simulator, PRESSURE_ALTITUDE, [31089.0]);
        observations(&simulator, PLANE_ALTITUDE, [31589.0]);
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, duration(100.0))
            .unwrap();
        for tick in 1..=8 {
            observations(&simulator, PRESSURE_ALTITUDE, [31089.0 - error]);
            observations(&simulator, PLANE_ALTITUDE, [31589.0 - error]);
            simulator.borrow_mut().clear_operations();
            aircraft
                .submit(
                    &mut *simulator.borrow_mut(),
                    targets,
                    duration(100.0 + f64::from(tick) * 0.5),
                )
                .unwrap();
            if tick >= 2 {
                let pitch_axis = last_write(&simulator.borrow(), STICK_EVENTS[0]);
                if error.abs() > 5.0 {
                    assert_eq!(pitch_axis.signum(), -error.signum());
                    assert!(pitch_axis.abs() <= 82.0);
                } else {
                    assert_eq!(pitch_axis, 0.0);
                }
            }
        }
        let pending = aircraft
            .pending_targets(&mut *simulator.borrow_mut(), targets)
            .unwrap();
        assert_eq!(pending.pressure_altitude.is_some(), error.abs() > 10.0);
        assert_eq!(pending.vertical_speed.is_some(), error.abs() > 10.0);
    }
}

#[test]
fn speed_and_flight_path_keep_independent_dwell_intervals_and_both_gate_playback() {
    for (altitude_error, vertical_speed, bank) in
        [(10.26, 0.0, 0.0), (0.0, 150.0, 0.0), (0.0, 0.0, 1.0)]
    {
        let (_, _, simulator) =
            flight_runtime("pressure_altitude = 31089.0\ncalibrated_airspeed = 287.0");
        let targets = InitialisationConfig {
            pressure_altitude: Some(31089.0),
            calibrated_airspeed: Some(287.0),
            ..InitialisationConfig::default()
        };
        let mut aircraft = A32nxInitialiser::default();
        for tick in 0..=16 {
            let disturbed = (1..10).contains(&tick);
            let error = if disturbed { altitude_error } else { 0.0 };
            observations(&simulator, PRESSURE_ALTITUDE, [31089.0 - error]);
            observations(&simulator, PLANE_ALTITUDE, [31589.0 - error]);
            observations(
                &simulator,
                CALIBRATED_AIRSPEED,
                [if tick == 0 { 287.0 } else { 287.84 }],
            );
            observations(
                &simulator,
                VERTICAL_SPEED,
                [if disturbed { vertical_speed } else { 0.0 }],
            );
            observations(
                &simulator,
                "A:PLANE BANK DEGREES",
                [if disturbed { -bank } else { 0.0 }],
            );
            aircraft
                .submit(
                    &mut *simulator.borrow_mut(),
                    targets,
                    duration(100.0 + f64::from(tick) * 0.5),
                )
                .unwrap();
            let pending = aircraft
                .pending_targets(&mut *simulator.borrow_mut(), targets)
                .unwrap();
            if tick >= 9 {
                assert_eq!(pending.calibrated_airspeed, None);
            }
            assert_eq!(
                pending.vertical_speed.is_none(),
                tick == 16,
                "flight path became ready before its own three-second dwell: tick={tick}"
            );
        }
        assert!(
            aircraft
                .is_ready(&mut *simulator.borrow_mut(), targets)
                .unwrap()
        );
        observations(&simulator, PRESSURE_ALTITUDE, [31089.0]);
        observations(&simulator, PLANE_ALTITUDE, [31589.0]);
        observations(&simulator, CALIBRATED_AIRSPEED, [290.0]);
        observations(&simulator, "A:PLANE BANK DEGREES", [0.0]);
        observations(&simulator, VERTICAL_SPEED, [0.0]);
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, duration(108.1))
            .unwrap();
        assert!(aircraft.flight_path.unwrap().is_ready());
        assert!(
            !aircraft
                .is_ready(&mut *simulator.borrow_mut(), targets)
                .unwrap()
        );
    }
}

#[test]
fn sidestick_feedback_converges_with_a_simple_host_plant_and_no_pilot_input() {
    // This exercises feedback, signs and the handoff; it does not model A32NX dynamics.
    for target in [0.0, 1000.0, -1000.0] {
        let (fixture, mut runtime, simulator) =
            flight_runtime(&format!("vertical_speed = {target}"));
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        let mut speed = target + 1200.0;
        let mut stick = 0.0;
        let mut playback = false;
        for tick in 1..300 {
            let elapsed = f64::from(tick) * 0.2;
            speed += stick * 1000.0 * 0.2;
            observations(&simulator, VERTICAL_SPEED, [speed]);
            observations(&simulator, "A:ROTATION VELOCITY BODY X", [-stick * 5.0]);
            let trim = 2.0 + 1.0 - (-elapsed / 4.0).exp();
            observations(
                &simulator,
                crate::ths::TRIM_POSITION,
                [(trim + 4.0) * 100.0 / 17.5],
            );
            simulator.borrow_mut().time = duration(100.0 + elapsed);
            simulator.borrow_mut().clear_operations();
            runtime.pre_update().unwrap();
            stick = -last_write(&simulator.borrow(), STICK_EVENTS[0]) / 16384.0;
            assert!(stick.abs() <= 0.2001);
            assert!(
                write_operations(&simulator.borrow())
                    .iter()
                    .all(|op| matches!(op,
                Operation::Write { variable, .. } if STICK_EVENTS.contains(&variable.as_str())))
            );
            if fixture.directory.read_dir().unwrap().any(|entry| {
                entry
                    .unwrap()
                    .file_name()
                    .to_string_lossy()
                    .starts_with("telemetry_")
            }) {
                assert!((speed - target).abs() <= crate::flight_path::VS_TOLERANCE);
                assert_eq!(stick, 0.0);
                playback = true;
                break;
            }
        }
        assert!(
            playback,
            "vertical speed {target} did not settle automatically"
        );
        runtime.stop().unwrap();
    }
}

#[test]
fn sidestick_signs_slew_and_law_loss_use_irregular_simulator_time() {
    for target in [-1000.0, 1000.0] {
        let (_, _, simulator) = flight_runtime("");
        let targets = InitialisationConfig {
            vertical_speed: Some(target),
            roll: Some(10.0),
            ..InitialisationConfig::default()
        };
        let mut aircraft = A32nxInitialiser::default();
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, duration(100.0))
            .unwrap();
        let mut last = [0.0; 2];
        let mut previous_time = 100.0;
        for now in [100.0, 100.0, 100.1, 100.35, 100.8, 101.3] {
            simulator.borrow_mut().clear_operations();
            aircraft
                .submit(&mut *simulator.borrow_mut(), targets, duration(now))
                .unwrap();
            for (i, event) in STICK_EVENTS.into_iter().enumerate() {
                let axis = last_write(&simulator.borrow(), event);
                assert!(axis.abs() <= 3277.0);
                assert!((axis - last[i]).abs() <= 0.2 * 16384.0 * (now - previous_time) + 1.0);
                last[i] = axis;
            }
            previous_time = now;
        }
        assert_eq!(last[0].signum(), -target.signum());
        assert!(last[1] < 0.0);
        for variable in FCDC_VARIABLES {
            observations(&simulator, variable, [packed(0, 0)]);
        }
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, duration(101.4))
            .unwrap();
        for event in STICK_EVENTS {
            assert_eq!(last_write(&simulator.borrow(), event), 0.0);
        }
        assert!(
            !aircraft
                .is_ready(&mut *simulator.borrow_mut(), targets)
                .unwrap()
        );
    }
}

#[test]
fn sidestick_feedback_requires_airborne_normal_law_from_one_valid_fcdc() {
    let pitch = 1 << 10;
    let roll = 1 << 15;
    for (words, ground, enabled) in [
        ([packed(pitch | roll, 3), packed(0, 0)], 0.0, true),
        ([packed(0, 0), packed(pitch | roll, 3)], 0.0, true),
        ([packed(pitch | roll, 0), packed(0, 3)], 0.0, false),
        ([packed(pitch, 3), packed(roll, 3)], 0.0, false),
        ([packed(pitch | roll, 3); 2], 1.0, false),
    ] {
        let (_, _, simulator) = flight_runtime("");
        for (variable, word) in FCDC_VARIABLES.into_iter().zip(words) {
            observations(&simulator, variable, [word; 3]);
        }
        observations(&simulator, SIM_ON_GROUND, [ground; 3]);
        let targets = InitialisationConfig {
            vertical_speed: Some(1000.0),
            ..InitialisationConfig::default()
        };
        let mut aircraft = A32nxInitialiser::default();
        for now in [100.0, 100.0, 100.5] {
            aircraft
                .submit(&mut *simulator.borrow_mut(), targets, duration(now))
                .unwrap();
        }
        let axis = last_write(&simulator.borrow(), STICK_EVENTS[0]);
        assert_eq!(axis < 0.0, enabled);
        assert!(
            !aircraft
                .is_ready(&mut *simulator.borrow_mut(), targets)
                .unwrap()
        );
    }
}

#[test]
fn trim_settling_requires_elapsed_time_and_duplicate_samples_preserve_or_reset_the_window() {
    let (_, _, simulator) = flight_runtime("");
    let targets = InitialisationConfig {
        vertical_speed: Some(0.0),
        ..InitialisationConfig::default()
    };
    let mut aircraft = A32nxInitialiser::default();
    for now in [100.0, 100.0, 100.5, 100.5, 101.5] {
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, duration(now))
            .unwrap();
    }
    assert!(!aircraft.flight_path.unwrap().is_ready());
    observations(&simulator, crate::ths::TRIM_POSITION, [44.0; 16]);
    for now in [101.5, 101.5, 102.0, 104.999, 104.999] {
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, duration(now))
            .unwrap();
        assert!(
            !aircraft
                .is_ready(&mut *simulator.borrow_mut(), targets)
                .unwrap()
        );
    }
    aircraft
        .submit(&mut *simulator.borrow_mut(), targets, duration(105.0))
        .unwrap();
    assert!(
        aircraft
            .is_ready(&mut *simulator.borrow_mut(), targets)
            .unwrap()
    );
}

#[test]
fn timeout_and_shutdown_neutralise_active_sidesticks_and_leave_no_setup_commands() {
    for timeout in [false, true] {
        let (fixture, mut runtime, simulator) =
            flight_runtime("vertical_speed = 1000.0\npitch = 1.9\nths = 1.0");
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        for now in [100.5, 101.0] {
            simulator.borrow_mut().time = duration(now);
            runtime.pre_update().unwrap();
        }
        assert!(last_write(&simulator.borrow(), STICK_EVENTS[0]) < 0.0);
        simulator.borrow_mut().clear_operations();
        if timeout {
            simulator.borrow_mut().time = duration(160.0);
            let error = runtime.pre_update().unwrap_err();
            assert!(matches!(error.downcast_ref::<InitialisationError>(),
                Some(InitialisationError::Timeout { pending, details, .. })
                if pending == &["vertical_speed"]
                    && details.contains("difference(target-current)=+1000.0000 ft/min")
                    && !details.contains("\n  pitch")
                    && !details.contains("\n  ths")));
        } else {
            runtime.stop().unwrap();
        }
        for event in STICK_EVENTS {
            assert_eq!(last_write(&simulator.borrow(), event), 0.0);
        }
        assert_eq!(last_write(&simulator.borrow(), ARMED_VARIABLE), 0.0);
        assert!(write_operations(&simulator.borrow()).iter().all(|op| matches!(op,
            Operation::Write { variable, value }
                if *value == 0.0 && (STICK_EVENTS.contains(&variable.as_str()) || variable == ARMED_VARIABLE))));
        assert_no_telemetry(&fixture);
        simulator.borrow_mut().clear_operations();
        runtime.stop().unwrap();
        for event in STICK_EVENTS {
            assert!(!writes_to(&simulator, event));
        }
    }
}

#[test]
fn partial_sidestick_failure_releases_both_channels_and_retries_failed_release_in_idle() {
    for failed in STICK_EVENTS {
        let (_fixture, mut runtime, simulator) = flight_runtime("vertical_speed = 1000.0");
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        simulator.borrow_mut().time = duration(100.5);
        simulator.borrow_mut().failure = Some(Failure::Write(failed.to_owned()));
        simulator.borrow_mut().clear_operations();
        let error = runtime.pre_update().unwrap_err();
        assert!(matches!(
            error.downcast_ref::<TerminationError>(),
            Some(TerminationError::CleanupAfterFailure { .. })
        ));
        for event in STICK_EVENTS {
            assert_eq!(last_write(&simulator.borrow(), event), 0.0);
        }
        assert_eq!(last_write(&simulator.borrow(), ARMED_VARIABLE), 0.0);
        simulator.borrow_mut().failure = None;
        observations(&simulator, ARMED_VARIABLE, [0.0]);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert_eq!(last_write(&simulator.borrow(), failed), 0.0);
        assert_eq!(write_operations(&simulator.borrow()).len(), 1);
        simulator.borrow_mut().clear_operations();
        runtime.stop().unwrap();
        assert!(
            write_operations(&simulator.borrow())
                .iter()
                .all(|op| matches!(op,
            Operation::Write { variable, .. } if variable == ARMED_VARIABLE))
        );
    }
}

#[test]
fn missing_the_playback_altitude_while_unsettled_fails_and_releases_sidesticks() {
    let (fixture, mut runtime, simulator) =
        flight_runtime("pressure_altitude = 10000.0\nvertical_speed = 1000.0");
    observations(&simulator, PRESSURE_ALTITUDE, [9000.0]);
    runtime.pre_update().unwrap();
    runtime.pre_update().unwrap();
    observations(&simulator, PRESSURE_ALTITUDE, [10050.0]);
    observations(&simulator, VERTICAL_SPEED, [500.0]);
    simulator.borrow_mut().time = duration(101.0);
    let error = runtime.pre_update().unwrap_err();
    assert!(matches!(
        error.downcast_ref::<InitialisationError>(),
        Some(InitialisationError::MissedPlaybackAltitude { .. })
    ));
    for event in STICK_EVENTS {
        assert_eq!(last_write(&simulator.borrow(), event), 0.0);
    }
    assert_no_telemetry(&fixture);
}

#[test]
fn altitude_setup_accesses_only_its_group_and_stops_writing_during_playback() {
    let (fixture, mut runtime, simulator) = flight_runtime("pressure_altitude = 10000.0");
    runtime.pre_update().unwrap();
    simulator.borrow_mut().clear_operations();
    runtime.pre_update().unwrap();
    assert_no_telemetry(&fixture);
    assert!(writes_to(&simulator, PLANE_ALTITUDE));
    assert!(
        simulator
            .borrow()
            .write_units
            .iter()
            .any(|(variable, unit)| variable == PLANE_ALTITUDE && unit.as_deref() == Some("feet"))
    );
    for event in THROTTLE_EVENTS {
        assert!(!writes_to(&simulator, event));
    }
    settle_250_knots(&fixture, &mut runtime, &simulator, 101.0);
    simulator.borrow_mut().clear_operations();
    runtime.pre_update().unwrap();
    runtime.stop().unwrap();
    assert!(simulator.borrow().operations.iter().all(|op| matches!(op,
        Operation::Write { variable, .. } if variable == STICK_EVENTS[0] || variable == ARMED_VARIABLE)));
    assert_eq!(fixture.telemetry_contents().lines().count(), 2);
}

#[test]
fn altitude_setup_seeds_attitude_then_releases_it_to_the_flight_controls() {
    let (_, _, simulator) = flight_runtime("pressure_altitude = 10000.0");
    let targets = InitialisationConfig {
        pressure_altitude: Some(10000.0),
        ..InitialisationConfig::default()
    };
    observations(&simulator, PRESSURE_ALTITUDE, [10000.0; 3]);
    observations(&simulator, "A:PLANE PITCH DEGREES", [-12.0; 3]);
    observations(&simulator, "A:PLANE BANK DEGREES", [35.0; 3]);
    observations(&simulator, "A:ROTATION VELOCITY BODY X", [-2.0; 3]);
    observations(&simulator, "A:ROTATION VELOCITY BODY Z", [3.0; 3]);
    let mut aircraft = A32nxInitialiser::default();
    aircraft
        .submit(&mut *simulator.borrow_mut(), targets, duration(100.0))
        .unwrap();
    for (variable, _, _) in ATTITUDE_VARIABLES {
        assert_eq!(last_write(&simulator.borrow(), variable), 0.0);
    }
    for now in [100.25, 100.75] {
        simulator.borrow_mut().clear_operations();
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, duration(now))
            .unwrap();
        for variable in ATTITUDE_VARIABLES
            .map(|(variable, _, _)| variable)
            .into_iter()
            .chain([PLANE_ALTITUDE])
        {
            assert!(!writes_to(&simulator, variable));
        }
        for event in STICK_EVENTS {
            assert!(writes_to(&simulator, event));
        }
    }
    assert!(last_write(&simulator.borrow(), STICK_EVENTS[1]) < 0.0);
}

#[test]
fn altitude_readiness_requires_both_level_angles_and_small_rotation_with_inclusive_bounds() {
    for (variable, tolerance) in [
        ("A:PLANE PITCH DEGREES", 0.5),
        ("A:PLANE BANK DEGREES", 0.5),
        ("A:ROTATION VELOCITY BODY X", 0.1),
        ("A:ROTATION VELOCITY BODY Z", 0.1),
    ] {
        for (actual, ready) in [
            (-tolerance, true),
            (tolerance, true),
            (-tolerance - 0.0001, false),
            (tolerance + 0.0001, false),
        ] {
            let (_, _, simulator) = flight_runtime("pressure_altitude = 10000.0");
            observations(&simulator, PRESSURE_ALTITUDE, [10000.0]);
            observations(&simulator, variable, [actual]);
            let targets = InitialisationConfig {
                pressure_altitude: Some(10000.0),
                ..InitialisationConfig::default()
            };
            let mut aircraft = A32nxInitialiser::default();
            aircraft
                .submit(&mut *simulator.borrow_mut(), targets, Duration::ZERO)
                .unwrap();
            simulator.borrow_mut().clear_operations();
            let pending = aircraft
                .pending_targets(&mut *simulator.borrow_mut(), targets)
                .unwrap();
            assert_eq!(
                pending.pressure_altitude.is_none(),
                ready,
                "{variable} readback {actual}"
            );
            assert!(
                simulator.borrow().operations.is_empty(),
                "readiness must use pre-write observations"
            );
        }
    }
}

#[test]
fn airspeed_only_setup_observes_flight_path_without_altitude_io() {
    let (_, _, simulator) = flight_runtime("calibrated_airspeed = 250.0");
    let targets = InitialisationConfig {
        calibrated_airspeed: Some(250.0),
        ..InitialisationConfig::default()
    };
    let mut aircraft = A32nxInitialiser::default();
    simulator.borrow_mut().clear_operations();
    aircraft
        .submit(&mut *simulator.borrow_mut(), targets, Duration::ZERO)
        .unwrap();
    for variable in [PRESSURE_ALTITUDE, PLANE_ALTITUDE] {
        assert!(!simulator.borrow().operations.iter().any(|op| match op {
            Operation::Read { variable: name, .. }
            | Operation::ValidateRead { variable: name, .. }
            | Operation::Write { variable: name, .. } => name == variable,
            _ => false,
        }));
    }
    for (variable, _, _) in ATTITUDE_VARIABLES {
        assert!(writes_to(&simulator, variable));
    }
}

#[test]
fn configured_pitch_and_roll_override_altitude_defaults_with_aeronautical_signs() {
    for (settings, native_pitch, native_roll) in [
        ("pitch = 3.5\nroll = -12.0", -3.5, 12.0),
        ("pitch = -2.0\nroll = 15.0", 2.0, -15.0),
        ("pitch = 3.5", -3.5, 0.0),
    ] {
        let (fixture, mut runtime, simulator) =
            flight_runtime(&format!("pressure_altitude = 10000.0\n{settings}"));
        observations(&simulator, PRESSURE_ALTITUDE, [10000.0; 2]);
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        assert_eq!(
            last_write(&simulator.borrow(), "A:PLANE PITCH DEGREES"),
            native_pitch
        );
        assert_eq!(
            last_write(&simulator.borrow(), "A:PLANE BANK DEGREES"),
            native_roll
        );
        assert_no_telemetry(&fixture);
        observations(&simulator, "A:PLANE PITCH DEGREES", [native_pitch]);
        observations(&simulator, "A:PLANE BANK DEGREES", [native_roll]);
        simulator.borrow_mut().time = duration(100.25);
        runtime.pre_update().unwrap();
        settle_250_knots(&fixture, &mut runtime, &simulator, 100.75);
        assert_eq!(fixture.telemetry_contents().lines().count(), 1);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        runtime.stop().unwrap();
        assert!(simulator.borrow().operations.iter().all(|operation| matches!(operation,
            Operation::Write { variable, .. } if variable == "K:AXIS_ELEVATOR_SET" || variable == ARMED_VARIABLE)));
    }
}

#[test]
fn individual_angle_targets_only_command_the_requested_axis_and_its_rotation_rate() {
    for (settings, angle, rate, omitted_angle, omitted_rate, native_target) in [
        (
            "pitch = 3.0",
            "A:PLANE PITCH DEGREES",
            "A:ROTATION VELOCITY BODY X",
            "A:PLANE BANK DEGREES",
            "A:ROTATION VELOCITY BODY Z",
            -3.0,
        ),
        (
            "roll = -15.0",
            "A:PLANE BANK DEGREES",
            "A:ROTATION VELOCITY BODY Z",
            "A:PLANE PITCH DEGREES",
            "A:ROTATION VELOCITY BODY X",
            15.0,
        ),
    ] {
        let (fixture, mut runtime, simulator) = flight_runtime(settings);
        observations(&simulator, omitted_angle, [f64::NAN]);
        observations(&simulator, omitted_rate, [f64::NAN]);
        runtime.pre_update().unwrap();
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert_eq!(last_write(&simulator.borrow(), angle), native_target);
        assert_eq!(last_write(&simulator.borrow(), rate), 0.0);
        assert!(
            simulator
                .borrow()
                .operations
                .iter()
                .all(|operation| match operation {
                    Operation::Write { variable, .. } if variable == AIRCRAFT_READY => true,
                    Operation::Read { variable, .. }
                    | Operation::ValidateRead { variable, .. }
                    | Operation::Write { variable, .. } => variable == angle || variable == rate,
                    _ => false,
                })
        );
        assert_no_telemetry(&fixture);
        observations(&simulator, angle, [native_target]);
        simulator.borrow_mut().time = duration(100.25);
        runtime.pre_update().unwrap();
        assert_eq!(fixture.telemetry_contents().lines().count(), 1);
        runtime.stop().unwrap();
    }
}

#[test]
fn timeout_reports_fresh_signed_deviations_and_preserves_timeout_on_diagnostic_failure() {
    for fail_altitude in [false, true] {
        let (fixture, mut runtime, simulator) = flight_runtime(
            "pressure_altitude = 31089.0\ncalibrated_airspeed = 287.0\npitch = 1.9\nroll = 0.77",
        );
        observations(&simulator, "A:PLANE BANK DEGREES", [-0.77]);
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        // These are different from the previous frame and must be sampled before cleanup.
        observations(&simulator, PRESSURE_ALTITUDE, [31050.0]);
        observations(&simulator, CALIBRATED_AIRSPEED, [283.0]);
        observations(&simulator, "A:PLANE PITCH DEGREES", [-2.7]);
        observations(&simulator, "A:PLANE BANK DEGREES", [-0.77]);
        observations(&simulator, "A:ROTATION VELOCITY BODY X", [0.25]);
        observations(&simulator, "A:ROTATION VELOCITY BODY Z", [0.0]);
        if fail_altitude {
            simulator.borrow_mut().failure = Some(Failure::Read(PRESSURE_ALTITUDE.to_owned()));
        }
        simulator.borrow_mut().time = duration(1000.0);
        simulator.borrow_mut().clear_operations();
        let error = runtime.pre_update().unwrap_err();
        assert!(matches!(error.downcast_ref::<InitialisationError>(),
            Some(InitialisationError::Timeout { pending, .. })
            if pending == &["pressure_altitude", "calibrated_airspeed", "vertical_speed"]));
        let message = error.to_string();
        if fail_altitude {
            assert!(message.contains("pressure_altitude: readback unavailable:"));
        } else {
            assert!(message.contains("pressure_altitude: target=31089.0000 feet, current=31050.0000 feet, difference(target-current)=+39.0000 feet, tolerance=+/-10.0000 feet"));
        }
        assert!(message.contains("calibrated_airspeed: target=287.0000 knots, current=283.0000 knots, difference(target-current)=+4.0000 knots, tolerance=+/-1.0000 knots"));
        assert!(message.contains("pitch: target=1.9000 deg, current=2.7000 deg, difference(target-current)=-0.8000 deg, tolerance=+/-0.5000 deg"));
        assert!(message.contains("pitch_rate: target=0.0000 deg/s, current=-0.2500 deg/s, difference(target-current)=+0.2500 deg/s, tolerance=+/-0.1000 deg/s"));
        assert!(!message.contains("\n  roll"));
        assert!(!message.contains("zfw:"));
        assert_eq!(
            write_operations(&simulator.borrow()),
            [
                Operation::Write {
                    variable: AIRCRAFT_READY.to_owned(),
                    value: 1.0
                },
                Operation::Write {
                    variable: ARMED_VARIABLE.to_owned(),
                    value: 0.0
                },
            ]
        );
        assert_no_telemetry(&fixture);
        runtime.stop().unwrap();
    }
}

#[test]
fn timeout_diagnostics_omit_inclusive_tolerances_and_wrap_roll_like_readiness() {
    let (_, _, simulator) = flight_runtime("");
    let targets = InitialisationConfig {
        zfw: Some(60000.0),
        gw: Some(65000.0),
        gwcg: Some(25.0),
        pressure_altitude: Some(10000.0),
        calibrated_airspeed: Some(250.0),
        vertical_speed: None,
        pitch: Some(3.0),
        roll: Some(180.0),
        ..InitialisationConfig::default()
    };
    for (variable, value) in [
        ("L:A32NX_AIRFRAME_ZFW", 60100.0),
        ("L:A32NX_AIRFRAME_GW", 64900.0),
        ("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", 25.01),
        (PRESSURE_ALTITUDE, 10010.0),
        (CALIBRATED_AIRSPEED, 249.0),
        ("A:PLANE PITCH DEGREES", -2.5),
        ("A:PLANE BANK DEGREES", 180.0),
        ("A:ROTATION VELOCITY BODY X", 0.1),
        ("A:ROTATION VELOCITY BODY Z", -0.1),
    ] {
        observations(&simulator, variable, [value]);
    }
    let initialiser = A32nxInitialiser::default();
    simulator.borrow_mut().clear_operations();
    assert!(
        initialiser
            .timeout_diagnostics(&mut *simulator.borrow_mut(), targets)
            .is_empty()
    );
    assert!(write_operations(&simulator.borrow()).is_empty());
    for (variable, value) in [
        ("L:A32NX_AIRFRAME_ZFW", 59899.0),
        ("L:A32NX_AIRFRAME_GW", 65101.0),
        ("L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC", 25.02),
    ] {
        observations(&simulator, variable, [value]);
    }
    let message = initialiser.timeout_diagnostics(&mut *simulator.borrow_mut(), TARGETS);
    assert!(message.contains(
        "zfw: target=60000.0000 kg, current=59899.0000 kg, difference(target-current)=+101.0000 kg"
    ));
    assert!(message.contains(
        "gw: target=65000.0000 kg, current=65101.0000 kg, difference(target-current)=-101.0000 kg"
    ));
    assert!(message.contains("gwcg: target=25.0000 % MAC, current=25.0200 % MAC, difference(target-current)=-0.0200 % MAC"));
}

#[test]
fn attitude_only_timeout_reports_the_unsettled_angles_and_stops_commands() {
    let (fixture, mut runtime, simulator) = flight_runtime("pitch = 3.0\nroll = 15.0");
    runtime.pre_update().unwrap();
    runtime.pre_update().unwrap();
    simulator.borrow_mut().time = duration(160.0);
    simulator.borrow_mut().clear_operations();
    let error = runtime.pre_update().unwrap_err();
    assert!(matches!(error.downcast_ref::<InitialisationError>(),
        Some(InitialisationError::Timeout { pending, .. }) if pending == &["pitch", "roll"]));
    assert_eq!(
        write_operations(&simulator.borrow()),
        [Operation::Write {
            variable: ARMED_VARIABLE.to_owned(),
            value: 0.0
        }]
    );
    assert_no_telemetry(&fixture);
    runtime.stop().unwrap();
}

#[test]
fn invalid_programmatic_angles_fail_before_simulator_commands() {
    for (field, value) in [
        ("pitch", f64::NAN),
        ("pitch", f64::INFINITY),
        ("pitch", -90.001),
        ("pitch", 90.001),
        ("roll", f64::NEG_INFINITY),
        ("roll", -180.001),
        ("roll", 180.001),
    ] {
        let (_, _, simulator) = flight_runtime("");
        simulator.borrow_mut().clear_operations();
        let targets = InitialisationConfig {
            pitch: Some(if field == "pitch" { value } else { 0.0 }),
            roll: Some(if field == "roll" { value } else { 0.0 }),
            ..InitialisationConfig::default()
        };
        let error = A32nxInitialiser::default()
            .submit(&mut *simulator.borrow_mut(), targets, Duration::ZERO)
            .unwrap_err();
        assert!(
            matches!(error, InitialisationError::InvalidAttitudeTarget { field: actual, .. } if actual == field)
        );
        assert!(simulator.borrow().operations.is_empty());
    }
}

#[test]
fn configured_roll_accepts_equivalent_readback_across_the_180_degree_boundary() {
    for (target, native_readback) in [
        (180.0, 180.0),
        (-180.0, -180.0),
        (179.75, 180.0),
        (-179.75, -180.0),
    ] {
        let (_, _, simulator) = flight_runtime("");
        observations(&simulator, "A:PLANE BANK DEGREES", [native_readback]);
        let targets = InitialisationConfig {
            roll: Some(target),
            ..InitialisationConfig::default()
        };
        let mut aircraft = A32nxInitialiser::default();
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, Duration::ZERO)
            .unwrap();
        assert!(
            aircraft
                .is_ready(&mut *simulator.borrow_mut(), targets)
                .unwrap()
        );
    }
}

#[test]
fn altitude_ramps_in_both_directions_using_irregular_elapsed_simulator_time() {
    for (initial, target) in [(1000.0, 2000.0), (2000.0, 1000.0)] {
        let (_, _, simulator) = flight_runtime(&format!("pressure_altitude = {target}"));
        let targets = InitialisationConfig {
            pressure_altitude: Some(target),
            ..InitialisationConfig::default()
        };
        let mut aircraft = A32nxInitialiser::default();
        let mut actual = initial;
        for elapsed in [0.0, 0.017, 0.1, 0.1, 0.65, 0.9, 1.0, 1.6, 2.0, 2.6] {
            observations(&simulator, PRESSURE_ALTITUDE, [actual]);
            observations(&simulator, PLANE_ALTITUDE, [actual + 300.0]);
            simulator.borrow_mut().clear_operations();
            aircraft
                .submit(
                    &mut *simulator.borrow_mut(),
                    targets,
                    duration(100.0 + elapsed),
                )
                .unwrap();
            let demand = last_write(&simulator.borrow(), PLANE_ALTITUDE);
            let expected = initial + (target - initial).signum() * (500.0 * elapsed).min(1000.0);
            assert!((demand - (expected + 300.0)).abs() < 1e-8);
            actual = demand - 300.0;
        }
        assert!(aircraft.pressure_altitude.unwrap().is_ready(target));
    }
}

#[test]
fn ramp_travel_extends_the_runtime_deadline_and_timeout_stops_all_commands() {
    let (fixture, mut runtime, simulator) = flight_runtime(
        "pressure_altitude = 1000.0\ncalibrated_airspeed = 287.0\nlanding_gear = 'up'",
    );
    observations(&simulator, PRESSURE_ALTITUDE, [0.0; 2]);
    observations(&simulator, PLANE_ALTITUDE, [0.0; 2]);
    observations(&simulator, CALIBRATED_AIRSPEED, [0.0; 4]);
    runtime.pre_update().unwrap();
    runtime.pre_update().unwrap();
    simulator.borrow_mut().time = duration(170.0); // Travel extends the 60-second allowance.
    runtime.pre_update().unwrap();
    assert_no_telemetry(&fixture);
    // 60 s settling + max(1,000 ft / 500 ft/s, 287 kt / 5 kt/s) = 117.4 s.
    simulator.borrow_mut().time = duration(217.4);
    simulator.borrow_mut().clear_operations();
    let error = runtime.pre_update().unwrap_err();
    assert!(matches!(
        error.downcast_ref::<InitialisationError>(),
        Some(InitialisationError::Timeout { timeout, pending, .. })
            if *timeout == duration(117.4)
                && pending == &["landing_gear", "pressure_altitude", "calibrated_airspeed", "vertical_speed"]
    ));
    assert_eq!(
        write_operations(&simulator.borrow()),
        [
            Operation::Write {
                variable: AIRCRAFT_READY.to_owned(),
                value: 1.0
            },
            Operation::Write {
                variable: ARMED_VARIABLE.to_owned(),
                value: 0.0
            },
        ]
    );
    assert_no_telemetry(&fixture);
    runtime.stop().unwrap();
}

#[test]
fn pressure_altitude_feedback_converges_with_weather_offsets_and_scale_changes() {
    for (offset, scale) in [
        (0.0, 1.0),
        (500.0, 1.0),
        (-500.0, 1.0),
        (500.0, 0.85),
        (-500.0, 1.15),
    ] {
        let (fixture, mut runtime, simulator) = flight_runtime("pressure_altitude = 10000.0");
        runtime.pre_update().unwrap();
        let mut geometric = 1000.0;
        let mut ready = false;
        for step in 1..=400 {
            let pressure = (geometric - offset) * scale;
            observations(&simulator, PRESSURE_ALTITUDE, [pressure]);
            observations(&simulator, PLANE_ALTITUDE, [geometric]);
            simulator.borrow_mut().time = duration(100.0 + f64::from(step));
            simulator.borrow_mut().clear_operations();
            runtime.pre_update().unwrap();
            geometric = last_write(&simulator.borrow(), PLANE_ALTITUDE);
            if (pressure - 10000.0).abs() <= 10.0 {
                ready = true;
                break;
            }
            assert_no_telemetry(&fixture);
        }
        assert!(ready, "did not converge for offset {offset}, scale {scale}");
        let now = simulator.borrow().time.as_secs_f64() + 0.5;
        settle_250_knots(&fixture, &mut runtime, &simulator, now);
        assert_eq!(fixture.telemetry_contents().lines().count(), 1);
        runtime.stop().unwrap();
    }
}

#[test]
fn altitude_tolerances_are_inclusive_during_natural_flight_verification() {
    for (actual, ready) in [
        (9990.0, true),
        (10010.0, true),
        (9989.999, false),
        (10010.001, false),
    ] {
        let (_, _, simulator) = flight_runtime("pressure_altitude = 10000.0");
        let targets = InitialisationConfig {
            pressure_altitude: Some(10000.0),
            ..InitialisationConfig::default()
        };
        let mut aircraft = A32nxInitialiser::default();
        observations(&simulator, PRESSURE_ALTITUDE, [10000.0]);
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, Duration::ZERO)
            .unwrap();
        for now in [1.0, 2.0, 3.0, 4.0, 5.0] {
            observations(&simulator, PRESSURE_ALTITUDE, [actual]);
            aircraft
                .submit(&mut *simulator.borrow_mut(), targets, duration(now))
                .unwrap();
        }
        simulator.borrow_mut().clear_operations();
        assert_eq!(
            aircraft
                .is_ready(&mut *simulator.borrow_mut(), targets)
                .unwrap(),
            ready
        );
        assert!(simulator.borrow().operations.is_empty());
    }
}

#[test]
fn immediate_command_echo_cannot_satisfy_airspeed_readiness() {
    let (fixture, mut runtime, simulator) = flight_runtime("calibrated_airspeed = 250.0");
    observations(&simulator, CALIBRATED_AIRSPEED, [250.0]);
    simulator.borrow_mut().echo_airspeed_write = true;
    runtime.pre_update().unwrap();
    runtime.pre_update().unwrap();
    assert_no_telemetry(&fixture);
    assert_eq!(simulator.borrow().reads[CALIBRATED_AIRSPEED], [250.0]);
    simulator.borrow_mut().time = duration(100.25);
    runtime.pre_update().unwrap();
    assert_no_telemetry(&fixture);
    simulator.borrow_mut().echo_airspeed_write = false;
    settle_250_knots(&fixture, &mut runtime, &simulator, 100.75);
    assert_eq!(fixture.telemetry_contents().lines().count(), 1);
    runtime.stop().unwrap();
}

#[test]
fn all_groups_must_be_ready_together_and_flight_observations_precede_every_setup_write() {
    let (fixture, mut runtime, simulator) =
        flight_runtime(&format!("{FLIGHT_CONDITION_SETTINGS}\nflaps = 2"));
    observations(&simulator, PRESSURE_ALTITUDE, [10000.0; 3]);
    observations(&simulator, CALIBRATED_AIRSPEED, [250.0, 248.0, 250.0]);
    observations(&simulator, MOVEMENT_VARIABLES[0], [1.0, 0.0, 0.0]);
    runtime.pre_update().unwrap();
    for now in [100.25, 100.75, 101.0] {
        simulator.borrow_mut().time = duration(now);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        let operations = simulator.borrow().operations.clone();
        let first_write = operations
            .iter()
            .position(|op| matches!(op, Operation::Write { .. }))
            .unwrap();
        let observations = operations.iter().enumerate().filter(|(_, op)| matches!(op, Operation::Read { variable, .. } if variable.starts_with("A:"))).map(|(index, _)| index).collect::<Vec<_>>();
        assert!(observations.len() >= 8);
        assert!(observations.iter().all(|index| *index < first_write));
        for event in [PLANE_ALTITUDE, CALIBRATED_AIRSPEED, "K:FLAPS_2"] {
            assert_eq!(
                operations
                    .iter()
                    .filter(
                        |op| matches!(op, Operation::Write { variable, .. } if variable == event)
                    )
                    .count(),
                if (event == CALIBRATED_AIRSPEED || event == PLANE_ALTITUDE) && now != 100.25 {
                    0
                } else {
                    1
                }
            );
        }
        assert_no_telemetry(&fixture);
    }
    settle_250_knots(&fixture, &mut runtime, &simulator, 101.5);
    assert_eq!(fixture.telemetry_contents().lines().count(), 1);
    runtime.stop().unwrap();
}

#[test]
fn malformed_observations_and_non_finite_corrections_fail_before_setup_writes() {
    for (variable, value) in [
        (PRESSURE_ALTITUDE, f64::NAN),
        (PLANE_ALTITUDE, f64::INFINITY),
        ("A:PLANE PITCH DEGREES", f64::NAN),
        ("A:PLANE BANK DEGREES", f64::INFINITY),
        ("A:ROTATION VELOCITY BODY X", f64::NEG_INFINITY),
        ("A:ROTATION VELOCITY BODY Z", f64::NAN),
        (CALIBRATED_AIRSPEED, f64::NEG_INFINITY),
        (CALIBRATED_AIRSPEED, -0.1),
    ] {
        let (fixture, mut runtime, simulator) =
            flight_runtime(&format!("{FLIGHT_CONDITION_SETTINGS}\nflaps = 2"));
        observations(&simulator, variable, [value]);
        runtime.pre_update().unwrap();
        let error = runtime.pre_update().unwrap_err();
        assert!(matches!(
            error.downcast_ref::<InitialisationError>(),
            Some(
                InitialisationError::Readback(SimulatorError::NonFiniteRead { .. })
                    | InitialisationError::InvalidReadback { .. }
            )
        ));
        assert_no_replay_writes(&simulator.borrow());
        assert_no_telemetry(&fixture);
        runtime.stop().unwrap();
    }
    let (fixture, mut runtime, simulator) = flight_runtime("pressure_altitude = 10000.0");
    observations(&simulator, PRESSURE_ALTITUDE, [-f64::MAX]);
    observations(&simulator, PLANE_ALTITUDE, [f64::MAX]);
    runtime.pre_update().unwrap();
    assert!(matches!(
        runtime
            .pre_update()
            .unwrap_err()
            .downcast_ref::<InitialisationError>(),
        Some(InitialisationError::InvalidInitialisationDuration { .. })
    ));
    assert_no_replay_writes(&simulator.borrow());
    assert_no_telemetry(&fixture);
    runtime.stop().unwrap();
}

#[test]
fn positive_targets_allow_negative_initial_altitudes_and_geometric_demands() {
    for (pressure, geometric, target, demanded) in [
        (-500.0, 1000.0, 10000.0, 1500.0),
        (500.0, -500.0, 100.0, -900.0),
    ] {
        let (_, _, simulator) = flight_runtime("pressure_altitude = 10000.0");
        observations(&simulator, PRESSURE_ALTITUDE, [pressure; 2]);
        observations(&simulator, PLANE_ALTITUDE, [geometric; 2]);
        let mut aircraft = A32nxInitialiser::default();
        aircraft
            .submit(
                &mut *simulator.borrow_mut(),
                InitialisationConfig {
                    pressure_altitude: Some(target),
                    ..InitialisationConfig::default()
                },
                Duration::ZERO,
            )
            .unwrap();
        assert_eq!(last_write(&simulator.borrow(), PLANE_ALTITUDE), geometric);
        aircraft
            .submit(
                &mut *simulator.borrow_mut(),
                InitialisationConfig {
                    pressure_altitude: Some(target),
                    ..InitialisationConfig::default()
                },
                duration(1.0),
            )
            .unwrap();
        assert_eq!(last_write(&simulator.borrow(), PLANE_ALTITUDE), demanded);
    }
}

#[test]
fn ignored_setters_timeout_before_new_writes_and_a_subsequent_run_uses_fresh_observations() {
    let (fixture, mut runtime, simulator) = flight_runtime(FLIGHT_CONDITION_SETTINGS);
    runtime.pre_update().unwrap();
    runtime.pre_update().unwrap();
    simulator.borrow_mut().time = duration(161.999);
    runtime.pre_update().unwrap();
    assert_no_telemetry(&fixture);
    simulator.borrow_mut().clear_operations();
    simulator.borrow_mut().time = duration(162.0);
    let error = runtime.pre_update().unwrap_err();
    assert!(
        matches!(error.downcast_ref::<InitialisationError>(), Some(InitialisationError::Timeout { targets, pending, timeout, .. })
            if targets.pressure_altitude == Some(10000.0) && targets.calibrated_airspeed == Some(250.0)
                && *timeout == duration(62.0)
                && pending == &["pressure_altitude", "calibrated_airspeed", "vertical_speed"])
    );
    assert_eq!(
        write_operations(&simulator.borrow()),
        [
            Operation::Write {
                variable: AIRCRAFT_READY.to_owned(),
                value: 1.0
            },
            Operation::Write {
                variable: ARMED_VARIABLE.to_owned(),
                value: 0.0
            },
        ]
    );
    observations(&simulator, ARMED_VARIABLE, [0.0; 3]);
    for now in [162.0, 163.0, 164.0] {
        simulator.borrow_mut().time = duration(now);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert_eq!(
            simulator.borrow().operations,
            [Operation::Read {
                variable: ARMED_VARIABLE.to_owned(),
                unit: None,
            }]
        );
    }
    observations(&simulator, ARMED_VARIABLE, [0.0, 1.0]);
    observations(&simulator, PRESSURE_ALTITUDE, [10000.0]);
    observations(&simulator, CALIBRATED_AIRSPEED, [250.0]);
    runtime.pre_update().unwrap();
    runtime.pre_update().unwrap();
    assert_no_telemetry(&fixture);
    settle_250_knots(&fixture, &mut runtime, &simulator, 164.25);
    assert_eq!(fixture.telemetry_contents().lines().count(), 1);
    runtime.pre_update().unwrap();
    runtime.stop().unwrap();
    assert_eq!(fixture.telemetry_contents().lines().count(), 2);
}

#[test]
fn flight_condition_io_failures_reset_arming_and_allow_another_run() {
    for failure in [
        Failure::ValidateRead(PRESSURE_ALTITUDE.to_owned()),
        Failure::Read(PLANE_ALTITUDE.to_owned()),
        Failure::Read(CALIBRATED_AIRSPEED.to_owned()),
        Failure::Read(VERTICAL_SPEED.to_owned()),
        Failure::Write(PLANE_ALTITUDE.to_owned()),
        Failure::Write(CALIBRATED_AIRSPEED.to_owned()),
        Failure::ValidateRead("A:PLANE PITCH DEGREES".to_owned()),
        Failure::ValidateRead("A:PLANE BANK DEGREES".to_owned()),
        Failure::ValidateRead("A:ROTATION VELOCITY BODY X".to_owned()),
        Failure::ValidateRead("A:ROTATION VELOCITY BODY Z".to_owned()),
        Failure::Read("A:PLANE PITCH DEGREES".to_owned()),
        Failure::Read("A:PLANE BANK DEGREES".to_owned()),
        Failure::Read("A:ROTATION VELOCITY BODY X".to_owned()),
        Failure::Read("A:ROTATION VELOCITY BODY Z".to_owned()),
        Failure::Write("A:PLANE PITCH DEGREES".to_owned()),
        Failure::Write("A:PLANE BANK DEGREES".to_owned()),
        Failure::Write("A:ROTATION VELOCITY BODY X".to_owned()),
        Failure::Write("A:ROTATION VELOCITY BODY Z".to_owned()),
        Failure::Write(THROTTLE_EVENTS[0].to_owned()),
        Failure::Write(THROTTLE_EVENTS[1].to_owned()),
    ] {
        let (fixture, mut runtime, simulator) = flight_runtime(FLIGHT_CONDITION_SETTINGS);
        runtime.pre_update().unwrap();
        // The first sample commands IAS; capture its baseline after readback settles.
        simulator.borrow_mut().time = duration(100.25);
        runtime.pre_update().unwrap();
        simulator.borrow_mut().time = duration(100.75);
        runtime.pre_update().unwrap();
        simulator.borrow_mut().time = duration(101.25);
        simulator.borrow_mut().failure = Some(failure);
        let error = runtime.pre_update().unwrap_err();
        assert!(matches!(
            error.downcast_ref::<InitialisationError>(),
            Some(InitialisationError::Submit(_) | InitialisationError::Readback(_))
        ));
        assert_eq!(
            simulator.borrow().operations.last(),
            Some(&Operation::Write {
                variable: ARMED_VARIABLE.to_owned(),
                value: 0.0
            })
        );
        assert_no_telemetry(&fixture);
        simulator.borrow_mut().failure = None;
        observations(&simulator, ARMED_VARIABLE, [0.0, 1.0]);
        observations(&simulator, PRESSURE_ALTITUDE, [10000.0]);
        observations(&simulator, CALIBRATED_AIRSPEED, [250.0]);
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        settle_250_knots(&fixture, &mut runtime, &simulator, 101.5);
        runtime.pre_update().unwrap();
        runtime.stop().unwrap();
        assert_eq!(fixture.telemetry_contents().lines().count(), 2);
    }
}

#[test]
fn unsupported_aircraft_skip_flight_condition_interfaces_and_shutdown_does_not_restore_them() {
    let (fixture, mut runtime, simulator) = flight_runtime(FLIGHT_CONDITION_SETTINGS);
    simulator.borrow_mut().string_reads = VecDeque::from([Ok("C172".to_owned())]);
    for variable in [PLANE_ALTITUDE, PRESSURE_ALTITUDE, CALIBRATED_AIRSPEED] {
        simulator.borrow_mut().reads.remove(variable);
    }
    runtime.pre_update().unwrap();
    simulator.borrow_mut().clear_operations();
    runtime.pre_update().unwrap();
    runtime.pre_update().unwrap();
    runtime.stop().unwrap();
    assert!(simulator.borrow().operations.iter().all(|op| matches!(op, Operation::Write { variable, .. } if variable == "K:AXIS_ELEVATOR_SET" || variable == ARMED_VARIABLE)));
    assert_eq!(fixture.telemetry_contents().lines().count(), 2);

    let (fixture, mut runtime, simulator) = flight_runtime(FLIGHT_CONDITION_SETTINGS);
    runtime.pre_update().unwrap();
    runtime.pre_update().unwrap();
    simulator.borrow_mut().clear_operations();
    runtime.stop().unwrap();
    runtime.stop().unwrap();
    assert!(
        simulator.borrow().operations.iter().all(
            |op| matches!(op, Operation::Write { variable, .. } if variable == ARMED_VARIABLE || variable == AIRCRAFT_READY)
        )
    );
    assert_eq!(last_write(&simulator.borrow(), AIRCRAFT_READY), 1.0);
    assert_no_telemetry(&fixture);
}

fn speed_targets() -> InitialisationConfig {
    InitialisationConfig {
        calibrated_airspeed: Some(250.0),
        vertical_speed: None,
        ..InitialisationConfig::default()
    }
}

fn speed_step(
    aircraft: &mut A32nxInitialiser,
    simulator: &SimulatorHandle,
    now: f64,
    observed: f64,
) -> bool {
    observations(simulator, CALIBRATED_AIRSPEED, [observed]);
    let mut simulator = simulator.borrow_mut();
    simulator.clear_operations();
    aircraft
        .submit(&mut *simulator, speed_targets(), duration(now))
        .unwrap();
    aircraft.is_ready(&mut *simulator, speed_targets()).unwrap()
}

fn writes_to(simulator: &SimulatorHandle, event: &str) -> bool {
    simulator.borrow().operations.iter().any(|operation| {
        matches!(operation,
        Operation::Write { variable, .. } if variable == event)
    })
}

#[test]
fn airspeed_commands_ramp_in_both_directions_and_allow_a_zero_initial_speed() {
    for initial in [0.0, 300.0] {
        let (_, _, simulator) = flight_runtime("calibrated_airspeed = 250.0");
        let mut aircraft = A32nxInitialiser::default();
        let mut observed = initial;
        let mut previous_time = 100.0;
        let mut previous_demand = initial;
        let mut corrections = 0;
        for now in [
            100.0, 100.0, 100.17, 100.31, 100.81, 100.9, 101.6, 101.9, 102.5, 102.99, 103.0, 103.6,
            104.1, 104.1, 104.5,
        ] {
            assert!(!speed_step(&mut aircraft, &simulator, now, observed));
            if writes_to(&simulator, CALIBRATED_AIRSPEED) {
                let demand = last_write(&simulator.borrow(), CALIBRATED_AIRSPEED);
                if now == 100.0 {
                    assert_eq!(demand, initial);
                } else {
                    assert!((demand - previous_demand).abs() <= 5.0 * (now - previous_time) + 1e-8);
                    assert_eq!(
                        (demand - previous_demand).signum(),
                        (250.0 - initial).signum()
                    );
                    corrections += 1;
                }
                previous_time = now;
                previous_demand = demand;
                observed = demand;
            }
        }
        assert!(corrections >= 3);
        assert!((observed - initial).abs() <= 22.5 + 1e-8);
    }
}

#[test]
fn speed_setup_verifies_three_unforced_seconds_then_stops_for_playback() {
    let (fixture, mut runtime, simulator) = flight_runtime("calibrated_airspeed = 250.0");
    runtime.pre_update().unwrap();
    for tick in 0..=9 {
        observations(&simulator, CALIBRATED_AIRSPEED, [250.0]);
        simulator.borrow_mut().time = duration(100.0 + f64::from(tick) * 0.5);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert_eq!(writes_to(&simulator, CALIBRATED_AIRSPEED), tick == 0);
        for event in THROTTLE_EVENTS {
            assert_eq!(
                writes_to(&simulator, event),
                matches!(tick, 0 | 3 | 5 | 7 | 9)
            );
        }
        if tick < 9 {
            assert_no_telemetry(&fixture);
        }
    }
    assert_eq!(fixture.telemetry_contents().lines().count(), 1);
    simulator.borrow_mut().clear_operations();
    runtime.pre_update().unwrap();
    for event in [CALIBRATED_AIRSPEED, THROTTLE_EVENTS[0], THROTTLE_EVENTS[1]] {
        assert!(!writes_to(&simulator, event));
    }
    runtime.stop().unwrap();
}

#[test]
fn stable_airspeed_samples_allow_small_readback_steps_between_simulator_frames() {
    for frame_times_ms in [&[17_u64, 16, 17][..], &[13_u64, 29, 11, 23][..]] {
        let (fixture, mut runtime, simulator) = flight_runtime(
            "pressure_altitude = 31089.0\ncalibrated_airspeed = 287.0\npitch = 1.9\nroll = 0.77",
        );
        observations(&simulator, PRESSURE_ALTITUDE, [31089.0]);
        observations(&simulator, CALIBRATED_AIRSPEED, [287.0]);
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        let mut elapsed_ms = 0;
        let mut playback = false;
        for frame in 0..600 {
            elapsed_ms += frame_times_ms[frame % frame_times_ms.len()];
            let elapsed = elapsed_ms as f64 / 1000.0;
            // The complete drift samples remain within tolerance, even though
            // a small readback step over one short frame can exceed that rate.
            let noise = if frame % 2 == 0 { 0.004 } else { -0.004 };
            observations(
                &simulator,
                CALIBRATED_AIRSPEED,
                [287.354 - 0.0133 * elapsed + noise],
            );
            observations(&simulator, PRESSURE_ALTITUDE, [31082.4]);
            observations(&simulator, PLANE_ALTITUDE, [31582.4]);
            observations(&simulator, VERTICAL_SPEED, [-10.7]);
            observations(&simulator, "A:PLANE PITCH DEGREES", [-3.0]);
            observations(&simulator, "A:PLANE BANK DEGREES", [-0.794]);
            observations(
                &simulator,
                crate::ths::TRIM_POSITION,
                [(1.006 + 0.002 * elapsed + 4.0) * 100.0 / 17.5],
            );
            simulator.borrow_mut().time = duration(100.0) + Duration::from_millis(elapsed_ms);
            simulator.borrow_mut().clear_operations();
            runtime.pre_update().unwrap();
            assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
            for event in THROTTLE_EVENTS {
                if writes_to(&simulator, event) {
                    assert_eq!(last_write(&simulator.borrow(), event), IDLE_AXIS);
                }
            }
            if fixture.directory.read_dir().unwrap().any(|entry| {
                entry
                    .unwrap()
                    .file_name()
                    .to_string_lossy()
                    .starts_with("telemetry_")
            }) {
                assert!((4.5..6.0).contains(&elapsed));
                playback = true;
                break;
            }
        }
        assert!(
            playback,
            "frame cadence {frame_times_ms:?} kept resetting readiness"
        );
        runtime.stop().unwrap();
    }
}

#[test]
fn ready_signal_waits_for_both_ramps_and_speed_verification_starts_after_release() {
    for (initial_altitude, initial_speed, release_tick) in
        [(7500.0, 240.0, 10), (9900.0, 220.0, 12)]
    {
        let (fixture, mut runtime, simulator) = flight_runtime(FLIGHT_CONDITION_SETTINGS);
        runtime.pre_update().unwrap();
        for tick in 0..=release_tick + 9 {
            let elapsed = f64::from(tick) * 0.5;
            let altitude = (initial_altitude + 500.0 * elapsed).min(10000.0);
            let speed = (initial_speed + 5.0 * elapsed).min(250.0);
            observations(&simulator, PRESSURE_ALTITUDE, [altitude]);
            observations(&simulator, PLANE_ALTITUDE, [altitude + 500.0]);
            observations(&simulator, CALIBRATED_AIRSPEED, [speed]);
            simulator.borrow_mut().time = duration(100.0 + elapsed);
            simulator.borrow_mut().clear_operations();
            runtime.pre_update().unwrap();
            let ready_writes: Vec<_> = simulator
                .borrow()
                .operations
                .iter()
                .filter_map(|op| match op {
                    Operation::Write { variable, value } if variable == AIRCRAFT_READY => {
                        Some(*value)
                    }
                    _ => None,
                })
                .collect();
            assert_eq!(
                ready_writes,
                if tick == 0 {
                    vec![0.0]
                } else if tick == release_tick {
                    vec![1.0]
                } else {
                    vec![]
                }
            );
            if tick == 0 {
                assert_eq!(
                    simulator
                        .borrow()
                        .operations
                        .iter()
                        .find(|op| matches!(op, Operation::Write { .. })),
                    Some(&Operation::Write {
                        variable: AIRCRAFT_READY.to_owned(),
                        value: 0.0
                    })
                );
            }
            if tick == release_tick {
                assert_eq!(
                    simulator
                        .borrow()
                        .operations
                        .iter()
                        .rfind(|op| matches!(op, Operation::Write { .. })),
                    Some(&Operation::Write {
                        variable: AIRCRAFT_READY.to_owned(),
                        value: 1.0
                    })
                );
            }
            if tick < release_tick + 9 {
                assert_no_telemetry(&fixture);
            }
        }
        assert_eq!(fixture.telemetry_contents().lines().count(), 1);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        runtime.stop().unwrap();
        assert!(!writes_to(&simulator, AIRCRAFT_READY));
    }
}

#[test]
fn setup_failure_and_shutdown_release_the_ready_signal_once() {
    for fail_setup in [false, true] {
        let (fixture, mut runtime, simulator) = flight_runtime(FLIGHT_CONDITION_SETTINGS);
        runtime.pre_update().unwrap();
        if fail_setup {
            simulator.borrow_mut().failure = Some(Failure::Write(PLANE_ALTITUDE.to_owned()));
            assert!(runtime.pre_update().is_err());
        } else {
            runtime.pre_update().unwrap();
            assert_eq!(last_write(&simulator.borrow(), AIRCRAFT_READY), 0.0);
            runtime.stop().unwrap();
        }
        assert_eq!(last_write(&simulator.borrow(), AIRCRAFT_READY), 1.0);
        assert_no_telemetry(&fixture);
        simulator.borrow_mut().clear_operations();
        runtime.stop().unwrap();
        assert!(!writes_to(&simulator, AIRCRAFT_READY));
    }
}

#[test]
fn failed_ready_writes_retain_errors_and_retry_before_accepting_another_arm() {
    for failure_point in ["start", "handoff", "timeout"] {
        let (fixture, mut runtime, simulator) = flight_runtime(FLIGHT_CONDITION_SETTINGS);
        runtime.pre_update().unwrap();
        if failure_point != "start" {
            runtime.pre_update().unwrap();
        }
        if failure_point == "handoff" {
            for now in [100.5, 101.0, 101.5] {
                simulator.borrow_mut().time = duration(now);
                runtime.pre_update().unwrap();
            }
        }
        if failure_point == "handoff" {
            observations(&simulator, PRESSURE_ALTITUDE, [10000.0]);
        }
        simulator.borrow_mut().failure = Some(Failure::Write(AIRCRAFT_READY.to_owned()));
        simulator.borrow_mut().fail_arming_reset = true;
        simulator.borrow_mut().time = duration(match failure_point {
            "start" => 100.0,
            "handoff" => 102.0,
            _ => 162.0,
        });
        simulator.borrow_mut().clear_operations();
        let error = runtime.pre_update().unwrap_err();
        let TerminationError::CleanupAfterFailure { primary, cleanup } =
            error.downcast_ref::<TerminationError>().unwrap();
        if failure_point == "start" {
            assert!(
                matches!(primary.downcast_ref::<InitialisationError>(), Some(InitialisationError::Submit(SimulatorError::CalculatorCodeWriteFailed { variable, value: 0.0 })) if variable == AIRCRAFT_READY)
            );
        } else if failure_point == "handoff" {
            assert!(
                matches!(primary.downcast_ref::<InitialisationError>(), Some(InitialisationError::Submit(SimulatorError::CalculatorCodeWriteFailed { variable, value: 1.0 })) if variable == AIRCRAFT_READY)
            );
        } else {
            assert!(matches!(
                primary.downcast_ref::<InitialisationError>(),
                Some(InitialisationError::Timeout { .. })
            ));
        }
        assert!(matches!(cleanup, CleanupError::Aircraft {
            source: SimulatorError::CalculatorCodeWriteFailed { variable, value: 1.0 },
            telemetry: None,
            arming: Some(SimulatorError::CalculatorCodeWriteFailed { variable: arming, value: 0.0 }),
        } if variable == AIRCRAFT_READY && arming == ARMED_VARIABLE));
        assert_no_telemetry(&fixture);
        observations(&simulator, ARMED_VARIABLE, [1.0]);
        simulator.borrow_mut().clear_operations();
        assert!(runtime.pre_update().is_err());
        assert!(simulator.borrow().operations.iter().all(|op| matches!(op, Operation::Write { variable, .. } if variable == AIRCRAFT_READY || variable == ARMED_VARIABLE)));

        observations(&simulator, PRESSURE_ALTITUDE, [9500.0; 32]);
        simulator.borrow_mut().failure = None;
        simulator.borrow_mut().fail_arming_reset = false;
        simulator
            .borrow_mut()
            .string_reads
            .push_back(Ok("A20N".to_owned()));
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        // Retry the pending arming reset, observe low, then accept a fresh edge.
        observations(&simulator, ARMED_VARIABLE, [0.0, 1.0]);
        runtime.pre_update().unwrap();
        runtime.pre_update().unwrap();
        assert_eq!(last_write(&simulator.borrow(), AIRCRAFT_READY), 1.0);
        simulator.borrow_mut().clear_operations();
        runtime.pre_update().unwrap();
        assert_eq!(last_write(&simulator.borrow(), AIRCRAFT_READY), 0.0);
        runtime.stop().unwrap();
        assert_eq!(last_write(&simulator.borrow(), AIRCRAFT_READY), 1.0);
    }
}

#[test]
fn speed_corrections_allow_an_unforced_interval_before_adjusting_the_throttles() {
    let (_, _, simulator) = flight_runtime("calibrated_airspeed = 250.0");
    let mut aircraft = A32nxInitialiser::default();
    assert!(!speed_step(&mut aircraft, &simulator, 100.0, 240.0));
    assert!(writes_to(&simulator, CALIBRATED_AIRSPEED));
    for (now, observed) in [(100.1, 250.0), (100.2, 249.8), (100.4, 249.6)] {
        assert!(!speed_step(&mut aircraft, &simulator, now, observed));
        for event in [CALIBRATED_AIRSPEED, THROTTLE_EVENTS[0], THROTTLE_EVENTS[1]] {
            assert!(!writes_to(&simulator, event));
        }
    }
    assert!(!speed_step(&mut aircraft, &simulator, 100.5, 249.5));
    assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
    assert!(!speed_step(&mut aircraft, &simulator, 100.9, 249.1));
    assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
    assert!(!speed_step(&mut aircraft, &simulator, 101.0, 249.0));
    assert!(writes_to(&simulator, CALIBRATED_AIRSPEED));
    for event in THROTTLE_EVENTS {
        assert_eq!(last_write(&simulator.borrow(), event), IDLE_AXIS);
    }
    // Same-time readback cannot establish a new baseline.
    assert!(!speed_step(&mut aircraft, &simulator, 101.0, 300.0));
    assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
    assert!(!speed_step(&mut aircraft, &simulator, 101.1, 250.5));
    assert!(!speed_step(&mut aircraft, &simulator, 101.5, 250.5));
    assert!(!speed_step(&mut aircraft, &simulator, 102.0, 250.0));
    assert_eq!(
        last_write(&simulator.borrow(), THROTTLE_EVENTS[0]),
        IDLE_AXIS
    );
    // After release, wait for setter readback to settle and then measure natural drift.
    for (now, observed) in [(102.1, 300.0), (102.5, 250.0), (103.0, 249.5)] {
        assert!(!speed_step(&mut aircraft, &simulator, now, observed));
        for event in [CALIBRATED_AIRSPEED, THROTTLE_EVENTS[0], THROTTLE_EVENTS[1]] {
            assert!(!writes_to(&simulator, event));
        }
    }
    assert!(!speed_step(&mut aircraft, &simulator, 103.5, 249.0));
    assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
    for event in THROTTLE_EVENTS {
        assert!(last_write(&simulator.borrow(), event) > IDLE_AXIS);
    }
}

#[test]
fn off_target_readback_is_corrected_without_increasing_thrust_before_release() {
    let (_, _, simulator) = flight_runtime("calibrated_airspeed = 287.0");
    let targets = InitialisationConfig {
        calibrated_airspeed: Some(287.0),
        vertical_speed: None,
        ..InitialisationConfig::default()
    };
    let mut aircraft = A32nxInitialiser::default();
    for (now, observed) in [(100.0, 300.0), (100.5, 300.0), (101.0, 299.5)] {
        observations(&simulator, CALIBRATED_AIRSPEED, [observed]);
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, duration(now))
            .unwrap();
    }
    for event in THROTTLE_EVENTS {
        assert_eq!(last_write(&simulator.borrow(), event), IDLE_AXIS);
    }
    assert_eq!(last_write(&simulator.borrow(), CALIBRATED_AIRSPEED), 295.5);
}

#[test]
fn throttle_feedback_obeys_rate_limits_and_never_commands_below_idle() {
    let (_, _, simulator) = flight_runtime("calibrated_airspeed = 250.0");
    let mut aircraft = A32nxInitialiser::default();
    assert!(!speed_step(&mut aircraft, &simulator, 100.0, 250.0));
    let mut previous = IDLE_AXIS;
    for event in THROTTLE_EVENTS {
        assert_eq!(last_write(&simulator.borrow(), event), previous);
    }
    let mut now = 100.5;
    let mut observed = 230.0;
    assert!(!speed_step(&mut aircraft, &simulator, now, observed));
    for tick in 1..=5 {
        now += 1.0;
        observed -= 1.0;
        assert!(!speed_step(&mut aircraft, &simulator, now, observed));
        let command = last_write(&simulator.borrow(), THROTTLE_EVENTS[0]);
        assert_eq!(command, (IDLE_AXIS + f64::from(tick) * 8192.0).min(16384.0));
        assert!((command - previous).abs() <= 8192.0 + 1.0);
        assert_eq!(last_write(&simulator.borrow(), THROTTLE_EVENTS[1]), command);
        assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
        previous = command;
    }
    assert_eq!(previous, 16384.0);
    // Acceleration beyond the recovery demand reduces thrust immediately, even below target.
    for _ in 0..16 {
        now += 1.0;
        observed += 2.0;
        assert!(!speed_step(&mut aircraft, &simulator, now, observed));
        for event in THROTTLE_EVENTS {
            let command = last_write(&simulator.borrow(), event);
            assert!(command <= previous);
            assert!((command - previous).abs() <= 4096.0 + 1.0);
            assert!((IDLE_AXIS..=16384.0).contains(&command));
        }
        assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
        previous = last_write(&simulator.borrow(), THROTTLE_EVENTS[0]);
    }
    assert_eq!(previous, IDLE_AXIS);
}

#[test]
fn small_drift_sign_changes_do_not_stall_the_next_throttle_ramp() {
    let (_, _, simulator) = flight_runtime("calibrated_airspeed = 250.0");
    let mut aircraft = A32nxInitialiser::default();
    assert!(!speed_step(&mut aircraft, &simulator, 100.0, 250.0));
    // These reversals stay within the accepted drift tolerance at 60 Hz.
    for tick in 1..=60 {
        let noise = if tick % 2 == 0 { 0.0002 } else { -0.0002 };
        assert!(!speed_step(
            &mut aircraft,
            &simulator,
            100.0 + f64::from(tick) / 60.0,
            250.0 + noise,
        ));
    }
    assert!(!speed_step(&mut aircraft, &simulator, 101.5, 249.9));
    assert!(!speed_step(&mut aircraft, &simulator, 102.5, 249.6));
    for event in THROTTLE_EVENTS {
        // Small accepted reversals must not suppress correction once real drift appears.
        assert!(last_write(&simulator.borrow(), event) > IDLE_AXIS + 383.0);
    }
}

#[test]
fn raw_throttle_setup_does_not_access_mapping_variables() {
    let (_, _, simulator) = flight_runtime("calibrated_airspeed = 250.0");
    let mut aircraft = A32nxInitialiser::default();
    for (now, speed) in [(100.0, 250.0), (100.5, 250.0), (101.5, 249.5)] {
        assert!(!speed_step(&mut aircraft, &simulator, now, speed));
        for operation in &simulator.borrow().operations {
            let variable = match operation {
                Operation::Read { variable, .. }
                | Operation::ValidateRead { variable, .. }
                | Operation::Write { variable, .. }
                | Operation::LocalVariableExists(variable)
                | Operation::ReadString(variable) => variable,
            };
            assert!(!variable.contains("THROTTLE_MAPPING"));
        }
    }
    assert!(last_write(&simulator.borrow(), THROTTLE_EVENTS[0]) > IDLE_AXIS);
}

#[test]
fn airspeed_diagnostics_report_confirmed_stability_and_drift_sample_age() {
    let (_, _, simulator) = flight_runtime("calibrated_airspeed = 250.0");
    let mut aircraft = A32nxInitialiser::default();
    for now in [100.0, 100.5, 102.5, 104.5, 104.517, 104.75] {
        assert!(!speed_step(&mut aircraft, &simulator, now, 250.0));
    }
    observations(&simulator, CALIBRATED_AIRSPEED, [250.0]);
    simulator.borrow_mut().clear_operations();
    let message = aircraft.timeout_diagnostics(&mut *simulator.borrow_mut(), speed_targets());
    assert!(message.contains("calibrated_airspeed: ready=false stable_for=2.000/3.000 s"));
    assert!(message.contains("drift=0.0000 kt/s drift_tolerance=+/-0.1000 kt/s"));
    assert!(message.contains(
        "sample_age=0.250 s observation_interval=1.000 s ramp_finished=true throttle_feedback=true"
    ));
    assert!(write_operations(&simulator.borrow()).is_empty());
}

#[test]
fn real_speed_or_sampled_drift_excursions_restart_the_airspeed_stability_window() {
    for speed_excursion in [false, true] {
        let (_, _, simulator) = flight_runtime("calibrated_airspeed = 250.0");
        let mut aircraft = A32nxInitialiser::default();
        for now in [100.0, 100.5, 102.5, 104.5] {
            assert!(!speed_step(&mut aircraft, &simulator, now, 250.0));
        }
        let settled_speed = if speed_excursion { 250.0 } else { 250.4 };
        assert!(!speed_step(
            &mut aircraft,
            &simulator,
            105.0,
            if speed_excursion { 251.1 } else { 250.25 },
        ));
        assert!(!speed_step(&mut aircraft, &simulator, 106.5, settled_speed));
        let ready_at = if speed_excursion { 110.5 } else { 112.5 };
        for now in [107.0, 108.5, 109.0, 110.5, 110.5, 112.5] {
            assert_eq!(
                speed_step(&mut aircraft, &simulator, now, settled_speed),
                now >= ready_at,
                "speed_excursion={speed_excursion}, time={now}",
            );
            assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
        }
    }
}

#[test]
fn disturbed_speed_restarts_verification_and_repeated_time_cannot_advance_it() {
    let (_, _, simulator) = flight_runtime("calibrated_airspeed = 250.0");
    let mut aircraft = A32nxInitialiser::default();
    for now in [100.0, 100.5, 102.5] {
        assert!(!speed_step(&mut aircraft, &simulator, now, 250.0));
    }
    assert!(!speed_step(&mut aircraft, &simulator, 102.5, 248.0));
    assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
    assert!(!speed_step(&mut aircraft, &simulator, 104.5, 249.5));
    assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
    assert!(last_write(&simulator.borrow(), THROTTLE_EVENTS[0]) > IDLE_AXIS);
    for now in [105.0, 106.5, 108.5, 110.5, 110.5] {
        assert!(!speed_step(&mut aircraft, &simulator, now, 250.0));
        assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
    }
    assert!(speed_step(&mut aircraft, &simulator, 112.5, 250.0));
    aircraft.reset();
    assert!(!speed_step(&mut aircraft, &simulator, 0.0, 250.0));
    assert!(writes_to(&simulator, CALIBRATED_AIRSPEED));
}

#[test]
fn completed_speed_ramps_recover_speed_errors_with_thrust_and_never_resume_ias_writes() {
    for initial in [240.0, 260.0] {
        let (_, _, simulator) = flight_runtime("calibrated_airspeed = 250.0");
        let mut aircraft = A32nxInitialiser::default();
        for (now, observed) in [(100.0, initial), (100.5, initial), (101.0, initial - 0.5)] {
            assert!(!speed_step(&mut aircraft, &simulator, now, observed));
        }
        let demand = last_write(&simulator.borrow(), CALIBRATED_AIRSPEED);
        assert!(!speed_step(&mut aircraft, &simulator, 101.5, demand));
        assert!(!speed_step(&mut aircraft, &simulator, 102.0, demand - 0.5));
        assert!(writes_to(&simulator, CALIBRATED_AIRSPEED));
        // The last IAS command must settle before natural drift is measured.
        for now in [102.0, 102.1, 102.5, 103.0] {
            assert!(!speed_step(&mut aircraft, &simulator, now, 240.0));
            assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
            for event in THROTTLE_EVENTS {
                assert!(!writes_to(&simulator, event));
            }
        }
        // Build a demand above idle so both upward and downward correction are observable.
        for now in [103.5, 104.5, 105.5] {
            assert!(!speed_step(&mut aircraft, &simulator, now, 240.0));
            assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
        }
        let previous = last_write(&simulator.borrow(), THROTTLE_EVENTS[0]);
        assert!(!speed_step(&mut aircraft, &simulator, 106.5, initial));
        let direction = (250.0 - initial).signum();
        let command = last_write(&simulator.borrow(), THROTTLE_EVENTS[0]);
        assert_eq!((command - previous).signum(), direction);
        assert_eq!(last_write(&simulator.borrow(), THROTTLE_EVENTS[1]), command);
        assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));

        // Zero drift or small acceleration toward target still needs thrust correction.
        assert!(!speed_step(
            &mut aircraft,
            &simulator,
            107.5,
            initial + direction * 0.25,
        ));
        assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
        let next_command = last_write(&simulator.borrow(), THROTTLE_EVENTS[0]);
        assert_eq!((next_command - command).signum(), direction);
        assert!((next_command - command).abs() <= 8192.0);
    }
}

#[test]
fn trim_settling_does_not_restart_steady_airspeed_verification() {
    let (_, _, simulator) = flight_runtime("calibrated_airspeed = 250.0\nths = 1.0");
    let targets = InitialisationConfig {
        ths: Some(1.0),
        ..speed_targets()
    };
    let mut aircraft = A32nxInitialiser::default();
    for tick in 0..=20 {
        simulator.borrow_mut().clear_operations();
        observations(&simulator, CALIBRATED_AIRSPEED, [250.0]);
        let trim = if tick < 13 {
            f64::from(tick)
        } else {
            5.0 / 17.5 * 100.0
        };
        observations(&simulator, "L:A32NX_HYD_TRIM_WHEEL_PERCENT", [trim]);
        aircraft
            .submit(
                &mut *simulator.borrow_mut(),
                targets,
                duration(100.0 + f64::from(tick) * 0.5),
            )
            .unwrap();
        assert_eq!(writes_to(&simulator, CALIBRATED_AIRSPEED), tick == 0);
        assert_eq!(
            aircraft
                .is_ready(&mut *simulator.borrow_mut(), targets)
                .unwrap(),
            tick == 20
        );
        if tick >= 9 {
            assert!(
                aircraft
                    .check_calibrated_airspeed(&mut *simulator.borrow_mut(), Some(250.0))
                    .unwrap()
            );
        }
    }
}

#[test]
fn airspeed_setup_ignores_missing_and_enabled_autothrust_status() {
    for status in [None, Some(1.0), Some(2.0)] {
        let (_, _, simulator) = flight_runtime("calibrated_airspeed = 250.0");
        if let Some(status) = status {
            observations(&simulator, "L:A32NX_AUTOTHRUST_STATUS", [status]);
        }
        let mut aircraft = A32nxInitialiser::default();
        assert!(!speed_step(&mut aircraft, &simulator, 100.0, 250.0));
        assert!(!speed_step(&mut aircraft, &simulator, 100.5, 250.0));
        assert!(!speed_step(&mut aircraft, &simulator, 101.5, 249.5));
        assert!(last_write(&simulator.borrow(), THROTTLE_EVENTS[0]) > IDLE_AXIS);
        assert!(
            simulator
                .borrow()
                .operations
                .iter()
                .all(|operation| match operation {
                    Operation::Read { variable, .. }
                    | Operation::ValidateRead { variable, .. }
                    | Operation::LocalVariableExists(variable) => !variable.contains("AUTOTHRUST"),
                    _ => true,
                })
        );
    }
}

#[test]
fn delayed_ias_setter_readback_is_discarded_before_throttle_feedback_starts() {
    let (_, _, simulator) = flight_runtime("calibrated_airspeed = 287.0");
    let targets = InitialisationConfig {
        calibrated_airspeed: Some(287.0),
        vertical_speed: None,
        ..InitialisationConfig::default()
    };
    let mut aircraft = A32nxInitialiser::default();
    for (now, observed) in [
        (100.0, 275.3),
        (100.1, 276.243), // The first later read still contains the setter transient.
        (100.25, 286.946),
        (100.5, 286.9),
        (101.0, 286.4), // Once settled, the aircraft is actually decelerating.
        (101.5, 286.0),
        (102.0, 285.5),
        (102.5, 285.0),
        (103.0, 284.5), // Terminal ramp command releases the computers.
        (103.1, 275.3), // Discard a further setter transient after release.
        (103.5, 287.0), // Capture the settled baseline.
        (104.0, 286.5),
        (104.5, 286.0),
    ] {
        observations(&simulator, CALIBRATED_AIRSPEED, [observed]);
        simulator.borrow_mut().clear_operations();
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, duration(now))
            .unwrap();
        if now > 100.0 && now < 101.0 {
            assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
            for event in THROTTLE_EVENTS {
                assert!(!writes_to(&simulator, event));
            }
        }
        if (101.0..=103.0).contains(&now) {
            for event in THROTTLE_EVENTS {
                if writes_to(&simulator, event) {
                    assert_eq!(last_write(&simulator.borrow(), event), IDLE_AXIS);
                }
            }
        }
        if now > 103.0 {
            assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
        }
    }
    for event in THROTTLE_EVENTS {
        assert!(last_write(&simulator.borrow(), event) > IDLE_AXIS);
    }
}

#[test]
fn inhibited_aircraft_cannot_accumulate_throttle_demand_before_release() {
    let (_, _, simulator) =
        flight_runtime("pressure_altitude = 31089.0\ncalibrated_airspeed = 287.0");
    let targets = InitialisationConfig {
        pressure_altitude: Some(31089.0),
        calibrated_airspeed: Some(287.0),
        ..InitialisationConfig::default()
    };
    let mut aircraft = A32nxInitialiser::default();
    for now in [100.0, 100.5, 101.0, 102.0, 104.0, 106.0, 108.0, 110.0] {
        observations(
            &simulator,
            CALIBRATED_AIRSPEED,
            [250.0 - (now - 100.0) * 0.5],
        );
        simulator.borrow_mut().clear_operations();
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, duration(now))
            .unwrap();
        assert_eq!(aircraft.stage, super::SetupStage::Repositioning);
        for event in THROTTLE_EVENTS {
            if writes_to(&simulator, event) {
                assert_eq!(last_write(&simulator.borrow(), event), IDLE_AXIS);
            }
        }
    }
    observations(&simulator, PRESSURE_ALTITUDE, [31089.0]);
    observations(&simulator, PLANE_ALTITUDE, [31589.0]);
    observations(&simulator, CALIBRATED_AIRSPEED, [287.0]);
    aircraft
        .submit(&mut *simulator.borrow_mut(), targets, duration(144.0))
        .unwrap();
    assert_eq!(aircraft.stage, super::SetupStage::Stabilising);
    for (now, speed) in [(144.5, 287.0), (146.5, 286.4)] {
        observations(&simulator, PRESSURE_ALTITUDE, [31089.0]);
        observations(&simulator, PLANE_ALTITUDE, [31589.0]);
        observations(&simulator, CALIBRATED_AIRSPEED, [speed]);
        simulator.borrow_mut().clear_operations();
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, duration(now))
            .unwrap();
    }
    for event in THROTTLE_EVENTS {
        assert!(last_write(&simulator.borrow(), event) > IDLE_AXIS);
    }
}

#[test]
fn completed_speed_ramp_keeps_correcting_ias_until_computers_are_released() {
    let (_, _, simulator) =
        flight_runtime("pressure_altitude = 31089.0\ncalibrated_airspeed = 287.0");
    let targets = InitialisationConfig {
        pressure_altitude: Some(31089.0),
        calibrated_airspeed: Some(287.0),
        ..InitialisationConfig::default()
    };
    let mut aircraft = A32nxInitialiser::default();
    let mut speed = 287.0;
    let mut last_ias_at = 100.0;
    let mut released_at = None;
    for tick in 0..=90 {
        let elapsed = f64::from(tick) * 0.5;
        let now = 100.0 + elapsed;
        let altitude = (9500.0 + 500.0 * elapsed).min(31089.0);
        if tick != 0 {
            // The log shows about -3.3 kt/s during the inhibited handoff gap.
            // This synthetic setter has no bias; engine feedback remains disabled.
            speed -= 3.3 * 0.5;
        }
        observations(&simulator, PRESSURE_ALTITUDE, [altitude]);
        observations(&simulator, PLANE_ALTITUDE, [altitude + 500.0]);
        observations(&simulator, CALIBRATED_AIRSPEED, [speed]);
        simulator.borrow_mut().clear_operations();
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, duration(now))
            .unwrap();
        if writes_to(&simulator, CALIBRATED_AIRSPEED) {
            speed = last_write(&simulator.borrow(), CALIBRATED_AIRSPEED);
            last_ias_at = now;
        }
        for event in THROTTLE_EVENTS {
            if writes_to(&simulator, event) {
                assert_eq!(last_write(&simulator.borrow(), event), IDLE_AXIS);
            }
        }
        assert!(
            (speed - 287.0).abs() <= 5.0,
            "speed escaped during inhibited altitude travel: time={now}, speed={speed}"
        );
        if aircraft.stage == super::SetupStage::Stabilising {
            assert!(now - last_ias_at <= 1.0);
            assert_eq!(last_write(&simulator.borrow(), AIRCRAFT_READY), 1.0);
            released_at = Some(now);
            break;
        }
    }
    let released_at = released_at.unwrap();
    assert!(released_at >= 143.0);
    for offset in [0.5, 1.0, 1.5, 2.0] {
        observations(&simulator, PRESSURE_ALTITUDE, [31089.0]);
        observations(&simulator, PLANE_ALTITUDE, [31589.0]);
        observations(&simulator, CALIBRATED_AIRSPEED, [speed - 0.2 * offset]);
        simulator.borrow_mut().clear_operations();
        aircraft
            .submit(
                &mut *simulator.borrow_mut(),
                targets,
                duration(released_at + offset),
            )
            .unwrap();
        assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
    }
}

#[test]
fn throttle_reduction_starts_below_target_and_quickens_for_large_speed_errors() {
    for (observed, drift, minimum_reduction) in [
        (280.235, 0.4778, 400.0),
        (284.322, 0.3657, 1500.0),
        (286.884, 0.4867, 3500.0),
    ] {
        let (_, _, simulator) = flight_runtime("calibrated_airspeed = 287.0");
        let targets = InitialisationConfig {
            calibrated_airspeed: Some(287.0),
            ..InitialisationConfig::default()
        };
        let mut aircraft = A32nxInitialiser::default();
        observations(&simulator, CALIBRATED_AIRSPEED, [287.0]);
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, duration(100.0))
            .unwrap();
        let baseline = observed - 2.0 * drift;
        // Prepare a saturated controller before replaying acceleration from the log.
        // This tests the component's response; the runtime separately enforces its deadline.
        observations(&simulator, CALIBRATED_AIRSPEED, [baseline]);
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, duration(100.5))
            .unwrap();
        let mut now = 100.5;
        for _ in 0..256 {
            now += 2.0;
            observations(&simulator, CALIBRATED_AIRSPEED, [baseline]);
            simulator.borrow_mut().clear_operations();
            aircraft
                .submit(&mut *simulator.borrow_mut(), targets, duration(now))
                .unwrap();
            if last_write(&simulator.borrow(), THROTTLE_EVENTS[0]) == 16384.0 {
                break;
            }
        }
        assert_eq!(last_write(&simulator.borrow(), THROTTLE_EVENTS[0]), 16384.0);
        observations(&simulator, CALIBRATED_AIRSPEED, [observed]);
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, duration(now + 2.0))
            .unwrap();
        for event in THROTTLE_EVENTS {
            assert!(last_write(&simulator.borrow(), event) <= 16384.0 - minimum_reduction);
        }
    }
}

#[test]
fn overspeed_deceleration_does_not_trigger_the_fast_thrust_increase() {
    let (_, _, simulator) = flight_runtime("calibrated_airspeed = 287.0");
    let targets = InitialisationConfig {
        calibrated_airspeed: Some(287.0),
        ..InitialisationConfig::default()
    };
    let mut aircraft = A32nxInitialiser::default();
    for (now, observed) in [(100.0, 287.0), (100.5, 290.059), (101.513, 289.530)] {
        observations(&simulator, CALIBRATED_AIRSPEED, [observed]);
        simulator.borrow_mut().clear_operations();
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, duration(now))
            .unwrap();
    }
    for event in THROTTLE_EVENTS {
        let increase = last_write(&simulator.borrow(), event) - IDLE_AXIS;
        assert!((0.0..=1024.0).contains(&increase));
    }
    assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
}

#[test]
fn throttle_feedback_settles_with_lag_and_limited_acceleration_at_high_axis_demands() {
    for (initial_speed, engine_lag) in [(287.0, 2.0_f64), (283.3, 2.0), (283.3, 4.0)] {
        let (_, _, simulator) = flight_runtime("calibrated_airspeed = 287.0");
        let targets = InitialisationConfig {
            calibrated_airspeed: Some(287.0),
            ..InitialisationConfig::default()
        };
        let mut aircraft = A32nxInitialiser::default();
        observations(&simulator, CALIBRATED_AIRSPEED, [287.0]);
        aircraft
            .submit(&mut *simulator.borrow_mut(), targets, duration(100.0))
            .unwrap();
        let mut now = 100.0;
        let mut speed = initial_speed;
        let mut peak_speed = speed;
        let mut command = IDLE_AXIS;
        let mut engine_axis = IDLE_AXIS;
        let mut fixed_since = now;
        let mut ready = false;
        for step in 0..2000 {
            let dt = [0.017, 0.043, 0.071, 0.029][step % 4];
            now += dt;
            engine_axis += (command - engine_axis) * (1.0 - (-dt / engine_lag).exp());
            // Synthetic saturation represents the roughly 0.4 kt/s acceleration
            // seen over a wide range of high axis demands. This is not an A32NX model.
            speed += (0.0002 * (engine_axis + 2400.0)).min(0.4) * dt;
            peak_speed = peak_speed.max(speed);
            observations(&simulator, CALIBRATED_AIRSPEED, [speed]);
            simulator.borrow_mut().clear_operations();
            aircraft
                .submit(&mut *simulator.borrow_mut(), targets, duration(now))
                .unwrap();
            ready = aircraft
                .is_ready(&mut *simulator.borrow_mut(), targets)
                .unwrap();
            assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
            if writes_to(&simulator, THROTTLE_EVENTS[0]) {
                let next = last_write(&simulator.borrow(), THROTTLE_EVENTS[0]);
                if command != next {
                    fixed_since = now;
                }
                command = next;
                assert_eq!(command, last_write(&simulator.borrow(), THROTTLE_EVENTS[1]));
            }
            if ready || now >= 160.0 {
                break;
            }
        }
        assert!(
            ready,
            "initial speed {initial_speed}, lag {engine_lag}: speed={speed}, peak={peak_speed}, axis={command}"
        );
        assert!((speed - 287.0).abs() <= 1.0);
        assert!(now - fixed_since >= 3.0);
        assert!(
            peak_speed <= 289.5,
            "large overshoot with lag {engine_lag}: {peak_speed}"
        );
    }
}

#[test]
fn airspeed_feedback_settles_with_engine_lag_and_small_oscillations_in_readback() {
    for (amplitude, period, engine_lag) in
        [(0.07, 2.3, 2.0_f64), (0.05, 1.3, 4.0), (0.04, 4.0, 4.0)]
    {
        let (_, _, simulator) = flight_runtime("calibrated_airspeed = 287.0");
        let targets = InitialisationConfig {
            calibrated_airspeed: Some(287.0),
            ..InitialisationConfig::default()
        };
        let mut aircraft = A32nxInitialiser::default();
        let mut now = 100.0;
        let mut speed = 287.35;
        let mut command = IDLE_AXIS;
        let mut engine_axis = IDLE_AXIS;
        let mut fixed_since = now;
        let mut ready = false;
        // This synthetic plant checks feedback under lag and small speed oscillations;
        // it does not represent a validated A32NX flight-dynamics model.
        for step in 0..2000 {
            let dt = [0.017, 0.043, 0.071, 0.029][step % 4];
            now += dt;
            engine_axis += (command - engine_axis) * (1.0 - (-dt / engine_lag).exp());
            speed += 0.0002 * (engine_axis + 2600.0) * dt;
            let phase = std::f64::consts::TAU * (now - 100.0) / period;
            let noise = amplitude * phase.sin();
            observations(&simulator, CALIBRATED_AIRSPEED, [speed + noise]);
            observations(
                &simulator,
                "A:ROTATION VELOCITY BODY X",
                [0.05 * phase.cos()],
            );
            observations(
                &simulator,
                "A:PLANE PITCH DEGREES",
                [-1.9 + 0.05 * period / std::f64::consts::TAU * phase.sin()],
            );
            simulator.borrow_mut().clear_operations();
            aircraft
                .submit(&mut *simulator.borrow_mut(), targets, duration(now))
                .unwrap();
            ready = aircraft
                .is_ready(&mut *simulator.borrow_mut(), targets)
                .unwrap();
            if writes_to(&simulator, THROTTLE_EVENTS[0]) {
                let previous_command = command;
                command = last_write(&simulator.borrow(), THROTTLE_EVENTS[0]);
                if command != previous_command {
                    fixed_since = now;
                }
                assert_eq!(command, last_write(&simulator.borrow(), THROTTLE_EVENTS[1]));
            }
            assert!((IDLE_AXIS..=16384.0).contains(&command));
            if let Some(path) = aircraft.flight_path {
                assert!(path.normal_law);
                assert_eq!(last_write(&simulator.borrow(), STICK_EVENTS[0]), 0.0);
            }
            if ready || now >= 160.0 {
                break;
            }
        }
        assert!(
            ready,
            "did not settle within the 60-second allowance; amplitude={amplitude}, period={period}, engine_lag={engine_lag}, speed={speed}, axis={command}"
        );
        assert!((speed - 287.0).abs() <= 1.0);
        assert!(now - fixed_since >= 3.0);
    }
}

#[test]
fn raw_axis_feedback_converges_with_engine_lag_irregular_intervals_and_setter_bias() {
    for (equilibrium, setter_bias, setter_delay) in [
        (-6000.0, 0.0, 0.0),
        (4000.0, 0.0, 0.0),
        (12000.0, 0.0, 0.0),
        (4000.0, 12.8, 0.0),
        (4000.0, 12.8, 0.2),
    ] {
        let (_, _, simulator) = flight_runtime("calibrated_airspeed = 250.0");
        let mut aircraft = A32nxInitialiser::default();
        assert!(!speed_step(&mut aircraft, &simulator, 100.0, 230.0));
        let mut now = 100.0;
        let mut speed = last_write(&simulator.borrow(), CALIBRATED_AIRSPEED) + setter_bias;
        let mut setter_echo = speed - setter_bias;
        let mut setter_echo_until = now + setter_delay;
        let mut engine_axis = IDLE_AXIS;
        let mut command = IDLE_AXIS;
        let mut verified_since = None;
        let mut final_ias_at = None;
        let mut ready = false;
        for step in 0..2000 {
            let dt = [0.017, 0.043, 0.071, 0.029][step % 4];
            now += dt;
            engine_axis += (command - engine_axis) * (1.0 - (-dt / 2.0_f64).exp());
            speed += 0.0002 * (engine_axis - equilibrium) * dt;
            let readback = if now < setter_echo_until {
                setter_echo
            } else {
                speed
            };
            ready = speed_step(&mut aircraft, &simulator, now, readback);
            if writes_to(&simulator, THROTTLE_EVENTS[0]) {
                command = last_write(&simulator.borrow(), THROTTLE_EVENTS[0]);
                assert_eq!(command, last_write(&simulator.borrow(), THROTTLE_EVENTS[1]));
            }
            assert!((IDLE_AXIS..=16384.0).contains(&command));
            if writes_to(&simulator, CALIBRATED_AIRSPEED) {
                assert!(final_ias_at.is_none(), "IAS writes resumed after the ramp");
                if now >= 104.0 {
                    final_ias_at = Some(now);
                }
                setter_echo = last_write(&simulator.borrow(), CALIBRATED_AIRSPEED);
                setter_echo_until = now + setter_delay;
                speed = setter_echo + setter_bias;
                verified_since = None;
            } else {
                verified_since.get_or_insert(now);
            }
            if ready || now >= 160.0 {
                break;
            }
        }
        assert!(
            ready,
            "no convergence for raw equilibrium {equilibrium}, setter bias {setter_bias}, readback delay {setter_delay}; final axis {command}"
        );
        assert!(now < 160.0);
        assert!(now - final_ias_at.unwrap() >= 3.0);
        assert!(now - verified_since.unwrap() >= 3.0);
        assert!((speed - 250.0).abs() <= 1.0);
    }
}

#[test]
fn invalid_flight_condition_targets_are_rejected_before_simulator_io() {
    for field in ["pressure_altitude", "calibrated_airspeed"] {
        for value in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut simulator = FakeSimulator::new(Duration::ZERO);
            let error = if field == "pressure_altitude" {
                crate::altitude::A32nxPressureAltitude::prepare(
                    &mut simulator,
                    value,
                    Duration::ZERO,
                    None,
                )
                .unwrap_err()
            } else {
                crate::airspeed::A32nxCalibratedAirspeed::prepare(
                    &mut simulator,
                    value,
                    Duration::ZERO,
                    None,
                    false,
                )
                .unwrap_err()
            };
            assert!(
                matches!(error, InitialisationError::InvalidFlightConditionTarget { field: actual, .. } if actual == field)
            );
            assert!(simulator.operations.is_empty());
        }
    }
}

#[test]
fn invalid_airspeed_correction_fails_before_submitting_any_new_commands() {
    let (fixture, mut runtime, simulator) = flight_runtime("calibrated_airspeed = 250.0");
    observations(&simulator, CALIBRATED_AIRSPEED, [0.0, 1000.0, 1000.0]);
    runtime.pre_update().unwrap();
    runtime.pre_update().unwrap();
    simulator.borrow_mut().time = duration(100.5);
    runtime.pre_update().unwrap();
    simulator.borrow_mut().time = duration(101.0);
    simulator.borrow_mut().clear_operations();
    let error = runtime.pre_update().unwrap_err();
    assert!(matches!(
        error.downcast_ref::<InitialisationError>(),
        Some(InitialisationError::InvalidAirspeedCorrection {
            previous_demand: 0.0,
            observed: 1000.0,
            target: 5.0,
            value: -5.0,
        })
    ));
    assert_eq!(
        simulator.borrow().operations.last(),
        Some(&Operation::Write {
            variable: ARMED_VARIABLE.to_owned(),
            value: 0.0
        })
    );
    assert_no_replay_writes(&simulator.borrow());
    assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
    for event in THROTTLE_EVENTS {
        assert!(!writes_to(&simulator, event));
    }
    assert_no_telemetry(&fixture);
}

#[test]
fn omitting_speed_performs_no_throttle_io_and_csv_throttle_takes_over_normally() {
    let (_, _, simulator) = flight_runtime("pressure_altitude = 10000.0");
    simulator.borrow_mut().clear_operations();
    A32nxInitialiser::default()
        .submit(
            &mut *simulator.borrow_mut(),
            InitialisationConfig {
                pressure_altitude: Some(10000.0),
                ..InitialisationConfig::default()
            },
            Duration::ZERO,
        )
        .unwrap();
    for event in THROTTLE_EVENTS {
        assert!(!writes_to(&simulator, event));
    }
    assert!(!simulator.borrow().operations.iter().any(
        |op| matches!(op, Operation::Read { variable, .. } if variable == CALIBRATED_AIRSPEED)
    ));

    let (fixture, mut runtime, simulator) = flight_runtime("calibrated_airspeed = 250.0");
    let config = fs::read_to_string(&fixture.config_path)
        .unwrap()
        .replace("K:AXIS_ELEVATOR_SET", THROTTLE_EVENTS[0])
        .replace(
            "source_range = [-100.0, 100.0]",
            "source_range = [-38.0, 85.5]",
        )
        .replace(
            "simulator_range = [-1.0, 1.0]",
            "simulator_range = [-16383.0, 16384.0]",
        );
    fs::write(&fixture.config_path, config).unwrap();
    fs::write(
        fixture.directory.join("scenario.csv"),
        SCENARIO.replace("1,100", "1,20"),
    )
    .unwrap();
    runtime.pre_update().unwrap();
    settle_250_knots(&fixture, &mut runtime, &simulator, 100.5);
    let settled = last_write(&simulator.borrow(), THROTTLE_EVENTS[0]);
    simulator.borrow_mut().clear_operations();
    runtime.pre_update().unwrap();
    let csv_command = -16383.0 + 38.0 * 32767.0 / 123.5;
    assert!((last_write(&simulator.borrow(), THROTTLE_EVENTS[0]) - csv_command).abs() < 1e-9);
    assert!((settled - csv_command).abs() > 1.0);
    assert!(!writes_to(&simulator, THROTTLE_EVENTS[1]));
    assert!(!writes_to(&simulator, CALIBRATED_AIRSPEED));
    runtime.stop().unwrap();
}
