use std::cell::Cell;
use std::fs;
use std::hint::black_box;
use std::io::{BufWriter, Write};
use std::rc::Rc;
use std::time::{Duration, Instant};

use criterion::{Criterion, Throughput};
use testpilot::bench_support::{GaugeRuntime, SimulatorAdapter, new_runtime};
use testpilot::error::SimulatorError;

use super::TempDirectory;

const BATCH_FRAMES: u64 = 256;
const FRAME_STEP: Duration = Duration::from_nanos(16_666_667);
const INPUTS: [&str; 2] = ["K:AXIS_ELEVATOR_SET", "K:AXIS_AILERONS_SET"];
const OUTPUTS: [&str; 4] = [
    "A:PLANE PITCH DEGREES",
    "A:PLANE BANK DEGREES",
    "A:ELEVATOR POSITION",
    "A:AILERON POSITION",
];
const NAMES: [&str; 6] = [
    "pitch",
    "roll",
    "elevator_position",
    "aileron_position",
    "sidestick_pitch_position",
    "sidestick_roll_position",
];

#[derive(Clone, Copy)]
pub enum Workload {
    EveryFrame,
    LateFrames,
    Limited,
    NoRecordings,
}

pub const WORKLOADS: [(&str, Workload); 4] = [
    ("every_frame", Workload::EveryFrame),
    ("late_frames", Workload::LateFrames),
    ("recordings_30hz", Workload::Limited),
    ("without_recordings", Workload::NoRecordings),
];

impl Workload {
    fn elapsed_after(self, frames: u64) -> Duration {
        let late = if matches!(self, Self::LateFrames) {
            frames / 32
        } else {
            0
        };
        let nanos = u128::from(frames) * FRAME_STEP.as_nanos()
            + u128::from(late) * (Duration::from_millis(250) - FRAME_STEP).as_nanos();
        Duration::new(
            (nanos / 1_000_000_000)
                .try_into()
                .expect("supported scenario duration"),
            (nanos % 1_000_000_000) as u32,
        )
    }

    fn recordings(self) -> usize {
        if matches!(self, Self::NoRecordings) {
            0
        } else {
            4
        }
    }

    fn step(self, frame: u64) -> Duration {
        if matches!(self, Self::LateFrames) && frame.is_multiple_of(32) {
            Duration::from_millis(250)
        } else {
            FRAME_STEP
        }
    }

    fn records_frame(self, frame: u64) -> bool {
        // At this fixed frame interval, the 30 Hz case is due every second frame.
        !matches!(self, Self::Limited) || frame.is_multiple_of(2)
    }
}

/// Fixed-size observable state; no growing operation log in the timed path.
#[derive(Default)]
struct State {
    time: Cell<Duration>,
    armed: Cell<f64>,
    clock_reads: Cell<u64>,
    arm_reads: Cell<u64>,
    writes: Cell<[u64; 2]>,
    last_injected: Cell<[f64; 2]>,
    reads: Cell<[u64; 4]>,
    validations: Cell<u64>,
}

struct FakeSimulator(Rc<State>);

fn increment<const N: usize>(counts: &Cell<[u64; N]>, index: usize) {
    let mut values = counts.get();
    values[index] += 1;
    counts.set(values);
}

fn responses(seconds: f64) -> [f64; 4] {
    [seconds * 0.1, seconds * -0.2, 0.25, -0.5]
}

fn expected_injections(seconds: f64) -> [f64; 2] {
    [seconds / 10.0 - 0.5, 0.5 - seconds / 20.0].map(|value| value * 16_383.5 + 0.5)
}

fn input_values(seconds: f64) -> [f64; 2] {
    // Triangular ramps keep sustained runs in range. Both sampling grids include
    // every eight-second turning point. Before eight seconds these match the
    // original short-batch ramps exactly apart from floating-point rounding.
    let distance = ((seconds % 16.0) - 8.0).abs();
    [0.3 - distance / 10.0, 0.1 + distance / 20.0]
}

impl SimulatorAdapter for FakeSimulator {
    fn simulation_time(&self) -> Result<Duration, SimulatorError> {
        self.0.clock_reads.set(self.0.clock_reads.get() + 1);
        Ok(black_box(self.0.time.get()))
    }

    fn write(&mut self, variable: &str, value: f64) -> Result<(), SimulatorError> {
        if variable == "L:REPLAYER_ARMED" {
            self.0.armed.set(value);
        } else {
            let index = INPUTS
                .iter()
                .position(|name| *name == variable)
                .expect("known injection destination");
            increment(&self.0.writes, index);
            let mut values = self.0.last_injected.get();
            values[index] = black_box(value);
            self.0.last_injected.set(values);
        }
        Ok(())
    }

    fn validate_read(&mut self, variable: &str, unit: Option<&str>) -> Result<(), SimulatorError> {
        let index = OUTPUTS
            .iter()
            .position(|name| *name == variable)
            .expect("known recording source");
        assert_eq!(unit, Some(if index < 2 { "degrees" } else { "Position" }));
        self.0.validations.set(self.0.validations.get() + 1);
        Ok(())
    }

    fn read(&mut self, variable: &str, _unit: Option<&str>) -> Result<f64, SimulatorError> {
        if variable == "L:REPLAYER_ARMED" {
            self.0.arm_reads.set(self.0.arm_reads.get() + 1);
            return Ok(black_box(self.0.armed.get()));
        }
        let index = OUTPUTS
            .iter()
            .position(|name| *name == variable)
            .expect("known recording source");
        increment(&self.0.reads, index);
        Ok(black_box(responses(self.0.time.get().as_secs_f64())[index]))
    }
}

fn config(workload: Workload) -> String {
    let mut text = "format_version = 1\ninput_file = \"scenario.csv\"\n".to_owned();
    for (index, variable) in INPUTS.iter().enumerate() {
        text.push_str(&format!(
            "\n[inject.{index}]\nname = \"{}\"\nvariable = \"{variable}\"\nsource_range = [-1.0, 1.0]\nsimulator_range = [-16383.0, 16384.0]\n",
            NAMES[index + 4]
        ));
    }
    for index in 0..workload.recordings() {
        let unit = if index < 2 { "degrees" } else { "Position" };
        text.push_str(&format!(
            "\n[record.{index}]\nname = \"{}\"\nvariable = \"{}\"\nunit = \"{unit}\"\n",
            NAMES[index], OUTPUTS[index]
        ));
        if matches!(workload, Workload::Limited) {
            text.push_str("max_sampling_rate = 30.0\n");
        }
    }
    text
}

fn scenario_csv() -> String {
    let mut csv = format!(
        "{}.time,{}.value,{}.time,{}.value\n",
        NAMES[4], NAMES[4], NAMES[5], NAMES[5]
    );
    let mut pitch_ms = 0_u64;
    let mut roll_ms = 0_u64;
    // Both independent series extend beyond the longest 256-frame workload.
    for row in 0..1_500 {
        let pitch_time = pitch_ms as f64 / 1000.0;
        let roll_time = roll_ms as f64 / 1000.0;
        csv.push_str(&format!(
            "{pitch_time},{},{roll_time},{}\n",
            pitch_time / 10.0 - 0.5,
            0.5 - roll_time / 20.0
        ));
        pitch_ms += if row % 2 == 0 { 5 } else { 11 };
        roll_ms += if row % 2 == 0 { 7 } else { 13 };
    }
    csv
}

/// One armed replay, owning its files for the entire warm-up/profile interval.
/// Field order closes the runtime's handles before deleting the directory on unwind.
pub struct SustainedRun {
    runtime: GaugeRuntime<FakeSimulator>,
    state: Rc<State>,
    workload: Workload,
    frame: u64,
    budget: u64,
    directory: TempDirectory,
}

impl SustainedRun {
    pub fn new(workload: Workload, frames: u64) -> Self {
        assert!(frames > 0);
        let directory = TempDirectory::new();
        let config_path = directory.0.join("replayer_config.toml");
        fs::write(&config_path, config(workload)).expect("write configuration");
        let mut writer = BufWriter::new(
            fs::File::create(directory.0.join("scenario.csv")).expect("create scenario"),
        );
        writeln!(
            writer,
            "{}.time,{}.value,{}.time,{}.value",
            NAMES[4], NAMES[4], NAMES[5], NAMES[5]
        )
        .expect("write header");
        let end_ms = workload.elapsed_after(frames).as_millis() + 32;
        let mut pitch_ms = 0_u128;
        let mut roll_ms = 0_u128;
        let mut even = true;
        loop {
            let pitch_time = pitch_ms as f64 / 1000.0;
            let roll_time = roll_ms as f64 / 1000.0;
            writeln!(
                writer,
                "{pitch_time},{},{roll_time},{}",
                input_values(pitch_time)[0],
                input_values(roll_time)[1]
            )
            .expect("write sample");
            if pitch_ms > end_ms {
                break;
            }
            pitch_ms += if even { 5 } else { 11 };
            roll_ms += if even { 7 } else { 13 };
            even = !even;
        }
        writer.flush().expect("flush scenario");
        drop(writer);
        let state = Rc::new(State::default());
        let mut runtime =
            new_runtime(config_path, FakeSimulator(Rc::clone(&state))).expect("runtime");
        state.armed.set(1.0);
        runtime.pre_update().expect("arm at time zero");
        Self {
            runtime,
            state,
            workload,
            frame: 0,
            budget: frames,
            directory,
        }
    }

    pub fn advance(&mut self, frames: u64) {
        let last = self
            .frame
            .checked_add(frames)
            .expect("frame count overflow");
        assert!(
            last <= self.budget,
            "fixture must outlast the measured replay"
        );
        for frame in self.frame + 1..=last {
            self.state
                .time
                .set(self.state.time.get() + self.workload.step(frame));
            self.runtime.pre_update().expect("sustained frame");
        }
        self.frame = last;
    }

    pub fn finish(mut self) {
        assert_eq!(self.state.writes.get(), [self.frame + 1; 2]);
        assert_eq!(self.state.clock_reads.get(), self.frame + 1);
        assert_eq!(self.state.arm_reads.get(), self.frame + 1);
        let samples = if matches!(self.workload, Workload::Limited) {
            self.frame / 2 + 1
        } else {
            self.frame + 1
        };
        assert_eq!(
            self.state.reads.get(),
            if self.workload.recordings() == 0 {
                [0; 4]
            } else {
                [samples; 4]
            }
        );
        for (actual, expected) in self.state.last_injected.get().into_iter().zip(
            input_values(self.state.time.get().as_secs_f64()).map(|value| value * 16_383.5 + 0.5),
        ) {
            assert_close(actual, expected);
        }
        self.runtime
            .stop()
            .expect("stop and flush sustained replay");
        assert_eq!(self.state.armed.get(), 0.0);
        drop(self.runtime);
        drop(self.directory);
    }
}

/// Stable sampling-profiler boundary containing only sustained frame execution.
#[inline(never)]
pub fn profile_frames(run: &mut SustainedRun, frames: u64) {
    run.advance(frames);
}

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 1e-8,
        "expected {expected}, got {actual}"
    );
}

fn verify_telemetry(directory: &TempDirectory, workload: Workload, frames: u64) {
    let path = fs::read_dir(&directory.0)
        .expect("list telemetry")
        .map(|entry| entry.expect("read directory entry").path())
        .find(|path| {
            path.file_name()
                .expect("filename")
                .to_string_lossy()
                .starts_with("telemetry_")
        })
        .expect("telemetry file exists");
    let mut reader = csv::Reader::from_path(path).expect("read telemetry");
    let names = if workload.recordings() == 0 {
        &NAMES[4..]
    } else {
        &NAMES[..]
    };
    let expected_header: Vec<_> = names
        .iter()
        .flat_map(|name| [format!("{name}.time"), format!("{name}.value")])
        .collect();
    assert_eq!(
        reader
            .headers()
            .expect("CSV header")
            .iter()
            .collect::<Vec<_>>(),
        expected_header
    );
    let mut rows = reader.records();
    let mut elapsed = Duration::ZERO;
    for frame in 0..=frames {
        if frame != 0 {
            elapsed += workload.step(frame);
        }
        if !workload.records_frame(frame) {
            continue;
        }
        let row = rows
            .next()
            .expect("expected telemetry row")
            .expect("valid CSV row");
        let seconds = elapsed.as_secs_f64();
        let expected: Vec<_> = responses(seconds)
            .into_iter()
            .take(workload.recordings())
            .chain(expected_injections(seconds))
            .collect();
        assert_eq!(row.len(), expected.len() * 2);
        for (index, value) in expected.into_iter().enumerate() {
            assert_close(row[index * 2].parse().expect("numeric time"), seconds);
            assert_close(row[index * 2 + 1].parse().expect("numeric value"), value);
        }
    }
    assert!(rows.next().is_none(), "unexpected extra telemetry row");
}

/// Returns only timed steady-state work, excluding creation, arming and teardown.
fn run_batch(
    workload: Workload,
    frames: u64,
    config: &str,
    scenario: &str,
    verify_csv: bool,
) -> Duration {
    assert!((1..=BATCH_FRAMES).contains(&frames));
    let directory = TempDirectory::new();
    let config_path = directory.0.join("replayer_config.toml");
    fs::write(&config_path, config).expect("write configuration");
    fs::write(directory.0.join("scenario.csv"), scenario).expect("write scenario");
    let state = Rc::new(State::default());
    let mut runtime = new_runtime(config_path, FakeSimulator(Rc::clone(&state))).expect("runtime");
    state.armed.set(1.0);
    runtime
        .pre_update()
        .expect("arm and initialize at time zero");

    let started = Instant::now();
    for frame in 1..=frames {
        state.time.set(state.time.get() + workload.step(frame));
        runtime.pre_update().expect("running frame");
    }
    let duration = started.elapsed();

    // Catch premature completion or accidental idle measurements in every batch.
    assert_eq!(state.writes.get(), [frames + 1; 2]);
    assert_eq!(state.clock_reads.get(), frames + 1);
    assert_eq!(state.arm_reads.get(), frames + 1);
    assert_eq!(state.validations.get(), workload.recordings() as u64);
    let samples = if matches!(workload, Workload::Limited) {
        frames / 2 + 1
    } else {
        frames + 1
    };
    assert_eq!(
        state.reads.get(),
        if workload.recordings() == 0 {
            [0; 4]
        } else {
            [samples; 4]
        }
    );
    for (actual, expected) in state
        .last_injected
        .get()
        .into_iter()
        .zip(expected_injections(state.time.get().as_secs_f64()))
    {
        assert_close(actual, expected);
    }
    black_box(state.last_injected.get());
    runtime.stop().expect("flush and stop");
    assert_eq!(state.armed.get(), 0.0);
    drop(runtime);
    if verify_csv {
        verify_telemetry(&directory, workload, frames);
    }
    duration
}

pub fn benchmark_full_frame(c: &mut Criterion) {
    let scenario = scenario_csv();
    let mut group = c.benchmark_group("full_frame");
    group.throughput(Throughput::Elements(1));
    group.sample_size(20);
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(3));
    for (name, workload) in WORKLOADS {
        let config = config(workload);
        // Full CSV verification is outside measurement and covers partial batches too.
        for frames in [1, 3, BATCH_FRAMES] {
            run_batch(workload, frames, &config, &scenario, true);
        }
        group.bench_function(name, |b| {
            b.iter_custom(|iterations| {
                let mut remaining = iterations;
                let mut elapsed = Duration::ZERO;
                while remaining != 0 {
                    let frames = remaining.min(BATCH_FRAMES);
                    elapsed += run_batch(workload, frames, &config, &scenario, false);
                    remaining -= frames;
                }
                // Criterion divides this total by iterations, giving time per frame.
                elapsed
            });
        });
    }
    group.finish();
}
