use std::fs;
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

extern crate testpilot;

use criterion::{Criterion, black_box, criterion_group, criterion_main};

use testpilot::cursor::{Frame, Scenario};
use testpilot::playback::{AffineRange, Sample};
use testpilot::recording::TelemetryRecorder;
use testpilot::{config::InjectionConfig, config::ReplayConfig};

fn unique_tmp_dir() -> PathBuf {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |time| time.as_nanos() as u64);
    let pid = std::process::id();
    let mut path = std::env::temp_dir();
    path.push(format!("replay-bench-{pid}-{nanos}"));
    fs::create_dir_all(&path).expect("create temporary benchmark directory");
    path
}

fn make_scenario_csv(path: &Path, rows: usize) {
    let mut file = String::new();
    file.push_str("input.time,input.value\n");
    for index in 0..rows {
        let time = (index as f64) * 0.001;
        let value = (index as f64) * 1.5;
        file.push_str(&format!("{time},{value}\n"));
    }
    fs::write(path, file).expect("write benchmark scenario file");
}

fn build_scenario(rows: usize) -> (Scenario, PathBuf) {
    let directory = unique_tmp_dir();
    let scenario_path = directory.join("scenario.csv");
    make_scenario_csv(&scenario_path, rows);
    let config = ReplayConfig {
        input_file: PathBuf::from("scenario.csv"),
        inject: vec![InjectionConfig {
            name: "input".to_string(),
            variable: "K:TEST_VAR".to_string(),
            source_range: [-1000.0, 1000.0],
            simulator_range: [-1.0, 1.0],
        }],
        record: Vec::new(),
    };
    let scenario = Scenario::new(&scenario_path, &config).expect("create benchmark scenario");
    (scenario, directory)
}

fn make_recording_sink(path: &Path) -> TelemetryRecorder {
    let path = path.join("output");
    fs::create_dir_all(&path).expect("create benchmark output directory");
    TelemetryRecorder::new(
        path,
        &["pitch".to_string()],
        &["input".to_string()],
        SystemTime::now(),
    )
    .expect("create benchmark telemetry recorder")
}

/// Measures interpolation and conversion on a single `Frame` interval.
///
/// The benchmark exercises:
/// - `Frame::value_at` across varying elapsed times in one segment
/// - `AffineRange::convert` for each sample
///
/// This captures the same compute path used by the continuous-input replay injector.
fn benchmark_frame_interpolation(c: &mut Criterion) {
    let start = Sample::new(Duration::ZERO, -100.0).expect("valid sample");
    let end = Sample::new(Duration::from_secs(1), 100.0).expect("valid sample");
    let conversion = AffineRange::new([-100.0, 100.0], [-1.0, 1.0]).expect("valid affine range");
    let frame = Frame {
        signal: "input",
        variable: "K:TEST",
        previous: start,
        next: Some(end),
        conversion,
    };
    c.bench_function("frame interpolation and affine conversion", |b| {
        let mut elapsed = Duration::ZERO;
        b.iter(|| {
            let value = frame
                .value_at(black_box(elapsed))
                .expect("interpolation should succeed");
            black_box(
                conversion
                    .convert(black_box(value))
                    .expect("conversion should succeed"),
            );
            elapsed = elapsed.saturating_add(Duration::from_nanos(13));
            if elapsed >= Duration::from_secs(1) {
                elapsed = Duration::ZERO;
            }
        });
    });
}

/// Measures scenario cursor advancement under monotonic elapsed updates.
///
/// The benchmark repeatedly calls `Scenario::advance` with increasing elapsed values
/// to exercise segment bracketing and interval-crossing behavior in scheduler state.
fn benchmark_scenario_advance(c: &mut Criterion) {
    c.bench_function("scenario.advance with increasing elapsed", |b| {
        b.iter_batched(
            || build_scenario(10_000),
            |(mut scenario, _)| {
                for index in 0..128_u64 {
                    let elapsed = Duration::from_millis(2 * index);
                    scenario
                        .advance(black_box(elapsed))
                        .expect("advance should succeed");
                }
                black_box(scenario.signal_count())
            },
            criterion::BatchSize::PerIteration,
        );
    });
}

/// Measures telemetry frame emission in the hot serialization path.
///
/// The benchmark repeatedly calls `TelemetryRecorder::write_frame` with pre-borrowed
/// recording and injected values, covering per-frame row serialization and CSV write
/// behavior.
fn benchmark_telemetry_writer(c: &mut Criterion) {
    c.bench_function("telemetry write_frame", |b| {
        let directory = unique_tmp_dir();
        let mut recorder = make_recording_sink(&directory);
        let recording_values = [Some(0.25)];
        let injected_values = [0.5];
        b.iter(|| {
            recorder
                .write_frame(
                    black_box(Duration::from_secs_f64(0.016)),
                    black_box(&recording_values[..]),
                    black_box(&injected_values),
                )
                .expect("telemetry frame should be written");
        });
        recorder.flush().expect("flush telemetry writer");
        fs::remove_dir_all(&directory).expect("remove benchmark directory");
    });
}

criterion_group!(
    benches,
    benchmark_frame_interpolation,
    benchmark_scenario_advance,
    benchmark_telemetry_writer
);
criterion_main!(benches);
