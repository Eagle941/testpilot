use std::fs;
use std::hint::black_box;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime};

mod support;

use criterion::{Criterion, criterion_group, criterion_main};
use support::{TempDirectory, full_frame::benchmark_full_frame};

use testpilot::cursor::{Frame, Scenario};
use testpilot::playback::{AffineRange, Sample};
use testpilot::recording::TelemetryRecorder;
use testpilot::{config::InjectionConfig, config::ReplayConfig};

fn make_scenario_csv(path: &Path, rows: usize) {
    let mut file = String::new();
    file.push_str("input.time,input.value\n");
    for index in 0..rows {
        let time = (index as f64) * 0.001;
        let value = (index % 201) as f64 - 100.0;
        file.push_str(&format!("{time},{value}\n"));
    }
    fs::write(path, file).expect("write benchmark scenario file");
}

fn build_scenario(rows: usize) -> (Scenario, TempDirectory) {
    let directory = TempDirectory::new();
    let scenario_path = directory.0.join("scenario.csv");
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
        b.iter_custom(|iterations| {
            let mut total = Duration::ZERO;
            for _ in 0..iterations {
                let (mut scenario, directory) = build_scenario(10_000);
                let started = Instant::now();
                for index in 0..128_u64 {
                    let elapsed = Duration::from_millis(2 * index);
                    scenario
                        .advance(black_box(elapsed))
                        .expect("advance should succeed");
                }
                black_box(scenario.signal_count());
                total += started.elapsed();
                drop(scenario);
                drop(directory);
            }
            total
        });
    });
}

/// Measures telemetry frame emission in the hot serialization path.
///
/// The benchmark repeatedly calls `TelemetryRecorder::write_frame` with pre-borrowed
/// recording and injected values, covering per-frame row serialization and CSV write
/// behavior.
fn benchmark_telemetry_writer(c: &mut Criterion) {
    c.bench_function("telemetry write_frame", |b| {
        let recording_values = [Some(0.25)];
        let injected_values = [0.5];
        b.iter_custom(|iterations| {
            let mut remaining = iterations;
            let mut total = Duration::ZERO;
            while remaining != 0 {
                let directory = TempDirectory::new();
                let mut recorder = make_recording_sink(&directory.0);
                let frames = remaining.min(256);
                let started = Instant::now();
                for index in 0..frames {
                    recorder
                        .write_frame(
                            black_box(Duration::from_nanos((index + 1) * 16_666_667)),
                            black_box(&recording_values[..]),
                            black_box(&injected_values),
                        )
                        .expect("telemetry frame should be written");
                }
                total += started.elapsed();
                recorder.flush().expect("flush telemetry writer");
                drop(recorder);
                drop(directory);
                remaining -= frames;
            }
            total
        });
    });
}

criterion_group!(
    benches,
    benchmark_frame_interpolation,
    benchmark_scenario_advance,
    benchmark_telemetry_writer,
    benchmark_full_frame
);
criterion_main!(benches);
