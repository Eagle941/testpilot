use std::hint::black_box;
use std::time::Duration;

use criterion::{Criterion, Throughput};
use testpilot::bench_support::{
    ReadCommandCache, cached_clock_code, format_write_code, prepare_clock_code, prepare_read_code,
    prepare_write_code,
};

/// Current adapter preparation: static clock/arming, cached recordings, dynamic writes.
/// Warm once before timing so configured command construction stays outside the loop.
pub fn prepare_cached_frame(cache: &mut ReadCommandCache, scratch: &mut String, value: f64) {
    black_box(cached_clock_code());
    black_box(
        cache
            .get(black_box("L:REPLAYER_ARMED"), None)
            .expect("arming command"),
    );
    for variable in ["K:AXIS_ELEVATOR_SET", "K:AXIS_AILERONS_SET"] {
        black_box(
            prepare_write_code(scratch, black_box(variable), black_box(value))
                .expect("axis command"),
        );
    }
    for (variable, unit) in [
        ("A:PLANE PITCH DEGREES", "degrees"),
        ("A:PLANE BANK DEGREES", "degrees"),
        ("A:ELEVATOR POSITION", "Position"),
        ("A:AILERON POSITION", "Position"),
    ] {
        black_box(
            cache
                .get(black_box(variable), Some(black_box(unit)))
                .expect("recording command"),
        );
    }
}

/// One normal frame's clock/arming reads, two injections, and four recordings.
/// Includes command destruction, but never executes a simulator API call.
pub fn prepare_frame(scratch: &mut String, value: f64) {
    black_box(prepare_clock_code().expect("clock command"));
    black_box(
        prepare_read_code(scratch, black_box("L:REPLAYER_ARMED"), None).expect("arming command"),
    );
    for variable in ["K:AXIS_ELEVATOR_SET", "K:AXIS_AILERONS_SET"] {
        black_box(
            prepare_write_code(scratch, black_box(variable), black_box(value))
                .expect("axis command"),
        );
    }
    for (variable, unit) in [
        ("A:PLANE PITCH DEGREES", "degrees"),
        ("A:PLANE BANK DEGREES", "degrees"),
        ("A:ELEVATOR POSITION", "Position"),
        ("A:AILERON POSITION", "Position"),
    ] {
        black_box(
            prepare_read_code(scratch, black_box(variable), Some(black_box(unit)))
                .expect("recording command"),
        );
    }
}

pub fn benchmark_boundary(c: &mut Criterion) {
    let mut group = c.benchmark_group("boundary");
    group.sample_size(20);
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(3));
    group.throughput(Throughput::Elements(1));
    let mut scratch = String::with_capacity(128);
    // Warm the scratch capacity before any measurement.
    prepare_frame(&mut scratch, -12345.6789);
    group.bench_function("format_axis", |b| {
        b.iter(|| {
            format_write_code(
                &mut scratch,
                black_box("K:AXIS_ELEVATOR_SET"),
                black_box(-12345.6789),
            )
            .expect("format axis");
            black_box(scratch.as_str());
        })
    });
    group.bench_function("prepare_axis", |b| {
        b.iter(|| {
            black_box(
                prepare_write_code(
                    &mut scratch,
                    black_box("K:AXIS_ELEVATOR_SET"),
                    black_box(-12345.6789),
                )
                .expect("prepare axis"),
            );
        })
    });
    group.bench_function("prepare_aircraft_read", |b| {
        b.iter(|| {
            black_box(
                prepare_read_code(
                    &mut scratch,
                    black_box("A:PLANE PITCH DEGREES"),
                    Some(black_box("degrees")),
                )
                .expect("prepare read"),
            );
        })
    });
    group.bench_function("prepare_armed_read", |b| {
        b.iter(|| {
            black_box(
                prepare_read_code(&mut scratch, black_box("L:REPLAYER_ARMED"), None)
                    .expect("prepare arming"),
            );
        })
    });
    group.bench_function("prepare_clock_read", |b| {
        b.iter(|| {
            black_box(prepare_clock_code().expect("prepare clock"));
        })
    });
    group.bench_function("prepare_frame", |b| {
        b.iter(|| prepare_frame(&mut scratch, black_box(-12345.6789)))
    });
    let mut cache = ReadCommandCache::new();
    prepare_cached_frame(&mut cache, &mut scratch, -12345.6789);
    group.bench_function("prepare_cached_frame", |b| {
        b.iter(|| prepare_cached_frame(&mut cache, &mut scratch, black_box(-12345.6789)))
    });
    group.finish();
}
