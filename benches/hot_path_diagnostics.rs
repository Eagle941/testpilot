//! Allocation reports and a sustained CPU profiling loop, separate from Criterion.

#[cfg(feature = "bench-allocations")]
use std::fs::File;
use std::io::{self, Write};
use std::path::PathBuf;
use std::time::Instant;

#[path = "support/allocations.rs"]
#[cfg(feature = "bench-allocations")]
mod allocations;
// The shared modules also expose Criterion entry points used by hot_path.rs.
#[allow(dead_code)]
mod support;

#[cfg(feature = "bench-allocations")]
use allocations::{Counts, Measurement};
#[cfg(feature = "bench-allocations")]
use support::boundary::{prepare_cached_frame, prepare_frame};
use support::full_frame::{SustainedRun, WORKLOADS, Workload, profile_frames};

// This allocator is never linked into the normal Criterion or WASM binaries.
#[global_allocator]
#[cfg(feature = "bench-allocations")]
static ALLOCATOR: allocations::CountingAllocator = allocations::CountingAllocator;

const HELP: &str = "Usage: hot_path_diagnostics <allocations|profile> [options]
  --frames N       Measured frames (allocations: 4096; profile: 1000000)
  --warmup N       Untimed, uncounted warm-up frames (default: 1024)
  --workload NAME  every_frame, late_frames, recordings_30hz, without_recordings
                  allocations defaults to all; profile defaults to every_frame
  --output PATH    Allocation CSV path (default: stdout)
  --test           Smoke-test this diagnostics build with small fixtures
  --help           Show this help

Profile mode creates one disk-backed scenario and replay, warms it, then emits
PROFILE_BEGIN/PROFILE_END around profile_frames. Memory is bounded; temporary
disk use scales with the requested frame count. Files are removed on completion.
Allocation counts cover only the calling thread during measured frames; no timing
results are reported in allocation mode. Build allocations with bench-allocations;
build profile with bench-support only, to omit the allocator instrumentation.";

struct Options {
    mode: String,
    frames: u64,
    warmup: u64,
    workload: Option<(&'static str, Workload)>,
    output: Option<PathBuf>,
}

fn parse(args: &[String]) -> Result<Options, String> {
    let mode = args
        .first()
        .ok_or("expected allocations or profile")?
        .clone();
    if mode != "allocations" && mode != "profile" {
        return Err(format!("unknown mode: {mode}"));
    }
    let mut options = Options {
        frames: if mode == "allocations" {
            4096
        } else {
            1_000_000
        },
        mode,
        warmup: 1024,
        workload: None,
        output: None,
    };
    let mut remaining = args[1..].iter();
    while let Some(flag) = remaining.next() {
        let value = remaining
            .next()
            .ok_or_else(|| format!("missing value for {flag}"))?;
        match flag.as_str() {
            "--frames" => {
                options.frames = value
                    .parse()
                    .map_err(|_| "frames must be an unsigned integer")?
            }
            "--warmup" => {
                options.warmup = value
                    .parse()
                    .map_err(|_| "warmup must be an unsigned integer")?
            }
            "--workload" => {
                options.workload = Some(
                    WORKLOADS
                        .into_iter()
                        .find(|(name, _)| name == value)
                        .ok_or_else(|| format!("unknown workload: {value}"))?,
                )
            }
            "--output" => options.output = Some(PathBuf::from(value)),
            _ => return Err(format!("unknown option: {flag}")),
        }
    }
    if options.frames == 0 || options.frames.checked_add(options.warmup).is_none() {
        return Err("frames must be positive and frames + warmup must fit in u64".into());
    }
    if options.mode == "profile" && options.output.is_some() {
        return Err("--output is only used by allocations".into());
    }
    Ok(options)
}

#[cfg(feature = "bench-allocations")]
fn report(output: &mut dyn Write, name: &str, frames: u64, counts: Counts) {
    writeln!(
        output,
        "{name},{frames},{},{},{},{},{:.6},{:.6}",
        counts.allocations,
        counts.reallocations,
        counts.deallocations,
        counts.requested_bytes,
        (counts.allocations + counts.reallocations) as f64 / frames as f64,
        counts.requested_bytes as f64 / frames as f64
    )
    .expect("write allocation report");
}

#[cfg(feature = "bench-allocations")]
fn measure_allocations(options: &Options) {
    allocations::self_check();
    let mut output: Box<dyn Write> = if let Some(path) = &options.output {
        Box::new(File::create(path).expect("create allocation report"))
    } else {
        Box::new(io::stdout())
    };
    writeln!(output, "workload,frames,allocations,reallocations,deallocations,requested_bytes,allocation_calls_per_frame,requested_bytes_per_frame").expect("write header");
    for (name, workload) in WORKLOADS {
        if options
            .workload
            .is_some_and(|(selected, _)| selected != name)
        {
            continue;
        }
        let mut run = SustainedRun::new(workload, options.frames + options.warmup);
        run.advance(options.warmup);
        let measurement = Measurement::begin();
        profile_frames(&mut run, options.frames);
        let counts = measurement.finish();
        run.finish();
        report(&mut output, name, options.frames, counts);
    }
    let mut scratch = String::with_capacity(128);
    for _ in 0..options.warmup.max(1) {
        prepare_frame(&mut scratch, -12345.6789);
    }
    let measurement = Measurement::begin();
    for _ in 0..options.frames {
        prepare_frame(&mut scratch, -12345.6789);
    }
    let counts = measurement.finish();
    report(
        &mut output,
        "boundary_prepare_frame",
        options.frames,
        counts,
    );
    let mut cache = testpilot::bench_support::ReadCommandCache::new();
    for _ in 0..options.warmup.max(1) {
        prepare_cached_frame(&mut cache, &mut scratch, -12345.6789);
    }
    let measurement = Measurement::begin();
    for _ in 0..options.frames {
        prepare_cached_frame(&mut cache, &mut scratch, -12345.6789);
    }
    let counts = measurement.finish();
    report(
        &mut output,
        "boundary_prepare_cached_frame",
        options.frames,
        counts,
    );
    output.flush().expect("flush allocation report");
}

#[cfg(not(feature = "bench-allocations"))]
fn measure_allocations(_options: &Options) {
    eprintln!("allocation reports require --features bench-allocations");
    std::process::exit(2);
}

fn profile(options: &Options) {
    let (name, workload) = options.workload.unwrap_or(WORKLOADS[0]);
    let mut run = SustainedRun::new(workload, options.frames + options.warmup);
    run.advance(options.warmup);
    println!(
        "PROFILE_BEGIN pid={} workload={name} frames={}",
        std::process::id(),
        options.frames
    );
    io::stdout().flush().expect("flush profile marker");
    let start = Instant::now();
    profile_frames(&mut run, options.frames);
    let elapsed = start.elapsed();
    println!(
        "PROFILE_END workload={name} frames={} elapsed_seconds={:.6}",
        options.frames,
        elapsed.as_secs_f64()
    );
    run.finish();
}

fn main() {
    let args: Vec<_> = std::env::args()
        .skip(1)
        .filter(|arg| arg != "--bench")
        .collect();
    if args.is_empty() || args.iter().any(|arg| arg == "--help") {
        println!("{HELP}");
        return;
    }
    if args == ["--test"] {
        let options = Options {
            mode: "allocations".into(),
            frames: 33,
            warmup: 1024,
            workload: None,
            output: None,
        };
        if cfg!(feature = "bench-allocations") {
            measure_allocations(&options);
        } else {
            for workload in WORKLOADS {
                profile(&Options {
                    mode: "profile".into(),
                    frames: 33,
                    warmup: 1024,
                    workload: Some(workload),
                    output: None,
                });
            }
        }
        assert!(parse(&["allocations".into(), "--frames".into(), "0".into()]).is_err());
        assert!(parse(&["profile".into(), "--workload".into(), "unknown".into()]).is_err());
        return;
    }
    let options = parse(&args).unwrap_or_else(|error| {
        eprintln!("{error}\n{HELP}");
        std::process::exit(2)
    });
    if options.mode == "allocations" {
        measure_allocations(&options);
    } else {
        if cfg!(feature = "bench-allocations") {
            eprintln!("CPU profiling requires --features bench-support without bench-allocations");
            std::process::exit(2);
        }
        profile(&options);
    }
}
