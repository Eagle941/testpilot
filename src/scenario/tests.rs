use std::path::Path;
use std::time::Duration;

use crate::config::{ReplayConfig, parse_config};
use crate::playback::Sample;

use super::*;

const CONFIG: &str = r#"
format_version = 1
aircraft_target = "flybywire-a32nx"
input_file = "scenario.csv"

[inject.0]
name = "sidestick_pitch_position"
variable = "K:AXIS_ELEVATOR_SET"
time_column = "sidestick_pitch_position.time"
value_column = "sidestick_pitch_position.value"
source_range = [-100.0, 100.0]
simulator_range = [-1.0, 1.0]

[inject.1]
name = "sidestick_roll_position"
variable = "K:AXIS_AILERONS_SET"
time_column = "sidestick_roll_position.time"
value_column = "sidestick_roll_position.value"
source_range = [-100.0, 100.0]
simulator_range = [-1.0, 1.0]

[record.0]
name = "pitch"
variable = "A:PLANE PITCH DEGREES"
"#;

const HEADER: &str = "sidestick_pitch_position.time,sidestick_pitch_position.value,sidestick_roll_position.time,sidestick_roll_position.value\n";

fn time(seconds: f64) -> Duration {
    Duration::try_from_secs_f64(seconds).unwrap()
}

fn config() -> ReplayConfig {
    parse_config(CONFIG).unwrap_or_else(|error| panic!("valid test config rejected: {error}"))
}

fn validate(body: &str) -> Result<ScenarioSummary, ScenarioError> {
    validate_scenario(format!("{HEADER}{body}").as_bytes(), &config())
}

#[test]
fn validates_default_unequal_length_scenario() {
    let summary = validate_scenario(
        include_bytes!("../../example/scenario.csv").as_slice(),
        &config(),
    )
    .unwrap_or_else(|error| panic!("default scenario rejected: {error}"));

    assert_eq!(summary.duration, time(40.0));
    assert_eq!(summary.signals[0].sample_count, 4);
    assert_eq!(summary.signals[0].final_time, time(20.0));
    assert_eq!(summary.signals[1].sample_count, 5);
    assert_eq!(summary.signals[1].final_time, time(40.0));
}

#[test]
fn validates_irregular_independent_series() {
    let summary = validate(
        "0,0,0,0\n\
             0.15,10,0.2,5\n\
             0.425,-5,0.7,0\n\
             0.7,0,,\n",
    )
    .unwrap_or_else(|error| panic!("valid scenario rejected: {error}"));

    assert_eq!(summary.duration, time(0.7));
    assert_eq!(summary.signals[0].sample_count, 4);
    assert_eq!(summary.signals[1].sample_count, 3);
}

#[test]
fn rejects_missing_duplicate_and_non_adjacent_headers() {
    let missing = "sidestick_pitch_position.time,other,sidestick_roll_position.time,sidestick_roll_position.value\n0,0,0,0\n";
    assert!(matches!(
        validate_scenario(missing.as_bytes(), &config()),
        Err(ScenarioError::MissingColumn { .. })
    ));

    let duplicate = "sidestick_pitch_position.time,sidestick_pitch_position.value,sidestick_pitch_position.value,sidestick_roll_position.time,sidestick_roll_position.value\n0,0,0,0,0\n";
    assert!(matches!(
        validate_scenario(duplicate.as_bytes(), &config()),
        Err(ScenarioError::DuplicateHeader { .. })
    ));

    let non_adjacent = "sidestick_pitch_position.time,other,sidestick_pitch_position.value,sidestick_roll_position.time,sidestick_roll_position.value\n0,0,0,0,0\n";
    assert!(matches!(
        validate_scenario(non_adjacent.as_bytes(), &config()),
        Err(ScenarioError::NonAdjacentColumns { .. })
    ));
}

#[test]
fn rejects_half_populated_and_sparse_pairs() {
    assert!(matches!(
        validate("0,0,0,0\n0.5,,0.5,0\n"),
        Err(ScenarioError::HalfPopulatedPair { .. })
    ));
    assert!(matches!(
        validate("0,0,0,0\n,,0.5,0\n0.5,0,1,0\n"),
        Err(ScenarioError::SparseSeries { .. })
    ));
}

#[test]
fn rejects_invalid_timestamps() {
    for (body, predicate) in [
        ("0.1,0,0,0\n1,0,1,0\n", "first"),
        ("0,0,0,0\n0,0,1,0\n", "order"),
        ("0,0,0,0\n-1,0,1,0\n", "negative"),
        ("0,0,0,0\nNaN,0,1,0\n", "finite"),
        ("0,0,0,0\n1e30,0,1,0\n", "range"),
    ] {
        let error = validate(body).expect_err("invalid timestamp accepted");
        match predicate {
            "first" => assert!(matches!(error, ScenarioError::FirstTimestampNotZero { .. })),
            "order" => assert!(matches!(error, ScenarioError::NonIncreasingTime { .. })),
            "negative" => assert!(matches!(error, ScenarioError::NegativeTime { .. })),
            "finite" => assert!(matches!(error, ScenarioError::NonFiniteTime { .. })),
            "range" => assert!(matches!(error, ScenarioError::TimeOutOfRange { .. })),
            _ => panic!("unknown test predicate"),
        }
    }
}

#[test]
fn rejects_invalid_and_out_of_range_values() {
    assert!(matches!(
        validate("0,nope,0,0\n1,0,1,0\n"),
        Err(ScenarioError::ParseInvalid(_))
    ));
    assert!(matches!(
        validate("0,NaN,0,0\n1,0,1,0\n"),
        Err(ScenarioError::NonFiniteValue { .. })
    ));
    assert!(matches!(
        validate("0,101,0,0\n1,0,1,0\n"),
        Err(ScenarioError::ValueOutsideSourceRange { .. })
    ));
}

#[test]
fn rejects_missing_samples() {
    assert!(matches!(
        validate(",,0,0\n,,1,0\n"),
        Err(ScenarioError::MissingSamples { .. })
    ));
}

#[test]
fn initializes_rows_and_catches_up_across_multiple_intervals() {
    let path = std::env::temp_dir().join(format!(
        "replay-incremental-scenario-{}.csv",
        std::process::id()
    ));
    let contents = format!("{HEADER}0,1,0,2\n0.1,3,0.75,4\n0.2,5,1.5,6\n0.5,7,,\n");
    if let Err(error) = std::fs::write(&path, contents) {
        panic!("failed to create scenario fixture: {error}");
    }

    let result = ScenarioPlayback::new(&path, &config());
    let mut playback = match result {
        Ok(playback) => playback,
        Err(error) => panic!("failed to open scenario fixture: {error:#}"),
    };
    assert_eq!(playback.signal_count(), 2);

    let rows = playback.interpolation_rows().collect::<Vec<_>>();
    assert_eq!(rows.len(), 2);
    assert_eq!(rows[0].signal, "sidestick_pitch_position");
    assert_eq!(rows[0].variable, "K:AXIS_ELEVATOR_SET");
    assert_eq!(rows[0].previous, Sample::new(Duration::ZERO, 1.0).unwrap());
    assert_eq!(rows[0].next, Some(Sample::new(time(0.1), 3.0).unwrap()));
    assert_eq!(rows[1].signal, "sidestick_roll_position");
    assert_eq!(rows[1].variable, "K:AXIS_AILERONS_SET");
    assert_eq!(rows[1].previous, Sample::new(Duration::ZERO, 2.0).unwrap());
    assert_eq!(rows[1].next, Some(Sample::new(time(0.75), 4.0).unwrap()));

    playback
        .advance(time(0.35))
        .unwrap_or_else(|error| panic!("catch-up read failed: {error:#}"));
    assert!(!playback.completed());
    let rows = playback.interpolation_rows().collect::<Vec<_>>();
    assert_eq!(rows[0].previous, Sample::new(time(0.2), 5.0).unwrap());
    assert_eq!(rows[0].next, Some(Sample::new(time(0.5), 7.0).unwrap()));
    assert_eq!(rows[0].value_at(time(0.35)), Ok(6.0));
    assert_eq!(rows[1].previous, Sample::new(Duration::ZERO, 2.0).unwrap());

    let _ = std::fs::remove_file(path);
}

#[test]
fn advances_and_holds_unequal_length_series() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("example/scenario.csv");
    let mut playback = ScenarioPlayback::new(path, &config())
        .unwrap_or_else(|error| panic!("failed to open default scenario: {error:#}"));

    let mut held_pitch = false;
    for frame in 0..=1200 {
        let elapsed = time(f64::from(frame) / 30.0);
        playback
            .advance(elapsed)
            .unwrap_or_else(|error| panic!("advance failed at {elapsed:?}: {error:#}"));
        assert!(!playback.completed());
        for rows in playback.interpolation_rows() {
            assert!(rows.previous.time <= elapsed);
            match rows.next {
                Some(next) => assert!(elapsed <= next.time),
                None => {
                    assert_eq!(rows.value_at(elapsed).unwrap(), rows.previous.value);
                    if rows.signal == "sidestick_pitch_position" {
                        held_pitch = true;
                        assert_eq!(rows.previous.time, time(20.0));
                        assert_eq!(rows.previous.value, 0.0);
                    }
                }
            }
            rows.value_at(elapsed)
                .unwrap_or_else(|error| panic!("interpolation failed at {elapsed:?}: {error}"));
        }
    }
    assert!(held_pitch);

    playback
        .advance(time(40.1))
        .unwrap_or_else(|error| panic!("completion advance failed: {error:#}"));
    assert!(playback.completed());
}

#[test]
fn reports_csv_structure_errors() {
    assert!(matches!(validate("0,0,0\n"), Err(ScenarioError::Csv(_))));
}

#[test]
fn validates_long_inputs_with_constant_state_size() {
    let mut csv = String::from(HEADER);
    for index in 0..20_000 {
        csv.push_str(&format!("{index},0,{index},0\n"));
    }

    let summary = validate_scenario(csv.as_bytes(), &config())
        .unwrap_or_else(|error| panic!("long scenario rejected: {error}"));
    assert_eq!(summary.signals[0].sample_count, 20_000);
    assert_eq!(summary.signals[1].sample_count, 20_000);
}
