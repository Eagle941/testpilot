use super::shared::*;
use crate::error::ScenarioError;

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
    match validate_scenario(missing.as_bytes(), &config()) {
        Err(ScenarioError::MissingColumn { column, .. })
            if column == "sidestick_pitch_position.value" => {}
        unexpected => panic!("expected missing-column validation error, got: {unexpected:?}"),
    }

    let duplicate = "sidestick_pitch_position.time,sidestick_pitch_position.value,sidestick_pitch_position.value,sidestick_roll_position.time,sidestick_roll_position.value\n0,0,0,0,0\n";
    match validate_scenario(duplicate.as_bytes(), &config()) {
        Err(ScenarioError::DuplicateHeader { .. }) => {}
        unexpected => panic!("expected duplicate-header validation error, got: {unexpected:?}"),
    }

    let non_adjacent = "sidestick_pitch_position.time,other,sidestick_pitch_position.value,sidestick_roll_position.time,sidestick_roll_position.value\n0,0,0,0,0\n";
    match validate_scenario(non_adjacent.as_bytes(), &config()) {
        Err(ScenarioError::NonAdjacentColumns { .. }) => {}
        unexpected => panic!("expected non-adjacent-column validation error, got: {unexpected:?}"),
    }
}

#[test]
fn rejects_half_populated_and_sparse_pairs() {
    match validate("0,0,0,0\n0.5,,0.5,0\n") {
        Err(ScenarioError::HalfPopulatedPair { .. }) => {}
        unexpected => panic!("expected half-populated-pair validation error, got: {unexpected:?}"),
    }
    match validate("0,0,0,0\n,,0.5,0\n0.5,0,1,0\n") {
        Err(ScenarioError::SparseSeries { .. }) => {}
        unexpected => panic!("expected sparse-series validation error, got: {unexpected:?}"),
    }
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
            "first" => match error {
                ScenarioError::FirstTimestampNotZero { .. } => {}
                unexpected => {
                    panic!("expected first-timestamp validation error, got: {unexpected:?}")
                }
            },
            "order" => match error {
                ScenarioError::NonIncreasingTime { .. } => {}
                unexpected => {
                    panic!("expected non-increasing-time validation error, got: {unexpected:?}")
                }
            },
            "negative" => match error {
                ScenarioError::NegativeTime { .. } => {}
                unexpected => {
                    panic!("expected negative-time validation error, got: {unexpected:?}")
                }
            },
            "finite" => match error {
                ScenarioError::NonFiniteTime { .. } => {}
                unexpected => {
                    panic!("expected non-finite-time validation error, got: {unexpected:?}")
                }
            },
            "range" => match error {
                ScenarioError::TimeOutOfRange { .. } => {}
                unexpected => {
                    panic!("expected timestamp-out-of-range validation error, got: {unexpected:?}")
                }
            },
            _ => panic!("unknown test predicate"),
        }
    }
}

#[test]
fn rejects_invalid_and_out_of_range_values() {
    match validate("0,nope,0,0\n1,0,1,0\n") {
        Err(ScenarioError::ParseInvalid(_)) => {}
        unexpected => panic!("expected parse-invalid validation error, got: {unexpected:?}"),
    }
    match validate("0,NaN,0,0\n1,0,1,0\n") {
        Err(ScenarioError::NonFiniteValue { .. }) => {}
        unexpected => panic!("expected non-finite-value validation error, got: {unexpected:?}"),
    }
    match validate("0,101,0,0\n1,0,1,0\n") {
        Err(ScenarioError::ValueOutsideSourceRange { .. }) => {}
        unexpected => {
            panic!("expected value-outside-source-range validation error, got: {unexpected:?}")
        }
    }
}

#[test]
fn rejects_missing_samples() {
    match validate(",,0,0\n,,1,0\n") {
        Err(ScenarioError::MissingSamples { .. }) => {}
        unexpected => panic!("expected missing-samples validation error, got: {unexpected:?}"),
    }
}

#[test]
fn reports_csv_structure_errors() {
    match validate("0,0,0\n") {
        Err(ScenarioError::Csv(_)) => {}
        unexpected => panic!("expected CSV-parse validation error, got: {unexpected:?}"),
    }
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
