use super::shared::*;
use crate::error::ScenarioError;

#[test]
fn validates_default_unequal_length_scenario() {
    let summary = validate_scenario(
        include_bytes!("../../../example/scenario.csv").as_slice(),
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
        Err(ScenarioError::MissingColumn { column, .. })
            if column == "sidestick_pitch_position.value"
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
