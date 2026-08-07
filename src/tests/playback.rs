use std::path::Path;
use std::time::Duration;

use crate::cursor::Scenario;
use crate::playback::Sample;

use super::shared::*;

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

    let result = Scenario::new(&path, &config());
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
    let mut playback = Scenario::new(path, &config())
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
