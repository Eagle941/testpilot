//! MSFS-specific replay runtime used by the gauge entry point.

use msfs::legacy::NamedVariable;

use crate::error::{GaugeError, RecordingError};
use crate::replayer::{InterpolationFrame, Replayer, ReplayerUpdate};
use crate::simulator::{MsfsSimulator, SimulatorAdapter};

const ARMED_VARIABLE: &str = "REPLAYER_ARMED";

/// Owns the MSFS variables and replay state used by the gauge event loop.
pub(crate) struct GaugeRuntime {
    armed_variable: NamedVariable,
    replayer: Replayer,
    simulator: MsfsSimulator,
    recordings_validated: bool,
}

impl GaugeRuntime {
    /// Creates an idle runtime and resets the arming variable.
    pub(crate) fn new() -> GaugeRuntime {
        let armed_variable = NamedVariable::from(ARMED_VARIABLE);
        armed_variable.set_value(0.0);

        GaugeRuntime {
            armed_variable,
            replayer: Replayer::new(),
            simulator: MsfsSimulator::new(),
            recordings_validated: false,
        }
    }

    /// Processes one simulator pre-update event.
    pub(crate) fn pre_update(&mut self) -> anyhow::Result<()> {
        let simulation_time = self.simulator.simulation_time()?;
        let update = self
            .replayer
            .pre_update(self.armed_variable.get_value::<f64>(), simulation_time)?;

        match update {
            Some(ReplayerUpdate::Running(mut frame)) => {
                GaugeRuntime::process_frame(
                    &mut frame,
                    &mut self.simulator,
                    &mut self.recordings_validated,
                )?;
            }
            Some(ReplayerUpdate::Completed) => {
                self.stop()?;
                println!("TESTPILOT: scenario completed");
            }
            None => {}
        }
        Ok(())
    }

    /// Flushes telemetry, releases replay state, and resets the arming variable.
    pub(crate) fn stop(&mut self) -> Result<(), RecordingError> {
        let result = self.replayer.reset();
        self.recordings_validated = false;
        self.armed_variable.set_value(0.0);
        result
    }

    fn process_frame(
        frame: &mut InterpolationFrame<'_>,
        simulator: &mut impl SimulatorAdapter,
        recordings_validated: &mut bool,
    ) -> Result<(), GaugeError> {
        if !*recordings_validated {
            GaugeRuntime::validate_recordings(frame, simulator)?;
            *recordings_validated = true;
        }
        GaugeRuntime::inject_frame(frame, simulator)?;

        let elapsed = frame.elapsed();
        let mut sampled_values = Vec::with_capacity(frame.recordings().len());
        let mut has_due_recording = false;
        let (recordings, schedules) = frame.recordings_and_schedules();
        for (recording, schedule) in recordings.iter().zip(schedules.iter_mut()) {
            if !schedule.should_sample(elapsed) {
                sampled_values.push(None);
                continue;
            }
            has_due_recording = true;
            let value = simulator
                .read(&recording.variable, recording.unit.as_deref())
                .map_err(|source| GaugeError::SampleSignal {
                    signal: recording.name.clone(),
                    source,
                })?;
            sampled_values.push(Some(value));
        }
        if has_due_recording {
            frame.record(&sampled_values)?;
        }
        Ok(())
    }

    fn validate_recordings(
        frame: &InterpolationFrame<'_>,
        simulator: &mut impl SimulatorAdapter,
    ) -> Result<(), GaugeError> {
        for recording in frame.recordings() {
            simulator
                .validate_read(&recording.variable, recording.unit.as_deref())
                .map_err(|source| GaugeError::ValidateRecordingSignal {
                    signal: recording.name.clone(),
                    source,
                })?;
        }
        Ok(())
    }

    fn inject_frame(
        frame: &InterpolationFrame<'_>,
        simulator: &mut impl SimulatorAdapter,
    ) -> Result<(), GaugeError> {
        let elapsed = frame.elapsed();
        for data_points in frame.data_points() {
            let source_value =
                data_points
                    .value_at(elapsed)
                    .map_err(|source| GaugeError::InterpolateSignal {
                        signal: data_points.signal.to_owned(),
                        source,
                    })?;
            let simulator_value =
                data_points
                    .conversion
                    .convert(source_value)
                    .map_err(|source| GaugeError::ConvertSignal {
                        signal: data_points.signal.to_owned(),
                        source,
                    })?;
            simulator
                .write(data_points.variable, simulator_value)
                .map_err(|source| GaugeError::InjectSignal {
                    signal: data_points.signal.to_owned(),
                    source,
                })?;
        }
        Ok(())
    }
}
