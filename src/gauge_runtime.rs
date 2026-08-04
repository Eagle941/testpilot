//! MSFS-specific replay runtime used by the gauge entry point.

use msfs::legacy::NamedVariable;

use crate::error::{GaugeError, RecordingError};
use crate::replayer::{InterpolationFrame, Replayer, ReplayerUpdate};
use crate::simulator::{MsfsSimulator, SimulatorAdapter};

const ARMED_VARIABLE: &str = "REPLAYER_ARMED";

/// Owns the MSFS variables and replay state used by the gauge event loop.
pub struct GaugeRuntime {
    /// Cached local variable used for scenario arming.
    armed_variable: NamedVariable,
    /// Replay orchestrator.
    replayer: Replayer,
    /// Adapter around msfs-rs legacy calculator code.
    simulator: MsfsSimulator,
}

impl GaugeRuntime {
    /// Creates an idle runtime and initializes the arming switch to `0.0`.
    pub fn new() -> GaugeRuntime {
        let armed_variable = NamedVariable::from(ARMED_VARIABLE);
        armed_variable.set_value(0.0);

        GaugeRuntime {
            armed_variable,
            replayer: Replayer::new(),
            simulator: MsfsSimulator::new(),
        }
    }

    /// Handles one `MSFSEvent::PreUpdate` cycle.
    ///
    /// Reads current simulation time, evaluates arming transitions, and applies the
    /// resulting replay update if the scenario is active.
    ///
    /// A start transition is a `0 -> 1` armed value transition; the first running
    /// frame after that transition is marked with `started_now = true` so one-time
    /// initialization logic can run exactly once per run.
    pub fn pre_update(&mut self) -> anyhow::Result<()> {
        let simulation_time = self.simulator.simulation_time()?;
        let armed = self.armed_variable.get_value::<f64>();

        if let Some(update) = self.replayer.pre_update(armed, simulation_time)? {
            self.handle_replayer_update(update)?;
        }

        Ok(())
    }

    /// Stops the active replay, flushing telemetry and resetting arming state.
    ///
    /// This method is idempotent from the perspective of runtime state; if no
    /// scenario is active, it still resets arming state and returns `Ok(())`.
    pub fn stop(&mut self) -> Result<(), RecordingError> {
        let result = self.replayer.reset();
        self.armed_variable.set_value(0.0);
        result
    }

    /// Routes a replay update produced by [`Replayer::pre_update`].
    ///
    /// Running updates are processed as a single frame; completed updates stop
    /// telemetry and transition to idle state.
    fn handle_replayer_update(&mut self, update: ReplayerUpdate<'_>) -> anyhow::Result<()> {
        match update {
            ReplayerUpdate::Running { frame, started_now } => {
                self.handle_running_frame(frame, started_now)?
            }
            ReplayerUpdate::Completed => self.finish_run()?,
        }

        Ok(())
    }

    /// Processes one running frame from the replay engine.
    ///
    /// If the frame is the first after a start transition, recording variables are
    /// validated before input/record operations are executed.
    ///
    /// `started_now` is true only for the first frame of a newly started run, and
    /// allows one-time per-run work to execute exactly once.
    fn handle_running_frame(
        &mut self,
        mut frame: InterpolationFrame<'_>,
        started_now: bool,
    ) -> Result<(), GaugeError> {
        if started_now {
            self.validate_recordings(&frame)?;
        }

        self.inject_inputs(&frame)?;
        self.record_outputs(&mut frame)?;

        Ok(())
    }

    /// Finalizes a completed replay run and resets scenario state.
    ///
    /// Telemetry flush and arming reset are delegated to [`Self::stop`].
    fn finish_run(&mut self) -> Result<(), RecordingError> {
        self.stop()?;
        println!("TESTPILOT: scenario completed");
        Ok(())
    }

    /// Validates all configured recording signals for simulator readability.
    ///
    /// Validation runs only on the first running frame after a start transition so
    /// expensive per-run checks are separated from hot per-frame logic.
    fn validate_recordings(&mut self, frame: &InterpolationFrame<'_>) -> Result<(), GaugeError> {
        for recording in frame.recordings() {
            self.simulator
                .validate_read(&recording.variable, recording.unit.as_deref())
                .map_err(|source| GaugeError::ValidateRecordingSignal {
                    signal: recording.name.clone(),
                    source,
                })?;
        }

        Ok(())
    }

    /// Interpolates and writes all configured input signals for this frame.
    ///
    /// Interpolation and conversion failures are surfaced as gauge-level errors.
    fn inject_inputs(&mut self, frame: &InterpolationFrame<'_>) -> Result<(), GaugeError> {
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

            self.simulator
                .write(data_points.variable, simulator_value)
                .map_err(|source| GaugeError::InjectSignal {
                    signal: data_points.signal.to_owned(),
                    source,
                })?;
        }

        Ok(())
    }

    /// Samples configured recordings when they are due and writes a telemetry row.
    ///
    /// Rows are written only when at least one recording is due on this frame.
    fn record_outputs(&mut self, frame: &mut InterpolationFrame<'_>) -> Result<(), GaugeError> {
        let elapsed = frame.elapsed();
        let mut sampled_values = Vec::with_capacity(frame.recordings().len());
        let mut any_due = false;
        let (recordings, schedules) = frame.recordings_and_schedules();

        for (recording, schedule) in recordings.iter().zip(schedules.iter_mut()) {
            if !schedule.should_sample(elapsed) {
                sampled_values.push(None);
                continue;
            }

            any_due = true;
            let value = self
                .simulator
                .read(&recording.variable, recording.unit.as_deref())
                .map_err(|source| GaugeError::SampleSignal {
                    signal: recording.name.clone(),
                    source,
                })?;
            sampled_values.push(Some(value));
        }

        if any_due {
            frame.record(&sampled_values)?;
        }

        Ok(())
    }
}
