//! MSFS-specific replay runtime used by the gauge entry point.

use crate::arm::ArmingMonitor;
use crate::error::GaugeError;
use crate::replayer::{InterpolationFrame, Replayer, ReplayerUpdate};
use crate::simulator::{MsfsSimulator, SimulatorAdapter};

const ARMED_VARIABLE: &str = "L:REPLAYER_ARMED";

/// Owns the MSFS variables and replay state used by the gauge event loop.
pub struct GaugeRuntime {
    /// Replay orchestrator.
    replayer: Replayer,
    arming: ArmingMonitor,
    /// Adapter around msfs-rs legacy calculator code.
    simulator: MsfsSimulator,
}

impl GaugeRuntime {
    /// Creates an idle runtime and initializes the arming switch to `0.0`.
    pub fn new() -> Result<GaugeRuntime, GaugeError> {
        let mut runtime = GaugeRuntime {
            arming: ArmingMonitor::new(ARMED_VARIABLE),
            replayer: Replayer::new(),
            simulator: MsfsSimulator::new(),
        };
        runtime.arming.reset(&mut runtime.simulator)?;

        Ok(runtime)
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
        let init_now = self.arming.ready_to_start(&mut self.simulator)?;

        match self.replayer.pre_update(init_now, simulation_time)? {
            Some(ReplayerUpdate::Running { frame, started_now }) => {
                Self::handle_running_frame(frame, started_now, &mut self.simulator)?;
            }
            Some(ReplayerUpdate::Completed) => self.stop()?,
            None => {}
        }

        Ok(())
    }

    /// Stops the active replay, flushing telemetry and resetting arming state.
    ///
    /// This method is idempotent from the perspective of runtime state; if no
    /// scenario is active, it still resets arming state and returns `Ok(())`.
    pub fn stop(&mut self) -> Result<(), GaugeError> {
        let recorder_result = self.replayer.reset().map_err(GaugeError::RecordFrame);
        self.arming.reset(&mut self.simulator)?;

        recorder_result
    }

    /// Processes one running frame from the replay engine.
    ///
    /// If the frame is the first after a start transition, recording variables are
    /// validated before input/record operations are executed.
    ///
    /// `started_now` is true only for the first frame of a newly started run, and
    /// allows one-time per-run work to execute exactly once.
    fn handle_running_frame(
        mut frame: InterpolationFrame<'_>,
        started_now: bool,
        simulator: &mut MsfsSimulator,
    ) -> Result<(), GaugeError> {
        if started_now {
            Self::validate_recordings(simulator, &frame)?;
        }

        Self::inject_inputs(simulator, &frame)?;
        Self::record_outputs(simulator, &mut frame)?;

        Ok(())
    }

    /// Validates configured recordings before a run starts.
    ///
    /// This verifies every requested recording signal is readable by the simulator:
    /// variable prefix/format is supported, required read units are present for
    /// `A:` variables, `L:` variables do not provide units, and calculator read
    /// code can be generated.
    ///
    /// Validation runs only on the first running frame after a start transition so
    /// expensive per-run checks are separated from hot per-frame logic.
    fn validate_recordings(
        simulator: &mut MsfsSimulator,
        frame: &InterpolationFrame<'_>,
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

    /// Interpolates and writes all configured input signals for this frame.
    ///
    /// Interpolation and conversion failures are surfaced as gauge-level errors.
    fn inject_inputs(
        simulator: &mut MsfsSimulator,
        frame: &InterpolationFrame<'_>,
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

    /// Samples configured recordings when they are due and writes a telemetry row.
    ///
    /// Rows are written only when at least one recording is due on this frame.
    fn record_outputs(
        simulator: &mut MsfsSimulator,
        frame: &mut InterpolationFrame<'_>,
    ) -> Result<(), GaugeError> {
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
            let value = simulator
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
