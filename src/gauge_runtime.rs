//! MSFS-specific replay runtime used by the gauge entry point.

use msfs::legacy::NamedVariable;

use crate::error::GaugeError;
use crate::replayer::{InterpolationFrame, Replayer, ReplayerUpdate};
use crate::simulator::{MsfsSimulator, SimulatorAdapter};

const ARMED_VARIABLE: &str = "REPLAYER_ARMED";

/// Owns the MSFS variables and replay state used by the gauge event loop.
pub(crate) struct GaugeRuntime {
    armed_variable: NamedVariable,
    replayer: Replayer,
    simulator: MsfsSimulator,
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
        }
    }

    /// Processes one simulator pre-update event.
    pub(crate) fn pre_update(&mut self) -> anyhow::Result<()> {
        let simulation_time = self.simulator.simulation_time()?;
        let update = self
            .replayer
            .pre_update(self.armed_variable.get_value::<f64>(), simulation_time)?;

        match update {
            Some(ReplayerUpdate::Running(frame)) => {
                GaugeRuntime::inject_frame(&frame, &mut self.simulator)?;
            }
            Some(ReplayerUpdate::Completed) => {
                println!("TESTPILOT: scenario completed");
                self.stop();
            }
            None => {}
        }
        Ok(())
    }

    /// Releases replay state and resets the arming variable.
    pub(crate) fn stop(&mut self) {
        self.replayer.reset();
        self.armed_variable.set_value(0.0);
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
