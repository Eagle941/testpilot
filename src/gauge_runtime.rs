//! MSFS-specific replay runtime used by the gauge entry point.

use std::time::Duration;

use msfs::legacy::{NamedVariable, execute_calculator_code};

use crate::error::{GaugeError, SimulatorError};
use crate::replayer::{InterpolationFrame, ReplayerGauge, ReplayerUpdate};
use crate::simulator::VariableWriter;

const ARMED_VARIABLE: &str = "REPLAYER_ARMED";
const SIMULATION_TIME_CODE: &str = "(E:SIMULATION TIME, seconds)";

/// Owns the MSFS variables and replay state used by the gauge event loop.
pub(crate) struct GaugeRuntime {
    armed_variable: NamedVariable,
    replayer: ReplayerGauge,
    variable_writer: VariableWriter,
}

impl GaugeRuntime {
    /// Creates an idle runtime and resets the arming variable.
    pub(crate) fn new() -> GaugeRuntime {
        let armed_variable = NamedVariable::from(ARMED_VARIABLE);
        armed_variable.set_value(0.0);

        GaugeRuntime {
            armed_variable,
            replayer: ReplayerGauge::new(),
            variable_writer: VariableWriter::new(),
        }
    }

    /// Processes one simulator pre-update event.
    pub(crate) fn pre_update(&mut self) -> anyhow::Result<()> {
        let simulation_time = GaugeRuntime::simulation_time()?;
        let update = self
            .replayer
            .pre_update(self.armed_variable.get_value::<f64>(), simulation_time)?;

        match update {
            Some(ReplayerUpdate::Running(frame)) => {
                GaugeRuntime::inject_frame(&frame, &mut self.variable_writer)?;
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
        variable_writer: &mut VariableWriter,
    ) -> Result<(), GaugeError> {
        let elapsed = frame.elapsed();
        println!(
            "TESTPILOT: elapsed simulation seconds={:.6}",
            elapsed.as_secs_f64()
        );
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
            variable_writer
                .write(data_points.variable, simulator_value)
                .map_err(|source| GaugeError::InjectSignal {
                    signal: data_points.signal.to_owned(),
                    source,
                })?;
            match data_points.next {
                Some(next) => println!(
                    "TESTPILOT: {} -> {} previous=({}, {}) next=({}, {}) source={} simulator={}",
                    data_points.signal,
                    data_points.variable,
                    data_points.previous.time.as_secs_f64(),
                    data_points.previous.value,
                    next.time.as_secs_f64(),
                    next.value,
                    source_value,
                    simulator_value
                ),
                None => println!(
                    "TESTPILOT: {} -> {} final=({}, {}) hold source={} simulator={}",
                    data_points.signal,
                    data_points.variable,
                    data_points.previous.time.as_secs_f64(),
                    data_points.previous.value,
                    source_value,
                    simulator_value
                ),
            }
        }
        Ok(())
    }

    fn simulation_time() -> Result<Duration, SimulatorError> {
        let value = execute_calculator_code::<f64>(SIMULATION_TIME_CODE)
            .ok_or(SimulatorError::SimulationTimeUnavailable)?;
        Duration::try_from_secs_f64(value)
            .map_err(|_| SimulatorError::InvalidSimulationTime { value })
    }
}
