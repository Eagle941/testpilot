//! MSFS-specific replay runtime used by the gauge entry point.

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
        let simulation_time_seconds = GaugeRuntime::simulation_time_seconds()?;
        let update = self.replayer.pre_update(
            self.armed_variable.get_value::<f64>(),
            simulation_time_seconds,
        )?;

        match update {
            ReplayerUpdate::Running(frame) => {
                GaugeRuntime::inject_frame(&frame, &mut self.variable_writer)?;
            }
            ReplayerUpdate::Completed => {
                println!("TESTPILOT: scenario completed");
                self.stop();
            }
            _ => {}
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
        let elapsed_seconds = frame.elapsed_seconds();
        println!("TESTPILOT: elapsed simulation seconds={elapsed_seconds:.6}");
        for data_points in frame.data_points() {
            let source_value = data_points.value_at(elapsed_seconds).map_err(|source| {
                GaugeError::InterpolateSignal {
                    signal: data_points.signal.to_owned(),
                    source,
                }
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
                    data_points.previous.time_seconds,
                    data_points.previous.value,
                    next.time_seconds,
                    next.value,
                    source_value,
                    simulator_value
                ),
                None => println!(
                    "TESTPILOT: {} -> {} final=({}, {}) hold source={} simulator={}",
                    data_points.signal,
                    data_points.variable,
                    data_points.previous.time_seconds,
                    data_points.previous.value,
                    source_value,
                    simulator_value
                ),
            }
        }
        Ok(())
    }

    fn simulation_time_seconds() -> Result<f64, SimulatorError> {
        let value = execute_calculator_code::<f64>(SIMULATION_TIME_CODE)
            .ok_or(SimulatorError::SimulationTimeUnavailable)?;
        if !value.is_finite() {
            return Err(SimulatorError::InvalidSimulationTime { value });
        }
        Ok(value)
    }
}
