use std::error::Error;

use anyhow::Context;
use msfs::{
    MSFSEvent,
    legacy::{NamedVariable, execute_calculator_code},
};

use crate::error::SimulatorError;
use crate::replayer::{InterpolationFrame, ReplayerEvent, ReplayerGauge};
use crate::simulator::VariableWriter;

const ARMED_VARIABLE: &str = "REPLAYER_ARMED";
const SIMULATION_TIME_CODE: &str = "(E:SIMULATION TIME, seconds)";

#[msfs::gauge(name=testpilot)]
async fn testpilot(mut gauge: msfs::Gauge) -> Result<(), Box<dyn Error>> {
    let armed_variable = NamedVariable::from(ARMED_VARIABLE);
    armed_variable.set_value(0.0);
    println!("TESTPILOT: waiting for L:REPLAYER_ARMED = 1");

    let mut replayer = ReplayerGauge::new();
    let mut variable_writer = VariableWriter::new();
    while let Some(event) = gauge.next_event().await {
        let result = match event {
            MSFSEvent::PreUpdate => match simulation_time_seconds() {
                Ok(simulation_time_seconds) => {
                    replayer.pre_update(armed_variable.get_value::<f64>(), simulation_time_seconds)
                }
                Err(error) => Err(error),
            },
            MSFSEvent::PreKill => {
                replayer.reset();
                armed_variable.set_value(0.0);
                continue;
            }
            _ => continue,
        };

        match result {
            Ok(update) => {
                let event = update.event();
                if let Some(frame) = update.interpolation()
                    && let Err(error) = inject_frame(frame, &mut variable_writer)
                {
                    eprintln!("TESTPILOT: {error:#}");
                    replayer.reset();
                    armed_variable.set_value(0.0);
                    return Err(error.into());
                }
                if event == ReplayerEvent::Completed {
                    println!("TESTPILOT: scenario completed");
                    replayer.reset();
                    armed_variable.set_value(0.0);
                }
            }
            Err(error) => {
                eprintln!("TESTPILOT: {error:#}");
                replayer.reset();
                armed_variable.set_value(0.0);
                return Err(error.into());
            }
        }
    }

    replayer.reset();
    armed_variable.set_value(0.0);
    Ok(())
}

fn inject_frame(
    frame: &InterpolationFrame<'_>,
    variable_writer: &mut VariableWriter,
) -> anyhow::Result<()> {
    let elapsed_seconds = frame.elapsed_seconds();
    println!("TESTPILOT: elapsed simulation seconds={elapsed_seconds:.6}");
    for data_points in frame.data_points() {
        let source_value = data_points.value_at(elapsed_seconds)?;
        let simulator_value = data_points.conversion.convert(source_value)?;
        variable_writer
            .write(data_points.variable, simulator_value)
            .with_context(|| format!("failed to inject signal `{}`", data_points.signal))?;
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

fn simulation_time_seconds() -> anyhow::Result<f64> {
    let value = execute_calculator_code::<f64>(SIMULATION_TIME_CODE)
        .ok_or(SimulatorError::SimulationTimeUnavailable)?;
    if !value.is_finite() {
        return Err(SimulatorError::InvalidSimulationTime { value }.into());
    }
    Ok(value)
}
