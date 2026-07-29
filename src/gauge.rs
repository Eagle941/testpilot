use std::error::Error;

use anyhow::{anyhow, bail};
use msfs::{
    MSFSEvent,
    legacy::{NamedVariable, execute_calculator_code},
};

use crate::playback::LinearSegment;
use crate::replayer::{InterpolationFrame, ReplayerEvent, ReplayerGauge};

const ARMED_VARIABLE: &str = "REPLAYER_ARMED";
const SIMULATION_TIME_CODE: &str = "(E:SIMULATION TIME, seconds)";

#[msfs::gauge(name=replayer)]
async fn replayer(mut gauge: msfs::Gauge) -> Result<(), Box<dyn Error>> {
    let armed_variable = NamedVariable::from(ARMED_VARIABLE);
    armed_variable.set_value(0.0);
    println!("REPLAYER: waiting for L:REPLAYER_ARMED = 1");

    let mut replayer = ReplayerGauge::new();
    while let Some(event) = gauge.next_event().await {
        let result = match event {
            MSFSEvent::PreUpdate => {
                replayer.pre_update(armed_variable.get_value::<f64>(), simulation_time_seconds)
            }
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
                if let Some(frame) = update.interpolation() {
                    print_interpolation(frame)?;
                }
                if event == ReplayerEvent::Completed {
                    println!("REPLAYER: scenario completed");
                    replayer.reset();
                    armed_variable.set_value(0.0);
                }
            }
            Err(error) => {
                eprintln!("REPLAYER: {error:#}");
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

fn print_interpolation(frame: &InterpolationFrame<'_>) -> anyhow::Result<()> {
    let elapsed_seconds = frame.elapsed_seconds();
    println!("REPLAYER: elapsed simulation seconds={elapsed_seconds:.6}");
    for data_points in frame.data_points() {
        let source_value = LinearSegment::new(data_points.previous, data_points.next)?
            .value_at(elapsed_seconds)?;
        let simulator_value = data_points.conversion.convert(source_value)?;
        println!(
            "REPLAYER: {} -> {} previous=({}, {}) next=({}, {}) source={} simulator={}",
            data_points.signal,
            data_points.variable,
            data_points.previous.time_seconds,
            data_points.previous.value,
            data_points.next.time_seconds,
            data_points.next.value,
            source_value,
            simulator_value
        );
    }
    Ok(())
}

fn simulation_time_seconds() -> anyhow::Result<f64> {
    let value = execute_calculator_code::<f64>(SIMULATION_TIME_CODE)
        .ok_or_else(|| anyhow!("failed to read {SIMULATION_TIME_CODE}"))?;
    if !value.is_finite() {
        bail!("invalid simulation time {value}");
    }
    Ok(value)
}
