use std::error::Error;

use crate::arm::ArmState;
use crate::config::{CONFIG_PATH, read_config_file};
use msfs::{MSFSEvent, legacy::NamedVariable};

const ARMED_VARIABLE: &str = "REPLAYER_ARMED";

#[msfs::gauge(name=replayer)]
async fn replayer(mut gauge: msfs::Gauge) -> Result<(), Box<dyn Error>> {
    let armed_variable = NamedVariable::from(ARMED_VARIABLE);
    let mut arm_state = ArmState::default();

    armed_variable.set_value(0.0);
    println!("REPLAYER: waiting for L:REPLAYER_ARMED = 1");

    while let Some(event) = gauge.next_event().await {
        match event {
            MSFSEvent::PreUpdate => {
                if arm_state.start(armed_variable.get_value::<f64>()) {
                    match read_config_file(CONFIG_PATH) {
                        Ok(config) => println!(
                            "REPLAYER: loaded {CONFIG_PATH} ({} inject, {} record)",
                            config.inject.len(),
                            config.record.len()
                        ),
                        Err(error) => {
                            armed_variable.set_value(0.0);
                            eprintln!("REPLAYER: {error:#}");
                            return Err(error.into());
                        }
                    }
                }
            }
            MSFSEvent::PreKill => armed_variable.set_value(0.0),
            _ => {}
        }
    }

    Ok(())
}
