use std::error::Error;

use msfs::MSFSEvent;

use crate::gauge_runtime::GaugeRuntime;

#[msfs::gauge(name=testpilot)]
async fn testpilot(mut gauge: msfs::Gauge) -> Result<(), Box<dyn Error>> {
    println!("TESTPILOT: waiting for L:REPLAYER_ARMED = 1");

    let mut runtime = GaugeRuntime::new();
    while let Some(event) = gauge.next_event().await {
        match event {
            MSFSEvent::PreUpdate => {
                if let Err(error) = runtime.pre_update() {
                    eprintln!("TESTPILOT: {error:#}");
                    runtime.stop();
                    return Err(error.into());
                }
            }
            MSFSEvent::PreKill => runtime.stop(),
            _ => {}
        }
    }

    runtime.stop();
    Ok(())
}
