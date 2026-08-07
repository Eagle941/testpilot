use std::error::Error;

use msfs::MSFSEvent;

use crate::gauge_runtime::GaugeRuntime;

#[msfs::gauge(name=testpilot)]
/// MSFS gauge entrypoint that drives the replay runtime each `PreUpdate`.
async fn testpilot(mut gauge: msfs::Gauge) -> Result<(), Box<dyn Error>> {
    let mut runtime = GaugeRuntime::new()?;

    println!("TESTPILOT: waiting for L:REPLAYER_ARMED = 1");
    while let Some(event) = gauge.next_event().await {
        match event {
            MSFSEvent::PreUpdate => {
                if let Err(error) = runtime.pre_update() {
                    // Pre-update failures are handled here and cleanup is deferred to a single
                    // best-effort stop path below.
                    println!("TESTPILOT ERROR: {error:#}");
                    break;
                }
            }
            MSFSEvent::PreKill => break,
            _ => {}
        }
    }

    // Cleanup is intentionally best-effort: report but do not fail this entrypoint
    // on a secondary shutdown/write-path error.
    if let Err(error) = runtime.stop() {
        println!("TESTPILOT ERROR: cleanup failed: {error}");
    }

    Ok(())
}
