use std::error::Error;

use msfs::MSFSEvent;

use crate::{gauge_runtime::GaugeRuntime, replayer::Replayer, simulator::MsfsSimulator};

#[msfs::gauge(name=testpilot)]
/// MSFS gauge entrypoint that drives the replay runtime each `PreUpdate`.
async fn testpilot(mut gauge: msfs::Gauge) -> Result<(), Box<dyn Error>> {
    let mut runtime = GaugeRuntime::new(Replayer::new(), MsfsSimulator::new())?;
    // PreKill stops replay, but this future must remain pending until msfs-rs
    // closes the event stream and polls it on PostKill. Returning on PreKill
    // would make that final poll panic by resuming an already completed future.
    // While awaiting closure (next_event() returns None), suppress updates so
    // replay cannot restart, and ignore repeated PreKill cleanup requests.
    let mut stopping = false;

    println!("TESTPILOT: waiting for L:REPLAYER_ARMED = 1");
    while let Some(event) = gauge.next_event().await {
        match event {
            MSFSEvent::PreUpdate if !stopping => {
                if let Err(error) = runtime.pre_update() {
                    // Pre-update failures are treated as recoverable runtime errors:
                    // reset simulator state back to idle and continue waiting for the next
                    // arming edge.
                    println!("TESTPILOT ERROR: {error:#}");
                    if let Err(stop_error) = runtime.stop() {
                        println!("TESTPILOT ERROR: recovery failed: {stop_error:#}");
                    }
                }
            }
            MSFSEvent::PreKill if !stopping => {
                // Block further replay work even if best-effort cleanup fails.
                stopping = true;
                if let Err(error) = runtime.stop() {
                    println!("TESTPILOT ERROR: cleanup failed: {error}");
                }
            }
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
