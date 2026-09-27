use std::error::Error;

use msfs::MSFSEvent;

use crate::aircraft_initialisation::A32nxInitialiser;
use crate::{gauge_runtime::GaugeRuntime, replayer::Replayer, simulator::MsfsSimulator};

#[msfs::gauge(name=testpilot)]
/// MSFS gauge entrypoint that drives the replay runtime each `PreUpdate`.
async fn testpilot(mut gauge: msfs::Gauge) -> Result<(), Box<dyn Error>> {
    let mut runtime = match GaugeRuntime::new(
        Replayer::new(),
        Box::new(MsfsSimulator::new()),
        Box::new(A32nxInitialiser::default()),
    ) {
        Ok(runtime) => runtime,
        Err(error) => {
            println!("TESTPILOT ERROR: runtime setup failed: {error}");
            // Setup failures must also await PostKill stream closure: returning
            // early leaves msfs-rs polling a completed future on the next event.
            while gauge.next_event().await.is_some() {}
            return Ok(());
        }
    };
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
                    // Terminal failures suppress further updates. Keep the future alive
                    // until event-stream closure, just as for PreKill, to avoid a reload panic.
                    stopping = true;
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
