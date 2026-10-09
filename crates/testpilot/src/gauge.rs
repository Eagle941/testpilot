use std::error::Error;

use msfs::MSFSEvent;

use testpilot_a32nx::A32nxInitialiser;
use testpilot_core::runtime::Runtime;
use testpilot_msfs::MsfsSimulator;

use crate::config::CONFIG_PATH;

#[msfs::gauge(name=testpilot)]
/// MSFS gauge entrypoint that drives the replay runtime each `PreUpdate`.
async fn testpilot(mut gauge: msfs::Gauge) -> Result<(), Box<dyn Error>> {
    let mut runtime = match Runtime::new(
        CONFIG_PATH,
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
                    println!("TESTPILOT ERROR: {error:#}");
                    // Cleanup has returned the runtime to Idle. Keep forwarding updates
                    // so a new arming transition can start another run.
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
