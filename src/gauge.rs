use std::error::Error;

#[msfs::gauge(name=replayer)]
async fn replayer(mut gauge: msfs::Gauge) -> Result<(), Box<dyn Error>> {
    while gauge.next_event().await.is_some() {}
    Ok(())
}
