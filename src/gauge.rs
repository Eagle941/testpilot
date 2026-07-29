use std::error::Error;
use std::path::Path;

use anyhow::{Context, anyhow, bail};
use msfs::{MSFSEvent, legacy::NamedVariable};

use crate::arm::ArmState;
use crate::config::{CONFIG_PATH, read_config_file};
use crate::scenario::ScenarioPlayback;

const ARMED_VARIABLE: &str = "REPLAYER_ARMED";

#[msfs::gauge(name=replayer)]
async fn replayer(mut gauge: msfs::Gauge) -> Result<(), Box<dyn Error>> {
    let mut replayer = ReplayerGauge::new();

    while let Some(event) = gauge.next_event().await {
        let result = match event {
            MSFSEvent::PreUpdate => replayer.pre_update(),
            MSFSEvent::PreKill => {
                replayer.reset();
                Ok(())
            }
            _ => Ok(()),
        };

        if let Err(error) = result {
            replayer.report_failure(&error);
            return Err(error.into());
        }
    }

    replayer.reset();
    Ok(())
}

struct ReplayerGauge {
    armed_variable: NamedVariable,
    arm_state: ArmState,
    scenario: Option<ScenarioPlayback>,
}

impl ReplayerGauge {
    fn new() -> Self {
        let armed_variable = NamedVariable::from(ARMED_VARIABLE);
        armed_variable.set_value(0.0);
        println!("REPLAYER: waiting for L:REPLAYER_ARMED = 1");

        Self {
            armed_variable,
            arm_state: ArmState::default(),
            scenario: None,
        }
    }

    fn pre_update(&mut self) -> anyhow::Result<()> {
        if self.arm_state.start(self.armed_variable.get_value::<f64>()) {
            self.begin_scenario()?;
            return Ok(());
        }

        self.prime_scenario()
    }

    fn begin_scenario(&mut self) -> anyhow::Result<()> {
        if self.scenario.is_some() {
            bail!("a replay scenario is already loaded");
        }

        let config = read_config_file(CONFIG_PATH)?;
        let config_directory = Path::new(CONFIG_PATH)
            .parent()
            .ok_or_else(|| anyhow!("configuration path `{CONFIG_PATH}` has no parent directory"))?;
        let scenario_path = config_directory.join(&config.input_file);
        let playback = ScenarioPlayback::open(&scenario_path, &config)
            .with_context(|| format!("failed to open scenario `{}`", scenario_path.display()))?;

        println!(
            "REPLAYER: opened {} with {} signal cursors",
            scenario_path.display(),
            playback.signal_count()
        );
        self.scenario = Some(playback);
        Ok(())
    }

    fn prime_scenario(&mut self) -> anyhow::Result<()> {
        let Some(playback) = self.scenario.as_mut() else {
            return Ok(());
        };
        if playback.read_frame()? {
            println!("REPLAYER: scenario cursors ready");
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.scenario = None;
        self.armed_variable.set_value(0.0);
    }

    fn report_failure(&mut self, error: &anyhow::Error) {
        eprintln!("REPLAYER: {error:#}");
        self.reset();
    }
}
