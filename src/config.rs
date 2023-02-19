use std::{collections::HashMap, fs, path::Path};

use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
pub struct Config {
    pub log_level: log::LevelFilter,
    panels: HashMap<String, Panel>,
}

impl Config {
    pub fn from_file(path: impl AsRef<Path>) -> Result<Self, Box<dyn std::error::Error>> {
        let config_content = fs::read_to_string(path)
            .map_err(|e| format!("Failed to open config file 'config.toml': {e}"))?;
        let config: Config = toml::from_str(&config_content)
            .map_err(|e| format!("Failed to parse configuration: {e}"))?;
        Ok(config)
    }

    pub fn eventsim_port(&self) -> Option<String> {
        self.panels.get("eventsim").map(|panel| panel.port.clone())
    }
}

#[derive(Debug, Serialize, Deserialize)]
struct Panel {
    port: String,
}
