use std::collections::HashMap;

use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
pub struct Config {
    panels: HashMap<String, Panel>,
}

impl Config {
    pub fn eventsim_port(&self) -> Option<String> {
        self.panels.get("eventsim").map(|panel| panel.port.clone())
    }
}

#[derive(Debug, Serialize, Deserialize)]
struct Panel {
    port: String,
}
