use log::{debug, error};
use panel::EventSimPanel;
use sim::{AircraftSimState, SimClientEvent, SimCommunicator};
use std::sync::mpsc;
use std::{process, thread};

use crate::config::Config;

mod config;
mod panel;
mod sim;

#[derive(Debug)]
pub enum Event {
    /// The hardware state of the panel changed.
    SetSimulator(SimClientEvent),
    /// The simulator aircraft state changed.
    SetPanel(AircraftSimState),
}

fn try_main(config: Config) -> Result<(), Box<dyn std::error::Error>> {
    debug!("Using config {:?}", config);

    let eventsim_port = config
        .eventsim_port()
        .ok_or("EventSim port unspecified in config")?;

    let (sim_tx, sim_rx) = mpsc::channel();
    let (hw_tx, hw_rx) = mpsc::channel();

    let sim_handle = thread::spawn(move || SimCommunicator::new(sim_tx, hw_rx).run());
    let panel_handle =
        thread::spawn(move || EventSimPanel::new(eventsim_port, hw_tx, sim_rx).run());

    panel_handle
        .join()
        .expect("Couldn't join on the associated thread")?;
    sim_handle
        .join()
        .expect("Couldn't join on the associated thread");

    Ok(())
}

fn main() {
    // Parse the app configuration
    let config = Config::from_file("config.toml").unwrap_or_else(|e| {
        eprintln!("{e}");
        process::exit(1)
    });

    // Override the log level based on the configuration
    let level = config.log_level.as_str();
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(level)).init();

    // Run the application
    if let Err(e) = try_main(config) {
        error!("{e}");
    }
}
