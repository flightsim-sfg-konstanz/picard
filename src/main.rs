use log::{debug, error};
use panel::EventSimPanel;
use sim::{AircraftSimState, SimClientEvent, SimCommunicator};
use std::sync::mpsc;
use std::{fs, thread};

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

fn try_main() -> Result<(), Box<dyn std::error::Error>> {
    let config: Config = toml::from_str(&fs::read_to_string("config.toml")?)?;
    debug!("Using config {:?}", config);
    let eventsim_port = config
        .eventsim_port()
        .ok_or("EventSim port unspecified in config")?;

    let (sim_tx, sim_rx) = mpsc::channel();
    let (hw_tx, hw_rx) = mpsc::channel();

    let sim_handle = thread::spawn(move || SimCommunicator::new(sim_tx, hw_rx).run());
    let panel_handle =
        thread::spawn(move || EventSimPanel::new(eventsim_port, hw_tx, sim_rx).run());

    sim_handle
        .join()
        .expect("Couldn't join on the associated thread");
    panel_handle
        .join()
        .expect("Couldn't join on the associated thread")?;

    Ok(())
}

fn main() {
    // Setup logging output
    env_logger::init();

    // Run the application
    if let Err(e) = try_main() {
        error!("{e}");
    }
}
