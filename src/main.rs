use log::{debug, error};
use panel::Panel;
use sim::{AircraftSimState, SimClientEvent, SimCommunicator};
use std::sync::mpsc;
use std::{process, thread};

use crate::config::Config;
use crate::panels::airspeedindicator::AirspeedIndicatorPanel;
use crate::panels::eventsim::EventSimPanel;

mod config;
mod panel;
mod panels;
mod sim;

#[derive(Debug)]
pub enum Event {
    /// The hardware state of the panel changed.
    SetSimulator(SimClientEvent),
    /// The simulator aircraft state changed.
    SetPanel(AircraftSimState),
}

fn run(config: Config) {
    // Channel to transmit from hardware panels to the SimConnect client
    let (hw_tx, hw_rx) = mpsc::channel();

    let mut panels: Vec<Box<dyn Panel>> = Vec::new();
    let mut sim_txs = Vec::new();

    // Initialization of EventSim panel
    if let Some(port) = config.eventsim_port() {
        let (sim_tx, sim_rx) = mpsc::channel();
        let panel = EventSimPanel::new(port, hw_tx.clone(), sim_rx);
        panels.push(Box::new(panel));
        sim_txs.push(sim_tx);
    };

    // Initialization of airspeed indicator
    if let Some(port) = config.airspeedindicator_port() {
        let (sim_tx, sim_rx) = mpsc::channel();
        let panel = AirspeedIndicatorPanel::new(port, sim_rx);
        panels.push(Box::new(panel));
        sim_txs.push(sim_tx);
    };

    // Start threads
    let mut handles = Vec::new();
    for mut panel in panels {
        handles.push(thread::spawn(move || {
            if let Err(e) = panel.run() {
                error!("{e}");
            }
        }));
    }
    handles.push(thread::spawn(move || {
        SimCommunicator::new(sim_txs, hw_rx).run()
    }));

    for handle in handles {
        let _ = handle.join();
    }
}

fn main() {
    // Parse the app configuration
    let config = Config::from_file("config.toml").unwrap_or_else(|e| {
        eprintln!("{e}");
        process::exit(1)
    });
    debug!("Using config {:?}", config);

    // Override the log level based on the configuration
    let level = config.log_level.as_str();
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(level)).init();

    // Run the application
    run(config);
}
