use log::{error, info};
use panel::run_panel;
use sim::{AircraftSimState, SimClientEvent, SimCommunicator};
use std::sync::mpsc;
use std::thread;

mod panel;
mod sim;

#[derive(Debug)]
pub enum Event {
    /// The hardware state of the panel changed.
    SetSimulator(SimClientEvent),
    /// The simulator aircraft state changed.
    SetPanel(AircraftSimState),
}

fn try_main(port_path: String) -> Result<(), Box<dyn std::error::Error>> {
    let (sim_tx, sim_rx) = mpsc::channel();
    let (hw_tx, hw_rx) = mpsc::channel();

    let mut sim_communicator = SimCommunicator::new(sim_tx, hw_rx);
    let sim_handle = thread::spawn(move || sim_communicator.run());
    let panel_handle = thread::spawn(move || run_panel(&port_path, hw_tx, sim_rx));

    sim_handle
        .join()
        .expect("Couldn't join on the associated thread");
    panel_handle
        .join()
        .expect("Couldn't join on the associated thread");

    Ok(())
}

fn main() {
    // Setup logging output
    env_logger::init();

    // Parse commandline arguments such as COM port
    let mut args = std::env::args();
    if args.len() < 2 {
        // Print available serial ports
        let ports = serialport::available_ports().expect("No ports found!");
        for p in ports {
            info!("{}", p.port_name);
        }
        return;
    }
    let port_path = args.nth(1).unwrap();

    // Run the application
    if let Err(e) = try_main(port_path) {
        error!("Fatal error: {}", e);
    }
}
