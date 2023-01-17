use log::error;
use panel::EventSimPanel;
use serialport::SerialPortType;
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

fn run(port: String) -> Result<(), Box<dyn std::error::Error>> {
    let (sim_tx, sim_rx) = mpsc::channel();
    let (hw_tx, hw_rx) = mpsc::channel();

    let sim_handle = thread::spawn(move || SimCommunicator::new(sim_tx, hw_rx).run());
    let panel_handle = thread::spawn(move || EventSimPanel::new(port, hw_tx, sim_rx).run());

    sim_handle
        .join()
        .expect("Couldn't join on the associated thread");
    panel_handle
        .join()
        .expect("Couldn't join on the associated thread")?;

    Ok(())
}

/// List all available serial ports and their information.
///
/// Slightly adapted from https://github.com/serialport/serialport-rs/blob/main/examples/list_ports.rs.
fn list_serialports() -> Result<(), Box<dyn std::error::Error>> {
    let ports = serialport::available_ports()?;
    match ports.len() {
        0 => println!("No ports found."),
        1 => println!("Found 1 port:"),
        n => println!("Found {} ports:", n),
    };
    for p in ports {
        println!("{}", p.port_name);
        match p.port_type {
            SerialPortType::UsbPort(info) => {
                println!("    Type: USB");
                println!("    VID:{:04x} PID:{:04x}", info.vid, info.pid);
                println!(
                    "    Serial Number: {}",
                    info.serial_number.as_ref().map_or("", String::as_str)
                );
                println!(
                    "    Manufacturer: {}",
                    info.manufacturer.as_ref().map_or("", String::as_str)
                );
                println!(
                    "    Product: {}",
                    info.product.as_ref().map_or("", String::as_str)
                );
            }
            SerialPortType::BluetoothPort => {
                println!("    Type: Bluetooth");
            }
            SerialPortType::PciPort => {
                println!("    Type: PCI");
            }
            SerialPortType::Unknown => {
                println!("    Type: Unknown");
            }
        }
    }
    Ok(())
}

fn try_main() -> Result<(), Box<dyn std::error::Error>> {
    match std::env::args().nth(1) {
        Some(port) => run(port),
        None => list_serialports(),
    }
}

fn main() {
    // Setup logging output
    env_logger::init();

    // Run the application
    if let Err(e) = try_main() {
        error!("{e}");
    }
}
