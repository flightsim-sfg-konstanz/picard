use std::io::Write;
use std::time::Instant;
use std::{
    io::{BufRead, BufReader},
    sync::mpsc,
    time::Duration,
};

use log::info;

use crate::sim::SimClientEvent;
use crate::Event;

/// The baud rate of the Arduino used for the serial connection.
const BAUD_RATE: u32 = 115200;

pub fn run_panel(port_path: &str, hw_tx: mpsc::Sender<Event>, sim_rx: mpsc::Receiver<Event>) {
    // Open serial port
    info!("Attempting to connect to serial port {}", port_path);
    let mut serial = serialport::new(port_path, BAUD_RATE)
        .timeout(Duration::from_millis(10))
        .open()
        .expect("Failed to open port");

    let reader = BufReader::with_capacity(1, serial.try_clone().unwrap());
    let mut aircraft_sim_state = Option::None;
    let mut line_reader = reader.lines();
    let mut et = Instant::now();
    let mut connected = false;

    // Initiate handshake with the Arduino
    writeln!(serial, "SYN").unwrap();

    loop {
        // Receive control messages
        if connected {
            match sim_rx.try_recv() {
                Ok(Event::SetPanel(state)) => {
                    if aircraft_sim_state
                        .map(|old_state| old_state != state)
                        .unwrap_or(true)
                    {
                        state.send_state(&mut serial).unwrap();
                    }
                    aircraft_sim_state = Some(state);
                }
                Err(mpsc::TryRecvError::Disconnected) => panic!("fuck me"),
                _ => {}
            }
        }

        // Read messages from serial port
        if let Some(msg) = line_reader.next() {
            if let Ok(msg) = &msg {
                info!("Serial port received: {:?}", msg);
            };
            match msg.as_deref() {
                Ok("SYN|ACK") => {
                    writeln!(serial, "ACK").unwrap();
                    info!("Connected with panel established via {}", port_path);
                    connected = true;
                }
                Ok("RST") => {
                    drop(hw_tx);
                    panic!("panel closed connection");
                }
                Ok(cmd) => match cmd {
                    "MISC1:0" => hw_tx
                        .send(Event::SetSimulator(SimClientEvent::TaxiLightsOff))
                        .unwrap(),
                    "MISC1:1" => hw_tx
                        .send(Event::SetSimulator(SimClientEvent::TaxiLightsOn))
                        .unwrap(),
                    "MISC2:0" => hw_tx
                        .send(Event::SetSimulator(SimClientEvent::LandingLightsOff))
                        .unwrap(),
                    "MISC2:1" => hw_tx
                        .send(Event::SetSimulator(SimClientEvent::LandingLightsOn))
                        .unwrap(),
                    "MISC3:0" => hw_tx
                        .send(Event::SetSimulator(SimClientEvent::NavLightsOff))
                        .unwrap(),
                    "MISC3:1" => hw_tx
                        .send(Event::SetSimulator(SimClientEvent::NavLightsOn))
                        .unwrap(),
                    "MISC4:0" => hw_tx
                        .send(Event::SetSimulator(SimClientEvent::StrobeLightsOff))
                        .unwrap(),
                    "MISC4:1" => hw_tx
                        .send(Event::SetSimulator(SimClientEvent::StrobeLightsOn))
                        .unwrap(),
                    "FLAPS_UP" => hw_tx
                        .send(Event::SetSimulator(SimClientEvent::FlapsUp))
                        .unwrap(),
                    "FLAPS_DN" => hw_tx
                        .send(Event::SetSimulator(SimClientEvent::FlapsDown))
                        .unwrap(),
                    "PARKING_BRAKE:0" => {
                        hw_tx
                            .send(Event::SetSimulator(SimClientEvent::ParkingBrakeOff))
                            .unwrap();
                    }
                    "PARKING_BRAKE:1" => {
                        hw_tx
                            .send(Event::SetSimulator(SimClientEvent::ParkingBrakeOn))
                            .unwrap();
                    }
                    "LANDING_GEAR:0" => {
                        hw_tx
                            .send(Event::SetSimulator(SimClientEvent::LandingGearUp))
                            .unwrap();
                    }
                    "LANDING_GEAR:1" => {
                        hw_tx
                            .send(Event::SetSimulator(SimClientEvent::LandingGearDown))
                            .unwrap();
                    }
                    _ => {}
                },
                Err(e) => {
                    if e.kind() != std::io::ErrorKind::TimedOut {
                        info!("{:?}", e)
                    }
                }
            }
        }

        // Send keepalive packets
        let now = Instant::now();
        if now > et + Duration::from_millis(500) {
            writeln!(serial, "PING").unwrap();
            et = now;
        }
    }
}
