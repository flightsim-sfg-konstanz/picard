use core::fmt;
use log::{debug, info};
use std::io::Write;
use std::time::Instant;
use std::{
    io::{BufRead, BufReader},
    sync::mpsc,
    time::Duration,
};

use crate::sim::{AircraftSimState, SimClientEvent};
use crate::Event;

/// The baud rate of the Arduino used for the serial connection.
const BAUD_RATE: u32 = 115200;

/// Errors related to the panel.
#[derive(Debug)]
pub enum PanelError {
    /// Failed to open the serial port
    SerialOpen(String, serialport::Error),
    /// The panel was or is disconnected
    Disconnect,
    /// Error that relates to the serial port
    Serial(serialport::Error),
    /// I/O error that wraps the standard error type
    Io(std::io::Error),
}

impl fmt::Display for PanelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PanelError::SerialOpen(port, e) => {
                write!(
                    f,
                    "Failed to connect with panel on serial port '{port}': {e}"
                )
            }
            PanelError::Disconnect => write!(f, "The panel disconnected"),
            PanelError::Serial(e) => write!(f, "Serial communication error: {}", e),
            PanelError::Io(e) => write!(f, "Panel I/O error: {}", e),
        }
    }
}

impl std::error::Error for PanelError {}

impl From<serialport::Error> for PanelError {
    fn from(value: serialport::Error) -> Self {
        Self::Serial(value)
    }
}

impl From<std::io::Error> for PanelError {
    fn from(value: std::io::Error) -> Self {
        Self::Io(value)
    }
}

/// Represents the EventSim Main Panel and holds all state and information.
#[derive(Debug)]
pub struct EventSimPanel {
    connected: bool,
    hw_tx: mpsc::Sender<Event>,
    sim_rx: mpsc::Receiver<Event>,
    port_path: String,
    aircraft_sim_state: Option<AircraftSimState>,
}

impl EventSimPanel {
    /// Create a new panel instance.
    pub fn new(
        port_path: String,
        hw_tx: mpsc::Sender<Event>,
        sim_rx: mpsc::Receiver<Event>,
    ) -> Self {
        Self {
            connected: false,
            hw_tx,
            sim_rx,
            port_path,
            aircraft_sim_state: None,
        }
    }

    /// Connect to the panel and run an event loop.
    pub fn run(&mut self) -> Result<(), PanelError> {
        debug!(
            "Attempting to connect to panel on serial port {}",
            self.port_path
        );
        let mut serial = serialport::new(&self.port_path, BAUD_RATE)
            .timeout(Duration::from_millis(10))
            .open()
            .map_err(|e| PanelError::SerialOpen(self.port_path.clone(), e))?;

        let reader = BufReader::with_capacity(1, serial.try_clone()?);
        let mut line_reader = reader.lines();
        let mut et = Instant::now();

        // Initiate handshake with the Arduino
        writeln!(serial, "SYN")?;

        loop {
            // Receive control messages
            if self.connected {
                match self.sim_rx.try_recv() {
                    Ok(Event::SetPanel(state)) => {
                        // Send aircraft state only if it has changed since the last time.
                        // FIXME: This is very inefficient because we always transmit the full state
                        if self
                            .aircraft_sim_state
                            .as_ref()
                            .map(|old_state| old_state != &state)
                            .unwrap_or(true)
                        {
                            state.send_state(&mut serial)?;
                        }
                        self.aircraft_sim_state = Some(state);
                    }
                    Err(mpsc::TryRecvError::Disconnected) => {
                        // The simconnect thread cannot exit, we exit always first
                        unreachable!();
                    }
                    _ => {}
                }
            }

            // Read messages from serial port
            if let Some(msg) = line_reader.next() {
                match msg {
                    Ok(msg) => match msg.as_str() {
                        "SYN|ACK" => {
                            writeln!(serial, "ACK")?;
                            info!("Connection with panel established via {}", self.port_path);
                            self.connected = true;
                        }
                        "RST" => return Err(PanelError::Disconnect),
                        "PING" => writeln!(serial, "PONG")?,
                        "PONG" => {}
                        cmd => self.handle_serial_command(cmd),
                    },
                    // Ignore timouts
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                    // Exit on all other errors
                    Err(e) => return Err(e.into()),
                }
            }

            // Send keepalive packets
            let now = Instant::now();
            if now > et + Duration::from_millis(500) {
                writeln!(serial, "PING")?;
                et = now;
            }
        }
    }

    fn handle_serial_command(&self, cmd: &str) {
        debug!("Serial port received command: {:?}", cmd);
        let event = match cmd {
            "MISC1:0" => SimClientEvent::TaxiLightsOff,
            "MISC1:1" => SimClientEvent::TaxiLightsOn,
            "MISC2:0" => SimClientEvent::LandingLightsOff,
            "MISC2:1" => SimClientEvent::LandingLightsOn,
            "MISC3:0" => SimClientEvent::NavLightsOff,
            "MISC3:1" => SimClientEvent::NavLightsOn,
            "MISC4:0" => SimClientEvent::StrobeLightsOff,
            "MISC4:1" => SimClientEvent::StrobeLightsOn,
            "FLAPS_UP" => SimClientEvent::FlapsUp,
            "FLAPS_DN" => SimClientEvent::FlapsDown,
            "PARKING_BRAKE:0" => SimClientEvent::ParkingBrakeOff,
            "PARKING_BRAKE:1" => SimClientEvent::ParkingBrakeOn,
            "LANDING_GEAR:0" => SimClientEvent::LandingGearUp,
            "LANDING_GEAR:1" => SimClientEvent::LandingGearDown,
            _ => return,
        };
        self.hw_tx
            .send(Event::SetSimulator(event))
            .expect("SimConnect thread offline");
    }
}
