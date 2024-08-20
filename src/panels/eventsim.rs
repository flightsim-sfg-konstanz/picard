use log::debug;
use log::info;
use serialport::SerialPort;
use std::io::BufRead;
use std::io::BufReader;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;
use std::time::Instant;

use crate::panel::Panel;
use crate::panel::PanelError;
use crate::sim::AircraftSimState;
use crate::sim::SimClientEvent;
use crate::Event;

/// The baud rate of the Arduino used for the serial connection.
const BAUD_RATE: u32 = 115200;

/// Represents the EventSim Main Panel and holds all state and information.
#[derive(Debug)]
pub struct EventSimPanel {
    port: String,
    connected: bool,
    hw_tx: mpsc::Sender<Event>,
    sim_rx: mpsc::Receiver<Event>,
    aircraft_sim_state: Option<AircraftSimState>,
}

impl Panel for EventSimPanel {
    /// Connect to the panel and run an event loop.
    fn run(&mut self) -> Result<(), PanelError> {
        debug!(
            "Attempting to connect to panel on serial port {}",
            self.port
        );
        let mut serial = serialport::new(&self.port, BAUD_RATE)
            .timeout(Duration::from_millis(10))
            .open()
            .map_err(|e| PanelError::SerialOpen(self.port.clone(), e))?;

        // Reset device
        serial.write_data_terminal_ready(true)?;
        serial.clear(serialport::ClearBuffer::All)?;
        // Wait for device to finish resetting
        thread::sleep(Duration::from_millis(2000));

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
                            send_state(&state, &mut serial)?;
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
                            info!(
                                "Connection with EventSim panel established via {}",
                                self.port
                            );
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
}

impl EventSimPanel {
    /// Create a new panel instance.
    pub fn new(
        port: impl AsRef<str>,
        hw_tx: mpsc::Sender<Event>,
        sim_rx: mpsc::Receiver<Event>,
    ) -> Self {
        Self {
            connected: false,
            hw_tx,
            sim_rx,
            port: port.as_ref().into(),
            aircraft_sim_state: None,
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

fn send_state(
    state: &AircraftSimState,
    tx: &mut Box<dyn SerialPort>,
) -> Result<(), std::io::Error> {
    writeln!(tx, "PARKING_BRAKE:{}", state.parking_brake_indicator as i32)?;
    writeln!(tx, "FRONT_GEAR_LED:{}", state.gear_center_state.as_int())?;
    writeln!(tx, "LEFT_GEAR_LED:{}", state.gear_left_state.as_int())?;
    writeln!(tx, "RIGHT_GEAR_LED:{}", state.gear_right_state.as_int())?;
    Ok(())
}
