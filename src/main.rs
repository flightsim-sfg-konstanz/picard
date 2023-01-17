use log::{error, info};
use serialport::SerialPort;
use simconnect_sdk::{FlxClientEvent, Notification, SimConnect, SimConnectError, SimConnectObject};
use std::io::Write;
use std::thread;
use std::time::Instant;
use std::{
    io::{BufRead, BufReader},
    sync::mpsc,
    time::Duration,
};

/// The baud rate of the Arduino used for the serial connection.
const BAUD_RATE: u32 = 115200;

/// A data structure that will be used to receive data from SimConnect.
/// See the documentation of `SimConnectObject` for more information on the arguments of the `simconnect` attribute.
#[derive(Debug, Clone, SimConnectObject)]
#[simconnect(period = "sim-frame", condition = "changed")]
struct AircraftSimData {
    #[simconnect(name = "GEAR CENTER POSITION", unit = "percent over 100")]
    gear_center_position: f64,
    #[simconnect(name = "GEAR LEFT POSITION", unit = "percent over 100")]
    gear_left_position: f64,
    #[simconnect(name = "GEAR RIGHT POSITION", unit = "percent over 100")]
    gear_right_position: f64,

    /// Parking brake indicator.
    ///
    /// WARNING: Must be the last entry in the struct due to a bug in the `simconnect-sdk` crate, otherwise the gear
    /// position values are interpreted incorrectly.
    #[simconnect(name = "BRAKE PARKING INDICATOR")]
    parking_brake_indicator: bool,
}

#[derive(Debug, PartialEq, Eq)]
struct AircraftSimState {
    parking_brake_indicator: bool,
    gear_center_state: LandingGearStatus,
    gear_left_state: LandingGearStatus,
    gear_right_state: LandingGearStatus,
}

impl From<AircraftSimData> for AircraftSimState {
    fn from(value: AircraftSimData) -> Self {
        Self {
            parking_brake_indicator: value.parking_brake_indicator,
            gear_center_state: value.gear_center_position.into(),
            gear_left_state: value.gear_left_position.into(),
            gear_right_state: value.gear_right_position.into(),
        }
    }
}

impl AircraftSimState {
    fn send_state(&self, tx: &mut Box<dyn SerialPort>) -> Result<(), std::io::Error> {
        writeln!(tx, "PARKING_BRAKE:{}", self.parking_brake_indicator as i32)?;
        writeln!(tx, "FRONT_GEAR_LED:{}", self.gear_center_state.as_int())?;
        writeln!(tx, "LEFT_GEAR_LED:{}", self.gear_left_state.as_int())?;
        writeln!(tx, "RIGHT_GEAR_LED:{}", self.gear_right_state.as_int())?;
        Ok(())
    }
}

#[derive(Debug, PartialEq, Eq)]
enum LandingGearStatus {
    Unknown,
    Up,
    Down,
}

impl From<f64> for LandingGearStatus {
    fn from(value: f64) -> Self {
        if value == 0.0 {
            Self::Up
        } else if value == 1.0 {
            Self::Down
        } else {
            Self::Unknown
        }
    }
}

impl LandingGearStatus {
    fn as_int(&self) -> i32 {
        match self {
            LandingGearStatus::Up => 0,
            LandingGearStatus::Down => 1,
            LandingGearStatus::Unknown => 2,
        }
    }
}

#[derive(Debug)]
enum Event {
    /// The hardware state of the panel changed.
    SetSimulator(SimClientEvent),
    /// The simulator aircraft state changed.
    SetPanel(AircraftSimState),
}

#[derive(Debug, Clone, Copy)]
#[repr(u32)]
enum SimClientEvent {
    LandingLightsOn,
    LandingLightsOff,
    TaxiLightsOn,
    TaxiLightsOff,
    StrobeLightsOn,
    StrobeLightsOff,
    NavLightsOn,
    NavLightsOff,
    FlapsUp,
    FlapsDown,
    ParkingBrakeOn,
    ParkingBrakeOff,
    LandingGearUp,
    LandingGearDown,
}

impl FlxClientEvent for SimClientEvent {
    fn event_id(&self) -> u32 {
        *self as u32
    }

    fn event_name(&self) -> *const std::ffi::c_char {
        (match self {
            SimClientEvent::LandingLightsOn => "LANDING_LIGHTS_ON\0",
            SimClientEvent::LandingLightsOff => "LANDING_LIGHTS_OFF\0",
            SimClientEvent::TaxiLightsOn => "TAXI_LIGHTS_ON\0",
            SimClientEvent::TaxiLightsOff => "TAXI_LIGHTS_OFF\0",
            SimClientEvent::StrobeLightsOn => "STROBES_ON\0",
            SimClientEvent::StrobeLightsOff => "STROBES_OFF\0",
            SimClientEvent::NavLightsOn => "NAV_LIGHTS_ON\0",
            SimClientEvent::NavLightsOff => "NAV_LIGHTS_OFF\0",
            SimClientEvent::FlapsUp => "FLAPS_DECR\0",
            SimClientEvent::FlapsDown => "FLAPS_INCR\0",
            SimClientEvent::ParkingBrakeOn => "PARKING_BRAKE_SET\0",
            SimClientEvent::ParkingBrakeOff => "PARKING_BRAKE_SET\0",
            SimClientEvent::LandingGearUp => "GEAR_UP\0",
            SimClientEvent::LandingGearDown => "GEAR_DOWN\0",
        })
        .as_ptr() as *const std::ffi::c_char
    }

    fn data(&self) -> u32 {
        match self {
            SimClientEvent::ParkingBrakeOn => 1,
            SimClientEvent::ParkingBrakeOff => 0,
            _ => 0,
        }
    }
}

struct SimCommunicator {
    connected: bool,
    sim_tx: mpsc::Sender<Event>,
    hw_rx: mpsc::Receiver<Event>,
}

impl SimCommunicator {
    pub fn new(sim_tx: mpsc::Sender<Event>, hw_rx: mpsc::Receiver<Event>) -> Self {
        Self {
            connected: false,
            sim_tx,
            hw_rx,
        }
    }

    pub fn run(&mut self) {
        loop {
            info!("Attempting to connect via SimConnect");
            match SimConnect::new("FSSK EventSim Main Panel") {
                Ok(client) => {
                    if let Err(e) = self.run_event_loop(client) {
                        error!("SimConnect communication error: {:?}", e);
                    }
                }
                Err(e) => {
                    error!("Failed to connect via SimConnect: {:?}", e);
                }
            }

            // We are now disconnected
            self.connected = false;

            // Wait before reconnecting
            std::thread::sleep(Duration::from_secs(5));
        }
    }

    fn run_event_loop(&mut self, mut client: SimConnect) -> Result<(), SimConnectError> {
        loop {
            // Receive control messages if we are connected
            if self.connected {
                match self.hw_rx.try_recv() {
                    Ok(Event::SetSimulator(event)) => client.transmit_event(event)?,
                    Err(mpsc::TryRecvError::Disconnected) => panic!("fuck me"),
                    _ => {}
                }
            }

            match client.get_next_dispatch()? {
                Some(Notification::Open) => {
                    info!("SimConnect connection opened");
                    // After the connection is successfully open, we register the aircraft data struct
                    client.register_object::<AircraftSimData>()?;
                    // We register the events we want to send to the simulator
                    client.map_client_event_to_sim_event(SimClientEvent::LandingLightsOn)?;
                    client.map_client_event_to_sim_event(SimClientEvent::LandingLightsOff)?;
                    client.map_client_event_to_sim_event(SimClientEvent::TaxiLightsOn)?;
                    client.map_client_event_to_sim_event(SimClientEvent::TaxiLightsOff)?;
                    client.map_client_event_to_sim_event(SimClientEvent::StrobeLightsOn)?;
                    client.map_client_event_to_sim_event(SimClientEvent::StrobeLightsOff)?;
                    client.map_client_event_to_sim_event(SimClientEvent::NavLightsOn)?;
                    client.map_client_event_to_sim_event(SimClientEvent::NavLightsOff)?;
                    client.map_client_event_to_sim_event(SimClientEvent::FlapsUp)?;
                    client.map_client_event_to_sim_event(SimClientEvent::FlapsDown)?;
                    client.map_client_event_to_sim_event(SimClientEvent::ParkingBrakeOn)?;
                    client.map_client_event_to_sim_event(SimClientEvent::ParkingBrakeOff)?;
                    client.map_client_event_to_sim_event(SimClientEvent::LandingGearUp)?;
                    client.map_client_event_to_sim_event(SimClientEvent::LandingGearDown)?;

                    // We are now successfully connected
                    self.connected = true;
                }
                Some(Notification::Quit) => {
                    info!("SimConnect connection quit");
                    return Ok(());
                }
                Some(Notification::Object(data)) => {
                    let aircraft_state = AircraftSimData::try_from(&data).unwrap();
                    info!("Received SimConnect aircraft state {:?}", aircraft_state);
                    self.sim_tx
                        .send(Event::SetPanel(aircraft_state.into()))
                        .expect("Failed to send to the control thread");
                }
                Some(unkn) => {
                    dbg!(unkn);
                }
                _ => {}
            }

            // Sleep for about a frame to reduce CPU usage
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    }
}

fn run_panel(port_path: &str, hw_tx: mpsc::Sender<Event>, sim_rx: mpsc::Receiver<Event>) {
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
                    connected = false;
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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    let mut args = std::env::args();
    if args.len() < 2 {
        // Print available serial ports
        let ports = serialport::available_ports().expect("No ports found!");
        for p in ports {
            info!("{}", p.port_name);
        }
        return Ok(());
    }
    let port_path = args.nth(1).unwrap();

    let (sim_tx, sim_rx) = mpsc::channel();
    let (hw_tx, hw_rx) = mpsc::channel();

    let mut sim_communicator = SimCommunicator::new(sim_tx, hw_rx);
    let sim_handle = thread::spawn(move || sim_communicator.run());
    let panel_handle = thread::spawn(move || run_panel(&port_path, hw_tx, sim_rx));

    sim_handle.join().unwrap();
    panel_handle.join().unwrap();

    Ok(())
}
