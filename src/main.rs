use serialport::SerialPort;
use simconnect_sdk::{FlxClientEvent, Notification, SimConnect, SimConnectObject};
use std::io::Write;
use std::{
    io::{BufRead, BufReader},
    sync::{atomic::AtomicBool, mpsc},
    time::Duration,
};

/// The baud rate of the Arduino used for the serial connection.
const BAUD_RATE: u32 = 115200;

/// A data structure that will be used to receive data from SimConnect.
/// See the documentation of `SimConnectObject` for more information on the arguments of the `simconnect` attribute.
#[derive(Debug, Clone, SimConnectObject)]
#[simconnect(period = "sim-frame", condition = "changed", interval = 50)]
#[allow(dead_code)]
struct AirplaneData {
    #[simconnect(name = "GEAR CENTER POSITION", unit = "percent over 100")]
    gear_center_position: f64,
    #[simconnect(name = "GEAR LEFT POSITION", unit = "percent over 100")]
    gear_left_position: f64,
    #[simconnect(name = "GEAR RIGHT POSITION", unit = "percent over 100")]
    gear_right_position: f64,

    /// Parking brake indicator.
    ///
    /// WARNING: Must be the last entry in the struct due to a bug in the `simconnect-sdk` crate, otherwise the gear position values are interpreted false.
    #[simconnect(name = "BRAKE PARKING INDICATOR")]
    parking_brake_indicator: bool,
}

#[derive(Debug)]
struct SimStatus {
    parking_brake_indicator: bool,
    gear_center_state: LandingGearStatus,
    gear_left_state: LandingGearStatus,
    gear_right_state: LandingGearStatus,
}

impl From<AirplaneData> for SimStatus {
    fn from(value: AirplaneData) -> Self {
        Self {
            parking_brake_indicator: value.parking_brake_indicator,
            gear_center_state: value.gear_center_position.into(),
            gear_left_state: value.gear_left_position.into(),
            gear_right_state: value.gear_left_position.into(),
        }
    }
}

impl SimStatus {
    fn send(&self, tx: &mut Box<dyn SerialPort>) -> Result<(), std::io::Error> {
        writeln!(tx, "PARKING_BRAKE:{}", self.parking_brake_indicator as i32)?;
        writeln!(tx, "FRONT_GEAR_LED:{}", self.gear_center_state.as_int())?;
        writeln!(tx, "LEFT_GEAR_LED:{}", self.gear_left_state.as_int())?;
        writeln!(tx, "RIGHT_GEAR_LED:{}", self.gear_right_state.as_int())?;
        Ok(())
    }
}

#[derive(Debug)]
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
    Keepalive,
    SynAck,
    /// Reset the connection to the Arduino
    Reset,
    Command(String),
    Sim(SimStatus),
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

fn run_simconnect(event_tx: mpsc::Sender<Event>) {
    let mut client = SimConnect::new("FSSK EventSim Main Panel").unwrap();

    loop {
        let notification = client.get_next_dispatch().unwrap();

        match notification {
            Some(Notification::Open) => {
                println!("Connection opened.");
                // After the connection is successfully open, we register the struct
                client.register_object::<AirplaneData>().unwrap();
            }
            Some(Notification::Object(data)) => {
                event_tx
                    .send(Event::Sim(AirplaneData::try_from(&data).unwrap().into()))
                    .unwrap();
            }
            _ => (),
        }

        // sleep for about a frame to reduce CPU usage
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}

fn run_keepalive(tx: mpsc::Sender<Event>) {
    loop {
        std::thread::sleep(Duration::from_millis(500));
        tx.send(Event::Keepalive).unwrap();
    }
}

fn run_serial_reader(event_tx: mpsc::Sender<Event>, serial_rx: Box<dyn SerialPort>) {
    let reader = BufReader::with_capacity(1, serial_rx);
    for line in reader.lines() {
        match line {
            Ok(command) => match command.as_str() {
                "SYN|ACK" => event_tx.send(Event::SynAck).unwrap(),
                "RST" => event_tx.send(Event::Reset).unwrap(),
                _ => event_tx.send(Event::Command(command)).unwrap(),
            },
            Err(e) => {
                if e.kind() != std::io::ErrorKind::TimedOut {
                    println!("{:?}", e)
                }
            }
        }
    }
    println!("Serial thread exited");
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args();
    if args.len() < 2 {
        // Print available serial ports
        let ports = serialport::available_ports().expect("No ports found!");
        for p in ports {
            println!("{}", p.port_name);
        }
        return Ok(());
    }
    let port_path = args.nth(1).unwrap();

    let mut connected = AtomicBool::new(false);

    let (event_tx, event_rx) = std::sync::mpsc::channel();

    println!("Connecting to port {}", port_path);

    // Open serial port
    let mut port_tx = serialport::new(&port_path, BAUD_RATE)
        .timeout(Duration::from_millis(10))
        .open()
        .expect("Failed to open port");

    // Serial reader thread
    let serial_event_tx = event_tx.clone();
    let port_rx = port_tx.try_clone()?;
    std::thread::spawn(move || run_serial_reader(serial_event_tx, port_rx));

    // SimConnect thread
    let simconnect_event_tx = event_tx.clone();
    std::thread::spawn(move || run_simconnect(simconnect_event_tx));
    // Periodic keepalive event thread
    let keepalive_event_tx = event_tx;
    std::thread::spawn(move || run_keepalive(keepalive_event_tx));

    // Initiate handshake with the Arduino
    writeln!(port_tx, "SYN")?;

    let client = SimConnect::new("FSSK EventSim Main Panel 2").unwrap();
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

    for event in event_rx {
        println!("Received event {:?}", &event);
        match event {
            Event::Keepalive if *connected.get_mut() => writeln!(port_tx, "PING")?,
            Event::SynAck => {
                writeln!(port_tx, "ACK")?;
                *connected.get_mut() = true;
            }
            Event::Reset => *connected.get_mut() = false,
            Event::Sim(simstate) => simstate.send(&mut port_tx)?,
            Event::Command(cmd) => match cmd.as_str() {
                "MISC1:0" => client.transmit_event(SimClientEvent::TaxiLightsOff)?,
                "MISC1:1" => client.transmit_event(SimClientEvent::TaxiLightsOn)?,
                "MISC2:0" => client.transmit_event(SimClientEvent::LandingLightsOff)?,
                "MISC2:1" => client.transmit_event(SimClientEvent::LandingLightsOn)?,
                "MISC3:0" => client.transmit_event(SimClientEvent::NavLightsOff)?,
                "MISC3:1" => client.transmit_event(SimClientEvent::NavLightsOn)?,
                "MISC4:0" => client.transmit_event(SimClientEvent::StrobeLightsOff)?,
                "MISC4:1" => client.transmit_event(SimClientEvent::StrobeLightsOn)?,
                "FLAPS_UP" => client.transmit_event(SimClientEvent::FlapsUp)?,
                "FLAPS_DN" => client.transmit_event(SimClientEvent::FlapsDown)?,
                "PARKING_BRAKE:0" => client.transmit_event(SimClientEvent::ParkingBrakeOff)?,
                "PARKING_BRAKE:1" => client.transmit_event(SimClientEvent::ParkingBrakeOn)?,
                "LANDING_GEAR:0" => client.transmit_event(SimClientEvent::LandingGearUp)?,
                "LANDING_GEAR:1" => client.transmit_event(SimClientEvent::LandingGearDown)?,
                _ => {}
            },
            _ => {}
        }
    }

    Ok(())
}
