use std::{sync::mpsc, time::Duration};

use log::{debug, error, info, warn};
use simconnect_sdk::{FlxClientEvent, Notification, SimConnect, SimConnectError, SimConnectObject};

use crate::Event;

const SIMCONNECT_NAME: &str = "FSSK Panels";

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
    #[simconnect(name = "AIRSPEED INDICATED", unit = "knots")]
    airspeed: f64,

    /// Parking brake indicator.
    ///
    /// WARNING: Must be the last entry in the struct due to a bug in the `simconnect-sdk` crate, otherwise the gear
    /// position values are interpreted incorrectly.
    #[simconnect(name = "BRAKE PARKING INDICATOR")]
    parking_brake_indicator: bool,
}

#[derive(Debug, PartialEq)]
pub struct AircraftSimState {
    pub parking_brake_indicator: bool,
    pub gear_center_state: LandingGearStatus,
    pub gear_left_state: LandingGearStatus,
    pub gear_right_state: LandingGearStatus,
    pub airspeed: f64,
}

impl From<AircraftSimData> for AircraftSimState {
    fn from(value: AircraftSimData) -> Self {
        Self {
            parking_brake_indicator: value.parking_brake_indicator,
            gear_center_state: value.gear_center_position.into(),
            gear_left_state: value.gear_left_position.into(),
            gear_right_state: value.gear_right_position.into(),
            airspeed: value.airspeed,
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum LandingGearStatus {
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
    pub fn as_int(&self) -> i32 {
        match self {
            LandingGearStatus::Up => 0,
            LandingGearStatus::Down => 1,
            LandingGearStatus::Unknown => 2,
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(u32)]
pub enum SimClientEvent {
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

pub struct SimCommunicator {
    connected: bool,
    sim_txs: Vec<mpsc::Sender<Event>>,
    hw_rx: mpsc::Receiver<Event>,
}

impl SimCommunicator {
    pub fn new(sim_txs: Vec<mpsc::Sender<Event>>, hw_rx: mpsc::Receiver<Event>) -> Self {
        Self {
            connected: false,
            sim_txs,
            hw_rx,
        }
    }

    pub fn run(&mut self) {
        loop {
            debug!("Attempting to connect via SimConnect");
            match SimConnect::new(SIMCONNECT_NAME) {
                Ok(client) => match self.run_event_loop(client) {
                    // If we receive the exit signal, exit the thread
                    Ok(true) => return,
                    // Peaceful disconnect from simulator, reconnect later
                    Ok(false) => {}
                    // Got SimConnect error, notify user
                    Err(e) => error!("SimConnect communication error: {:?}", e),
                },
                Err(e) => {
                    warn!("Failed to connect via SimConnect: {:?}", e);
                }
            }

            // We are now disconnected
            self.connected = false;

            // Wait before reconnecting
            std::thread::sleep(Duration::from_secs(5));
        }
    }

    fn run_event_loop(&mut self, mut client: SimConnect) -> Result<bool, SimConnectError> {
        loop {
            // Receive control messages if we are connected
            if self.connected {
                match self.hw_rx.try_recv() {
                    Ok(Event::SetSimulator(event)) => client.transmit_event(event)?,
                    Err(mpsc::TryRecvError::Disconnected) => return Ok(true),
                    _ => {}
                }
            }

            match client.get_next_dispatch()? {
                Some(Notification::Open) => {
                    info!("Connection with flight simulator established");
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
                    info!("Disconnected from flight simulator");
                    return Ok(false);
                }
                Some(Notification::Object(data)) => {
                    let aircraft_state = AircraftSimData::try_from(&data)?;
                    debug!("Received SimConnect aircraft state {:?}", aircraft_state);
                    for sim_tx in &self.sim_txs {
                        sim_tx
                            .send(Event::SetPanel(aircraft_state.clone().into()))
                            .expect("Failed to send to panel");
                    }
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
