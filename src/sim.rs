use std::{
    fmt::Display,
    sync::{mpsc, Arc, RwLock},
    time::Duration,
};

use log::{debug, error, info, warn};
use simconnect_sdk::{
    FlxClientEvent, Notification, SimConnect, SimConnectError, SimConnectObject, SystemEvent,
};

use crate::Event;

const SIMCONNECT_NAME: &str = "FSSK Panels";

#[derive(Debug)]
pub struct SimState {
    pub sim_connected: bool,
    pub sim_running: bool,
}

impl SimState {
    pub fn new() -> Self {
        Self {
            sim_connected: false,
            sim_running: false,
        }
    }

    fn disconnected(&mut self) {
        self.sim_connected = false;
        self.sim_running = false;
    }
}

#[derive(Debug)]
enum SimError {
    /// Failed to connect to the simulator
    Connect(SimConnectError),
    /// Error during communication with the simulator during runtime
    Runtime(SimConnectError),
}

impl std::error::Error for SimError {}

impl Display for SimError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SimError::Connect(e) => write!(f, "Failed to connect via SimConnect: {}", e),
            SimError::Runtime(e) => write!(f, "SimConnect communication error: {}", e),
        }
    }
}

impl From<SimConnectError> for SimError {
    fn from(value: SimConnectError) -> Self {
        SimError::Runtime(value)
    }
}

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

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FuelSystemPumpStatus {
    Off = 0,
    On = 1,
    Auto = 2,
}

impl From<bool> for FuelSystemPumpStatus {
    fn from(value: bool) -> Self {
        match value {
            true => Self::On,
            false => Self::Off,
        }
    }
}

/// Client events which we are sending to the simulator
///
/// This enum starts at a higher number to not cause conflicts with system events defined in the simconnect dependency.
#[derive(Debug, Clone)]
#[repr(u32)]
pub enum SimClientEvent {
    AlternatorSet {
        state: bool,
        alternator_index: u32,
    } = 1337,
    Battery1Set(bool),
    Battery2Set(bool),
    AvionicsMaster1Set(bool),
    AvionicsMaster2Set(bool),
    BeaconLightOn,
    BeaconLightOff,
    NavLightsOn,
    NavLightsOff,
    StrobeLightsOn,
    StrobeLightsOff,
    TaxiLightsOn,
    TaxiLightsOff,
    LandingLightsOn,
    LandingLightsOff,
    ElecFuelPump1Set(FuelSystemPumpStatus),
    PitotHeatOn,
    PitotHeatOff,
    CabinPwrOn,
    CabinPwrOff,
    PanelLightsPowerSettingSet {
        light_circuit_index: u32,
        power_setting: f32,
    },
    PedestalLightsPowerSettingSet {
        light_circuit_index: u32,
        power_setting: f32,
    },
    LightPotentiometerSet {
        index: u32,
        potentiometer_value: f32,
    },
    FlapsUp,
    FlapsDown,
    ParkingBrakeOn,
    ParkingBrakeOff,
    LandingGearUp,
    LandingGearDown,
}
impl FlxClientEvent for SimClientEvent {
    fn event_id(&self) -> u32 {
        // SAFETY: Because `Self` is marked `repr(u8)`, its layout is a `repr(C)` `union`
        // between `repr(C)` structs, each of which has the `u8` discriminant as its first
        // field, so we can read the discriminant without offsetting the pointer.
        unsafe { *<*const _>::from(self).cast::<u32>() }
    }

    fn event_name(&self) -> *const std::ffi::c_char {
        (match self {
            SimClientEvent::AlternatorSet { .. } => "ALTERNATOR_SET\0",
            SimClientEvent::Battery1Set { .. } => "BATTERY1_SET\0",
            SimClientEvent::Battery2Set { .. } => "BATTERY2_SET\0",
            SimClientEvent::AvionicsMaster1Set { .. } => "AVIONICS_MASTER_1_SET\0",
            SimClientEvent::AvionicsMaster2Set { .. } => "AVIONICS_MASTER_2_Set\0",
            SimClientEvent::BeaconLightOn => "BEACON_LIGHTS_ON\0",
            SimClientEvent::BeaconLightOff => "BEACON_LIGHTS_OFF\0",
            SimClientEvent::NavLightsOn => "NAV_LIGHTS_ON\0",
            SimClientEvent::NavLightsOff => "NAV_LIGHTS_OFF\0",
            SimClientEvent::StrobeLightsOn => "STROBES_ON\0",
            SimClientEvent::StrobeLightsOff => "STROBES_OFF\0",
            SimClientEvent::TaxiLightsOn => "TAXI_LIGHTS_ON\0",
            SimClientEvent::TaxiLightsOff => "TAXI_LIGHTS_OFF\0",
            SimClientEvent::LandingLightsOn => "LANDING_LIGHTS_ON\0",
            SimClientEvent::LandingLightsOff => "LANDING_LIGHTS_OFF\0",
            SimClientEvent::ElecFuelPump1Set { .. } => "ELECT_FUEL_PUMP1_SET\0",
            SimClientEvent::PitotHeatOn => "PITOT_HEAT_ON\0",
            SimClientEvent::PitotHeatOff => "PITOT_HEAT_OFF\0",
            SimClientEvent::CabinPwrOn => todo!(),
            SimClientEvent::CabinPwrOff => todo!(),
            SimClientEvent::PanelLightsPowerSettingSet { .. } => "PANEL_LIGHTS_POWER_SETTING_SET\0",
            SimClientEvent::PedestalLightsPowerSettingSet { .. } => {
                "PEDESTRAL_LIGHTS_POWER_SETTING_SET\0"
            }
            SimClientEvent::LightPotentiometerSet { .. } => "LIGHT_POTENTIOMETER_SET\0",
            SimClientEvent::FlapsUp => "FLAPS_DECR\0",
            SimClientEvent::FlapsDown => "FLAPS_INCR\0",
            SimClientEvent::ParkingBrakeOn => "PARKING_BRAKE_SET\0",
            SimClientEvent::ParkingBrakeOff => "PARKING_BRAKE_SET\0",
            SimClientEvent::LandingGearUp => "GEAR_UP\0",
            SimClientEvent::LandingGearDown => "GEAR_DOWN\0",
        })
        .as_ptr() as *const std::ffi::c_char
    }

    fn data(&self) -> (u32, u32, u32, u32, u32) {
        match self {
            SimClientEvent::AlternatorSet {
                state,
                alternator_index,
            } => (*state as u32, *alternator_index, 0, 0, 0),
            SimClientEvent::Battery1Set(state) => (*state as u32, 0, 0, 0, 0),
            SimClientEvent::Battery2Set(state) => (*state as u32, 0, 0, 0, 0),
            SimClientEvent::AvionicsMaster1Set(state) => (*state as u32, 0, 0, 0, 0),
            SimClientEvent::AvionicsMaster2Set(state) => (*state as u32, 0, 0, 0, 0),
            SimClientEvent::ElecFuelPump1Set(state) => (state.clone() as u32, 0, 0, 0, 0),
            SimClientEvent::PanelLightsPowerSettingSet {
                light_circuit_index,
                power_setting,
            } => (
                *light_circuit_index,
                (*power_setting * 100.0) as u32,
                0,
                0,
                0,
            ),
            SimClientEvent::PedestalLightsPowerSettingSet {
                light_circuit_index,
                power_setting,
            } => (
                *light_circuit_index,
                (*power_setting * 100.0) as u32,
                0,
                0,
                0,
            ),
            SimClientEvent::LightPotentiometerSet {
                index,
                potentiometer_value,
            } => (*index, (*potentiometer_value * 100.0) as u32, 0, 0, 0),
            SimClientEvent::ParkingBrakeOn => (1, 0, 0, 0, 0),
            SimClientEvent::ParkingBrakeOff => (0, 0, 0, 0, 0),
            _ => (0, 0, 0, 0, 0),
        }
    }
}

pub struct SimCommunicator {
    sim_state: Arc<RwLock<SimState>>,
    sim_txs: Vec<mpsc::Sender<Event>>,
    hw_rx: mpsc::Receiver<Event>,
}

impl SimCommunicator {
    pub fn new(
        sim_state: Arc<RwLock<SimState>>,
        sim_txs: Vec<mpsc::Sender<Event>>,
        hw_rx: mpsc::Receiver<Event>,
    ) -> Self {
        Self {
            sim_state,
            sim_txs,
            hw_rx,
        }
    }

    pub fn run(&mut self) {
        loop {
            debug!("Attempting to connect via SimConnect");
            match self.connect_and_process() {
                Err(SimError::Connect(e)) => warn!("{}", e),
                Err(SimError::Runtime(e)) => error!("{}", e),
                Ok(_) => {}
            };
            // Wait before reconnecting
            std::thread::sleep(Duration::from_secs(5));
        }
    }

    fn connect_and_process(&mut self) -> Result<(), SimError> {
        // Connect via SimConnect
        let client = SimConnect::new(SIMCONNECT_NAME).map_err(SimError::Connect)?;
        // Run forever until we cause an error
        if let Err(e) = self.run_event_loop(client) {
            self.sim_state.write().unwrap().disconnected();
            return Err(e.into());
        }
        // Simulator closed the connection normally
        Ok(())
    }

    fn run_event_loop(&mut self, mut client: SimConnect) -> Result<(), SimConnectError> {
        loop {
            // Loop to process all pending events in the channel at once
            for msg in self.hw_rx.try_iter() {
                self.process_hw_event(&client, msg)?;
            }

            // Loop to process all pending sim events
            while let Some(notification) = client.get_next_dispatch()? {
                debug!("Received notification {:?}", notification);
                match notification {
                    Notification::Open => {
                        info!("Connection with flight simulator established");
                        Self::register_events(&mut client)?;
                        self.sim_state.write().unwrap().sim_connected = true;
                    }
                    Notification::Quit => {
                        info!("Disconnected from flight simulator");
                        self.sim_state.write().unwrap().disconnected();
                        return Ok(());
                    }
                    Notification::SystemEvent(SystemEvent::Sim { state }) => {
                        info!("Current simulation state: {}", state);
                        self.sim_state.write().unwrap().sim_running = state;
                    }
                    Notification::Object(data) => {
                        let aircraft_state = AircraftSimData::try_from(&data)?;
                        debug!("Received SimConnect aircraft state {:?}", aircraft_state);
                        for sim_tx in &self.sim_txs {
                            sim_tx
                                .send(Event::SetPanel(aircraft_state.clone().into()))
                                .expect("Failed to send to panel");
                        }
                    }
                    _ => {}
                }
            }

            // Sleep for about a frame to reduce CPU usage
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    }

    /// Handle the events received from hardware
    ///
    /// Note: hardware is responsible to only issue events when the simulator is running.
    fn process_hw_event(&self, client: &SimConnect, msg: Event) -> Result<(), SimConnectError> {
        if let Event::SetSimulator(event) = msg {
            debug!("Sending sim event {:?}", event);
            client.transmit_event(event)?;
        }
        Ok(())
    }

    fn register_events(client: &mut SimConnect) -> Result<(), SimConnectError> {
        // Receive a status whether the sim is running
        // On first connect, this status is always sent by the flight simulator
        client.subscribe_to_system_event(simconnect_sdk::SystemEventRequest::Sim)?;
        // We register the aircraft data struct
        client.register_object::<AircraftSimData>()?;
        // We register the events we want to send to the simulator
        client.map_client_event_to_sim_event(SimClientEvent::AlternatorSet {
            state: false,
            alternator_index: 0,
        })?;
        client.map_client_event_to_sim_event(SimClientEvent::Battery1Set(false))?;
        client.map_client_event_to_sim_event(SimClientEvent::Battery2Set(false))?;
        client.map_client_event_to_sim_event(SimClientEvent::AvionicsMaster1Set(false))?;
        client.map_client_event_to_sim_event(SimClientEvent::AvionicsMaster2Set(false))?;
        client.map_client_event_to_sim_event(SimClientEvent::BeaconLightOn)?;
        client.map_client_event_to_sim_event(SimClientEvent::BeaconLightOff)?;
        client.map_client_event_to_sim_event(SimClientEvent::NavLightsOn)?;
        client.map_client_event_to_sim_event(SimClientEvent::NavLightsOff)?;
        client.map_client_event_to_sim_event(SimClientEvent::StrobeLightsOn)?;
        client.map_client_event_to_sim_event(SimClientEvent::StrobeLightsOff)?;
        client.map_client_event_to_sim_event(SimClientEvent::TaxiLightsOn)?;
        client.map_client_event_to_sim_event(SimClientEvent::TaxiLightsOff)?;
        client.map_client_event_to_sim_event(SimClientEvent::LandingLightsOn)?;
        client.map_client_event_to_sim_event(SimClientEvent::LandingLightsOff)?;
        client.map_client_event_to_sim_event(SimClientEvent::ElecFuelPump1Set(
            FuelSystemPumpStatus::Off,
        ))?;
        client.map_client_event_to_sim_event(SimClientEvent::PitotHeatOn)?;
        client.map_client_event_to_sim_event(SimClientEvent::PitotHeatOff)?;
        client.map_client_event_to_sim_event(SimClientEvent::PanelLightsPowerSettingSet {
            light_circuit_index: 0,
            power_setting: 0.0,
        })?;
        client.map_client_event_to_sim_event(SimClientEvent::PedestalLightsPowerSettingSet {
            light_circuit_index: 0,
            power_setting: 0.0,
        })?;
        client.map_client_event_to_sim_event(SimClientEvent::LightPotentiometerSet {
            index: 0,
            potentiometer_value: 0.0,
        })?;
        client.map_client_event_to_sim_event(SimClientEvent::FlapsUp)?;
        client.map_client_event_to_sim_event(SimClientEvent::FlapsDown)?;
        client.map_client_event_to_sim_event(SimClientEvent::ParkingBrakeOn)?;
        client.map_client_event_to_sim_event(SimClientEvent::ParkingBrakeOff)?;
        client.map_client_event_to_sim_event(SimClientEvent::LandingGearUp)?;
        client.map_client_event_to_sim_event(SimClientEvent::LandingGearDown)?;

        Ok(())
    }
}
