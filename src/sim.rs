use std::{
    collections::HashMap,
    ffi::CStr,
    fmt::Display,
    sync::{mpsc, Arc, RwLock},
    time::{Duration, Instant},
};

use log::{debug, error, info, warn};
use simconnect_sdk::{
    FlxClientEvent, InputEvent, Notification, SimConnect, SimConnectError, SimConnectObject,
    SystemEvent, VariableId,
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
    // Electrical outputs of the Cessna T182T realism mod
    #[simconnect(name = "L:ELEC_STBY_TEST_LAMP", unit = "number")]
    stby_batt_test_lamp: f64,
    #[simconnect(name = "L:ELEC_LIGHT_SW_CB_PANELS", unit = "number")]
    sw_cb_panels_light: f64,

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
    pub stby_batt_test_lamp: bool,
    /// Brightness of the switch and circuit breaker panel lighting from 0 to 1
    pub sw_cb_panels_light: f64,
}

impl From<AircraftSimData> for AircraftSimState {
    fn from(value: AircraftSimData) -> Self {
        Self {
            parking_brake_indicator: value.parking_brake_indicator,
            gear_center_state: value.gear_center_position.into(),
            gear_left_state: value.gear_left_position.into(),
            gear_right_state: value.gear_right_position.into(),
            airspeed: value.airspeed,
            stby_batt_test_lamp: value.stby_batt_test_lamp != 0.0,
            sw_cb_panels_light: value.sw_cb_panels_light,
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
#[derive(Debug, Clone, PartialEq)]
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
    StbyBatteryTestSet(bool),
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
    PanelLightsSet {
        state: bool,
        light_index: u32,
    },
    PanelLightsPowerSettingSet {
        light_circuit_index: u32,
        power_setting: f32,
    },
    PedestalLightsSet {
        state: bool,
        light_index: u32,
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

    fn event_name(&self) -> &'static CStr {
        match self {
            SimClientEvent::AlternatorSet { .. } => c"ALTERNATOR_SET",
            SimClientEvent::Battery1Set { .. } => c"BATTERY1_SET",
            SimClientEvent::Battery2Set { .. } => c"BATTERY2_SET",
            SimClientEvent::BeaconLightOn => c"BEACON_LIGHTS_ON",
            SimClientEvent::BeaconLightOff => c"BEACON_LIGHTS_OFF",
            SimClientEvent::NavLightsOn => c"NAV_LIGHTS_ON",
            SimClientEvent::NavLightsOff => c"NAV_LIGHTS_OFF",
            SimClientEvent::StrobeLightsOn => c"STROBES_ON",
            SimClientEvent::StrobeLightsOff => c"STROBES_OFF",
            SimClientEvent::TaxiLightsOn => c"TAXI_LIGHTS_ON",
            SimClientEvent::TaxiLightsOff => c"TAXI_LIGHTS_OFF",
            SimClientEvent::LandingLightsOn => c"LANDING_LIGHTS_ON",
            SimClientEvent::LandingLightsOff => c"LANDING_LIGHTS_OFF",
            SimClientEvent::ElecFuelPump1Set { .. } => c"ELECT_FUEL_PUMP1_SET",
            SimClientEvent::PitotHeatOn => c"PITOT_HEAT_ON",
            SimClientEvent::PitotHeatOff => c"PITOT_HEAT_OFF",
            SimClientEvent::PanelLightsSet { .. } => c"PANEL_LIGHTS_SET",
            SimClientEvent::PanelLightsPowerSettingSet { .. } => c"PANEL_LIGHTS_POWER_SETTING_SET",
            SimClientEvent::PedestalLightsSet { .. } => c"PEDESTRAL_LIGHTS_SET",
            SimClientEvent::PedestalLightsPowerSettingSet { .. } => {
                c"PEDESTRAL_LIGHTS_POWER_SETTING_SET"
            }
            SimClientEvent::LightPotentiometerSet { .. } => c"LIGHT_POTENTIOMETER_SET",
            SimClientEvent::FlapsUp => c"FLAPS_DECR",
            SimClientEvent::FlapsDown => c"FLAPS_INCR",
            SimClientEvent::ParkingBrakeOn => c"PARKING_BRAKE_SET",
            SimClientEvent::ParkingBrakeOff => c"PARKING_BRAKE_SET",
            SimClientEvent::LandingGearUp => c"GEAR_UP",
            SimClientEvent::LandingGearDown => c"GEAR_DOWN",
            // Set as cockpit switches, see `cockpit_switch`
            SimClientEvent::AvionicsMaster1Set(_)
            | SimClientEvent::AvionicsMaster2Set(_)
            | SimClientEvent::StbyBatteryTestSet(_)
            | SimClientEvent::CabinPwrOn
            | SimClientEvent::CabinPwrOff => unreachable!(),
        }
    }

    fn data(&self) -> (u32, u32, u32, u32, u32) {
        match self {
            SimClientEvent::AlternatorSet {
                state,
                alternator_index,
            } => (*state as u32, *alternator_index, 0, 0, 0),
            SimClientEvent::Battery1Set(state) => (*state as u32, 0, 0, 0, 0),
            SimClientEvent::Battery2Set(state) => (*state as u32, 0, 0, 0, 0),
            SimClientEvent::ElecFuelPump1Set(state) => (state.clone() as u32, 0, 0, 0, 0),
            SimClientEvent::PanelLightsSet { state, light_index } => {
                (*state as u32, *light_index, 0, 0, 0)
            }
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
            SimClientEvent::PedestalLightsSet { state, light_index } => {
                (*state as u32, *light_index, 0, 0, 0)
            }
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

impl SimClientEvent {
    /// Get the cockpit switch and its state if the event is not sent as sim event
    fn cockpit_switch(&self) -> Option<(CockpitSwitch, bool)> {
        match self {
            SimClientEvent::AvionicsMaster1Set(state) => {
                Some((CockpitSwitch::AvionicsBus1, *state))
            }
            SimClientEvent::AvionicsMaster2Set(state) => {
                Some((CockpitSwitch::AvionicsBus2, *state))
            }
            SimClientEvent::StbyBatteryTestSet(state) => {
                Some((CockpitSwitch::StbyBatteryTest, *state))
            }
            SimClientEvent::CabinPwrOn => Some((CockpitSwitch::CabinPower12V, true)),
            SimClientEvent::CabinPwrOff => Some((CockpitSwitch::CabinPower12V, false)),
            _ => None,
        }
    }
}

/// Cockpit switches of the T182T mod which cannot be set with a sim event
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum CockpitSwitch {
    AvionicsBus1,
    AvionicsBus2,
    StbyBatteryTest,
    CabinPower12V,
}

/// How a cockpit switch is set in the simulator
enum SwitchTarget {
    /// The switch follows an `L:` variable
    Variable(&'static str),
    /// The switch is set with an input event of the cockpit
    InputEvent(&'static str),
}

impl CockpitSwitch {
    const ALL: [CockpitSwitch; 4] = [
        CockpitSwitch::AvionicsBus1,
        CockpitSwitch::AvionicsBus2,
        CockpitSwitch::StbyBatteryTest,
        CockpitSwitch::CabinPower12V,
    ];

    fn target(&self) -> SwitchTarget {
        match self {
            // The avionics switches use the avionics bus template of Asobo
            CockpitSwitch::AvionicsBus1 => SwitchTarget::InputEvent("ELECTRICAL_Avionics_Bus_1"),
            CockpitSwitch::AvionicsBus2 => SwitchTarget::InputEvent("ELECTRICAL_Avionics_Bus_2"),
            CockpitSwitch::StbyBatteryTest => SwitchTarget::Variable("L:XMLVAR_STBYBattery_Test"),
            CockpitSwitch::CabinPower12V => {
                SwitchTarget::Variable("L:XMLVAR_ELECTRICAL_Switch_Power12V_ON")
            }
        }
    }
}

/// Minimum time before an input event is set to the same value again
///
/// The template of the avionics switches toggles the switch if it differs from the value we set. Setting the value
/// again before the toggle took effect would toggle the switch back.
const INPUT_EVENT_REFRESH: Duration = Duration::from_secs(1);

/// Holds what is needed to set the cockpit switches during a SimConnect connection
#[derive(Default)]
struct CockpitSwitches {
    /// The registered `L:` variables of the switches
    variables: HashMap<CockpitSwitch, VariableId>,
    /// The input event hashes of the switches in the current flight
    input_events: HashMap<CockpitSwitch, u64>,
    /// The last value each input event was set to and when
    input_events_last_set: HashMap<CockpitSwitch, (bool, Instant)>,
}

impl CockpitSwitches {
    /// Register the `L:` variables of the switches
    fn register_variables(&mut self, client: &mut SimConnect) -> Result<(), SimConnectError> {
        for switch in CockpitSwitch::ALL {
            if let SwitchTarget::Variable(name) = switch.target() {
                let variable = client.register_variable(name, "number")?;
                self.variables.insert(switch, variable);
            }
        }
        Ok(())
    }

    /// Store the hashes of the input events of the switches
    ///
    /// The simulator lists the input events in multiple parts, `last` is set for the last part.
    fn add_input_events(&mut self, events: &[InputEvent], last: bool) {
        for switch in CockpitSwitch::ALL {
            if let SwitchTarget::InputEvent(name) = switch.target() {
                if let Some(event) = events.iter().find(|e| e.name.eq_ignore_ascii_case(name)) {
                    info!("Found input event {} for {:?}", event.name, switch);
                    self.input_events.insert(switch, event.hash);
                } else if last && !self.input_events.contains_key(&switch) {
                    warn!(
                        "Aircraft has no input event {}, cannot set {:?}",
                        name, switch
                    );
                }
            }
        }
    }

    /// Forget the input events, they are only valid for the current flight
    fn clear_input_events(&mut self) {
        self.input_events.clear();
        self.input_events_last_set.clear();
    }

    /// Check whether an input event has to be set and remember the value if so
    ///
    /// This is the case if the value changed or if the last set is at least `INPUT_EVENT_REFRESH` ago.
    fn input_event_due(&mut self, switch: CockpitSwitch, state: bool, now: Instant) -> bool {
        let due = match self.input_events_last_set.get(&switch) {
            Some(&(last_state, last_time)) => {
                last_state != state || now.duration_since(last_time) >= INPUT_EVENT_REFRESH
            }
            None => true,
        };
        if due {
            self.input_events_last_set.insert(switch, (state, now));
        }
        due
    }

    /// Set a cockpit switch in the simulator
    ///
    /// Input events that the simulator has not listed yet are skipped.
    fn set(
        &mut self,
        client: &SimConnect,
        switch: CockpitSwitch,
        state: bool,
    ) -> Result<(), SimConnectError> {
        let value = if state { 1.0 } else { 0.0 };
        match switch.target() {
            SwitchTarget::Variable(_) => {
                if let Some(&variable) = self.variables.get(&switch) {
                    client.set_variable(variable, value)?;
                }
            }
            SwitchTarget::InputEvent(_) => {
                if let Some(&hash) = self.input_events.get(&switch) {
                    if self.input_event_due(switch, state, Instant::now()) {
                        client.set_input_event(hash, value)?;
                    }
                }
            }
        }
        Ok(())
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
        let mut cockpit_switches = CockpitSwitches::default();
        loop {
            // Loop to process all pending events in the channel at once
            for msg in self.hw_rx.try_iter() {
                self.process_hw_event(&client, &mut cockpit_switches, msg)?;
            }

            // Loop to process all pending sim events
            while let Some(notification) = client.get_next_dispatch()? {
                debug!("Received notification {:?}", notification);
                match notification {
                    Notification::Open => {
                        info!("Connection with flight simulator established");
                        Self::register_events(&mut client)?;
                        cockpit_switches.register_variables(&mut client)?;
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
                        // The input events depend on the aircraft, so we request them for every flight
                        cockpit_switches.clear_input_events();
                        if state {
                            if let Err(e) = client.enumerate_input_events() {
                                warn!("Failed to request input events: {}", e);
                            }
                        }
                    }
                    Notification::InputEventList { events, last } => {
                        cockpit_switches.add_input_events(&events, last);
                    }
                    Notification::Object(data) => {
                        let aircraft_state = AircraftSimData::try_from(&data)?;
                        debug!("Received SimConnect aircraft state {:?}", aircraft_state);
                        // We drop the panels whose thread has exited
                        self.sim_txs.retain(|sim_tx| {
                            sim_tx
                                .send(Event::SetPanel(aircraft_state.clone().into()))
                                .is_ok()
                        });
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
    fn process_hw_event(
        &self,
        client: &SimConnect,
        cockpit_switches: &mut CockpitSwitches,
        msg: Event,
    ) -> Result<(), SimConnectError> {
        if let Event::SetSimulator(event) = msg {
            debug!("Sending sim event {:?}", event);
            match event.cockpit_switch() {
                Some((switch, state)) => cockpit_switches.set(client, switch, state)?,
                None => client.transmit_event(event)?,
            }
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
        client.map_client_event_to_sim_event(SimClientEvent::PanelLightsSet {
            state: false,
            light_index: 0,
        })?;
        client.map_client_event_to_sim_event(SimClientEvent::PanelLightsPowerSettingSet {
            light_circuit_index: 0,
            power_setting: 0.0,
        })?;
        client.map_client_event_to_sim_event(SimClientEvent::PedestalLightsSet {
            state: false,
            light_index: 0,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn input_events_are_found_in_all_parts() {
        let mut cockpit_switches = CockpitSwitches::default();
        let event = |name: &str, hash| InputEvent {
            name: name.to_owned(),
            hash,
        };
        cockpit_switches.add_input_events(&[event("ELECTRICAL_Avionics_Bus_2", 2)], false);
        cockpit_switches.add_input_events(
            &[
                event("ELECTRICAL_Master_Battery_1", 7),
                event("electrical_avionics_bus_1", 1),
            ],
            true,
        );
        let input_events = &cockpit_switches.input_events;
        assert_eq!(input_events.get(&CockpitSwitch::AvionicsBus1), Some(&1));
        assert_eq!(input_events.get(&CockpitSwitch::AvionicsBus2), Some(&2));
        assert_eq!(input_events.len(), 2);
    }

    #[test]
    fn input_events_are_set_on_change_and_refreshed() {
        let mut cockpit_switches = CockpitSwitches::default();
        let switch = CockpitSwitch::AvionicsBus1;
        let start = Instant::now();
        let at = |ms| start + Duration::from_millis(ms);
        assert!(cockpit_switches.input_event_due(switch, true, at(0)));
        assert!(!cockpit_switches.input_event_due(switch, true, at(25)));
        assert!(cockpit_switches.input_event_due(switch, false, at(50)));
        assert!(!cockpit_switches.input_event_due(switch, false, at(1049)));
        assert!(cockpit_switches.input_event_due(switch, false, at(1050)));
        cockpit_switches.clear_input_events();
        assert!(cockpit_switches.input_event_due(switch, false, at(1075)));
    }

    #[test]
    fn input_events_are_cleared() {
        let mut cockpit_switches = CockpitSwitches::default();
        let event = InputEvent {
            name: "ELECTRICAL_Avionics_Bus_1".to_owned(),
            hash: 1,
        };
        cockpit_switches.add_input_events(&[event], true);
        cockpit_switches.clear_input_events();
        assert!(cockpit_switches.input_events.is_empty());
    }

    #[test]
    fn switch_events_are_set_in_the_cockpit() {
        assert_eq!(
            SimClientEvent::AvionicsMaster2Set(true).cockpit_switch(),
            Some((CockpitSwitch::AvionicsBus2, true))
        );
        assert_eq!(
            SimClientEvent::StbyBatteryTestSet(true).cockpit_switch(),
            Some((CockpitSwitch::StbyBatteryTest, true))
        );
        assert_eq!(
            SimClientEvent::CabinPwrOff.cockpit_switch(),
            Some((CockpitSwitch::CabinPower12V, false))
        );
        assert_eq!(SimClientEvent::Battery2Set(true).cockpit_switch(), None);
    }
}
