use log::debug;
use log::warn;
use std::io::BufRead;
use std::io::BufReader;
use std::sync::mpsc;
use std::sync::Arc;
use std::sync::RwLock;
use std::thread;
use std::time::Duration;

use crate::bitfield::BitField;
use crate::panel::Panel;
use crate::panel::PanelError;
use crate::sim::AircraftSimState;
use crate::sim::SimClientEvent;
use crate::Event;
use crate::SimState;

/// The baud rate of the Arduino used for the serial connection.
const BAUD_RATE: u32 = 115200;

/// The number of data bytes in a normal COBS frame
const DATA_LEN: usize = 6;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Switch {
    Alternator = 1 << 15,
    Battery = 1 << 14,
    Avionics1 = 1 << 13,
    Avionics2 = 1 << 12,
    StbyBatteryArm = 1 << 11,
    StbyBatteryTest = 1 << 10,
    Beacon = 1 << 7,
    Nav = 1 << 6,
    Strobe = 1 << 5,
    Taxi = 1 << 4,
    Landing = 1 << 3,
    FuelPump = 1 << 2,
    PitotHeat = 1 << 1,
    CabinPower12V = 1 << 0,
}

impl From<Switch> for u16 {
    fn from(value: Switch) -> Self {
        value as u16
    }
}

/// Represents the EventSim Main Panel and holds all state and information.
#[derive(Debug)]
pub struct C182TSwitchPanel {
    sim_state: Arc<RwLock<SimState>>,
    port: String,
    hw_tx: mpsc::Sender<Event>,
    sim_rx: mpsc::Receiver<Event>,
    switch_states: BitField<u16>,
    aircraft_sim_state: Option<AircraftSimState>,
}

impl Panel for C182TSwitchPanel {
    /// Connect to the panel and run an event loop.
    fn run(&mut self) -> Result<(), PanelError> {
        debug!(
            "Attempting to connect to C182T switch panel on serial port {}",
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
        let mut frame_reader = reader.split(0x00);

        loop {
            // Loop to process all pending events in the channel at once
            for event in self.sim_rx.try_iter() {
                if let Event::SetPanel(state) = event {
                    self.aircraft_sim_state = Some(state);
                }
            }

            // Read messages from serial port
            if let Some(frame) = frame_reader.next() {
                match frame {
                    Ok(mut frame) => match cobs::decode_in_place(&mut frame) {
                        Ok(data_len) => self.handle_serial_frame(&frame, data_len),
                        Err(e) => warn!("Error decoding COBS frame {:?}", e),
                    },
                    // Ignore timouts
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                    // Exit on all other errors
                    Err(e) => return Err(e.into()),
                }
            }

            // Send messages to serial port
            let buf = self.output_data();
            let mut encoded = cobs::encode_vec(&buf);
            encoded.push(0);
            serial.write_all(&encoded)?;
        }
    }
}

impl C182TSwitchPanel {
    /// Create a new panel instance.
    pub fn new(
        sim_state: Arc<RwLock<SimState>>,
        port: impl AsRef<str>,
        hw_tx: mpsc::Sender<Event>,
        sim_rx: mpsc::Receiver<Event>,
    ) -> Self {
        Self {
            sim_state,
            hw_tx,
            sim_rx,
            port: port.as_ref().into(),
            switch_states: BitField::new(0),
            aircraft_sim_state: None,
        }
    }

    fn handle_serial_frame(&mut self, data: &[u8], data_len: usize) {
        if data_len != DATA_LEN {
            warn!(
                "C182T switch panel command is of unexpected length {}",
                data_len
            );
            return;
        }

        // Update all switch positions
        let state = u16::from_be_bytes([data[0], data[1]]);
        self.switch_states.update(state);

        // Set switch positions in simulator
        self.send_sim_event(SimClientEvent::AlternatorSet {
            state: self.switch_states.is_set(Switch::Alternator),
            alternator_index: 1,
        });
        self.send_sim_event(SimClientEvent::Battery1Set(
            self.switch_states.is_set(Switch::Battery),
        ));
        self.send_sim_event(SimClientEvent::AvionicsMaster1Set(
            self.switch_states.is_set(Switch::Avionics1),
        ));
        self.send_sim_event(SimClientEvent::AvionicsMaster2Set(
            self.switch_states.is_set(Switch::Avionics2),
        ));
        self.send_sim_event(SimClientEvent::Battery2Set(
            !self.switch_states.is_set(Switch::StbyBatteryArm),
        ));
        self.send_sim_event(SimClientEvent::StbyBatteryTestSet(
            !self.switch_states.is_set(Switch::StbyBatteryTest),
        ));
        match !self.switch_states.is_set(Switch::Beacon) {
            true => self.send_sim_event(SimClientEvent::BeaconLightOn),
            false => self.send_sim_event(SimClientEvent::BeaconLightOff),
        };
        match !self.switch_states.is_set(Switch::Nav) {
            true => self.send_sim_event(SimClientEvent::NavLightsOn),
            false => self.send_sim_event(SimClientEvent::NavLightsOff),
        };
        match !self.switch_states.is_set(Switch::Strobe) {
            true => self.send_sim_event(SimClientEvent::StrobeLightsOn),
            false => self.send_sim_event(SimClientEvent::StrobeLightsOff),
        };
        match (
            !self.switch_states.is_set(Switch::Taxi),
            !self.switch_states.is_set(Switch::Landing),
        ) {
            (true, true) => {
                self.send_sim_event(SimClientEvent::TaxiLightsOn);
                self.send_sim_event(SimClientEvent::LandingLightsOn);
            }
            (true, false) => {
                self.send_sim_event(SimClientEvent::TaxiLightsOff);
                self.send_sim_event(SimClientEvent::LandingLightsOff);
            }
            (false, true) => {
                self.send_sim_event(SimClientEvent::TaxiLightsOn);
                self.send_sim_event(SimClientEvent::LandingLightsOn);
            }
            (false, false) => {
                self.send_sim_event(SimClientEvent::TaxiLightsOn);
                self.send_sim_event(SimClientEvent::LandingLightsOff);
            }
        };
        self.send_sim_event(SimClientEvent::ElecFuelPump1Set(
            (!self.switch_states.is_set(Switch::FuelPump)).into(),
        ));
        match !self.switch_states.is_set(Switch::PitotHeat) {
            true => self.send_sim_event(SimClientEvent::PitotHeatOn),
            false => self.send_sim_event(SimClientEvent::PitotHeatOff),
        };
        match !self.switch_states.is_set(Switch::CabinPower12V) {
            true => self.send_sim_event(SimClientEvent::CabinPwrOn),
            false => self.send_sim_event(SimClientEvent::CabinPwrOff),
        };

        // Send dimmer positions in simulator
        let panel_dimmer = (u8::MAX - data[2]) as f32 / 255.0;
        self.send_sim_event(SimClientEvent::LightPotentiometerSet {
            index: 3,
            potentiometer_value: panel_dimmer,
        });
        // Like the knobs in the cockpit, keep the lights on and only set their power
        let stby_instrument_dimmer = (u8::MAX - data[3]) as f32 / 255.0;
        self.send_sim_event(SimClientEvent::PanelLightsSet {
            state: true,
            light_index: 1,
        });
        self.send_sim_event(SimClientEvent::PanelLightsPowerSettingSet {
            light_circuit_index: 1,
            power_setting: stby_instrument_dimmer,
        });
        let pedestal_dimmer = (u8::MAX - data[4]) as f32 / 255.0;
        self.send_sim_event(SimClientEvent::PedestalLightsSet {
            state: true,
            light_index: 1,
        });
        self.send_sim_event(SimClientEvent::PedestalLightsPowerSettingSet {
            light_circuit_index: 1,
            power_setting: pedestal_dimmer,
        });
        let avionics_dimmer = (u8::MAX - data[5]) as f32 / 255.0;
        self.send_sim_event(SimClientEvent::LightPotentiometerSet {
            index: 4,
            potentiometer_value: avionics_dimmer,
        });
    }

    fn send_sim_event(&self, event: SimClientEvent) {
        if self.sim_state.read().unwrap().sim_running {
            self.hw_tx
                .send(Event::SetSimulator(event))
                .expect("SimConnect thread offline");
        }
    }

    /// Get the STBY BATT TEST lamp state and the backlight PWM duty cycle for the panel
    ///
    /// Both are off while the sim is not running.
    fn output_data(&self) -> [u8; 2] {
        if !self.sim_state.read().unwrap().sim_running {
            return [0, 0];
        }
        match &self.aircraft_sim_state {
            Some(state) => [
                state.stby_batt_test_lamp as u8,
                (state.sw_cb_panels_light * 255.0).round() as u8,
            ],
            None => [0, 0],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::LandingGearStatus;

    /// Create a panel for a running sim and the receiver of its events
    fn running_panel() -> (C182TSwitchPanel, mpsc::Receiver<Event>) {
        let mut sim_state = SimState::new();
        sim_state.sim_running = true;
        let (hw_tx, hw_rx) = mpsc::channel();
        let (_sim_tx, sim_rx) = mpsc::channel();
        let panel = C182TSwitchPanel::new(Arc::new(RwLock::new(sim_state)), "COM0", hw_tx, sim_rx);
        (panel, hw_rx)
    }

    /// Get the sim events of a frame with the given switches closed and the dimmers at the given positions
    fn events_for(closed: &[Switch], dimmers: [u8; 4]) -> Vec<SimClientEvent> {
        let (mut panel, hw_rx) = running_panel();
        let state = closed
            .iter()
            .fold(u16::MAX, |state, &switch| state & !u16::from(switch));
        let [high, low] = state.to_be_bytes();
        let [dimmer_1, dimmer_2, dimmer_3, dimmer_4] = dimmers;
        let data = [high, low, dimmer_1, dimmer_2, dimmer_3, dimmer_4];
        panel.handle_serial_frame(&data, DATA_LEN);
        let mut events = Vec::new();
        for event in hw_rx.try_iter() {
            if let Event::SetSimulator(event) = event {
                events.push(event);
            }
        }
        events
    }

    #[test]
    fn no_events_are_sent_while_the_sim_is_not_running() {
        let (mut panel, hw_rx) = running_panel();
        panel.sim_state.write().unwrap().sim_running = false;
        panel.handle_serial_frame(&[0x00, 0x00, 0xff, 0xff, 0xff, 0xff], DATA_LEN);
        assert!(hw_rx.try_iter().next().is_none());
    }

    #[test]
    fn stby_battery_test_switch_is_sent() {
        let events = events_for(&[Switch::StbyBatteryTest], [u8::MAX; 4]);
        assert!(events.contains(&SimClientEvent::StbyBatteryTestSet(true)));
        let events = events_for(&[], [u8::MAX; 4]);
        assert!(events.contains(&SimClientEvent::StbyBatteryTestSet(false)));
    }

    #[test]
    fn cabin_power_switch_only_uses_its_own_contact() {
        let events = events_for(&[Switch::PitotHeat], [u8::MAX; 4]);
        assert!(events.contains(&SimClientEvent::CabinPwrOff));
        let events = events_for(&[Switch::CabinPower12V], [u8::MAX; 4]);
        assert!(events.contains(&SimClientEvent::CabinPwrOn));
        assert!(events.contains(&SimClientEvent::PitotHeatOff));
    }

    #[test]
    fn dimmers_keep_the_lights_on() {
        let events = events_for(&[], [u8::MAX, 0, u8::MAX, 0]);
        assert!(events.contains(&SimClientEvent::PanelLightsSet {
            state: true,
            light_index: 1,
        }));
        assert!(
            events.contains(&SimClientEvent::PanelLightsPowerSettingSet {
                light_circuit_index: 1,
                power_setting: 1.0,
            })
        );
        assert!(events.contains(&SimClientEvent::PedestalLightsSet {
            state: true,
            light_index: 1,
        }));
        assert!(
            events.contains(&SimClientEvent::PedestalLightsPowerSettingSet {
                light_circuit_index: 1,
                power_setting: 0.0,
            })
        );
    }

    #[test]
    fn output_data_follows_the_sim() {
        let (mut panel, _hw_rx) = running_panel();
        assert_eq!(panel.output_data(), [0, 0]);
        let aircraft_sim_state = |stby_batt_test_lamp, sw_cb_panels_light| AircraftSimState {
            parking_brake_indicator: false,
            gear_center_state: LandingGearStatus::Down,
            gear_left_state: LandingGearStatus::Down,
            gear_right_state: LandingGearStatus::Down,
            airspeed: 0.0,
            stby_batt_test_lamp,
            sw_cb_panels_light,
        };
        panel.aircraft_sim_state = Some(aircraft_sim_state(true, 0.5));
        assert_eq!(panel.output_data(), [1, 128]);
        panel.aircraft_sim_state = Some(aircraft_sim_state(false, 1.3));
        assert_eq!(panel.output_data(), [0, 255]);
        panel.sim_state.write().unwrap().sim_running = false;
        assert_eq!(panel.output_data(), [0, 0]);
    }
}
