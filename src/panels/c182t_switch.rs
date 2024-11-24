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
use crate::sim::FuelSystemPumpStatus;
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
    CabinPower12V,
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
    switch_states: BitField<u16>,
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
            let buf = vec![!self.switch_states.is_set(Switch::StbyBatteryTest) as u8];
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
    ) -> Self {
        Self {
            sim_state,
            hw_tx,
            port: port.as_ref().into(),
            switch_states: BitField::new(0),
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
        self.switch_states
            .when_changed(Switch::Alternator, |state| {
                self.send_sim_event(SimClientEvent::AlternatorSet {
                    state,
                    alternator_index: 1,
                })
            });
        self.switch_states.when_changed(Switch::Battery, |state| {
            self.send_sim_event(SimClientEvent::Battery1Set(state));
        });
        self.switch_states.when_changed(Switch::Avionics1, |state| {
            self.send_sim_event(SimClientEvent::AvionicsMaster1Set(state));
        });
        self.switch_states.when_changed(Switch::Avionics2, |state| {
            self.send_sim_event(SimClientEvent::AvionicsMaster2Set(state));
        });
        self.switch_states
            .when_changed(Switch::StbyBatteryArm, |state| {
                self.send_sim_event(SimClientEvent::Battery2Set(!state));
            });
        self.switch_states.when_changed(Switch::Beacon, |state| {
            self.send_sim_event(if !state {
                SimClientEvent::BeaconLightOn
            } else {
                SimClientEvent::BeaconLightOff
            });
        });
        self.switch_states.when_changed(Switch::Nav, |state| {
            self.send_sim_event(if !state {
                SimClientEvent::NavLightsOn
            } else {
                SimClientEvent::NavLightsOff
            });
        });
        self.switch_states.when_changed(Switch::Strobe, |state| {
            self.send_sim_event(if !state {
                SimClientEvent::StrobeLightsOn
            } else {
                SimClientEvent::StrobeLightsOff
            });
        });
        if self.switch_states.has_changed(Switch::Taxi)
            || self.switch_states.has_changed(Switch::Landing)
        {
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
            }
        };
        self.switch_states.when_changed(Switch::FuelPump, |state| {
            let pump_status = match !state {
                true => FuelSystemPumpStatus::On,
                false => FuelSystemPumpStatus::Off,
            };
            self.send_sim_event(SimClientEvent::ElecFuelPump1Set(pump_status));
        });
        self.switch_states.when_changed(Switch::PitotHeat, |state| {
            self.send_sim_event(if !state {
                SimClientEvent::PitotHeatOn
            } else {
                SimClientEvent::PitotHeatOff
            });
        });
        // Send dimmer positions in simulator
        let panel_dimmer = (u8::MAX - data[2]) as f32 / 255.0;
        self.send_sim_event(SimClientEvent::LightPotentiometerSet {
            index: 3,
            potentiometer_value: panel_dimmer,
        });
        let stby_instrument_dimmer = (u8::MAX - data[3]) as f32 / 255.0;
        self.send_sim_event(SimClientEvent::PanelLightsPowerSettingSet {
            light_circuit_index: 1,
            power_setting: stby_instrument_dimmer,
        });
        let pedestal_dimmer = (u8::MAX - data[4]) as f32 / 255.0;
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
}
