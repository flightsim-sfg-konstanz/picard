use log::debug;
use log::warn;
use std::io::BufRead;
use std::io::BufReader;
use std::sync::mpsc;
use std::sync::Arc;
use std::sync::RwLock;
use std::thread;
use std::time::Duration;

use crate::panel::Panel;
use crate::panel::PanelError;
use crate::sim::AircraftSimState;
use crate::sim::SimClientEvent;
use crate::Event;
use crate::SimState;

/// The baud rate of the Arduino used for the serial connection.
const BAUD_RATE: u32 = 115200;

/// The number of data bytes in a normal COBS frame
const DATA_LEN: usize = 1;

/// The position from 0 to 1 at which a brake counts as applied
const BRAKE_APPLIED_POSITION: f64 = 0.5;

/// Represents the parking brake panel and holds all state and information.
#[derive(Debug)]
pub struct ParkingBrakePanel {
    sim_state: Arc<RwLock<SimState>>,
    port: String,
    hw_tx: mpsc::Sender<Event>,
    sim_rx: mpsc::Receiver<Event>,
    aircraft_sim_state: Option<AircraftSimState>,
    parking_brake_set: bool,
}

impl Panel for ParkingBrakePanel {
    /// Connect to the panel and run an event loop.
    fn run(&mut self) -> Result<(), PanelError> {
        debug!(
            "Attempting to connect to parking brake panel on serial port {}",
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

        let reader = BufReader::with_capacity(1, serial);
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
                    // Ignore timeouts
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                    // Exit on all other errors
                    Err(e) => return Err(e.into()),
                }
            }
        }
    }
}

impl ParkingBrakePanel {
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
            aircraft_sim_state: None,
            // Keep the parking brake set if the handle is pulled when we start
            parking_brake_set: true,
        }
    }

    fn handle_serial_frame(&mut self, data: &[u8], data_len: usize) {
        if data_len != DATA_LEN {
            warn!(
                "Parking brake panel command is of unexpected length {}",
                data_len
            );
            return;
        }

        // The switch contact is closed unless the handle is pushed in fully
        let handle_pulled = data[0] == 0;
        let sim_running = self.sim_state.read().unwrap().sim_running;
        self.parking_brake_set = match sim_running {
            // Like the parking brake valve, hold the brakes only if they are applied while the handle is pulled
            true => handle_pulled && (self.parking_brake_set || self.brakes_applied()),
            // A flight is loaded with the parking brake set if the handle is pulled
            false => handle_pulled,
        };

        // Set parking brake in simulator
        match self.parking_brake_set {
            true => self.send_sim_event(SimClientEvent::ParkingBrakeOn),
            false => self.send_sim_event(SimClientEvent::ParkingBrakeOff),
        };
    }

    /// Check whether both brakes are applied in the simulator
    fn brakes_applied(&self) -> bool {
        match &self.aircraft_sim_state {
            Some(state) => {
                state.brake_left_position >= BRAKE_APPLIED_POSITION
                    && state.brake_right_position >= BRAKE_APPLIED_POSITION
            }
            None => false,
        }
    }

    fn send_sim_event(&self, event: SimClientEvent) {
        if self.sim_state.read().unwrap().sim_running {
            self.hw_tx
                .send(Event::SetSimulator(event))
                .expect("SimConnect thread offline");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::LandingGearStatus;

    /// Create a panel for a running sim and the receiver of its events
    fn running_panel() -> (ParkingBrakePanel, mpsc::Receiver<Event>) {
        let mut sim_state = SimState::new();
        sim_state.sim_running = true;
        let (hw_tx, hw_rx) = mpsc::channel();
        let (_sim_tx, sim_rx) = mpsc::channel();
        let panel = ParkingBrakePanel::new(Arc::new(RwLock::new(sim_state)), "COM0", hw_tx, sim_rx);
        (panel, hw_rx)
    }

    /// Set the brake positions of the sim
    fn set_brakes(panel: &mut ParkingBrakePanel, left: f64, right: f64) {
        panel.aircraft_sim_state = Some(AircraftSimState {
            parking_brake_indicator: false,
            gear_center_state: LandingGearStatus::Down,
            gear_left_state: LandingGearStatus::Down,
            gear_right_state: LandingGearStatus::Down,
            airspeed: 0.0,
            brake_left_position: left,
            brake_right_position: right,
            stby_batt_test_lamp: false,
            sw_cb_panels_light: 0.0,
        });
    }

    /// Get the sim events of a frame with the handle in the given position
    fn events_for(
        panel: &mut ParkingBrakePanel,
        hw_rx: &mpsc::Receiver<Event>,
        handle_pulled: bool,
    ) -> Vec<SimClientEvent> {
        let data = [if handle_pulled { 0 } else { 1 }];
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
    fn parking_brake_is_only_set_with_the_brakes_applied() {
        let (mut panel, hw_rx) = running_panel();
        let on = vec![SimClientEvent::ParkingBrakeOn];
        let off = vec![SimClientEvent::ParkingBrakeOff];
        assert_eq!(events_for(&mut panel, &hw_rx, false), off);

        // Pulled without brakes
        set_brakes(&mut panel, 0.0, 0.0);
        assert_eq!(events_for(&mut panel, &hw_rx, true), off);
        // Only one brake applied
        set_brakes(&mut panel, 1.0, 0.2);
        assert_eq!(events_for(&mut panel, &hw_rx, true), off);
        // Both brakes applied while the handle is pulled
        set_brakes(&mut panel, 0.6, 0.5);
        assert_eq!(events_for(&mut panel, &hw_rx, true), on);
        // The brakes stay set after the pedals are released
        set_brakes(&mut panel, 0.0, 0.0);
        assert_eq!(events_for(&mut panel, &hw_rx, true), on);
        // Pushing the handle in releases the brakes
        assert_eq!(events_for(&mut panel, &hw_rx, false), off);
        assert_eq!(events_for(&mut panel, &hw_rx, true), off);
        // Applying the brakes with the handle pushed in does not set the parking brake
        set_brakes(&mut panel, 1.0, 1.0);
        assert_eq!(events_for(&mut panel, &hw_rx, false), off);
        assert_eq!(events_for(&mut panel, &hw_rx, true), on);
    }

    #[test]
    fn parking_brake_follows_the_handle_on_load() {
        let (mut panel, hw_rx) = running_panel();
        set_brakes(&mut panel, 0.0, 0.0);

        // Loaded with the handle pulled
        panel.sim_state.write().unwrap().sim_running = false;
        assert!(events_for(&mut panel, &hw_rx, false).is_empty());
        assert!(events_for(&mut panel, &hw_rx, true).is_empty());
        panel.sim_state.write().unwrap().sim_running = true;
        assert_eq!(
            events_for(&mut panel, &hw_rx, true),
            vec![SimClientEvent::ParkingBrakeOn]
        );

        // Loaded with the handle pushed in
        panel.sim_state.write().unwrap().sim_running = false;
        assert!(events_for(&mut panel, &hw_rx, false).is_empty());
        panel.sim_state.write().unwrap().sim_running = true;
        assert_eq!(
            events_for(&mut panel, &hw_rx, true),
            vec![SimClientEvent::ParkingBrakeOff]
        );
    }

    #[test]
    fn parking_brake_follows_the_handle_on_start() {
        let (mut panel, hw_rx) = running_panel();
        assert_eq!(
            events_for(&mut panel, &hw_rx, true),
            vec![SimClientEvent::ParkingBrakeOn]
        );
        let (mut panel, hw_rx) = running_panel();
        assert_eq!(
            events_for(&mut panel, &hw_rx, false),
            vec![SimClientEvent::ParkingBrakeOff]
        );
        assert_eq!(
            events_for(&mut panel, &hw_rx, true),
            vec![SimClientEvent::ParkingBrakeOff]
        );
    }
}
