use log::{debug, info};
use std::io::{BufRead, BufReader};
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use crate::panel::{Panel, PanelError};
use crate::Event;

/// The baud rate of the Arduino used for the serial connection.
const BAUD_RATE: u32 = 38400;

/// Represents the AirspeedIndicator Main Panel and holds all state and information.
#[derive(Debug)]
pub struct AirspeedIndicatorPanel {
    port: String,
    sim_rx: mpsc::Receiver<Event>,
}

impl Panel for AirspeedIndicatorPanel {
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
        // Wait for device to finish resetting
        thread::sleep(Duration::from_millis(2000));

        // Setup reader for initial device message
        let mut reader = BufReader::with_capacity(1, serial.try_clone()?);
        let mut buf = vec![];

        // Verify that we are connected to the correct arduino
        reader.read_until(b';', &mut buf)?;
        if String::from_utf8_lossy(&buf) == "Name<Airspeed-Indicator>;" {
            info!(
                "Connection with airspeed indicator panel established via {}",
                self.port
            );
        } else {
            return Err(PanelError::WrongDevice);
        }

        loop {
            // Receive control messages
            match self.sim_rx.try_recv() {
                Ok(Event::SetPanel(state)) => {
                    writeln!(
                        serial,
                        "Type<I-A>::Target<Airspeed-Indicator>::Content<{}>::Origin<Interface>;",
                        state.airspeed as i32
                    )?;
                }
                Err(mpsc::TryRecvError::Disconnected) => {
                    // The simconnect thread cannot exit, we exit always first
                    unreachable!();
                }
                _ => {}
            }
        }
    }
}

impl AirspeedIndicatorPanel {
    /// Create a new panel instance.
    pub fn new(port: impl AsRef<str>, sim_rx: mpsc::Receiver<Event>) -> Self {
        Self {
            sim_rx,
            port: port.as_ref().into(),
        }
    }
}
