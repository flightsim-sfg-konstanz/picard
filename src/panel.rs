use core::fmt;

/// Errors related to the panel.
#[derive(Debug)]
pub enum PanelError {
    /// Failed to open the serial port
    SerialOpen(String, serialport::Error),
    /// The panel was or is disconnected
    Disconnect,
    /// Error that relates to the serial port
    Serial(serialport::Error),
    /// I/O error that wraps the standard error type
    Io(std::io::Error),
}

impl fmt::Display for PanelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PanelError::SerialOpen(port, e) => {
                write!(
                    f,
                    "Failed to connect with panel on serial port '{port}': {e}"
                )
            }
            PanelError::Disconnect => write!(f, "The panel disconnected"),
            PanelError::Serial(e) => write!(f, "Serial communication error: {}", e),
            PanelError::Io(e) => write!(f, "Panel I/O error: {}", e),
        }
    }
}

impl std::error::Error for PanelError {}

impl From<serialport::Error> for PanelError {
    fn from(value: serialport::Error) -> Self {
        Self::Serial(value)
    }
}

impl From<std::io::Error> for PanelError {
    fn from(value: std::io::Error) -> Self {
        Self::Io(value)
    }
}
