use serialport::SerialPort;
use simconnect;
use std::io::Write;
use std::{
    io::{BufRead, BufReader},
    sync::{atomic::AtomicBool, mpsc},
    time::Duration,
};
use std::{mem, thread};

/// The baud rate of the Arduino used for the serial connection.
const BAUD_RATE: u32 = 115200;

#[derive(Debug)]
struct DataStruct {
    gear_center_pos: f32,
}

#[derive(Debug)]
enum Event {
    Keepalive,
    SynAck,
    /// Reset the connection to the Arduino
    Reset,
    Command(String),
}

fn run_simconnect() {
    let mut conn = simconnect::SimConnector::new();
    conn.connect("FSSK EventSim Main Panel");
    // Assign a sim variable to a client defined id
    conn.add_data_definition(
        0,
        "GEAR CENTER POSITION",
        "percent over 100",
        simconnect::SIMCONNECT_DATATYPE_SIMCONNECT_DATATYPE_FLOAT32,
        0,
    );
    conn.request_data_on_sim_object(
        0,
        0,
        0,
        simconnect::SIMCONNECT_PERIOD_SIMCONNECT_PERIOD_VISUAL_FRAME,
        simconnect::SIMCONNECT_DATA_REQUEST_FLAG_CHANGED,
        0,
        3,
        0,
    ); //request_id, define_id, object_id (user), period, falgs, origin, interval, limit - tells simconnect to send data for the defined id and on the user aircraft

    loop {
        match conn.get_next_message() {
            Ok(simconnect::DispatchResult::SimobjectData(data)) => unsafe {
                match data.dwDefineID {
                    0 => {
                        #[allow(unaligned_references)]
                        let sim_data: DataStruct = mem::transmute_copy(&data.dwData);
                        println!("{:?}", sim_data.gear_center_pos);
                    }
                    _ => (),
                }
            },
            _ => (),
        }

        // Will use up lots of CPU if this is not included, as get_next_message() is non-blocking
        thread::sleep(Duration::from_millis(1000 / 120));
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
        .open_native()
        .expect("Failed to open port");

    // Serial reader thread
    let serial_event_tx = event_tx.clone();
    let port_rx = port_tx.try_clone()?;
    std::thread::spawn(move || run_serial_reader(serial_event_tx, port_rx));

    // SimConnect thread
    std::thread::spawn(run_simconnect);
    // Periodic keepalive event thread
    let keepalive_event_tx = event_tx;
    std::thread::spawn(move || run_keepalive(keepalive_event_tx));

    // Initiate handshake with the Arduino
    writeln!(port_tx, "SYN")?;

    for event in event_rx {
        println!("Received event {:?}", &event);
        match event {
            Event::Keepalive if *connected.get_mut() => writeln!(port_tx, "PING")?,
            Event::SynAck => {
                writeln!(port_tx, "ACK")?;
                *connected.get_mut() = true;
            }
            Event::Reset => *connected.get_mut() = false,
            _ => {}
        }
    }

    Ok(())
}
