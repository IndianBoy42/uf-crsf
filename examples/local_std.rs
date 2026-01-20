//! Local Serial Port Parser Example
//!
//! This example demonstrates how to read raw CRSF data from a physical serial port
//! on a desktop OS (Linux, macOS, Windows) and parse it into structured packets.
//!
//! ### Real-world Integration vs. CLI Script
//! - **Main Loop:** In a long-running application, this loop would likely run in
//!   a dedicated thread to prevent UI blocking.
//! - **Error Handling:** A real app would attempt to reconnect if the serial port
//!   is unplugged or becomes unavailable.
//! - **Packet Processing:** Instead of just printing, you would route packets
//!   to different parts of your system (e.g., telemetry to a database, RC channels
//!   to a virtual joystick).
//!
//! ### Hardware/IO Considerations
//! - **Baud Rate:** CRSF standard is 420,000 baud. Some high-speed links (like
//!   ELRS at 1:2 ratio) can go up to 3.75 Mbps or 5.25 Mbps.
//! - **Standard OS Serial:** Standard serial libraries often have jitter or
//!   latency. For high-performance RC link monitoring, specialized drivers or
//!   low-latency settings (e.g., `AS_ASYNC` on Linux) might be needed.
//! - **Embedded microcontrollers:** You would replace `serialport` with a hardware
//!   UART driver (see `stm32demo` for an example).

use std::env;
use std::io::ErrorKind;
use std::process::exit;
use std::time::Duration;
use uf_crsf::CrsfParser;

fn main() {
    // 1. Enumerate and select a serial port.
    // In a real app, you might provide a GUI dropdown or a config file.
    let ports = match serialport::available_ports() {
        Ok(ports) => ports,
        Err(e) => {
            eprintln!("Failed to enumerate serial ports: {}", e);
            exit(1);
        }
    };

    if ports.is_empty() {
        eprintln!("No serial ports found.");
        eprintln!("Please specify a serial port path as an argument.");
        exit(1);
    }

    let path = env::args().nth(1).unwrap_or_else(|| {
        const DEFAULT_PORT: &str = "/dev/tty.usbmodem00000000001B1";
        if ports.iter().any(|p| p.port_name == DEFAULT_PORT) {
            println!(
                "No serial port specified. Using default port: {}",
                DEFAULT_PORT
            );
            DEFAULT_PORT.to_string()
        } else {
            println!("No serial port specified. Available ports:");
            for p in &ports {
                println!("  {}", p.port_name);
            }
            println!("\nUsing first available port: {}", ports[0].port_name);
            ports[0].port_name.clone()
        }
    });

    // 2. Open the port at the standard CRSF baud rate.
    // 420,000 is the most common rate for handset-to-module communication.
    let mut port = serialport::new(&path, 420_000)
        .timeout(Duration::from_millis(10))
        .open()
        .unwrap_or_else(|e| {
            eprintln!("Failed to open serial port '{}': {}", &path, e);
            exit(1);
        });

    let mut buf = [0; 1024];
    let mut parser = CrsfParser::new();
    println!("Reading from serial port '{}'...", path);

    // 3. Continuous processing loop.
    loop {
        match port.read(buf.as_mut_slice()) {
            Ok(n) => {
                // The parser's `iter_packets` helper makes it easy to handle
                // multiple packets arriving in a single read buffer, or packets
                // split across multiple reads.
                for packet in parser.iter_packets(&buf[..n]) {
                    // This is where you would handle specific packet types.
                    // For example:
                    // match packet {
                    //     Packet::RCChannels(channels) => handle_rc(channels),
                    //     Packet::LinkStatistics(stats) => update_signal_quality(stats),
                    //     _ => {}
                    // }
                    println!("{:?}", packet);
                }
            }
            Err(ref e) if e.kind() == ErrorKind::TimedOut => {
                // This is expected when no data is coming in.
                // In a real app, you might use this to check for signal loss.
            }
            Err(e) => {
                eprintln!("Error reading from serial port: {}", e);
                break;
            }
        }
    }
}
