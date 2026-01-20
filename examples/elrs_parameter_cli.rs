//! ELRS Parameter CLI Example
//!
//! This is a command-line utility demonstrating how to use DeviceManager and CrsfParser
//! to interact with a simulated ELRS device.
//!
//! Features:
//! - Discover ELRS devices on the bus
//! - List all parameters with their current values
//! - Get detailed information about a specific parameter
//! - Set parameter values
//!
//! Note: This is a simulation using mock data. In a real application,
//! you would interface with actual CRSF hardware via UART or similar.

use heapless::Vec;
use std::io::{self, Write};
use std::string::String as StdString;
use uf_crsf::device::{DeviceManager, DeviceManagerConfig, Parameter};
use uf_crsf::packets::{
    DeviceInformation, Packet, PacketAddress, ParameterData, ParameterDataType,
    ParameterSettingsEntry,
};
use uf_crsf::parser::CrsfParser;

/// Mock ELRS device simulator
struct MockElrsDevice {
    device_info: DeviceInformation,
    parameters: Vec<ParameterSettingsEntry, 16>,
    param_values: heapless::Vec<Option<Vec<u8, 4>>, 16>,
}

impl MockElrsDevice {
    fn new() -> Self {
        let device_info = DeviceInformation::new(
            PacketAddress::Handset as u8,
            PacketAddress::Transmitter as u8,
            "ELRS TX",
            0x454C5253, // "ELRS"
            0x12345678,
            0x00030102,
            6,
            1,
        )
        .unwrap();

        let mut parameters = Vec::new();
        let mut param_values = heapless::Vec::new();

        // Parameter 0: ROOT folder
        let mut root_children = Vec::new();
        root_children.push(1).unwrap();
        root_children.push(2).unwrap();
        root_children.push(3).unwrap();
        root_children.push(4).unwrap();
        root_children.push(5).unwrap();
        parameters
            .push(
                ParameterSettingsEntry::new(
                    0,
                    ParameterDataType::Folder as u8,
                    "ROOT",
                    Some(ParameterData::Folder {
                        children: root_children,
                    }),
                )
                .unwrap(),
            )
            .unwrap();
        param_values.push(None).unwrap();

        // Parameter 1: Packet Rate
        let options = heapless::String::<128>::try_from(
            "50Hz(-117dBm);150Hz(-112dBm);250Hz(-108dBm);500Hz(-105dBm)",
        )
        .unwrap();
        let unit = heapless::String::<128>::try_from("Hz").unwrap();
        parameters
            .push(
                ParameterSettingsEntry::new(
                    0,
                    ParameterDataType::TextSelection as u8,
                    "Packet Rate",
                    Some(ParameterData::TextSelection {
                        options,
                        value: 2,
                        min: 0,
                        max: 3,
                        default: 2,
                        unit,
                    }),
                )
                .unwrap(),
            )
            .unwrap();
        let mut value = Vec::new();
        value.push(2).unwrap();
        param_values.push(Some(value)).unwrap();

        // Parameter 2: TX Power
        let unit = heapless::String::<128>::try_from("mW").unwrap();
        parameters
            .push(
                ParameterSettingsEntry::new(
                    0,
                    ParameterDataType::Float as u8,
                    "TX Power",
                    Some(ParameterData::Float {
                        value: 100,
                        min: 10,
                        max: 1000,
                        default: 100,
                        decimal_point: 0,
                        step_size: 10,
                        unit,
                    }),
                )
                .unwrap(),
            )
            .unwrap();
        let mut value = Vec::new();
        value.extend_from_slice(&100i32.to_be_bytes()).unwrap();
        param_values.push(Some(value)).unwrap();

        // Parameter 3: Switch Mode
        let options =
            heapless::String::<128>::try_from("No Acks;1;2;3;4;5;6;7;8;9;10;11;12;13;14;15;16")
                .unwrap();
        let unit = heapless::String::<128>::try_from("").unwrap();
        parameters
            .push(
                ParameterSettingsEntry::new(
                    0,
                    ParameterDataType::TextSelection as u8,
                    "Switch Mode",
                    Some(ParameterData::TextSelection {
                        options,
                        value: 0,
                        min: 0,
                        max: 16,
                        default: 0,
                        unit,
                    }),
                )
                .unwrap(),
            )
            .unwrap();
        let mut value = Vec::new();
        value.push(0).unwrap();
        param_values.push(Some(value)).unwrap();

        // Parameter 4: Binding
        let info = heapless::String::<128>::try_from("Press to enter bind mode").unwrap();
        parameters
            .push(
                ParameterSettingsEntry::new(
                    0,
                    ParameterDataType::Command as u8,
                    "Bind",
                    Some(ParameterData::Command {
                        status: 0,
                        timeout: 100,
                        info,
                    }),
                )
                .unwrap(),
            )
            .unwrap();
        param_values.push(None).unwrap();

        // Parameter 5: Telemetry Rate
        let options =
            heapless::String::<128>::try_from("Off;4Hz;8Hz;16Hz;32Hz;64Hz;128Hz").unwrap();
        let unit = heapless::String::<128>::try_from("Hz").unwrap();
        parameters
            .push(
                ParameterSettingsEntry::new(
                    0,
                    ParameterDataType::TextSelection as u8,
                    "Telemetry Rate",
                    Some(ParameterData::TextSelection {
                        options,
                        value: 5,
                        min: 0,
                        max: 6,
                        default: 5,
                        unit,
                    }),
                )
                .unwrap(),
            )
            .unwrap();
        let mut value = Vec::new();
        value.push(5).unwrap();
        param_values.push(Some(value)).unwrap();

        Self {
            device_info,
            parameters,
            param_values,
        }
    }

    fn handle_read(&mut self, param_id: u8) -> Option<Packet> {
        if (param_id as usize) < self.parameters.len() {
            Some(Packet::ParameterSettingsEntry {
                parameter_id: param_id,
                chunks_remaining: 0,
                entry: self.parameters[param_id as usize].clone(),
            })
        } else {
            None
        }
    }

    fn handle_write(&mut self, param_id: u8, data: &[u8]) -> bool {
        if (param_id as usize) < self.param_values.len() {
            let mut value = Vec::new();
            for &byte in data {
                if value.push(byte).is_err() {
                    return false;
                }
            }
            self.param_values[param_id as usize] = Some(value);
            true
        } else {
            false
        }
    }

    fn update_parameter_value(&mut self, param_id: u8, value: &[u8]) {
        if let Some(entry) = self.parameters.get_mut(param_id as usize) {
            if let Some(ref mut data) = entry.data {
                match data {
                    ParameterData::TextSelection { value: v, .. } => {
                        if !value.is_empty() {
                            *v = value[0];
                        }
                    }
                    ParameterData::Float {
                        value: v,
                        decimal_point: _,
                        ..
                    } => {
                        if value.len() >= 4 {
                            let bytes: [u8; 4] = [value[0], value[1], value[2], value[3]];
                            *v = i32::from_be_bytes(bytes);
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}

fn print_help() {
    println!("ELRS Parameter CLI");
    println!("==================");
    println!();
    println!("Available commands:");
    println!("  list              - List all parameters");
    println!("  show <id>         - Show detailed info about parameter <id>");
    println!("  set <id> <value> - Set parameter <id> to <value>");
    println!("  discover          - Discover ELRS devices");
    println!("  devices           - List discovered devices");
    println!("  help              - Show this help message");
    println!("  quit, exit        - Exit the CLI");
    println!();
}

fn print_parameter(param: &Parameter) {
    println!("  [{}] {} (parent: {})", param.id, param.name, param.parent);
    if param.hidden {
        println!("      [HIDDEN]");
    }

    match &param.data {
        Some(ParameterData::Folder { children }) => {
            println!("      Type: Folder");
            println!("      Children: {:?}", children.as_slice());
        }
        Some(ParameterData::TextSelection { options, value, .. }) => {
            println!("      Type: Text Selection");
            println!("      Options: {}", options);
            println!("      Current Value: {}", value);

            // Parse and display option labels
            for (i, option) in options.split(';').enumerate() {
                let marker = if i as u8 == *value { " <=" } else { "" };
                println!("        [{}] {}{}", i, option, marker);
            }
        }
        Some(ParameterData::Float {
            value,
            min,
            max,
            unit,
            decimal_point,
            ..
        }) => {
            println!("      Type: Float");
            let divisor = 10_i32.pow(*decimal_point as u32);
            println!(
                "      Current Value: {:.prec$} {}",
                *value as f32 / divisor as f32,
                unit,
                prec = *decimal_point as usize
            );
            println!(
                "      Range: {:.prec$} to {:.prec$}",
                *min as f32 / divisor as f32,
                *max as f32 / divisor as f32,
                prec = *decimal_point as usize
            );
        }
        Some(ParameterData::String { value, max_length }) => {
            println!("      Type: String");
            println!("      Current Value: {}", value);
            println!("      Max Length: {}", max_length);
        }
        Some(ParameterData::Command {
            status,
            timeout,
            info,
        }) => {
            println!("      Type: Command");
            println!(
                "      Status: {} (0=Idle, 1=Running, 2=Executing, 3=Complete)",
                status
            );
            println!("      Timeout: {} ms", *timeout as u32 * 100);
            println!("      Info: {}", info);
        }
        Some(ParameterData::Info { info }) => {
            println!("      Type: Info");
            println!("      Info: {}", info);
        }
        Some(ParameterData::Vtx { data }) => {
            println!("      Type: VTX");
            println!("      Data: {:?}", data.as_slice());
        }
        None => {
            println!("      Type: Unknown (no data)");
        }
    }
    println!();
}

fn main() {
    println!("=== ELRS Parameter CLI ===");
    println!("Simulating CRSF device communication\n");

    let config = DeviceManagerConfig::default();
    let mut manager = DeviceManager::new(config);
    let mut mock_device = MockElrsDevice::new();
    let _parser = CrsfParser::new();

    // Simulate device discovery
    println!("Discovering devices...");
    let device_info_packet = Packet::DeviceInformation(mock_device.device_info.clone());
    manager.handle_packet(&device_info_packet);
    println!("Found {} device(s)\n", manager.devices().count());

    print_help();

    let mut time_ms = 0u32;

    loop {
        print!("elrs> ");
        io::stdout().flush().unwrap();

        let mut input = StdString::new();
        io::stdin()
            .read_line(&mut input)
            .expect("Failed to read line");

        let input = input.trim();
        let parts: std::vec::Vec<&str> = input.split_whitespace().collect();

        if parts.is_empty() {
            continue;
        }

        let command = parts[0].to_lowercase();

        match command.as_str() {
            "help" => {
                print_help();
            }
            "quit" | "exit" => {
                println!("Goodbye!");
                break;
            }
            "list" => {
                println!("\n--- Parameters ---");
                let mut found_device = false;
                let devices: std::vec::Vec<_> = manager.devices().collect();

                for device_addr in devices {
                    if let Some(device) = manager.get_device(device_addr) {
                        let device_name = device.name.clone();
                        let is_elrs = device.is_elrs_tx();
                        let params_len = device.parameters.len();
                        let params_total = device.parameters_total;

                        println!("\nDevice: {}", device_name);
                        println!("Is ELRS: {}", is_elrs);
                        println!("Parameters: {}/{}\n", params_len, params_total);

                        // Request all parameters from mock device
                        for i in 0..mock_device.parameters.len() {
                            if let Some(packet) = mock_device.handle_read(i as u8) {
                                manager.handle_packet(&packet);
                            }
                        }

                        // Re-fetch device after adding parameters
                        if let Some(updated_device) = manager.get_device(device_addr) {
                            // Display loaded parameters
                            for param in updated_device.iter_parameters() {
                                print_parameter(param);
                            }
                        }
                        found_device = true;
                    }
                }
                if !found_device {
                    println!("No devices found. Use 'discover' first.");
                }
            }
            "show" => {
                if parts.len() < 2 {
                    println!("Usage: show <id>");
                    continue;
                }

                let param_id: u8 = match parts[1].parse() {
                    Ok(id) => id,
                    Err(_) => {
                        println!("Invalid parameter ID");
                        continue;
                    }
                };

                let mut found = false;
                for device_addr in manager.devices() {
                    if let Some(device) = manager.get_device(device_addr) {
                        if let Some(param) = device.get_parameter(param_id) {
                            println!("\n--- Parameter Details ---");
                            print_parameter(param);
                            found = true;
                        }
                    }
                }

                if !found {
                    println!(
                        "Parameter {} not found. Use 'list' to see available parameters.",
                        param_id
                    );
                }
            }
            "set" => {
                if parts.len() < 3 {
                    println!("Usage: set <id> <value>");
                    continue;
                }

                let param_id: u8 = match parts[1].parse() {
                    Ok(id) => id,
                    Err(_) => {
                        println!("Invalid parameter ID");
                        continue;
                    }
                };

                let mut found = false;
                let devices: std::vec::Vec<_> = manager.devices().collect();

                for device_addr in devices {
                    if let Some(device) = manager.get_device(device_addr) {
                        if let Some(param) = device.get_parameter(param_id) {
                            match &param.data {
                                Some(ParameterData::TextSelection { .. }) => {
                                    let value: u8 = match parts[2].parse() {
                                        Ok(v) => v,
                                        Err(_) => {
                                            println!("Invalid value");
                                            continue;
                                        }
                                    };

                                    let write_data = [value];
                                    if mock_device.handle_write(param_id, &write_data) {
                                        mock_device.update_parameter_value(param_id, &write_data);
                                        if let Some(write_packet) = manager.write_parameter(
                                            device_addr,
                                            param_id,
                                            &write_data,
                                        ) {
                                            println!("Parameter {} set to {}", param_id, value);
                                            println!(
                                                "Write packet created ({} bytes)",
                                                write_packet.len()
                                            );
                                        } else {
                                            println!("Failed to write parameter");
                                        }
                                    } else {
                                        println!("Failed to write parameter");
                                    }
                                }
                                Some(ParameterData::Float { .. }) => {
                                    let value: i32 = match parts[2].parse() {
                                        Ok(v) => v,
                                        Err(_) => {
                                            println!("Invalid value");
                                            continue;
                                        }
                                    };

                                    let write_data = value.to_be_bytes();
                                    if mock_device.handle_write(param_id, &write_data) {
                                        mock_device.update_parameter_value(param_id, &write_data);
                                        if let Some(write_packet) = manager.write_parameter(
                                            device_addr,
                                            param_id,
                                            &write_data,
                                        ) {
                                            println!("Parameter {} set to {}", param_id, value);
                                            println!(
                                                "Write packet created ({} bytes)",
                                                write_packet.len()
                                            );
                                        } else {
                                            println!("Failed to write parameter");
                                        }
                                    } else {
                                        println!("Failed to write parameter");
                                    }
                                }
                                _ => {
                                    println!("Parameter type not supported for writing");
                                }
                            }
                            found = true;
                        }
                    }
                }

                if !found {
                    println!(
                        "Parameter {} not found. Use 'list' to see available parameters.",
                        param_id
                    );
                }
            }
            "discover" => {
                println!("\nDiscovering devices...");
                // Send device ping
                if let Some(ping_packet) = manager.send_device_ping(PacketAddress::Handset) {
                    println!("Sent ping packet ({} bytes)", ping_packet.len());
                }

                // Simulate receiving device info
                let device_info_packet = Packet::DeviceInformation(mock_device.device_info.clone());
                manager.handle_packet(&device_info_packet);
                println!("Found {} device(s)\n", manager.devices().count());

                for device_addr in manager.devices() {
                    if let Some(device) = manager.get_device(device_addr) {
                        println!("  Device: {}", device.name);
                        println!("  Address: 0x{:02X}", device_addr as u8);
                        println!("  Serial: 0x{:08X}", device.serial_number);
                        println!("  Is ELRS: {}", device.is_elrs_tx());
                        println!("  Parameters: {}", device.parameters_total);
                        println!();
                    }
                }
            }
            "devices" => {
                println!("\n--- Discovered Devices ---");
                let count = manager.devices().count();
                if count == 0 {
                    println!("No devices found. Use 'discover' first.");
                } else {
                    println!("Found {} device(s)\n", count);
                    for device_addr in manager.devices() {
                        if let Some(device) = manager.get_device(device_addr) {
                            println!("  Device: {}", device.name);
                            println!("  Address: 0x{:02X}", device_addr as u8);
                            println!("  Serial: 0x{:08X}", device.serial_number);
                            println!("  Is ELRS: {}", device.is_elrs_tx());
                            println!(
                                "  Parameters: {}/{} loaded",
                                device.parameters.len(),
                                device.parameters_total
                            );
                            println!();
                        }
                    }
                }
            }
            _ => {
                println!(
                    "Unknown command: '{}'. Type 'help' for available commands.",
                    command
                );
            }
        }

        // Update time and process timeouts
        time_ms += 100;
        manager.update_time(time_ms);
        let retry_packets = manager.process_timeouts();
        for (i, packet) in retry_packets.iter().enumerate() {
            println!("Retry packet {}: {} bytes", i + 1, packet.len());
        }
    }
}
