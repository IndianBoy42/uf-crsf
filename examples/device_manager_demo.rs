//! Device Manager Example
//!
//! This example demonstrates how to use the DeviceManager to discover CRSF devices,
//! enumerate their parameters, and read/write parameter values.
//!
//! The example simulates a typical workflow:
//! 1. Discover devices on the bus
//! 2. Request all parameters from a discovered device
//! 3. Display parameter information
//! 4. Write a parameter value
//!
//! Note: This is a simulated example using mock data. In a real application,
//! you would interface with actual CRSF hardware via UART or similar.

use uf_crsf::device::{DeviceManager, DeviceManagerConfig};
use uf_crsf::packets::{
    DeviceInformation, Packet, PacketAddress, ParameterData, ParameterDataType,
    ParameterSettingsEntry,
};
use uf_crsf::parser::CrsfParser;

fn main() {
    println!("=== CRSF Device Manager Demo ===\n");

    // Create device manager with custom config
    let config = DeviceManagerConfig {
        timeout_ms: 500,
        retry_count: 3,
        device_ping_interval_ms: 1000,
    };
    let mut manager = DeviceManager::new(config);
    let mut parser = CrsfParser::new();

    // Simulate time
    let mut time_ms = 0u32;
    manager.update_time(time_ms);

    println!("Step 1: Sending device ping...");
    if let Some(ping_packet) = manager.send_device_ping(PacketAddress::Handset) {
        println!("  -> Ping packet created ({} bytes)", ping_packet.len());
        println!("     (In a real app, this would be sent over UART)\n");
    }

    // Simulate receiving a DeviceInformation response
    println!("Step 2: Simulating device discovery response...");
    let device_info = DeviceInformation::new(
        PacketAddress::Handset as u8,
        PacketAddress::Transmitter as u8,
        "ExpressLRS TX",
        0x454C5253, // "ELRS" in hex
        0x12345678,
        0x00030102, // Version 3.1.2
        18,         // 18 parameters
        1,          // Parameter version 1
    )
    .unwrap();

    let device_info_packet = Packet::DeviceInformation(device_info.clone());
    manager.handle_packet(&device_info_packet);

    println!("  -> Device discovered:");
    println!("     Name: {}", device_info.device_name());
    println!("     Serial: 0x{:08X}", device_info.serial_number);
    println!("     Firmware: 0x{:08X}", device_info.firmware_id);
    println!("     Parameters: {}\n", device_info.parameters_total);

    // Check discovered devices
    println!("Step 3: Enumerating discovered devices...");
    for device_addr in manager.devices() {
        if let Some(device) = manager.get_device(device_addr) {
            println!("  -> Device at address 0x{:02X}:", device_addr as u8);
            println!("     Name: {}", device.name);
            println!("     Is ELRS TX: {}", device.is_elrs_tx());
            println!(
                "     Parameters loaded: {}/{}",
                device.parameters.len(),
                device.parameters_total
            );
        }
    }
    println!();

    // Request all parameters from the discovered device
    println!("Step 4: Requesting all parameters from device...");
    if let Some(param_request) = manager.request_all_parameters(PacketAddress::Transmitter) {
        println!(
            "  -> Parameter read request created ({} bytes)",
            param_request.len()
        );
        println!("     (In a real app, this would be sent over UART)\n");
    }

    // Simulate receiving parameter responses
    println!("Step 5: Simulating parameter responses...");

    // Parameter 0: ROOT folder
    let root_folder = create_root_folder_parameter();
    let root_packet = Packet::ParameterSettingsEntry {
        parameter_id: 0,
        chunks_remaining: 0,
        entry: root_folder,
    };
    manager.handle_packet(&root_packet);
    println!("  -> Received parameter 0 (ROOT folder)");

    // Parameter 1: Packet Rate (Text Selection)
    let packet_rate = create_packet_rate_parameter();
    let rate_packet = Packet::ParameterSettingsEntry {
        parameter_id: 1,
        chunks_remaining: 0,
        entry: packet_rate,
    };
    manager.handle_packet(&rate_packet);
    println!("  -> Received parameter 1 (Packet Rate)");

    // Parameter 2: TX Power (Float)
    let tx_power = create_tx_power_parameter();
    let power_packet = Packet::ParameterSettingsEntry {
        parameter_id: 2,
        chunks_remaining: 0,
        entry: tx_power,
    };
    manager.handle_packet(&power_packet);
    println!("  -> Received parameter 2 (TX Power)");

    // Parameter 3: Device Name (String)
    let device_name_param = create_device_name_parameter();
    let name_packet = Packet::ParameterSettingsEntry {
        parameter_id: 3,
        chunks_remaining: 0,
        entry: device_name_param,
    };
    manager.handle_packet(&name_packet);
    println!("  -> Received parameter 3 (Device Name)");

    // Parameter 4: Bind command
    let bind_cmd = create_bind_command_parameter();
    let bind_packet = Packet::ParameterSettingsEntry {
        parameter_id: 4,
        chunks_remaining: 0,
        entry: bind_cmd,
    };
    manager.handle_packet(&bind_packet);
    println!("  -> Received parameter 4 (Bind Command)\n");

    // Display loaded parameters
    println!("Step 6: Displaying loaded parameters...");
    if let Some(device) = manager.get_device(PacketAddress::Transmitter) {
        println!("  Device: {}", device.name);
        println!(
            "  Parameters loaded: {}/{}\n",
            device.parameters.len(),
            device.parameters_total
        );

        for param in device.iter_parameters() {
            print!("  [{}] {} (parent: {})", param.id, param.name, param.parent);
            if param.hidden {
                print!(" [HIDDEN]");
            }
            println!();

            match &param.data {
                Some(ParameterData::Folder { children }) => {
                    println!("      Type: Folder");
                    println!("      Children: {:?}", children.as_slice());
                }
                Some(ParameterData::TextSelection { options, value, .. }) => {
                    println!("      Type: Text Selection");
                    println!("      Options: {}", options);
                    println!("      Value: {}", value);
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
                        "      Value: {:.prec$} {}",
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
                    println!("      Value: {}", value);
                    println!("      Max Length: {}", max_length);
                }
                Some(ParameterData::Command {
                    status,
                    timeout,
                    info,
                }) => {
                    println!("      Type: Command");
                    println!("      Status: {}", status);
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
    }

    // Demonstrate parameter writing
    println!("Step 7: Writing a parameter value...");
    // Write new packet rate value (change from 0 to 1)
    let write_data = [1u8]; // New value
    if let Some(write_packet) = manager.write_parameter(
        PacketAddress::Transmitter,
        1, // Parameter ID for Packet Rate
        &write_data,
    ) {
        println!(
            "  -> Parameter write packet created ({} bytes)",
            write_packet.len()
        );
        println!("     Parameter ID: 1 (Packet Rate)");
        println!("     New Value: 1");
        println!("     (In a real app, this would be sent over UART)\n");
    }

    // Simulate timeout and retry
    println!("Step 8: Demonstrating timeout and retry...");
    time_ms += 600; // Advance time past timeout
    manager.update_time(time_ms);

    let retry_packets = manager.process_timeouts();
    println!(
        "  -> Processed timeouts, {} retry packets generated",
        retry_packets.len()
    );
    for (i, packet) in retry_packets.iter().enumerate() {
        println!("     Retry packet {}: {} bytes", i + 1, packet.len());
    }

    println!("\n=== Demo Complete ===");
}

// Helper functions to create mock parameters

fn create_root_folder_parameter() -> ParameterSettingsEntry {
    use heapless::{String, Vec};

    let mut children = Vec::<u8, 32>::new();
    children.push(1).unwrap();
    children.push(2).unwrap();
    children.push(3).unwrap();
    children.push(4).unwrap();

    ParameterSettingsEntry::new(
        0,
        ParameterDataType::Folder as u8,
        "ROOT",
        Some(ParameterData::Folder { children }),
    )
    .unwrap()
}

fn create_packet_rate_parameter() -> ParameterSettingsEntry {
    use heapless::String;

    let options =
        String::try_from("50Hz(-117dBm);150Hz(-112dBm);250Hz(-108dBm);500Hz(-105dBm)").unwrap();
    let unit = String::try_from("Hz").unwrap();

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
    .unwrap()
}

fn create_tx_power_parameter() -> ParameterSettingsEntry {
    use heapless::String;

    let unit = String::try_from("mW").unwrap();

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
    .unwrap()
}

fn create_device_name_parameter() -> ParameterSettingsEntry {
    use heapless::String;

    let value = String::try_from("My ELRS TX").unwrap();

    ParameterSettingsEntry::new(
        0,
        ParameterDataType::String as u8,
        "Device Name",
        Some(ParameterData::String {
            value,
            max_length: 16,
        }),
    )
    .unwrap()
}

fn create_bind_command_parameter() -> ParameterSettingsEntry {
    use heapless::String;

    let info = String::try_from("Press to enter bind mode").unwrap();

    ParameterSettingsEntry::new(
        0,
        ParameterDataType::Command as u8,
        "Bind",
        Some(ParameterData::Command {
            status: 0,    // Idle
            timeout: 100, // 10 seconds
            info,
        }),
    )
    .unwrap()
}
