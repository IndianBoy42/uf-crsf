//! Device Manager Example
//!
//! This example demonstrates how to use the `DeviceManager` to discover CRSF devices,
//! enumerate their parameters, and read/write parameter values.
//!
//! ### Real-world Integration vs. CLI Script
//! In this short script, we simulate time and hardware responses linearly. In a real
//! long-running application (e.g., a ground station or a Lua script runner):
//! - **Main Loop:** You would have a continuous loop or an async task reading from
//!   a UART port. Every byte received would be passed to `CrsfParser`, and every
//!   complete `Packet` would be passed to `DeviceManager::handle_packet`.
//! - **Time Management:** `manager.update_time()` must be called with a monotonic
//!   millisecond timer, and `manager.process_timeouts()` should be checked
//!   frequently (e.g., every 10-100ms) to trigger retries for lost packets.
//! - **Concurrency:** In multi-threaded environments, the `DeviceManager` should
//!   be protected by a Mutex or managed by a dedicated communication thread.
//!
//! ### Hardware/IO Considerations
//! - **CLI/Desktop:** Uses standard OS serial APIs (e.g., `serialport` crate).
//! - **Embedded (MCU):** Uses hardware UART peripherals. Typically, you'd use:
//!     - **Interrupts/DMA:** To fill a circular buffer from the UART.
//!     - **Non-blocking Read:** Poll the buffer and feed it to the parser.
//!     - **Blocking Write:** Send generated packets back to the UART.
//!
//! The `DeviceManager` itself is `no_std` and allocator-free, making it suitable
//! for both environments.

use uf_crsf::device::{DeviceManager, DeviceManagerConfig};
use uf_crsf::packets::{
    DeviceInformation, Packet, PacketAddress, ParameterData, ParameterDataType,
    ParameterSettingsEntry,
};
use uf_crsf::parser::CrsfParser;

fn main() {
    println!("=== CRSF Device Manager Demo ===\n");

    // Configure the manager.
    // CRSF is a half-duplex protocol often running over a lossy link (like a
    // wireless handset-to-module connection). Timeouts and retries are
    // essential for a robust UI experience.
    let config = DeviceManagerConfig {
        timeout_ms: 500,               // How long to wait for a response before retrying
        retry_count: 3,                // Number of attempts before giving up on a request
        device_ping_interval_ms: 1000, // Interval between discovery pings
    };
    let mut manager = DeviceManager::new(config);
    let mut _parser = CrsfParser::new();

    // In a real system, this would be your system uptime in milliseconds.
    let mut time_ms = 0u32;
    manager.update_time(time_ms);

    println!("Step 1: Sending device ping...");
    // The "Handset" (address 0xEA) is the typical source for discovery pings
    // in a radio controller setup. This triggers "Device Information" responses.
    if let Some(ping_packet) = manager.send_device_ping() {
        println!("  -> Ping packet created ({} bytes)", ping_packet.len());
        println!("     (In a real app, this would be sent over UART)\n");
    }

    // Simulate receiving a DeviceInformation response.
    // This is how a device like an ExpressLRS Transmitter module announces itself.
    println!("Step 2: Simulating device discovery response...");
    let device_info = DeviceInformation::new(
        PacketAddress::Handset as u8,     // Destination
        PacketAddress::Transmitter as u8, // Source
        "ExpressLRS TX",                  // Human readable name
        0x454C5253,                       // "ELRS" identifier
        0x12345678,                       // Serial Number
        0x00030102,                       // Firmware Version (v3.1.2)
        18,                               // Total number of parameters available
        1,                                // Parameter protocol version
    )
    .unwrap();

    // When a packet is parsed from the UART, pass it to the manager.
    let device_info_packet = Packet::DeviceInformation(device_info.clone());
    manager.handle_packet(&device_info_packet);

    println!("  -> Device discovered:");
    println!("     Name: {}", device_info.device_name());
    println!("     Serial: 0x{:08X}", device_info.serial_number);
    println!("     Firmware: 0x{:08X}", device_info.firmware_id);
    println!("     Parameters: {}\n", device_info.parameters_total);

    // The manager tracks multiple devices on the bus (e.g., TX module, RX, VTX).
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

    // Start fetching the parameter menu.
    // CRSF parameters are fetched one-by-one or in sequence.
    println!("Step 4: Requesting all parameters from device...");
    if let Some(param_request) = manager.request_all_parameters(PacketAddress::Transmitter) {
        println!(
            "  -> Parameter read request created ({} bytes)",
            param_request.len()
        );
        println!("     (In a real app, this would be sent over UART)\n");
    }

    // Simulating the asynchronous responses from the device.
    println!("Step 5: Simulating parameter responses...");

    // Parameter 0: The "ROOT" folder.
    // In ELRS, the UI is hierarchical. The ROOT folder contains IDs of top-level items.
    let root_folder = create_root_folder_parameter();
    let root_packet = Packet::ParameterSettingsEntry(root_folder);
    manager.handle_packet(&root_packet);
    println!("  -> Received parameter 0 (ROOT folder)");

    // Parameter 1: Packet Rate.
    // TextSelection is the most common type, representing a dropdown menu.
    let packet_rate = create_packet_rate_parameter();
    let rate_packet = Packet::ParameterSettingsEntry(packet_rate);
    manager.handle_packet(&rate_packet);
    println!("  -> Received parameter 1 (Packet Rate)");

    // Parameter 2: TX Power.
    // Floats in CRSF use integer representation with a decimal_point scaler.
    let tx_power = create_tx_power_parameter();
    let power_packet = Packet::ParameterSettingsEntry(tx_power);
    manager.handle_packet(&power_packet);
    println!("  -> Received parameter 2 (TX Power)");

    // Parameter 3: Device Name.
    let device_name_param = create_device_name_parameter();
    let name_packet = Packet::ParameterSettingsEntry(device_name_param);
    manager.handle_packet(&name_packet);
    println!("  -> Received parameter 3 (Device Name)");

    // Parameter 4: Bind command.
    // Commands are special parameters that trigger actions (like binding)
    // when written to.
    let bind_cmd = create_bind_command_parameter();
    let bind_packet = Packet::ParameterSettingsEntry(bind_cmd);
    manager.handle_packet(&bind_packet);
    println!("  -> Received parameter 4 (Bind Command)\n");

    // Inspect the current state of the discovered device.
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

    // Changing a parameter value.
    println!("Step 7: Writing a parameter value...");
    // When you want to change a setting, the manager generates the appropriate
    // CRSF "Parameter Write" packet.
    let write_data = [1u8]; // Index 1 in the TextSelection options
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

    // Demonstrate the retry logic.
    // If a response isn't received within `timeout_ms`, the manager will
    // re-queue the request when `process_timeouts` is called.
    println!("Step 8: Demonstrating timeout and retry...");
    time_ms += 600; // Jump ahead past the 500ms timeout
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
    use heapless::Vec;

    let mut children = Vec::<u8, 32>::new();
    children.push(1).unwrap();
    children.push(2).unwrap();
    children.push(3).unwrap();
    children.push(4).unwrap();

    ParameterSettingsEntry::new(
        PacketAddress::Handset as u8,     // Destination
        PacketAddress::Transmitter as u8, // Source
        0,                                // Parameter ID
        0,                                // Chunks remaining
        0,                                // Parent folder (0 = ROOT)
        ParameterDataType::Folder as u8,
        "ROOT",
    )
    .unwrap()
    .add_data(ParameterData::Folder { children })
}

fn create_packet_rate_parameter() -> ParameterSettingsEntry {
    use heapless::String;

    let options =
        String::try_from("50Hz(-117dBm);150Hz(-112dBm);250Hz(-108dBm);500Hz(-105dBm)").unwrap();
    let unit = String::try_from("Hz").unwrap();

    ParameterSettingsEntry::new(
        PacketAddress::Handset as u8,
        PacketAddress::Transmitter as u8,
        1,
        0,
        0, // Parent folder is ROOT
        ParameterDataType::TextSelection as u8,
        "Packet Rate",
    )
    .unwrap()
    .add_data(ParameterData::TextSelection {
        options,
        value: 2,
        min: 0,
        max: 3,
        default: 2,
        unit,
    })
}

fn create_tx_power_parameter() -> ParameterSettingsEntry {
    use heapless::String;

    let unit = String::try_from("mW").unwrap();

    ParameterSettingsEntry::new(
        PacketAddress::Handset as u8,
        PacketAddress::Transmitter as u8,
        2,
        0,
        0,
        ParameterDataType::Float as u8,
        "TX Power",
    )
    .unwrap()
    .add_data(ParameterData::Float {
        value: 100,
        min: 10,
        max: 1000,
        default: 100,
        decimal_point: 0,
        step_size: 10,
        unit,
    })
}

fn create_device_name_parameter() -> ParameterSettingsEntry {
    use heapless::String;

    let value = String::try_from("My ELRS TX").unwrap();

    ParameterSettingsEntry::new(
        PacketAddress::Handset as u8,
        PacketAddress::Transmitter as u8,
        3,
        0,
        0,
        ParameterDataType::String as u8,
        "Device Name",
    )
    .unwrap()
    .add_data(ParameterData::String {
        value,
        max_length: 16,
    })
}

fn create_bind_command_parameter() -> ParameterSettingsEntry {
    use heapless::String;

    let info = String::try_from("Press to enter bind mode").unwrap();

    ParameterSettingsEntry::new(
        PacketAddress::Handset as u8,
        PacketAddress::Transmitter as u8,
        4,
        0,
        0,
        ParameterDataType::Command as u8,
        "Bind",
    )
    .unwrap()
    .add_data(ParameterData::Command {
        status: 0,    // Idle
        timeout: 100, // 10 seconds
        info,
    })
}
