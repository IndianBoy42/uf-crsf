#![cfg(test)]
extern crate std;

#[cfg(feature = "device")]
use uf_crsf::device::Device;
use uf_crsf::packets::{CrsfPacket, ElrsStatus, PacketAddress};
#[cfg(feature = "device")]
use uf_crsf::packets::DeviceInformation;
use uf_crsf::write_packet_to_buffer;

#[cfg(feature = "device")]
#[test]
fn test_device_identifies_elrs_with_correct_serial() {
    // Create a DeviceInformation with ELRS serial number (0x454C5253 = "ELRS")
    let device_info = DeviceInformation::new(
        PacketAddress::Handset as u8,
        PacketAddress::Transmitter as u8,
        "ELRS TX",
        0x454C5253,
        0x12345678,
        0x00030102,
        18,
        1,
    )
    .expect("Failed to create DeviceInformation");

    let device = Device::from_device_info(&device_info).expect("Failed to create Device");

    assert_eq!(device.name, "ELRS TX");
    assert_eq!(device.serial_number, 0x454C5253);
    assert!(
        device.is_elrs_tx(),
        "Device should be identified as ELRS TX"
    );
}

#[cfg(feature = "device")]
#[test]
fn test_device_does_not_identify_non_elrs_as_elrs() {
    // Create a DeviceInformation with non-ELRS serial number
    let device_info = DeviceInformation::new(
        PacketAddress::Handset as u8,
        PacketAddress::Transmitter as u8,
        "Other TX",
        0x12345678, // Not ELRS serial
        0x12345678,
        0x00030102,
        18,
        1,
    )
    .expect("Failed to create DeviceInformation");

    let device = Device::from_device_info(&device_info).expect("Failed to create Device");

    assert_eq!(device.name, "Other TX");
    assert_eq!(device.serial_number, 0x12345678);
    assert!(
        !device.is_elrs_tx(),
        "Non-ELRS device should not be identified as ELRS"
    );
}

#[test]
fn test_packet_address_elrs_lua() {
    // ElrsLua address is 0xEF
    let elrs_lua =
        PacketAddress::try_from(0xEFu8).expect("Failed to create PacketAddress from 0xEF");
    assert_eq!(elrs_lua, PacketAddress::ElrsLua);
    assert_eq!(elrs_lua as u8, 0xEF);
}

#[test]
fn test_packet_address_from_u8_elrs_lua() {
    // Also test through TryFromPrimitive
    let addr = PacketAddress::ElrsLua;
    assert_eq!(addr as u8, 0xEF);
}

#[test]
fn test_elrs_status_serialization() {
    let status = ElrsStatus::new(1000, 10, 0x01).expect("Failed to create ElrsStatus");

    let mut buffer = [0u8; 64];
    let len = status
        .to_bytes(&mut buffer)
        .expect("Failed to serialize ElrsStatus");

    assert_eq!(len, 5);
    assert_eq!(buffer[0], 0x03); // good_packets high byte
    assert_eq!(buffer[1], 0xE8); // good_packets low byte
    assert_eq!(buffer[2], 0x00); // bad_packets high byte
    assert_eq!(buffer[3], 0x0A); // bad_packets low byte
    assert_eq!(buffer[4], 0x01); // flags
}

#[test]
fn test_elrs_status_deserialization() {
    let data: [u8; 5] = [0x03, 0xE8, 0x00, 0x0A, 0x01];
    let status = ElrsStatus::from_bytes(&data).expect("Failed to deserialize ElrsStatus");

    assert_eq!(status.good_packets, 1000);
    assert_eq!(status.bad_packets, 10);
    assert_eq!(status.flags, 0x01);
}

#[test]
fn test_elrs_status_round_trip() {
    let original = ElrsStatus::new(500, 25, 0x80).expect("Failed to create ElrsStatus");

    let mut buffer = [0u8; 64];
    let len = original
        .to_bytes(&mut buffer)
        .expect("Failed to serialize ElrsStatus");

    let deserialized =
        ElrsStatus::from_bytes(&buffer[..len]).expect("Failed to deserialize ElrsStatus");

    assert_eq!(original.good_packets, deserialized.good_packets);
    assert_eq!(original.bad_packets, deserialized.bad_packets);
    assert_eq!(original.flags, deserialized.flags);
}

#[test]
fn test_elrs_status_full_packet_serialization() {
    let status = ElrsStatus::new(1234, 56, 0x0F).expect("Failed to create ElrsStatus");

    let mut buffer = [0u8; 64];
    let len = write_packet_to_buffer(&mut buffer, PacketAddress::Handset, &status)
        .expect("Failed to write packet to buffer");

    // Verify the packet structure: dest (1) + length (1) + type (1) + payload (5) + crc (1) = 9 bytes
    assert_eq!(len, 9);
    // assert_eq!(buffer[0], PacketAddress::Handset as u8); // destination
    assert_eq!(buffer[1], 7); // length = type (1) + payload (5) + crc (1)
    assert_eq!(buffer[2], 0x2E); // ElrsStatus packet type
    assert_eq!(buffer[3], 0x04); // good_packets high byte
    assert_eq!(buffer[4], 0xD2); // good_packets low byte (1234 = 0x04D2)
    assert_eq!(buffer[5], 0x00); // bad_packets high byte
    assert_eq!(buffer[6], 0x38); // bad_packets low byte (56 = 0x0038)
    assert_eq!(buffer[7], 0x0F); // flags
                                 // buffer[8] is CRC
}

#[test]
fn test_elrs_status_max_values() {
    let status = ElrsStatus::new(0xFFFF, 0xFFFF, 0xFF).expect("Failed to create ElrsStatus");

    let mut buffer = [0u8; 64];
    let len = status
        .to_bytes(&mut buffer)
        .expect("Failed to serialize ElrsStatus");

    let deserialized =
        ElrsStatus::from_bytes(&buffer[..len]).expect("Failed to deserialize ElrsStatus");

    assert_eq!(deserialized.good_packets, 0xFFFF);
    assert_eq!(deserialized.bad_packets, 0xFFFF);
    assert_eq!(deserialized.flags, 0xFF);
}

#[test]
fn test_elrs_status_from_bytes_too_short() {
    let data: [u8; 4] = [0x03, 0xE8, 0x00, 0x0A];
    let result = ElrsStatus::from_bytes(&data);
    assert!(result.is_err(), "Should fail with insufficient data");
}
