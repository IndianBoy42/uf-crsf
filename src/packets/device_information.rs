use crate::packets::{CrsfPacket, PacketType};
use crate::CrsfParsingError;
use core::mem::size_of;
use heapless::String;

const MAX_DEVICE_NAME_LEN: usize = 43;
const EXTENDED_HEADER_SIZE: usize = 2 * size_of::<u8>();
const FIXED_FIELDS_SIZE: usize = 3 * size_of::<u32>() + 2 * size_of::<u8>();

/// Response packet identifying a CRSF device and its parameters.
///
/// This packet (type 0x29) is sent in response to a [DevicePing] discovery request.
/// It contains device identification, hardware/firmware versions, and metadata about
/// the exposed parameters.
///
/// # Response Flow
///
/// After receiving a [DevicePing], each device sends [DeviceInformation] back to
/// the requester:
///
/// ```text
/// Handset → Broadcast: DevicePing
/// TX Module → Handset: DeviceInformation (serial: 0x454C5253, params: 25, version: 1)
/// RX → Handset: DeviceInformation (serial: 0x..., params: 8, version: 1)
/// ```
///
/// # Parameter Enumeration
///
/// The `parameters_total` field indicates how many parameter IDs exist on this device.
/// After receiving this packet, begin enumeration by requesting parameter 0 via
/// [ParameterRead]. Parameter 0 is typically a root folder whose value field
/// contains the top-level child parameter IDs.
///
/// # Parameter Versioning
///
/// The `parameter_version_number` tracks the parameter schema version. If this changes
/// from a previous connection, the parameter structure (IDs, types, defaults) may have
/// been updated. Applications should:
/// - Clear cached parameters when version changes
/// - Re-enumerate all parameters
/// - Update UI to reflect new structure
///
/// # ExpressLRS Devices
///
/// | Device Type | Serial Number | Typical Name | Param Count |
/// |-------------|---------------|-------------|-------------|
/// | TX Module | 0x454C5253 ("ELRS") | "ELRS TX" | 20-50 |
/// | Receiver | 0x454C5253 ("ELRS") | "ELRS RX" | 5-15 |
/// | FC (Betaflight) | Varies | "Betaflight" | 10+ |
///
/// # Example Usage
///
/// ```no_run
/// # use uf_crsf::packets::device_information::DeviceInformation;
/// # use uf_crsf::parser::CrsfParser;
/// let mut parser = CrsfParser::new();
///
/// // Process incoming packets
/// let bytes = uart_read();
/// for packet_result in parser.iter_packets(&bytes) {
///     if let Ok(packet) = packet_result {
///         if let Packet::DeviceInformation(info) = packet {
///             println!("Found device: {}", info.device_name());
///             println!("  Serial: 0x{:08X}", info.serial_number);
///             println!("  Parameters: {}", info.parameters_total);
///             println!("  Firmware: 0x{:08X}", info.firmware_id);
///
///             // Start parameter enumeration
///             let request = ParameterRead::new(
///                 info.src_addr,  // Respond to this device
///                 my_address,
///                 0,              // Start with parameter 0 (root)
///                 0,              // First chunk
///             )?;
///             uart_write(&request);
///         }
///     }
/// }
/// ```
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeviceInformation {
    /// Destination address for this response packet.
    ///
    /// This is the address of the device that sent the [DevicePing].
    /// Responses are directed back to the requester, not broadcast.
    pub dst_addr: u8,
    /// Source address (the responding device's address).
    ///
    /// Use this address when sending subsequent [ParameterRead] or
    /// [ParameterWrite] packets to this device.
    pub src_addr: u8,
    /// Human-readable device name (up to 42 characters).
    ///
    /// Display this to users to identify which device they're configuring.
    /// Common values: "ELRS TX", "Betaflight", "ArduPilot", "TBS Tracer".
    device_name: String<MAX_DEVICE_NAME_LEN>,
    /// Unique serial number identifying this device.
    ///
    /// ExpressLRS devices typically use 0x454C5253 ("ELRS") for their serial.
    /// Other devices may use device-specific serials or board identifiers.
    /// This field helps correlate devices across sessions.
    pub serial_number: u32,
    /// Hardware identifier (vendor-specific encoding).
    ///
    /// Encodes board type or hardware revision. The exact format is
    /// vendor-dependent:
    /// - **ExpressLRS**: Often encodes radio type (ESP8266/ESP32/SX12xx)
    /// - **Betaflight**: May encode MCU type
    /// - **TBS**: Board model identifier
    ///
    /// Consult vendor documentation for interpretation.
    pub hardware_id: u32,
    /// Firmware identifier encoding version and variant.
    ///
    /// Format varies by vendor. For ExpressLRS, high bytes often indicate
    /// major.minor version. This field can help determine feature support.
    pub firmware_id: u32,
    /// Total number of parameters exposed by this device.
    ///
    /// Parameters are numbered from 0 to `parameters_total - 1`. Use this
    /// value to determine when parameter enumeration is complete
    /// (when you've received all parameters up to this count).
    pub parameters_total: u8,
    /// Parameter schema version number.
    ///
    /// Tracks the structure of the parameter tree. If this changes,
    /// the IDs, types, or organization of parameters may have changed.
    /// Applications should clear caches and re-enumerate when this changes.
    pub parameter_version_number: u8,
}

impl DeviceInformation {
    /// Creates a new DeviceInformation packet.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        dst_addr: u8,
        src_addr: u8,
        device_name: &str,
        serial_number: u32,
        hardware_id: u32,
        firmware_id: u32,
        parameters_total: u8,
        parameter_version_number: u8,
    ) -> Result<Self, CrsfParsingError> {
        if device_name.len() > MAX_DEVICE_NAME_LEN {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }
        let mut s = String::new();
        s.push_str(device_name)
            .map_err(|_| CrsfParsingError::InvalidPayloadLength)?;
        Ok(Self {
            dst_addr,
            src_addr,
            device_name: s,
            serial_number,
            hardware_id,
            firmware_id,
            parameters_total,
            parameter_version_number,
        })
    }

    /// Returns the device name as a string slice.
    pub fn device_name(&self) -> &str {
        self.device_name.as_str()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for DeviceInformation {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "DeviceInformation {{ dst_addr: {=u8}, src_addrs: {=u8}, device_name: {}, serial_number: {=u32}, hardware_id: {=u32}, firmware_id: {=u32}, parameters_total: {=u8}, parameter_version_number: {=u8} }}",
            self.dst_addr,
            self.src_addr,
            self.device_name(),
            self.serial_number,
            self.hardware_id,
            self.firmware_id,
            self.parameters_total,
            self.parameter_version_number,
        )
    }
}

impl CrsfPacket for DeviceInformation {
    const PACKET_TYPE: PacketType = PacketType::DeviceInfo;
    // Minimum payload is dst, src, a null terminator for the string + 14 bytes of other data
    const MIN_PAYLOAD_SIZE: usize = EXTENDED_HEADER_SIZE + 1 + FIXED_FIELDS_SIZE;

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, CrsfParsingError> {
        let name_bytes = self.device_name().as_bytes();
        let name_len = name_bytes.len();
        let payload_len = EXTENDED_HEADER_SIZE + name_len + 1 + FIXED_FIELDS_SIZE;

        if buffer.len() < payload_len {
            return Err(CrsfParsingError::BufferOverflow);
        }

        buffer[0] = self.dst_addr;
        buffer[1] = self.src_addr;

        let mut offset = EXTENDED_HEADER_SIZE;
        buffer[offset..offset + name_len].copy_from_slice(name_bytes);
        offset += name_len;
        buffer[offset] = 0; // Null terminator
        offset += 1;

        buffer[offset..offset + 4].copy_from_slice(&self.serial_number.to_be_bytes());
        offset += 4;
        buffer[offset..offset + 4].copy_from_slice(&self.hardware_id.to_be_bytes());
        offset += 4;
        buffer[offset..offset + 4].copy_from_slice(&self.firmware_id.to_be_bytes());
        offset += 4;
        buffer[offset] = self.parameters_total;
        offset += 1;
        buffer[offset] = self.parameter_version_number;

        Ok(payload_len)
    }

    fn from_bytes(data: &[u8]) -> Result<Self, CrsfParsingError> {
        if data.len() < Self::MIN_PAYLOAD_SIZE {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }

        let dst_addr = data[0];
        let src_addr = data[1];

        let payload = &data[EXTENDED_HEADER_SIZE..];
        let null_pos = payload
            .iter()
            .position(|&b| b == 0)
            .ok_or(CrsfParsingError::InvalidPayload)?;
        let s = core::str::from_utf8(&payload[..null_pos])
            .map_err(|_| CrsfParsingError::InvalidPayload)?;
        let mut device_name = String::new();
        device_name
            .push_str(s)
            .map_err(|_e| CrsfParsingError::InvalidPayloadLength)?;

        let mut offset = null_pos + 1;
        if payload.len() < offset + FIXED_FIELDS_SIZE {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }

        let serial_number = u32::from_be_bytes(
            payload[offset..offset + 4]
                .try_into()
                .map_err(|_e| CrsfParsingError::InvalidPayloadLength)?,
        );
        offset += 4;
        let hardware_id = u32::from_be_bytes(
            payload[offset..offset + 4]
                .try_into()
                .map_err(|_e| CrsfParsingError::InvalidPayloadLength)?,
        );
        offset += 4;
        let firmware_id = u32::from_be_bytes(
            payload[offset..offset + 4]
                .try_into()
                .map_err(|_e| CrsfParsingError::InvalidPayloadLength)?,
        );
        offset += 4;
        let parameters_total = payload[offset];
        offset += 1;
        let parameter_version_number = payload[offset];

        Ok(Self {
            dst_addr,
            src_addr,
            device_name,
            serial_number,
            hardware_id,
            firmware_id,
            parameters_total,
            parameter_version_number,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_device_information_to_bytes() {
        let info = DeviceInformation::new(
            0xEA,
            0xEE,
            "TBS Tracer",
            0x12345678,
            0xABCDEF01,
            0x98765432,
            42,
            5,
        )
        .unwrap();

        let mut buffer = [0u8; 60];
        let len = info.to_bytes(&mut buffer).unwrap();

        let expected_name = b"TBS Tracer\0";
        let expected_len = EXTENDED_HEADER_SIZE + expected_name.len() + FIXED_FIELDS_SIZE;
        assert_eq!(len, expected_len);

        assert_eq!(buffer[0], 0xEA);
        assert_eq!(buffer[1], 0xEE);
        assert_eq!(
            &buffer[EXTENDED_HEADER_SIZE..EXTENDED_HEADER_SIZE + expected_name.len()],
            expected_name
        );
        let mut offset = EXTENDED_HEADER_SIZE + expected_name.len();
        assert_eq!(&buffer[offset..offset + 4], &0x12345678u32.to_be_bytes());
        offset += 4;
        assert_eq!(&buffer[offset..offset + 4], &0xABCDEF01u32.to_be_bytes());
        offset += 4;
        assert_eq!(&buffer[offset..offset + 4], &0x98765432u32.to_be_bytes());
        offset += 4;
        assert_eq!(buffer[offset], 42);
        offset += 1;
        assert_eq!(buffer[offset], 5);
    }

    #[test]
    fn test_device_information_from_bytes() {
        let data =
            b"\xEA\xEE\nTBS Tracer\0\x12\x34\x56\x78\xAB\xCD\xEF\x01\x98\x76\x54\x32\x2A\x05";
        let info = DeviceInformation::from_bytes(data).unwrap();

        assert_eq!(info.dst_addr, 0xEA);
        assert_eq!(info.src_addr, 0xEE);
        assert_eq!(info.device_name(), "\nTBS Tracer");
        assert_eq!(info.serial_number, 0x12345678);
        assert_eq!(info.hardware_id, 0xABCDEF01);
        assert_eq!(info.firmware_id, 0x98765432);
        assert_eq!(info.parameters_total, 42);
        assert_eq!(info.parameter_version_number, 5);
    }

    #[test]
    fn test_device_information_round_trip() {
        let info = DeviceInformation::new(0x12, 0x34, "MyDevice", 1, 2, 3, 4, 5).unwrap();

        let mut buffer = [0u8; 60];
        let len = info.to_bytes(&mut buffer).unwrap();
        let round_trip_info = DeviceInformation::from_bytes(&buffer[..len]).unwrap();

        assert_eq!(info, round_trip_info);
    }

    #[test]
    fn test_device_information_from_bytes_invalid_len_too_short() {
        let data = b"\xEA\xEEToo short\0";
        let result = DeviceInformation::from_bytes(data);
        assert!(matches!(
            result,
            Err(CrsfParsingError::InvalidPayloadLength)
        ));
    }

    #[test]
    fn test_device_information_from_bytes_invalid_len_no_room_for_fixed_fields() {
        let data = b"\xEA\xEEThis string is long enough but no room for fixed fields\0";
        let result = DeviceInformation::from_bytes(data);
        assert!(matches!(
            result,
            Err(CrsfParsingError::InvalidPayloadLength)
        ));
    }

    #[test]
    fn test_device_buffer_too_small() {
        let info = DeviceInformation::new(0x12, 0x34, "MyDevice", 1, 2, 3, 4, 5).unwrap();
        let mut buffer = [0u8; DeviceInformation::MIN_PAYLOAD_SIZE - 1];
        let result = info.to_bytes(&mut buffer);
        assert_eq!(result, Err(CrsfParsingError::BufferOverflow));
    }

    #[test]
    fn test_device_information_from_bytes_no_null() {
        let data = b"\xEA\xEE\nNo null terminator here and lots of other data that should be enough for the rest of the fields 12345678901234";
        let result = DeviceInformation::from_bytes(data);
        assert!(matches!(result, Err(CrsfParsingError::InvalidPayload)));
    }

    #[test]
    fn test_device_information_new_name_too_long() {
        let name = "x".repeat(44);
        let result = DeviceInformation::new(0x12, 0x34, &name, 1, 2, 3, 4, 5);
        assert_eq!(result, Err(CrsfParsingError::InvalidPayloadLength));
    }
}
