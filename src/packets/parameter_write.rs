use crate::packets::{CrsfPacket, PacketType};
use crate::CrsfParsingError;
use core::mem::size_of;
use heapless::Vec;

/// Maximum size of parameter write data in bytes.
///
/// The CRSF protocol limits parameter values to 32 bytes. This accommodates:
/// - Float: 4 bytes (little-endian f32)
/// - TextSelection: 1 byte (option index)
/// - String: Variable length (up to 31 bytes + null terminator)
/// - Command: 1 byte (triggers action)
const MAX_DATA_SIZE: usize = 32;

/// Writes a new value to a device parameter.
///
/// This packet (type 0x2D) changes a parameter's value on a CRSF device.
/// Common use cases include setting TX power, changing VTX channels, or
/// adjusting receiver PWM frequencies.
///
/// # Data Encoding by Parameter Type
///
/// The `data` bytes must be formatted according to the parameter's type:
///
/// | Type | Data Format | Size | Example |
/// |------|------------|------|---------|
/// | Float | Little-endian f32 | 4 bytes | 2000.0 mW → `[0xD0, 0x07, 0x00, 0x00]` |
/// | TextSelection | Option index (u8) | 1 byte | Select option 2 → `[2]` |
/// | String | UTF-8 string | Variable | "MyRadio" → `"MyRadio".as_bytes()` |
/// | Command | Any value (triggers) | Any | `[0]` |
/// | Folder/Info | Not writable | - | N/A |
///
/// # Write Confirmation
///
/// The CRSF spec doesn't define an explicit write confirmation packet.
/// However, the device typically updates its state and subsequent
/// [ParameterSettingsEntry] responses will reflect the new value.
/// Some implementations (e.g., ExpressLRS) send a new [ParameterSettingsEntry]
/// for the written parameter as implicit confirmation.
///
/// # CRSF Addressing
///
/// - `dst_addr`: Target device (e.g., [PacketAddress::Transmitter])
/// - `src_addr`: Your device's address (e.g., [PacketAddress::Handset])
///
/// # Example
///
/// ```no_run
/// # use uf_crsf::packets::ParameterWrite;
/// # use uf_crsf::packets::CrsfPacket;
/// use uf_crsf::packets::PacketAddress;
///
/// // Write TX Power parameter (ID 5, Float type) to 2000 mW
/// let power_value: f32 = 2000.0;
/// let mut data_bytes = [0u8; 4];
/// data_bytes.copy_from_slice(&power_value.to_le_bytes());
///
/// let write = ParameterWrite::new(
///     PacketAddress::Transmitter as u8,  // dst_addr
///     PacketAddress::Handset as u8,       // src_addr
///     5,   // parameter_number
///     &data_bytes,
/// ).unwrap();
///
/// let mut buffer = [0u8; 64];
/// let len = write.to_bytes(&mut buffer).unwrap();
/// // Send buffer[..len] over UART...
/// ```
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParameterWrite {
    /// Destination device address.
    ///
    /// The CRSF address of the device receiving the write. Common targets:
    /// - [PacketAddress::Transmitter]: ExpressLRS TX module (e.g., for power/mode settings)
    /// - [PacketAddress::Receiver]: ExpressLRS receiver (e.g., for output mode)
    /// - [PacketAddress::FlightController]: FC for VTX or flight mode settings
    pub dst_addr: u8,
    /// Origin device address.
    ///
    /// Your device's CRSF address. For handset applications, use
    /// [PacketAddress::Handset]. This identifies you as the source of
    /// the write request.
    pub src_addr: u8,
    /// The parameter ID to write.
    ///
    /// Must correspond to a valid parameter ID previously discovered via
    /// [ParameterSettingsEntry] enumeration. Writing to an invalid
    /// parameter ID typically results in no action or an error response.
    pub parameter_number: u8,
    /// The new parameter value bytes.
    ///
    /// Format depends on parameter type - see [ParameterWrite] documentation.
    /// Maximum size is 32 bytes. The DeviceManager doesn't encode values
    /// for you - you must construct the byte array based on the parameter's
    /// type as discovered via [ParameterSettingsEntry].
    pub data: Vec<u8, MAX_DATA_SIZE>,
}

impl ParameterWrite {
    /// Creates a new ParameterWrite packet.
    pub fn new(
        dst_addr: u8,
        src_addr: u8,
        parameter_number: u8,
        data: &[u8],
    ) -> Result<Self, CrsfParsingError> {
        if data.len() > MAX_DATA_SIZE {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }
        let mut vec = Vec::new();
        vec.extend_from_slice(data)
            .map_err(|_| CrsfParsingError::InvalidPayloadLength)?;
        Ok(Self {
            dst_addr,
            src_addr,
            parameter_number,
            data: vec,
        })
    }
}

impl CrsfPacket for ParameterWrite {
    const PACKET_TYPE: PacketType = PacketType::ParameterWrite;
    const MIN_PAYLOAD_SIZE: usize = 3 * size_of::<u8>() + 1; // dst + src + parameter_number + at least 1 data byte

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, CrsfParsingError> {
        let total_size = 3 * size_of::<u8>() + self.data.len();
        if buffer.len() < total_size {
            return Err(CrsfParsingError::BufferOverflow);
        }
        buffer[0] = self.dst_addr;
        buffer[1] = self.src_addr;
        buffer[2] = self.parameter_number;
        buffer[3..total_size].copy_from_slice(&self.data);
        Ok(total_size)
    }

    fn from_bytes(data: &[u8]) -> Result<Self, CrsfParsingError> {
        if data.len() < Self::MIN_PAYLOAD_SIZE {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }
        let dst_addr = data[0];
        let src_addr = data[1];
        let parameter_number = data[2];
        let data_bytes = &data[3..];
        if data_bytes.len() > MAX_DATA_SIZE {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }
        let mut vec = Vec::new();
        vec.extend_from_slice(data_bytes)
            .map_err(|_| CrsfParsingError::InvalidPayloadLength)?;
        Ok(Self {
            dst_addr,
            src_addr,
            parameter_number,
            data: vec,
        })
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for ParameterWrite {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "ParameterWrite {{ dst_addr: {=u8}, src_addr: {=u8}, parameter_number: {=u8}, data: [..{=usize} bytes] }}",
            self.dst_addr,
            self.src_addr,
            self.parameter_number,
            self.data.len(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parameter_write_to_bytes_single_byte() {
        let write = ParameterWrite::new(0xEC, 0xEA, 4, &[2]).unwrap();
        let mut buffer = [0u8; 10];
        let len = write.to_bytes(&mut buffer).unwrap();
        assert_eq!(len, 4);
        assert_eq!(buffer[..4], [0xEC, 0xEA, 4, 2]);
    }

    #[test]
    fn test_parameter_write_from_bytes_single_byte() {
        let data: [u8; 4] = [0xEC, 0xEA, 4, 2];
        let write = ParameterWrite::from_bytes(&data).unwrap();
        assert_eq!(write.dst_addr, 0xEC);
        assert_eq!(write.src_addr, 0xEA);
        assert_eq!(write.parameter_number, 4);
        assert_eq!(&*write.data, &[2]);
    }

    #[test]
    fn test_parameter_write_to_bytes_multiple_bytes() {
        let write = ParameterWrite::new(0xEC, 0xEA, 10, &[0x00, 0x12, 0x34, 0x56]).unwrap();
        let mut buffer = [0u8; 10];
        let len = write.to_bytes(&mut buffer).unwrap();
        assert_eq!(len, 7);
        assert_eq!(buffer[..7], [0xEC, 0xEA, 10, 0x00, 0x12, 0x34, 0x56]);
    }

    #[test]
    fn test_parameter_write_from_bytes_multiple_bytes() {
        let data: [u8; 7] = [0xEC, 0xEA, 10, 0x00, 0x12, 0x34, 0x56];
        let write = ParameterWrite::from_bytes(&data).unwrap();
        assert_eq!(write.parameter_number, 10);
        assert_eq!(&*write.data, &[0x00, 0x12, 0x34, 0x56]);
    }

    #[test]
    fn test_parameter_write_round_trip() {
        let write = ParameterWrite::new(0xEC, 0xEA, 15, &[1, 2, 3, 4, 5]).unwrap();
        let mut buffer = [0u8; 32];
        let len = write.to_bytes(&mut buffer).unwrap();
        let round_trip_write = ParameterWrite::from_bytes(&buffer[..len]).unwrap();
        assert_eq!(write, round_trip_write);
    }

    #[test]
    fn test_parameter_write_from_bytes_too_short() {
        let data: [u8; 2] = [0xEC, 0xEA];
        let result = ParameterWrite::from_bytes(&data);
        assert!(matches!(
            result,
            Err(CrsfParsingError::InvalidPayloadLength)
        ));
    }

    #[test]
    fn test_parameter_write_buffer_too_small() {
        let write = ParameterWrite::new(0xEC, 0xEA, 4, &[1, 2, 3]).unwrap();
        let mut buffer = [0u8; 5]; // Needs 6 bytes total
        let result = write.to_bytes(&mut buffer);
        assert_eq!(result, Err(CrsfParsingError::BufferOverflow));
    }

    #[test]
    fn test_parameter_write_new_data_too_large() {
        let large_data: [u8; 33] = [0; 33];
        let result = ParameterWrite::new(0xEC, 0xEA, 4, &large_data);
        assert!(matches!(
            result,
            Err(CrsfParsingError::InvalidPayloadLength)
        ));
    }

    #[test]
    fn test_parameter_write_from_bytes_data_too_large() {
        let mut large_data: [u8; 37] = [0; 37];
        large_data[0] = 0xEC;
        large_data[1] = 0xEA;
        large_data[2] = 4;
        let result = ParameterWrite::from_bytes(&large_data);
        assert!(matches!(
            result,
            Err(CrsfParsingError::InvalidPayloadLength)
        ));
    }
}
