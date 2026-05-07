use crate::packets::{CrsfPacket, PacketType};
use crate::CrsfParsingError;
use core::mem::size_of;

/// Request to read a specific parameter from a CRSF device.
///
/// This packet (type 0x2C) is sent to a device to request parameter metadata and value.
/// Devices respond with [ParameterSettingsEntry] packets containing the parameter's
/// type, constraints, and current value.
///
/// # Chunked Parameter Transfer
///
/// Parameters with large metadata (long names, many options) are sent across multiple
/// [ParameterSettingsEntry] packets. The `chunk_number` field identifies which chunk
/// is being requested:
/// - Chunk 0: Contains the primary parameter data and metadata
/// - Chunks 1+: Additional data (e.g., continuation of long TextSelection options)
///
/// The [ParameterSettingsEntry::chunks_remaining] field indicates how many more
/// chunks are available. The [DeviceManager] automatically handles chunk reassembly.
///
/// # Use Cases
///
/// **Handset Application:** Request all parameters sequentially starting from 0
/// to discover the device's full parameter tree. The DeviceManager handles this
/// automatically via [crate::device::DeviceManager::request_all_parameters()].
///
/// **Parameter Refresh:** If a parameter packet is corrupted or missed due to
/// link issues, re-send the ParameterRead with the appropriate chunk number to
/// recover it.
///
/// # CRSF Addressing
///
/// - `dst_addr`: The device being queried (e.g., [PacketAddress::Transmitter])
/// - `src_addr`: Your device's address (e.g., [PacketAddress::Handset])
///
/// # Example
///
/// ```no_run
/// # use uf_crsf::packets::ParameterRead;
/// # use uf_crsf::packets::CrsfPacket;
/// use uf_crsf::packets::PacketAddress;
///
/// // Request parameter 5 (TX Power) from the transmitter
/// let read = ParameterRead::new(
///     PacketAddress::Transmitter as u8,  // dst_addr
///     PacketAddress::Handset as u8,       // src_addr
///     5,   // parameter_number (TX Power)
///     0,   // chunk_number (first chunk)
/// ).unwrap();
///
/// let mut buffer = [0u8; 64];
/// let len = read.to_bytes(&mut buffer).unwrap();
/// // Send buffer[..len] over UART...
/// ```
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ParameterRead {
    /// Destination device address.
    ///
    /// The CRSF address of the device being queried. Common targets:
    /// - [PacketAddress::Transmitter] : ExpressLRS TX module
    /// - [PacketAddress::Receiver] : ExpressLRS receiver
    /// - [PacketAddress::FlightController] : Betaflight/ArduPilot
    pub dst_addr: u8,
    /// Origin device address.
    ///
    /// Your device's CRSF address. This identifies you as the requester.
    /// For handset applications, use [PacketAddress::Handset].
    pub src_addr: u8,
    /// The parameter ID to request.
    ///
    /// Parameters are numbered starting from 0 (the root folder). To discover
    /// all parameters, request them sequentially. The DeviceManager tracks
    /// which parameters have been loaded and automatically requests the next.
    pub parameter_number: u8,
    /// Chunk number for large parameters.
    ///
    /// Most parameters fit in a single packet (chunk 0). For parameters with
    /// large metadata (e.g., TextSelection with many options), the device
    /// sends multiple [ParameterSettingsEntry] packets. Set this to request a
    /// specific chunk if a previous packet was missed.
    pub chunk_number: u8,
}

impl ParameterRead {
    /// Creates a new ParameterRead packet.
    pub fn new(
        dst_addr: u8,
        src_addr: u8,
        parameter_number: u8,
        chunk_number: u8,
    ) -> Result<Self, CrsfParsingError> {
        Ok(Self {
            dst_addr,
            src_addr,
            parameter_number,
            chunk_number,
        })
    }
}

impl CrsfPacket for ParameterRead {
    const PACKET_TYPE: PacketType = PacketType::ParameterRead;
    const MIN_PAYLOAD_SIZE: usize = 4 * size_of::<u8>();

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, CrsfParsingError> {
        self.validate_buffer_size(buffer)?;
        buffer[0] = self.dst_addr;
        buffer[1] = self.src_addr;
        buffer[2] = self.parameter_number;
        buffer[3] = self.chunk_number;
        Ok(Self::MIN_PAYLOAD_SIZE)
    }

    fn from_bytes(data: &[u8]) -> Result<Self, CrsfParsingError> {
        if data.len() < Self::MIN_PAYLOAD_SIZE {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }
        Ok(Self {
            dst_addr: data[0],
            src_addr: data[1],
            parameter_number: data[2],
            chunk_number: data[3],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parameter_read_to_bytes() {
        let read = ParameterRead::new(0xEC, 0xEA, 4, 0).unwrap();
        let mut buffer = [0u8; 4];
        let len = read.to_bytes(&mut buffer).unwrap();
        assert_eq!(len, 4);
        assert_eq!(buffer, [0xEC, 0xEA, 4, 0]);
    }

    #[test]
    fn test_parameter_read_from_bytes() {
        let data: [u8; 4] = [0xEC, 0xEA, 4, 0];
        let read = ParameterRead::from_bytes(&data).unwrap();
        assert_eq!(read.dst_addr, 0xEC);
        assert_eq!(read.src_addr, 0xEA);
        assert_eq!(read.parameter_number, 4);
        assert_eq!(read.chunk_number, 0);
    }

    #[test]
    fn test_parameter_read_from_bytes_with_chunk() {
        let data: [u8; 4] = [0xEC, 0xEA, 10, 2];
        let read = ParameterRead::from_bytes(&data).unwrap();
        assert_eq!(read.parameter_number, 10);
        assert_eq!(read.chunk_number, 2);
    }

    #[test]
    fn test_parameter_read_round_trip() {
        let read = ParameterRead::new(0xEC, 0xEA, 15, 3).unwrap();
        let mut buffer = [0u8; 4];
        read.to_bytes(&mut buffer).unwrap();
        let round_trip_read = ParameterRead::from_bytes(&buffer).unwrap();
        assert_eq!(read, round_trip_read);
    }

    #[test]
    fn test_parameter_read_from_bytes_too_short() {
        let data: [u8; 3] = [0xEC, 0xEA, 4];
        let result = ParameterRead::from_bytes(&data);
        assert!(matches!(
            result,
            Err(CrsfParsingError::InvalidPayloadLength)
        ));
    }

    #[test]
    fn test_parameter_read_buffer_too_small() {
        let read = ParameterRead::new(0xEC, 0xEA, 4, 0).unwrap();
        let mut buffer = [0u8; ParameterRead::MIN_PAYLOAD_SIZE - 1];
        let result = read.to_bytes(&mut buffer);
        assert_eq!(result, Err(CrsfParsingError::BufferOverflow));
    }

    #[test]
    fn test_parameter_read_from_bytes_with_extra_payload() {
        // Should ignore extra payload
        let data: [u8; 6] = [0xEC, 0xEA, 4, 0, 1, 2];
        let read = ParameterRead::from_bytes(&data).unwrap();
        assert_eq!(read.parameter_number, 4);
        assert_eq!(read.chunk_number, 0);
    }
}
