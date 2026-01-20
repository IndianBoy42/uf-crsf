use crate::packets::{CrsfPacket, PacketType};
use crate::CrsfParsingError;
use core::mem::size_of;

/// Represents a Parameter Read packet (0x2C).
///
/// Used to request a specific parameter from a device.
/// This command is for re-requesting a parameter/chunk that didn't make it through the link.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ParameterRead {
    /// Destination device address.
    pub dst_addr: u8,
    /// Origin device address.
    pub src_addr: u8,
    /// The parameter number to read.
    pub parameter_number: u8,
    /// The chunk number to request (starts with 0).
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
