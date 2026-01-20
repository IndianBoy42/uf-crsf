use crate::packets::{CrsfPacket, PacketType};
use crate::CrsfParsingError;
use heapless::Vec;

/// MSP (Multiwii Serial Protocol) frame over CRSF (0x7A/0x7B).
///
/// Used for forwarding MSP commands to the flight controller or other devices.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspPacket {
    /// Destination device address.
    pub dst_addr: u8,
    /// Origin device address.
    pub src_addr: u8,
    /// MSP Status byte.
    /// bits 0-3: cyclic sequence number
    /// bit 4: beginning of new frame (1 if true)
    /// bits 5-6: MSP version (1 or 2)
    /// bit 7: error (response only)
    pub status: u8,
    /// MSP payload body (max 57 bytes).
    pub body: Vec<u8, 57>,
}

impl MspPacket {
    /// Creates a new MspPacket.
    pub fn new(
        dst_addr: u8,
        src_addr: u8,
        status: u8,
        body: &[u8],
    ) -> Result<Self, CrsfParsingError> {
        if body.len() > 57 {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }
        let mut b = Vec::new();
        b.extend_from_slice(body)
            .map_err(|_| CrsfParsingError::InvalidPayloadLength)?;
        Ok(Self {
            dst_addr,
            src_addr,
            status,
            body: b,
        })
    }

    /// Returns the sequence number from status.
    pub fn sequence(&self) -> u8 {
        self.status & 0x0F
    }

    /// Returns true if this is the start of a new MSP frame.
    pub fn is_start(&self) -> bool {
        (self.status & 0x10) != 0
    }

    /// Returns the MSP version from status.
    pub fn version(&self) -> u8 {
        (self.status >> 5) & 0x03
    }

    /// Returns true if the error bit is set (for responses).
    pub fn is_error(&self) -> bool {
        (self.status & 0x80) != 0
    }
}

impl CrsfPacket for MspPacket {
    const PACKET_TYPE: PacketType = PacketType::MspRequest; // Default, can be Response too
                                                            // dst (1) + src (1) + status (1) + body (min 0) = 3 bytes
    const MIN_PAYLOAD_SIZE: usize = 3;

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, CrsfParsingError> {
        let total_size = 3 + self.body.len();
        if buffer.len() < total_size {
            return Err(CrsfParsingError::BufferOverflow);
        }
        buffer[0] = self.dst_addr;
        buffer[1] = self.src_addr;
        buffer[2] = self.status;
        buffer[3..total_size].copy_from_slice(&self.body);
        Ok(total_size)
    }

    fn from_bytes(data: &[u8]) -> Result<Self, CrsfParsingError> {
        if data.len() < Self::MIN_PAYLOAD_SIZE {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }
        let dst_addr = data[0];
        let src_addr = data[1];
        let status = data[2];
        let mut body = Vec::new();
        body.extend_from_slice(&data[3..])
            .map_err(|_| CrsfParsingError::InvalidPayloadLength)?;
        Ok(Self {
            dst_addr,
            src_addr,
            status,
            body,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_msp_packet_round_trip() {
        let body = [0x01, 0x02, 0x03];
        let packet = MspPacket::new(0xC8, 0xEA, 0x30, &body).unwrap(); // Start, version 1
        let mut buffer = [0u8; 64];
        let len = packet.to_bytes(&mut buffer).unwrap();
        assert_eq!(len, 6);
        let round_trip = MspPacket::from_bytes(&buffer[..len]).unwrap();
        assert_eq!(packet, round_trip);
        assert!(round_trip.is_start());
        assert_eq!(round_trip.version(), 1);
    }
}
