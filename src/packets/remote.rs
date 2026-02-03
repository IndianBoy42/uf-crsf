use crate::packets::{CrsfPacket, PacketType};
use crate::CrsfParsingError;

mod timing_correction;
pub use timing_correction::TimingCorrection;

/// Represents a Remote-related packet (frame type 0x3A).
///
/// This is a container for various sub-packets related to remote functionality,
/// identified by a sub-type.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Remote {
    pub dst_addr: u8,
    pub src_addr: u8,
    pub payload: RemotePayload,
}

/// Enum for the different payloads of a Remote packet.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RemotePayload {
    TimingCorrection(TimingCorrection),
    // Future subtypes can be added here.
}

pub trait RemotePacket: Sized {
    /// The CRSF frame type identifier for this packet.
    const SUB_TYPE: u8;

    /// The minimum expected length of the packet's payload in bytes.
    /// For fixed-size packets, this is the same as the payload size.
    const MIN_PAYLOAD_SIZE: usize;

    /// Creates a packet instance from a payload byte slice.
    /// The slice is guaranteed to have a length of at least `MIN_PAYLOAD_SIZE`.
    fn from_bytes(data: &[u8]) -> Result<Self, CrsfParsingError>;
    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, CrsfParsingError>;

    fn validate_buffer_size(&self, buffer: &[u8]) -> Result<(), CrsfParsingError> {
        if buffer.len() < Self::MIN_PAYLOAD_SIZE {
            return Err(CrsfParsingError::BufferOverflow);
        }
        Ok(())
    }

    fn packet_len() -> usize {
        2 + 1 + Self::MIN_PAYLOAD_SIZE
    }
}

impl Remote {
    pub fn new(
        dst_addr: u8,
        src_addr: u8,
        payload: impl Into<RemotePayload>,
    ) -> Result<Self, CrsfParsingError> {
        Ok(Self {
            dst_addr,
            src_addr,
            payload: payload.into(),
        })
    }

    fn pack_remote<R: RemotePacket>(
        &self,
        buffer: &mut [u8],
        p: &R,
    ) -> Result<usize, CrsfParsingError> {
        if buffer.len() < R::packet_len() {
            return Err(CrsfParsingError::BufferOverflow);
        }
        buffer[0] = self.dst_addr;
        buffer[1] = self.src_addr;
        buffer[2] = R::SUB_TYPE;
        p.to_bytes(&mut buffer[3..]);
        Ok(R::packet_len())
    }
}

impl CrsfPacket for Remote {
    const PACKET_TYPE: PacketType = PacketType::RadioId;
    // Minimum payload for an extended header with a sub-type and its data.
    // For TimingCorrection: 1 (dst) + 1 (src) + 1 (sub-type) + 8 (data) = 11 bytes
    const MIN_PAYLOAD_SIZE: usize = 2 + 1;

    fn from_bytes(data: &[u8]) -> Result<Self, CrsfParsingError> {
        if data.len() < 3 {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }

        let dst_addr = data[0];
        let src_addr = data[1];
        let sub_type = data[2];

        let payload = match sub_type {
            TimingCorrection::SUB_TYPE => {
                if data.len() < (TimingCorrection::MIN_PAYLOAD_SIZE + 3) {
                    return Err(CrsfParsingError::InvalidPayloadLength);
                }
                let sub_payload = &data[3..];

                let timing_correction = TimingCorrection::from_bytes(sub_payload)?;
                RemotePayload::TimingCorrection(timing_correction)
            }
            _ => return Err(CrsfParsingError::InvalidPayload), // Unknown sub-type
        };

        Ok(Self {
            dst_addr,
            src_addr,
            payload,
        })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, CrsfParsingError> {
        match &self.payload {
            RemotePayload::TimingCorrection(p) => self.pack_remote(buffer, p),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_remote_new() {
        let packet = Remote::new(
            0xEA,
            0xEE,
            RemotePayload::TimingCorrection(TimingCorrection {
                update_interval: 50000,
                offset: -7,
            }),
        )
        .unwrap();
        assert_eq!(packet.dst_addr, 0xEA);
        assert_eq!(packet.src_addr, 0xEE);
        match packet.payload {
            RemotePayload::TimingCorrection(tc) => {
                assert_eq!(tc.update_interval, 50000);
                assert_eq!(tc.offset, -7);
            }
        }
    }

    #[test]
    fn test_timing_correction_from_bytes() {
        // Full payload for a 0x3A packet
        let data: [u8; 11] = [
            0xEA, // dst_addr
            0xEE, // src_addr
            TimingCorrection::SUB_TYPE,
            0x00,
            0x00,
            0xC3,
            0x50, // update_interval = 50000
            0xFF,
            0xFF,
            0xFF,
            0xF9, // offset = -7
        ];
        let packet = Remote::from_bytes(&data).unwrap();
        assert_eq!(packet.dst_addr, 0xEA);
        assert_eq!(packet.src_addr, 0xEE);
        match packet.payload {
            RemotePayload::TimingCorrection(tc) => {
                assert_eq!(tc.update_interval, 50000);
                assert_eq!(tc.offset, -7);
            }
        }
    }

    #[test]
    fn test_timing_correction_to_bytes() {
        let packet = Remote {
            dst_addr: 0xEA,
            src_addr: 0xEE,
            payload: RemotePayload::TimingCorrection(TimingCorrection {
                update_interval: 50000,
                offset: -7,
            }),
        };
        let mut buffer = [0u8; 11];
        let len = packet.to_bytes(&mut buffer).unwrap();
        assert_eq!(len, 11);
        let expected: [u8; 11] = [
            0xEA,
            0xEE,
            TimingCorrection::SUB_TYPE,
            0x00,
            0x00,
            0xC3,
            0x50,
            0xFF,
            0xFF,
            0xFF,
            0xF9,
        ];
        assert_eq!(buffer, expected);
    }

    #[test]
    fn test_remote_round_trip() {
        let packet = Remote {
            dst_addr: 0xC8,
            src_addr: 0xEC,
            payload: RemotePayload::TimingCorrection(TimingCorrection {
                update_interval: 12345,
                offset: -6789,
            }),
        };
        let mut buffer = [0u8; 11];
        packet.to_bytes(&mut buffer).unwrap();
        let round_trip = Remote::from_bytes(&buffer).unwrap();
        assert_eq!(packet, round_trip);
    }

    #[test]
    fn test_from_bytes_invalid_len() {
        let data: [u8; 2] = [0; 2];
        let result = Remote::from_bytes(&data);
        assert!(matches!(
            result,
            Err(CrsfParsingError::InvalidPayloadLength)
        ));
    }

    #[test]
    fn test_from_bytes_unknown_subtype() {
        let data: [u8; 11] = [0xEA, 0xEE, 0x11, 0, 0, 0, 0, 0, 0, 0, 0];
        let result = Remote::from_bytes(&data);
        assert!(matches!(result, Err(CrsfParsingError::InvalidPayload)));
    }
}
