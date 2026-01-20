use crate::packets::{CrsfPacket, PacketType};
use crate::CrsfParsingError;
use core::mem::size_of;

/// Represents an ELRS Status packet (0x2E).
///
/// Used for link statistics, providing good and bad packet counts along with status flags.
/// This is sent from Device to Handset in response to link statistics requests.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ElrsStatus {
    /// Good packet count.
    pub good_packets: u16,
    /// Bad packet count.
    pub bad_packets: u16,
    /// Status flags.
    pub flags: u8,
}

impl ElrsStatus {
    /// Creates a new ElrsStatus packet.
    pub fn new(good_packets: u16, bad_packets: u16, flags: u8) -> Result<Self, CrsfParsingError> {
        Ok(Self {
            good_packets,
            bad_packets,
            flags,
        })
    }
}

impl CrsfPacket for ElrsStatus {
    const PACKET_TYPE: PacketType = PacketType::ElrsStatus;
    const MIN_PAYLOAD_SIZE: usize = 2 * size_of::<u16>() + size_of::<u8>();

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, CrsfParsingError> {
        self.validate_buffer_size(buffer)?;
        buffer[0..2].copy_from_slice(&self.good_packets.to_be_bytes());
        buffer[2..4].copy_from_slice(&self.bad_packets.to_be_bytes());
        buffer[4] = self.flags;
        Ok(Self::MIN_PAYLOAD_SIZE)
    }

    fn from_bytes(data: &[u8]) -> Result<Self, CrsfParsingError> {
        if data.len() < Self::MIN_PAYLOAD_SIZE {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }
        let good_packets = u16::from_be_bytes(
            data[0..2]
                .try_into()
                .map_err(|_| CrsfParsingError::InvalidPayloadLength)?,
        );
        let bad_packets = u16::from_be_bytes(
            data[2..4]
                .try_into()
                .map_err(|_| CrsfParsingError::InvalidPayloadLength)?,
        );
        let flags = data[4];
        Ok(Self {
            good_packets,
            bad_packets,
            flags,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_elrs_status_to_bytes() {
        let status = ElrsStatus::new(1000, 10, 0x01).unwrap();
        let mut buffer = [0u8; 5];
        let len = status.to_bytes(&mut buffer).unwrap();
        assert_eq!(len, 5);
        assert_eq!(buffer, [0x03, 0xE8, 0x00, 0x0A, 0x01]);
    }

    #[test]
    fn test_elrs_status_from_bytes() {
        let data: [u8; 5] = [0x03, 0xE8, 0x00, 0x0A, 0x01];
        let status = ElrsStatus::from_bytes(&data).unwrap();
        assert_eq!(status.good_packets, 1000);
        assert_eq!(status.bad_packets, 10);
        assert_eq!(status.flags, 0x01);
    }

    #[test]
    fn test_elrs_status_round_trip() {
        let status = ElrsStatus::new(500, 25, 0x80).unwrap();
        let mut buffer = [0u8; 5];
        let len = status.to_bytes(&mut buffer).unwrap();
        let round_trip_status = ElrsStatus::from_bytes(&buffer[..len]).unwrap();
        assert_eq!(status, round_trip_status);
    }

    #[test]
    fn test_elrs_status_max_values() {
        let status = ElrsStatus::new(0xFFFF, 0xFFFF, 0xFF).unwrap();
        let mut buffer = [0u8; 5];
        status.to_bytes(&mut buffer).unwrap();
        let round_trip_status = ElrsStatus::from_bytes(&buffer).unwrap();
        assert_eq!(round_trip_status.good_packets, 0xFFFF);
        assert_eq!(round_trip_status.bad_packets, 0xFFFF);
        assert_eq!(round_trip_status.flags, 0xFF);
    }

    #[test]
    fn test_elrs_status_from_bytes_too_short() {
        let data: [u8; 4] = [0x03, 0xE8, 0x00, 0x0A];
        let result = ElrsStatus::from_bytes(&data);
        assert!(matches!(
            result,
            Err(CrsfParsingError::InvalidPayloadLength)
        ));
    }

    #[test]
    fn test_elrs_status_buffer_too_small() {
        let status = ElrsStatus::new(1000, 10, 0x01).unwrap();
        let mut buffer = [0u8; ElrsStatus::MIN_PAYLOAD_SIZE - 1];
        let result = status.to_bytes(&mut buffer);
        assert_eq!(result, Err(CrsfParsingError::BufferOverflow));
    }

    #[test]
    fn test_elrs_status_from_bytes_with_extra_payload() {
        // Should ignore extra payload
        let data: [u8; 8] = [0x03, 0xE8, 0x00, 0x0A, 0x01, 0xFF, 0xFF, 0xFF];
        let status = ElrsStatus::from_bytes(&data).unwrap();
        assert_eq!(status.good_packets, 1000);
        assert_eq!(status.bad_packets, 10);
        assert_eq!(status.flags, 0x01);
    }

    #[test]
    fn test_elrs_status_all_zeros() {
        let status = ElrsStatus::new(0, 0, 0).unwrap();
        let mut buffer = [0u8; 5];
        let len = status.to_bytes(&mut buffer).unwrap();
        let round_trip_status = ElrsStatus::from_bytes(&buffer[..len]).unwrap();
        assert_eq!(round_trip_status.good_packets, 0);
        assert_eq!(round_trip_status.bad_packets, 0);
        assert_eq!(round_trip_status.flags, 0);
    }
}
