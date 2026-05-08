use crate::packets::remote::RemotePacket;
use crate::CrsfParsingError;
use core::mem::size_of;

/// Timing Correction frame (0x3A), also known as "CRSF Shot" or "RC-Sync".
///
/// Used to synchronize handset packet generation with radio module transmission.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TimingCorrection {
    /// Packet interval in 100ns units.
    pub update_interval: u32,
    /// Mixer sync offset in 100ns units.
    /// Positive values mean data came too early, negative mean late.
    pub offset: i32,
}

impl TimingCorrection {
    /// Creates a new TimingCorrection packet.
    pub fn new(update_interval: u32, offset: i32) -> Self {
        Self {
            update_interval,
            offset,
        }
    }
}

impl RemotePacket for TimingCorrection {
    /// Sub-type ID for Timing Correction.
    const SUB_TYPE: u8 = 0x10;
    const MIN_PAYLOAD_SIZE: usize = size_of::<u32>() + size_of::<i32>();

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, CrsfParsingError> {
        self.validate_buffer_size(buffer)?;
        buffer[0..4].copy_from_slice(&self.update_interval.to_be_bytes());
        buffer[4..8].copy_from_slice(&self.offset.to_be_bytes());
        Ok(Self::MIN_PAYLOAD_SIZE)
    }

    fn from_bytes(data: &[u8]) -> Result<Self, CrsfParsingError> {
        if data.len() < Self::MIN_PAYLOAD_SIZE {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }
        let update_interval = u32::from_be_bytes(
            data[0..4]
                .try_into()
                .map_err(|_e| CrsfParsingError::InvalidPayloadLength)?,
        );
        let offset = i32::from_be_bytes(
            data[4..8]
                .try_into()
                .map_err(|_e| CrsfParsingError::InvalidPayloadLength)?,
        );
        Ok(Self {
            update_interval,
            offset,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_timing_correction_round_trip() {
        let packet = TimingCorrection::new(50000, -1000);
        let mut buffer = [0u8; 9];
        let len = packet.to_bytes(&mut buffer).unwrap();
        assert_eq!(len, TimingCorrection::MIN_PAYLOAD_SIZE);
        let round_trip = TimingCorrection::from_bytes(&buffer[..len]).unwrap();
        assert_eq!(packet, round_trip);
    }
}
