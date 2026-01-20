use crate::packets::{CrsfPacket, PacketType};
use crate::CrsfParsingError;
use heapless::Vec;

/// ArduPilot Passthrough frame (0x80).
///
/// Used for ArduPilot specialized telemetry.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ArduPilotPassthrough {
    /// Single packet frame (0xF0).
    Single { appid: u16, data: u32 },
    /// Multi-packet frame (0xF2).
    Multi {
        packets: Vec<PassthroughTelemetryPacket, 9>,
    },
    /// Status text frame (0xF1).
    StatusText { severity: u8, text: Vec<u8, 50> },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PassthroughTelemetryPacket {
    pub appid: u16,
    pub data: u32,
}

impl ArduPilotPassthrough {
    pub const SUB_TYPE_SINGLE: u8 = 0xF0;
    pub const SUB_TYPE_STATUS: u8 = 0xF1;
    pub const SUB_TYPE_MULTI: u8 = 0xF2;
}

impl CrsfPacket for ArduPilotPassthrough {
    const PACKET_TYPE: PacketType = PacketType::ArdupilotResponse;
    const MIN_PAYLOAD_SIZE: usize = 1;

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, CrsfParsingError> {
        match self {
            ArduPilotPassthrough::Single { appid, data } => {
                if buffer.len() < 7 {
                    return Err(CrsfParsingError::BufferOverflow);
                }
                buffer[0] = Self::SUB_TYPE_SINGLE;
                buffer[1..3].copy_from_slice(&appid.to_be_bytes());
                buffer[3..7].copy_from_slice(&data.to_be_bytes());
                Ok(7)
            }
            ArduPilotPassthrough::StatusText { severity, text } => {
                let len = 2 + text.len();
                if buffer.len() < len {
                    return Err(CrsfParsingError::BufferOverflow);
                }
                buffer[0] = Self::SUB_TYPE_STATUS;
                buffer[1] = *severity;
                buffer[2..len].copy_from_slice(text);
                Ok(len)
            }
            ArduPilotPassthrough::Multi { packets } => {
                let len = 2 + packets.len() * 6;
                if buffer.len() < len {
                    return Err(CrsfParsingError::BufferOverflow);
                }
                buffer[0] = Self::SUB_TYPE_MULTI;
                buffer[1] = packets.len() as u8;
                for (i, p) in packets.iter().enumerate() {
                    let offset = 2 + i * 6;
                    buffer[offset..offset + 2].copy_from_slice(&p.appid.to_be_bytes());
                    buffer[offset + 2..offset + 6].copy_from_slice(&p.data.to_be_bytes());
                }
                Ok(len)
            }
        }
    }

    fn from_bytes(data: &[u8]) -> Result<Self, CrsfParsingError> {
        if data.is_empty() {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }
        let sub_type = data[0];
        match sub_type {
            Self::SUB_TYPE_SINGLE => {
                if data.len() < 7 {
                    return Err(CrsfParsingError::InvalidPayloadLength);
                }
                let appid = u16::from_be_bytes(data[1..3].try_into().unwrap());
                let data_val = u32::from_be_bytes(data[3..7].try_into().unwrap());
                Ok(ArduPilotPassthrough::Single {
                    appid,
                    data: data_val,
                })
            }
            Self::SUB_TYPE_STATUS => {
                if data.len() < 2 {
                    return Err(CrsfParsingError::InvalidPayloadLength);
                }
                let severity = data[1];
                let mut text = Vec::new();
                text.extend_from_slice(&data[2..])
                    .map_err(|_| CrsfParsingError::InvalidPayloadLength)?;
                Ok(ArduPilotPassthrough::StatusText { severity, text })
            }
            Self::SUB_TYPE_MULTI => {
                if data.len() < 2 {
                    return Err(CrsfParsingError::InvalidPayloadLength);
                }
                let count = data[1] as usize;
                if data.len() < 2 + count * 6 {
                    return Err(CrsfParsingError::InvalidPayloadLength);
                }
                let mut packets = Vec::new();
                for i in 0..count {
                    let offset = 2 + i * 6;
                    let appid = u16::from_be_bytes(data[offset..offset + 2].try_into().unwrap());
                    let data_val =
                        u32::from_be_bytes(data[offset + 2..offset + 6].try_into().unwrap());
                    packets
                        .push(PassthroughTelemetryPacket {
                            appid,
                            data: data_val,
                        })
                        .map_err(|_| CrsfParsingError::InvalidPayloadLength)?;
                }
                Ok(ArduPilotPassthrough::Multi { packets })
            }
            _ => Err(CrsfParsingError::InvalidPayload),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ardupilot_single_round_trip() {
        let packet = ArduPilotPassthrough::Single {
            appid: 0x1234,
            data: 0x567890AB,
        };
        let mut buffer = [0u8; 64];
        let len = packet.to_bytes(&mut buffer).unwrap();
        assert_eq!(len, 7);
        let round_trip = ArduPilotPassthrough::from_bytes(&buffer[..len]).unwrap();
        assert_eq!(packet, round_trip);
    }
}
