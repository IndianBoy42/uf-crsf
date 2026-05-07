use crate::packets::{CrsfPacket, PacketType};
use crate::CrsfParsingError;

/// Broadcast discovery request for CRSF devices on the bus.
///
/// This packet (type 0x28) is used to discover CRSF-capable devices. When a device
/// receives a ping addressed to it, it responds with [DeviceInformation] containing
/// its identification and parameter metadata.
///
/// # Broadcast vs Directed Pings
///
/// The `dst_addr` field can be either:
/// - **Broadcast (0xFF)**: All devices on the bus respond (most common)
/// - **Specific address**: Only the targeted device responds
///
/// Handset applications typically use broadcast to discover all devices.
///
/// # Discovery Flow
///
/// 1. Send [DevicePing] with dst=0xFF, src=[PacketAddress::Handset]
/// 2. All devices respond with [DeviceInformation]
/// 3. Each response identifies the device (name, serial, param count)
/// 4. Begin parameter enumeration via [ParameterRead]
///
/// # Rate Limiting
///
/// Avoid flooding the bus with pings. In embedded systems, wait at least
/// 500ms-1000ms between broadcasts. The [DeviceManager] can handle this
/// automatically via [crate::device::DeviceManagerConfig::device_ping_interval_ms].
///
/// # ExpressLRS Behavior
///
/// ExpressLRS TX modules respond to pings with serial number 0x454C5253 ("ELRS").
/// Receivers may not respond to broadcast pings in all configurations.
///
/// # Example
///
/// ```no_run
/// # use uf_crsf::packets::DevicePing;
/// # use uf_crsf::packets::CrsfPacket;
/// use uf_crsf::packets::PacketAddress;
///
/// // Broadcast discovery ping to find all devices
/// let ping = DevicePing::new(
///     PacketAddress::Broadcast as u8,  // dst_addr: all devices
///     PacketAddress::Handset as u8,     // src_addr: this controller
/// ).unwrap();
///
/// let mut buffer = [0u8; 64];
/// let len = ping.to_bytes(&mut buffer).unwrap();
/// // Send buffer[..len] over UART...
///
/// // Expected responses: DeviceInformation from each device
/// ```
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DevicePing {
    /// Destination address for the ping.
    ///
    /// - [PacketAddress::Broadcast] (0xFF): All devices respond (recommended)
    /// - Specific address: Only that device responds
    ///
    /// For initial device discovery, always use broadcast to find all devices.
    pub dst_addr: u8,
    /// Origin/source address of the ping.
    ///
    /// Your device's CRSF address. Devices responding to this ping will
    /// send their [DeviceInformation] back to this address.
    ///
    /// For handset applications, use [PacketAddress::Handset].
    pub src_addr: u8,
}

impl DevicePing {
    pub fn new(dst_addr: u8, src_addr: u8) -> Result<Self, CrsfParsingError> {
        Ok(Self { dst_addr, src_addr })
    }
}

impl CrsfPacket for DevicePing {
    const PACKET_TYPE: PacketType = PacketType::DevicePing;
    const MIN_PAYLOAD_SIZE: usize = 2 * size_of::<u8>();

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, CrsfParsingError> {
        self.validate_buffer_size(buffer)?;
        buffer[0] = self.dst_addr;
        buffer[1] = self.src_addr;
        Ok(Self::MIN_PAYLOAD_SIZE)
    }

    fn from_bytes(data: &[u8]) -> Result<Self, CrsfParsingError> {
        Ok(Self {
            dst_addr: data[0],
            src_addr: data[1],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_device_ping_new() {
        let ping = DevicePing::new(0xEA, 0xEE).unwrap();
        assert_eq!(ping.dst_addr, 0xEA);
        assert_eq!(ping.src_addr, 0xEE);
    }

    #[test]
    fn test_parameter_ping_to_bytes() {
        let ping = DevicePing::new(0xEA, 0xEE).unwrap();
        let mut buffer = [0u8; 2];
        let len = ping.to_bytes(&mut buffer).unwrap();
        assert_eq!(len, 2);
        assert_eq!(buffer, [0xEA, 0xEE]);
    }

    #[test]
    fn test_parameter_ping_from_bytes() {
        let data: [u8; 2] = [0xEA, 0xEE];
        let ping = DevicePing::from_bytes(&data).unwrap();
        assert_eq!(
            ping,
            DevicePing {
                dst_addr: 0xEA,
                src_addr: 0xEE
            }
        );
    }

    #[test]
    fn test_parameter_ping_from_bytes_with_payload() {
        // Should ignore extra payload
        let data: [u8; 5] = [0xEA, 0xEE, 3, 4, 5];
        let ping = DevicePing::from_bytes(&data).unwrap();
        assert_eq!(
            ping,
            DevicePing {
                dst_addr: 0xEA,
                src_addr: 0xEE
            }
        );
    }
    #[test]
    fn test_parameter_ping_buffer_too_small() {
        let ping = DevicePing {
            dst_addr: 0xEA,
            src_addr: 0xEE,
        };
        let mut buffer = [0u8; DevicePing::MIN_PAYLOAD_SIZE - 1];
        let result = ping.to_bytes(&mut buffer);
        assert_eq!(result, Err(CrsfParsingError::BufferOverflow));
    }
}
