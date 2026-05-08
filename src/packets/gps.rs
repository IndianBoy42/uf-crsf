use crate::packets::CrsfPacket;
use crate::packets::PacketType;
use crate::CrsfParsingError;
use core::mem::size_of;

/// Represents a GPS packet (type 0x02).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Gps {
    /// Latitude in degrees * 10^7.
    pub latitude: i32,
    /// Longitude in degrees * 10^7.
    pub longitude: i32,
    /// Groundspeed in 0.01 km/h units.
    pub groundspeed: u16,
    /// Heading in 0.01 degree units.
    pub heading: u16,
    /// Altitude with 1000m offset.
    ///
    /// The CRSF protocol stores altitude with a 1000 meter offset to allow
    /// representing altitudes from -1000m to 64535m using an unsigned u16.
    /// - Raw value 0 corresponds to -1000m (below sea level)
    /// - Raw value 1000 corresponds to 0m (sea level)
    /// - Raw value 2000 corresponds to 1000m above sea level
    ///
    /// Use [`Self::altitude_meters`] to get the actual altitude in meters
    /// and [`Self::set_altitude_meters`] to set altitude from meters.
    pub altitude: u16,
    /// Number of satellites in view.
    pub satellites: u8,
}

impl Gps {
    /// Creates a new GPS packet with raw values.
    pub fn new(
        latitude: i32,
        longitude: i32,
        groundspeed: u16,
        heading: u16,
        altitude: u16,
        satellites: u8,
    ) -> Result<Self, CrsfParsingError> {
        Ok(Self {
            latitude,
            longitude,
            groundspeed,
            heading,
            altitude,
            satellites,
        })
    }

    /// Creates a GPS packet from components with altitude in meters.
    ///
    /// This constructor handles the 1000m offset for altitude automatically.
    ///
    /// # Arguments
    ///
    /// * `latitude` - Latitude in degrees * 10^7
    /// * `longitude` - Longitude in degrees * 10^7
    /// * `groundspeed` - Groundspeed in 0.01 km/h units
    /// * `heading` - Heading in 0.01 degree units
    /// * `altitude_meters` - Altitude in meters (e.g., -500 for 500m below sea level,
    ///   0 for sea level, 1500 for 1500m above sea level)
    /// * `satellites` - Number of satellites in view
    ///
    /// # Errors
    ///
    /// Returns `CrsfParsingError::InvalidPayload` if the altitude in meters
    /// is outside the representable range of -1000m to 64535m.
    pub fn from_components(
        latitude: i32,
        longitude: i32,
        groundspeed: u16,
        heading: u16,
        altitude_meters: i32,
        satellites: u8,
    ) -> Result<Self, CrsfParsingError> {
        let altitude_raw = Self::meters_to_raw(altitude_meters)?;
        Ok(Self {
            latitude,
            longitude,
            groundspeed,
            heading,
            altitude: altitude_raw,
            satellites,
        })
    }

    /// Returns the altitude in meters, accounting for the 1000m offset.
    ///
    /// The CRSF protocol stores altitude with a 1000m offset:
    /// - Raw value 0 = -1000m
    /// - Raw value 1000 = 0m (sea level)
    /// - Raw value 2000 = 1000m
    ///
    /// # Returns
    ///
    /// The actual altitude in meters as an i32.
    pub fn altitude_meters(&self) -> i32 {
        i32::from(self.altitude) - 1000
    }

    /// Sets the altitude from meters, applying the 1000m offset.
    ///
    /// # Arguments
    ///
    /// * `meters` - The altitude in meters (e.g., -500 for 500m below sea level,
    ///   0 for sea level, 1500 for 1500m above sea level)
    ///
    /// # Errors
    ///
    /// Returns `CrsfParsingError::InvalidPayload` if the altitude is outside
    /// the representable range of -1000m to 64535m.
    pub fn set_altitude_meters(&mut self, meters: i32) -> Result<(), CrsfParsingError> {
        self.altitude = Self::meters_to_raw(meters)?;
        Ok(())
    }

    /// Converts altitude in meters to raw CRSF value with 1000m offset.
    ///
    /// # Arguments
    ///
    /// * `meters` - The altitude in meters
    ///
    /// # Errors
    ///
    /// Returns `CrsfParsingError::InvalidPayload` if the altitude is outside
    /// the representable range of -1000m to 64535m.
    fn meters_to_raw(meters: i32) -> Result<u16, CrsfParsingError> {
        let raw = meters + 1000;
        if raw < 0 || raw > u16::MAX as i32 {
            return Err(CrsfParsingError::InvalidPayload);
        }
        Ok(raw as u16)
    }
}

impl CrsfPacket for Gps {
    const PACKET_TYPE: PacketType = PacketType::Gps;
    const MIN_PAYLOAD_SIZE: usize = 2 * size_of::<i32>() + 3 * size_of::<u16>() + size_of::<u8>();

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, CrsfParsingError> {
        self.validate_buffer_size(buffer)?;
        buffer[0..4].copy_from_slice(&self.latitude.to_be_bytes());
        buffer[4..8].copy_from_slice(&self.longitude.to_be_bytes());
        buffer[8..10].copy_from_slice(&self.groundspeed.to_be_bytes());
        buffer[10..12].copy_from_slice(&self.heading.to_be_bytes());
        buffer[12..14].copy_from_slice(&self.altitude.to_be_bytes());
        buffer[14] = self.satellites;

        Ok(Self::MIN_PAYLOAD_SIZE)
    }

    fn from_bytes(data: &[u8]) -> Result<Self, CrsfParsingError> {
        if data.len() < Self::MIN_PAYLOAD_SIZE {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }

        Ok(Self {
            latitude: i32::from_be_bytes(
                data[0..4]
                    .try_into()
                    .map_err(|_e| CrsfParsingError::InvalidPayloadLength)?,
            ),
            longitude: i32::from_be_bytes(
                data[4..8]
                    .try_into()
                    .map_err(|_e| CrsfParsingError::InvalidPayloadLength)?,
            ),
            groundspeed: u16::from_be_bytes(
                data[8..10]
                    .try_into()
                    .map_err(|_e| CrsfParsingError::InvalidPayloadLength)?,
            ),
            heading: u16::from_be_bytes(
                data[10..12]
                    .try_into()
                    .map_err(|_e| CrsfParsingError::InvalidPayloadLength)?,
            ),
            altitude: u16::from_be_bytes(
                data[12..14]
                    .try_into()
                    .map_err(|_e| CrsfParsingError::InvalidPayloadLength)?,
            ),
            satellites: data[14],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::packets::{write_packet_to_buffer, PacketAddress};

    #[test]
    fn test_gps_new() {
        let gps = Gps::new(1, 2, 3, 4, 5, 6).unwrap();
        assert_eq!(gps.latitude, 1);
        assert_eq!(gps.longitude, 2);
        assert_eq!(gps.groundspeed, 3);
        assert_eq!(gps.heading, 4);
        assert_eq!(gps.altitude, 5);
        assert_eq!(gps.satellites, 6);
    }

    #[test]
    fn test_gps_from_bytes() {
        assert_eq!(Gps::MIN_PAYLOAD_SIZE, 15);
        let gps = Gps {
            latitude: 124108701,
            longitude: -276434195,
            groundspeed: 26,
            heading: 3500,
            altitude: 1050,
            satellites: 15,
        };
        let mut buffer = [0u8; 64];
        let len = write_packet_to_buffer(&mut buffer, PacketAddress::Broadcast, &gps).unwrap();
        let payload = &buffer[3..len - 1];
        let parsed_gps = Gps::from_bytes(payload).unwrap();
        assert_eq!(gps, parsed_gps);
    }

    #[test]
    fn test_gps_to_bytes() {
        let gps = Gps {
            latitude: 124108701,
            longitude: -276434195,
            groundspeed: 26,
            heading: 3500,
            altitude: 1050,
            satellites: 15,
        };

        let mut buffer = [0u8; 15];
        let len = gps.to_bytes(&mut buffer).unwrap();

        let mut expected_buffer = [0u8; 64];
        let expected_len =
            write_packet_to_buffer(&mut expected_buffer, PacketAddress::Broadcast, &gps).unwrap();
        let expected_payload = &expected_buffer[3..expected_len - 1];

        assert_eq!(len, 15);
        assert_eq!(buffer, expected_payload);
    }

    #[test]
    fn test_gps_round_trip() {
        let gps = Gps::new(525200000, 134050000, 5000, 18000, 1100, 12).unwrap();

        let mut buffer: [u8; 15] = [0; 15];
        gps.to_bytes(&mut buffer).unwrap();

        let parsed_gps = Gps::from_bytes(&buffer).unwrap();
        assert_eq!(gps, parsed_gps);
    }

    #[test]
    fn test_from_bytes_invalid_len() {
        let raw_bytes: [u8; 14] = [0; 14];
        let result = Gps::from_bytes(&raw_bytes);
        assert!(matches!(
            result,
            Err(CrsfParsingError::InvalidPayloadLength)
        ));
    }

    #[test]
    fn test_to_bytes_buffer_too_small() {
        let gps = Gps {
            latitude: 0,
            longitude: 0,
            groundspeed: 0,
            heading: 0,
            altitude: 0,
            satellites: 0,
        };
        let mut buffer: [u8; 14] = [0; 14];
        let result = gps.to_bytes(&mut buffer);
        assert!(matches!(result, Err(CrsfParsingError::BufferOverflow)));
    }

    #[test]
    fn test_gps_from_hardware_bytes() {
        // Raw packet from hardware: [234, 17, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 235, 0, 26]
        // Payload is the 15 bytes after the type.
        let payload: [u8; 15] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 235, 0];
        let gps = Gps::from_bytes(&payload).unwrap();

        assert_eq!(gps.latitude, 0);
        assert_eq!(gps.longitude, 0);
        assert_eq!(gps.groundspeed, 0);
        assert_eq!(gps.heading, 0);
        assert_eq!(gps.altitude, 1003);
        assert_eq!(gps.satellites, 0);

        // Test round-trip
        let mut buffer: [u8; 15] = [0; 15];
        gps.to_bytes(&mut buffer).unwrap();
        assert_eq!(buffer, payload);
        let parsed_gps = Gps::from_bytes(&buffer).unwrap();
        assert_eq!(gps, parsed_gps);
    }

    // Altitude offset handling tests
    #[test]
    fn test_altitude_negative_below_sea_level() {
        // 500m below sea level
        let gps = Gps::from_components(0, 0, 0, 0, -500, 0).unwrap();
        assert_eq!(gps.altitude, 500); // raw value = -500 + 1000 = 500
        assert_eq!(gps.altitude_meters(), -500);
    }

    #[test]
    fn test_altitude_sea_level() {
        // At sea level
        let gps = Gps::from_components(0, 0, 0, 0, 0, 0).unwrap();
        assert_eq!(gps.altitude, 1000); // raw value = 0 + 1000 = 1000
        assert_eq!(gps.altitude_meters(), 0);
    }

    #[test]
    fn test_altitude_positive_above_sea_level() {
        // 1500m above sea level
        let gps = Gps::from_components(0, 0, 0, 0, 1500, 0).unwrap();
        assert_eq!(gps.altitude, 2500); // raw value = 1500 + 1000 = 2500
        assert_eq!(gps.altitude_meters(), 1500);
    }

    #[test]
    fn test_altitude_minimum_boundary() {
        // Minimum representable altitude: -1000m
        let gps = Gps::from_components(0, 0, 0, 0, -1000, 0).unwrap();
        assert_eq!(gps.altitude, 0); // raw value = -1000 + 1000 = 0
        assert_eq!(gps.altitude_meters(), -1000);
    }

    #[test]
    fn test_altitude_maximum_boundary() {
        // Maximum representable altitude: 64535m (u16::MAX - 1000)
        let gps = Gps::from_components(0, 0, 0, 0, 64535, 0).unwrap();
        assert_eq!(gps.altitude, 65535); // raw value = 64535 + 1000 = 65535
        assert_eq!(gps.altitude_meters(), 64535);
    }

    #[test]
    fn test_altitude_too_low_error() {
        // Below minimum: -1001m should fail
        let result = Gps::from_components(0, 0, 0, 0, -1001, 0);
        assert!(matches!(result, Err(CrsfParsingError::InvalidPayload)));
    }

    #[test]
    fn test_altitude_too_high_error() {
        // Above maximum: 64536m should fail
        let result = Gps::from_components(0, 0, 0, 0, 64536, 0);
        assert!(matches!(result, Err(CrsfParsingError::InvalidPayload)));
    }

    #[test]
    fn test_set_altitude_meters() {
        let mut gps = Gps::new(0, 0, 0, 0, 0, 0).unwrap();

        // Set to sea level
        gps.set_altitude_meters(0).unwrap();
        assert_eq!(gps.altitude, 1000);

        // Set to 500m above
        gps.set_altitude_meters(500).unwrap();
        assert_eq!(gps.altitude, 1500);

        // Set to 200m below
        gps.set_altitude_meters(-200).unwrap();
        assert_eq!(gps.altitude, 800);
    }

    #[test]
    fn test_set_altitude_meters_out_of_range() {
        let mut gps = Gps::new(0, 0, 0, 0, 0, 0).unwrap();

        // Too low
        assert!(matches!(
            gps.set_altitude_meters(-1001),
            Err(CrsfParsingError::InvalidPayload)
        ));

        // Too high
        assert!(matches!(
            gps.set_altitude_meters(64536),
            Err(CrsfParsingError::InvalidPayload)
        ));
    }

    #[test]
    fn test_altitude_round_trip() {
        // Test that altitude_meters and from_components are inverse operations
        let test_altitudes = [
            -1000i32, // Minimum
            -500,     // Below sea level
            -1,       // Just below sea level
            0,        // Sea level
            1,        // Just above sea level
            500,      // Above sea level
            1000,     // 1km
            64535,    // Maximum
        ];

        for &alt in &test_altitudes {
            let gps = Gps::from_components(0, 0, 0, 0, alt, 0).unwrap();
            assert_eq!(
                gps.altitude_meters(),
                alt,
                "Altitude round-trip failed for {}m",
                alt
            );
        }
    }

    #[test]
    fn test_hardware_altitude_with_offset() {
        // From test_gps_from_hardware_bytes: raw altitude = 1003
        // This should correspond to 3m above sea level
        let payload: [u8; 15] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 235, 0];
        let gps = Gps::from_bytes(&payload).unwrap();

        assert_eq!(gps.altitude, 1003);
        assert_eq!(gps.altitude_meters(), 3); // 1003 - 1000 = 3m
    }
}
