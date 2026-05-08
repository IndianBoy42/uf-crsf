use crate::packets::CrsfPacket;
use crate::packets::PacketType;
use crate::CrsfParsingError;

/// Battery Sensor packet (CRSF frame type 0x08).
///
/// This packet represents battery telemetry data sent from the flight controller (FC)
/// to the receiver and then to the transmitter. It provides real-time information
/// about the battery's electrical state, including voltage, current draw, capacity
/// consumed, and remaining charge percentage.
///
/// # When It's Sent
///
/// The flight controller periodically sends this packet (typically every 100-200ms)
/// when battery monitoring is enabled. The data is usually obtained from a power
/// module or battery monitoring circuit connected to the FC.
///
/// # CRSF Protocol Details
///
/// - **Frame Type**: 0x08 (Battery Sensor)
/// - **Payload Size**: 8 bytes
/// - **Direction**: FC → Receiver → Transmitter
///
/// The payload is structured as follows:
/// - Bytes 0-1: Voltage (16-bit signed big-endian, in 10mV units)
/// - Bytes 2-3: Current (16-bit signed big-endian, in 10mA units)
/// - Bytes 4-6: Capacity Used (24-bit unsigned big-endian, in mAh)
/// - Byte 7: Remaining Percentage (8-bit unsigned, 0-100%)
///
/// # 24-Bit Capacity Field
///
/// The capacity used field is a 24-bit value packed into 3 bytes. This allows for
/// a maximum capacity value of 16,777,215 mAh (approximately 16,777 Ah). The value
/// is encoded in big-endian format, meaning the most significant byte comes first.
///
/// When serializing, the value is truncated to 24 bits (bytes 1-3 of the 4-byte u32).
/// When deserializing, the value is zero-extended to 32 bits for easier handling.
///
/// # Example
///
/// ```
/// use uf_crsf::packets::{Battery, CrsfPacket};
///
/// // Create a battery packet (12.34V, 10.0A discharged, 5000mAh used, 75% remaining)
/// let battery = Battery::new(1234, 1000, 5000, 75).unwrap();
///
/// // Convert to human-readable units
/// let voltage_v = battery.voltage as f32 / 100.0; // 12.34V
/// let current_a = battery.current as f32 / 100.0; // 10.0A
///
/// // Serialize to bytes for transmission
/// let mut buffer = [0u8; Battery::MIN_PAYLOAD_SIZE];
/// let bytes_written = battery.to_bytes(&mut buffer).unwrap();
/// assert_eq!(bytes_written, 8);
///
/// // Deserialize from bytes
/// let parsed = Battery::from_bytes(&buffer).unwrap();
/// assert_eq!(parsed.voltage, 1234);
/// assert_eq!(parsed.remaining, 75);
/// ```
#[derive(Default, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Battery {
    /// Battery voltage in 10mV units (e.g., 1234 represents 12.34V).
    ///
    /// This is the total pack voltage measured at the flight controller.
    /// For LiPo batteries, typical values range from:
    /// - 3.0V per cell (empty) to 4.2V per cell (fully charged)
    /// - A 4S battery would range from 12.0V (empty) to 16.8V (full)
    ///
    /// To convert to volts: `voltage_v = self.voltage as f32 / 100.0`
    ///
    /// # Typical Values
    /// - 1S (3.7V nominal): 300-420 (3.0V-4.2V)
    /// - 2S (7.4V nominal): 600-840 (6.0V-8.4V)
    /// - 3S (11.1V nominal): 900-1260 (9.0V-12.6V)
    /// - 4S (14.8V nominal): 1200-1680 (12.0V-16.8V)
    /// - 6S (22.2V nominal): 1800-2520 (18.0V-25.2V)
    pub voltage: i16,

    /// Battery current in 10mA units (e.g., 100 represents 1.0A).
    ///
    /// This is the instantaneous current draw from the battery. Positive values
    /// indicate discharge (battery being drained), negative values indicate charge
    /// (battery being charged, though this is rare in flight).
    ///
    /// To convert to amperes: `current_a = self.current as f32 / 100.0`
    ///
    /// # Typical Values
    /// - Idle/Disarmed: 0-50 (0-0.5A for FC and RX)
    /// - Hovering: 500-2000 (5-20A depending on craft size)
    /// - Full throttle: 5000-30000 (50-300A for racing/freestyle quads)
    ///
    /// # Range
    /// The 16-bit signed integer allows values from -327.68A to +327.67A,
    /// which covers virtually all FPV drone applications.
    pub current: i16,

    /// Capacity used in milliampere-hours (mAh), stored as a 24-bit value.
    ///
    /// This represents the total energy consumed from the battery since it was
    /// fully charged. It is accumulated over time by integrating the current draw.
    /// This value can be compared against the battery's rated capacity to determine
    /// how much energy remains.
    ///
    /// # Usage Example
    /// ```
    /// use uf_crsf::packets::Battery;
    ///
    /// // If you have a 5000mAh battery and 3500mAh has been used:
    /// let battery = Battery::new(1600, 100, 3500, 30).unwrap();
    /// let remaining_mah = 5000 - battery.capacity_used; // 1500mAh remaining
    /// ```
    ///
    /// # Maximum Value
    /// The 24-bit encoding allows values up to 16,777,215 mAh (about 16.8 Ah).
    /// This is sufficient for even very large battery packs.
    pub capacity_used: u32,

    /// Battery remaining capacity as a percentage (0-100).
    ///
    /// This is typically calculated by the flight controller based on either:
    /// - Voltage monitoring (estimating SOC from voltage under load)
    /// - Coulomb counting (tracking mAh consumed vs rated capacity)
    /// - A combination of both methods
    ///
    /// # Typical Values
    /// - 100%: Fully charged
    /// - 80-90%: Normal operating range (land soon)
    /// - 70-80%: Critical (land immediately)
    /// - <70%: Dangerously low, risk of battery damage
    ///
    /// # Note
    /// Some flight controllers may report values outside 0-100 due to calibration
    /// errors or voltage sag under high current draw. Values should be clamped
    /// to 0-100 for display purposes.
    pub remaining: u8,
}

impl Battery {
    /// Creates a new Battery packet with the specified values.
    ///
    /// # Arguments
    ///
    /// * `voltage` - Battery voltage in 10mV units (e.g., 1234 for 12.34V)
    /// * `current` - Current draw in 10mA units (positive for discharge)
    /// * `capacity_used` - Capacity consumed in mAh (max 16,777,215)
    /// * `remaining` - Remaining percentage (typically 0-100)
    ///
    /// # Errors
    ///
    /// Currently this function always returns Ok, but may validate ranges
    /// in future versions (e.g., ensuring capacity_used fits in 24 bits).
    ///
    /// # Example
    ///
    /// ```
    /// use uf_crsf::packets::Battery;
    ///
    /// // 4S LiPo at storage voltage (15.2V), drawing 5A, 2000mAh used, 60% remaining
    /// let battery = Battery::new(1520, 500, 2000, 60).unwrap();
    /// ```
    pub fn new(
        voltage: i16,
        current: i16,
        capacity_used: u32,
        remaining: u8,
    ) -> Result<Self, CrsfParsingError> {
        Ok(Self {
            voltage,
            current,
            capacity_used,
            remaining,
        })
    }
}

impl CrsfPacket for Battery {
    const PACKET_TYPE: PacketType = PacketType::BatterySensor;
    // 24 bit (3 bytes) unpacked into u32 (4 bytes)
    const MIN_PAYLOAD_SIZE: usize = 2 * size_of::<i16>() + 3 + size_of::<u8>();

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, CrsfParsingError> {
        self.validate_buffer_size(buffer)?;
        buffer[0..2].copy_from_slice(&self.voltage.to_be_bytes());
        buffer[2..4].copy_from_slice(&self.current.to_be_bytes());
        // Take only the last 3 bytes (24-bit capacity encoding)
        buffer[4..7].copy_from_slice(&self.capacity_used.to_be_bytes()[1..]);
        buffer[7] = self.remaining;
        Ok(Self::MIN_PAYLOAD_SIZE)
    }

    fn from_bytes(data: &[u8]) -> Result<Self, CrsfParsingError> {
        if data.len() < Self::MIN_PAYLOAD_SIZE {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }
        let mut capacity_bytes: [u8; 4] = [0; 4];
        capacity_bytes[1..].copy_from_slice(&data[4..7]);

        Ok(Self {
            voltage: i16::from_be_bytes(
                data[0..2]
                    .try_into()
                    .map_err(|_| CrsfParsingError::InvalidPayloadLength)?,
            ),
            current: i16::from_be_bytes(
                data[2..4]
                    .try_into()
                    .map_err(|_| CrsfParsingError::InvalidPayloadLength)?,
            ),
            capacity_used: u32::from_be_bytes(capacity_bytes),
            remaining: data[7],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_battery_new() {
        let battery = Battery::new(12345, -1000, 1234567, 75).unwrap();
        assert_eq!(battery.voltage, 12345);
        assert_eq!(battery.current, -1000);
        assert_eq!(battery.capacity_used, 1234567);
        assert_eq!(battery.remaining, 75);
    }

    #[test]
    fn test_battery_to_bytes() {
        assert_eq!(Battery::MIN_PAYLOAD_SIZE, 8);
        let battery = Battery::new(12345, -1000, 1234567, 75).unwrap();

        let mut buffer = [0u8; Battery::MIN_PAYLOAD_SIZE];
        battery.to_bytes(&mut buffer).unwrap();

        let expected_bytes: [u8; Battery::MIN_PAYLOAD_SIZE] =
            [0x30, 0x39, 0xfc, 0x18, 0x12, 0xd6, 0x87, 0x4b];

        assert_eq!(buffer, expected_bytes);
    }

    #[test]
    fn test_battery_from_bytes() {
        let data: [u8; Battery::MIN_PAYLOAD_SIZE] =
            [0x30, 0x39, 0xfc, 0x18, 0x12, 0xd6, 0x87, 0x4b];

        let battery = Battery::from_bytes(&data).unwrap();

        assert_eq!(
            battery,
            Battery {
                voltage: 12345,
                current: -1000,
                capacity_used: 1234567,
                remaining: 75,
            }
        );
    }

    #[test]
    fn test_battery_round_trip() {
        let battery = Battery {
            voltage: 12345,
            current: -1000,
            capacity_used: 1234567,
            remaining: 75,
        };

        let mut buffer = [0u8; Battery::MIN_PAYLOAD_SIZE];
        battery.to_bytes(&mut buffer).unwrap();

        let round_trip_battery = Battery::from_bytes(&buffer).unwrap();

        assert_eq!(battery, round_trip_battery);
    }

    #[test]
    fn test_edge_cases() {
        let battery = Battery {
            voltage: -32768,
            current: 32767,
            capacity_used: 16777215, // Max 24-bit value
            remaining: 255,
        };

        let mut buffer = [0u8; Battery::MIN_PAYLOAD_SIZE];
        battery.to_bytes(&mut buffer).unwrap();
        let round_trip_battery = Battery::from_bytes(&buffer).unwrap();
        assert_eq!(battery, round_trip_battery);
    }

    #[test]
    fn test_battery_to_bytes_buffer_too_small() {
        let battery = Battery {
            voltage: 12345,
            current: -1000,
            capacity_used: 1234567,
            remaining: 75,
        };

        let mut buffer = [0u8; 5];
        let result = battery.to_bytes(&mut buffer);
        assert_eq!(result, Err(CrsfParsingError::BufferOverflow));
    }

    #[test]
    fn test_battery_from_bytes_invalide_size() {
        let data: [u8; 3] = [0x04; 3];
        let result = Battery::from_bytes(&data);
        assert_eq!(result, Err(CrsfParsingError::InvalidPayloadLength));
    }
}
