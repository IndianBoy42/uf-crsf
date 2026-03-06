use crate::packets::CrsfPacket;
use crate::packets::PacketType;
use crate::CrsfParsingError;

/// Represents an RC Channels Packed packet (Frame Type 0x16).
///
/// This packet contains 16 channels of RC data transmitted from the radio to the receiver.
/// It is the primary means of sending stick positions and switch states in the CRSF protocol.
///
/// # Frame Timing
///
/// This packet is sent continuously at regular intervals, typically:
/// - **Standard CRSF**: Every 4-5ms (250-200 Hz)
/// - **Race mode**: As fast as 2ms (500 Hz)
/// - **Lower rates**: Up to 20ms (50 Hz) depending on protocol configuration
///
/// # Payload Encoding
///
/// The packet uses an efficient bit-packing scheme to fit 16 channels into 22 bytes:
/// - Each channel value is encoded as an **11-bit unsigned integer** (range 0-2047)
/// - 16 channels × 11 bits = 176 bits total
/// - 176 bits ÷ 8 = 22 bytes
///
/// Channels are packed sequentially in little-endian fashion, with each channel's
/// bits split across multiple bytes as needed.
///
/// # Channel Value System
///
/// CRSF uses an 11-bit representation for channel values:
///
/// | Representation | Value | Description |
/// |---------------|-------|-------------|
/// | CRSF Ticks    | 0-2047 | Raw 11-bit values stored in packet |
/// | Microseconds  | 1000-2000µs | Standard servo pulse width |
/// | Center        | 992 ticks / 1500µs | Neutral stick position |
/// | Min/Max       | 172/1811 ticks | 988µs / 2012µs (typical endpoints) |
///
/// ## Conversion Formulas
///
/// To convert between CRSF ticks and microseconds:
///
/// ```text
/// MICROSECONDS = (CRSF_TICKS - 992) * 5 / 8 + 1500
/// CRSF_TICKS   = (MICROSECONDS - 1500) * 8 / 5 + 992
/// ```
///
/// These constants are provided for convenience:
/// - `TICKS_TO_US` multiplier: `5/8` (0.625)
/// - `US_TO_TICKS` multiplier: `8/5` (1.6)
///
/// # Standard Channel Mapping
///
/// The typical assignment of channels in CRSF systems:
///
/// | Channel | Function | Description |
/// |---------|----------|-------------|
/// | 1       | Roll     | Aileron / Left-Right on right stick |
/// | 2       | Pitch    | Elevator / Up-Down on right stick |
/// | 3       | Throttle | Throttle / Up-Down on left stick |
/// | 4       | Yaw      | Rudder / Left-Right on left stick |
/// | 5-8     | AUX1-AUX4 | Flight mode switches, arming, etc. |
/// | 9-16    | AUX5-AUX12 | Additional channels for advanced setups |
///
/// Note: Some radio systems (e.g., EdgeTX/OpenTX) may use 1-based indexing in their UI
/// while this struct uses 0-based indexing (`channels[0]` = Channel 1).
///
/// # Failsafe Behavior
///
/// When signal quality degrades or is lost:
///
/// 1. **Frame loss**: The receiver stops sending RC Channels packets
/// 2. **Flight controller detection**: FC monitors for timeout (recommended: 1 second)
/// 3. **Failsafe activation**: FC enters failsafe mode after timeout
///
/// Recommended timeout before failsafe: **1 second** (1000ms)
///
/// During failsafe, the receiver may continue sending packets with "hold" values
/// (last known good positions) or preset failsafe values, depending on configuration.
///
/// # Examples
///
/// ## Creating from raw CRSF ticks
///
/// ```
/// use uf_crsf::packets::RcChannelsPacked;
///
/// // Create with raw CRSF tick values
/// // 992 = center (1500µs), 172 = minimum (988µs), 1811 = maximum (2012µs)
/// let channels = RcChannelsPacked {
///     channels: [
///         992,  // Channel 1: Roll (center)
///         992,  // Channel 2: Pitch (center)
///         172,  // Channel 3: Throttle (minimum)
///         992,  // Channel 4: Yaw (center)
///         1811, // Channel 5: AUX1 (maximum)
///         992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992
///     ]
/// };
/// ```
///
/// ## Converting between units
///
/// ```
/// use uf_crsf::packets::RcChannelsPacked;
///
/// // Convert CRSF ticks to microseconds
/// let throttle_ticks: u16 = 172;
/// let throttle_us = RcChannelsPacked::ticks_to_us(throttle_ticks);
/// // Note: Due to integer math, minimum throttle is 988µs
/// assert!(throttle_us >= 988 && throttle_us <= 989);
///
/// // Convert microseconds to CRSF ticks
/// let roll_us: u16 = 1500;
/// let roll_ticks = RcChannelsPacked::us_to_ticks(roll_us);
/// assert_eq!(roll_ticks, 992); // Center position
/// ```
///
/// ## Modifying channel values
///
/// ```
/// use uf_crsf::packets::RcChannelsPacked;
///
/// // Create channels and modify values
/// let mut channels = RcChannelsPacked { channels: [992; 16] };
///
/// // Set throttle to minimum using helper
/// channels.channels[2] = RcChannelsPacked::us_to_ticks(1000);
///
/// // Set roll to center
/// channels.channels[0] = RcChannelsPacked::us_to_ticks(1500);
///
/// // Set pitch slightly forward (1600µs)
/// channels.channels[1] = RcChannelsPacked::us_to_ticks(1600);
/// ```
///
/// ## Parsing from bytes
///
/// ```
/// use uf_crsf::packets::{RcChannelsPacked, CrsfPacket};
///
/// // Example raw packet payload (22 bytes)
/// let payload: [u8; 22] = [
///     0x03, 0x1F, 0x58, 0xC0, 0x07, 0x16, 0xB0, 0x80,
///     0x05, 0x2C, 0x60, 0x01, 0x0B, 0xF8, 0xC0, 0x07,
///     0x00, 0x00, 0x00, 0x00, 0x00, 0xFC,
/// ];
///
/// let channels = RcChannelsPacked::from_bytes(&payload).unwrap();
/// ```
#[derive(Debug, PartialEq, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RcChannelsPacked {
    /// The 16 channel values as CRSF ticks (11-bit values, 0-2047).
    ///
    /// Use [`RcChannelsPacked::ticks_to_us`] to convert to microseconds,
    /// or [`RcChannelsPacked::us_to_ticks`] to convert from microseconds.
    pub channels: [u16; 16],
}

impl RcChannelsPacked {
    /// The multiplier for converting CRSF ticks to microseconds.
    ///
    /// `MICROSECONDS = (TICKS - 992) * TICKS_TO_US_NUMERATOR / TICKS_TO_US_DENOMINATOR + 1500`
    pub const TICKS_TO_US_NUMERATOR: i32 = 5;

    /// The denominator for converting CRSF ticks to microseconds.
    pub const TICKS_TO_US_DENOMINATOR: i32 = 8;

    /// The offset value used in CRSF tick to microsecond conversions.
    ///
    /// A CRSF tick value of 992 corresponds to 1500µs (center position).
    pub const CENTER_TICKS: i32 = 992;

    /// The microsecond value corresponding to center position (1500µs).
    pub const CENTER_US: i32 = 1500;

    /// The minimum valid CRSF tick value (172 = ~988µs).
    pub const MIN_TICKS: u16 = 172;

    /// The maximum valid CRSF tick value (1811 = ~2012µs).
    pub const MAX_TICKS: u16 = 1811;

    /// The minimum microsecond value (988µs).
    pub const MIN_US: u16 = 988;

    /// The maximum microsecond value (2012µs).
    pub const MAX_US: u16 = 2012;

    /// Converts CRSF ticks to microseconds.
    ///
    /// # Formula
    ///
    /// `MICROSECONDS = (TICKS - 992) * 5 / 8 + 1500`
    ///
    /// # Parameters
    ///
    /// * `ticks` - The CRSF tick value (0-2047, typically 172-1811)
    ///
    /// # Returns
    ///
    /// The equivalent value in microseconds (typically 988-2012µs)
    ///
    /// # Examples
    ///
    /// ```
    /// use uf_crsf::packets::RcChannelsPacked;
    ///
    /// // Center position
    /// assert_eq!(RcChannelsPacked::ticks_to_us(992), 1500);
    ///
    /// // Minimum throttle
    /// assert_eq!(RcChannelsPacked::ticks_to_us(172), 988);
    ///
    /// // Maximum stick deflection (2011 or 2012 due to integer division)
    /// let max_us = RcChannelsPacked::ticks_to_us(1811);
    /// assert!(max_us >= 2011 && max_us <= 2012);
    /// ```
    pub fn ticks_to_us(ticks: u16) -> u16 {
        let ticks_i32 = i32::from(ticks);
        let us = (ticks_i32 - Self::CENTER_TICKS) * Self::TICKS_TO_US_NUMERATOR
            / Self::TICKS_TO_US_DENOMINATOR
            + Self::CENTER_US;
        us as u16
    }

    /// Converts microseconds to CRSF ticks.
    ///
    /// # Formula
    ///
    /// `TICKS = (MICROSECONDS - 1500) * 8 / 5 + 992`
    ///
    /// # Parameters
    ///
    /// * `us` - The value in microseconds (typically 988-2012µs)
    ///
    /// # Returns
    ///
    /// The equivalent CRSF tick value (typically 172-1811)
    ///
    /// # Examples
    ///
    /// ```
    /// use uf_crsf::packets::RcChannelsPacked;
    ///
    /// // Center position
    /// assert_eq!(RcChannelsPacked::us_to_ticks(1500), 992);
    ///
    /// // Minimum throttle (172 or 173 due to integer division)
    /// let min_ticks = RcChannelsPacked::us_to_ticks(988);
    /// assert!(min_ticks >= 172 && min_ticks <= 173);
    ///
    /// // Maximum stick deflection
    /// assert_eq!(RcChannelsPacked::us_to_ticks(2012), 1811);
    /// ```
    pub fn us_to_ticks(us: u16) -> u16 {
        let us_i32 = i32::from(us);
        let ticks = (us_i32 - Self::CENTER_US) * Self::TICKS_TO_US_DENOMINATOR
            / Self::TICKS_TO_US_NUMERATOR
            + Self::CENTER_TICKS;
        ticks as u16
    }
}

impl CrsfPacket for RcChannelsPacked {
    const PACKET_TYPE: PacketType = PacketType::RcChannelsPacked;
    const MIN_PAYLOAD_SIZE: usize = 16 * 11 / 8; // 16 channels, 11 bit each

    #[allow(clippy::cast_possible_truncation)]
    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, CrsfParsingError> {
        self.validate_buffer_size(buffer)?;
        let ch = &self.channels;
        buffer[0] = (ch[0]) as u8;
        buffer[1] = ((ch[0] >> 8) | (ch[1] << 3)) as u8;
        buffer[2] = ((ch[1] >> 5) | (ch[2] << 6)) as u8;
        buffer[3] = (ch[2] >> 2) as u8;
        buffer[4] = ((ch[2] >> 10) | (ch[3] << 1)) as u8;
        buffer[5] = ((ch[3] >> 7) | (ch[4] << 4)) as u8;
        buffer[6] = ((ch[4] >> 4) | (ch[5] << 7)) as u8;
        buffer[7] = (ch[5] >> 1) as u8;
        buffer[8] = ((ch[5] >> 9) | (ch[6] << 2)) as u8;
        buffer[9] = ((ch[6] >> 6) | (ch[7] << 5)) as u8;
        buffer[10] = (ch[7] >> 3) as u8;
        buffer[11] = ch[8] as u8;
        buffer[12] = ((ch[8] >> 8) | (ch[9] << 3)) as u8;
        buffer[13] = ((ch[9] >> 5) | (ch[10] << 6)) as u8;
        buffer[14] = (ch[10] >> 2) as u8;
        buffer[15] = ((ch[10] >> 10) | (ch[11] << 1)) as u8;
        buffer[16] = ((ch[11] >> 7) | (ch[12] << 4)) as u8;
        buffer[17] = ((ch[12] >> 4) | (ch[13] << 7)) as u8;
        buffer[18] = (ch[13] >> 1) as u8;
        buffer[19] = ((ch[13] >> 9) | (ch[14] << 2)) as u8;
        buffer[20] = ((ch[14] >> 6) | (ch[15] << 5)) as u8;
        buffer[21] = (ch[15] >> 3) as u8;
        Ok(Self::MIN_PAYLOAD_SIZE)
    }

    fn from_bytes(data: &[u8]) -> Result<Self, CrsfParsingError> {
        if data.len() != Self::MIN_PAYLOAD_SIZE {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }

        const MASK_11BIT: u16 = 0x07FF;
        let data_u16: [u16; Self::MIN_PAYLOAD_SIZE] = core::array::from_fn(|i| u16::from(data[i]));
        let mut ch = [MASK_11BIT; 16];
        ch[0] &= data_u16[0] | (data_u16[1] << 8);
        ch[1] &= (data_u16[1] >> 3) | (data_u16[2] << 5);
        ch[2] &= (data_u16[2] >> 6) | (data_u16[3] << 2) | (data_u16[4] << 10);
        ch[3] &= (data_u16[4] >> 1) | (data_u16[5] << 7);
        ch[4] &= (data_u16[5] >> 4) | (data_u16[6] << 4);
        ch[5] &= (data_u16[6] >> 7) | (data_u16[7] << 1) | (data_u16[8] << 9);
        ch[6] &= (data_u16[8] >> 2) | (data_u16[9] << 6);
        ch[7] &= (data_u16[9] >> 5) | (data_u16[10] << 3);
        ch[8] &= data_u16[11] | (data_u16[12] << 8);
        ch[9] &= (data_u16[12] >> 3) | (data_u16[13] << 5);
        ch[10] &= (data_u16[13] >> 6) | (data_u16[14] << 2) | (data_u16[15] << 10);
        ch[11] &= (data_u16[15] >> 1) | (data_u16[16] << 7);
        ch[12] &= (data_u16[16] >> 4) | (data_u16[17] << 4);
        ch[13] &= (data_u16[17] >> 7) | (data_u16[18] << 1) | (data_u16[19] << 9);
        ch[14] &= (data_u16[19] >> 2) | (data_u16[20] << 6);
        ch[15] &= (data_u16[20] >> 5) | (data_u16[21] << 3);
        Ok(RcChannelsPacked { channels: ch })
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::packets::{write_packet_to_buffer, PacketAddress};

    #[test]
    fn test_rc_channels_from_hardware_bytes() {
        // This is the existing test, renamed
        let payload: [u8; 22] = [
            0x03, 0x1F, 0x58, 0xC0, 0x07, 0x16, 0xB0, 0x80, 0x05, 0x2C, 0x60, 0x01, 0x0B, 0xF8,
            0xC0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 252,
        ];
        let rc = RcChannelsPacked::from_bytes(&payload).unwrap();
        let mut buffer: [u8; 22] = [0; 22];
        let consumed = rc.to_bytes(&mut buffer).unwrap();
        assert_eq!(consumed, 22);
        assert_eq!(&buffer, &payload);
    }

    #[test]
    fn test_rc_channels_packed_round_trip() {
        let channels = RcChannelsPacked {
            channels: [
                1000, 1001, 1002, 1003, 1500, 1501, 1502, 1503, 2000, 2001, 2002, 2003, 992, 100,
                500, 1900,
            ],
        };

        let mut buffer: [u8; 22] = [0; 22];
        channels.to_bytes(&mut buffer).unwrap();

        let parsed_channels = RcChannelsPacked::from_bytes(&buffer).unwrap();
        assert_eq!(channels, parsed_channels);
    }

    #[test]
    fn test_from_bytes_invalid_len() {
        let raw_bytes: [u8; 21] = [0; 21];
        let result = RcChannelsPacked::from_bytes(&raw_bytes);
        assert!(matches!(
            result,
            Err(CrsfParsingError::InvalidPayloadLength)
        ));
    }

    #[test]
    fn test_to_bytes_buffer_too_small() {
        let channels = RcChannelsPacked { channels: [0; 16] };
        let mut buffer: [u8; 21] = [0; 21];
        let result = channels.to_bytes(&mut buffer);
        assert!(matches!(result, Err(CrsfParsingError::BufferOverflow)));
    }

    #[test]
    fn test_rc_channels_from_bytes() {
        assert_eq!(RcChannelsPacked::MIN_PAYLOAD_SIZE, 22);
        let channels = RcChannelsPacked {
            channels: [
                1000, 1001, 1002, 1003, 1500, 1501, 1502, 1503, 2000, 2001, 2002, 2003, 992, 100,
                500, 1900,
            ],
        };
        let mut buffer = [0u8; 64];
        let len = write_packet_to_buffer(&mut buffer, PacketAddress::Broadcast, &channels).unwrap();
        let payload = &buffer[3..len - 1];
        let parsed_channels = RcChannelsPacked::from_bytes(payload).unwrap();
        assert_eq!(channels, parsed_channels);
    }

    #[test]
    fn test_rc_channels_to_bytes() {
        let channels = RcChannelsPacked {
            channels: [
                1000, 1001, 1002, 1003, 1500, 1501, 1502, 1503, 2000, 2001, 2002, 2003, 992, 100,
                500, 1900,
            ],
        };

        let mut buffer = [0u8; 22];
        let len = channels.to_bytes(&mut buffer).unwrap();

        let mut expected_buffer = [0u8; 64];
        let expected_len =
            write_packet_to_buffer(&mut expected_buffer, PacketAddress::Broadcast, &channels)
                .unwrap();
        let expected_payload = &expected_buffer[3..expected_len - 1];

        assert_eq!(len, 22);
        assert_eq!(buffer, expected_payload);
    }

    #[test]
    fn test_ticks_to_us_center() {
        // Center position: 992 ticks = 1500µs
        assert_eq!(RcChannelsPacked::ticks_to_us(992), 1500);
    }

    #[test]
    fn test_ticks_to_us_minimum() {
        // Minimum: 172 ticks = 988µs
        assert_eq!(RcChannelsPacked::ticks_to_us(172), 988);
    }

    #[test]
    fn test_ticks_to_us_maximum() {
        // Maximum: 1811 ticks ≈ 2012µs (integer division may result in 2011)
        // (1811 - 992) * 5 / 8 + 1500 = 819 * 5 / 8 + 1500 = 4095 / 8 + 1500 = 511 + 1500 = 2011
        let us = RcChannelsPacked::ticks_to_us(1811);
        assert!(us >= 2011 && us <= 2012);
    }

    #[test]
    fn test_us_to_ticks_center() {
        // Center position: 1500µs = 992 ticks
        assert_eq!(RcChannelsPacked::us_to_ticks(1500), 992);
    }

    #[test]
    fn test_us_to_ticks_minimum() {
        // Minimum: 988µs ≈ 172 ticks (integer division may result in 173)
        // (988 - 1500) * 8 / 5 + 992 = -512 * 8 / 5 + 992 = -4096 / 5 + 992 = -819 + 992 = 173
        let ticks = RcChannelsPacked::us_to_ticks(988);
        assert!(ticks >= 172 && ticks <= 173);
    }

    #[test]
    fn test_us_to_ticks_maximum() {
        // Maximum: 2012µs = 1811 ticks
        assert_eq!(RcChannelsPacked::us_to_ticks(2012), 1811);
    }

    #[test]
    fn test_conversion_round_trip() {
        // Test that conversions are reversible
        let test_values = [
            1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000,
        ];
        for &us in &test_values {
            let ticks = RcChannelsPacked::us_to_ticks(us);
            let back_to_us = RcChannelsPacked::ticks_to_us(ticks);
            // Allow for rounding errors (±1µs)
            assert!(
                (i32::from(back_to_us) - i32::from(us)).abs() <= 1,
                "Round-trip failed for {}µs: got {}µs",
                us,
                back_to_us
            );
        }
    }

    #[test]
    fn test_conversion_constants() {
        // Verify the constants are correct
        assert_eq!(RcChannelsPacked::CENTER_TICKS, 992);
        assert_eq!(RcChannelsPacked::CENTER_US, 1500);
        assert_eq!(RcChannelsPacked::MIN_TICKS, 172);
        assert_eq!(RcChannelsPacked::MAX_TICKS, 1811);
        assert_eq!(RcChannelsPacked::MIN_US, 988);
        assert_eq!(RcChannelsPacked::MAX_US, 2012);
    }
}
