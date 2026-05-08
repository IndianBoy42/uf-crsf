use crate::packets::CrsfPacket;
use crate::packets::PacketType;
use crate::CrsfParsingError;

/// Represents a Barometric Altitude & Vertical Speed packet.
///
/// This frame sends altitude and vertical speed in a bit-efficient way using 3 bytes.
///
/// # Altitude Encoding (16 bits)
/// Uses dual-mode encoding based on MSB:
/// - MSB = 0: Altitude in decimeters with -10000dm offset (range: -1000m to ~2276.7m)
///   - Value 0 represents -1000m, value 10000 represents 0m (start altitude)
///   - Maximum value 0x7fff (32767) represents 22767dm = 2276.7m
/// - MSB = 1: Altitude in meters, no offset (range: ~3276m)
///   - Values 0x8000 to 0xffff represent 0m to 32767m
///
/// # Vertical Speed Encoding (8 bits, signed)
/// Uses logarithmic compression for higher precision at low speeds:
/// - Range: ±2616 cm/s (±26.16 m/s)
/// - Precision varies by speed:
///   - ~3 cm/s precision at low speeds (near 0)
///   - ~70 cm/s precision at speeds around 25 m/s
/// - Constants: KL=100 (linearity), KR=0.026 (range)
#[derive(Default, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BaroAltitude {
    /// Packed altitude above start (calibration) point.
    /// See `get_altitude_dm()` for unpacking.
    pub altitude_packed: u16,
    /// Packed vertical speed. See `get_vertical_speed_cm_s()` for unpacking.
    pub vertical_speed_packed: i8,
}

impl BaroAltitude {
    pub fn new(altitude_packed: u16, vertical_speed_packed: i8) -> Result<Self, CrsfParsingError> {
        Ok(Self {
            altitude_packed,
            vertical_speed_packed,
        })
    }
}

impl BaroAltitude {
    /// MSB = 0: altitude is in decimeters - 10000dm offset (so 0 represents -1000m; 10000 represents 0m (starting altitude); 0x7fff represents 2276.7m);
    /// MSB = 1: altitude is in meters. Without any offset.
    pub fn get_altitude_dm(&self) -> i32 {
        if (self.altitude_packed & 0x8000) != 0 {
            (i32::from(self.altitude_packed & 0x7fff)) * 10
        } else {
            (i32::from(self.altitude_packed)) - 10000
        }
    }

    pub fn get_altitude_packed(altitude_dm: i32) -> u16 {
        const ALT_MIN_DM: i32 = 10000;
        const ALT_THRESHOLD_DM: i32 = 0x8000 - ALT_MIN_DM;
        const ALT_MAX_DM: i32 = 0x7ffe * 10 - 5;

        if altitude_dm < -ALT_MIN_DM {
            0
        } else if altitude_dm > ALT_MAX_DM {
            0xfffe
        } else if altitude_dm < ALT_THRESHOLD_DM {
            (altitude_dm + ALT_MIN_DM) as u16
        } else {
            (((altitude_dm + 5) / 10) | 0x8000) as u16
        }
    }
}

// ---------------------------------------------------------------------------
// Vertical speed: LUT-based implementation (default, no libm required)
// ---------------------------------------------------------------------------
#[cfg(not(feature = "baro-math"))]
impl BaroAltitude {
    /// Decode the packed vertical speed to cm/s using a precomputed lookup table.
    ///
    /// The LUT replaces the `powf` call that would otherwise require `libm`,
    /// making this work on FPU-less microcontrollers without soft-float overhead.
    pub fn get_vertical_speed_cm_s(&self) -> i16 {
        VSPEED_DECODE_LUT[(self.vertical_speed_packed as usize).wrapping_add(128)]
    }

    /// Encode a vertical speed in cm/s to the packed i8 representation using
    /// binary search on the decode LUT.
    ///
    /// The function is monotonic in the positive domain, so binary search
    /// finds the closest packed value whose LUT entry is nearest to the
    /// requested speed.
    pub fn get_vertical_speed_packed(vertical_speed_cm_s: i16) -> i8 {
        if vertical_speed_cm_s == 0 {
            return 0;
        }

        let abs_val = vertical_speed_cm_s.unsigned_abs() as i16;
        let sign: i8 = if vertical_speed_cm_s > 0 { 1 } else { -1 };

        // Binary search on the positive part of the LUT (indices 128..255).
        // Index 128 = packed 0, index 255 = packed 127.
        let mut lo: usize = 128;
        let mut hi: usize = 255;

        while lo < hi {
            let mid = lo + (hi - lo) / 2;
            if VSPEED_DECODE_LUT[mid] < abs_val {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }

        // `lo` is now the first index whose LUT value >= abs_val.
        // Check whether the neighbour below is closer.
        if lo > 128 {
            let diff_lo = (VSPEED_DECODE_LUT[lo] - abs_val).unsigned_abs();
            let diff_prev = (abs_val - VSPEED_DECODE_LUT[lo - 1]).unsigned_abs();
            if diff_prev < diff_lo {
                lo -= 1;
            }
        }

        ((lo - 128) as i8) * sign
    }
}

// ---------------------------------------------------------------------------
// Vertical speed: libm-based implementation (opt-in via `baro-math` feature)
// ---------------------------------------------------------------------------
#[cfg(feature = "baro-math")]
impl BaroAltitude {
    /// Decode the packed vertical speed to cm/s using `libm`'s `powf`.
    pub fn get_vertical_speed_cm_s(&self) -> i16 {
        ((libm::powf(
            core::f32::consts::E,
            (self.vertical_speed_packed.unsigned_abs() as f32) * KR,
        ) - 1.0)
            * KL
            * (self.vertical_speed_packed.signum() as f32)) as i16
    }

    /// Encode a vertical speed in cm/s to the packed i8 representation using
    /// `libm`'s `logf`.
    pub fn get_vertical_speed_packed(vertical_speed_cm_s: i16) -> i8 {
        (libm::logf((vertical_speed_cm_s.unsigned_abs() as f32) / KL + 1.0) / KR
            * (vertical_speed_cm_s.signum() as f32)) as i8
    }
}

/// Linearity constant for vertical speed encoding.
#[cfg(feature = "baro-math")]
const KL: f32 = 100.0;
/// Range constant for vertical speed encoding.
#[cfg(feature = "baro-math")]
const KR: f32 = 0.026;

/// Decode lookup table: maps packed i8 value to vertical speed in cm/s.
///
/// Index = `packed_i8 + 128` (so index 0 = packed -128, index 128 = packed 0,
/// index 255 = packed 127).
///
/// Values are computed with the formula `(e^(|packed| * KR) - 1) * KL * signum(packed)`
/// using truncation to `i16`, matching the original `libm`-based `as i16` cast.
#[cfg(not(feature = "baro-math"))]
const VSPEED_DECODE_LUT: [i16; 256] = [
    -2688, -2616, -2546, -2479, -2412, -2348, -2285, -2224, -2164, -2106, -2049,
    -1994, -1940, -1888, -1837, -1787, -1739, -1692, -1646, -1601, -1557, -1515,
    -1473, -1433, -1393, -1355, -1318, -1281, -1246, -1211, -1178, -1145, -1113,
    -1082, -1051, -1022, -993, -965, -938, -911, -885, -860, -835, -811, -788,
    -765, -743, -721, -700, -679, -659, -640, -621, -602, -584, -567, -550, -533,
    -517, -501, -485, -470, -456, -441, -428, -414, -401, -388, -375, -363, -351,
    -340, -328, -317, -307, -296, -286, -276, -266, -257, -248, -239, -230, -222,
    -213, -205, -198, -190, -182, -175, -168, -161, -154, -148, -142, -135, -129,
    -123, -118, -112, -107, -101, -96, -91, -86, -81, -77, -72, -68, -63, -59, -55,
    -51, -47, -43, -40, -36, -33, -29, -26, -23, -19, -16, -13, -10, -8, -5, -2, 0,
    2, 5, 8, 10, 13, 16, 19, 23, 26, 29, 33, 36, 40, 43, 47, 51, 55, 59, 63, 68,
    72, 77, 81, 86, 91, 96, 101, 107, 112, 118, 123, 129, 135, 142, 148, 154, 161,
    168, 175, 182, 190, 198, 205, 213, 222, 230, 239, 248, 257, 266, 276, 286, 296,
    307, 317, 328, 340, 351, 363, 375, 388, 401, 414, 428, 441, 456, 470, 485, 501,
    517, 533, 550, 567, 584, 602, 621, 640, 659, 679, 700, 721, 743, 765, 788, 811,
    835, 860, 885, 911, 938, 965, 993, 1022, 1051, 1082, 1113, 1145, 1178, 1211,
    1246, 1281, 1318, 1355, 1393, 1433, 1473, 1515, 1557, 1601, 1646, 1692, 1739,
    1787, 1837, 1888, 1940, 1994, 2049, 2106, 2164, 2224, 2285, 2348, 2412, 2479,
    2546, 2616,
];

impl CrsfPacket for BaroAltitude {
    const PACKET_TYPE: PacketType = PacketType::BaroAltitude;
    const MIN_PAYLOAD_SIZE: usize = size_of::<u16>() + size_of::<i8>();

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, CrsfParsingError> {
        self.validate_buffer_size(buffer)?;
        buffer[0..2].copy_from_slice(&self.altitude_packed.to_be_bytes());
        buffer[2] = self.vertical_speed_packed as u8;
        Ok(Self::MIN_PAYLOAD_SIZE)
    }

    fn from_bytes(data: &[u8]) -> Result<Self, CrsfParsingError> {
        if data.len() != Self::MIN_PAYLOAD_SIZE {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }

        Ok(Self {
            altitude_packed: u16::from_be_bytes(
                data[0..2]
                    .try_into()
                    .map_err(|_| CrsfParsingError::InvalidPayloadLength)?,
            ),
            vertical_speed_packed: data[2] as i8,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_baro_altitude_new() {
        let packet = BaroAltitude::new(12345, -50).unwrap();
        assert_eq!(packet.altitude_packed, 12345);
        assert_eq!(packet.vertical_speed_packed, -50);
    }

    #[test]
    fn test_altitude_packing() {
        assert_eq!(BaroAltitude::get_altitude_packed(-10000), 0);
        assert_eq!(BaroAltitude::get_altitude_packed(22766), 32766);
        assert_eq!(BaroAltitude::get_altitude_packed(-10001), 0);
        assert_eq!(BaroAltitude::get_altitude_packed(327660), 0xfffe);
        assert_eq!(BaroAltitude::get_altitude_packed(0), 10000);
        assert_eq!(BaroAltitude::get_altitude_packed(22767), 0x7FFF);
    }

    #[test]
    fn test_altitude_unpacking() {
        let baro_altitude_dm = BaroAltitude {
            altitude_packed: 0,
            vertical_speed_packed: 0,
        };
        assert_eq!(baro_altitude_dm.get_altitude_dm(), -10000);

        let baro_altitude_m = BaroAltitude {
            altitude_packed: 0x8000,
            vertical_speed_packed: 0,
        };
        assert_eq!(baro_altitude_m.get_altitude_dm(), 0);

        let baro_altitude_dm = BaroAltitude {
            altitude_packed: 10000,
            vertical_speed_packed: 0,
        };
        assert_eq!(baro_altitude_dm.get_altitude_dm(), 0);

        let baro_altitude_dm = BaroAltitude {
            altitude_packed: 0x7fff,
            vertical_speed_packed: 0,
        };
        assert_eq!(baro_altitude_dm.get_altitude_dm(), 22767);
    }

    #[test]
    fn test_vertical_speed_packing() {
        assert_eq!(BaroAltitude::get_vertical_speed_packed(0), 0);
        assert_eq!(BaroAltitude::get_vertical_speed_packed(2500), 125);
        assert_eq!(BaroAltitude::get_vertical_speed_packed(-2500), -125);
    }

    #[test]
    fn test_vertical_speed_unpacking() {
        let baro_altitude = BaroAltitude {
            altitude_packed: 0,
            vertical_speed_packed: 0,
        };
        assert_eq!(baro_altitude.get_vertical_speed_cm_s(), 0);

        let baro_altitude = BaroAltitude {
            altitude_packed: 0,
            vertical_speed_packed: 127,
        };
        assert_eq!(
            (baro_altitude.get_vertical_speed_cm_s() as f32).round(),
            2616.0
        );

        let baro_altitude = BaroAltitude {
            altitude_packed: 0,
            vertical_speed_packed: -127,
        };
        assert_eq!(
            (baro_altitude.get_vertical_speed_cm_s() as f32).round(),
            -2616.0
        );
    }

    #[test]
    fn test_baro_altitude_to_bytes() {
        let baro_altitude = BaroAltitude {
            altitude_packed: 12345,
            vertical_speed_packed: -50,
        };

        let mut buffer = [0u8; BaroAltitude::MIN_PAYLOAD_SIZE];
        baro_altitude.to_bytes(&mut buffer).unwrap();

        let expected_bytes: [u8; BaroAltitude::MIN_PAYLOAD_SIZE] = [0x30, 0x39, 0xce];

        assert_eq!(buffer, expected_bytes);
    }

    #[test]
    fn test_baro_altitude_from_bytes() {
        assert_eq!(BaroAltitude::MIN_PAYLOAD_SIZE, 3);
        let data: [u8; BaroAltitude::MIN_PAYLOAD_SIZE] = [0x30, 0x39, 0xce];

        let baro_altitude = BaroAltitude::from_bytes(&data).unwrap();

        assert_eq!(
            baro_altitude,
            BaroAltitude {
                altitude_packed: 12345,
                vertical_speed_packed: -50,
            }
        );
    }

    #[test]
    fn test_baro_altitude_round_trip() {
        let baro_altitude = BaroAltitude {
            altitude_packed: 12345,
            vertical_speed_packed: -50,
        };

        let mut buffer = [0u8; BaroAltitude::MIN_PAYLOAD_SIZE];
        baro_altitude.to_bytes(&mut buffer).unwrap();

        let round_trip_baro_altitude = BaroAltitude::from_bytes(&buffer).unwrap();

        assert_eq!(baro_altitude, round_trip_baro_altitude);
    }

    #[test]
    fn test_edge_cases() {
        let baro_altitude = BaroAltitude {
            altitude_packed: 65535,
            vertical_speed_packed: -128,
        };

        let mut buffer = [0u8; BaroAltitude::MIN_PAYLOAD_SIZE];
        baro_altitude.to_bytes(&mut buffer).unwrap();
        let round_trip_baro_altitude = BaroAltitude::from_bytes(&buffer).unwrap();
        assert_eq!(baro_altitude, round_trip_baro_altitude);
    }

    #[test]
    fn test_baro_altitude_to_bytes_buffer_too_small() {
        let baro_altitude = BaroAltitude {
            altitude_packed: 12345,
            vertical_speed_packed: -50,
        };
        let mut buffer = [0u8; 2];
        let result = baro_altitude.to_bytes(&mut buffer);
        assert_eq!(result, Err(CrsfParsingError::BufferOverflow));
    }

    #[test]
    fn test_baro_altitude_from_bytes_invalide_size() {
        let data: [u8; 1] = [0x04];
        let result = BaroAltitude::from_bytes(&data);
        assert_eq!(result, Err(CrsfParsingError::InvalidPayloadLength));
    }
}
