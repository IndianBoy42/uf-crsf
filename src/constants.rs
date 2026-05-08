//! CRSF protocol constants.
//!
//! This module defines the fundamental constants for the TBS Crossfire protocol,
//! including packet size limits, framing structure, and shared CRC calculator.

use crc::Crc;

/// Maximum CRSF packet size in bytes, including all framing.
///
/// The CRSF specification limits packets to 64 bytes total. This includes:
/// - Destination address (1 byte)
/// - Length field (1 byte)
/// - Packet type (1 byte)
/// - Payload (0-60 bytes)
/// - CRC checksum (1 byte)
///
/// # Why 64 Bytes?
///
/// The 64-byte limit was chosen to balance throughput and latency:
/// - Small enough for real-time telemetry (fast transmission over RF)
/// - Large enough for complex data (e.g., GPS coordinates, device info)
/// - Matches typical UART buffer sizes on microcontrollers
///
/// # Hardware Implications
///
/// **For receiver implementation:**
/// - Your UART RX buffer should accommodate multiple packets (e.g., 256-512 bytes)
/// - Consider DMA buffer sizing - ensure it's at least `CRSF_MAX_PACKET_SIZE`
///
/// **For transmitter implementation:**
/// - You'll need a transmit buffer of at least this size
/// - ExpressLRS typically batches telemetry into 4-8 packets per frame
///
/// **For flight controller:**
/// - Betaflight/INAV processes packets in loops - ensure you handle backpressure
/// - Don't block indefinitely when packets arrive faster than you process them
///
/// # Example: Buffer Allocation
///
/// ```ignore
/// // Good: Allocate max packet size plus headroom
/// let mut tx_buffer = [0u8; CRSF_MAX_PACKET_SIZE];
///
/// // Better: Allocate space for multiple packets
/// let mut tx_buffer = [0u8; CRSF_MAX_PACKET_SIZE * 4];
///
/// // Embedded microcontroller with DMA
/// let mut uart_rx_buffer: [u8; 512] = [0; 512]; // Fits ~8 packets
/// ```
pub const CRSF_MAX_PACKET_SIZE: usize = 64;

/// Minimum CRSF packet size in bytes, including all framing.
///
/// A valid CRSF packet must be at least 4 bytes:
/// - Destination address (1 byte)
/// - Length field (1 byte) - must be >= 2 (type + CRC)
/// - Packet type (1 byte)
/// - CRC checksum (1 byte)
///
/// The minimum payload size is 0 bytes (length field = 2), though in practice
/// most packet types have specific, non-zero payload sizes.
///
/// # Protocol Framing Diagram
///
/// ```text
/// Byte | 0       | 1          | 2          | 3...N-2      | N-1    |
///      | Address | Length     | Type       | Payload      | CRC    |
///      | 0xC8-   | Payload+2 | 0x00-0xFF  | Variable     | CRC8   |
///      | 0xEA    | (2-62)     |            |              | DVB-S2 |
/// ```
///
/// Where:
/// - **Address**: Destination device (see [`PacketAddress`])
/// - **Length**: Total bytes from Type to CRC inclusive (2-62)
/// - **Type**: Packet type identifier (see [`PacketType`])
/// - **Payload**: Variable-length data specific to packet type
/// - **CRC**: CRC-8/DVB-S2 checksum over bytes 2 to N-2
///
/// # Edge Cases
///
/// - **Length = 0**: Invalid (below minimum)
/// - **Length = 1**: Invalid (type + CRC requires 2 bytes)
/// - **Length = 2**: Valid, zero-byte payload (rare, used by some command packets)
/// - **Length > 62**: Invalid (exceeds max payload)
///
/// # Parser Behavior
///
/// The [`CrsfParser`] validates packet length and returns
/// [`CrsfStreamError::InvalidPacketLength`] when a packet falls outside
/// this range.
pub const CRSF_MIN_PACKET_SIZE: usize = 4;

pub const CRSF_SYNC_BYTE: u8 = 0xC8;

/// CRC-8/DVB-S2 calculator shared between parser and packet serializer.
///
/// This CRC is calculated over the packet type and payload bytes (not the
/// sync byte or length byte). Both the parser and the serializer use this
/// same instance to avoid duplicating the 256-entry lookup table.
pub(crate) static CRC8_DVB_S2: Crc<u8> = Crc::<u8>::new(&crc::CRC_8_DVB_S2);
