//! Error types for CRSF protocol parsing and stream processing.
//!
//! This module defines two error types:
//!
//! - [`CrsfParsingError`]: Errors that occur when parsing packet payloads into
//!   structured types
//! - [`CrsfStreamError`]: Errors that occur during stream-based packet reading,
//!   including framing, CRC, and I/O errors

#[cfg(any(feature = "embedded_io_async", feature = "embedded_io"))]
use embedded_io::ErrorKind;

/// Errors that occur when parsing CRSF packet payloads.
///
/// These errors indicate issues after successful packet framing validation
/// (correct sync byte, length, and CRC) but before successful payload
/// deserialization into specific packet types.
///
/// # Recovery Strategies
///
/// **For application developers:**
/// - Log the error details for debugging
/// - Skip unknown packet types (useful when working with devices that send
///   custom or future packet types not yet implemented in this library)
/// - For `InvalidPayload`, consider the packet corrupted and discard it
///
/// # Common Causes
///
/// | Error Variant | Common Cause | Recovery |
/// |---------------|--------------|----------|
/// | `UnexpectedPacketType` | Device sends packet type you don't expect | Log and skip |
/// | `PacketNotImlemented` | Library hasn't implemented this packet type yet | Log and skip |
/// | `InvalidPayloadLength` | Payload size doesn't match expected packet type | Discard packet |
/// | `InvalidPayload` | Payload data is malformed or out of range | Discard packet |
/// | `BufferOverflow` | Internal buffer too small for packet data | Increase buffer size (shouldn't occur in practice) |
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CrsfParsingError {
    /// The packet type byte does not match any known CRSF packet type.
    ///
    /// This typically occurs when:
    /// - A device sends a custom packet type not defined in the CRSF spec
    /// - You're parsing a packet at an address where it's not expected
    /// - The packet type byte is corrupted (unlikely after CRC validation)
    ///
    /// # When to Ignore
    ///
    /// In receiver or flight controller roles, you may receive broadcast packets
    /// with types you don't care about. It's safe to ignore these errors and
    /// continue parsing subsequent packets.
    ///
    /// # Example
    ///
    /// ```ignore
    /// match Packet::parse(&raw_packet) {
    ///     Ok(packet) => handle_packet(packet),
    ///     Err(CrsfParsingError::UnexpectedPacketType(type)) => {
    ///         // Log and skip - device might be sending custom telemetry
    ///         log::warn!("Unknown packet type: 0x{:02x}", type);
    ///     }
    ///     Err(e) => return Err(e),
    /// }
    /// ```
    UnexpectedPacketType(u8),

    /// The packet type is known but not yet implemented in this library.
    ///
    /// This indicates that the packet type exists in the CRSF specification
    /// (or is documented in ExpressLRS), but this library doesn't yet support
    /// parsing it into a structured type. The raw bytes are available via
    /// [`RawCrsfPacket::payload()`](crate::parser::RawCrsfPacket::payload) for
    /// manual parsing if needed.
    ///
    /// # Common Unimplemented Types
    ///
    /// Some packet types may be intentionally unimplemented if they are:
    /// - Deprecated or vendor-specific
    /// - Only used by specific hardware not in common use
    /// - Binary blobs that don't have a clear structure
    ///
    /// # Example
    ///
    /// ```ignore
    /// match Packet::parse(&raw_packet) {
    ///     Ok(packet) => handle_packet(packet),
    ///     Err(CrsfParsingError::PacketNotImlemented(type)) => {
    ///         // Access raw bytes for manual handling
    ///         let payload = raw_packet.payload();
    ///         handle_custom_packet(type, payload);
    ///     }
    ///     Err(e) => return Err(e),
    /// }
    /// ```
    PacketNotImlemented(u8),

    /// The payload size does not match the expected size for this packet type.
    ///
    /// Each CRSF packet type has a fixed payload size defined in the protocol
    /// specification. This error occurs when the received payload length differs
    /// from that expected size.
    ///
    /// # Common Causes
    ///
    /// - Packet corruption between sender and receiver (unlikely after CRC validation)
    /// - Device firmware bug sending incorrect payload length
    /// - Mismatch between the packet type byte and the actual payload content
    ///
    /// # Recovery
    ///
    /// Discard the packet entirely. The payload is inconsistent and cannot be
    /// safely interpreted.
    InvalidPayloadLength,

    /// The payload data contains invalid values or is malformed.
    ///
    /// This error occurs when the payload bytes are the correct length, but
    /// contain values outside valid ranges or fail consistency checks. Common
    /// examples:
    ///
    /// - GPS coordinates with invalid values
    /// - Enum values that don't map to known variants
    /// - Reserved bits set to non-zero values
    ///
    /// # Recovery
    ///
    /// Discard the packet. The data cannot be trusted for safe operation.
    InvalidPayload,

    /// An internal buffer overflow occurred during parsing.
    ///
    /// This error should never occur in practice and indicates a bug in the
    /// library or an attempt to parse a packet larger than the maximum allowed
    /// size ([`CRSF_MAX_PACKET_SIZE`](crate::constants::CRSF_MAX_PACKET_SIZE)).
    ///
    /// # When This Occurs
    ///
    /// - Packet length field exceeds 64 bytes (violates CRSF spec)
    /// - Internal buffer sizing issue (library bug - please report)
    ///
    /// # Recovery
    ///
    /// Reset the parser state and continue. If this occurs repeatedly, the
    /// sending device may be malfunctioning or sending malformed packets.
    BufferOverflow,
}

/// Errors that occur during CRSF stream packet reading.
///
/// These errors occur during the packet framing and validation phase when
/// reading bytes from a stream (UART, SPI, etc.). They indicate issues with
/// the packet structure, CRC validation, or I/O operations before payload
/// parsing.
///
/// # Recovery Strategies
///
/// **For embedded systems (no_std):**
/// - Reset the parser state via [`CrsfParser::reset()`](crate::parser::CrsfParser::reset)
/// - Continue reading subsequent bytes - the stream is self-synchronizing
/// - Log error details for debugging (use `defmt` feature for embedded logging)
///
/// **For application-layer handling:**
/// - `InvalidSync`: Normal when data stream is corrupted or contains non-CRSF data
/// - `InvalidCrc`: Packet corrupted in transit - transmitter/receiver link issue
/// - `Io` / `UnexpectedEof`: Communication channel problem - check hardware connection
///
/// # Common Causes by Role
///
/// | Role | Common Errors | Typical Causes |
/// |------|---------------|----------------|
/// | **Flight Controller** | `InvalidCrc` | RF noise, weak link, antenna issues |
/// | **Receiver** | `InvalidSync` | UART baud rate mismatch, electrical noise |
/// | **Transmitter** | `UnexpectedEof` | UART FIFO overflow, buffer underrun |
/// | **Handset App** | `Io` | USB/serial disconnection, driver issues |
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CrsfStreamError {
    /// The packet length field indicates an invalid packet size.
    ///
    /// The CRSF specification requires packets to be 4-64 bytes. This error
    /// occurs when the length byte (2nd byte in the frame) specifies a size
    /// outside this range.
    ///
    /// # Common Causes
    ///
    /// - Data corruption in the length byte
    /// - UART framing error causing byte misalignment
    /// - Non-CRSF data on the serial line
    /// - Device firmware bug
    ///
    /// # Recovery
    ///
    /// The parser automatically resets to `AwaitingSync` state. Continue
    /// reading subsequent bytes - the protocol will resynchronize when a
    /// valid sync byte (device address) is encountered.
    InvalidPacketLength(u8),

    /// The sync byte (first byte) is not a valid CRSF device address.
    ///
    /// CRSF packets begin with a destination address byte (0xC8-0xEA for
    /// standard devices). This error occurs when the first byte of a packet
    /// is not in this range.
    ///
    /// # Common Causes
    ///
    /// - Non-CRSF data on the serial line (e.g., debug output from other components)
    /// - UART baud rate mismatch causing bit alignment errors
    /// - Electrical noise flipping bits
    /// - Data stream corrupted or interrupted
    ///
    /// # When This Is Normal
    ///
    /// During startup or when connecting to a device, you may receive
    /// spurious bytes before valid CRSF data begins. This is expected behavior
    /// - continue reading until sync is achieved.
    ///
    /// # Recovery
    ///
    /// The parser remains in `AwaitingSync` state. Simply continue feeding
    /// bytes - the first valid device address will start packet parsing.
    InvalidSync(u8),

    /// The CRC check failed - the packet is corrupted.
    ///
    /// CRSF uses CRC-8/DVB-S2 for packet integrity verification. This error
    /// occurs when the calculated CRC doesn't match the CRC byte in the packet.
    ///
    /// # Fields
    ///
    /// - `calculated_crc`: The CRC computed from the packet payload
    /// - `packet_crc`: The CRC byte received in the packet
    ///
    /// # Common Causes
    ///
    /// - RF interference causing bit errors during wireless transmission
    /// - Weak RF link with high packet loss
    /// - Electrical noise on wired serial connections
    /// - Overrun/underrun in UART FIFO causing dropped or duplicated bytes
    ///
    /// # Diagnostic Value
    ///
    /// Compare `calculated_crc` and `packet_crc`:
    /// - If they differ by a single bit: likely single-bit error from noise
    /// - If completely different: severe corruption or framing error
    ///
    /// # Recovery
    ///
    /// The parser automatically resets to `AwaitingSync` state. Continue
    /// reading - corrupted packets are dropped but subsequent valid packets
    /// will parse successfully.
    ///
    /// # Hardware-Specific Guidance
    ///
    /// **STM32/ARM Cortex-M:**
    /// - Ensure UART FIFOs are properly configured
    /// - Check for DMA transfer issues (circular buffer overflow)
    /// - Verify hardware flow control (RTS/CTS) if available
    ///
    /// **ESP32/ESP8266:**
    /// - Increase UART RX buffer size in menuconfig
    /// - Check for WiFi coexistence issues causing timing problems
    ///
    /// **RP2040:**
    /// - Ensure PIO or UART is configured with correct baud rate tolerance
    /// - Check for clock domain issues when reading from different cores
    InvalidCrc { calculated_crc: u8, packet_crc: u8 },

    /// The packet type byte doesn't match the expected type for this context.
    ///
    /// This occurs when using [`Packet::parse()`](crate::packets::Packet::parse)
    /// with a type filter (if implemented) or when the packet type doesn't
    /// match expectations based on the destination address.
    ///
    /// # Common Causes
    ///
    /// - Parsing a packet at an address where you expect a specific type
    /// - Device sends unexpected packet type (firmware behavior change)
    /// - Packet type byte corrupted (though CRC should catch this)
    ///
    /// # Recovery
    ///
    /// Use [`Packet::parse()`](crate::packets::Packet::parse) without type
    /// constraints, or handle this error and skip the packet if you're only
    /// interested in specific packet types.
    UnexpectedPacketType(u8),

    /// An error occurred while parsing the packet payload.
    ///
    /// Wraps a [`CrsfParsingError`] that occurred during payload deserialization
    /// after successful framing validation.
    ///
    /// See [`CrsfParsingError`] documentation for detailed recovery strategies
    /// based on the specific error variant.
    ParsingError(CrsfParsingError),

    /// The internal input buffer is too small to hold the current data.
    ///
    /// This error occurs when using the I/O abstractions
    /// ([`BlockingCrsfReader`] or [`AsyncCrsfReader`]) and the internal
    /// `heapless::Deque` cannot hold additional bytes before parsing.
    ///
    /// # Common Causes
    ///
    /// - Reading faster than parsing can process packets
    /// - Packets arrive at a higher rate than your application can handle
    /// - Malformed packet preventing synchronization (parser keeps buffering)
    ///
    /// # Recovery
    ///
    /// Increase the buffer size by modifying the
    /// `BLOCKING_IO_BUFFER_SIZE` or `ASYNC_IO_BUFFER_SIZE` constants, or
    /// reset the parser and discard buffered data if synchronization appears
    /// stuck.
    InputBufferTooSmall,

    /// An I/O error occurred reading from the underlying stream.
    ///
    /// This variant is only available when the `embedded_io` or
    /// `embedded_io_async` features are enabled.
    ///
    /// # Common Causes
    ///
    /// - UART hardware failure or disconnection
    /// - USB/serial port unplugged (handset applications)
    /// - Driver or peripheral error (embedded systems)
    /// - Timeout when using async I/O with timeout
    ///
    /// # Recovery
    ///
    /// Check the specific [`ErrorKind`] for guidance:
    /// - `Interrupted`: Retry the operation
    /// - `WouldBlock` (async only): Try again later
    /// - `Other`: Check hardware, reset peripheral, or abort
    ///
    /// # Hardware-Specific Guidance
    ///
    /// **STM32 (HAL):**
    /// - Check UART overrun flag (ORE)
    /// - Verify DMA transfer errors
    /// - Ensure USART clock is enabled
    ///
    /// **nRF52/53:**
    /// - Check EasyDMA buffer alignment requirements
    /// - Verify UART interrupt priority levels
    ///
    /// **RP2040:**
    /// - Check for UART RX FIFO overrun
    /// - Verify GPIO pin configuration (TX/RX swapped?)
    ///
    /// **Handset/Desktop (serial port):**
    /// - Verify USB driver is loaded
    /// - Check cable connections
    /// - Ensure correct port selected
    #[cfg(any(feature = "embedded_io_async", feature = "embedded_io"))]
    Io(ErrorKind),

    /// The underlying stream ended unexpectedly while reading a packet.
    ///
    /// This occurs when the read operation returns 0 bytes (EOF) before a
    /// complete packet is received.
    ///
    /// # Common Causes
    ///
    /// - Device disconnected or powered off
    /// - Serial port closed (handset applications)
    /// - UART FIFO drain during read (embedded systems)
    /// - Peripheral reset or hardware fault
    ///
    /// # Recovery
    ///
    /// The connection is likely terminated. Reinitialize the peripheral,
    /// reconnect to the device, or abort the operation.
    ///
    /// # Hardware-Specific Guidance
    ///
    /// **STM32/ARM:**
    /// - Check if UART peripheral is still enabled
    /// - Verify GPIO pins are still configured correctly
    /// - Look for brownout or power issues on the connected device
    ///
    /// **Handset/Desktop:**
    /// - The USB/serial device was unplugged
    /// - The connection was closed by the OS or driver
    /// - Attempt to reopen the serial port
    #[cfg(any(feature = "embedded_io_async", feature = "embedded_io"))]
    UnexpectedEof,
}

impl From<CrsfParsingError> for CrsfStreamError {
    fn from(e: CrsfParsingError) -> Self {
        CrsfStreamError::ParsingError(e)
    }
}

#[cfg(test)]
mod tests {
    // Test utilities for error documentation examples
    extern crate std;

    use super::*;

    #[test]
    fn test_error_display() {
        // Verify all error variants can be created and used
        let parsing_err = CrsfParsingError::UnexpectedPacketType(0x10);
        let stream_err = CrsfStreamError::from(parsing_err);
        assert!(matches!(stream_err, CrsfStreamError::ParsingError(_)));
    }
}
