use crate::{
    constants,
    error::CrsfStreamError,
    packets::{Packet, PacketAddress},
};
use crc::Crc;
use num_enum::TryFromPrimitive;

/// Parser state machine for CRSF packet parsing.
///
/// The parser cycles through these states as it processes a byte stream:
///
/// ```text
/// AwaitingSync -> AwaitingLength -> Reading(n) -> AwaitingCrc -> (complete) -> AwaitingSync
///        |               ^                                    |
///        |               | (invalid length)                  | (CRC error)
///        +---------------+------------------------------------+
///        (invalid sync byte)
/// ```
///
/// The parser automatically resets to `AwaitingSync` on any error,
/// allowing it to resynchronize with a corrupted byte stream.
#[derive(Debug, Default, Ord, PartialOrd, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum State {
    /// Waiting for a valid sync byte (device address).
    ///
    /// The parser stays in this state until it receives a byte in the range
    /// 0xC8-0xEA (valid CRSF device addresses).
    ///
    /// # Error Handling
    ///
    /// Any byte outside this range results in
    /// [`CrsfStreamError::InvalidSync`], but the parser remains in this state
    /// (it doesn't reset). This allows it to skip past non-CRSF data in the
    /// stream.
    #[default]
    AwaitingSync,
    /// Waiting for the packet length byte.
    ///
    /// After a valid sync byte, the next byte specifies the packet length.
    /// The parser validates that this length is within the CRSF limits
    /// (2-62 for payload length, or 4-64 total packet size).
    ///
    /// # Error Handling
    ///
    /// If the length is invalid, the parser returns
    /// [`CrsfStreamError::InvalidPacketLength`] and resets to
    /// `AwaitingSync`.
    AwaitingLength,
    /// Reading payload bytes.
    ///
    /// The `n` parameter indicates the total number of bytes to read
    /// (excluding the sync and length bytes). This includes the packet type,
    /// payload, and CRC.
    ///
    /// The parser accumulates bytes in an internal buffer until it has read
    /// `n` bytes, then transitions to `AwaitingCrc`.
    Reading(usize),
    /// Waiting for the CRC byte (final byte of the packet).
    ///
    /// The parser has received all bytes except the CRC. When the next byte
    /// arrives, it validates the CRC and either:
    /// - Returns the complete packet (if CRC is valid)
    /// - Returns [`CrsfStreamError::InvalidCrc`] and resets (if CRC is invalid)
    AwaitingCrc,
}

/// CRSF packet parser for processing raw byte streams.
///
/// `CrsfParser` is a state machine that parses incoming byte sequences from
/// UART, SPI, or other serial interfaces into validated CRSF packets. It
/// handles packet framing, CRC validation, and stream resynchronization.
///
/// # Lifecycle
///
/// The parser is stateful and maintains an internal buffer for accumulating
/// packet bytes:
///
/// ```text
/// 1. Create:  CrsfParser::new()  -> parser in AwaitingSync state
/// 2. Feed:    parser.push_byte(byte)  -> returns Option<Packet> or Error
/// 3. Repeat:  Continue feeding bytes until packets are received
/// 4. Reset:   parser.reset()  -> manually reset to AwaitingSync state
/// ```
///
/// The parser is designed to be called repeatedly from UART ISRs, DMA
/// completion handlers, or main loop polling. It's safe to call
/// `push_byte()` multiple times per iteration.
///
/// # Thread Safety
///
/// `CrsfParser` is **not** thread-safe internally. If used in a multi-core
/// or multi-threaded context, you must:
/// - Use a mutex or lock around the parser
/// - Or designate a single core/thread to handle UART RX and parsing
/// - Use double-buffering: one thread collects bytes, another parses
///
/// # Error Recovery
///
/// The parser automatically resets to `AwaitingSync` state on any error:
/// - Invalid sync byte → continues scanning for valid address
/// - Invalid packet length → resets, looks for next sync
/// - CRC mismatch → resets, discards corrupted packet
///
/// This self-synchronizing behavior means you can continue feeding bytes
/// after any error without manual intervention.
///
/// # Usage Patterns
///
/// ## Pattern 1: Byte-by-byte (ISR/DMA)
///
/// ```ignore
/// use uf_crsf::{parser::CrsfParser, packets::Packet};
///
/// let mut parser = CrsfParser::new();
///
/// // In UART ISR or DMA completion handler:
/// uart_interrupt_handler() {
///     while let Some(byte) = uart_rx_fifo.pop() {
///         match parser.push_byte(byte) {
///             Ok(Some(Packet::LinkStatistics(stats))) => {
///                 // Process telemetry
///                 update_telemetry(stats);
///             }
///             Ok(Some(packet)) => {
///                 // Handle other packet types
///             }
///             Ok(None) => {
///                 // Packet not yet complete, continue
///             }
///             Err(e) => {
///                 // Log and continue - parser auto-resets
///                 log::warn!("Parse error: {:?}", e);
///             }
///         }
///     }
/// }
/// ```
///
/// ## Pattern 2: Buffer Iteration (Main Loop)
///
/// ```ignore
/// use uf_crsf::{parser::CrsfParser, packets::Packet};
///
/// let mut parser = CrsfParser::new();
/// let mut rx_buffer = [0u8; 256];
///
/// main_loop() {
///     let bytes_read = uart.read(&mut rx_buffer).unwrap();
///
///     // Parse all packets in buffer
///     for result in parser.iter_packets(&rx_buffer[..bytes_read]) {
///         match result {
///             Ok(Packet::RcChannelsPacked(channels)) => {
///                 update_mixer(channels.channels);
///             }
///             Ok(_) => {}
///             Err(e) => log::warn!("Parse error: {:?}", e),
///         }
///     }
/// }
/// ```
///
/// ## Pattern 3: Mixed Raw/Parsed Processing
///
/// ```ignore
/// use uf_crsf::{parser::CrsfParser, packets::Packet};
///
/// let mut parser = CrsfParser::new();
///
/// for byte in uart_stream {
///     match parser.push_byte_raw(byte) {
///         Ok(Some(raw_packet)) => {
///             // Fast path: check packet type before full parsing
///             match raw_packet.raw_packet_type() {
///                 0x16 => {
///                     // RC channels - parse quickly
///                     if let Ok(Packet::RcChannelsPacked(ch)) = Packet::parse(&raw_packet) {
///                         process_rc(ch);
///                     }
///                 }
///                 _ => {
///                     // Parse other types normally
///                     if let Ok(packet) = Packet::parse(&raw_packet) {
///                         process_packet(packet);
///                     }
///                 }
///             }
///         }
///         Ok(None) => {}
///         Err(e) => log::warn!("Parse error: {:?}", e),
///     }
/// }
/// ```
///
/// # Performance Considerations
///
/// - **Zero allocation**: No heap usage, suitable for no_std environments
/// - **Zero-copy for raw packets**: [`RawCrsfPacket`] is a view into the buffer
/// - **State machine overhead**: Minimal (enum + index + buffer)
/// - **CRC calculation**: Uses efficient lookup-table-based CRC-8/DVB-S2
///
/// Typical performance on STM32F4 @ 168MHz:
/// - ~1-2μs to process a single byte via `push_byte()`
/// - ~10-20μs to parse a complete 14-byte telemetry packet
/// - ~50-100μs to process a burst of 8 packets
///
/// # Memory Usage
///
/// - **Stack**: ~72 bytes (struct + state + buffer)
/// - **Heap**: 0 bytes (no allocation)
/// - **Static**: None (parser is stack-allocated)
///
/// # Hardware-Specific Tips
///
/// **STM32 with DMA:**
/// - Use circular DMA buffer + parse from main loop
/// - Or use UART RXNE interrupt for byte-by-byte processing
/// - Ensure DMA transfer complete interrupt priority is high enough
///
/// **RP2040 with PIO:**
/// - Use PIO to capture UART data into a buffer
/// - Parse from main loop or second core
/// - Watch for buffer underrun if processing too slowly
///
/// **ESP32:**
/// - Use UART ISR with FIFO threshold interrupt
/// - Parse in ISR for lowest latency, or in task for simplicity
/// - Watch for stack overflow in FreeRTOS task
#[derive(Debug)]
pub struct CrsfParser {
    /// Internal buffer for accumulating packet bytes.
    ///
    /// Sized to hold the maximum CRSF packet (64 bytes).
    buffer: [u8; constants::CRSF_MAX_PACKET_SIZE],
    /// Current parser state.
    state: State,
    /// Current write position in the buffer.
    position: usize,
}

/// CRC-8/DVB-S2 polynomial for CRSF packet integrity checking.
///
/// This CRC is calculated over the packet type and payload bytes (not the
/// sync byte or length byte). The calculated CRC must match the final byte
/// of the packet for the packet to be valid.
const CRC8_DVB_S2: Crc<u8> = Crc::<u8>::new(&crc::CRC_8_DVB_S2);

impl CrsfParser {
    /// Creates a new parser in `AwaitingSync` state.
    ///
    /// The parser is ready to receive bytes immediately after construction.
    /// No additional initialization is required.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use uf_crsf::parser::CrsfParser;
    ///
    /// let mut parser = CrsfParser::new();
    /// // Parser is now ready to receive bytes
    /// ```
    pub fn new() -> Self {
        Self {
            buffer: [0; constants::CRSF_MAX_PACKET_SIZE],
            state: State::AwaitingSync,
            position: 0,
        }
    }

    /// Feeds a single byte to the parser and returns a raw packet if complete.
    ///
    /// This method processes one byte at a time through the state machine.
    /// Use this when you have byte-level access to the data stream (e.g., from
    /// a UART ISR or DMA callback).
    ///
    /// # Returns
    ///
    /// - `Ok(Some(raw_packet))`: A complete packet was received and validated
    /// - `Ok(None)`: More bytes are needed to complete the current packet
    /// - `Err(e)`: An error occurred (see [`CrsfStreamError`])
    ///
    /// # When to Use vs. `push_byte()`
    ///
    /// Use `push_byte_raw()` when you want to:
    /// - Inspect the packet type before full parsing
    /// - Handle unknown or custom packet types manually
    /// - Access the raw payload bytes without deserialization
    ///
    /// Use [`push_byte()`] when you want a fully parsed [`Packet`] enum.
    ///
    /// # Example: Early Packet Type Filtering
    ///
    /// ```ignore
    /// use uf_crsf::{parser::CrsfParser, packets::Packet};
    ///
    /// let mut parser = CrsfParser::new();
    ///
    /// for byte in uart_stream {
    ///     match parser.push_byte_raw(byte) {
    ///         Ok(Some(raw_packet)) => {
    ///             // Check packet type before full parsing
    ///             match raw_packet.raw_packet_type() {
    ///                 0x16 => {
    ///                     // RC channels - high priority, parse quickly
    ///                     if let Ok(Packet::RcChannelsPacked(ch)) = Packet::parse(&raw_packet) {
    ///                         update_mixer(ch.channels);
    ///                     }
    ///                 }
    ///                 0x14 => {
    ///                     // Link statistics - lower priority
    ///                     if let Ok(Packet::LinkStatistics(ls)) = Packet::parse(&raw_packet) {
    ///                         update_telemetry(ls);
    ///                     }
    ///                 }
    ///                 _ => {
    ///                     // Parse other packet types
    ///                     if let Ok(packet) = Packet::parse(&raw_packet) {
    ///                         handle_packet(packet);
    ///                     }
    ///                 }
    ///             }
    ///         }
    ///         Ok(None) => {}
    ///         Err(e) => log::warn!("Parse error: {:?}", e),
    ///     }
    /// }
    /// ```
    pub fn push_byte_raw(
        &mut self,
        byte: u8,
    ) -> Result<Option<RawCrsfPacket<'_>>, CrsfStreamError> {
        match self.state {
            State::AwaitingSync => {
                if PacketAddress::try_from_primitive(byte).is_ok() {
                    self.position = 0;
                    self.buffer[self.position] = byte;
                    self.state = State::AwaitingLength;
                    Ok(None)
                } else {
                    self.state = State::AwaitingSync;
                    Err(CrsfStreamError::InvalidSync(byte))
                }
            }
            State::AwaitingLength => {
                let n = byte as usize + 2;

                if !(constants::CRSF_MIN_PACKET_SIZE..constants::CRSF_MAX_PACKET_SIZE).contains(&n)
                {
                    self.reset();
                    // A false sync can make this "length" byte invalid. Re-evaluate
                    // the same byte as a new sync candidate so it is not lost.
                    self.try_start_frame_from_sync(byte);
                    return Err(CrsfStreamError::InvalidPacketLength(byte));
                }
                self.position = 1;
                self.buffer[self.position] = byte;
                self.state = State::Reading(n - 1);
                Ok(None)
            }
            State::Reading(n) => {
                self.position += 1;
                self.buffer[self.position] = byte;
                if self.position == n - 1 {
                    self.state = State::AwaitingCrc;
                }
                Ok(None)
            }
            State::AwaitingCrc => {
                self.position += 1;
                self.buffer[self.position] = byte;

                let mut digest = CRC8_DVB_S2.digest();
                digest.update(&self.buffer[2..self.position]);
                let calculated_crc = digest.finalize();
                let packet_crc = self.buffer[self.position];

                if calculated_crc != packet_crc {
                    self.reset();
                    // Preserve this byte as potential sync for the next frame.
                    self.try_start_frame_from_sync(byte);
                    return Err(CrsfStreamError::InvalidCrc {
                        calculated_crc,
                        packet_crc,
                    });
                }
                let start = 0;
                let end = self.position + 1;
                self.reset();
                let bytes = &self.buffer[start..end];
                match RawCrsfPacket::new(bytes) {
                    None => Err(CrsfStreamError::InputBufferTooSmall),
                    Some(packet) => Ok(Some(packet)),
                }
            }
        }
    }

    /// Creates an iterator for parsing multiple packets from a byte buffer.
    ///
    /// This is the preferred method when you have a buffer of bytes (e.g., from
    /// a DMA transfer or UART read) and want to parse all packets in a single
    /// operation.
    ///
    /// # Returns
    ///
    /// An iterator over [`Result<Packet, CrsfStreamError>].
    ///
    /// # Lifecycle
    ///
    /// The iterator borrows the parser mutably, so you cannot access the
    /// parser directly while the iterator is alive. The iterator advances the
    /// parser's internal state as it processes bytes.
    ///
    /// # Example: Main Loop Processing
    ///
    /// ```ignore
    /// use uf_crsf::{parser::CrsfParser, packets::Packet};
    ///
    /// let mut parser = CrsfParser::new();
    /// let mut rx_buffer = [0u8; 256];
    ///
    /// main_loop() {
    ///     // Read data from UART
    ///     let bytes_read = uart.read(&mut rx_buffer).unwrap();
    ///
    ///     // Parse all packets in buffer
    ///     for result in parser.iter_packets(&rx_buffer[..bytes_read]) {
    ///         match result {
    ///             Ok(Packet::RcChannelsPacked(channels)) => {
    ///                 update_mixer(channels.channels);
    ///             }
    ///             Ok(Packet::LinkStatistics(stats)) => {
    ///                 update_telemetry(stats);
    ///             }
    ///             Ok(_) => {} // Ignore other packets
    ///             Err(e) => {
    ///                 // Log and continue - parser auto-resets
    ///                 log::warn!("Parse error: {:?}", e);
    ///             }
    ///         }
    ///     }
    /// }
    /// ```
    ///
    /// # Performance
    ///
    /// This method is efficient because:
    /// - No copying of buffer data
    /// - Parser processes bytes sequentially
    /// - Iterator yields results as packets are completed
    ///
    /// # Error Handling
    ///
    /// Errors are yielded as `Err` items in the iterator. The parser
    /// automatically resets, so you can continue processing after an error:
    ///
    /// ```ignore
    /// for result in parser.iter_packets(&buffer) {
    ///     match result {
    ///         Ok(packet) => handle_packet(packet),
    ///         Err(CrsfStreamError::InvalidSync(_)) => {
    ///             // Normal - skip non-CRSF bytes
    ///         }
    ///         Err(e) => log::warn!("Parse error: {:?}", e),
    ///     }
    /// }
    /// ```
    pub fn iter_packets<'a, 'b>(&'a mut self, buffer: &'b [u8]) -> PacketIterator<'a, 'b> {
        PacketIterator {
            parser: self,
            buffer,
            pos: 0,
        }
    }

    /// Feeds a single byte to the parser and returns a parsed packet if complete.
    ///
    /// This is the most common entry point for parsing CRSF packets. It
    /// processes one byte at a time through the state machine and returns a
    /// fully parsed [`Packet`] enum when a complete packet is received.
    ///
    /// # Returns
    ///
    /// - `Ok(Some(packet))`: A complete packet was received and parsed
    /// - `Ok(None)`: More bytes are needed to complete the current packet
    /// - `Err(e)`: An error occurred (see [`CrsfStreamError`])
    ///
    /// # Example: ISR-Based Processing
    ///
    /// ```ignore
    /// use uf_crsf::{parser::CrsfParser, packets::Packet};
    ///
    /// let mut parser = CrsfParser::new();
    ///
    /// // In UART RX interrupt handler:
    /// uart_interrupt_handler() {
    ///     while let Some(byte) = uart_rx_fifo.pop() {
    ///         match parser.push_byte(byte) {
    ///             Ok(Some(Packet::RcChannelsPacked(channels))) => {
    ///                 // High-priority: Update RC channels immediately
    ///                 update_mixer(channels.channels);
    ///             }
    ///             Ok(Some(Packet::LinkStatistics(stats))) => {
    ///                 // Lower priority: Update telemetry
    ///                 update_telemetry(stats);
    ///             }
    ///             Ok(Some(packet)) => {
    ///                 // Handle other packet types
    ///                 handle_packet(packet);
    ///             }
    ///             Ok(None) => {
    ///                 // Packet not yet complete, continue
    ///             }
    ///             Err(e) => {
    ///                 // Log and continue - parser auto-resets
    ///                 log::warn!("Parse error: {:?}", e);
    ///             }
    ///         }
    ///     }
    /// }
    /// ```
    ///
    /// # Comparison with `push_byte_raw()`
    ///
    /// | Method | Return Type | Use Case |
    /// |--------|-------------|----------|
    /// | `push_byte()` | `Option<Packet>` | Most common, fully parsed packets |
    /// | `push_byte_raw()` | `Option<RawCrsfPacket>` | Access raw bytes, custom parsing, type filtering |
    pub fn push_byte(&mut self, byte: u8) -> Result<Option<Packet>, CrsfStreamError> {
        match self.push_byte_raw(byte) {
            Ok(Some(raw_packet)) => match Packet::parse(&raw_packet) {
                Ok(packet) => Ok(Some(packet)),
                Err(e) => Err(CrsfStreamError::ParsingError(e)),
            },
            Ok(None) => Ok(None),
            Err(e) => Err(e),
        }
    }

    /// Resets the parser to `AwaitingSync` state.
    ///
    /// This method is called automatically on error, but you may also call it
    /// manually in certain scenarios:
    ///
    /// - When switching data sources (e.g., disconnecting/reconnecting UART)
    /// - When the byte stream is known to be corrupted beyond recovery
    /// - When restarting communication after a timeout
    ///
    /// # Example: Manual Reset on Disconnect
    ///
    /// ```ignore
    /// use uf_crsf::parser::CrsfParser;
    ///
    /// let mut parser = CrsfParser::new();
    ///
    /// uart_disconnected() {
    ///     // Clear parser state
    ///     parser.reset();
    ///
    ///     // Reinitialize UART
    ///     uart.reinitialize();
    ///
    ///     // Parser is ready to receive bytes again
    /// }
    /// ```
    ///
    /// # Note
    ///
    /// Resetting the parser clears the internal buffer and position. Any
    /// partially received packet will be lost. Only call this when you're
    /// sure you want to discard the current packet state.
    pub fn reset(&mut self) {
        self.position = 0;
        self.state = State::AwaitingSync;
    }

    fn try_start_frame_from_sync(&mut self, byte: u8) -> bool {
        if PacketAddress::try_from_primitive(byte).is_ok() {
            self.position = 0;
            self.buffer[self.position] = byte;
            self.state = State::AwaitingLength;
            true
        } else {
            false
        }
    }
}

impl Default for CrsfParser {
    fn default() -> Self {
        Self::new()
    }
}

/// Represents a valid, but unparsed, CRSF packet.
///
/// This struct is a zero-copy view into a byte buffer that has been validated
/// to contain a complete CRSF packet, including the sync byte, length, type,
/// payload, and CRC. It provides methods to access the different parts of the
/// packet without parsing the payload itself.
#[derive(Debug, Clone, Ord, PartialOrd, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawCrsfPacket<'a> {
    bytes: &'a [u8],
}

impl<'a> RawCrsfPacket<'a> {
    /// Creates a new `RawCrsfPacket` from a byte slice.
    ///
    /// Returns `None` if the slice is shorter than the minimum possible
    /// CRSF packet length (4 bytes).
    pub fn new(bytes: &'a [u8]) -> Option<Self> {
        if bytes.len() >= 4 {
            Some(Self { bytes })
        } else {
            None
        }
    }

    /// Returns the destination address byte of the packet.
    pub fn dst_addr(&self) -> u8 {
        self.bytes[0]
    }

    /// Returns the raw packet type byte.
    pub fn raw_packet_type(&self) -> u8 {
        self.bytes[2]
    }

    /// Returns a slice representing the packet's payload.
    ///
    /// The payload does not include the CRSF framing (destination, size, type, CRC).
    pub fn payload(&self) -> &[u8] {
        &self.bytes[3..self.bytes.len() - 1]
    }

    /// Returns the CRC check byte of the packet.
    #[expect(clippy::missing_panics_doc, reason = "infallible")]
    pub fn crc(&self) -> u8 {
        *self.bytes.last().expect("infallible due to length check")
    }

    /// Returns the total length of the packet in bytes, including the framing.
    pub fn len(&self) -> usize {
        self.bytes.len()
    }

    /// Returns `true` if the packet has a length of zero.
    ///
    /// Note: A valid CRSF packet should not be empty.
    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }
}

pub struct PacketIterator<'a, 'b> {
    parser: &'a mut CrsfParser,
    buffer: &'b [u8],
    pos: usize,
}

impl Iterator for PacketIterator<'_, '_> {
    type Item = Result<Packet, CrsfStreamError>;

    fn next(&mut self) -> Option<Self::Item> {
        while self.pos < self.buffer.len() {
            let byte = self.buffer[self.pos];
            self.pos += 1;

            match self.parser.push_byte(byte) {
                Ok(Some(packet)) => return Some(Ok(packet)),
                Ok(None) => (),
                Err(err) => return Some(Err(err)),
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::packets::{
        write_packet_to_buffer, CrsfPacket, LinkStatistics, PacketAddress, PacketType,
        RcChannelsPacked,
    };

    #[test]
    fn test_construction() {
        let raw_bytes: [u8; 14] = [0xC8, 12, 0x14, 16, 19, 99, 151, 1, 2, 3, 8, 88, 148, 252];
        let mut parser = CrsfParser::new();

        for b in &raw_bytes[0..raw_bytes.len() - 1] {
            let result = parser.push_byte_raw(*b);
            assert!(matches!(result, Ok(None)));
        }

        let raw_packet_result = parser.push_byte_raw(raw_bytes[13]);
        let raw_packet = raw_packet_result
            .expect("Failed to get raw packet result")
            .expect("Expected a complete raw packet, but got None");

        assert_eq!(raw_packet.len(), raw_bytes.len());

        assert_eq!(raw_packet.payload().len(), LinkStatistics::MIN_PAYLOAD_SIZE);
        assert_eq!(
            raw_packet.raw_packet_type(),
            PacketType::LinkStatistics as u8
        );

        let data = &raw_packet.payload();
        let ls = LinkStatistics::from_bytes(data).unwrap();
        let p = Packet::parse(&raw_packet).unwrap();
        assert_eq!(Packet::LinkStatistics(ls.clone()), p);

        assert_eq!(ls.uplink_rssi_1, 16);
    }

    #[test]
    fn test_parsing() {
        let raw_bytes: [u8; 40] = [
            0xC8, 12, 0x14, 16, 19, 99, 151, 1, 2, 3, 8, 88, 148, 252, 0xC8, 24, 0x16, 0xE0, 0x03,
            0x1F, 0x58, 0xC0, 0x07, 0x16, 0xB0, 0x80, 0x05, 0x2C, 0x60, 0x01, 0x0B, 0xF8, 0xC0,
            0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 103,
        ];
        let mut parser = CrsfParser::new();
        let results: std::vec::Vec<Result<Packet, CrsfStreamError>> =
            parser.iter_packets(&raw_bytes).collect();

        let expected = [
            992, 992, 352, 992, 352, 352, 352, 352, 352, 352, 992, 992, 0, 0, 0, 0,
        ];
        assert_eq!(results.len(), 2);

        assert!(results[0].is_ok());
        assert!(results[1].is_ok());
        assert_eq!(
            Packet::RCChannels(RcChannelsPacked { channels: expected }),
            results[1].clone().ok().unwrap()
        );
    }

    #[test]
    fn test_raw_to_full_packet_conversion() {
        let link_stats_packet = LinkStatistics {
            uplink_rssi_1: 16,
            uplink_rssi_2: 19,
            uplink_link_quality: 99,
            uplink_snr: 51 as i8,
            active_antenna: 1,
            rf_mode: 2,
            uplink_tx_power: 3,
            downlink_rssi: 8,
            downlink_link_quality: 88,
            downlink_snr: 48 as i8,
        };

        // Serialize it into a buffer
        let mut buffer = [0u8; 64];
        let bytes_written = write_packet_to_buffer(
            &mut buffer,
            PacketAddress::FlightController,
            &link_stats_packet,
        )
        .unwrap();
        let raw_bytes = &buffer[..bytes_written];

        let mut parser = CrsfParser::new();

        // 1. Parse raw bytes to get a RawCrsfPacket
        let mut raw_packet_result = Ok(None);
        for &byte in raw_bytes {
            raw_packet_result = parser.push_byte_raw(byte);
            if let Ok(Some(_)) = &raw_packet_result {
                break;
            }
        }
        let raw_packet = raw_packet_result
            .expect("Failed to get raw packet result")
            .expect("Expected a complete raw packet, but got None");

        let packet = Packet::parse(&raw_packet).expect("Failed to parse raw packet into a Packet");

        // Verify the resulting packet
        assert!(matches!(packet, Packet::LinkStatistics(_)));
        if let Packet::LinkStatistics(stats) = packet {
            assert_eq!(stats, link_stats_packet)
        }
    }
}
