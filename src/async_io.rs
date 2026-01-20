//! Async I/O abstractions for CRSF packet reading and writing.
//!
//! This module provides [`AsyncCrsfReader`] and [`write_packet`] for working
//! with asynchronous `embedded_io_async::Read` and `embedded_io_async::Write`
//! streams.
//!
//! # When to Use
//!
//! Use this module when:
//! - Building async firmware (e.g., embassy, RTIC)
//! - Working with async UART/serial abstractions
//! - Implementing timeout/ cancellation behavior
//! - Using cooperative multitasking or async executors
//!
//! # For Blocking Support
//!
//! If you don't need async I/O, use [`blocking_io`](crate::blocking_io) module
//! instead (requires `embedded_io` feature).
//!
//! # Compatibility
//!
//! This module works with any implementation of `embedded_io_async` traits:
//! - Embassy UART/USB drivers
//! - Custom async UART implementations
//! - Async serial/TCI socket wrappers

use crate::error::CrsfStreamError;
use crate::packets::{write_packet_to_buffer, CrsfPacket, Packet, PacketAddress};
use crate::parser::CrsfParser;
use embedded_io_async::{Error, Write};
use heapless::Deque;

/// Size of the internal input buffer for [`AsyncCrsfReader`].
///
/// Sized to hold 2 complete CRSF packets, providing headroom for burst reads
/// and handling cases where a partial packet remains in buffer after a
/// complete packet is parsed.
///
/// If you experience [`CrsfStreamError::InputBufferTooSmall`] errors in
/// production, increase this constant and recompile the library.
const ASYNC_IO_BUFFER_SIZE: usize = crate::constants::CRSF_MAX_PACKET_SIZE * 2;

/// Async CRSF packet reader for `embedded_io_async::Read` streams.
///
/// This type provides a convenient abstraction for asynchronously reading
/// complete CRSF packets from an async I/O stream (e.g., async UART,
/// async TCP socket). It handles:
///
/// - Buffering incoming bytes
/// - Parsing packet framing
/// - Validating CRC checksums
/// - Returning fully parsed packets
///
/// # Architecture
///
/// The reader maintains an internal buffer that decouples underlying async
/// read operations from packet parsing:
///
/// ```text
/// Async Stream -> Read -> Input Buffer -> Parser -> Packet
///                 await    (128b)        (64b)    (Enum)
/// ```
///
/// This design allows:
/// - Larger reads from the stream (more efficient)
/// - Packet parsing at your own pace
/// - Handling partial packets across read boundaries
/// - Async cancellation without corrupting parser state
///
/// # Lifecycle
///
/// ```text
/// 1. Create:  AsyncCrsfReader::new(stream)
/// 2. Read:    reader.read_packet().await -> Result<Packet, CrsfStreamError>
/// 3. Repeat:  Continue reading packets as needed
/// 4. Stream ends: read_packet() returns UnexpectedEof
/// ```
///
/// # Thread Safety
///
/// Like underlying [`CrsfParser`], this type is not thread-safe. In async
/// contexts, ensure only one task accesses the reader at a time.
///
/// # Usage Examples
///
/// ## Example 1: Embassy Async Firmware
///
/// ```ignore
/// use embassy_executor::Spawner;
/// use uf_crsf::async_io::{AsyncCrsfReader, write_packet};
/// use uf_crsf::packets::{Packet, PacketAddress};
/// use embedded_io_async::Write;
///
/// #[embassy_executor::main]
/// async fn main(spawner: Spawner) {
///     let mut uart = embassy_rp::uart::Uart::new(...);
///
///     let mut reader = AsyncCrsfReader::new(&mut uart);
///
///     loop {
///         match reader.read_packet().await {
///             Ok(Packet::RcChannelsPacked(channels)) => {
///                 update_mixer(channels.channels);
///             }
///             Ok(Packet::LinkStatistics(stats)) => {
///                 update_telemetry(stats);
///             }
///             Ok(_) => {}
///             Err(e) => {
///                 eprintln!("Error: {:?}", e);
///             }
///         }
///
///         // Send telemetry back
///         let battery = Battery::new(voltage, current).unwrap();
///         write_packet(&mut uart, PacketAddress::Receiver, &battery).await.ok();
///     }
/// }
/// ```
///
/// ## Example 2: Async Timeout Handling
///
/// ```ignore
/// use uf_crsf::async_io::AsyncCrsfReader;
/// use embassy_time::{Duration, Timer, timeout};
///
/// let mut reader = AsyncCrsfReader::new(&mut uart);
///
/// loop {
///     match timeout(Duration::from_millis(100), reader.read_packet()).await {
///         Ok(Ok(packet)) => {
///             // Process packet
///             handle_packet(packet);
///         }
///         Ok(Err(e)) => {
///             // Parser error
///             eprintln!("Parse error: {:?}", e);
///         }
///         Err(_) => {
///             // Timeout - no packet received within 100ms
///             eprintln!("Timeout waiting for packet");
///             // Continue waiting for next packet
///         }
///     }
/// }
/// ```
///
/// ## Example 3: Async Receiver Implementation
///
/// ```ignore
/// use uf_crsf::async_io::{AsyncCrsfReader, write_packet};
/// use uf_crsf::packets::{Packet, PacketAddress};
///
/// let mut uart_tx = get_async_uart_tx(); // To flight controller
/// let mut uart_rx = get_async_uart_rx(); // From flight controller
/// let mut reader = AsyncCrsfReader::new(&mut uart_rx);
///
/// // Main async loop
/// loop {
///     // Read telemetry from flight controller
///     match reader.read_packet().await {
///         Ok(Packet::Battery(batt)) => {
///             forward_to_transmitter(batt).await;
///         }
///         Ok(Packet::Attitude(att)) => {
///             forward_to_transmitter(att).await;
///         }
///         Ok(_) => {}
///         Err(e) => eprintln!("Error: {:?}", e),
///     }
///
///     // Send RC channels to flight controller
///     let channels = RcChannelsPacked::new(get_rc_channels()).unwrap();
///     write_packet(&mut uart_tx, PacketAddress::FlightController, &channels).await.ok();
/// }
/// ```
///
/// # Hardware-Specific Guidance
///
/// **RP2040 with Embassy:**
/// ```ignore
/// use embassy_rp::uart::Uart;
/// use embedded_io_async::Read;
///
/// let mut uart = Uart::new(peripherals.UART0, irq, tx, rx, ...);
/// let mut reader = AsyncCrsfReader::new(&mut uart);
///
/// let packet = reader.read_packet().await?;
/// ```
///
/// **STM32 with Embassy:**
/// ```ignore
/// use embassy_stm32::uart::Uart;
///
/// let mut uart = Uart::new(peripherals.USART1, irq, tx, rx, ...);
/// let mut reader = AsyncCrsfReader::new(&mut uart);
///
/// let packet = reader.read_packet().await?;
/// ```
///
/// **ESP32 with ESP-IDF (embassy-esp32):**
/// ```ignore
/// use embassy_esp32::uart::Uart;
///
/// let mut uart = Uart::new(peripherals.UART0, ...);
/// let mut reader = AsyncCrsfReader::new(&mut uart);
///
/// let packet = reader.read_packet().await?;
/// ```
///
/// # Performance Considerations
///
/// - **Buffer size**: 128 bytes (2 packets) provides good headroom for burst reads
/// - **Read size**: Reads up to 64 bytes per loop iteration
/// - **Latency**: `read_packet().await` awaits until a complete packet is received
/// - **Async overhead**: Minimal - uses standard `embedded_io_async` traits
/// - **Memory**: ~128 bytes for input buffer + 64 bytes for parser = ~192 bytes
///
/// # Error Handling
///
/// | Error | Cause | Recovery |
/// |-------|-------|----------|
/// | `InvalidSync` | Non-CRSF bytes in stream | Parser auto-resets, continue |
/// | `InvalidCrc` | Corrupted packet | Parser auto-resets, continue |
/// | `Io` | UART/serial async error | Check hardware, reset connection |
/// | `UnexpectedEof` | Async stream closed | Connection lost, reconnect |
/// | `InputBufferTooSmall` | Buffer overflow | Increase `ASYNC_IO_BUFFER_SIZE` |
///
/// # Cancellation Safety
///
/// If `read_packet().await` is cancelled (e.g., via timeout), the parser's
/// internal state remains valid. The next `read_packet().await` will continue
/// from where it left off without losing data.
pub struct AsyncCrsfReader<R> {
    /// Internal parser for byte-level CRSF processing.
    parser: CrsfParser,
    /// The underlying async reader (UART, serial port, etc.).
    reader: R,
    /// Internal buffer for accumulating bytes between packets.
    input_buffer: Deque<u8, ASYNC_IO_BUFFER_SIZE>,
}

impl<R: embedded_io_async::Read> AsyncCrsfReader<R> {
    /// Creates a new async CRSF packet reader.
    ///
    /// The reader is initialized with an empty buffer and parser in
    /// `AwaitingSync` state. It's ready to receive packets immediately after
    /// construction.
    ///
    /// # Arguments
    ///
    /// * `reader` - Any type implementing `embedded_io_async::Read`
    ///   (async UART, async serial port, async TCP stream, etc.)
    ///
    /// # Example
    ///
    /// ```ignore
    /// use uf_crsf::async_io::AsyncCrsfReader;
    /// use embassy_rp::uart::Uart;
    ///
    /// let mut uart = Uart::new(...);
    /// let mut reader = AsyncCrsfReader::new(&mut uart);
    /// ```
    pub fn new(reader: R) -> Self {
        Self {
            parser: CrsfParser::new(),
            reader,
            input_buffer: Deque::new(),
        }
    }

    /// Asynchronously reads a complete CRSF packet from the underlying stream.
    ///
    /// This method awaits until a complete, validated packet is received. It
    /// handles:
    ///
    /// - Reading bytes from the stream asynchronously
    /// - Buffering partial packets
    /// - Validating CRC checksums
    /// - Parsing into a [`Packet`] enum
    ///
    /// # Returns
    ///
    /// - `Ok(packet)`: A fully parsed and validated CRSF packet
    /// - `Err(e)`: An error occurred (see [`CrsfStreamError`])
    ///
    /// # Async Behavior
    ///
    /// This method uses `await` to:
    /// - Wait for data from the stream
    /// - Handle backpressure from slow connections
    ///
    /// It can be cancelled at any `await` point (e.g., via timeout).
    /// Cancellation is safe - the parser state remains valid.
    ///
    /// # Buffer Management
    ///
    /// The reader uses an internal buffer (128 bytes by default) to accumulate
    /// bytes between packets. This allows:
    /// - Reading multiple packets in a single async read
    /// - Handling packets that span multiple read boundaries
    /// - Efficient buffering of burst data
    ///
    /// # Example: With Timeout
    ///
    /// ```ignore
    /// use uf_crsf::async_io::AsyncCrsfReader;
    /// use embassy_time::{Duration, timeout};
    ///
    /// let mut reader = AsyncCrsfReader::new(&mut uart);
    ///
    /// // Read with 100ms timeout
    /// match timeout(Duration::from_millis(100), reader.read_packet()).await {
    ///     Ok(Ok(Packet::LinkStatistics(stats))) => {
    ///         println!("RSSI: {}", stats.uplink_rssi_1);
    ///     }
    ///     Ok(Ok(packet)) => handle_packet(packet),
    ///     Ok(Err(e)) => eprintln!("Parse error: {:?}", e),
    ///     Err(_) => eprintln!("Timeout - no packet received"),
    /// }
    /// ```
    ///
    /// # Example: Continuous Reading
    ///
    /// ```ignore
    /// use uf_crsf::async_io::AsyncCrsfReader;
    /// use uf_crsf::packets::Packet;
    ///
    /// let mut reader = AsyncCrsfReader::new(&mut uart);
    ///
    /// loop {
    ///     match reader.read_packet().await {
    ///         Ok(Packet::RcChannelsPacked(channels)) => {
    ///             update_mixer(channels.channels);
    ///         }
    ///         Ok(packet) => handle_packet(packet),
    ///         Err(e) => eprintln!("Error: {:?}", e),
    ///     }
    /// }
    /// ```
    pub async fn read_packet(&mut self) -> Result<Packet, CrsfStreamError> {
        let mut temp_read_buf = [0; crate::constants::CRSF_MAX_PACKET_SIZE];

        loop {
            // First, process any buffered bytes
            while let Some(byte) = self.input_buffer.pop_front() {
                match self.parser.push_byte(byte) {
                    Ok(Some(packet)) => return Ok(packet),
                    Ok(None) => (),
                    Err(e) => return Err(e),
                }
            }

            // Read more data from the stream asynchronously
            let bytes_read = self
                .reader
                .read(&mut temp_read_buf)
                .await
                .map_err(|e| CrsfStreamError::Io(e.kind()))?;

            if bytes_read == 0 {
                return Err(CrsfStreamError::UnexpectedEof);
            }

            // Buffer new bytes
            for byte in &temp_read_buf[..bytes_read] {
                self.input_buffer
                    .push_back(*byte)
                    .map_err(|_| CrsfStreamError::InputBufferTooSmall)?;
            }
        }
    }
}

/// Asynchronously writes a CRSF packet to an `embedded_io_async::Write` stream.
///
/// This function serializes the given packet into a buffer (including sync
/// byte, length, type, payload, and CRC) and asynchronously writes the
/// entire packet to the specified stream.
///
/// This is an async convenience wrapper around [`write_packet_to_buffer`]
/// and `Write::write_all`.
///
/// # Arguments
///
/// * `writer` - The destination stream (async UART, async serial port, async TCP socket, etc.)
/// * `dest` - The destination device address (see [`PacketAddress`])
/// * `packet` - The packet to serialize and send
///
/// # Returns
///
/// - `Ok(())`: The packet was successfully written
/// - `Err(e)`: An I/O or serialization error occurred
///
/// # Async Behavior
///
/// This function uses `await` to:
/// - Wait for the write operation to complete
/// - Handle backpressure from slow connections
///
/// It can be cancelled at any `await` point. Cancellation may result in
/// a partially written packet.
///
/// # Example
///
/// ```ignore
/// use uf_crsf::async_io::write_packet;
/// use uf_crsf::packets::{Battery, PacketAddress};
///
/// let mut uart = get_async_uart();
///
/// // Send battery telemetry to receiver
/// let battery = Battery::new(1240, 100, 5000, 75).unwrap();
/// write_packet(&mut uart, PacketAddress::Receiver, &battery).await?;
/// ```
///
/// # Hardware-Specific Guidance
///
/// **RP2040 with Embassy:**
/// ```ignore
/// use embassy_rp::uart::Uart;
/// use uf_crsf::async_io::write_packet;
///
/// let mut uart_tx = uart.tx;
/// let packet = RcChannelsPacked::new([1500; 16]).unwrap();
/// write_packet(&mut uart_tx, PacketAddress::FlightController, &packet).await?;
/// ```
///
/// **STM32 with Embassy:**
/// ```ignore
/// use embassy_stm32::uart::Uart;
///
/// let mut uart_tx = uart.tx;
/// let packet = RcChannelsPacked::new([1500; 16]).unwrap();
/// write_packet(&mut uart_tx, PacketAddress::FlightController, &packet).await?;
/// ```
///
/// **ESP32 with Embassy-ESP32:**
/// ```ignore
/// use embassy_esp32::uart::Uart;
///
/// let mut uart_tx = uart.tx;
/// let packet = RcChannelsPacked::new([1500; 16]).unwrap();
/// write_packet(&mut uart_tx, PacketAddress::FlightController, &packet).await?;
/// ```
///
/// # Timeout Support
///
/// Combine with timeout for timeout-protected writes:
///
/// ```ignore
/// use embassy_time::{Duration, timeout};
///
/// match timeout(Duration::from_millis(100), write_packet(&mut uart, dest, &packet)).await {
///     Ok(Ok(())) => println!("Packet sent"),
///     Ok(Err(e)) => eprintln!("Write error: {:?}", e),
///     Err(_) => eprintln!("Write timeout"),
/// }
/// ```
pub async fn write_packet<W: Write, P: CrsfPacket>(
    writer: &mut W,
    dest: PacketAddress,
    packet: &P,
) -> Result<(), CrsfStreamError> {
    let mut buffer = [0u8; crate::constants::CRSF_MAX_PACKET_SIZE];
    let len = write_packet_to_buffer(&mut buffer, dest, packet)?;
    writer
        .write_all(&buffer[..len])
        .await
        .map_err(|e| CrsfStreamError::Io(e.kind()))?;
    Ok(())
}
