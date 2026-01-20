//! Blocking I/O abstractions for CRSF packet reading and writing.
//!
//! This module provides [`BlockingCrsfReader`] and [`write_packet`] for working
//! with synchronous `embedded_io::Read` and `embedded_io::Write` streams.
//!
//! # When to Use
//!
//! Use this module when:
//! - Working with blocking UART/serial ports
//! - Implementing simple polling-based firmware
//! - Building handheld/desktop applications with serial communication
//! - Your platform doesn't support async/await
//!
//! # For Async Support
//!
//! If you need async I/O, use the [`async_io`](crate::async_io) module instead
//! (requires `embedded_io_async` feature).

use crate::error::CrsfStreamError;
use crate::packets::{write_packet_to_buffer, CrsfPacket, Packet, PacketAddress};
use crate::parser::CrsfParser;
use embedded_io::{Error, Read, Write};
use heapless::Deque;

/// Size of the internal input buffer for [`BlockingCrsfReader`].
///
/// Sized to hold 2 complete CRSF packets, providing headroom for burst reads
/// and handling cases where a partial packet remains in the buffer after a
/// complete packet is parsed.
///
/// If you experience [`CrsfStreamError::InputBufferTooSmall`] errors in
/// production, increase this constant and recompile the library.
const BLOCKING_IO_BUFFER_SIZE: usize = crate::constants::CRSF_MAX_PACKET_SIZE * 2;

/// Blocking CRSF packet reader for `embedded_io::Read` streams.
///
/// This type provides a convenient abstraction for reading complete CRSF packets
/// from a blocking I/O stream (e.g., UART, USB-serial, TCP socket). It handles:
///
/// - Buffering incoming bytes
/// - Parsing packet framing
/// - Validating CRC checksums
/// - Returning fully parsed packets
///
/// # Architecture
///
/// The reader maintains an internal buffer that decouples the underlying read
/// operations from packet parsing:
///
/// ```text
/// Stream -> Read -> Input Buffer -> Parser -> Packet
///           (64b)     (128b)        (64b)    (Enum)
/// ```
///
/// This design allows:
/// - Larger reads from the stream (more efficient)
/// - Packet parsing at your own pace
/// - Handling partial packets across read boundaries
///
/// # Lifecycle
///
/// ```text
/// 1. Create:  BlockingCrsfReader::new(stream)
/// 2. Read:    reader.read_packet() -> Result<Packet, CrsfStreamError>
/// 3. Repeat:  Continue reading packets as needed
/// 4. Stream ends: read_packet() returns UnexpectedEof
/// ```
///
/// # Thread Safety
///
/// Like the underlying [`CrsfParser`], this type is not thread-safe. If used
/// in a multi-threaded context, wrap it in a mutex or use a single-threaded
/// event loop pattern.
///
/// # Usage Examples
///
/// ## Example 1: Flight Controller Polling Loop
///
/// ```ignore
/// use uf_crsf::{
///     blocking_io::{BlockingCrsfReader, write_packet},
///     packets::{RcChannelsPacked, PacketAddress},
///     packets::Packet,
/// };
/// use embedded_io::blocking::{Read, Write};
///
/// let mut uart_tx = get_uart_tx(); // To receiver
/// let mut uart_rx = get_uart_rx(); // From receiver
/// let mut reader = BlockingCrsfReader::new(&mut uart_rx);
///
/// main_loop() {
///     // Read RC channels from receiver
///     match reader.read_packet() {
///         Ok(Packet::RcChannelsPacked(channels)) => {
///             // Update flight controller mixer
///             update_mixer(channels.channels);
///         }
///         Ok(Packet::LinkStatistics(stats)) => {
///             // Update telemetry display
///             update_telemetry(stats);
///         }
///         Ok(_) => {}
///         Err(e) => {
///             eprintln!("Read error: {:?}", e);
///             // Parser auto-resets, continue reading
///         }
///     }
///
///     // Send telemetry back to receiver
///     let battery = Battery::new(voltage, current).unwrap();
///     write_packet(&mut uart_tx, PacketAddress::Receiver, &battery).ok();
/// }
/// ```
///
/// ## Example 2: Desktop Serial Application
///
/// ```ignore
/// use uf_crsf::blocking_io::BlockingCrsfReader;
/// use serialport::SerialPort;
///
/// let mut port = serialport::new("/dev/ttyUSB0", 115_200)
///     .open()
///     .expect("Failed to open port");
///
/// let mut reader = BlockingCrsfReader::new(&mut port);
///
/// loop {
///     match reader.read_packet() {
///         Ok(packet) => {
///             println!("Received: {:?}", packet);
///         }
///         Err(e) => {
///             eprintln!("Error: {:?}", e);
///             break;
///         }
///     }
/// }
/// ```
///
/// ## Example 3: Receiver Implementation
///
/// ```ignore
/// use uf_crsf::{
///     blocking_io::{BlockingCrsfReader, write_packet},
///     packets::{Packet, PacketAddress},
/// };
///
/// let mut uart_tx = get_uart_tx(); // To flight controller
/// let mut uart_rx = get_uart_rx(); // From flight controller
/// let mut reader = BlockingCrsfReader::new(&mut uart_rx);
///
/// // Main loop
/// loop {
///     // Read telemetry from flight controller
///     match reader.read_packet() {
///         Ok(Packet::Battery(batt)) => {
///             // Forward telemetry to transmitter
///             forward_to_transmitter(batt);
///         }
///         Ok(Packet::Attitude(att)) => {
///             forward_to_transmitter(att);
///         }
///         Ok(_) => {}
///         Err(e) => eprintln!("Error: {:?}", e),
///     }
///
///     // Send RC channels to flight controller
///     let channels = RcChannelsPacked::new(get_rc_channels()).unwrap();
///     write_packet(&mut uart_tx, PacketAddress::FlightController, &channels).ok();
/// }
/// ```
///
/// # Hardware-Specific Guidance
///
/// **STM32 with HAL:**
/// ```ignore
/// use stm32f4xx_hal::serial::Serial;
///
/// let serial = Serial::new(
///     dp.USART1,
///     (tx_pin, rx_pin),
///     115_200.bps(),
///     clocks,
/// ).unwrap();
///
/// let mut reader = BlockingCrsfReader::new(serial);
/// let packet = reader.read_packet().unwrap();
/// ```
///
/// **ESP32 (std):**
/// ```ignore
/// use embedded_io::adapters::FromStd;
///
/// let port = serialport::new("/dev/ttyUSB0", 115_200).open().unwrap();
/// let mut reader = BlockingCrsfReader::new(FromStd::new(port));
/// ```
///
/// **RP2040 with hal:**
/// ```ignore
/// use rp2040_hal::uart::UartPeripheral;
///
/// let mut uart = UartPeripheral::new(
///     pac.UART0,
///     (tx, rx),
///     &mut pac.RESETS,
///     115_200.Hz(),
///     clocks,
/// ).enable();
///
/// let mut reader = BlockingCrsfReader::new(uart);
/// ```
///
/// # Performance Considerations
///
/// - **Buffer size**: 128 bytes (2 packets) provides good headroom for burst reads
/// - **Read size**: Reads up to 64 bytes per loop iteration
/// - **Latency**: `read_packet()` blocks until a complete packet is received
/// - **Memory**: ~128 bytes for input buffer + 64 bytes for parser = ~192 bytes
///
/// # Error Handling
///
/// | Error | Cause | Recovery |
/// |-------|-------|----------|
/// | `InvalidSync` | Non-CRSF bytes in stream | Parser auto-resets, continue |
/// | `InvalidCrc` | Corrupted packet | Parser auto-resets, continue |
/// | `Io` | UART/serial error | Check hardware, reset connection |
/// | `UnexpectedEof` | Stream closed | Connection lost, reconnect |
/// | `InputBufferTooSmall` | Buffer overflow | Increase `BLOCKING_IO_BUFFER_SIZE` |
pub struct BlockingCrsfReader<R> {
    /// Internal parser for byte-level CRSF processing.
    parser: CrsfParser,
    /// The underlying reader (UART, serial port, etc.).
    reader: R,
    /// Internal buffer for accumulating bytes between packets.
    input_buffer: Deque<u8, BLOCKING_IO_BUFFER_SIZE>,
}

impl<R: Read> BlockingCrsfReader<R> {
    /// Creates a new blocking CRSF packet reader.
    ///
    /// The reader is initialized with an empty buffer and the parser in
    /// `AwaitingSync` state. It's ready to receive packets immediately after
    /// construction.
    ///
    /// # Arguments
    ///
    /// * `reader` - Any type implementing `embedded_io::Read` (UART, serial
    ///   port, TCP stream, etc.)
    ///
    /// # Example
    ///
    /// ```ignore
    /// use uf_crsf::blocking_io::BlockingCrsfReader;
    /// use serialport::SerialPort;
    ///
    /// let port = serialport::new("/dev/ttyUSB0", 115_200)
    ///     .open()
    ///     .unwrap();
    ///
    /// let mut reader = BlockingCrsfReader::new(port);
    /// ```
    pub fn new(reader: R) -> Self {
        Self {
            parser: CrsfParser::new(),
            reader,
            input_buffer: Deque::new(),
        }
    }

    /// Reads a complete CRSF packet from the underlying stream.
    ///
    /// This method blocks until a complete, validated packet is received. It
    /// handles:
    ///
    /// - Reading bytes from the stream
    /// - Buffering partial packets
    /// - Validating CRC checksums
    /// - Parsing the packet into a [`Packet`] enum
    ///
    /// # Returns
    ///
    /// - `Ok(packet)`: A fully parsed and validated CRSF packet
    /// - `Err(e)`: An error occurred (see [`CrsfStreamError`])
    ///
    /// # Blocking Behavior
    ///
    /// This method blocks indefinitely until:
    /// - A complete packet is received and validated
    /// - An I/O error occurs
    /// - The stream closes (returns `UnexpectedEof`)
    ///
    /// If you need timeout behavior, wrap the underlying reader with a timeout
    /// layer or use the async version with timeout.
    ///
    /// # Buffer Management
    ///
    /// The reader uses an internal buffer (128 bytes by default) to accumulate
    /// bytes between packets. This allows:
    /// - Reading multiple packets in a single read operation
    /// - Handling packets that span multiple read boundaries
    /// - Efficient buffering of burst data
    ///
    /// # Example
    ///
    /// ```ignore
    /// use uf_crsf::blocking_io::BlockingCrsfReader;
    /// use uf_crsf::packets::Packet;
    ///
    /// let mut reader = BlockingCrsfReader::new(serial_port);
    ///
    /// loop {
    ///     match reader.read_packet() {
    ///         Ok(Packet::LinkStatistics(stats)) => {
    ///             println!("RSSI: {}, LQ: {}%", stats.uplink_rssi_1, stats.uplink_link_quality);
    ///         }
    ///         Ok(packet) => {
    ///             // Handle other packet types
    ///             handle_packet(packet);
    ///         }
    ///         Err(e) => {
    ///             eprintln!("Error: {:?}", e);
    ///             // Parser auto-resets on errors, continue reading
    ///         }
    ///     }
    /// }
    /// ```
    pub fn read_packet(&mut self) -> Result<Packet, CrsfStreamError> {
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

            // Read more data from the stream
            let bytes_read = self
                .reader
                .read(&mut temp_read_buf)
                .map_err(|e| CrsfStreamError::Io(e.kind()))?;

            if bytes_read == 0 {
                return Err(CrsfStreamError::UnexpectedEof);
            }

            // Buffer the new bytes
            for byte in &temp_read_buf[..bytes_read] {
                self.input_buffer
                    .push_back(*byte)
                    .map_err(|_| CrsfStreamError::InputBufferTooSmall)?;
            }
        }
    }
}

/// Synchronously writes a CRSF packet to an `embedded_io::Write` stream.
///
/// This function serializes the given packet into a buffer (including sync byte,
/// length, type, payload, and CRC) and writes the entire packet to the
/// specified stream.
///
/// This is a convenience wrapper around [`write_packet_to_buffer`] and
/// `Write::write_all`.
///
/// # Arguments
///
/// * `writer` - The destination stream (UART, serial port, TCP socket, etc.)
/// * `dest` - The destination device address (see [`PacketAddress`])
/// * `packet` - The packet to serialize and send
///
/// # Returns
///
/// - `Ok(())`: The packet was successfully written
/// - `Err(e)`: An I/O or serialization error occurred
///
/// # Blocking Behavior
///
/// This function blocks until the entire packet is written. For large packets
/// or slow streams, this may take significant time.
///
/// # Example
///
/// ```ignore
/// use uf_crsf::{
///     blocking_io::write_packet,
///     packets::{Battery, PacketAddress},
/// };
///
/// let mut uart = get_uart();
///
/// // Send battery telemetry to receiver
/// let battery = Battery::new(1240, 100, 5000, 75).unwrap();
/// write_packet(&mut uart, PacketAddress::Receiver, &battery)?;
/// ```
///
/// # Hardware-Specific Guidance
///
/// **STM32:**
/// ```ignore
/// use uf_crsf::blocking_io::write_packet;
/// use stm32f4xx_hal::serial::Serial;
///
/// let mut uart_tx = uart.tx;
/// let packet = RcChannelsPacked::new([1500; 16]).unwrap();
/// write_packet(&mut uart_tx, PacketAddress::FlightController, &packet)?;
/// ```
///
/// **ESP32:**
/// ```ignore
/// use embedded_io::adapters::FromStd;
///
/// let mut port = FromStd::new(serial_port);
/// write_packet(&mut port, PacketAddress::FlightController, &packet)?;
/// ```
///
/// **RP2040:**
/// ```ignore
/// let mut uart = UartPeripheral::new(...).enable();
/// write_packet(&mut uart, PacketAddress::FlightController, &packet)?;
/// ```
pub fn write_packet<W: Write, P: CrsfPacket>(
    writer: &mut W,
    dest: PacketAddress,
    packet: &P,
) -> Result<(), CrsfStreamError> {
    let mut buffer = [0u8; crate::constants::CRSF_MAX_PACKET_SIZE];
    let len = write_packet_to_buffer(&mut buffer, dest, packet)?;
    writer
        .write_all(&buffer[..len])
        .map_err(|e| CrsfStreamError::Io(e.kind()))?;
    Ok(())
}
