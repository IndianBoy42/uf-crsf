//! # uf-crsf: A `no_std` Rust Library for the TBS Crossfire Protocol
//!
//! This library provides a comprehensive, allocator-free implementation of the
//! TBS Crossfire (CRSF) protocol for embedded systems. It's designed for
//! integration into flight controllers, receivers, transmitters, and handset
//! applications that communicate over the CRSF protocol.
//!
//! ## Quick Start
//!
//! ```ignore
//! use uf_crsf::{parser::CrsfParser, packets::Packet};
//!
//! let mut parser = CrsfParser::new();
//! let mut rx_data = [0u8; 256]; // Your UART RX buffer
//!
//! // After reading bytes from UART:
//! for result in parser.iter_packets(&rx_data) {
//!     match result {
//!         Ok(Packet::LinkStatistics(stats)) => {
//!             // Handle telemetry
//!             println!("RSSI: {}, LQ: {}%", stats.uplink_rssi_1, stats.uplink_link_quality);
//!         }
//!         Ok(packet) => {
//!             // Other packet types...
//!         }
//!         Err(e) => {
//!             // Handle parsing errors - typically just log and continue
//!             eprintln!("Parse error: {:?}", e);
//!         }
//!     }
//! }
//! ```
//!
//! ## Key Features
//!
//! - **`no_std` compatible**: No heap allocations, suitable for bare-metal
//!   embedded systems
//! - **Zero-copy parsing**: Efficient byte-level parsing without allocation
//! - **Comprehensive packet support**: 34+ packet types from the CRSF spec
//!   and ExpressLRS
//! - **Device parameter management**: High-level API for device discovery and
//!   parameter reading/writing
//! - **Async and blocking I/O**: Support for both `embedded_io` and
//!   `embedded_io_async` traits
//! - **Embedded logging**: Optional `defmt` support for structured logging
//!
//! ## Architecture Overview
//!
//! The library is organized into several key modules:
//!
//! ### Parser ([`parser`])
//!
//! The [`CrsfParser`] is a state machine that parses raw byte streams into
//! validated CRSF packets. It handles:
//! - Packet framing (sync byte, length, CRC validation)
//! - Stream resynchronization after errors
//! - Zero-copy packet views via [`RawCrsfPacket`]
//!
//! Use the parser when you have a raw byte stream from UART, SPI, or another
//! transport.
//!
//! ### Packets ([`packets`])
//!
//! All CRSF packet types are defined in the [`packets`] module, each
//! implementing the [`CrsfPacket`] trait:
//! - **Telemetry**: [`LinkStatistics`], [`Battery`], [`GPS`], [`Attitude`], etc.
//! - **Commands**: [`Commands`], etc.
//! - **Device management**: [`DevicePing`], [`DeviceInformation`], etc.
//! - **RC channels**: [`RcChannelsPacked`]
//!
//! The [`Packet`] enum provides a unified type for handling all packet
//! variants.
//!
//! ### Device Management ([`device`])
//!
//! The [`DeviceManager`] provides a high-level API for:
//! - Discovering CRSF devices on the bus
//! - Reading and writing device parameters
//! - Handling parameter versioning and chunked transfers
//!
//! This is particularly useful for handset applications or configuration tools.
//!
//! ### I/O Abstractions ([`async_io`], [`blocking_io`])
//!
//! For easier integration with `embedded_io` and `embedded_io_async` traits:
//! - [`BlockingCrsfReader`]: Reads complete packets from blocking I/O streams
//! - [`AsyncCrsfReader`]: Async variant for non-blocking I/O
//! - [`write_packet()`]/[`write_packet_async()`]: Helper functions for writing packets
//!
//! ### Error Handling ([`error`])
//!
//! Two error types for different contexts:
//! - [`CrsfStreamError`]: Errors during stream packet reading (framing, CRC, I/O)
//! - [`CrsfParsingError`]: Errors during payload deserialization
//!
//! ## Common Integration Patterns
//!
//! ### Flight Controller Role
//!
//! Flight controllers (Betaflight, INAV) receive telemetry and RC data:
//!
//! ```ignore
//! use uf_crsf::{parser::CrsfParser, packets::Packet};
//!
//! let mut parser = CrsfParser::new();
//! let uart_rx_buffer: [u8; 256] = get_uart_data();
//!
//! for packet in parser.iter_packets(&uart_rx_buffer) {
//!     match packet {
//!         Ok(Packet::RcChannelsPacked(channels)) => {
//!             // Update RC channel inputs
//!             update_mixer(&channels.channels);
//!         }
//!         Ok(Packet::LinkStatistics(stats)) => {
//!             // Update telemetry display
//!             update_telemetry(stats);
//!         }
//!         Ok(Packet::Battery(batt)) => {
//!             // Monitor battery voltage
//!             if batt.voltage < LOW_VOLTAGE_THRESHOLD {
//!                 trigger_landing();
//!             }
//!         }
//!         Ok(_) => {} // Ignore other packets
//!         Err(e) => {
//!             // Log and continue - stream is self-synchronizing
//!             log::warn!("Parse error: {:?}", e);
//!         }
//!     }
//! }
//! ```
//!
//! ### Receiver Role
//!
//! CRSF receivers receive RC data from the transmitter and forward telemetry
//! from the flight controller:
//!
//! ```ignore
//! use uf_crsf::{
//!     blocking_io::{BlockingCrsfReader, write_packet},
//!     packets::{RcChannelsPacked, Battery, PacketAddress},
//! };
//!
//! let mut uart_tx = get_uart_tx(); // To flight controller
//! let mut uart_rx = get_uart_rx(); // From flight controller
//! let mut reader = BlockingCrsfReader::new(&mut uart_rx);
//!
//! // Forward RC channels to flight controller
//! let channels = RcChannelsPacked::new([1500; 16]).unwrap();
//! write_packet(&mut uart_tx, PacketAddress::FlightController, &channels).unwrap();
//!
//! // Read telemetry from flight controller
//! match reader.read_packet() {
//!     Ok(Packet::Battery(batt)) => {
//!         // Forward telemetry to transmitter
//!         // ...
//!     }
//!     Ok(_) => {}
//!     Err(e) => eprintln!("Error: {:?}", e),
//! }
//! ```
//!
//! ### Handset/Controller Application Role
//!
//! Desktop or mobile applications use the library to configure devices:
//!
//! ```ignore
//! use uf_crsf::{
//!     device::DeviceManager,
//!     packets::PacketAddress,
//! };
//! use embedded_io::blocking::{Read, Write};
//!
//! let mut serial = open_serial_port(); // USB-serial to CRSF device
//! let config = Default::default();
//! let mut manager = DeviceManager::new(&mut serial, config);
//!
//! // Discover devices
//! let devices = manager.discover_devices(Duration::from_secs(5)).unwrap();
//!
//! for device in &devices {
//!     println!("Device: {} (serial: {:x})", device.name, device.serial);
//!
//!     // Read parameters
//!     let params = manager.read_parameters(device).unwrap();
//!     for param in &params {
//!         println!("  {}: {}", param.name, param.value);
//!     }
//! }
//! ```
//!
//! ## Hardware Considerations
//!
//! ### Microcontroller Selection
//!
//! This library is designed for `no_std` environments and works on most
//! modern microcontrollers:
//!
//! **STM32 (Cortex-M):**
//! - Recommended: STM32F4, STM32G4, STM32H7 for high-speed UART handling
//! - Use DMA with circular buffers for efficient packet reception
//! - Typical UART baud rate: 420,000 baud (ExpressLRS) or 115,200 (standard)
//!
//! **nRF52/nRF53:**
//! - Works well with Nordic UART Service (NUS) for Bluetooth CRSF bridges
//! - Use EasyDMA for zero-copy UART transfers
//!
//! **ESP32:**
//! - Increase UART RX buffer size in menuconfig (default 256, recommend 1024+)
//! - ESP8266: Use hardware UART, not SoftwareSerial (too slow)
//!
//! **RP2040:**
//! - Use PIO or UART peripheral with proper baud rate configuration
//! - Watch for buffer overflow at high data rates
//!
//! ### UART Configuration
//!
//! CRSF typically operates at **420,000 baud** for ExpressLRS or
//! **115,200 baud** for legacy CRSF devices. Configure UART as:
//!
//! - **Data bits**: 8
//! - **Parity**: None
//! - **Stop bits**: 1
//! - **Flow control**: None (RTS/CTS not used in CRSF)
//!
//! ### Buffer Sizing
//!
//! **Receive buffer:**
//! - Minimum: 64 bytes (single packet)
//! - Recommended: 256-512 bytes (handles burst telemetry)
//! - Flight controller: 512-1024 bytes (high-rate telemetry streams)
//!
//! **Transmit buffer:**
//! - Minimum: 64 bytes
//! - Recommended: 128-256 bytes (allows packet queuing)
//!
//! ### Timing Considerations
//!
//! **ExpressLRS telemetry rate**: Typically 4-8 packets per 8ms frame
//! **Packet transmission time**: ~1-2ms per packet at 420k baud
//! **Processing budget**: 1-2ms per packet in flight controller loop
//!
//! Ensure your UART ISR or DMA handler runs fast enough to keep up with the
//! data rate. Use double-buffering or circular buffers to avoid packet loss.
//!
//! ## Feature Flags
//!
//! - **`device`** *(default)*: Enable device parameter management (`DeviceManager`,
//!   `ParameterSettingsEntry`, `ParameterRead`, `ParameterWrite`, etc.)
//! - **`defmt`**: Enable `defmt::Format` derives for structured logging
//! - **`embedded_io`**: Enable blocking I/O abstractions (`BlockingCrsfReader`)
//! - **`embedded_io_async`**: Enable async I/O abstractions (`AsyncCrsfReader`)
//!
//! ## Protocol References
//!
//! - [TBS CRSF Specification](https://github.com/tbs-fpv/tbs-crsf-spec)
//! - [CRSF Working Group](https://github.com/crsf-wg/crsf)
//! - [ExpressLRS Documentation](https://github.com/ExpressLRS/ExpressLRS)

#![no_std]
#![allow(clippy::needless_doctest_main)]
#![doc = include_str!("../README.md")]

pub mod constants;
#[cfg(feature = "device")]
pub mod device;
pub mod error;
pub mod packets;
pub mod parser;

#[cfg(feature = "embedded_io_async")]
pub mod async_io;

#[cfg(feature = "embedded_io")]
pub mod blocking_io;

// Re-export commonly used types
pub use error::{CrsfParsingError, CrsfStreamError};
pub use packets::{write_packet_to_buffer, Packet, PacketAddress, PacketType};
pub use parser::{CrsfParser, RawCrsfPacket};
