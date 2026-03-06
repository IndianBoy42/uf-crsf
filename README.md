# uf-crsf

[![CI](https://github.com/jettify/uf-crsf/actions/workflows/CI.yml/badge.svg)](https://github.com/jettify/uf-crsf/actions/workflows/CI.yml)
[![codecov](https://codecov.io/gh/jettify/uf-crsf/graph/badge.svg?token=2N16CN1OZX)](https://codecov.io/gh/jettify/uf-crsf)
[![crates.io](https://img.shields.io/crates/v/uf-crsf)](https://crates.io/crates/uf-crsf)
[![docs.rs](https://img.shields.io/docsrs/uf-crsf)](https://docs.rs/uf-crsf/latest/uf_crsf/)

A `no_std` Rust library for parsing the TBS Crossfire protocol, designed for embedded environments without an allocator.

This library provides a two-layer API:

- A low-level layer for raw packet parsing from a byte stream.
- A higher-level layer that converts raw packets into idiomatic Rust structs.

## Features

- `no_std` and allocator-free for embedded systems.
- Two-layer API for flexible parsing.
- Supports a wide range of CRSF packets.
- IO and MCU agnostic.
- Minimal dependencies.

## Implementation status

**Legend:**

- `🟢` - Implemented
- `🔴` - Not Implemented

| Packet Name | Packet Address | Status |
| :--- | :--- | :--- |
| **Broadcast Frames** | | |
| GPS | `0x02` | 🟢 |
| GPS Time | `0x03` | 🟢 |
| GPS Extended | `0x06` | 🟢 |
| Variometer Sensor | `0x07` | 🟢 |
| Battery Sensor | `0x08` | 🟢 |
| Barometric Altitude & Vertical Speed | `0x09` | 🟢 |
| Airspeed | `0x0A` | 🟢 |
| Heartbeat | `0x0B` | 🟢 |
| RPM | `0x0C` | 🟢 |
| TEMP | `0x0D` | 🟢 |
| Voltages | `0x0E` | 🟢 |
| Discontinued | `0x0F` | 🟢 |
| VTX Telemetry | `0x10` | 🟢 |
| Barometer | `0x11` | 🟢 |
| Magnetometer | `0x12` | 🟢 |
| Accel Gyro | `0x13` | 🟢 |
| Link Statistics | `0x14` | 🟢 |
| RC Channels Packed Payload | `0x16` | 🟢 |
| Subset RC Channels Packed | `0x17` | 🔴 |
| RC Channels Packed 11-bits | `0x18` | 🔴 |
| Link Statistics RX | `0x1C` | 🟢 |
| Link Statistics TX | `0x1D` | 🟢 |
| Attitude | `0x1E` | 🟢 |
| MAVLink FC | `0x1F` | 🟢 |
| Flight Mode | `0x21` | 🟢 |
| ESP_NOW Messages | `0x22` | 🟢 |
| **Extended Frames** | | |
| Parameter Ping Devices | `0x28` | 🟢 |
| Parameter Device Information | `0x29` | 🟢 |
| Parameter Settings (Entry) | `0x2B` | 🔴 |
| Parameter Settings (Read) | `0x2C` | 🔴 |
| Parameter Value (Write) | `0x2D` | 🔴 |
| Direct Commands | `0x32` | 🟢 |
| Logging | `0x34` | 🟢 |
| Remote Related Frames | `0x3A` | 🟢 |
| Game | `0x3C` | 🟢 |
| KISSFC Reserved | `0x78 - 0x79` | 🔴 |
| MSP Request | `0x7A` | 🔴 |
| MSP Response | `0x7B` | 🔴 |
| ArduPilot Legacy Reserved | `0x7F` | 🔴 |
| ArduPilot Reserved Passthrough Frame | `0x80` | 🟢 |
| mLRS Reserved | `0x81, 0x82` | 🔴 |
| CRSF MAVLink Envelope | `0xAA` | 🟢 |
| CRSF MAVLink System Status Sensor | `0xAC` | 🟢 |

## Note

Library is under active development and testing, API might change at any time.

## Installation

Add `uf-crsf` to your `Cargo.toml`:

```toml
[dependencies]
uf-crsf = "*" # replace * by the latest version of the crate.
```

Or use the command line:

```bash
cargo add uf-crsf
```

## Usage

Here is a basic example of how to parse a CRSF packet from a byte array:

```rust
use uf_crsf::CrsfParser;

fn main() {
    let mut parser = CrsfParser::new();

    // A sample CRSF packet payload for RC channels
    let buf: [u8; 26] = [
        0xC8, // Address
        0x18, // Length
        0x16, // Type (RC Channels)
        0x03, 0x1F, 0x58, 0xC0, 0x07, 0x16, 0xB0, 0x80, 0x05, 0x2C, 0x60, 0x01, 0x0B, 0xF8, 0xC0,
        0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 252,  // Packet
        0x42, // Crc
    ];

    for item in parser.iter_packets(&buf) {
        match item {
            Ok(p) => println!("{:?}", p),
            Err(e) => eprintln!("Error parsing packet: {:?}", e),
        }
    }
}
```

## Troubleshooting

### Common Issues

#### CRC Errors
**Symptom:** Frequent `InvalidCrc` errors when parsing packets.

**Possible Causes:**
- **Electrical noise** on the serial line - Check wiring, ensure proper grounding
- **Baud rate mismatch** - CRSF typically uses 420,000 baud for ExpressLRS
- **Weak RF link** - If receiving over wireless, check antenna connections
- **UART buffer overflow** - Increase RX buffer size or process data faster

**Solutions:**
1. Verify UART configuration: 8 data bits, no parity, 1 stop bit (8N1)
2. Check that baud rate matches your hardware (420,000 for ELRS, 115,200 for standard CRSF)
3. Ensure CTS/RTS flow control is disabled if not used
4. Use shorter cables or add termination resistors for high-speed signals

#### Sync Errors
**Symptom:** `InvalidSync` errors or parser not recognizing packets.

**Possible Causes:**
- **Non-CRSF data** on the line (e.g., debug output from other components)
- **Incorrect start byte** - CRSF packets start with device address (0xC8-0xEA)
- **Byte alignment issues** - UART framing errors

**Solutions:**
1. The parser automatically resynchronizes - just continue feeding bytes
2. Check that the connected device is actually sending CRSF protocol
3. Verify wiring - TX/RX may be swapped
4. Ensure device is powered and transmitting

#### Buffer Overflow
**Symptom:** `InputBufferTooSmall` errors in blocking/async readers.

**Cause:** Reading faster than parsing can process.

**Solutions:**
1. Increase buffer size in reader (modify `BLOCKING_IO_BUFFER_SIZE` or `ASYNC_IO_BUFFER_SIZE`)
2. Process packets more frequently in your main loop
3. Check for blocking operations in packet handlers

### Platform-Specific Tips

#### STM32 (Cortex-M)
- **Use DMA with circular buffers** for efficient reception
- **Check UART overrun flag (ORE)** if data is lost
- **Verify USART clock** is enabled in RCC
- Typical setup: USART1, 420,000 baud, 8N1, DMA RX

#### ESP32
- **Increase UART RX buffer size** in menuconfig (default 256, recommend 1024+)
- **Use hardware UART** (not SoftwareSerial) - SoftwareSerial is too slow for CRSF
- **Check WiFi coexistence** - WiFi can interfere with high-speed UART
- **UART FIFO threshold** - Configure appropriately for your data rate

#### RP2040
- **Use PIO for high-speed UART** if standard UART can't keep up
- **Check for UART RX FIFO overrun**
- **Verify GPIO pin configuration** - TX/RX may need swapping
- **Clock domain issues** - Ensure consistent clocking when reading from different cores

#### Desktop/Laptop (Linux/Windows)
- **Check USB driver** is loaded for your serial adapter
- **Verify cable connections** - USB-serial adapters can be flaky
- **Ensure correct port** is selected (check `serialport::available_ports()`)
- **Permissions** - May need to add user to `dialout` group on Linux

### Debugging Tips

1. **Use `push_byte_raw()`** to inspect packet types before full parsing
2. **Enable defmt logging** (with `defmt` feature) for structured logging on embedded
3. **Logic analyzer** - Capture actual traffic to verify protocol compliance
4. **Check packet length** - CRSF packets are 4-64 bytes; larger frames indicate corruption

### Getting Help

- **GitHub Issues:** https://github.com/jettify/uf-crsf/issues
- **CRSF Protocol Spec:** https://github.com/tbs-fpv/tbs-crsf-spec
- **ExpressLRS Documentation:** https://github.com/ExpressLRS/ExpressLRS

## License

This project is licensed under the `Apache 2.0`. See the [LICENSE](https://github.com/jettify/uf-crsf/blob/master/LICENSE) file for details.

## Protocol Specification

- [Official TBS CRSF Protocol Specification](https://github.com/tbs-fpv/tbs-crsf-spec)
- [CRSF Working Group Fork](https://github.com/crsf-wg/crsf)
