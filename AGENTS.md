# Agent Guidelines for uf-crsf

This document provides coding guidelines and development commands for agents working on the `uf-crsf` library - a `no_std` Rust library for parsing the TBS Crossfire protocol designed for embedded environments.

## Project Overview

- **Language**: Rust (Edition 2021)
- **Environment**: `no_std` (allocator-free, embedded-focused)
- **Primary Dependencies**: `heapless`, `crc`, `num_enum`, `libm`
- **Optional Features**: `defmt`, `embedded_io`, `embedded_io_async`

## Build, Lint, and Test Commands

### Build
```bash
# Standard build
cargo build

# Build with all features
cargo build --all-features

# Build examples (requires embedded_io features)
cargo build --examples --features=embedded_io_async,embedded_io

# Using just
just build  # or: just b
```

### Linting
```bash
# Standard lint check
cargo check
cargo clippy --all -- -D warnings

# Using just
just lint  # or: just l

# Pedantic clippy (strict mode)
just pedantic
```

### Formatting
```bash
# Format all code
cargo fmt --all

# Using just
just fmt
```

### Testing
```bash
# Run all tests with all features
cargo test --all-features -- --show-output

# Run a single test (by pattern)
cargo test --all-features <TEST_PATTERN> -- --show-output

# Run example tests
cargo test --examples --features=embedded_io_async,embedded_io

# Using just
just test                    # Run all tests
just test <TEST_PATTERN>     # Run specific test(s)
```

### CI Command
```bash
# Run the full CI suite locally
just ci
# This runs: lint, build, test (all-features), test (examples)
```

### Code Coverage
```bash
# Generate and open coverage report
cargo llvm-cov --open

# Using just
just cov
```

### Documentation
```bash
# Generate and open docs
cargo doc --all-features --no-deps --open

# Using just
just doc
```

## Code Style Guidelines

### Module Organization
- Use `#![no_std]` at the crate root
- Organize modules by functionality: `packets/`, `parser`, `error`, `constants`
- Each packet type lives in its own file under `src/packets/`
- Re-export public APIs from `mod.rs` files

### Imports
- Group imports: std/core → external crates → internal modules
- Use explicit imports, avoid glob imports (`use foo::*`)
- Example:
```rust
use crate::constants;
use crate::error::CrsfStreamError;
use crate::packets::{Packet, PacketAddress};
use crc::Crc;
use num_enum::TryFromPrimitive;
```

### Naming Conventions
- **Types**: `PascalCase` (e.g., `CrsfParser`, `LinkStatistics`)
- **Functions/Variables**: `snake_case` (e.g., `from_bytes`, `uplink_rssi`)
- **Constants**: `SCREAMING_SNAKE_CASE` (e.g., `CRSF_MAX_PACKET_SIZE`, `MIN_PAYLOAD_SIZE`)
- **Trait constants**: Use `SCREAMING_SNAKE_CASE` (e.g., `PACKET_TYPE`, `MIN_PAYLOAD_SIZE`)

### Type Annotations
- Prefer explicit types for public APIs
- Use type inference for local variables when obvious
- Always specify return types for public functions
- Use `usize` for sizes, lengths, and buffer indices
- Use fixed-size types (`u8`, `i16`, `u32`) for protocol fields

### Error Handling
- Use `Result<T, E>` for fallible operations
- Define domain-specific error types (see `CrsfParsingError`, `CrsfStreamError`)
- Implement `From` traits for error conversions
- Use `?` operator for error propagation
- Pattern:
```rust
pub enum CrsfParsingError {
    UnexpectedPacketType(u8),
    InvalidPayloadLength,
    InvalidPayload,
    BufferOverflow,
}
```

### Struct Definitions
- Add derives: `Debug`, `Clone`, `PartialEq` at minimum
- Add `#[cfg_attr(feature = "defmt", derive(defmt::Format))]` for embedded debugging
- Use doc comments with units for fields (e.g., "Voltage (in 10mV units)")
- Example:
```rust
#[derive(Default, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Battery {
    /// Voltage (in 10mV units, e.g., 1234 is 12.34V).
    pub voltage: i16,
    /// Current (in 10mA units, e.g., 100 is 1.0A).
    pub current: i16,
}
```

### Trait Implementation
- Implement the `CrsfPacket` trait for all packet types
- Required associated items:
  - `const PACKET_TYPE: PacketType`
  - `const MIN_PAYLOAD_SIZE: usize`
- Required methods:
  - `fn from_bytes(data: &[u8]) -> Result<Self, CrsfParsingError>`
  - `fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, CrsfParsingError>`

### Buffer Handling
- Always validate buffer sizes before operations
- Use slice operations over manual indexing when possible
- Use `copy_from_slice` for bulk copies
- Validate lengths with explicit checks:
```rust
if data.len() != Self::MIN_PAYLOAD_SIZE {
    return Err(CrsfParsingError::InvalidPayloadLength);
}
```

### Testing
- Place unit tests in a `#[cfg(test)]` module at the bottom of each file
- For integration tests, add `#![cfg(test)]` and `extern crate std;` at the top
- Use descriptive test names: `test_<feature>_<scenario>`
- Test both success and error paths
- Use `assert_eq!` for equality, `assert!` for booleans
- Example:
```rust
#[test]
fn test_battery_to_bytes() {
    assert_eq!(Battery::MIN_PAYLOAD_SIZE, 8);
    let battery = Battery::new(1234, 100, 5000, 75).unwrap();
    // ... test logic
}
```

### Documentation
- Use `///` for public API documentation
- Use `//` for inline comments
- Document all public types, functions, and modules
- Include examples in doc comments where helpful
- Document units, ranges, and special values

### Feature Gates
- Gate optional dependencies and code with `#[cfg(feature = "...")]`
- Common features: `defmt`, `embedded_io`, `embedded_io_async`
- Example:
```rust
#[cfg(feature = "embedded_io_async")]
pub mod async_io;
```

### Clippy Compliance
- Code must pass `cargo clippy --all -- -D warnings` (warnings as errors)
- Allow specific lints only when justified (e.g., `#![allow(clippy::needless_doctest_main)]`)
- Prefer clarity over brevity when clippy suggests obscure patterns

## Development Dependencies
- **libudev-dev** (Linux): Required for serial port examples
  - Debian/Ubuntu: `sudo apt install -y libudev-dev`
  - Fedora: `sudo dnf install systemd-devel`
- **just**: Optional but recommended task runner

## Common Patterns

### Byte Parsing
```rust
// Big-endian multi-byte values
let value = i16::from_be_bytes(data[0..2].try_into()
    .map_err(|_| CrsfParsingError::InvalidPayloadLength)?);

// 24-bit values (expand to u32)
let mut bytes: [u8; 4] = [0; 4];
bytes[1..].copy_from_slice(&data[4..7]);
let value = u32::from_be_bytes(bytes);
```

### Packet Serialization
```rust
fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, CrsfParsingError> {
    self.validate_buffer_size(buffer)?;
    buffer[0..2].copy_from_slice(&self.field.to_be_bytes());
    Ok(Self::MIN_PAYLOAD_SIZE)
}
```

## When Making Changes

1. **Before committing**: Run `just ci` to ensure all checks pass
2. **Add tests**: Include unit tests for new packet types or parsing logic
3. **Update docs**: Keep README.md and inline documentation current
4. **Check `no_std`**: Avoid using std library features (use `core` or `heapless`)
5. **Maintain compatibility**: This is an embedded library - avoid allocations

## Additional Resources
- Protocol spec: https://github.com/tbs-fpv/tbs-crsf-spec
- CRSF Working Group: https://github.com/crsf-wg/crsf
