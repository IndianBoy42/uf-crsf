# Plan: Improve `no_std` Embedded Microcontroller Support

## Executive Summary

The `uf-crsf` library is already a well-designed `no_std` library with zero heap allocations,
comprehensive packet support, and good documentation. The core parser (`CrsfParser`) is
lean (~72 bytes) and works well for embedded targets. However, there are several areas where
embedded support can be strengthened — from CI verification to memory optimization to
feature-gating high-level components that don't belong on constrained MCUs.

This plan organizes improvements into **6 independent work streams**, prioritized by
impact-to-effort ratio.

---

## Current State Assessment

### What Works Well (keep as-is)
- `#![no_std]` at crate root with zero heap allocations
- `CrsfParser` is lean (~72 bytes), suitable for ISR or main-loop byte-by-byte parsing
- All dependencies are `no_std`-compatible (`crc`, `heapless`, `num_enum`, `libm`)
- Comprehensive packet support (34+ types from CRSF spec + ExpressLRS)
- `embedded_io` / `embedded_io_async` integration via feature flags
- `defmt` support with proper feature gating
- Excellent documentation with hardware-specific guidance (STM32, nRF52, ESP32, RP2040)
- STM32 Embassy demo that's a production-quality reference

### Key Pain Points

| Issue | Severity | Impact |
|-------|----------|--------|
| No CI verification of `no_std` compilation | **High** | Regressions undetected |
| `Packet` enum is ~700+ bytes (dominated by `ParameterSettingsEntry`) | **High** | Stack bloat for all packet consumers |
| `DeviceManager` is ~80-120KB (impossible on small MCUs) | **Medium** | Users pay for unused code |
| `libm` only used for 2 functions in 1 file | **Low** | Unnecessary flash on constrained devices |
| Duplicate CRC table instances | **Low** | ~256 bytes wasted flash |
| Missing `defmt::Format` on device module types | **Low** | Debug gap on embedded |
| `ParameterChunkReassembler::reassemble()` 452-byte stack buffer | **Medium** | Stack pressure during parameter loading |
| Buffer sizes in I/O readers not configurable | **Low** | Users must fork to tune |
| STM32 demo uses stale `embedded-io-async` version | **Low** | Build issues for new users |

---

## Work Streams

### WS1: CI `no_std` Verification (High Priority)

**Goal:** Ensure the library always compiles and passes lint for embedded targets.

**Changes:**
- Add a cross-compilation CI step: `cargo build --target thumbv7em-none-eabihf --no-default-features`
- Add a step for each feature individually:
  - `cargo build --target thumbv7em-none-eabihf --features defmt`
  - `cargo build --target thumbv7em-none-eabihf --features logging`
  - `cargo build --target thumbv7em-none-eabihf --features embedded_io`
  - `cargo build --target thumbv7em-none-eabihf --features embedded_io_async`
- Add a "no features" lint step: `cargo clippy --target thumbv7em-none-eabihf --no-default-features -- -D warnings`
- Update `justfile` with corresponding recipes

**Files to modify:**
- `.github/workflows/CI.yml` — add cross-compilation job
- `justfile` — add `just check-no-std` recipe

**Deliverables:**
- CI fails if `no_std` compilation breaks
- Individual feature flag compilation verified

**Estimated effort:** Small (1-2 hours)

---

### WS2: Feature-Gate `DeviceManager` and Parameter Protocol (High Priority)

**Goal:** Allow embedded users to exclude the heavy `DeviceManager` and parameter types,
significantly reducing binary size and stack usage for flight controller / receiver roles.

**Rationale:** `DeviceManager` is only used by handset/config-tool applications. Flight
controllers and receivers only need `CrsfParser` + packet types. The `ParameterSettingsEntry`
variant alone adds ~650 bytes to every `Packet` enum instance.

**Changes:**

1. **New feature flag:** `device` (or `device-management`)
   - When enabled (default): current behavior
   - When disabled: exclude `device.rs` module and parameter protocol packet types

2. **Feature-gate modules and types:**
   ```rust
   #[cfg(feature = "device")]
   pub mod device;
   ```

3. **Split `Packet` enum:** Create a `CorePacket` enum (without parameter types) that's
   smaller, and a full `Packet` enum behind the `device` feature. OR simply gate the
   parameter-related variants behind the feature.

   **Option A — Gate variants (simpler):**
   ```rust
   pub enum Packet {
       LinkStatistics(LinkStatistics),
       // ... all core variants ...
       #[cfg(feature = "device")]
       ParameterSettingsEntry(ParameterSettingsEntry),
       #[cfg(feature = "device")]
       ParameterChunk(ParameterChunk),
       #[cfg(feature = "device")]
       ParameterRead(ParameterRead),
       #[cfg(feature = "device")]
       ParameterWrite(ParameterWrite),
       // ...
   }
   ```
   This is the simplest approach. The `Packet::parse()` match arms for these types
   would also be gated.

   **Option B — Split enum (more complex, cleaner):**
   ```rust
   // Core packets only — small (~100 bytes)
   pub enum Packet { ... }
   // Full packet including device protocol
   #[cfg(feature = "device")]
   pub enum FullPacket { ... }
   ```
   More disruptive to the API, but cleaner separation.

4. **Feature-gate dependent packet types:**
   - `ParameterSettingsEntry`, `ParameterChunk`, `ParameterChunkReassembler`
   - `ParameterRead`, `ParameterWrite`
   - `DeviceInformation`, `DevicePing`

   **Wait —** `DeviceInformation` and `DevicePing` are also used standalone. These should
   remain ungated. Only the `device` module's `DeviceManager` and the parameter-specific
   types need gating.

   **Revised approach:** Gate `DeviceManager` (in `device.rs`) behind a `device` feature.
   Gate `ParameterSettingsEntry`, `ParameterChunk`, `ParameterRead`, `ParameterWrite` behind
   same feature. Keep `DeviceInformation`, `DevicePing` ungated (they're small packets).

5. **Update `Cargo.toml`:**
   ```toml
   [features]
   default = ["device"]  # backward compatible
   device = []
   ```

**Files to modify:**
- `Cargo.toml` — add `device` feature
- `src/lib.rs` — gate `device` module, gate re-exports
- `src/packets/mod.rs` — gate parameter packet variants in `Packet` enum and `Packet::parse()`
- `src/packets/parameter_read.rs` — wrap in `#[cfg(feature = "device")]`
- `src/packets/parameter_write.rs` — same
- `src/packets/parameter_settings_entry.rs` — same

**Expected impact:**
- `Packet` enum size drops from ~700 bytes to ~100 bytes when `device` feature disabled
- No `DeviceManager` on embedded builds
- Binary size reduction on constrained targets

**Estimated effort:** Medium (3-5 hours)

---

### WS3: Replace `libm` with Fixed-Point LUT for Barometric Vertical Speed (Medium Priority)

**Goal:** Eliminate the `libm` dependency for users on FPU-less MCUs (Cortex-M0/M0+) or
flash-constrained devices.

**Current state:** `libm` is used only in `src/packets/baro_altitude.rs` for:
- `logf()` in `get_vertical_speed_packed()` (encode cm/s → packed i8)
- `powf()` in `get_vertical_speed_cm_s()` (decode packed i8 → cm/s)

The vertical speed packed value is `i8` (256 discrete values). The cm/s range is ±2500.

**Proposed approach:**

1. **Create a 256-entry LUT** mapping packed `i8` values to `i16` cm/s values
2. Use this LUT for decoding (replacing `powf`)
3. For encoding (replacing `logf`), use binary search on the LUT or a second inverse LUT
4. Feature-gate `libm` with a `float-math` (or `libm`) feature for users who want the
   original floating-point implementation

```rust
// Decode LUT: index = (packed_i8 + 128) as usize, value = cm/s
const VSPEED_LUT: [i16; 256] = [
    -2500, -2480, -2460, /* ... generated from the log/pow formulas ... */
];

pub fn get_vertical_speed_cm_s(packed: i8) -> i16 {
    VSPEED_LUT[(packed as i8 as i16 + 128) as usize]
}
```

**Files to modify:**
- `src/packets/baro_altitude.rs` — replace `libm` functions with LUT
- `Cargo.toml` — make `libm` optional behind a feature

**Trade-offs:**
- LUT uses ~512 bytes of flash (vs. `libm` which can be several KB of code)
- Slightly less precise (±1 cm/s rounding at boundaries)
- The encode direction (cm/s → packed) is trickier without logf; binary search on LUT is O(log 256) = O(8)

**Decision needed with user:** 
- Is the LUT approach acceptable, or should we keep `libm` as default with LUT as optional?
- Should we feature-gate `libm` or just replace it entirely?

**Estimated effort:** Small-Medium (2-3 hours)

---

### WS4: Consolidate CRC Tables and Add Configurable Buffer Sizes (Low Priority)

**Goal:** Small optimizations for flash-constrained targets and usability.

**4a. CRC Table Consolidation:**
- Currently there are two `CRC8_DVB_S2` instances: `const` in `parser.rs:251` and `static` in `packets/mod.rs:366`
- Consolidate to a single `static` instance in a shared location (e.g., `constants.rs`)
- Both modules import from the shared location

**4b. Configurable I/O Buffer Sizes:**
- Make `BLOCKING_IO_BUFFER_SIZE` and `ASYNC_IO_BUFFER_SIZE` configurable via const generics
- OR simply expose them as `pub const` so users can override with a feature flag

**Option A — Const generics (cleaner API):**
```rust
pub struct BlockingCrsfReader<R, const N: usize = 128> {
    parser: CrsfParser,
    reader: R,
    input_buffer: Deque<u8, N>,
}
```
This changes the type signature (breaking) but allows compile-time tuning.

**Option B — `pub const` (non-breaking):**
```rust
pub const BLOCKING_IO_BUFFER_SIZE: usize = 128;
```
Users can't easily override this without forking. Less useful.

**4c. Deduplicate `CRC8_DVB_S2` between parser and packet serializer:**
- Move to `constants.rs` as `pub(crate) static`

**Files to modify:**
- `src/constants.rs` — add shared CRC constant
- `src/parser.rs` — import shared CRC
- `src/packets/mod.rs` — import shared CRC
- `src/blocking_io.rs` — const generics (if chosen)
- `src/async_io.rs` — const generics (if chosen)

**Decision needed with user:** Should the I/O buffer use const generics (breaking change but more flexible)?

**Estimated effort:** Small (1-2 hours)

---

### WS5: Add `defmt::Format` to Device Module Types (Low Priority)

**Goal:** Complete `defmt` support for embedded debugging.

**Types missing `defmt::Format`:**
- `CrsfParser` (parser.rs)
- `DeviceManagerConfig` (device.rs)
- `Device` (device.rs) — may need manual impl due to `FnvIndexMap`
- `DeviceManager` (device.rs) — same
- `Parameter` (device.rs)
- `ParameterChunkReassembler` (parameter_settings_entry.rs)
- `ParameterDataType` (parameter_settings_entry.rs)

**Approach:** Add `#[cfg_attr(feature = "defmt", derive(defmt::Format))]` where possible.
For types containing `FnvIndexMap` (which may not implement `defmt::Format`), write manual
`impl defmt::Format` with summary output.

**Files to modify:**
- `src/parser.rs` — add defmt to `CrsfParser`
- `src/device.rs` — add defmt to `DeviceManagerConfig`, `Device`, `DeviceManager`, `Parameter`
- `src/packets/parameter_settings_entry.rs` — add defmt to `ParameterChunkReassembler`, `ParameterDataType`

**Estimated effort:** Small (1-2 hours)

---

### WS6: Update STM32 Demo and Add More Embedded Examples (Low Priority)

**Goal:** Ensure embedded examples are current and cover common platforms.

**6a. Update STM32 demo:**
- Bump `embedded-io-async` from `"0.6.1"` to `"0.7.0"` to match main crate
- Verify it builds with latest embassy

**6b. Consider additional examples:**
- nRF52 Embassy example (BLE CRSF bridge use case)
- ESP32 Embassy example
- Minimal bare-metal (no Embassy) example for Cortex-M

**Files to modify:**
- `examples/stm32demo/Cargo.toml` — bump `embedded-io-async` version

**Estimated effort:** Small-Medium (2-4 hours)

---

## Execution Order (Recommended)

```
Phase 1 — CI Foundation (independent, fast):
  WS1: CI no_std verification

Phase 2 — Core Optimizations (sequential, highest impact):
  WS2: Feature-gate DeviceManager and parameter protocol
  WS3: Replace libm with LUT

Phase 3 — Polish (parallel, low priority):
  WS4: CRC consolidation + configurable buffers
  WS5: defmt completeness
  WS6: Example updates
```

**Phase 1** can be done independently in a single commit.
**Phase 2** tasks are independent of each other (WS2, WS3 can be parallel).
**Phase 3** tasks are all independent and can be parallel.

---

## Key Decision Points for User

1. **WS2 approach:** Gate `Packet` enum variants with `#[cfg(feature = "device")]` (Option A, simpler) vs. split into `CorePacket` / `FullPacket` (Option B, cleaner)?
   - **Recommendation:** Option A — minimal API disruption, most users won't notice

2. **WS3 approach:** Replace `libm` entirely with LUT (simpler, smaller flash) vs. make `libm` optional with LUT default (more complex, backward compatible)?
   - **Recommendation:** LUT as default, `libm` as optional feature — best of both worlds

3. **WS4b approach:** Const generics for I/O buffer (breaking) vs. `pub const` (non-breaking)?
   - **Recommendation:** Const generics — the breaking change is minor and enables real flexibility

4. **Should the `device` feature be default-enabled?**
   - **Recommendation:** Yes — backward compatibility. Embedded users explicitly opt out with `default-features = false`.

---

## What This Plan Does NOT Cover (Out of Scope)

- **`Packet` enum boxing with `alloc`:** The library is explicitly allocator-free
- **DMA/interrupt integration:** Platform-specific, out of library scope
- **Additional packet types:** Separate from no_std improvements
- **The `todo.md` parameter tree state machine:** Separate feature work
- **Async runtime integration (Embassy/RTIC):** These are adapter concerns

---

## Verification Criteria

Each work stream should be verified by:

1. `cargo build` (host target, all features)
2. `cargo build --target thumbv7em-none-eabihf --no-default-features` (no_std, minimal)
3. `cargo build --target thumbv7em-none-eabihf --features defmt` (defmt only)
4. `cargo build --target thumbv7em-none-eabihf --features device` (device feature)
5. `cargo test --all-features` (full test suite)
6. `cargo clippy --all -- -D warnings` (no warnings)
7. Binary size comparison (before/after) for the STM32 demo
