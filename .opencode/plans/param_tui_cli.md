# Plan: Non-interactive CLI modes for `param_tui`

## Part 1: Analysis — What's Already in DeviceManager vs. What the TUI Adds

### DeviceManager already handles (Phases A–C almost complete):

| Concern | DeviceManager method | Notes |
|---------|---------------------|-------|
| Time tracking | `update_time()` | ✓ Complete |
| Packet processing | `handle_packet()` | ✓ Handles DeviceInformation, ParameterSettingsEntry, ParameterChunk |
| Auto-enqueue next param | `enqueue_next_parameter()` | ✓ Called internally after each param load |
| Auto-enqueue next chunk | `enqueue_next_chunk()` | ✓ Called internally for multi-chunk params |
| Chunk reassembly | `chunk_reassembler` | ✓ Automatic |
| Timeout/retry | `process_timeouts()` | ✓ Generates retry packets |
| Output drain | `drain_output()` | ✓ Returns auto-generated chunk/param requests |
| Param enumeration seed | `request_all_parameters()` | ✓ Seeds first param request |
| Write packets | `write_parameter()` | ✓ Generates ParameterWrite packets |
| Discovery pings | `send_device_ping()` | ✓ Rate-limited auto-ping |

### What the TUI loop adds on top (lines 701–879):

| Concern | Location | Can go into device.rs? |
|---------|----------|----------------------|
| **Phase A**: `mgr.update_time(time_ms)` | line 702 | Already in DM |
| **Phase B**: Read serial → parse → `handle_packet()` | lines 709-821 | Parser is external; the loop around handle_packet is ~100 lines of protocol logic |
| Phase B extra: `app.param_entries` tracking | lines 724-793 | ❌ UI-only — tracks per-param pending/retry/needs_reread for progress display. DM already has equivalent internal state in `pending_requests` and `device.parameters` |
| Phase B extra: `param_request_pending` flag | lines 730, 742, 748, 818 | ❌ Redundant — DM's `enqueue_next_parameter` already checks `pending_requests` for duplicates |
| Phase B extra: newly-loaded param detection | lines 760-793 | ❌ UI-only — scans DM's `device.parameters` after handle_packet to update `param_entries` UI state |
| **Phase C**: `process_timeouts()` + `drain_output()` → send | lines 826-837 | ✓ Already in DM — but caller must call both and combine results |
| **Phase D**: Parameter enumeration seeding | lines 841-879 | ⚠️ Almost — `request_all_parameters()` exists, but must be called externally. The initial seed after device discovery is NOT automatic |

### The two gaps in DeviceManager:

**Gap 1: Initial parameter enumeration is not auto-seeded.**
When `handle_device_info()` receives a `DeviceInformation` packet and creates a new `Device`, it does NOT start requesting parameters. The caller must manually call `request_all_parameters()`. The TUI handles this in Phase D (lines 864-878). This is the only protocol logic that's caller-side.

**Fix:** Add `self.enqueue_next_parameter(addr)` to `handle_device_info()`. After this, `drain_output()` will return the initial param request automatically. The entire Phase D in the TUI becomes unnecessary — the DeviceManager self-drives parameter enumeration from discovery to completion.

**Gap 2: No combined "give me everything to send" method.**
The caller must call `process_timeouts()` and `drain_output()` separately, then combine and send. This is a minor ergonomic gap.

**Fix:** Add `drain_all()` that returns retries + auto-output in one collection.

### What CANNOT go into device.rs (stays in example):

| Concern | Why |
|---------|-----|
| Serial I/O (`read_from_serial`, `send_packet_to_serial`) | Hardware-specific (`serialport` crate) |
| `CrsfParser` ownership | Parser is in the same crate but used independently; embedding it in DM would work but changes the API surface. Keeping it external is more flexible. |
| `app.param_entries` tracking | Pure UI state for progress display; DM already tracks equivalent info internally |
| `resolve_write_data()` | Uses `std::String` for errors, `f64::parse()` — `std`-only logic |
| TUI rendering, keyboard | Obviously UI-only |
| JSON serialization | `serde_json` — `std`-only |

---

## Part 2: Proposed Changes to `device.rs`

### Change 1: Auto-seed parameter enumeration in `handle_device_info()`

```rust
fn handle_device_info(&mut self, info: &DeviceInformation) {
    if let Ok(device) = Device::from_device_info(info) {
        let addr = device.address;
        // ... existing logging ...
        let _ = self.devices.insert(addr, device);
        
        // Auto-seed: request first parameter immediately after discovery.
        // Subsequent params are auto-enqueued by handle_parameter_entry/chunk.
        self.enqueue_next_parameter(addr);
    }
}
```

**Impact:** Eliminates Phase D from all callers. After this change, the complete parameter enumeration lifecycle is:
1. Caller feeds `DeviceInformation` packet → DM creates device, auto-requests param 0
2. Caller drains output, sends the request
3. DM receives param → `handle_parameter_entry` → auto-requests next param
4. Repeat until `device.parameters_loaded == true`

The TUI's Phase D block (lines 839-879) becomes just the status message update for UI — no protocol logic needed.

### Change 2: Add `drain_all()` convenience method

```rust
/// Returns all pending outgoing packets: retries from timeout processing
/// plus auto-generated requests (chunk requests, next-param requests).
///
/// Combines `process_timeouts()` and `drain_output()` into a single call.
/// This is the recommended way to get packets to send after each tick.
pub fn drain_all(&mut self) -> Vec<Vec<u8, { constants::CRSF_MAX_PACKET_SIZE }>, { MAX_PENDING_REQUESTS + MAX_PENDING_OUTPUT }> {
    let mut all = Vec::new();
    for pkt in self.process_timeouts() {
        let _ = all.push(pkt);
    }
    for pkt in self.drain_output() {
        let _ = all.push(pkt);
    }
    all
}
```

### Summary: Before vs. After for callers

**Before (TUI does ~170 lines of Phase B-D):**
```rust
// Phase B: parse + handle_packet + param_entries tracking (~100 lines)
// Phase C: process_timeouts + drain_output + send (~12 lines)
// Phase D: request_all_parameters seeding (~40 lines)
```

**After (caller does ~20 lines):**
```rust
// Parse incoming bytes and feed to manager
for packet in parser.iter_packets(&read_buf[..bytes_read]) {
    if let Ok(packet) = packet {
        manager.handle_packet(&packet);
    }
}

// Send everything the manager wants to transmit
for pkt in manager.drain_all() {
    send_packet_to_serial(port, &pkt)?;
}

// Update time (already one-liner)
manager.update_time(time_ms);
```

The TUI would additionally keep its `param_entries` for UI progress display, but that's purely cosmetic — not protocol logic.

---

## Part 3: CLI Architecture (in `param_tui.rs` only)

### CLI Structure (clap subcommands)

```rust
#[derive(Parser)]
#[command(name = "uf-crsf-param-tui", about = "CRSF/ELRS parameter browser & CLI tool")]
struct Args {
    #[arg(default_value = "/dev/ttyACM0", help = "Serial port path")]
    port: String,
    #[arg(long, default_value = "921600")]
    baud: u32,
    #[arg(long = "log-file", default_value = "/tmp/uf-crsf-tui.log")]
    log_file: String,
    #[arg(long, default_value = "10")]
    discovery_timeout: u64,

    #[command(subcommand)]
    command: Option<CliCommand>,
}

#[derive(Subcommand, Debug)]
enum CliCommand {
    /// Export device parameters as JSON
    Export {
        /// Include full schema (ranges, options, defaults)
        #[arg(long)]
        schema: bool,
        /// Output file path (default: stdout)
        #[arg(long, short)]
        output: Option<String>,
    },

    /// Write parameter value(s) by ID or name
    Set {
        /// "identifier=value" assignments (repeat for multiple writes).
        /// Identifier is param ID (number) or name (string).
        #[arg(long = "set", num_args = 1..)]
        assignments: Vec<String>,

        /// Write values from JSON file (same format as `export --schema`).
        /// Can be combined with --set; --set overrides JSON for same param.
        #[arg(long)]
        from_json: Option<String>,
    },
}
```

No subcommand → existing TUI (unchanged behavior).

### Extracted `process_tick()` for CLI

With the device.rs changes, the CLI's parameter loading loop becomes trivial:

```rust
fn load_all_params(app: &mut App, timeout_secs: u64) -> io::Result<Box<dyn SerialPort>> {
    let mut port = open_port(app)?;
    if !discover_device(app, &mut port, timeout_secs) {
        return Err(io::Error::other("Device discovery failed"));
    }

    // Spin until all parameters loaded
    let mut read_buf = [0u8; 512];
    loop {
        let time_ms = start_time().elapsed().as_millis() as u32;
        let bytes_read = read_from_serial(&mut port, &mut read_buf)?;

        let mut mgr = app.manager.lock().unwrap();
        mgr.update_time(time_ms);
        
        if bytes_read > 0 {
            let mut parser = app.parser.lock().unwrap();
            for pkt in parser.iter_packets(&read_buf[..bytes_read]).flatten() {
                mgr.handle_packet(&pkt);
            }
        }
        
        let outgoing = mgr.drain_all();
        let all_loaded = app.selected_device.is_some_and(|addr| {
            mgr.get_device(addr).is_some_and(|d| d.parameters_loaded)
        });
        drop(mgr);

        for pkt in outgoing {
            let _ = send_packet_to_serial(&mut port, &pkt);
        }

        if all_loaded {
            return Ok(port);
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}
```

### TUI refactoring with device.rs changes

`run_tui()` simplifies: Phase D is eliminated entirely. Phase B still needs the `param_entries` tracking for UI progress display (loading bar, retry counts), but the protocol logic is just `handle_packet()` + `drain_all()`. The TUI keeps `param_entries` as UI-only state, not protocol state.

The `param_request_pending` flag can be removed — it was guarding against redundant `request_all_parameters()` calls, but with auto-seeding, that's no longer needed.

### Write validation: `resolve_write_data()`

Extract from `apply_edit()` (lines 1006-1071):

```rust
fn resolve_write_data(param: &Parameter, input: &str) -> Result<Vec<u8>, String> {
    match &param.data {
        Some(ParameterData::Float { min, max, decimal_point, .. }) => {
            // parse f64 → multiply by 10^decimal_point → range check → to_le_bytes()
        }
        Some(ParameterData::TextSelection { options, min, max, .. }) => {
            // try u8 index first, then case-insensitive match against options.split(';')
        }
        Some(ParameterData::String { .. }) => Ok(input.as_bytes().to_vec()),
        Some(ParameterData::Command { .. }) => Ok(vec![0]),
        _ => Err(format!("Parameter '{}' is not writable", param.name)),
    }
}
```

Both `apply_edit()` (TUI) and `run_set()` (CLI) call this.

### JSON format, export, set — same as previous plan

(See the ExportRoot/ExportParam structs and run_export/run_set implementations in the previous plan draft. Those remain unchanged.)

---

## Part 4: Implementation Order

### Step 1: device.rs — Auto-seed in `handle_device_info()`
Add `self.enqueue_next_parameter(addr)` after device insertion. Run `just ci` to verify no regressions.

### Step 2: device.rs — Add `drain_all()` method
Combine `process_timeouts()` + `drain_output()`. Run `just ci`.

### Step 3: param_tui.rs — Refactor TUI to use `drain_all()`
Replace the Phase C two-call pattern with single `drain_all()`. Simplify/remove Phase D to just status messages. Remove `param_request_pending` flag. Verify TUI still works identically.

### Step 4: param_tui.rs — Extract `resolve_write_data()` from `apply_edit()`
Refactor `apply_edit()` to call it.

### Step 5: param_tui.rs — Add `load_all_params()` for CLI
Uses `open_port()`, `discover_device()`, then the simplified poll loop.

### Step 6: param_tui.rs — Add CLI subcommands
Add `serde`/`serde_json` to dev-deps. Add `CliCommand` enum, restructure `Args`, add JSON types, implement `run_export()` and `run_set()`.

### Step 7: Wire up `main()` dispatch
Match on `args.command`.

---

## Part 5: Files to Modify

| File | Changes |
|------|---------|
| `src/device.rs` | Auto-seed in `handle_device_info()`, add `drain_all()` |
| `Cargo.toml` | Add `serde`, `serde_json` to `[dev-dependencies]` |
| `examples/param_tui.rs` | CLI subcommands, `load_all_params()`, `resolve_write_data()`, JSON export, `run_export()`, `run_set()` |

## Summary of All New Functions

### In `device.rs` (library):
| Function | Purpose |
|----------|---------|
| Modified `handle_device_info()` | Auto-seed param enumeration after discovery |
| `drain_all()` | Combined timeout retries + auto-output |

### In `param_tui.rs` (example):
| Function | Purpose | Reuses |
|----------|---------|--------|
| `resolve_write_data()` | Validate user input against param schema → wire bytes | Extracted from `apply_edit()` |
| `load_all_params()` | Connect + discover + wait for all params | `open_port()`, `discover_device()`, DM API |
| `param_to_export()` | Convert `Parameter` → JSON export struct | `App::format_param_value()` |
| `param_type_name()` | `ParameterData` → type name string | — |
| `parse_assignment()` | Split "id=value" string | — |
| `resolve_param_id()` | Find param by ID or name | DM's `get_device()` + `iter_parameters()` |
| `run_export()` | Export CLI command | `load_all_params()`, `param_to_export()` |
| `run_set()` | Write CLI command | `load_all_params()`, `resolve_write_data()`, `resolve_param_id()` |
