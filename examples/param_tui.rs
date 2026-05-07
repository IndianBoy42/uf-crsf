//! CRSF Parameter TUI - Interactive parameter browser for CRSF/ELRS devices
//!
//! Connects to an ELRS transmitter (or any CRSF device) over serial and
//! provides a ratatui-based terminal UI to browse and modify parameters.
//!
//! Usage:
//!   cargo run --example param_tui -- /dev/ttyACM0
//!   cargo run --example param_tui -- /dev/ttyACM0 --baud 115200 --timeout 5
//!   cargo run --example param_tui -- --port /dev/ttyUSB0 --log-file /tmp/crsf.log

use std::fs::OpenOptions;
use std::io::{self, Write};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use log::{debug, error, info, trace, warn, LevelFilter, Log, Metadata, Record};

use clap::{Parser, Subcommand};
use crossterm::{
    event::{self, Event, KeyCode, KeyEventKind, KeyModifiers},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{
    backend::CrosstermBackend,
    layout::{Constraint, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, ListState, Paragraph, StatefulWidget, Wrap},
    Frame, Terminal,
};

use indexmap::IndexMap;
use serde::Serialize;
use serialport::SerialPort;
use uf_crsf::device::{DeviceManager, DeviceManagerConfig, Parameter};
use uf_crsf::packets::{write_packet_to_buffer, Packet, PacketAddress, ParameterData};
use uf_crsf::parser::CrsfParser;

/// Max retries for a single parameter read before giving up.
const PARAM_MAX_RETRIES: u8 = 3;

/// Per-parameter tracking state for the enumeration and reread scheduler.
#[derive(Debug, Clone)]
struct ParamEntry {
    /// How many consecutive read errors for this param.
    retries: u8,
    /// True if this param still needs to be read (initial scan or reread after write).
    pending: bool,
    /// Set after a successful write to signal that we need to reread the current value.
    needs_reread: bool,
}

impl Default for ParamEntry {
    fn default() -> Self {
        Self {
            retries: 0,
            pending: true,
            needs_reread: false,
        }
    }
}

#[derive(Parser)]
#[command(
    name = "uf-crsf-param-tui",
    about = "CRSF/ELRS parameter browser & CLI tool"
)]
struct Args {
    #[arg(default_value = "/dev/ttyACM0", help = "Serial port path")]
    port: String,
    #[arg(long, default_value = "921600", help = "Serial baud rate")]
    baud: u32,
    #[arg(
        long = "log-file",
        default_value = "/tmp/uf-crsf-tui.log",
        help = "Log file path"
    )]
    log_file: String,
    #[arg(
        long = "timeout",
        default_value = "10",
        help = "Device discovery timeout (seconds)"
    )]
    discovery_timeout: u64,

    #[command(subcommand)]
    command: Option<CliCommand>,
}

#[derive(Subcommand, Debug, Clone)]
enum CliCommand {
    /// Export device parameters as JSON
    Export {
        /// Export format: values (current values only), schema (type/range info only), full (both)
        #[arg(long, default_value = "schema", value_name = "FORMAT")]
        format: ExportFormat,
        /// Output file path (default: stdout)
        #[arg(long, short)]
        output: Option<String>,
        /// Get a single parameter by name or numeric ID
        #[arg(long, value_name = "IDENT")]
        get: Option<String>,
        /// JSON schema file from a previous `export --format schema/full`.
        /// When used with --get, only the requested parameter(s) are queried
        /// by ID, skipping full enumeration.
        #[arg(long, value_name = "FILE")]
        from_schema: Option<String>,
    },
    /// Write parameter value(s) by ID or name
    Set {
        /// "identifier=value" assignments (repeat for multiple writes).
        /// Identifier is param ID (number) or name (string).
        #[arg(long = "set", num_args = 1..)]
        assignments: Vec<String>,
        /// Write values from JSON file (same format as `export --format values`).
        /// Can be combined with --set; --set overrides JSON for same param.
        #[arg(long)]
        from_json: Option<String>,
        /// JSON schema file from a previous `export --format schema/full`.
        /// When provided, writes are sent directly by param ID without
        /// needing to enumerate all device parameters first.
        #[arg(long, value_name = "FILE")]
        from_schema: Option<String>,
        /// After writing, read back parameter(s) to verify the change.
        /// For command parameters, polls until the state returns to Ready.
        #[arg(long)]
        check: bool,
        /// Auto-confirm command parameters that need confirmation
        /// (sends CONFIRM when state is CONFIRMATION_NEEDED).
        /// Implies --check.
        #[arg(long, short = 'y')]
        confirm: bool,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, clap::ValueEnum)]
enum ExportFormat {
    Values,
    Schema,
    Full,
}

/// Logger that writes all log records to a file.
///
/// Used alongside stderr output during the pre-TUI phase.
/// During TUI (alternate screen), stderr is hidden but the file
/// still captures everything for post-mortem debugging.
struct FileLogger {
    file: Mutex<std::fs::File>,
}

impl Log for FileLogger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        if let Ok(mut file) = self.file.lock() {
            let elapsed = start_time().elapsed();
            let _ = writeln!(
                file,
                "[{:>5}][{:>3}.{:03}] {}",
                record.level(),
                elapsed.as_secs(),
                elapsed.subsec_millis(),
                record.args()
            );
            let _ = file.flush();
        }
    }

    fn flush(&self) {
        if let Ok(mut file) = self.file.lock() {
            let _ = file.flush();
        }
    }
}

fn start_time() -> Instant {
    use std::sync::OnceLock;
    static START: OnceLock<Instant> = OnceLock::new();
    *START.get_or_init(Instant::now)
}

fn init_logging(log_file: &str) {
    let file = OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true)
        .open(log_file)
        .expect("Failed to open log file");
    let logger = Box::new(FileLogger {
        file: Mutex::new(file),
    });
    log::set_logger(Box::leak(logger)).expect("Failed to set logger");
    log::set_max_level(LevelFilter::Trace);
    info!("Log started");
}

struct App {
    manager: Arc<Mutex<DeviceManager>>,
    parser: Arc<Mutex<CrsfParser>>,
    list_state: ListState,
    selected_device: Option<PacketAddress>,
    current_folder: u8,
    breadcrumb: Vec<(u8, String)>,
    editing: bool,
    edit_buffer: String,
    confirming_command: bool,
    status_message: String,
    connected: bool,
    port_path: String,
    baud_rate: u32,
    params_loaded: bool,
    param_request_pending: bool,
    /// Per-parameter tracking: param_id → entry state.
    param_entries: IndexMap<u8, ParamEntry>,
    /// Total number of parameters the device reports (`parameters_total` from DeviceInformation).
    param_total: u8,
}

impl App {
    fn new(port_path: String, baud_rate: u32) -> Self {
        let config = DeviceManagerConfig {
            timeout_ms: 500,
            retry_count: 3,
            device_ping_interval_ms: 0,
        };
        let manager = DeviceManager::new(config).with_address(PacketAddress::Handset);

        Self {
            manager: Arc::new(Mutex::new(manager)),
            parser: Arc::new(Mutex::new(CrsfParser::new())),
            list_state: ListState::default(),
            selected_device: None,
            current_folder: 0,
            breadcrumb: vec![(0, "ROOT".to_string())],
            editing: false,
            edit_buffer: String::new(),
            confirming_command: false,
            status_message: "Discovering devices...".to_string(),
            connected: false,
            port_path,
            baud_rate,
            params_loaded: false,
            param_request_pending: false,
            param_entries: IndexMap::new(),
            param_total: 0,
        }
    }

    fn get_current_parameters(&self) -> Vec<(u8, Parameter)> {
        let mgr = self.manager.lock().unwrap();
        let Some(dev_addr) = self.selected_device else {
            return Vec::new();
        };
        let Some(device) = mgr.get_device(dev_addr) else {
            return Vec::new();
        };

        if self.current_folder == 0 && self.breadcrumb.len() == 1 {
            if let Some(root) = device.root_folder() {
                if let Some(children) = root.folder_children() {
                    let mut params = Vec::new();
                    for &child_id in children {
                        if let Some(p) = device.get_parameter(child_id) {
                            params.push((child_id, p.clone()));
                        }
                    }
                    return params;
                }
            }
            let mut params: Vec<(u8, Parameter)> = device
                .iter_parameters()
                .map(|p| (p.id, p.clone()))
                .collect();
            params.sort_by_key(|(id, _)| *id);
            return params;
        }

        let mut params: Vec<(u8, Parameter)> = device
            .parameters_in_folder(self.current_folder)
            .map(|p| (p.id, p.clone()))
            .collect();
        params.sort_by_key(|(id, _)| *id);
        params
    }

    fn enter_folder(&mut self, param_id: u8, name: String) {
        self.breadcrumb.push((param_id, name));
        self.current_folder = param_id;
        self.list_state.select(Some(0));
    }

    fn go_back(&mut self) {
        if self.breadcrumb.len() > 1 {
            self.breadcrumb.pop();
            let (id, _) = self.breadcrumb.last().unwrap();
            self.current_folder = *id;
            self.list_state.select(Some(0));
        }
    }

    /// Called when device info arrives — seeds the param_entries map with
    /// entries for all declared parameter IDs.
    fn ensure_param_entries(&mut self, total: u8) {
        self.param_total = total;
        for id in 0..total {
            self.param_entries.entry(id).or_default();
        }
    }

    /// Count how many parameters have been successfully loaded vs skipped.
    fn param_progress(&self) -> (usize, usize) {
        let loaded = self
            .param_entries
            .iter()
            .filter(|(_, e)| !e.pending)
            .count();
        let skipped = self
            .param_entries
            .iter()
            .filter(|(_, e)| e.pending && e.retries >= PARAM_MAX_RETRIES)
            .count();
        (loaded, skipped)
    }

    fn format_param_value(param: &Parameter) -> String {
        match &param.data {
            Some(ParameterData::Float {
                value,
                unit,
                decimal_point,
                ..
            }) => {
                let divisor = 10_i32.pow(*decimal_point as u32);
                format!(
                    "{:.prec$} {}",
                    *value as f64 / divisor as f64,
                    unit,
                    prec = *decimal_point as usize
                )
            }
            Some(ParameterData::TextSelection { options, value, .. }) => {
                let opts: Vec<&str> = options.split(';').collect();
                if let Some(sel) = opts.get(*value as usize) {
                    sel.to_string()
                } else {
                    format!("index {}", value)
                }
            }
            Some(ParameterData::String { value, .. }) => value.to_string(),
            Some(ParameterData::Info { info }) => info.to_string(),
            Some(ParameterData::Command { status, .. }) => cmd_status_name(*status).to_string(),
            Some(ParameterData::Folder { children }) => format!("[{} items]", children.len()),
            Some(ParameterData::Vtx { data }) => format!("{:02X?}", data.as_slice()),
            None => "?".to_string(),
        }
    }

    fn format_param_detail(&self, param: &Parameter) -> Vec<Line<'static>> {
        let mut lines = Vec::new();
        lines.push(Line::from(Span::styled(
            format!("Parameter: {} (ID {})", param.name, param.id),
            Style::default().add_modifier(Modifier::BOLD),
        )));
        if param.hidden {
            lines.push(Line::from(Span::styled(
                "[HIDDEN]",
                Style::default().fg(Color::Yellow),
            )));
        }

        match &param.data {
            Some(ParameterData::Float {
                value,
                min,
                max,
                default,
                decimal_point,
                step_size,
                unit,
            }) => {
                let d = 10_i32.pow(*decimal_point as u32);
                let prec = *decimal_point as usize;
                lines.push(Line::from("Type: Float".to_string()));
                lines.push(Line::from(format!(
                    "Value: {:.prec$} {}",
                    *value as f64 / d as f64,
                    unit
                )));
                lines.push(Line::from(format!(
                    "Range: {:.prec$} .. {:.prec$}",
                    *min as f64 / d as f64,
                    *max as f64 / d as f64
                )));
                lines.push(Line::from(format!(
                    "Default: {:.prec$} {}",
                    *default as f64 / d as f64,
                    unit
                )));
                lines.push(Line::from(format!(
                    "Step: {:.prec$} {}",
                    *step_size as f64 / d as f64,
                    unit
                )));
                lines.push(Line::from(""));
                lines.push(Line::from(Span::styled(
                    "Press Enter to edit",
                    Style::default().fg(Color::Cyan),
                )));
            }
            Some(ParameterData::TextSelection {
                options,
                value,
                min,
                max,
                default,
                ..
            }) => {
                lines.push(Line::from("Type: TextSelection"));
                lines.push(Line::from(format!("Current: {} (index {})", value, value)));
                lines.push(Line::from("Options:"));
                for (i, opt) in options.split(';').enumerate() {
                    let marker = if i as u8 == *value { " <<<" } else { "" };
                    let style = if i as u8 == *value {
                        Style::default()
                            .fg(Color::Green)
                            .add_modifier(Modifier::BOLD)
                    } else {
                        Style::default()
                    };
                    lines.push(Line::from(Span::styled(
                        format!("  [{}] {}{}", i, opt, marker),
                        style,
                    )));
                }
                lines.push(Line::from(format!(
                    "Range: {} .. {}, Default: {}",
                    min, max, default
                )));
                lines.push(Line::from(""));
                lines.push(Line::from(Span::styled(
                    "Press Enter to select option index",
                    Style::default().fg(Color::Cyan),
                )));
            }
            Some(ParameterData::String { value, max_length }) => {
                lines.push(Line::from("Type: String"));
                lines.push(Line::from(format!("Value: {}", value)));
                lines.push(Line::from(format!("Max length: {}", max_length)));
                lines.push(Line::from(""));
                lines.push(Line::from(Span::styled(
                    "Press Enter to edit",
                    Style::default().fg(Color::Cyan),
                )));
            }
            Some(ParameterData::Command {
                status,
                timeout,
                info,
            }) => {
                lines.push(Line::from("Type: Command"));
                lines.push(Line::from(format!("Info: {}", info)));
                lines.push(Line::from(format!(
                    "Status: {} | Timeout: {}ms",
                    cmd_status_name(*status),
                    *timeout as u32 * 100
                )));
                lines.push(Line::from(""));
                match *status {
                    CMD_STATUS_READY => {
                        lines.push(Line::from(Span::styled(
                            "Press Enter to execute",
                            Style::default().fg(Color::Cyan),
                        )));
                    }
                    CMD_STATUS_PROGRESS => {
                        lines.push(Line::from(Span::styled(
                            "Command running... (p for poll)",
                            Style::default().fg(Color::Yellow),
                        )));
                    }
                    CMD_STATUS_CONFIRMATION_NEEDED => {
                        lines.push(Line::from(Span::styled(
                            format!("{} — Confirm? [y]es / [n]o", info),
                            Style::default().fg(Color::Yellow),
                        )));
                    }
                    _ => {
                        lines.push(Line::from(Span::styled(
                            "Press Enter to execute",
                            Style::default().fg(Color::Cyan),
                        )));
                    }
                }
            }
            Some(ParameterData::Folder { children }) => {
                lines.push(Line::from("Type: Folder"));
                let mgr = self.manager.lock().unwrap();
                if let Some(dev_addr) = self.selected_device {
                    if let Some(device) = mgr.get_device(dev_addr) {
                        let loaded_count = children
                            .iter()
                            .filter(|&&id| device.get_parameter(id).is_some())
                            .count();
                        let unloaded = children.len() - loaded_count;
                        lines.push(Line::from(format!(
                            "Items ({}){}",
                            loaded_count,
                            if unloaded > 0 {
                                format!("  ({} not loaded)", unloaded)
                            } else {
                                String::new()
                            },
                        )));
                        for &child_id in children.iter() {
                            if let Some(child) = device.get_parameter(child_id) {
                                let icon = match &child.data {
                                    Some(ParameterData::Folder { .. }) => "\u{25B6}",
                                    Some(ParameterData::Float { .. }) => "\u{25CB}",
                                    Some(ParameterData::TextSelection { .. }) => "\u{25C7}",
                                    Some(ParameterData::String { .. }) => "\u{25A1}",
                                    Some(ParameterData::Command { .. }) => "\u{25A0}",
                                    Some(ParameterData::Info { .. }) => "\u{2139}",
                                    Some(ParameterData::Vtx { .. }) => "\u{25B2}",
                                    None => "?",
                                };
                                let val = Self::format_param_value(child);
                                lines.push(Line::from(Span::styled(
                                    format!("  {} [{}] {}  {}", icon, child_id, child.name, val),
                                    Style::default().fg(Color::Cyan),
                                )));
                            }
                        }
                    }
                }
                lines.push(Line::from(""));
                lines.push(Line::from(Span::styled(
                    "Press Enter to navigate into",
                    Style::default().fg(Color::Cyan),
                )));
            }
            Some(ParameterData::Info { info }) => {
                lines.push(Line::from("Type: Info (read-only)"));
                lines.push(Line::from(format!("Value: {}", info)));
            }
            Some(ParameterData::Vtx { data }) => {
                lines.push(Line::from("Type: VTX"));
                lines.push(Line::from(format!("Data: {:02X?}", data.as_slice())));
            }
            None => {
                lines.push(Line::from("No data loaded"));
            }
        }
        lines
    }
}

fn send_packet_to_serial(port: &mut Box<dyn SerialPort>, packet_bytes: &[u8]) -> io::Result<()> {
    debug!(
        "TX: sending {} bytes: {:02X?}",
        packet_bytes.len(),
        packet_bytes
    );
    port.write_all(packet_bytes)?;
    // port.flush()?;
    Ok(())
}

fn read_from_serial(port: &mut Box<dyn SerialPort>, buf: &mut [u8]) -> io::Result<usize> {
    match port.read(buf) {
        Ok(n) => {
            if n > 0 {
                debug!("RX: read {n} bytes");
                trace!("{buf:x?}");
            }
            Ok(n)
        }
        Err(ref e) if e.kind() == io::ErrorKind::TimedOut => Ok(0),
        Err(e) => {
            warn!("RX read error: {}", e);
            Err(e)
        }
    }
}

fn build_ping_packet() -> Option<Vec<u8>> {
    use uf_crsf::packets::DevicePing;
    let ping =
        DevicePing::new(PacketAddress::Broadcast as u8, PacketAddress::Handset as u8).ok()?;
    let mut buffer = [0u8; 64];
    let len = write_packet_to_buffer(&mut buffer, PacketAddress::Broadcast, &ping).ok()?;
    Some(buffer[..len].to_vec())
}

fn build_param_write_packet(
    device_addr: PacketAddress,
    param_id: u8,
    data: &[u8],
) -> Option<Vec<u8>> {
    use uf_crsf::packets::ParameterWrite;
    let write = ParameterWrite::new(
        device_addr as u8,
        PacketAddress::Handset as u8,
        param_id,
        data,
    )
    .ok()?;
    let mut buffer = [0u8; 64];
    let len = write_packet_to_buffer(&mut buffer, device_addr, &write).ok()?;
    Some(buffer[..len].to_vec())
}

fn try_packet_addr(v: u8) -> Option<PacketAddress> {
    use num_enum::TryFromPrimitive;
    PacketAddress::try_from_primitive(v).ok()
}

/// Runs the pre-TUI discovery phase in normal terminal mode, then
/// transitions to the interactive TUI once a device is found.
fn run(app: &mut App, timeout_secs: u64) -> io::Result<()> {
    // ── Phase 1: Open serial port ──────────────────────────────────
    let mut port = open_port(app)?;

    // ── Phase 2: Discover device ───────────────────────────────────
    if !discover_device(app, &mut port, timeout_secs) {
        eprintln!();
        eprintln!("Troubleshooting:");
        eprintln!("  1. Is the device powered on?");
        eprintln!(
            "  2. Is the serial port correct? (current: {})",
            app.port_path
        );
        eprintln!("     Try: ls /dev/ttyACM* /dev/ttyUSB*");
        eprintln!(
            "  3. Is the baud rate correct? (current: {})",
            app.baud_rate
        );
        eprintln!("  4. Permission denied? Fix: sudo usermod -aG dialout $USER");
        eprintln!("  5. Is the device in use by another process?");
        eprintln!();
        return Err(io::Error::other("Device discovery failed"));
    }

    // ── Phase 3: Enter TUI ─────────────────────────────────────────
    let device_name = {
        let mgr = app.manager.lock().unwrap();
        mgr.get_device(app.selected_device.unwrap())
            .map(|d| d.name.clone().to_string())
            .unwrap_or_else(|| "Device".to_string())
    };
    eprintln!("\n✓ {} found! Entering TUI...", device_name);
    info!("Device found, entering TUI");
    run_tui(app, &mut port)
}

fn open_port(app: &App) -> io::Result<Box<dyn SerialPort>> {
    eprint!("Opening {} @ {} baud... ", app.port_path, app.baud_rate);
    match serialport::new(&app.port_path, app.baud_rate)
        .timeout(Duration::from_millis(50))
        .open()
    {
        Ok(p) => {
            eprintln!("OK");
            info!("Serial port opened");
            Ok(p)
        }
        Err(e) => {
            eprintln!("FAILED");
            error!("Failed to open {}: {}", app.port_path, e);
            eprintln!("\nError: Failed to open {}: {}", app.port_path, e);
            eprintln!();
            eprintln!("Possible causes:");
            eprintln!("  - Device not plugged in or powered on");
            eprintln!("  - Wrong port (check: ls /dev/ttyACM* /dev/ttyUSB*)");
            eprintln!("  - Permission denied (fix: sudo usermod -aG dialout $USER)");
            eprintln!("  - Device in use by another process");
            Err(io::Error::other(format!(
                "Failed to open {}",
                app.port_path
            )))
        }
    }
}

/// Sends pings and waits for a device response (visible stderr output).
/// Returns true if a device was found within the timeout.
fn discover_device(app: &mut App, port: &mut Box<dyn SerialPort>, timeout_secs: u64) -> bool {
    app.connected = true;
    let discover_start = Instant::now();
    let mut last_ping = Instant::now();
    let mut read_buf = [0u8; 512];

    eprint!("Discovering device");
    info!("Starting device discovery");

    let mut first = true;
    loop {
        let now = Instant::now();

        // Send pings every 1s
        if first || now.duration_since(last_ping) >= Duration::from_secs(1) {
            first = false;
            last_ping = now;
            let elapsed = discover_start.elapsed().as_secs();
            eprint!(".");
            debug!("Discovery ping at {}s", elapsed);
            if let Some(ping) = build_ping_packet() {
                let _ = send_packet_to_serial(port, &ping);
            }
            if elapsed >= timeout_secs {
                eprintln!(" (no response after {}s)", elapsed);
                warn!("Device discovery timed out after {}s", elapsed);
                return false;
            }
        }

        // Read serial with short polling
        match read_from_serial(port, &mut read_buf) {
            Ok(0) => {}
            Ok(bytes_read) => {
                let mut parser = app.parser.lock().unwrap();
                let mut mgr = app.manager.lock().unwrap();
                for packet in parser.iter_packets(&read_buf[..bytes_read]).flatten() {
                    if let Packet::DeviceInformation(info) = &packet {
                        let addr =
                            try_packet_addr(info.src_addr).unwrap_or(PacketAddress::Transmitter);
                        let param_total = info.parameters_total;
                        let device_name = info.device_name().to_string();
                        let src_addr = info.src_addr;
                        mgr.handle_packet(&packet);
                        drop(parser);
                        drop(mgr);
                        app.selected_device = Some(addr);
                        app.ensure_param_entries(param_total);
                        info!(
                            "Discovered: {} (0x{:02X}), {} params",
                            device_name, src_addr, param_total
                        );
                        return true;
                    }
                    mgr.handle_packet(&packet);
                }
            }
            e => {
                eprintln!("{e:?}");
            }
        }

        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Opens the serial port, discovers a device, and loads all parameters.
///
/// Returns the open serial port on success so callers can continue using it
/// (e.g., for writing parameters).
fn load_all_params(app: &mut App, timeout_secs: u64) -> io::Result<Box<dyn SerialPort>> {
    let mut port = open_port(app)?;
    if !discover_device(app, &mut port, timeout_secs) {
        return Err(io::Error::other("Device discovery failed"));
    }

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
        let all_loaded = app
            .selected_device
            .is_some_and(|addr| mgr.get_device(addr).is_some_and(|d| d.parameters_loaded));
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

/// RAII guard that restores terminal state on drop.
struct TerminalGuard;

impl TerminalGuard {
    fn enter() -> io::Result<Self> {
        enable_raw_mode()?;
        execute!(io::stdout(), EnterAlternateScreen)?;
        Ok(Self)
    }
}

impl Drop for TerminalGuard {
    fn drop(&mut self) {
        let _ = disable_raw_mode();
        let _ = execute!(io::stdout(), LeaveAlternateScreen);
    }
}

fn run_tui(app: &mut App, port: &mut Box<dyn SerialPort>) -> io::Result<()> {
    let _guard = TerminalGuard::enter()?;

    let stdout = io::stdout();
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    app.status_message = format!("Connected to {} @ {} baud", app.port_path, app.baud_rate);

    let mut read_buf = [0u8; 512];
    let tick_rate = Duration::from_millis(50);

    loop {
        let time_ms = start_time().elapsed().as_millis() as u32;

        {
            let mut mgr = app.manager.lock().unwrap();
            mgr.update_time(time_ms);
        }

        match read_from_serial(port, &mut read_buf) {
            Ok(bytes_read) if bytes_read > 0 => {
                // Phase 1: parse packets (borrows app.parser only)
                let packets: Vec<_> = {
                    let mut parser = app.parser.lock().unwrap();
                    parser.iter_packets(&read_buf[..bytes_read]).collect()
                };

                // Phase 2: process packets
                for packet_result in packets {
                    match packet_result {
                        Ok(ref packet) => {
                            debug!("Parsed packet: {:x?}", packet);

                            // Pre-extract info that needs &mut app before we borrow mgr
                            match packet {
                                Packet::ParameterSettingsEntry(entry)
                                    if entry.chunks_remaining == 0 =>
                                {
                                    let pid = entry.parameter_number;
                                    info!("Param {} loaded: {}", pid, entry.name);
                                    app.param_request_pending = false;
                                    if let Some(ent) = app.param_entries.get_mut(&pid) {
                                        ent.pending = false;
                                        ent.needs_reread = false;
                                        ent.retries = 0;
                                    }
                                }
                                Packet::ParameterChunk(chunk) if chunk.chunks_remaining == 0 => {
                                    // Final chunk of a chunked parameter — reassembly may
                                    // complete inside handle_packet below
                                    info!("Param {} final chunk received", chunk.param_number);
                                    app.param_request_pending = false;
                                }
                                Packet::ParameterWrite(write) => {
                                    // Device echoes the write back with the updated value
                                    let pid = write.parameter_number;
                                    info!("Param {} write acknowledged", pid);
                                    app.param_request_pending = false;
                                    if let Some(ent) = app.param_entries.get_mut(&pid) {
                                        ent.needs_reread = false;
                                    }
                                }
                                Packet::DeviceInformation(info) => {
                                    app.ensure_param_entries(info.parameters_total);
                                }
                                _ => {}
                            }

                            // Run handle_packet and detect if new params appeared
                            // (handles both single-chunk and multi-chunk assembly)
                            let newly_loaded: Vec<u8> = {
                                let mut mgr = app.manager.lock().unwrap();
                                mgr.handle_packet(packet);
                                // Scan for parameters that the DeviceManager loaded but
                                // our param_entries still thinks are pending
                                if let Some(dev_addr) = app.selected_device {
                                    if let Some(device) = mgr.get_device(dev_addr) {
                                        device
                                            .parameters
                                            .keys()
                                            .copied()
                                            .filter(|pid| {
                                                app.param_entries
                                                    .get(pid)
                                                    .is_some_and(|e| e.pending)
                                            })
                                            .collect()
                                    } else {
                                        Vec::new()
                                    }
                                } else {
                                    Vec::new()
                                }
                            };

                            for pid in newly_loaded {
                                info!("Param {} loaded (chunked)", pid);
                                app.param_request_pending = false;
                                if let Some(ent) = app.param_entries.get_mut(&pid) {
                                    ent.pending = false;
                                    ent.needs_reread = false;
                                    ent.retries = 0;
                                }
                            }
                        }
                        Err(_e) => {
                            // Find the param we were waiting for and bump its retry counter
                            let failed_id = app
                                .param_entries
                                .iter()
                                .find(|(_, e)| e.pending)
                                .map(|(&id, _)| id);
                            if let Some(pid) = failed_id {
                                if let Some(ent) = app.param_entries.get_mut(&pid) {
                                    ent.retries += 1;
                                    if ent.retries >= PARAM_MAX_RETRIES {
                                        info!(
                                            "Param {} failed after {} retries, will retry later",
                                            pid, PARAM_MAX_RETRIES
                                        );
                                    } else {
                                        info!(
                                            "Param {} error, will retry (attempt {})",
                                            pid, ent.retries
                                        );
                                    }
                                }
                            }
                            app.param_request_pending = false;
                        }
                    }
                }
            }
            _ => {}
        }

        {
            let mut mgr = app.manager.lock().unwrap();
            let outgoing = mgr.drain_all();
            drop(mgr);
            for pkt in outgoing {
                let _ = send_packet_to_serial(port, &pkt);
            }
        }

        // Parameter enumeration: auto-seeded by DeviceManager on discovery.
        // Just check for completion to update the UI status.
        if let Some(dev_addr) = app.selected_device {
            let mgr = app.manager.lock().unwrap();

            let is_loaded = mgr
                .get_device(dev_addr)
                .is_some_and(|d| d.parameters_loaded);

            if is_loaded && !app.params_loaded {
                app.params_loaded = true;
                let (loaded, skipped) = app.param_progress();
                app.status_message = if skipped > 0 {
                    format!(
                        "Parameters enumerated ({} loaded, {} skipped)",
                        loaded, skipped
                    )
                } else {
                    format!("All {} parameters loaded", loaded)
                };
                info!(
                    "Parameter enumeration complete: {} loaded, {} skipped",
                    loaded, skipped
                );
            } else if !is_loaded && !app.param_request_pending {
                let (loaded, skipped) = app.param_progress();
                app.status_message = format!(
                    "Requesting parameters... ({} loaded, {} skipped)",
                    loaded, skipped
                );
            }
        }

        // Keyboard
        while event::poll(Duration::from_millis(0))? {
            if let Event::Key(key) = event::read()? {
                if key.kind != KeyEventKind::Press {
                    continue;
                }
                if app.confirming_command {
                    match key.code {
                        KeyCode::Char('y') | KeyCode::Enter => {
                            app.confirming_command = false;
                            execute_command(app, port);
                        }
                        KeyCode::Char('n') => {
                            app.confirming_command = false;
                            cancel_command(app, port);
                        }
                        KeyCode::Esc => {
                            app.confirming_command = false;
                        }
                        _ => {}
                    }
                } else if app.editing {
                    match key.code {
                        KeyCode::Enter => apply_edit(app, port),
                        KeyCode::Esc => {
                            app.editing = false;
                            app.edit_buffer.clear();
                        }
                        KeyCode::Backspace => {
                            app.edit_buffer.pop();
                        }
                        KeyCode::Char(c) => app.edit_buffer.push(c),
                        KeyCode::Up | KeyCode::Down => {
                            // For TextSelection parameters, cycle through options
                            let params = app.get_current_parameters();
                            let selected = app.list_state.selected().unwrap_or(0);
                            if selected < params.len() {
                                let (_id, param) = &params[selected];
                                if let Some(ParameterData::TextSelection {
                                    options,
                                    value,
                                    min,
                                    max,
                                    ..
                                }) = &param.data
                                {
                                    let opts: Vec<&str> = options.split(';').collect();
                                    // Determine current index from the edit buffer
                                    let current_idx = if app.edit_buffer.is_empty() {
                                        *value
                                    } else if let Ok(idx) = app.edit_buffer.parse::<u8>() {
                                        idx
                                    } else {
                                        opts.iter()
                                            .position(|o| o.eq_ignore_ascii_case(&app.edit_buffer))
                                            .unwrap_or(*value as usize)
                                            as u8
                                    };
                                    let new_idx = match key.code {
                                        KeyCode::Up => {
                                            if current_idx > *min {
                                                current_idx - 1
                                            } else {
                                                *max
                                            }
                                        }
                                        KeyCode::Down => {
                                            if current_idx < *max {
                                                current_idx + 1
                                            } else {
                                                *min
                                            }
                                        }
                                        _ => unreachable!(),
                                    };
                                    app.edit_buffer = opts
                                        .get(new_idx as usize)
                                        .copied()
                                        .unwrap_or("?")
                                        .to_string();
                                }
                            }
                        }
                        _ => {}
                    }
                } else {
                    if key.code == KeyCode::Char('c') && key.modifiers == KeyModifiers::CONTROL {
                        return Ok(());
                    }
                    match key.code {
                        KeyCode::Char('q') | KeyCode::Esc => return Ok(()),
                        KeyCode::Down => {
                            let params = app.get_current_parameters();
                            let max = if params.is_empty() {
                                0
                            } else {
                                params.len() - 1
                            };
                            let current = app.list_state.selected().unwrap_or(0);
                            app.list_state
                                .select(Some(current.saturating_add(1).min(max)));
                        }
                        KeyCode::Up => {
                            let current = app.list_state.selected().unwrap_or(0);
                            app.list_state.select(Some(current.saturating_sub(1)));
                        }
                        KeyCode::Right | KeyCode::Char(' ') => handle_select(app),
                        KeyCode::Left => app.go_back(),
                        KeyCode::Char('p') => {
                            poll_command(app, port);
                        }
                        KeyCode::Char('r') => {
                            if let Some(dev_addr) = app.selected_device {
                                app.params_loaded = false;
                                app.param_request_pending = false;
                                app.status_message = "Refreshing parameters...".to_string();
                                // Reset all entries to pending for a full rescan
                                for entry in app.param_entries.values_mut() {
                                    entry.retries = 0;
                                    entry.pending = true;
                                    entry.needs_reread = false;
                                }
                                let mut mgr = app.manager.lock().unwrap();
                                if let Some(device) = mgr.get_device_mut(dev_addr) {
                                    device.parameters.clear();
                                    device.parameters_loaded = false;
                                }
                                // Stale pending chunk requests will be cleaned up by
                                // process_timeouts; new enumeration re-seeds via
                                // request_all_parameters once param_request_pending is false.
                            }
                        }
                        _ => {}
                    }
                }
            }
        }

        terminal.draw(|f| ui(f, app))?;
        std::thread::sleep(tick_rate);
    }
}

fn handle_select(app: &mut App) {
    let params = app.get_current_parameters();
    let selected = app.list_state.selected().unwrap_or(0);
    if selected >= params.len() {
        return;
    }
    let (param_id, param) = &params[selected];

    match &param.data {
        Some(ParameterData::Folder { .. }) => {
            app.enter_folder(*param_id, param.name.to_string());
        }
        Some(ParameterData::Command { status, .. }) => match *status {
            CMD_STATUS_READY => {
                app.confirming_command = true;
            }
            CMD_STATUS_CONFIRMATION_NEEDED => {
                app.confirming_command = true;
            }
            CMD_STATUS_PROGRESS => {
                // Already running — do nothing on select, use 'p' to poll
            }
            _ => {
                app.confirming_command = true;
            }
        },
        Some(ParameterData::TextSelection { options, value, .. }) => {
            app.editing = true;
            // Pre-fill edit buffer with the current option name so
            // up/down arrows can cycle from a known starting point.
            let opts: Vec<&str> = options.split(';').collect();
            app.edit_buffer = opts
                .get(*value as usize)
                .copied()
                .unwrap_or("?")
                .to_string();
        }
        Some(ParameterData::Float { .. }) | Some(ParameterData::String { .. }) => {
            app.editing = true;
            app.edit_buffer.clear();
        }
        _ => {}
    }
}

/// Command status codes (CRSF spec 0x2B command payload).
const CMD_STATUS_READY: u8 = 0;
const CMD_STATUS_START: u8 = 1;
const CMD_STATUS_PROGRESS: u8 = 2;
const CMD_STATUS_CONFIRMATION_NEEDED: u8 = 3;
const CMD_STATUS_CONFIRM: u8 = 4;
const CMD_STATUS_CANCEL: u8 = 5;
const CMD_STATUS_POLL: u8 = 6;

fn cmd_status_name(status: u8) -> &'static str {
    match status {
        CMD_STATUS_READY => "Ready",
        CMD_STATUS_START => "Start",
        CMD_STATUS_PROGRESS => "In Progress",
        CMD_STATUS_CONFIRMATION_NEEDED => "Confirmation Needed",
        CMD_STATUS_CONFIRM => "Confirmed",
        CMD_STATUS_CANCEL => "Cancelled",
        CMD_STATUS_POLL => "Poll",
        _ => "Unknown",
    }
}

/// Validates user input against a parameter's schema and returns the
/// wire-format bytes for a ParameterWrite packet.
///
/// Returns `Ok(data)` if the input is valid, or `Err(message)` with a
/// human-readable error string.
fn resolve_write_data(param: &Parameter, input: &str) -> Result<Vec<u8>, String> {
    match &param.data {
        Some(ParameterData::Float {
            min,
            max,
            decimal_point,
            ..
        }) => {
            let min = *min;
            let max = *max;
            let decimal_point = *decimal_point;
            let val: f64 = input
                .parse()
                .map_err(|_| format!("Invalid number: {}", input))?;
            let divisor = 10_i32.pow(decimal_point as u32) as f64;
            let int_val = (val * divisor) as i32;
            if int_val < min || int_val > max {
                return Err(format!(
                    "Value {} out of range [{}, {}]",
                    val,
                    min as f64 / divisor,
                    max as f64 / divisor
                ));
            }
            Ok(int_val.to_le_bytes().to_vec())
        }
        Some(ParameterData::TextSelection {
            options, min, max, ..
        }) => {
            let min = *min;
            let max = *max;
            match input.parse::<u8>() {
                Ok(idx) if idx >= min && idx <= max => Ok(vec![idx]),
                Ok(idx) => Err(format!("Index {} out of range [{}, {}]", idx, min, max)),
                Err(_) => {
                    let opts: Vec<&str> = options.split(';').collect();
                    if let Some(pos) = opts.iter().position(|o| o.eq_ignore_ascii_case(input)) {
                        let pos_u8 = pos as u8;
                        if pos_u8 >= min && pos_u8 <= max {
                            Ok(vec![pos_u8])
                        } else {
                            Err(format!("Option '{}' out of range", input))
                        }
                    } else {
                        Err(format!(
                            "Invalid option: '{}'. Use index 0-{} or option name",
                            input, max
                        ))
                    }
                }
            }
        }
        Some(ParameterData::String { .. }) => Ok(input.as_bytes().to_vec()),
        Some(ParameterData::Command { status, .. }) => {
            let current = *status;
            match input.to_lowercase().as_str() {
                "start" => {
                    if current != CMD_STATUS_READY {
                        return Err(format!(
                            "Command is not ready (current: {})",
                            cmd_status_name(current)
                        ));
                    }
                    Ok(vec![CMD_STATUS_START])
                }
                "confirm" | "yes" | "y" => Ok(vec![CMD_STATUS_CONFIRM]),
                "cancel" | "no" | "n" => Ok(vec![CMD_STATUS_CANCEL]),
                "poll" => Ok(vec![CMD_STATUS_POLL]),
                other => Err(format!(
                    "Invalid command action: '{}'. Use start, confirm, cancel, or poll",
                    other
                )),
            }
        }
        _ => Err(format!("Parameter '{}' is not writable", param.name)),
    }
}

fn apply_edit(app: &mut App, port: &mut Box<dyn SerialPort>) {
    app.editing = false;
    let input = app.edit_buffer.clone();
    app.edit_buffer.clear();

    let params = app.get_current_parameters();
    let selected = app.list_state.selected().unwrap_or(0);
    if selected >= params.len() {
        return;
    }
    let (_, param) = &params[selected];
    let Some(dev_addr) = app.selected_device else {
        return;
    };

    let write_data = match resolve_write_data(param, &input) {
        Ok(data) => data,
        Err(msg) => {
            app.status_message = msg;
            return;
        }
    };

    let pid = param.id;
    info!(
        "Writing parameter {} ({} bytes) to device 0x{:02X}",
        pid,
        write_data.len(),
        dev_addr as u8
    );
    if let Some(pkt) = build_param_write_packet(dev_addr, pid, &write_data) {
        match send_packet_to_serial(port, &pkt) {
            Ok(()) => {
                app.status_message =
                    format!("Sent write for param {} ({} bytes)", pid, write_data.len());
                let mut mgr = app.manager.lock().unwrap();
                if let Some(device) = mgr.get_device_mut(dev_addr) {
                    device.parameters.remove(&pid);
                    device.parameters_loaded = false;
                }
                if let Some(reread_pkt) = mgr.request_parameter(dev_addr, pid, 0) {
                    drop(mgr);
                    let _ = send_packet_to_serial(port, &reread_pkt);
                    app.param_request_pending = true;
                    if let Some(entry) = app.param_entries.get_mut(&pid) {
                        entry.pending = true;
                        entry.needs_reread = true;
                    }
                }
            }
            Err(e) => {
                app.status_message = format!("Write error: {}", e);
            }
        }
    }
}

fn execute_command(app: &mut App, port: &mut Box<dyn SerialPort>) {
    let params = app.get_current_parameters();
    let selected = app.list_state.selected().unwrap_or(0);
    if selected >= params.len() {
        return;
    }
    let (param_id, param) = &params[selected];
    let Some(dev_addr) = app.selected_device else {
        return;
    };
    let Some(ParameterData::Command { status, .. }) = &param.data else {
        return;
    };
    let pid = *param_id;
    let action = if *status == CMD_STATUS_CONFIRMATION_NEEDED {
        CMD_STATUS_CONFIRM
    } else {
        CMD_STATUS_START
    };
    info!(
        "Executing command {} (action={}) to device 0x{:02X}",
        pid, action, dev_addr as u8
    );
    if let Some(pkt) = build_param_write_packet(dev_addr, pid, &[action]) {
        match send_packet_to_serial(port, &pkt) {
            Ok(()) => {
                app.status_message = format!("Command {} sent ({})", pid, cmd_status_name(action));
                let mut mgr = app.manager.lock().unwrap();
                if let Some(device) = mgr.get_device_mut(dev_addr) {
                    device.parameters.remove(&pid);
                    device.parameters_loaded = false;
                }
                if let Some(reread_pkt) = mgr.request_parameter(dev_addr, pid, 0) {
                    drop(mgr);
                    let _ = send_packet_to_serial(port, &reread_pkt);
                    app.param_request_pending = true;
                    if let Some(entry) = app.param_entries.get_mut(&pid) {
                        entry.pending = true;
                        entry.needs_reread = true;
                    }
                }
            }
            Err(e) => {
                app.status_message = format!("Command error: {}", e);
            }
        }
    }
}

/// Cancels a command that is in CONFIRMATION_NEEDED state.
fn cancel_command(app: &mut App, port: &mut Box<dyn SerialPort>) {
    let params = app.get_current_parameters();
    let selected = app.list_state.selected().unwrap_or(0);
    if selected >= params.len() {
        return;
    }
    let (param_id, param) = &params[selected];
    let Some(dev_addr) = app.selected_device else {
        return;
    };
    let Some(ParameterData::Command { .. }) = &param.data else {
        return;
    };
    let pid = *param_id;
    info!("Cancelling command {} to device 0x{:02X}", pid, dev_addr as u8);
    if let Some(pkt) = build_param_write_packet(dev_addr, pid, &[CMD_STATUS_CANCEL]) {
        match send_packet_to_serial(port, &pkt) {
            Ok(()) => {
                app.status_message = format!("Command {} cancelled", pid);
                let mut mgr = app.manager.lock().unwrap();
                if let Some(device) = mgr.get_device_mut(dev_addr) {
                    device.parameters.remove(&pid);
                    device.parameters_loaded = false;
                }
                if let Some(reread_pkt) = mgr.request_parameter(dev_addr, pid, 0) {
                    drop(mgr);
                    let _ = send_packet_to_serial(port, &reread_pkt);
                    app.param_request_pending = true;
                    if let Some(entry) = app.param_entries.get_mut(&pid) {
                        entry.pending = true;
                        entry.needs_reread = true;
                    }
                }
            }
            Err(e) => {
                app.status_message = format!("Cancel error: {}", e);
            }
        }
    }
}

/// Sends a POLL for a command parameter to get its latest status.
fn poll_command(app: &mut App, port: &mut Box<dyn SerialPort>) {
    let params = app.get_current_parameters();
    let selected = app.list_state.selected().unwrap_or(0);
    if selected >= params.len() {
        return;
    }
    let (param_id, param) = &params[selected];
    let Some(dev_addr) = app.selected_device else {
        return;
    };
    let Some(ParameterData::Command { .. }) = &param.data else {
        return;
    };
    let pid = *param_id;
    if let Some(pkt) = build_param_write_packet(dev_addr, pid, &[CMD_STATUS_POLL]) {
        if send_packet_to_serial(port, &pkt).is_ok() {
            let mut mgr = app.manager.lock().unwrap();
            if let Some(device) = mgr.get_device_mut(dev_addr) {
                device.parameters.remove(&pid);
                device.parameters_loaded = false;
            }
            if let Some(reread_pkt) = mgr.request_parameter(dev_addr, pid, 0) {
                drop(mgr);
                let _ = send_packet_to_serial(port, &reread_pkt);
                app.param_request_pending = true;
                if let Some(entry) = app.param_entries.get_mut(&pid) {
                    entry.pending = true;
                    entry.needs_reread = true;
                }
            }
        }
    }
}

fn ui(f: &mut Frame, app: &App) {
    let chunks = Layout::vertical([Constraint::Min(3), Constraint::Length(3)]).split(f.area());

    let main_chunks = Layout::horizontal([Constraint::Percentage(55), Constraint::Percentage(45)])
        .split(chunks[0]);

    draw_param_list(f, app, main_chunks[0]);
    draw_detail_panel(f, app, main_chunks[1]);
    draw_status_bar(f, app, chunks[1]);
}

fn draw_param_list(f: &mut Frame, app: &App, area: Rect) {
    let breadcrumb_str: String = app
        .breadcrumb
        .iter()
        .map(|(_, name)| name.as_str())
        .collect::<Vec<_>>()
        .join(" > ");

    let params = app.get_current_parameters();

    let items: Vec<ListItem> = params
        .iter()
        .map(|(id, param)| {
            let type_icon = match &param.data {
                Some(ParameterData::Folder { .. }) => "\u{25B6}",
                Some(ParameterData::Float { .. }) => "\u{25CB}",
                Some(ParameterData::TextSelection { .. }) => "\u{25C7}",
                Some(ParameterData::String { .. }) => "\u{25A1}",
                Some(ParameterData::Command { .. }) => "\u{25A0}",
                Some(ParameterData::Info { .. }) => "\u{2139}",
                Some(ParameterData::Vtx { .. }) => "\u{25B2}",
                None => "?",
            };

            let value_str = App::format_param_value(param);
            let hidden = if param.hidden { " [H]" } else { "" };

            let line = Line::from(vec![
                Span::styled(
                    format!("{:<2} ", type_icon),
                    Style::default().fg(Color::DarkGray),
                ),
                Span::styled(
                    format!("[{:>2}] ", id),
                    Style::default().fg(Color::DarkGray),
                ),
                Span::styled(param.name.to_string(), Style::default()),
                Span::raw(hidden.to_string()),
                Span::raw("  "),
                Span::styled(value_str, Style::default().fg(Color::Cyan)),
            ]);

            ListItem::new(line)
        })
        .collect();

    let title = if app.selected_device.is_some() {
        format!(" {} ", breadcrumb_str)
    } else {
        " Parameters ".to_string()
    };

    let list = List::new(items)
        .block(Block::default().borders(Borders::ALL).title(title))
        .highlight_style(Style::default().add_modifier(Modifier::REVERSED));

    StatefulWidget::render(list, area, f.buffer_mut(), &mut app.list_state.clone());
}

fn draw_detail_panel(f: &mut Frame, app: &App, area: Rect) {
    let params = app.get_current_parameters();
    let selected = app.list_state.selected().unwrap_or(0);

    let content = if selected < params.len() {
        let (_, param) = &params[selected];
        app.format_param_detail(param)
    } else if app.selected_device.is_none() {
        vec![
            Line::from(""),
            Line::from(Span::styled(
                "No device",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            )),
            Line::from(""),
            Line::from("Device discovery failed or was skipped."),
            Line::from("Restart the application to retry."),
        ]
    } else if !app.params_loaded {
        vec![
            Line::from(""),
            Line::from(Span::styled(
                "Loading parameters...",
                Style::default().fg(Color::Yellow),
            )),
            Line::from(""),
            Line::from("Parameters are being fetched from the device."),
            Line::from("This may take a few seconds."),
        ]
    } else {
        vec![Line::from("Select a parameter from the list")]
    };

    let detail = Paragraph::new(content)
        .block(Block::default().borders(Borders::ALL).title(" Detail "))
        .wrap(Wrap { trim: true });

    f.render_widget(detail, area);

    if app.confirming_command {
        let (text, title) = if selected < params.len() {
            if let Some(ParameterData::Command {
                status,
                info,
                ..
            }) = &params[selected].1.data
            {
                if *status == CMD_STATUS_CONFIRMATION_NEEDED {
                    (
                        format!("{} — Confirm? [y]es / [n]o", info),
                        " Confirm Required (Esc to dismiss) ",
                    )
                } else {
                    (
                        format!("Execute '{}'? [y]es / [n]o", params[selected].1.name),
                        " Execute Command (Esc to cancel) ",
                    )
                }
            } else {
                (
                    "Execute this command? [y]es / [n]o".to_string(),
                    " Confirm (Esc to cancel) ",
                )
            }
        } else {
            (
                "Execute this command? [y]es / [n]o".to_string(),
                " Confirm (Esc to cancel) ",
            )
        };
        let confirm = Paragraph::new(text)
            .style(Style::default().fg(Color::Yellow))
            .block(Block::default().borders(Borders::ALL).title(title));
        f.render_widget(
            confirm,
            Rect {
                x: area.x + 2,
                y: area.y + area.height.saturating_sub(5),
                width: area.width.saturating_sub(4),
                height: 3,
            },
        );
    } else if app.editing {
        let is_text_selection = selected < params.len()
            && matches!(
                params[selected].1.data,
                Some(ParameterData::TextSelection { .. })
            );
        let edit_title = if is_text_selection {
            " Select option (\u{2191}\u{2193} cycle, Enter confirm, Esc cancel) "
        } else {
            " Enter value (Esc to cancel) "
        };
        let input = Paragraph::new(format!("> {}", app.edit_buffer))
            .style(Style::default().fg(Color::Yellow))
            .block(Block::default().borders(Borders::ALL).title(edit_title));
        f.render_widget(
            input,
            Rect {
                x: area.x + 2,
                y: area.y + area.height.saturating_sub(5),
                width: area.width.saturating_sub(4),
                height: 3,
            },
        );
    }
}

fn draw_status_bar(f: &mut Frame, app: &App, area: Rect) {
    let device_info = if let Some(dev_addr) = app.selected_device {
        let mgr = app.manager.lock().unwrap();
        if let Some(device) = mgr.get_device(dev_addr) {
            let (loaded, skipped) = app.param_progress();
            format!(
                "{} | 0x{:02X} | Params: {}/{}{}{}",
                device.name,
                dev_addr as u8,
                loaded,
                app.param_total,
                if skipped > 0 {
                    format!(" ({} skipped)", skipped)
                } else {
                    String::new()
                },
                if app.params_loaded { " [LOADED]" } else { "" }
            )
        } else {
            format!("0x{:02X}", dev_addr as u8)
        }
    } else {
        "No device".to_string()
    };

    let (conn_indicator, conn_style) = if app.selected_device.is_some() {
        ("DEVICE FOUND", Style::default().fg(Color::Green))
    } else if app.connected {
        ("PORT OPEN", Style::default().fg(Color::Yellow))
    } else {
        ("DISCONNECTED", Style::default().fg(Color::Red))
    };

    let line = Line::from(vec![
        Span::styled(format!(" {} ", conn_indicator), conn_style),
        Span::raw(" | "),
        Span::styled(device_info, Style::default()),
        Span::raw(" | "),
        Span::styled(
            app.status_message.clone(),
            Style::default().fg(Color::Yellow),
        ),
        Span::raw(" | "),
        Span::styled(
            "q/Ctrl-C:Quit  \u{2191}\u{2193}:Nav  Enter:Edit  Backspace:Back  r:Refresh  p:Poll",
            Style::default().fg(Color::DarkGray),
        ),
    ]);

    let status = Paragraph::new(line).block(Block::default().borders(Borders::ALL));
    f.render_widget(status, area);
}

// -----------------------------------------------------------------------
// JSON export types & CLI helpers
// -----------------------------------------------------------------------

#[derive(Serialize)]
struct ExportRoot {
    device: ExportDevice,
    parameters: Vec<ExportNode>,
}

#[derive(Serialize)]
struct ExportDevice {
    name: String,
    address: String,
    serial_number: u32,
    hardware_id: u32,
    firmware_id: u32,
    parameter_version: u8,
    parameters_total: u8,
}

#[derive(Serialize)]
#[serde(tag = "type")]
enum ExportNode {
    #[serde(rename = "folder")]
    Folder {
        id: u8,
        name: String,
        #[serde(skip_serializing_if = "Option::is_none")]
        hidden: Option<bool>,
        children: Vec<ExportNode>,
    },
    #[serde(rename = "float")]
    Float {
        id: u8,
        name: String,
        #[serde(skip_serializing_if = "Option::is_none")]
        hidden: Option<bool>,
        #[serde(skip_serializing_if = "Option::is_none")]
        value: Option<f64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        unit: Option<String>,
        #[serde(skip_serializing_if = "Option::is_none")]
        min: Option<f64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        max: Option<f64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        default: Option<f64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        step: Option<f64>,
    },
    #[serde(rename = "text_selection")]
    TextSelection {
        id: u8,
        name: String,
        #[serde(skip_serializing_if = "Option::is_none")]
        hidden: Option<bool>,
        #[serde(skip_serializing_if = "Option::is_none")]
        value: Option<String>,
        #[serde(skip_serializing_if = "Option::is_none")]
        value_index: Option<u8>,
        #[serde(skip_serializing_if = "Option::is_none")]
        options: Option<Vec<String>>,
        #[serde(skip_serializing_if = "Option::is_none")]
        min: Option<u8>,
        #[serde(skip_serializing_if = "Option::is_none")]
        max: Option<u8>,
        #[serde(skip_serializing_if = "Option::is_none")]
        default: Option<u8>,
    },
    #[serde(rename = "string")]
    String_ {
        id: u8,
        name: String,
        #[serde(skip_serializing_if = "Option::is_none")]
        hidden: Option<bool>,
        #[serde(skip_serializing_if = "Option::is_none")]
        value: Option<String>,
        #[serde(skip_serializing_if = "Option::is_none")]
        max_length: Option<u8>,
    },
    #[serde(rename = "info")]
    Info {
        id: u8,
        name: String,
        #[serde(skip_serializing_if = "Option::is_none")]
        hidden: Option<bool>,
        #[serde(skip_serializing_if = "Option::is_none")]
        value: Option<String>,
    },
    #[serde(rename = "command")]
    Command {
        id: u8,
        name: String,
        #[serde(skip_serializing_if = "Option::is_none")]
        hidden: Option<bool>,
        #[serde(skip_serializing_if = "Option::is_none")]
        status: Option<String>,
        #[serde(skip_serializing_if = "Option::is_none")]
        status_code: Option<u8>,
        #[serde(skip_serializing_if = "Option::is_none")]
        timeout_ms: Option<u32>,
        #[serde(skip_serializing_if = "Option::is_none")]
        info: Option<String>,
    },
    #[serde(rename = "vtx")]
    Vtx {
        id: u8,
        name: String,
        #[serde(skip_serializing_if = "Option::is_none")]
        hidden: Option<bool>,
        #[serde(skip_serializing_if = "Option::is_none")]
        value: Option<String>,
    },
}

fn build_export_tree(device: &uf_crsf::device::Device, fmt: &ExportFormat) -> Vec<ExportNode> {
    build_folder_children(device, 0, fmt)
}

fn build_folder_children(
    device: &uf_crsf::device::Device,
    folder_id: u8,
    fmt: &ExportFormat,
) -> Vec<ExportNode> {
    let Some(folder) = device.get_parameter(folder_id) else {
        return Vec::new();
    };
    let Some(child_ids) = folder.folder_children() else {
        return Vec::new();
    };

    let mut nodes: Vec<ExportNode> = Vec::new();
    for &cid in child_ids.iter() {
        let Some(param) = device.get_parameter(cid) else {
            continue;
        };
        nodes.push(param_to_export_node(param, device, fmt));
    }
    nodes
}

fn param_to_export_node(
    param: &Parameter,
    device: &uf_crsf::device::Device,
    fmt: &ExportFormat,
) -> ExportNode {
    let include_values = fmt == &ExportFormat::Values || fmt == &ExportFormat::Full;
    let include_schema = fmt == &ExportFormat::Schema || fmt == &ExportFormat::Full;
    let hidden = if param.hidden { Some(true) } else { None };

    match &param.data {
        Some(ParameterData::Folder { .. }) => ExportNode::Folder {
            id: param.id,
            name: param.name.to_string(),
            hidden,
            children: build_folder_children(device, param.id, fmt),
        },
        Some(ParameterData::Float {
            value,
            min,
            max,
            default,
            step_size,
            decimal_point,
            unit,
            ..
        }) => {
            let d = 10_i32.pow(*decimal_point as u32) as f64;
            ExportNode::Float {
                id: param.id,
                name: param.name.to_string(),
                hidden,
                value: if include_values {
                    Some(*value as f64 / d)
                } else {
                    None
                },
                unit: if include_values || include_schema {
                    Some(unit.to_string())
                } else {
                    None
                },
                min: if include_schema {
                    Some(*min as f64 / d)
                } else {
                    None
                },
                max: if include_schema {
                    Some(*max as f64 / d)
                } else {
                    None
                },
                default: if include_schema {
                    Some(*default as f64 / d)
                } else {
                    None
                },
                step: if include_schema {
                    Some(*step_size as f64 / d)
                } else {
                    None
                },
            }
        }
        Some(ParameterData::TextSelection {
            options,
            value,
            min,
            max,
            default,
            ..
        }) => {
            let opts: Vec<&str> = options.split(';').collect();
            let value_name = opts.get(*value as usize).map(|s| s.to_string());
            ExportNode::TextSelection {
                id: param.id,
                name: param.name.to_string(),
                hidden,
                value: if include_values { value_name } else { None },
                value_index: if include_values { Some(*value) } else { None },
                options: if include_schema {
                    Some(opts.into_iter().map(|s| s.to_string()).collect())
                } else {
                    None
                },
                min: if include_schema { Some(*min) } else { None },
                max: if include_schema { Some(*max) } else { None },
                default: if include_schema { Some(*default) } else { None },
            }
        }
        Some(ParameterData::String { value, max_length }) => ExportNode::String_ {
            id: param.id,
            name: param.name.to_string(),
            hidden,
            value: if include_values {
                Some(value.to_string())
            } else {
                None
            },
            max_length: if include_schema { Some(*max_length) } else { None },
        },
        Some(ParameterData::Info { info }) => ExportNode::Info {
            id: param.id,
            name: param.name.to_string(),
            hidden,
            value: if include_values {
                Some(info.to_string())
            } else {
                None
            },
        },
        Some(ParameterData::Command {
            status, timeout, info, ..
        }) => ExportNode::Command {
            id: param.id,
            name: param.name.to_string(),
            hidden,
            status: if include_values {
                Some(cmd_status_name(*status).to_string())
            } else {
                None
            },
            status_code: if include_values { Some(*status) } else { None },
            timeout_ms: if include_schema {
                Some(*timeout as u32 * 100)
            } else {
                None
            },
            info: if include_schema || include_values {
                Some(info.to_string())
            } else {
                None
            },
        },
        Some(ParameterData::Vtx { data }) => ExportNode::Vtx {
            id: param.id,
            name: param.name.to_string(),
            hidden,
            value: if include_values {
                Some(format!("{:02X?}", data.as_slice()))
            } else {
                None
            },
        },
        None => ExportNode::Info {
            id: param.id,
            name: param.name.to_string(),
            hidden,
            value: if include_values {
                Some("(no data)".to_string())
            } else {
                None
            },
        },
    }
}

/// Flattens the nested export tree back to (id, name) pairs for name lookup.
fn flatten_export_nodes(nodes: &[serde_json::Value]) -> Vec<(u8, String)> {
    let mut result = Vec::new();
    for node in nodes {
        if let Some(id) = node.get("id").and_then(|v| v.as_u64()).map(|v| v as u8) {
            if let Some(name) = node.get("name").and_then(|v| v.as_str()) {
                result.push((id, name.to_string()));
            }
        }
        if let Some(children) = node.get("children").and_then(|v| v.as_array()) {
            result.extend(flatten_export_nodes(children));
        }
    }
    result
}

fn parse_assignment(s: &str) -> Option<(String, Option<String>)> {
    if let Some(pos) = s.find('=') {
        Some((s[..pos].to_string(), Some(s[pos + 1..].to_string())))
    } else {
        Some((s.to_string(), None))
    }
}

/// Resolves a parameter identifier (numeric ID or name) to a parameter ID.
fn resolve_param_id(app: &App, ident: &str) -> Option<u8> {
    if let Ok(id) = ident.parse::<u8>() {
        let mgr = app.manager.lock().unwrap();
        if let Some(dev_addr) = app.selected_device {
            if mgr
                .get_device(dev_addr)
                .and_then(|d| d.get_parameter(id))
                .is_some()
            {
                return Some(id);
            }
        }
    }
    let mgr = app.manager.lock().unwrap();
    if let Some(dev_addr) = app.selected_device {
        if let Some(device) = mgr.get_device(dev_addr) {
            for param in device.iter_parameters() {
                if param.name.eq_ignore_ascii_case(ident) {
                    return Some(param.id);
                }
            }
        }
    }
    None
}

fn run_export(
    app: &mut App,
    format: &ExportFormat,
    output: Option<&str>,
    get: Option<&str>,
    from_schema: Option<&str>,
    timeout: u64,
) -> io::Result<()> {
    if let Some(schema_path) = from_schema {
        return run_export_with_schema(app, format, output, get, schema_path, timeout);
    }

    eprintln!("Loading parameters...");
    let _port = load_all_params(app, timeout)?;

    let mgr = app.manager.lock().unwrap();
    let dev_addr = app.selected_device.unwrap();
    let device = mgr.get_device(dev_addr).unwrap();

    if let Some(ident) = get {
        let param_id = if let Ok(id) = ident.parse::<u8>() {
            if device.get_parameter(id).is_some() {
                Some(id)
            } else {
                None
            }
        } else {
            device
                .iter_parameters()
                .find(|p| p.name.eq_ignore_ascii_case(ident))
                .map(|p| p.id)
        };
        let Some(param_id) = param_id else {
            drop(mgr);
            eprintln!("No parameter matching '{}'", ident);
            std::process::exit(1);
        };
        let Some(param) = device.get_parameter(param_id) else {
            drop(mgr);
            eprintln!("Parameter {} not loaded", param_id);
            std::process::exit(1);
        };
        let node = param_to_export_node(param, device, format);
        drop(mgr);

        let json = serde_json::to_string_pretty(&node)
            .map_err(|e| io::Error::other(format!("JSON serialization failed: {}", e)))?;

        match output {
            Some(path) => {
                std::fs::write(path, &json)?;
                eprintln!("Exported to {}", path);
            }
            None => println!("{}", json),
        }
        return Ok(());
    }

    let export_device = ExportDevice {
        name: device.name.to_string(),
        address: format!("0x{:02X}", dev_addr as u8),
        serial_number: device.serial_number,
        hardware_id: device.hardware_id,
        firmware_id: device.firmware_id,
        parameter_version: device.parameter_version,
        parameters_total: device.parameters_total,
    };

    let nodes = build_export_tree(device, format);

    let root = ExportRoot {
        device: export_device,
        parameters: nodes,
    };

    drop(mgr);

    let json = serde_json::to_string_pretty(&root)
        .map_err(|e| io::Error::other(format!("JSON serialization failed: {}", e)))?;

    match output {
        Some(path) => {
            std::fs::write(path, &json)?;
            eprintln!("Exported to {}", path);
        }
        None => println!("{}", json),
    }

    Ok(())
}

fn run_set(
    app: &mut App,
    assignments: &[String],
    from_json: Option<&str>,
    from_schema: Option<&str>,
    timeout: u64,
    check: bool,
    confirm: bool,
) -> io::Result<()> {
    let check = check || confirm;
    if let Some(schema_path) = from_schema {
        return run_set_with_schema(app, assignments, from_json, schema_path, timeout, check, confirm);
    }

    eprintln!("Loading parameters...");
    let mut port = load_all_params(app, timeout)?;

    let dev_addr = app.selected_device.unwrap();

    let mut writes: Vec<(u8, Vec<u8>)> = Vec::new();

    if let Some(json_path) = from_json {
        let json_data = std::fs::read_to_string(json_path)?;
        let root: serde_json::Value = serde_json::from_str(&json_data)
            .map_err(|e| io::Error::other(format!("Invalid JSON: {}", e)))?;
        if let Some(params) = root.get("parameters").and_then(|p| p.as_array()) {
            let flat = flatten_export_nodes(params);
            for (id, _name) in flat {
                if let Some(val_str) = find_value_in_tree(params, id) {
                    let mgr = app.manager.lock().unwrap();
                    if let Some(device) = mgr.get_device(dev_addr) {
                        if let Some(param) = device.get_parameter(id) {
                            match resolve_write_data(param, &val_str) {
                                Ok(data) => writes.push((id, data)),
                                Err(e) => eprintln!("Skipping param {}: {}", id, e),
                            }
                        }
                    }
                }
            }
        }
    }

    for assignment in assignments {
        let Some((ident, value)) = parse_assignment(assignment) else {
            continue;
        };
        let Some(id) = resolve_param_id(app, &ident) else {
            eprintln!("Unknown parameter: '{}'", ident);
            continue;
        };
        let mgr = app.manager.lock().unwrap();
        if let Some(device) = mgr.get_device(dev_addr) {
            if let Some(param) = device.get_parameter(id) {
                if let Some(ref val) = value {
                    match resolve_write_data(param, val) {
                        Ok(data) => writes.push((id, data)),
                        Err(e) => eprintln!("Invalid value for param {}: {}", id, e),
                    }
                } else {
                    match &param.data {
                        Some(ParameterData::Command { status, .. }) => {
                            let action = if *status == CMD_STATUS_CONFIRMATION_NEEDED {
                                CMD_STATUS_CONFIRM
                            } else {
                                CMD_STATUS_START
                            };
                            writes.push((id, vec![action]));
                        }
                        _ => eprintln!(
                            "Parameter '{}' requires a value. Use {}=value",
                            ident, ident
                        ),
                    }
                }
            }
        }
    }

    if writes.is_empty() {
        eprintln!("No valid writes to perform");
        return Ok(());
    }

    for (pid, data) in &writes {
        info!(
            "Writing param {} ({} bytes) to device 0x{:02X}",
            pid,
            data.len(),
            dev_addr as u8
        );
        if let Some(pkt) = build_param_write_packet(dev_addr, *pid, data) {
            send_packet_to_serial(&mut port, &pkt)?;
            eprintln!("Sent write for param {}", pid);
        }
    }

    if check {
        verify_writes(app, &mut port, &writes, dev_addr, confirm)?;
    }

    Ok(())
}

fn prompt_yes_no(label: &str) -> io::Result<bool> {
    use std::io::Write;
    loop {
        eprint!("{} [y/n]: ", label);
        io::stderr().flush()?;
        let mut line = String::new();
        io::stdin().read_line(&mut line)?;
        match line.trim().to_lowercase().as_str() {
            "y" | "yes" => return Ok(true),
            "n" | "no" => return Ok(false),
            _ => eprintln!("Please enter y or n"),
        }
    }
}

fn verify_writes(
    app: &mut App,
    port: &mut Box<dyn SerialPort>,
    writes: &[(u8, Vec<u8>)],
    dev_addr: PacketAddress,
    confirm: bool,
) -> io::Result<()> {
    let ids: Vec<u8> = writes.iter().map(|(id, _)| *id).collect();
    let start = Instant::now();
    let max_duration = Duration::from_secs(30);
    let mut read_buf = [0u8; 512];
    let mut verified: Vec<u8> = Vec::new();

    // Request reread for each written parameter
    {
        let mut mgr = app.manager.lock().unwrap();
        for &pid in &ids {
            if let Some(device) = mgr.get_device_mut(dev_addr) {
                device.parameters.remove(&pid);
            }
            if let Some(pkt) = mgr.request_parameter(dev_addr, pid, 0) {
                send_packet_to_serial(port, &pkt)?;
            }
        }
    }

    eprint!("Verifying");
    while verified.len() < ids.len() && start.elapsed() < max_duration {
        eprint!(".");
        let time_ms = start_time().elapsed().as_millis() as u32;
        {
            let mut mgr = app.manager.lock().unwrap();
            mgr.update_time(time_ms);
        }

        {
            let mut mgr = app.manager.lock().unwrap();
            let outgoing = mgr.drain_all();
            drop(mgr);
            for pkt in outgoing {
                send_packet_to_serial(port, &pkt)?;
            }
        }

        let bytes_read = read_from_serial(port, &mut read_buf)?;
        if bytes_read > 0 {
            let mut parser = app.parser.lock().unwrap();
            let packets: Vec<_> = parser.iter_packets(&read_buf[..bytes_read]).collect();
            drop(parser);

            for packet_result in packets {
                if let Ok(ref packet) = packet_result {
                    let param_id = match packet {
                        Packet::ParameterSettingsEntry(entry)
                            if ids.contains(&entry.parameter_number) =>
                        {
                            Some(entry.parameter_number)
                        }
                        _ => None,
                    };

                    {
                        let mut mgr = app.manager.lock().unwrap();
                        mgr.handle_packet(packet);
                    }

                    if let Some(pid) = param_id {
                        if verified.contains(&pid) {
                            continue;
                        }

                        let status = {
                            let mgr = app.manager.lock().unwrap();
                            mgr.get_device(dev_addr)
                                .and_then(|d| d.get_parameter(pid))
                                .and_then(|p| match &p.data {
                                    Some(ParameterData::Command { status, .. }) => Some(*status),
                                    _ => None,
                                })
                        };

                        match status {
                            Some(CMD_STATUS_READY) => {
                                verified.push(pid);
                                eprintln!("\n  param {} = Ready", pid);
                            }
                            Some(CMD_STATUS_PROGRESS) => {
                                let mut mgr = app.manager.lock().unwrap();
                                if let Some(device) = mgr.get_device_mut(dev_addr) {
                                    device.parameters.remove(&pid);
                                }
                                if let Some(pkt) = mgr.request_parameter(dev_addr, pid, 0) {
                                    send_packet_to_serial(port, &pkt)?;
                                }
                            }
                            Some(CMD_STATUS_CONFIRMATION_NEEDED) => {
                                let action = if confirm {
                                    eprintln!("\n  param {} needs confirmation, auto-confirming", pid);
                                    CMD_STATUS_CONFIRM
                                } else {
                                    eprint!("\n  param {} needs confirmation", pid);
                                    let answer = prompt_yes_no("Confirm?")?;
                                    if answer {
                                        CMD_STATUS_CONFIRM
                                    } else {
                                        CMD_STATUS_CANCEL
                                    }
                                };
                                {
                                    let mut mgr = app.manager.lock().unwrap();
                                    if let Some(device) = mgr.get_device_mut(dev_addr) {
                                        device.parameters.remove(&pid);
                                    }
                                }
                                if let Some(pkt) = build_param_write_packet(dev_addr, pid, &[action]) {
                                    send_packet_to_serial(port, &pkt)?;
                                }
                            }
                            Some(_) => {
                                verified.push(pid);
                                eprintln!("\n  param {} = verified (unexpected status)", pid);
                            }
                            None => {
                                verified.push(pid);
                                let val = {
                                    let mgr = app.manager.lock().unwrap();
                                    mgr.get_device(dev_addr)
                                        .and_then(|d| d.get_parameter(pid))
                                        .map(|p| App::format_param_value(p))
                                        .unwrap_or_else(|| "?".to_string())
                                };
                                eprintln!("\n  param {} = {}", pid, val);
                            }
                        }
                    }
                }
            }
        }

        std::thread::sleep(Duration::from_millis(50));
    }

    eprintln!();
    for pid in &ids {
        if !verified.contains(pid) {
            eprintln!("  param {}: verification timed out", pid);
        }
    }

    Ok(())
}

/// Walks the nested JSON export tree to find a node by id and returns its value string.
fn find_value_in_tree(nodes: &[serde_json::Value], target_id: u8) -> Option<String> {
    for node in nodes {
        let id = node.get("id").and_then(|v| v.as_u64()).map(|v| v as u8);
        if id == Some(target_id) {
            if let Some(val) = node.get("value").and_then(|v| v.as_str()) {
                return Some(val.to_string());
            }
            if let Some(val) = node.get("value_index").and_then(|v| v.as_u64()) {
                if let Some(opts) = node.get("options").and_then(|v| v.as_array()) {
                    if let Some(opt) = opts.get(val as usize).and_then(|o| o.as_str()) {
                        return Some(opt.to_string());
                    }
                }
                return Some(val.to_string());
            }
            return None;
        }
        if let Some(children) = node.get("children").and_then(|v| v.as_array()) {
            if let Some(found) = find_value_in_tree(children, target_id) {
                return Some(found);
            }
        }
    }
    None
}

// -----------------------------------------------------------------------
// Schema-based (fast path) helpers
// -----------------------------------------------------------------------

/// Represents a parameter's schema extracted from a JSON export file.
#[derive(Debug, Clone)]
struct SchemaParam {
    id: u8,
    name: String,
    param_type: String,
    min: Option<f64>,
    max: Option<f64>,
    decimal_point: Option<u8>,
    options: Option<Vec<String>>,
    max_length: Option<u8>,
}

/// Parses a JSON schema file into a flat list of parameter schemas.
fn parse_schema_file(path: &str) -> io::Result<Vec<SchemaParam>> {
    let json_data = std::fs::read_to_string(path)?;
    let root: serde_json::Value = serde_json::from_str(&json_data)
        .map_err(|e| io::Error::other(format!("Invalid JSON in schema file: {}", e)))?;
    let params = root
        .get("parameters")
        .and_then(|p| p.as_array())
        .ok_or_else(|| io::Error::other("Schema JSON missing 'parameters' array"))?;
    let mut result = Vec::new();
    collect_schema_nodes(params, &mut result);
    Ok(result)
}

fn collect_schema_nodes(nodes: &[serde_json::Value], out: &mut Vec<SchemaParam>) {
    for node in nodes {
        let Some(id) = node.get("id").and_then(|v| v.as_u64()).map(|v| v as u8) else {
            continue;
        };
        let name = node
            .get("name")
            .and_then(|v| v.as_str())
            .unwrap_or("?")
            .to_string();
        let param_type = node
            .get("type")
            .and_then(|v| v.as_str())
            .unwrap_or("unknown")
            .to_string();

        let decimal_point = if param_type == "float" {
            let min_raw = node.get("min").and_then(|v| v.as_f64()).unwrap_or(0.0);
            let max_raw = node.get("max").and_then(|v| v.as_f64()).unwrap_or(0.0);
            infer_decimal_point(min_raw, max_raw)
        } else {
            None
        };

        let options = node
            .get("options")
            .and_then(|v| v.as_array())
            .map(|arr| arr.iter().filter_map(|v| v.as_str().map(|s| s.to_string())).collect());

        out.push(SchemaParam {
            id,
            name,
            param_type,
            min: node.get("min").and_then(|v| v.as_f64()),
            max: node.get("max").and_then(|v| v.as_f64()),
            decimal_point,
            options,
            max_length: node
                .get("max_length")
                .and_then(|v| v.as_u64())
                .map(|v| v as u8),
        });

        if let Some(children) = node.get("children").and_then(|v| v.as_array()) {
            collect_schema_nodes(children, out);
        }
    }
}

/// Infers the decimal_point value from inspecting the float representation
/// of min/max values in the schema. The schema outputs human-readable floats
/// (already divided by 10^decimal_point), so we count decimal places.
fn infer_decimal_point(min: f64, max: f64) -> Option<u8> {
    let check = if min.fract() != 0.0 { min } else { max };
    if check.fract() == 0.0 {
        return Some(0);
    }
    let s = format!("{}", check);
    let dp = s.find('.').map(|pos| (s.len() - pos - 1) as u8)?;
    Some(dp.min(4))
}

/// Resolves a parameter identifier (numeric ID or name) using schema data.
fn resolve_param_id_from_schema(schemas: &[SchemaParam], ident: &str) -> Option<u8> {
    if let Ok(id) = ident.parse::<u8>() {
        if schemas.iter().any(|s| s.id == id) {
            return Some(id);
        }
    }
    schemas
        .iter()
        .find(|s| s.name.eq_ignore_ascii_case(ident))
        .map(|s| s.id)
}

/// Validates user input against a schema parameter's type info and returns
/// wire-format bytes for a ParameterWrite packet.
fn resolve_write_data_from_schema(schema: &SchemaParam, input: &str) -> Result<Vec<u8>, String> {
    match schema.param_type.as_str() {
        "float" => {
            let val: f64 = input
                .parse()
                .map_err(|_| format!("Invalid number: {}", input))?;
            let dp = schema.decimal_point.unwrap_or(0) as u32;
            let divisor = 10_i32.pow(dp) as f64;
            let int_val = (val * divisor) as i32;
            if let (Some(min), Some(max)) = (schema.min, schema.max) {
                let min_int = (min * divisor) as i32;
                let max_int = (max * divisor) as i32;
                if int_val < min_int || int_val > max_int {
                    return Err(format!(
                        "Value {} out of range [{}, {}]",
                        val, min, max
                    ));
                }
            }
            Ok(int_val.to_le_bytes().to_vec())
        }
        "text_selection" => {
            let opts = schema.options.as_deref().unwrap_or(&[]);
            match input.parse::<u8>() {
                Ok(idx) => {
                    if !opts.is_empty() && (idx as usize) >= opts.len() {
                        return Err(format!(
                            "Index {} out of range (0-{})",
                            idx,
                            opts.len() - 1
                        ));
                    }
                    Ok(vec![idx])
                }
                Err(_) => {
                    if let Some(pos) = opts
                        .iter()
                        .position(|o| o.eq_ignore_ascii_case(input))
                    {
                        Ok(vec![pos as u8])
                    } else {
                        Err(format!(
                            "Invalid option: '{}'. Use index 0-{} or option name",
                            input,
                            opts.len().saturating_sub(1)
                        ))
                    }
                }
            }
        }
        "string" => {
            let max_len = schema.max_length.unwrap_or(255) as usize;
            if input.len() > max_len {
                return Err(format!(
                    "String too long ({} > {} max)",
                    input.len(),
                    max_len
                ));
            }
            Ok(input.as_bytes().to_vec())
        }
        "command" => match input.to_lowercase().as_str() {
            "start" => Ok(vec![CMD_STATUS_START]),
            "confirm" | "yes" | "y" => Ok(vec![CMD_STATUS_CONFIRM]),
            "cancel" | "no" | "n" => Ok(vec![CMD_STATUS_CANCEL]),
            "poll" => Ok(vec![CMD_STATUS_POLL]),
            other => Err(format!(
                "Invalid command action: '{}'. Use start, confirm, cancel, or poll",
                other
            )),
        },
        other => Err(format!("Parameter type '{}' is not writable", other)),
    }
}

/// Loads only the specific parameter IDs from the device, skipping full enumeration.
fn load_specific_params(
    app: &mut App,
    param_ids: &[u8],
    timeout: u64,
) -> io::Result<Box<dyn SerialPort>> {
    let mut port = open_port(app)?;
    if !discover_device(app, &mut port, timeout) {
        return Err(io::Error::other("Device discovery failed"));
    }

    let dev_addr = app.selected_device.unwrap();
    let mut remaining: std::collections::HashSet<u8> = param_ids.iter().copied().collect();
    let mut read_buf = [0u8; 512];
    let start = Instant::now();

    {
        let mut mgr = app.manager.lock().unwrap();
        for &pid in param_ids {
            if let Some(pkt) = mgr.request_parameter(dev_addr, pid, 0) {
                let _ = send_packet_to_serial(&mut port, &pkt);
            }
        }
    }

    eprint!("Loading {} parameter(s)", param_ids.len());

    while !remaining.is_empty() && start.elapsed() < Duration::from_secs(timeout) {
        let time_ms = start_time().elapsed().as_millis() as u32;
        {
            let mut mgr = app.manager.lock().unwrap();
            mgr.update_time(time_ms);
        }

        {
            let mut mgr = app.manager.lock().unwrap();
            let outgoing = mgr.drain_all();
            drop(mgr);
            for pkt in outgoing {
                let _ = send_packet_to_serial(&mut port, &pkt);
            }
        }

        let bytes_read = read_from_serial(&mut port, &mut read_buf)?;
        if bytes_read > 0 {
            let mut parser = app.parser.lock().unwrap();
            let packets: Vec<_> = parser.iter_packets(&read_buf[..bytes_read]).collect();
            drop(parser);

            for packet_result in packets {
                if let Ok(ref packet) = packet_result {
                    let loaded_id = match packet {
                        Packet::ParameterSettingsEntry(entry)
                            if entry.chunks_remaining == 0 =>
                        {
                            Some(entry.parameter_number)
                        }
                        Packet::ParameterChunk(chunk) if chunk.chunks_remaining == 0 => {
                            Some(chunk.param_number)
                        }
                        _ => None,
                    };

                    {
                        let mut mgr = app.manager.lock().unwrap();
                        mgr.handle_packet(packet);

                        if let Some(pid) = loaded_id {
                            if remaining.contains(&pid) {
                                if mgr
                                    .get_device(dev_addr)
                                    .and_then(|d| d.get_parameter(pid))
                                    .is_some()
                                {
                                    remaining.remove(&pid);
                                }
                            }
                        }
                    }

                    if let Some(pid) = loaded_id {
                        if !remaining.contains(&pid) {
                            eprint!(".");
                        }
                    }
                }
            }
        }

        std::thread::sleep(Duration::from_millis(50));
    }

    eprintln!();

    if !remaining.is_empty() {
        eprintln!(
            "Warning: {} parameter(s) could not be loaded: {:?}",
            remaining.len(),
            remaining
        );
    }

    Ok(port)
}

/// Export using schema: only query the specific param(s) by ID instead of
/// enumerating all parameters.
fn run_export_with_schema(
    app: &mut App,
    format: &ExportFormat,
    output: Option<&str>,
    get: Option<&str>,
    schema_path: &str,
    timeout: u64,
) -> io::Result<()> {
    let schemas = parse_schema_file(schema_path)?;
    eprintln!("Loaded schema with {} parameter(s)", schemas.len());

    let Some(ident) = get else {
        eprintln!("--from-schema requires --get to specify which parameter(s) to query");
        std::process::exit(1);
    };

    let param_ids = {
        let mut ids = Vec::new();
        for part in ident.split(',') {
            let trimmed = part.trim();
            if let Some(id) = resolve_param_id_from_schema(&schemas, trimmed) {
                if !ids.contains(&id) {
                    ids.push(id);
                }
            } else {
                eprintln!("No parameter matching '{}' in schema", trimmed);
                std::process::exit(1);
            }
        }
        ids
    };

    eprintln!("Querying {} parameter(s) by ID...", param_ids.len());
    let _port = load_specific_params(app, &param_ids, timeout)?;

    let mgr = app.manager.lock().unwrap();
    let dev_addr = app.selected_device.unwrap();
    let device = mgr.get_device(dev_addr).unwrap();

    if param_ids.len() == 1 {
        let pid = param_ids[0];
        let Some(param) = device.get_parameter(pid) else {
            drop(mgr);
            eprintln!("Parameter {} not loaded", pid);
            std::process::exit(1);
        };
        let node = param_to_export_node(param, device, format);
        drop(mgr);

        let json = serde_json::to_string_pretty(&node)
            .map_err(|e| io::Error::other(format!("JSON serialization failed: {}", e)))?;

        match output {
            Some(path) => {
                std::fs::write(path, &json)?;
                eprintln!("Exported to {}", path);
            }
            None => println!("{}", json),
        }
    } else {
        let nodes: Vec<ExportNode> = param_ids
            .iter()
            .filter_map(|&pid| {
                device.get_parameter(pid).map(|p| param_to_export_node(p, device, format))
            })
            .collect();
        drop(mgr);

        let json = serde_json::to_string_pretty(&nodes)
            .map_err(|e| io::Error::other(format!("JSON serialization failed: {}", e)))?;

        match output {
            Some(path) => {
                std::fs::write(path, &json)?;
                eprintln!("Exported to {}", path);
            }
            None => println!("{}", json),
        }
    }

    Ok(())
}

/// Set using schema: write parameters directly by ID without full enumeration.
fn run_set_with_schema(
    app: &mut App,
    assignments: &[String],
    from_json: Option<&str>,
    schema_path: &str,
    timeout: u64,
    check: bool,
    confirm: bool,
) -> io::Result<()> {
    let schemas = parse_schema_file(schema_path)?;
    eprintln!("Loaded schema with {} parameter(s)", schemas.len());

    let mut port = open_port(app)?;
    if !discover_device(app, &mut port, timeout) {
        return Err(io::Error::other("Device discovery failed"));
    }

    let dev_addr = app.selected_device.unwrap();
    let mut writes: Vec<(u8, Vec<u8>)> = Vec::new();

    if let Some(json_path) = from_json {
        let json_data = std::fs::read_to_string(json_path)?;
        let root: serde_json::Value = serde_json::from_str(&json_data)
            .map_err(|e| io::Error::other(format!("Invalid JSON: {}", e)))?;
        if let Some(params) = root.get("parameters").and_then(|p| p.as_array()) {
            let flat = flatten_export_nodes(params);
            for (id, _name) in flat {
                if let Some(val_str) = find_value_in_tree(params, id) {
                    if let Some(schema) = schemas.iter().find(|s| s.id == id) {
                        match resolve_write_data_from_schema(schema, &val_str) {
                            Ok(data) => writes.push((id, data)),
                            Err(e) => eprintln!("Skipping param {}: {}", id, e),
                        }
                    } else {
                        eprintln!("Skipping param {}: not found in schema", id);
                    }
                }
            }
        }
    }

    for assignment in assignments {
        let Some((ident, value)) = parse_assignment(assignment) else {
            continue;
        };
        let Some(id) = resolve_param_id_from_schema(&schemas, &ident) else {
            eprintln!("Unknown parameter: '{}' (not in schema)", ident);
            continue;
        };
        let Some(schema) = schemas.iter().find(|s| s.id == id) else {
            eprintln!("No schema for parameter {}", id);
            continue;
        };
        if let Some(ref val) = value {
            match resolve_write_data_from_schema(schema, val) {
                Ok(data) => writes.push((id, data)),
                Err(e) => eprintln!("Invalid value for param {}: {}", id, e),
            }
        } else {
            if schema.param_type == "command" {
                writes.push((id, vec![CMD_STATUS_START]));
            } else {
                eprintln!(
                    "Parameter '{}' requires a value. Use {}=value",
                    ident, ident
                );
            }
        }
    }

    if writes.is_empty() {
        eprintln!("No valid writes to perform");
        return Ok(());
    }

    for (pid, data) in &writes {
        info!(
            "Writing param {} ({} bytes) to device 0x{:02X}",
            pid,
            data.len(),
            dev_addr as u8
        );
        if let Some(pkt) = build_param_write_packet(dev_addr, *pid, data) {
            send_packet_to_serial(&mut port, &pkt)?;
            eprintln!("Sent write for param {}", pid);
        }
    }

    if check {
        app.ensure_param_entries(
            schemas.iter().map(|s| s.id).max().map(|m| m + 1).unwrap_or(0),
        );
        verify_writes(app, &mut port, &writes, dev_addr, confirm)?;
    }

    Ok(())
}

fn main() {
    let args = Args::parse();

    init_logging(&args.log_file);

    let mut app = App::new(args.port.clone(), args.baud);

    info!(
        "Starting CRSF Parameter tool on {} @ {} baud (timeout {}s)",
        args.port, args.baud, args.discovery_timeout
    );

    let result = match &args.command {
        Some(CliCommand::Export {
            format,
            output,
            get,
            from_schema,
        }) => {
            eprintln!("uf-crsf Parameter Export");
            run_export(
                &mut app,
                format,
                output.as_deref(),
                get.as_deref(),
                from_schema.as_deref(),
                args.discovery_timeout,
            )
        }
        Some(CliCommand::Set {
            assignments,
            from_json,
            from_schema,
            check,
            confirm,
        }) => {
            eprintln!("uf-crsf Parameter Set");
            run_set(
                &mut app,
                assignments,
                from_json.as_deref(),
                from_schema.as_deref(),
                args.discovery_timeout,
                *check,
                *confirm,
            )
        }
        None => {
            eprintln!("uf-crsf Parameter TUI");
            eprintln!("Logging to {}", args.log_file);
            eprintln!();
            run(&mut app, args.discovery_timeout)
        }
    };

    if let Err(e) = result {
        error!("Fatal error: {}", e);
        eprintln!("\nFatal error: {}", e);
        std::process::exit(1);
    }
}
