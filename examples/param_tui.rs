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

use log::{trace, debug, error, info, warn, LevelFilter, Log, Metadata, Record};

use clap::Parser;
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

use serialport::SerialPort;
use uf_crsf::device::{DeviceManager, DeviceManagerConfig, Parameter};
use uf_crsf::packets::{write_packet_to_buffer, Packet, PacketAddress, ParameterData};
use uf_crsf::parser::CrsfParser;

#[derive(Parser)]
#[command(name = "uf-crsf-param-tui", about = "CRSF/ELRS parameter browser TUI")]
struct Args {
    #[arg(default_value = "/dev/ttyACM0", help = "Serial port path")]
    port: String,
    #[arg(long, default_value = "400000", help = "Serial baud rate")]
    baud: u32,
    #[arg(
        long = "poll-ms",
        default_value = "200",
        help = "Parameter poll interval (ms)"
    )]
    poll_interval_ms: u32,
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
    status_message: String,
    connected: bool,
    port_path: String,
    baud_rate: u32,
    poll_interval_ms: u32,
    params_loaded: bool,
    last_poll: Instant,
    param_request_pending: bool,
    next_param_id: u8,
}

impl App {
    fn new(port_path: String, baud_rate: u32, poll_interval_ms: u32) -> Self {
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
            status_message: "Discovering devices...".to_string(),
            connected: false,
            port_path,
            baud_rate,
            poll_interval_ms,
            params_loaded: false,
            last_poll: Instant::now(),
            param_request_pending: false,
            next_param_id: 0,
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
                    params.push((0, root.clone()));
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
            Some(ParameterData::Command { status, .. }) => match status {
                0 => "Idle".to_string(),
                1 => "Running".to_string(),
                2 => "Executing".to_string(),
                _ => format!("Status {}", status),
            },
            Some(ParameterData::Folder { children }) => format!("[{} items]", children.len()),
            Some(ParameterData::Vtx { data }) => format!("{:02X?}", data.as_slice()),
            None => "?".to_string(),
        }
    }

    fn format_param_detail(param: &Parameter) -> Vec<Line<'static>> {
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
                    status,
                    *timeout as u32 * 100
                )));
                lines.push(Line::from(""));
                lines.push(Line::from(Span::styled(
                    "Press Enter to execute",
                    Style::default().fg(Color::Cyan),
                )));
            }
            Some(ParameterData::Folder { children }) => {
                lines.push(Line::from("Type: Folder"));
                lines.push(Line::from(format!("Children: {:?}", children.as_slice())));
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

fn build_param_read_packet(device_addr: PacketAddress, param_id: u8, chunk: u8) -> Option<Vec<u8>> {
    use uf_crsf::packets::ParameterRead;
    let read = ParameterRead::new(
        device_addr as u8,
        PacketAddress::Handset as u8,
        param_id,
        chunk,
    )
    .ok()?;
    let mut buffer = [0u8; 64];
    let len = write_packet_to_buffer(&mut buffer, device_addr, &read).ok()?;
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
            Ok(bytes_read) if bytes_read > 0 => {
                let mut parser = app.parser.lock().unwrap();
                let mut mgr = app.manager.lock().unwrap();
                for packet in parser.iter_packets(&read_buf[..bytes_read]).flatten() {
                    if let Packet::DeviceInformation(info) = &packet {
                        let addr =
                            try_packet_addr(info.src_addr).unwrap_or(PacketAddress::Transmitter);
                        app.selected_device = Some(addr);
                        info!(
                            "Discovered: {} (0x{:02X}), {} params",
                            info.device_name(),
                            info.src_addr,
                            info.parameters_total
                        );
                        mgr.handle_packet(&packet);
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

    app.last_poll = Instant::now();
    app.status_message = format!("Connected to {} @ {} baud", app.port_path, app.baud_rate);

    let mut read_buf = [0u8; 512];
    let tick_rate = Duration::from_millis(50);

    loop {
        let now = Instant::now();
        let time_ms = now.elapsed().as_millis() as u32;

        {
            let mut mgr = app.manager.lock().unwrap();
            mgr.update_time(time_ms);
        }

        match read_from_serial(port, &mut read_buf) {
            Ok(bytes_read) if bytes_read > 0 => {
                let mut parser = app.parser.lock().unwrap();
                let mut mgr = app.manager.lock().unwrap();
                for packet_result in parser.iter_packets(&read_buf[..bytes_read]) {
                    match packet_result {
                        Ok(ref packet) => {
                            debug!("Parsed packet: {:?}", packet);
                            match packet {
                                Packet::ParameterSettingsEntry(entry)
                                    if entry.chunks_remaining == 0 =>
                                {
                                    info!(
                                        "Param {} loaded: {}",
                                        entry.parameter_number, entry.name
                                    );
                                    app.param_request_pending = false;
                                }
                                Packet::DeviceInformation(_) => {
                                    // Already discovered in Phase 2
                                }
                                _ => {}
                            }
                            mgr.handle_packet(packet);
                        }
                        Err(e) => warn!("Packet parse error: {:?}", e),
                    }
                }
            }
            _ => {}
        }

        {
            let mut mgr = app.manager.lock().unwrap();
            let retry_packets = mgr.process_timeouts();
            drop(mgr);
            for retry in retry_packets {
                let _ = send_packet_to_serial(port, &retry);
            }
        }

        // Parameter enumeration
        if app.selected_device.is_some()
            && !app.params_loaded
            && !app.param_request_pending
            && now.duration_since(app.last_poll)
                >= Duration::from_millis(app.poll_interval_ms as u64)
        {
            app.last_poll = now;
            let mgr = app.manager.lock().unwrap();
            if let Some(dev_addr) = app.selected_device {
                if let Some(device) = mgr.get_device(dev_addr) {
                    if device.parameters_loaded {
                        app.params_loaded = true;
                        info!("All {} params loaded", device.parameters.len());
                        app.status_message =
                            format!("All {} parameters loaded", device.parameters.len());
                    } else if device.parameters.is_empty() && device.parameters_total > 0 {
                        drop(mgr);
                        if let Some(pkt) = build_param_read_packet(dev_addr, 0, 0) {
                            let _ = send_packet_to_serial(port, &pkt);
                            app.param_request_pending = true;
                            app.next_param_id = 1;
                            app.status_message = "Requesting parameters...".to_string();
                        }
                    } else {
                        let next_id = device.parameters.len() as u8;
                        if next_id < device.parameters_total {
                            drop(mgr);
                            if let Some(pkt) = build_param_read_packet(dev_addr, next_id, 0) {
                                let _ = send_packet_to_serial(port, &pkt);
                                app.param_request_pending = true;
                                app.next_param_id = next_id + 1;
                                app.status_message = format!("Requesting parameter {}...", next_id);
                            }
                        }
                        continue;
                    }
                }
            }
        }

        // Keyboard
        while event::poll(Duration::from_millis(0))? {
            if let Event::Key(key) = event::read()? {
                if key.kind != KeyEventKind::Press {
                    continue;
                }
                if app.editing {
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
                        KeyCode::Enter | KeyCode::Char(' ') => handle_select(app),
                        KeyCode::Backspace => app.go_back(),
                        KeyCode::Char('r') => {
                            if let Some(dev_addr) = app.selected_device {
                                app.params_loaded = false;
                                app.param_request_pending = false;
                                app.next_param_id = 0;
                                app.last_poll = Instant::now() - Duration::from_secs(1);
                                app.status_message = "Refreshing parameters...".to_string();
                                let mut mgr = app.manager.lock().unwrap();
                                if let Some(device) = mgr.get_device_mut(dev_addr) {
                                    device.parameters.clear();
                                    device.parameters_loaded = false;
                                }
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
        Some(ParameterData::Float { .. })
        | Some(ParameterData::TextSelection { .. })
        | Some(ParameterData::String { .. })
        | Some(ParameterData::Command { .. }) => {
            app.editing = true;
            app.edit_buffer.clear();
        }
        _ => {}
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
    let (param_id, param) = &params[selected];
    let Some(dev_addr) = app.selected_device else {
        return;
    };

    let write_data: Option<Vec<u8>> = match &param.data {
        Some(ParameterData::Float {
            min,
            max,
            decimal_point,
            ..
        }) => {
            let min = *min;
            let max = *max;
            let decimal_point = *decimal_point;
            let parsed: Result<f64, _> = input.parse();
            match parsed {
                Ok(val) => {
                    let divisor = 10_i32.pow(decimal_point as u32) as f64;
                    let int_val = (val * divisor) as i32;
                    if int_val < min || int_val > max {
                        app.status_message = format!(
                            "Value {} out of range [{}, {}]",
                            val,
                            min as f64 / divisor,
                            max as f64 / divisor
                        );
                        return;
                    }
                    Some(int_val.to_le_bytes().to_vec())
                }
                Err(_) => {
                    app.status_message = format!("Invalid number: {}", input);
                    return;
                }
            }
        }
        Some(ParameterData::TextSelection {
            options, min, max, ..
        }) => {
            let min = *min;
            let max = *max;
            match input.parse::<u8>() {
                Ok(idx) if idx >= min && idx <= max => Some(vec![idx]),
                Ok(idx) => {
                    app.status_message = format!("Index {} out of range [{}, {}]", idx, min, max);
                    return;
                }
                Err(_) => {
                    let opts: Vec<&str> = options.split(';').collect();
                    if let Some(pos) = opts.iter().position(|o| o.eq_ignore_ascii_case(&input)) {
                        let pos_u8 = pos as u8;
                        if pos_u8 >= min && pos_u8 <= max {
                            Some(vec![pos_u8])
                        } else {
                            app.status_message = format!("Option '{}' out of range", input);
                            return;
                        }
                    } else {
                        app.status_message = format!(
                            "Invalid option: '{}'. Use index 0-{} or option name",
                            input, max
                        );
                        return;
                    }
                }
            }
        }
        Some(ParameterData::String { .. }) => Some(input.as_bytes().to_vec()),
        Some(ParameterData::Command { .. }) => Some(vec![0]),
        _ => None,
    };

    if let Some(data) = write_data {
        let pid = *param_id;
        info!(
            "Writing parameter {} ({} bytes) to device 0x{:02X}",
            pid,
            data.len(),
            dev_addr as u8
        );
        if let Some(pkt) = build_param_write_packet(dev_addr, pid, &data) {
            match send_packet_to_serial(port, &pkt) {
                Ok(()) => {
                    app.status_message =
                        format!("Sent write for param {} ({} bytes)", pid, data.len());
                    app.params_loaded = false;
                    app.param_request_pending = false;
                    app.last_poll = Instant::now();
                    let mut mgr = app.manager.lock().unwrap();
                    if let Some(device) = mgr.get_device_mut(dev_addr) {
                        device.parameters.clear();
                        device.parameters_loaded = false;
                    }
                }
                Err(e) => {
                    app.status_message = format!("Write error: {}", e);
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
        App::format_param_detail(param)
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

    if app.editing {
        let input = Paragraph::new(format!("> {}", app.edit_buffer))
            .style(Style::default().fg(Color::Yellow))
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(" Enter value (Esc to cancel) "),
            );
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
            format!(
                "{} | 0x{:02X} | Params: {}/{}{}",
                device.name,
                dev_addr as u8,
                device.parameters.len(),
                device.parameters_total,
                if device.parameters_loaded {
                    " [LOADED]"
                } else {
                    ""
                }
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
            "q/Ctrl-C:Quit  \u{2191}\u{2193}:Nav  Enter:Edit  Backspace:Back  r:Refresh",
            Style::default().fg(Color::DarkGray),
        ),
    ]);

    let status = Paragraph::new(line).block(Block::default().borders(Borders::ALL));
    f.render_widget(status, area);
}

fn main() {
    let args = Args::parse();

    init_logging(&args.log_file);

    eprintln!("uf-crsf Parameter TUI");
    eprintln!("Logging to {}", args.log_file);
    eprintln!();

    info!(
        "Starting CRSF Parameter TUI on {} @ {} baud (poll {}ms, timeout {}s)",
        args.port, args.baud, args.poll_interval_ms, args.discovery_timeout
    );

    let mut app = App::new(args.port, args.baud, args.poll_interval_ms);

    if let Err(e) = run(&mut app, args.discovery_timeout) {
        error!("Fatal error: {}", e);
        eprintln!("\nFatal error: {}", e);
        std::process::exit(1);
    }
}
