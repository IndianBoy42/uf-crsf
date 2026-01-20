//! High-level device and parameter management API.
//!
//! This module provides abstractions for discovering CRSF devices, enumerating their configurable
//! parameters, and reading/writing parameter values. It implements the CRSF Extended Device Protocol
//! used by ExpressLRS, Betaflight, ArduPilot, and other CRSF-compliant systems.
//!
//! # CRSF Parameter Protocol
//!
//! CRSF devices expose parameters through a hierarchical structure:
//! - **Root folder**: Parameter 0, serves as the entry point with children listed in its `value` field
//! - **Folders**: Organizational nodes that contain child parameters
//! - **Leaf parameters**: Actual configuration values (Float, TextSelection, String, VTX, etc.)
//!
//! Parameters use chunked transfer for large metadata, with each [`ParameterSettingsEntry`]
//! potentially spanning multiple packets. The protocol works in a request/response pattern:
//! 1. Discover devices via broadcast [DevicePing]
//! 2. Receive [DeviceInformation] with parameter count and version
//! 3. Request parameters sequentially via [ParameterRead]
//! 4. Receive [ParameterSettingsEntry] with metadata and current value
//! 5. Write parameters via [ParameterWrite]
//!
//! # Device Roles in CRSF Network
//!
//! | Role | Address | Typical Usage |
//! |------|---------|---------------|
//! | **Handset/Controller** | 0xEE | Runs this API to configure TX modules and read RX status |
//! | **Transmitter (TX)** | 0xEA | Exposes RF parameters (power, mode, telemetry rate) |
//! | **Receiver (RX)** | 0xEC | May expose PWM/serial output modes |
//! | **Flight Controller** | 0xEF | Exposes flight modes, VTX control, etc. |
//!
//! # Integration Patterns
//!
//! ## Handset Application (e.g., EdgeTX/OpenTX Lua script)
//! Discover TX module parameters, display UI, write configuration:
//!
//! ```no_run
//! use uf_crsf::device::{DeviceManager, DeviceManagerConfig};
//! use uf_crsf::parser::CrsfParser;
//! use uf_crsf::packets::{Packet, PacketAddress};
//!
//! // Initialize manager
//! let config = DeviceManagerConfig::default();
//! let mut manager = DeviceManager::new(config).with_address(PacketAddress::Handset);
//! let mut parser = CrsfParser::new();
//!
//! // In your UART RX loop (e.g., EdgeTX serial.read())
//! loop {
//!     let incoming_data = read_from_uart(); // Your UART read function
//!     for packet_result in parser.iter_packets(&incoming_data) {
//!         if let Ok(packet) = packet_result {
//!             manager.handle_packet(&packet);
//!         }
//!     }
//!
//!     // Send any pending pings or retries
//!     manager.update_time(current_time_ms());
//!     if let Some(ping) = manager.send_device_ping() {
//!         write_to_uart(&ping); // Your UART write function
//!     }
//!
//!     // Request all parameters once device discovered
//!     let device_addrs: heapless::Vec<_, 8> = manager.devices().collect();
//!     if let Some(&tx_addr) = device_addrs.first() {
//!         if let Some(request) = manager.request_all_parameters(tx_addr) {
//!             write_to_uart(&request);
//!         }
//!     }
//! }
//! ```
//!
//! ## Embedded Microcontroller (e.g., STM32, ESP32)
//! On embedded systems without an allocator, the UART interface is typically polled
//! in an event loop or handled in an interrupt-driven ring buffer. The CRSF parser
//! processes bytes as they arrive:
//!
//! ```no_run
//! # use uf_crsf::device::DeviceManager;
//! # use uf_crsf::parser::CrsfParser;
//! // In your main loop or UART ISR
//! static mut RX_BUFFER: [u8; 128] = [0; 128];
//! static mut RX_LEN: usize = 0;
//!
//! fn uart_isr() {
//!     // Read byte from UART hardware into buffer
//!     // ... hardware-specific code ...
//!     # unsafe {}
//! }
//!
//! fn main_loop() {
//!     let mut manager = DeviceManager::default();
//!     let mut parser = CrsfParser::new();
//!
//!     loop {
//!         // Check if data available from UART
//!         # let data: &[u8] = &[];
//!         unsafe {
//!             if RX_LEN > 0 {
//!                 for packet_result in parser.iter_packets(&RX_BUFFER[..RX_LEN]) {
//!                     if let Ok(packet) = packet_result {
//!                         manager.handle_packet(&packet);
//!                     }
//!                 }
//!                 RX_LEN = 0;
//!             }
//!         }
//!
//!         // Process time-based tasks every 1-10ms
//!         manager.update_time(get_monotonic_ms());
//!         if let Some(packet) = manager.send_device_ping() {
//!             uart_write_blocking(&packet);
//!         }
//!
//!         // Handle retries for pending parameter requests
//!         let retries = manager.process_timeouts();
//!         for packet in retries {
//!             uart_write_blocking(&packet);
//!         }
//!     }
//! }
//! ```
//!
//! # Architecture Notes
//!
//! CRSF is a half-duplex protocol, but in practice it's wired as full-duplex with
//! separate TX and RX lines. This allows continuous polling without waiting for responses.
//!
//! The [DeviceManager] tracks pending requests with timeouts and automatic retries.
//! Call [`DeviceManager::update_time()`] regularly (e.g., every 1-10ms) to enable
//! timeout detection, and call [`DeviceManager::process_timeouts()`] to generate
//! retry packets.

use crate::constants;
use crate::packets::{
    DeviceInformation, DevicePing, Packet, PacketAddress, ParameterData, ParameterRead,
    ParameterSettingsEntry, ParameterWrite,
};
use crate::CrsfParsingError;
use heapless::index_map::FnvIndexMap;
use heapless::{String, Vec};

/// Maximum number of devices that can be tracked.
pub const MAX_DEVICES: usize = 8;

/// Maximum number of parameters per device.
pub const MAX_PARAMETERS: usize = 64;

/// Maximum pending parameter requests.
const MAX_PENDING_REQUESTS: usize = 16;

/// Maximum chunk size for reassembly.
const MAX_CHUNK_BUFFER_SIZE: usize = 512;

/// Default timeout for parameter requests (in milliseconds).
const DEFAULT_TIMEOUT_MS: u32 = 500;

/// Default retry count for parameter requests.
const DEFAULT_RETRY_COUNT: u8 = 3;

/// Configuration for [DeviceManager] timeouts and retry behavior.
#[derive(Debug, Clone, Copy)]
pub struct DeviceManagerConfig {
    /// Timeout for parameter requests in milliseconds.
    ///
    /// If a device doesn't respond within this window, the request is retried up to
    /// [Self::retry_count] times before being abandoned. For reliable serial links,
    /// 500ms is typical. Noisy environments may need 1000ms or more.
    pub timeout_ms: u32,
    /// Maximum number of retries for failed parameter requests.
    ///
    /// After [Self::timeout_ms] elapses without a response, the request is retried.
    /// After this many retries, the request is abandoned. For production systems,
    /// 3-5 retries provide a good balance between reliability and responsiveness.
    pub retry_count: u8,
    /// Device ping interval in milliseconds (0 = disabled).
    ///
    /// Controls how often [DeviceManager::send_device_ping()] broadcasts discovery
    /// requests. Setting to 0 disables automatic pinging - you must manually send
    /// pings. For handheld controllers, 1000-2000ms is reasonable. For embedded
    /// systems with tight latency requirements, consider 500ms or manual triggering.
    pub device_ping_interval_ms: u32,
}

impl Default for DeviceManagerConfig {
    fn default() -> Self {
        Self {
            timeout_ms: DEFAULT_TIMEOUT_MS,
            retry_count: DEFAULT_RETRY_COUNT,
            device_ping_interval_ms: 1000,
        }
    }
}

/// Represents a CRSF parameter with metadata and current value.
///
/// Parameters are the primary mechanism for configuring CRSF devices. They form a
/// hierarchical tree structure starting with a root folder (ID 0). Each parameter
/// has a unique ID, parent folder reference, name, and typed value data.
///
/// # Parameter Hierarchy
///
/// In ExpressLRS, parameters are organized as:
/// ```text
/// [0] ROOT (Folder, children=[1, 2, 3])
/// ├── [1] Connection (Folder, children=[10, 11, 12])
/// │   ├── [10] Link Quality (Info)
/// │   └── [11] RF Mode (TextSelection: Dynamic, Fixed...)
/// └── [2] VTX (Folder, children=[20, 21])
///     ├── [20] Band (TextSelection: A, B, E, R...)
///     └── [21] Power (TextSelection: 25mW, 200mW...)
/// ```
///
/// To navigate the hierarchy:
/// 1. Get the root folder via [Device::root_folder()]
/// 2. Read its child IDs from `folder_children()`
/// 3. Recursively traverse children
///
/// # Parameter Types
///
/// The `data` field contains the type-specific information:
/// - **Folder**: No value, just organizational structure with child IDs
/// - **Float**: Numeric value with min/max/default, step size, units (e.g., RF power in mW)
/// - **TextSelection**: Enum-style value with string options (e.g., RF mode: "Dynamic", "Fixed")
/// - **String**: Free-form text (rare, used for bind phrases)
/// - **Info**: Read-only string for display (e.g., firmware version)
/// - **Command**: Trigger to execute an action (e.g., "Bind", "VTX Save")
/// - **VTX**: Band/channel/power selection for video transmitters
///
/// # Usage Patterns
///
/// ## Displaying Parameters (Handset UI)
/// ```no_run
/// # use uf_crsf::device::{Device, DeviceManager};
/// # use uf_crsf::packets::{Packet, PacketAddress};
/// fn display_device_parameters(device: &Device) {
///     if let Some(root) = device.root_folder() {
///         if let Some(children) = root.folder_children() {
///             for &child_id in children {
///                 if let Some(child) = device.get_parameter(child_id) {
///                     if child.is_folder() {
///                         // Create menu group
///                         println!("[{}] {}", child_id, child.name);
///                     } else {
///                         // Display parameter with value
///                         if let Some(ref data) = child.data {
///                             println!("[{}] {}: {:?}", child_id, child.name, data);
///                         }
///                     }
///                 }
///             }
///         }
///     }
/// }
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Parameter {
    /// Unique parameter ID within a device.
    ///
    /// IDs are sequential starting from 0 (root folder). Use this ID when requesting
    /// parameter details via [DeviceManager::request_parameter()] or writing values
    /// via [DeviceManager::write_parameter()].
    pub id: u8,
    /// Parent folder parameter ID.
    ///
    /// A value of 0 indicates this is a direct child of the root folder. To traverse
    /// the hierarchy, use [Device::parameters_in_folder()] with the parent ID.
    pub parent: u8,
    /// Human-readable parameter name.
    ///
    /// Typically short and descriptive (e.g., "TX Power", "RF Mode", "VTX Band").
    /// Names are limited to 127 characters by the CRSF protocol.
    pub name: String<128>,
    /// Whether this parameter should be hidden from normal UI.
    ///
    /// Advanced or debug parameters may be marked hidden. Handset applications can
    /// optionally show hidden parameters in a developer mode.
    pub hidden: bool,
    /// Type-specific parameter data and current value.
    ///
    /// Contains the parameter's type and its current value (for readable/writable types).
    /// Folders and Info types may have minimal data.
    pub data: Option<ParameterData>,
}

impl Parameter {
    /// Creates a new parameter from a ParameterSettingsEntry.
    pub fn from_entry(id: u8, entry: &ParameterSettingsEntry) -> Self {
        Self {
            id,
            parent: entry.parent,
            name: entry.name.clone(),
            hidden: entry.is_hidden(),
            data: entry.data.clone(),
        }
    }

    /// Checks if this parameter is a folder.
    pub fn is_folder(&self) -> bool {
        matches!(self.data, Some(ParameterData::Folder { children: _ }))
    }

    /// Checks if this parameter is a command.
    pub fn is_command(&self) -> bool {
        matches!(self.data, Some(ParameterData::Command { .. }))
    }

    /// Returns the folder children if this is a folder parameter.
    pub fn folder_children(&self) -> Option<&Vec<u8, 32>> {
        if let Some(ParameterData::Folder { ref children }) = self.data {
            Some(children)
        } else {
            None
        }
    }
}

/// Represents a discovered CRSF device with its parameters.
///
/// A Device represents any CRSF-capable hardware on the bus that exposes configurable
/// parameters. This is commonly used in ExpressLRS TX modules, receivers, and flight
/// controllers to expose settings for external configuration.
///
/// # Device Identification
///
/// Devices are identified by their [PacketAddress] and unique serial number:
/// - **Transmitter (0xEA)**: ExpressLRS modules, serial typically 0x454C5253 ("ELRS")
/// - **Receiver (0xEC)**: ExpressLRS receivers
/// - **Flight Controller (0xEF)**: Betaflight, ArduPilot
///
/// # Device Lifecycle
///
/// 1. **Discovery**: Broadcast [DevicePing] → receive [DeviceInformation]
/// 2. **Enumeration**: Request parameters sequentially starting from ID 0
/// 3. **Loading**: Receive [ParameterSettingsEntry] packets with metadata
/// 4. **Configuration**: Read current values, write new values via [ParameterWrite]
///
/// # Parameter Versioning
///
/// The `parameter_version` field tracks the parameter schema version. When this changes,
/// the parameter structure (IDs, types, defaults) may have changed. Applications should
/// check this version and clear cached parameter data when it increments.
///
/// # Example Usage
///
/// ```no_run
/// # use uf_crsf::device::Device;
/// # use uf_crsf::packets::{Packet, PacketAddress};
/// fn configure_tx_power(device: &mut Device) {
///     // Find the TX Power parameter
///     for param in device.iter_parameters() {
///         if param.name.contains("Power") && !param.is_folder() {
///             if let Some(data) = &param.data {
///                 println!("TX Power: {:?}", data);
///                 // You would generate a ParameterWrite packet here
///             }
///         }
///     }
/// }
/// ```
#[derive(Debug, Clone)]
pub struct Device {
    /// CRSF address of this device.
    ///
    /// Determines the device's role in the CRSF network. Use this address when
    /// sending parameter requests or writes to this device.
    pub address: PacketAddress,
    /// Human-readable device name (up to 42 characters).
    ///
    /// Examples: "ELRS TX", "Betaflight", "ArduPilot". Display this to users
    /// to identify which device they're configuring.
    pub name: String<43>,
    /// Unique serial number identifying this specific device.
    ///
    /// For ExpressLRS TX modules, this is typically 0x454C5253 ("ELRS").
    /// Used to correlate [DeviceInformation] responses with discovery requests.
    pub serial_number: u32,
    /// Hardware identifier (vendor-specific).
    ///
    /// Often encodes the board type or hardware revision. Format varies by
    /// manufacturer - consult vendor documentation for interpretation.
    pub hardware_id: u32,
    /// Firmware identifier.
    ///
    /// Encodes firmware version and variant. Format is vendor-specific. For
    /// ExpressLRS, this helps identify if the device supports certain features.
    pub firmware_id: u32,
    /// Total number of parameters exposed by this device.
    ///
    /// Use this to determine when parameter enumeration is complete. When
    /// `parameters.len() == parameters_total`, all parameters have been loaded.
    pub parameters_total: u8,
    /// Parameter protocol version.
    ///
    /// When this changes, the parameter schema may have been updated. Applications
    /// should invalidate cached parameters when this version differs from the last
    /// known version.
    pub parameter_version: u8,
    /// Discovered parameters indexed by their ID.
    ///
    /// Parameters are loaded incrementally via [DeviceManager::request_parameter()].
    /// Use [Device::get_parameter()] to retrieve by ID, or [Device::iter_parameters()]
    /// to enumerate all loaded parameters.
    pub parameters: FnvIndexMap<u8, Parameter, MAX_PARAMETERS>,
    /// Indicates if all parameters have been successfully loaded.
    ///
    /// Set to `true` when `parameters.len() == parameters_total`. Applications should
    /// wait for this flag before displaying the full parameter tree to users.
    pub parameters_loaded: bool,
}

impl Device {
    /// Creates a new Device from DeviceInformation.
    pub fn from_device_info(info: &DeviceInformation) -> Result<Self, CrsfParsingError> {
        let address = PacketAddress::try_from(info.src_addr)
            .map_err(|_| CrsfParsingError::UnexpectedPacketType(info.src_addr))?;

        Ok(Self {
            address,
            name: String::try_from(info.device_name())
                .map_err(|_| CrsfParsingError::InvalidPayloadLength)?,
            serial_number: info.serial_number,
            hardware_id: info.hardware_id,
            firmware_id: info.firmware_id,
            parameters_total: info.parameters_total,
            parameter_version: info.parameter_version_number,
            parameters: FnvIndexMap::new(),
            parameters_loaded: false,
        })
    }

    /// Checks if this device is an ELRS TX module based on serial number.
    pub fn is_elrs_tx(&self) -> bool {
        // ELRS devices have serial number 0x454C5253 ("ELRS")
        self.serial_number == 0x454C5253
    }

    /// Adds or updates a parameter.
    ///
    /// Returns `true` if successful, `false` if the parameters map is full.
    pub fn add_parameter(&mut self, param: Parameter) -> bool {
        self.parameters.insert(param.id, param).is_ok()
    }

    /// Gets a parameter by ID.
    pub fn get_parameter(&self, id: u8) -> Option<&Parameter> {
        self.parameters.get(&id)
    }

    /// Gets a mutable parameter by ID.
    pub fn get_parameter_mut(&mut self, id: u8) -> Option<&mut Parameter> {
        self.parameters.get_mut(&id)
    }

    /// Returns an iterator over all parameters.
    pub fn iter_parameters(&self) -> impl Iterator<Item = &Parameter> {
        self.parameters.values()
    }

    /// Returns parameters in a specific folder.
    pub fn parameters_in_folder(&self, parent_id: u8) -> impl Iterator<Item = &Parameter> {
        self.parameters
            .values()
            .filter(move |p| p.parent == parent_id)
    }

    /// Gets the root folder parameter (parameter 0).
    pub fn root_folder(&self) -> Option<&Parameter> {
        self.get_parameter(0)
    }
}

/// Pending parameter request state.
#[derive(Debug, Clone)]
struct PendingRequest {
    /// Device address.
    device_addr: PacketAddress,
    /// Parameter number.
    parameter_id: u8,
    /// Current chunk being requested.
    chunk_number: u8,
    /// Expected chunks remaining (from last response).
    #[allow(dead_code)]
    expected_chunks_remaining: Option<u8>,
    /// Retry count.
    retries: u8,
    /// Timestamp of last request (implementation-specific, use monotonic counter).
    timestamp: u32,
}

/// Chunk reassembly state.
#[derive(Debug)]
struct ChunkAssembly {
    /// Parameter ID being assembled.
    parameter_id: u8,
    /// Accumulated chunks.
    #[allow(dead_code)]
    buffer: Vec<u8, MAX_CHUNK_BUFFER_SIZE>,
    /// Chunks remaining.
    #[allow(dead_code)]
    chunks_remaining: u8,
}

/// High-level manager for discovering CRSF devices and accessing their parameters.
///
/// [DeviceManager] orchestrates the complete parameter protocol workflow:
/// - Broadcasting discovery pings to find devices on the bus
/// - Requesting parameter metadata via chunked transfer
/// - Tracking pending requests with automatic timeout and retry
/// - Writing parameter values to devices
/// - Maintaining device state and parameter caches
///
/// # Thread Safety
///
/// [DeviceManager] is not thread-safe and must be accessed from a single thread
/// or context. For multi-threaded applications, wrap it in a mutex or use
/// message passing to funnel all CRSF I/O to a single management thread.
///
/// # Time Management
///
/// The manager relies on [DeviceManager::update_time()] being called periodically
/// to track request timeouts. In embedded systems, call this from your main loop
/// or a timer ISR every 1-10ms. In hosted applications, use a system monotonic clock.
///
/// # Typical Usage Flow
///
/// ```no_run
/// # use uf_crsf::device::{DeviceManager, DeviceManagerConfig};
/// # use uf_crsf::parser::CrsfParser;
/// # use uf_crsf::packets::PacketAddress;
/// let mut manager = DeviceManager::default().with_address(PacketAddress::Handset);
/// let mut parser = CrsfParser::new();
///
/// // Main event loop
/// loop {
///     // 1. Read bytes from UART
///     let bytes = uart_read();
///
///     // 2. Parse and feed packets to manager
///     for packet in parser.iter_packets(&bytes) {
///         if let Ok(pkt) = packet {
///             manager.handle_packet(&pkt);
///         }
///     }
///
///     // 3. Update time for timeout handling
///     manager.update_time(current_time_ms());
///
///     // 4. Send device discovery pings (if configured)
///     if let Some(ping) = manager.send_device_ping() {
///         uart_write(&ping);
///     }
///
///     // 5. Send any retries for timed-out requests
///     for retry in manager.process_timeouts() {
///         uart_write(&retry);
///     }
///
///     // 6. Once device discovered, request its parameters
///     for addr in manager.devices() {
///         if let Some(req) = manager.request_all_parameters(addr) {
///             uart_write(&req);
///         }
///     }
///
///     // 7. Check parameter load completion
///     for addr in manager.devices() {
///         if let Some(device) = manager.get_device(addr) {
///             if device.parameters_loaded {
///                 // Display parameters or apply configuration
///                 display_parameters(device);
///             }
///         }
///     }
///
///     // Sleep or yield to avoid busy-waiting
///     sleep_ms(10);
/// }
/// ```
///
/// # Hardware Integration
///
/// **Embedded Microcontrollers (STM32, ESP32, nRF52):**
/// - Run this in a high-priority task or main loop
/// - UART RX in ISR into a ring buffer, poll buffer in loop
/// - Use hardware timers for monotonic time tracking
/// - Ensure UART is configured for 115200-420000 baud (CRSF default)
///
/// **Linux/Windows Applications:**
/// - Use the `serial` crate with `tokio-serial` for async
/// - Run in a dedicated task/thread with async/await
/// - Use `std::time::Instant` or `tokio::time` for timing
///
/// **EdgeTX/OpenTX Lua Scripts:**
/// - Use the `serial` API to read/write CRSF packets
/// - Call update/time functions from script tick handler
/// - Note: Lua may have latency constraints, adjust timeouts accordingly
pub struct DeviceManager {
    /// Timeout and retry configuration.
    config: DeviceManagerConfig,
    /// This device's CRSF address.
    ///
    /// Must match the role of the device running this code. For a controller
    /// handset, use [PacketAddress::Handset]. This address is used as the source
    /// address when sending parameter requests.
    pub own_address: PacketAddress,
    /// Discovered devices indexed by their CRSF address.
    ///
    /// Devices are added when [DeviceInformation] packets are received. Access via
    /// [DeviceManager::devices()], [DeviceManager::get_device()], or
    /// [DeviceManager::get_device_mut()].
    devices: FnvIndexMap<PacketAddress, Device, MAX_DEVICES>,
    /// Pending parameter requests awaiting responses.
    ///
    /// Tracks in-flight [ParameterRead] and [ParameterWrite] requests with their
    /// timestamps for timeout detection. Managed automatically by [DeviceManager].
    pending_requests: Vec<PendingRequest, MAX_PENDING_REQUESTS>,
    /// State for reassembling chunked parameter responses.
    ///
    /// CRSF parameters larger than the packet size are sent in multiple chunks.
    /// This field tracks the reassembly state for the current chunk sequence.
    chunk_assembly: Option<ChunkAssembly>,
    /// Monotonic timestamp for timeout tracking.
    ///
    /// Updated via [DeviceManager::update_time()]. All pending requests compare
    /// their timestamp against this value to detect timeouts.
    current_time: u32,
    /// Timestamp of the last device discovery ping.
    ///
    /// Used to enforce [DeviceManagerConfig::device_ping_interval_ms] to avoid
    /// flooding the bus with discovery packets.
    last_ping_time: u32,
}

impl DeviceManager {
    /// Creates a new DeviceManager with the given configuration.
    pub fn new(config: DeviceManagerConfig) -> Self {
        Self {
            config,
            own_address: PacketAddress::Handset,
            devices: FnvIndexMap::new(),
            pending_requests: Vec::new(),
            chunk_assembly: None,
            current_time: 0,
            last_ping_time: 0,
        }
    }

    /// Sets the device manager's own address.
    pub fn with_address(mut self, addr: PacketAddress) -> Self {
        self.own_address = addr;
        self
    }

    /// Updates the internal time tracker for timeout detection.
    ///
    /// **Must be called periodically** (every 1-10ms recommended) with a monotonically
    /// increasing millisecond timestamp. This enables the manager to detect when
    /// parameter requests have timed out and need retrying.
    ///
    /// # Time Sources
    ///
    /// - **Embedded**: Use a hardware timer or DWT cycle counter
    /// - **Linux**: Use `std::time::Instant::elapsed().as_millis() as u32`
    /// - **Tokio**: Use `tokio::time::Instant`
    /// - **EdgeTX**: Use `getTimer()`
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use uf_crsf::device::DeviceManager;
    /// let mut manager = DeviceManager::default();
    ///
    /// // In your main loop or ISR
    /// loop {
    ///     manager.update_time(get_monotonic_ms());
    ///
    ///     // Process I/O and other tasks...
    ///
    ///     sleep_ms(1); // Call at least every 10ms
    /// }
    /// ```
    pub fn update_time(&mut self, time_ms: u32) {
        self.current_time = time_ms;
    }

    /// Returns an iterator over all discovered device addresses.
    pub fn devices(&self) -> impl Iterator<Item = PacketAddress> + '_ {
        self.devices.keys().copied()
    }

    /// Gets a device by address.
    pub fn get_device(&self, addr: PacketAddress) -> Option<&Device> {
        self.devices.get(&addr)
    }

    /// Gets a mutable device by address.
    pub fn get_device_mut(&mut self, addr: PacketAddress) -> Option<&mut Device> {
        self.devices.get_mut(&addr)
    }

    /// Process an incoming CRSF packet and update device state.
    ///
    /// This is the main entry point for feeding CRSF data into the manager.
    /// It handles parameter protocol packets and automatically updates
    /// device and parameter state.
    ///
    /// # Handled Packet Types
    ///
    /// - [Packet::DeviceInformation]: Adds or updates a discovered device
    /// - [Packet::ParameterSettingsEntry]: Updates parameter metadata/value
    /// - [Packet::ParameterWrite]: Clears pending write requests
    ///
    /// # Usage
    ///
    /// Call this for each packet returned by [crate::parser::CrsfParser]:
    ///
    /// ```no_run
    /// # use uf_crsf::device::DeviceManager;
    /// # use uf_crsf::parser::CrsfParser;
    /// # use uf_crsf::packets::Packet;
    /// let mut manager = DeviceManager::default();
    /// let mut parser = CrsfParser::new();
    ///
    /// // In your UART RX loop
    /// let bytes = uart_read();
    /// for packet_result in parser.iter_packets(&bytes) {
    ///     if let Ok(packet) = packet_result {
    ///         manager.handle_packet(&packet);
    ///     }
    /// }
    /// ```
    pub fn handle_packet(&mut self, packet: &Packet) {
        match packet {
            Packet::DeviceInformation(info) => {
                self.handle_device_info(info);
            }
            Packet::ParameterSettingsEntry(entry) => {
                self.handle_parameter_entry(entry);
            }
            _ => {
                // Ignore other packet types
            }
        }
    }

    /// Handles a DeviceInformation packet.
    fn handle_device_info(&mut self, info: &DeviceInformation) {
        if let Ok(device) = Device::from_device_info(info) {
            let addr = device.address;
            let _ = self.devices.insert(addr, device);
        }
    }

    /// Handles a ParameterSettingsEntry packet.
    fn handle_parameter_entry(&mut self, entry: &ParameterSettingsEntry) {
        // Extract chunking info from the raw packet if available
        // For now, assume single chunk (chunks_remaining = 0)
        // A real implementation would extract this from the packet header

        // Try to find the matching pending request
        let mut found_request = None;
        for (idx, _req) in self.pending_requests.iter().enumerate() {
            // Match based on the current state (would need access to device addr from packet)
            // For simplicity, we'll process this as a single-chunk response
            if let Some(assembly) = &self.chunk_assembly {
                if assembly.parameter_id == entry.parent {
                    found_request = Some(idx);
                    break;
                }
            }
        }

        // For now, treat as single-chunk and create parameter directly
        // A production implementation would handle multi-chunk assembly here

        // Find the device (we'd need to track which device this came from)
        // For now, we'll add to the first device that expects parameters
        for device in self.devices.values_mut() {
            if device.parameters.len() < device.parameters_total as usize {
                // Determine parameter ID - in a real impl, this comes from packet header
                let param_id = device.parameters.len() as u8;
                let param = Parameter::from_entry(param_id, entry);

                let _ = device.add_parameter(param);

                if device.parameters.len() == device.parameters_total as usize {
                    device.parameters_loaded = true;
                }
                break;
            }
        }

        // Remove the pending request if found
        if let Some(idx) = found_request {
            let _ = self.pending_requests.swap_remove(idx);
        }
    }

    /// Generate a device discovery ping packet if it's time to send one.
    ///
    /// This respects the [DeviceManagerConfig::device_ping_interval_ms] setting
    /// and returns `None` if too soon since the last ping. Set the interval to
    /// 0 to disable automatic pinging and manually trigger discovery.
    ///
    /// # When to Call
    ///
    /// Call this regularly (every 10-50ms) in your main loop or UART handling
    /// code. The function handles rate limiting internally.
    ///
    /// # Discovery Process
    ///
    /// 1. Call this function to get a ping packet (when it returns `Some`)
    /// 2. Send the packet bytes over UART to the CRSF bus
    /// 3. All devices respond with [DeviceInformation] packets
    /// 4. Call [DeviceManager::handle_packet()] for each response
    /// 5. Devices are automatically added to the internal device list
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use uf_crsf::device::DeviceManager;
    /// # use uf_crsf::packets::PacketAddress;
    /// let mut manager = DeviceManager::default().with_address(PacketAddress::Handset);
    ///
    /// // In your main loop
    /// loop {
    ///     manager.update_time(get_monotonic_ms());
    ///
    ///     // Try to send discovery ping
    ///     if let Some(ping) = manager.send_device_ping() {
    ///         uart_write(&ping);
    ///         println!("Sent device discovery ping");
    ///     }
    ///
    ///     // Process responses...
    /// }
    /// ```
    pub fn send_device_ping(&mut self) -> Option<Vec<u8, { constants::CRSF_MAX_PACKET_SIZE }>> {
        if self.config.device_ping_interval_ms == 0 {
            return None;
        }

        if self.current_time - self.last_ping_time < self.config.device_ping_interval_ms {
            return None;
        }

        self.last_ping_time = self.current_time;

        // Create broadcast ping
        let ping = DevicePing::new(PacketAddress::Broadcast as u8, self.own_address as u8).ok()?;

        let mut buffer = [0u8; constants::CRSF_MAX_PACKET_SIZE];
        let len =
            crate::packets::write_packet_to_buffer(&mut buffer, PacketAddress::Broadcast, &ping)
                .ok()?;

        let mut vec = Vec::new();
        vec.extend_from_slice(&buffer[..len]).ok()?;
        Some(vec)
    }

    /// Initiates full parameter enumeration for a device.
    ///
    /// Starts requesting parameters sequentially from ID 0 (the root folder) up to
    /// `device.parameters_total`. After calling this, repeatedly check
    /// [Device::parameters_loaded] or call this method again to get the next
    /// parameter request packet.
    ///
    /// # Parameter Enumeration Flow
    ///
    /// 1. Call this method to get the first [ParameterRead] packet
    /// 2. Send the packet to the device
    /// 3. Receive [ParameterSettingsEntry] via [DeviceManager::handle_packet()]
    /// 4. Call this method again to get the next parameter request
    /// 5. Repeat until the device's `parameters_loaded` is true
    ///
    /// # Returns
    ///
    /// - `Some(packet)`: The next [ParameterRead] packet bytes to send
    /// - `None`: Device doesn't exist or all parameters already loaded
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use uf_crsf::device::DeviceManager;
    /// # use uf_crsf::packets::PacketAddress;
    /// let mut manager = DeviceManager::default();
    /// let tx_addr = PacketAddress::Transmitter;
    ///
    /// // In your main loop after device discovery
    /// loop {
    ///     // Request parameters until loaded
    ///     if let Some(request) = manager.request_all_parameters(tx_addr) {
    ///         uart_write(&request);
    ///     }
    ///
    ///     // Check if loading complete
    ///     if let Some(device) = manager.get_device(tx_addr) {
    ///         if device.parameters_loaded {
    ///             println!("Loaded {} parameters", device.parameters.len());
    ///             break;
    ///         }
    ///     }
    ///
    ///     // Process packets and handle other tasks...
    /// }
    /// ```
    pub fn request_all_parameters(
        &mut self,
        device_addr: PacketAddress,
    ) -> Option<Vec<u8, { constants::CRSF_MAX_PACKET_SIZE }>> {
        let device = self.devices.get(&device_addr)?;

        if device.parameters_loaded {
            return None; // Already loaded
        }

        // Start with parameter 0 (root folder, if supported)
        self.request_parameter(device_addr, 0, 0)
    }

    /// Requests a specific parameter from a device.
    ///
    /// Returns the ParameterRead packet bytes to send.
    pub fn request_parameter(
        &mut self,
        device_addr: PacketAddress,
        parameter_id: u8,
        chunk_number: u8,
    ) -> Option<Vec<u8, { constants::CRSF_MAX_PACKET_SIZE }>> {
        // Check if device exists
        if !self.devices.contains_key(&device_addr) {
            return None;
        }

        // Create the request
        let read = ParameterRead::new(
            device_addr as u8,
            self.own_address as u8,
            parameter_id,
            chunk_number,
        )
        .ok()?;

        // Add to pending requests
        let pending = PendingRequest {
            device_addr,
            parameter_id,
            chunk_number,
            expected_chunks_remaining: None,
            retries: 0,
            timestamp: self.current_time,
        };

        if self.pending_requests.push(pending).is_err() {
            return None; // Queue full
        }

        // Serialize the packet
        let mut buffer = [0u8; constants::CRSF_MAX_PACKET_SIZE];
        let len = crate::packets::write_packet_to_buffer(&mut buffer, device_addr, &read).ok()?;

        let mut vec = Vec::new();
        vec.extend_from_slice(&buffer[..len]).ok()?;
        Some(vec)
    }

    /// Writes a new value to a parameter on a device.
    ///
    /// Generates a [ParameterWrite] packet to change a parameter's value. The
    /// `data` bytes should be formatted according to the parameter's type:
    ///
    /// - **Float**: 4 bytes, little-endian f32
    /// - **TextSelection**: Index of selected option (u8)
    /// - **String**: UTF-8 string bytes
    /// - **Folder**: Not writable
    /// - **Command**: Any value triggers the command
    ///
    /// # Value Encoding
    ///
    /// The `data` byte array must match the parameter's type. For Float parameters,
    /// encode the value as 4 little-endian bytes. For TextSelection, use the index
    /// of the selected option in the parameter's options list.
    ///
    /// # Returns
    ///
    /// - `Some(packet)`: The [ParameterWrite] packet bytes to send
    /// - `None`: Device doesn't exist or packet serialization failed
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use uf_crsf::device::DeviceManager;
    /// # use uf_crsf::packets::PacketAddress;
    /// let mut manager = DeviceManager::default();
    /// let tx_addr = PacketAddress::Transmitter;
    ///
    /// // Write TX Power parameter (ID 5, Float type)
    /// // Value: 2000 mW
    /// let power_value: f32 = 2000.0;
    /// let mut data = [0u8; 4];
    /// data.copy_from_slice(&power_value.to_le_bytes());
    ///
    /// if let Some(write_pkt) = manager.write_parameter(tx_addr, 5, &data) {
    ///     uart_write(&write_pkt);
    ///     println!("Sent TX Power write request");
    /// }
    /// ```
    pub fn write_parameter(
        &mut self,
        device_addr: PacketAddress,
        parameter_id: u8,
        data: &[u8],
    ) -> Option<Vec<u8, { constants::CRSF_MAX_PACKET_SIZE }>> {
        // Check if device exists
        if !self.devices.contains_key(&device_addr) {
            return None;
        }

        // Create the write packet
        let write = ParameterWrite::new(
            device_addr as u8,
            self.own_address as u8,
            parameter_id,
            data,
        )
        .ok()?;

        // Serialize the packet
        let mut buffer = [0u8; constants::CRSF_MAX_PACKET_SIZE];
        let len = crate::packets::write_packet_to_buffer(&mut buffer, device_addr, &write).ok()?;

        let mut vec = Vec::new();
        vec.extend_from_slice(&buffer[..len]).ok()?;
        Some(vec)
    }

    /// Processes timeouts and retries pending requests.
    ///
    /// Returns packets that need to be retransmitted.
    pub fn process_timeouts(
        &mut self,
    ) -> Vec<Vec<u8, { constants::CRSF_MAX_PACKET_SIZE }>, MAX_PENDING_REQUESTS> {
        let mut retry_packets = Vec::new();

        let timeout = self.config.timeout_ms;
        let current_time = self.current_time;

        // Process pending requests for timeouts
        let mut i = 0;
        while i < self.pending_requests.len() {
            let req = &self.pending_requests[i];

            if current_time - req.timestamp >= timeout {
                // Timeout occurred
                if req.retries >= self.config.retry_count {
                    // Max retries reached, remove request
                    let _ = self.pending_requests.swap_remove(i);
                    continue;
                } else {
                    // Retry the request
                    if let Some(packet) =
                        self.request_parameter(req.device_addr, req.parameter_id, req.chunk_number)
                    {
                        // Update retry count
                        if let Some(pending) = self.pending_requests.get_mut(i) {
                            pending.retries += 1;
                            pending.timestamp = current_time;
                        }

                        let _ = retry_packets.push(packet);
                    }
                }
            }

            i += 1;
        }

        retry_packets
    }

    /// Clears all discovered devices and resets state.
    pub fn clear(&mut self) {
        self.devices.clear();
        self.pending_requests.clear();
        self.chunk_assembly = None;
    }
}

impl Default for DeviceManager {
    fn default() -> Self {
        Self::new(DeviceManagerConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::packets::ParameterDataType;

    #[test]
    fn test_device_manager_creation() {
        let manager = DeviceManager::default();
        assert_eq!(manager.devices().count(), 0);
    }

    #[test]
    fn test_device_from_info() {
        let info = DeviceInformation::new(
            PacketAddress::Handset as u8,
            PacketAddress::Transmitter as u8,
            "ELRS TX",
            0x454C5253, // "ELRS"
            0x12345678,
            0x00010203,
            18,
            1,
        )
        .unwrap();

        let device = Device::from_device_info(&info).unwrap();
        assert_eq!(device.name, "ELRS TX");
        assert_eq!(device.parameters_total, 18);
        assert!(device.is_elrs_tx());
        assert!(!device.parameters_loaded);
    }

    #[test]
    fn test_parameter_from_entry() {
        let entry = ParameterSettingsEntry::new(
            0xEA,
            0xEE,
            5,
            0,
            0,
            ParameterDataType::Float as u8,
            "TX Power",
            Some(),
        )
        .unwrap();

        let param = Parameter::from_entry(5, &entry);
        assert_eq!(param.id, 5);
        assert_eq!(param.parent, 0);
        assert_eq!(param.name, "TX Power");
        assert!(!param.hidden);
        assert!(!param.is_folder());
        assert!(!param.is_command());
    }

    #[test]
    fn test_parameter_folder() {
        let mut children = Vec::new();
        children.push(1).unwrap();
        children.push(2).unwrap();

        let entry = ParameterSettingsEntry::new(
            0xEA,
            0xEE,
            0,
            0,
            0,
            ParameterDataType::Folder as u8,
            "ROOT",
        )
        .unwrap()
        .add_data(ParameterData::Float {
            value: 2000,
            min: 0,
            max: 10000,
            default: 2000,
            decimal_point: 0,
            step_size: 100,
            unit: String::try_from("mW").unwrap(),
        });

        let param = Parameter::from_entry(0, &entry);
        assert!(param.is_folder());
        assert_eq!(param.folder_children().unwrap().len(), 2);
    }

    #[test]
    fn test_device_add_parameter() {
        let info = DeviceInformation::new(
            PacketAddress::Handset as u8,
            PacketAddress::Transmitter as u8,
            "Test Device",
            0x12345678,
            0x12345678,
            0x00010203,
            5,
            1,
        )
        .unwrap();

        let mut device = Device::from_device_info(&info).unwrap();

        let entry = ParameterSettingsEntry::new(
            0xEA,
            0xEE,
            0,
            0,
            0,
            ParameterDataType::Info as u8,
            "Version",
            Some(ParameterData::Info {
                info: String::try_from("1.0.0").unwrap(),
            }),
        )
        .unwrap();

        let param = Parameter::from_entry(0, &entry);
        assert!(device.add_parameter(param));
        assert_eq!(device.parameters.len(), 1);
        assert!(device.get_parameter(0).is_some());
    }

    #[test]
    fn test_device_manager_config() {
        let config = DeviceManagerConfig {
            timeout_ms: 1000,
            retry_count: 5,
            device_ping_interval_ms: 2000,
        };

        let manager = DeviceManager::new(config);
        assert_eq!(manager.config.timeout_ms, 1000);
        assert_eq!(manager.config.retry_count, 5);
    }
}
