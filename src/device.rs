//! High-level device and parameter management API.
//!
//! This module provides abstractions for discovering CRSF devices, managing their parameters,
//! and handling the chunked parameter transfer protocol described in the CRSF specification.
//!
//! # Overview
//!
//! The main types provided by this module are:
//!
//! - [`Parameter`] - Represents a parameter with its metadata and current value
//! - [`Device`] - Represents a discovered CRSF device with its information and parameters
//! - [`DeviceManager`] - Manages device discovery, parameter reading/writing, and state
//!
//! # Example
//!
//! ```no_run
//! use uf_crsf::device::{DeviceManager, DeviceManagerConfig};
//! use uf_crsf::parser::CrsfParser;
//! use uf_crsf::packets::{Packet, PacketAddress};
//!
//! let config = DeviceManagerConfig::default();
//! let mut manager = DeviceManager::new(config);
//! let mut parser = CrsfParser::new();
//!
//! // Process incoming bytes
//! # let incoming_data: &[u8] = &[];
//! for packet_result in parser.iter_packets(incoming_data) {
//!     if let Ok(packet) = packet_result {
//!         manager.handle_packet(&packet);
//!     }
//! }
//!
//! // Check if devices were discovered
//! let device_addrs: heapless::Vec<_, 8> = manager.devices().collect();
//! if let Some(&device_id) = device_addrs.first() {
//!     // Request parameters from a device
//!     if let Some(request) = manager.request_all_parameters(device_id) {
//!         // Send request over the wire...
//!     }
//! }
//! ```

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

/// Configuration for the DeviceManager.
#[derive(Debug, Clone, Copy)]
pub struct DeviceManagerConfig {
    /// Timeout for parameter requests in milliseconds.
    pub timeout_ms: u32,
    /// Maximum number of retries for failed requests.
    pub retry_count: u8,
    /// Device ping interval in milliseconds (0 = disabled).
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

/// Represents a parameter with its metadata and current value.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Parameter {
    /// Parameter number/ID.
    pub id: u8,
    /// Parent folder parameter number (0 for root).
    pub parent: u8,
    /// Parameter name.
    pub name: String<128>,
    /// Whether this parameter is hidden.
    pub hidden: bool,
    /// Parameter data and value.
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

/// Represents a discovered CRSF device.
#[derive(Debug, Clone)]
pub struct Device {
    /// Device address.
    pub address: PacketAddress,
    /// Device name.
    pub name: String<43>,
    /// Serial number.
    pub serial_number: u32,
    /// Hardware ID.
    pub hardware_id: u32,
    /// Firmware ID.
    pub firmware_id: u32,
    /// Total number of parameters.
    pub parameters_total: u8,
    /// Parameter version number.
    pub parameter_version: u8,
    /// Loaded parameters.
    pub parameters: FnvIndexMap<u8, Parameter, MAX_PARAMETERS>,
    /// Whether all parameters have been loaded.
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

/// Device manager for discovering and managing CRSF devices and their parameters.
///
/// This struct maintains a list of discovered devices and provides methods for:
/// - Device discovery via ping/response
/// - Parameter enumeration and reading
/// - Parameter writing
/// - Chunked parameter transfer reassembly
/// - Timeout and retry handling
pub struct DeviceManager {
    /// Configuration.
    config: DeviceManagerConfig,
    /// Own device address.
    pub own_address: PacketAddress,
    /// Discovered devices indexed by address.
    devices: FnvIndexMap<PacketAddress, Device, MAX_DEVICES>,
    /// Pending parameter requests.
    pending_requests: Vec<PendingRequest, MAX_PENDING_REQUESTS>,
    /// Current chunk assembly state.
    chunk_assembly: Option<ChunkAssembly>,
    /// Monotonic timestamp counter (incremented by caller).
    current_time: u32,
    /// Last device ping timestamp.
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

    /// Updates the internal monotonic time counter.
    ///
    /// Call this periodically with a millisecond timestamp to enable timeout functionality.
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

    /// Handles an incoming CRSF packet.
    ///
    /// This processes DevicePing, DeviceInformation, ParameterSettingsEntry, and ParameterWrite responses.
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

    /// Sends a device ping to discover devices.
    ///
    /// Returns `Some` with the ping packet bytes if a ping should be sent.
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

    /// Requests all parameters from a device.
    ///
    /// Returns the first ParameterRead packet to send, or `None` if the device doesn't exist
    /// or parameters are already being loaded.
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

    /// Writes a parameter value to a device.
    ///
    /// Returns the ParameterWrite packet bytes to send.
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
            Some(ParameterData::Float {
                value: 2000,
                min: 0,
                max: 10000,
                default: 2000,
                decimal_point: 0,
                step_size: 100,
                unit: String::try_from("mW").unwrap(),
            }),
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
            Some(ParameterData::Folder {
                children: children.clone(),
            }),
        )
        .unwrap();

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
