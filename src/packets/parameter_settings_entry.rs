use crate::packets::{CrsfPacket, PacketType};
use crate::CrsfParsingError;
use heapless::String;
use heapless::Vec;

const MAX_STRING_LEN: usize = 128;
const MAX_CHILDREN: usize = 32;
// Real ELRS devices send options strings well over 200 bytes (e.g. "Packet Rate" has
// ~214 bytes of semicolon-delimited choices). Use 512 to cover the theoretical maximum
// of MAX_CHUNKS * MAX_CHUNK_PAYLOAD_SIZE minus header overhead.
const MAX_OPTIONS: usize = 512;

/// Maximum bytes of the entry payload that can fit in a single chunk.
///
/// Derivation: max CRSF frame is 64 bytes. For an extended frame:
/// 64 - sync(1) - len(1) - type(1) - dst(1) - src(1) - param_num(1)
///     - chunks_remaining(1) - crc(1) = 56 bytes for entry fragment.
const MAX_CHUNK_PAYLOAD_SIZE: usize = 56;

/// Maximum number of chunks supported when reassembling a parameter.
const MAX_CHUNKS: usize = 8;

/// Single chunk of a chunked [`ParameterSettingsEntry`] (0x2B) frame.
///
/// When a parameter's entry payload exceeds [`MAX_CHUNK_PAYLOAD_SIZE`] (56 bytes),
/// the device splits it across multiple 0x2B frames. Each chunk carries the
/// parameter number, remaining chunk count, and a fragment of the entry payload.
///
/// - **Chunk 0** fragment starts with: `parent | data_type | name\0 | [type data]`
/// - **Chunks 1+** fragment: continuation of type-specific data
///
/// Use [`ParameterChunkReassembler`] to collect chunks into a complete entry.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ParameterChunk {
    /// Destination device address.
    pub dst_addr: u8,
    /// Origin device address.
    pub src_addr: u8,
    /// Parameter number (index).
    pub param_number: u8,
    /// Remaining chunks count for this parameter (0 = this is the last chunk).
    pub chunks_remaining: u8,
    /// Raw entry payload fragment.
    ///
    /// For chunk 0 this starts with `parent | data_type | name\0`.
    /// For chunks 1+ this continues the type-specific data.
    pub payload: Vec<u8, MAX_CHUNK_PAYLOAD_SIZE>,
}

impl ParameterChunk {
    /// Parse a 0x2B frame payload as a chunk.
    ///
    /// The `data` slice is the full extended-frame payload:
    /// `dst(1) | src(1) | param_number(1) | chunks_remaining(1) | entry_fragment`.
    pub fn from_bytes(data: &[u8]) -> Result<Self, CrsfParsingError> {
        if data.len() < 4 {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }
        let dst_addr = data[0];
        let src_addr = data[1];
        let param_number = data[2];
        let chunks_remaining = data[3];
        let mut payload: Vec<u8, MAX_CHUNK_PAYLOAD_SIZE> = Vec::new();
        for &b in &data[4..] {
            payload
                .push(b)
                .map_err(|_| CrsfParsingError::InvalidPayloadLength)?;
        }
        Ok(Self {
            dst_addr,
            src_addr,
            param_number,
            chunks_remaining,
            payload,
        })
    }
}

/// Reassembles chunked [`ParameterSettingsEntry`] frames into a complete entry.
///
/// When a parameter's metadata exceeds 56 bytes, the device sends it across
/// multiple 0x2B frames (chunks). This reassembler collects chunks in arrival
/// order and produces a complete [`ParameterSettingsEntry`] once all chunks
/// are received.
///
/// # Usage
///
/// ```no_run
/// # use uf_crsf::packets::parameter_settings_entry::ParameterChunkReassembler;
/// let mut reassembler = ParameterChunkReassembler::new();
///
/// // Feed chunks as 0x2B frame payloads arrive from the device
/// for frame_payload in /* incoming stream */ {
///     let chunk = /* ParameterChunk::from_bytes(frame_payload)? */;
///     match reassembler.push(chunk)? {
///         Some(entry) => {
///             // Complete parameter received ready for use
///             break;
///         }
///         None => {
///             // More chunks expected, keep reading
///         }
///     }
/// }
/// # Ok::<_, uf_crsf::CrsfParsingError>(())
/// ```
#[derive(Clone, Debug, Default)]
pub struct ParameterChunkReassembler {
    /// Parameter number being assembled.
    param_number: u8,
    /// CRSF addresses from the chunk frames.
    dst_addr: u8,
    src_addr: u8,
    /// Buffered chunk payloads in order of arrival.
    chunks: Vec<Vec<u8, MAX_CHUNK_PAYLOAD_SIZE>, MAX_CHUNKS>,
    /// Number of chunks received so far.
    chunks_received: u8,
    /// Total expected chunks (from chunk 0's chunks_remaining + 1).
    total_chunks: u8,
    /// Assembly complete flag.
    complete: bool,
}

impl ParameterChunkReassembler {
    /// Create a fresh reassembler with no pending parameter.
    pub const fn new() -> Self {
        Self {
            param_number: 0,
            dst_addr: 0,
            src_addr: 0,
            chunks: Vec::new(),
            chunks_received: 0,
            total_chunks: 0,
            complete: false,
        }
    }

    /// Push a chunk into the reassembler.
    ///
    /// Returns `Ok(None)` when more chunks are expected, or
    /// `Ok(Some(entry))` with the complete [`ParameterSettingsEntry`]
    /// once all chunks have been collected.
    ///
    /// If a chunk for a different `param_number` arrives mid-assembly,
    /// the reassembler automatically resets and starts assembling the
    /// new parameter instead.
    pub fn push(
        &mut self,
        chunk: ParameterChunk,
    ) -> Result<Option<ParameterSettingsEntry>, CrsfParsingError> {
        // Auto-reset if a different parameter arrives mid-assembly
        if self.chunks_received > 0 && chunk.param_number != self.param_number {
            self.reset();
        }

        if self.chunks_received == 0 {
            // First chunk — initialise state from it
            self.param_number = chunk.param_number;
            self.dst_addr = chunk.dst_addr;
            self.src_addr = chunk.src_addr;
            self.total_chunks = chunk.chunks_remaining + 1;
            self.complete = false;
        }

        // Guard against exceeding the chunk budget
        if usize::from(self.chunks_received) >= MAX_CHUNKS {
            // Too many chunks — reset and report error
            self.reset();
            return Err(CrsfParsingError::InvalidPayloadLength);
        }

        // Buffer the chunk payload
        self.chunks
            .push(chunk.payload)
            .map_err(|_| CrsfParsingError::InvalidPayloadLength)?;
        self.chunks_received += 1;

        if chunk.chunks_remaining == 0 {
            // Last chunk — reassemble into a complete entry
            self.complete = true;
            let entry = self.reassemble()?;
            Ok(Some(entry))
        } else {
            Ok(None)
        }
    }

    /// Concatenate all buffered chunk payloads and parse the result.
    fn reassemble(&self) -> Result<ParameterSettingsEntry, CrsfParsingError> {
        let total_payload_len: usize = self.chunks.iter().map(|c| c.len()).sum();
        let total_size = 4 + total_payload_len; // 4-byte header + entry payload
        let mut buffer = [0u8; 4 + MAX_CHUNKS * MAX_CHUNK_PAYLOAD_SIZE];

        if total_size > buffer.len() {
            return Err(CrsfParsingError::BufferOverflow);
        }

        // Build reconstructed frame: dst | src | param_num | chunks_remaining=0 | [all fragments]
        buffer[0] = self.dst_addr;
        buffer[1] = self.src_addr;
        buffer[2] = self.param_number;
        buffer[3] = 0; // chunks_remaining = 0 (complete parameter)

        let mut offset = 4;
        for chunk in &self.chunks {
            buffer[offset..offset + chunk.len()].copy_from_slice(chunk);
            offset += chunk.len();
        }

        ParameterSettingsEntry::from_bytes(&buffer[..total_size])
    }

    /// Reset the reassembler to idle state.
    pub fn reset(&mut self) {
        self.param_number = 0;
        self.dst_addr = 0;
        self.src_addr = 0;
        self.chunks.clear();
        self.chunks_received = 0;
        self.total_chunks = 0;
        self.complete = false;
    }

    /// Returns `true` once all chunks have been received and the entry is ready.
    pub fn is_complete(&self) -> bool {
        self.complete
    }

    /// Returns the parameter number currently being assembled.
    pub fn param_number(&self) -> u8 {
        self.param_number
    }

    /// Returns the total expected number of chunks.
    pub fn total_chunks(&self) -> u8 {
        self.total_chunks
    }

    /// Returns the number of chunks received so far.
    pub fn chunks_received(&self) -> u8 {
        self.chunks_received
    }

    /// Returns `true` when no parameter assembly is in progress.
    pub fn is_idle(&self) -> bool {
        self.chunks_received == 0
    }
}

/// CRSF parameter data types (lower 7 bits of type byte).
///
/// Each parameter has a type determining its value format and constraints.
/// The type is stored in the data_type field with the hidden flag (bit 7)
/// for advanced/developer parameters.
///
/// # Common Types
///
/// - **Float (8)**: Numeric values with min/max/step (e.g., TX power in mW)
/// - **TextSelection (9)**: Enum-style choices (e.g., RF mode: "Dynamic", "Fixed")
/// - **String (10)**: Free-form text (rare, e.g., device name)
/// - **Folder (11)**: Organizational node with children (hierarchical structure)
/// - **Info (12)**: Read-only display (e.g., firmware version, link statistics)
/// - **Command (13)**: Action triggers (e.g., "Bind", "Save")
/// - **Vtx (15)**: Video transmitter parameters (band/channel/power)
///
/// # Deprecated Types
///
/// Types 0-5 (Uint8, Int8, Uint16, Int16, Uint32, Int32) are deprecated
/// in modern CRSF implementations. Use Float for numeric values.
///
/// # ExpressLRS Parameter Tree Structure
///
/// ExpressLRS devices typically organize parameters hierarchically:
/// ```text
/// ID 0: ROOT (Folder)
///   ├─ ID 1: Connection (Folder)
///   │   ├─ ID 10: Link Quality (Info)
///   │   ├─ ID 11: RF Mode (TextSelection: Dynamic/Fixed/Reserved)
///   │   └─ ID 12: Lock on First Connect (Float: 0/1)
///   ├─ ID 2: VTX (Folder)
///   │   ├─ ID 20: Band (TextSelection: A/B/E/R/F/L/T/A)
///   │   ├─ ID 21: Channel (Float: 1-8)
///   │   └─ ID 22: Power (TextSelection: 25/200/500/1000 mW...)
///   └─ ID 3: Lua (Folder)
///       └─ ID 30: Telemetry (TextSelection: Off/UART/Crossfire/ELRS)
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParameterDataType {
    /// Unsigned 8-bit integer (deprecated - use Float instead).
    Uint8 = 0,
    /// Signed 8-bit integer (deprecated - use Float instead).
    Int8 = 1,
    /// Unsigned 16-bit integer (deprecated - use Float instead).
    Uint16 = 2,
    /// Signed 16-bit integer (deprecated - use Float instead).
    Int16 = 3,
    /// Unsigned 32-bit integer (deprecated - use Float instead).
    Uint32 = 4,
    /// Signed 32-bit integer (deprecated - use Float instead).
    Int32 = 5,
    /// Floating point numeric value.
    ///
    /// The most common type for numeric parameters. Value is stored as
    /// i32 internally but represents an f32. Includes min/max/default
    /// values, decimal precision, step size, and units.
    ///
    /// **Example use cases:**
    /// - TX Power: 0-10000 mW, step 100 mW
    /// - PWM Frequency: 50-400 Hz, step 50 Hz
    /// - Telemetry Rate: 0-250 Hz, step 1 Hz
    Float = 8,
    /// Enum-style text selection from predefined options.
    ///
    /// Options are semicolon-delimited UTF-8 strings (e.g., "Off;On;Auto").
    /// The value field stores the 0-based index of selected option.
    ///
    /// **Example use cases:**
    /// - RF Mode: "Dynamic;Fixed;Reserved"
    /// - Switch Mode: "Momentary;Toggle;None"
    /// - ELRS Mode: "868;915;433"
    TextSelection = 9,
    /// Free-form string value.
    ///
    /// User-editable text with maximum length constraint. Rarely used.
    ///
    /// **Example use cases:**
    /// - Device name
    /// - Custom bind phrase (deprecated in favor of other methods)
    String = 10,
    /// Organizational folder containing child parameters.
    ///
    /// Folders organize the parameter hierarchy. The value field contains
    /// a list of child parameter IDs terminated by 0xFF. Folders are
    /// not writable - they only provide structure.
    ///
    /// **Navigation:** Start at ID 0 (root), read children list, recurse.
    Folder = 11,
    /// Read-only informational string.
    ///
    /// Display-only information shown to users. Cannot be written.
    ///
    /// **Example use cases:**
    /// - Firmware version: "v3.3.1"
    /// - Device info: "ESP32-S3 @ 240MHz"
    /// - Link quality: "RSSI: -45dBm, LQ: 98%"
    Info = 12,
    /// Command or action trigger.
    ///
    /// Writing any value to this parameter triggers the command. The status
    /// field indicates if command is executing. Commands have a timeout.
    ///
    /// **Example use cases:**
    /// - Bind: Initiate RX binding process
    /// - VTX Save: Apply and save VTX settings
    /// - Reset: Reset device to factory settings
    Command = 13,
    /// Video transmitter specific parameters.
    ///
    /// Raw binary data for VTX control. Format is VTX implementation
    /// dependent. Common in Betaflight FCs for VTX configuration.
    Vtx = 15,
    /// Out of range marker (internal use).
    ///
    /// Used to indicate invalid or out-of-bounds type values.
    OutOfRange = 127,
}

impl TryFrom<u8> for ParameterDataType {
    type Error = CrsfParsingError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        // Mask out hidden bit (bit 7)
        let type_value = value & 0x7F;
        match type_value {
            0 => Ok(ParameterDataType::Uint8),
            1 => Ok(ParameterDataType::Int8),
            2 => Ok(ParameterDataType::Uint16),
            3 => Ok(ParameterDataType::Int16),
            4 => Ok(ParameterDataType::Uint32),
            5 => Ok(ParameterDataType::Int32),
            8 => Ok(ParameterDataType::Float),
            9 => Ok(ParameterDataType::TextSelection),
            10 => Ok(ParameterDataType::String),
            11 => Ok(ParameterDataType::Folder),
            12 => Ok(ParameterDataType::Info),
            13 => Ok(ParameterDataType::Command),
            15 => Ok(ParameterDataType::Vtx),
            127 => Ok(ParameterDataType::OutOfRange),
            _ => Err(CrsfParsingError::InvalidPayload),
        }
    }
}

/// Type-specific parameter data and value.
///
/// This enum holds the value and metadata for a parameter, varying by
/// [ParameterDataType]. The DeviceManager stores these within [Parameter]
/// structures for easy access.
///
/// # Access Pattern
///
/// After receiving a [ParameterSettingsEntry], match on the `data` field:
///
/// ```no_run
/// # use uf_crsf::packets::parameter_settings_entry::ParameterData;
/// if let Some(ParameterData::Float { value, min, max, unit, .. }) = &parameter.data {
///     println!("TX Power: {} {} (range: {}-{})", value, unit, min, max);
/// } else if let Some(ParameterData::TextSelection { options, value, .. }) = &parameter.data {
///     let options_vec: Vec<&str> = options.split(';').collect();
///     if let Some(&selected) = options_vec.get(*value as usize) {
///         println!("RF Mode: {} (selected: {})", selected, value);
///     }
/// }
/// ```
///
/// # Value Encoding for Writes
///
/// When constructing [ParameterWrite] packets, encode values as follows:
///
/// - **Float**: Write 4 bytes as little-endian f32
/// - **TextSelection**: Write the option index (u8), not the string
/// - **String**: Write UTF-8 bytes without null terminator
/// - **Command**: Write any byte (e.g., [0]) to trigger
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ParameterData {
    /// Numeric floating point value with constraints.
    ///
    /// Used for continuous numeric parameters like power, frequencies, rates.
    ///
    /// **Encoding for [ParameterWrite]:** 4 bytes, little-endian f32
    /// ```rust,no_run
    /// # let value: f32 = 2000.0;
    /// let mut data = [0u8; 4];
    /// data.copy_from_slice(&value.to_le_bytes());
    /// ```
    ///
    /// **Example ExpressLRS parameters:**
    /// - TX Power: 0-10000 mW, step 100 mW, unit "mW"
    /// - Lock on First Connect: 0-1, step 1, unit "bool"
    /// - Telemetry Rate: 0-250 Hz, step 1, unit "Hz"
    Float {
        /// Current value as i32 (reinterpret f32 bits).
        value: i32,
        /// Minimum allowed value.
        min: i32,
        /// Maximum allowed value.
        max: i32,
        /// Default/reset value.
        default: i32,
        /// Number of decimal places for display (0-2).
        ///
        /// e.g., value 12500 with decimal_point 1 displays as "1250.0"
        decimal_point: u8,
        /// Minimum increment between values.
        step_size: i32,
        /// Unit label for display (e.g., "mW", "Hz", "").
        unit: String<MAX_STRING_LEN>,
    },
    /// Enum-style selection from predefined string options.
    ///
    /// Options are semicolon-delimited (e.g., "Off;On;Auto").
    /// The value field is the 0-based index into this options string.
    ///
    /// **Encoding for [ParameterWrite]:** 1 byte (option index)
    /// ```rust,no_run
    /// # let option_index: u8 = 1;
    /// let data = [option_index];  // Selects second option
    /// ```
    ///
    /// **Example ExpressLRS parameters:**
    /// - RF Mode: "Dynamic;Fixed;Reserved", default 0
    /// - Switch Mode: "Momentary;Toggle;None", default 0
    /// - ELRS Mode: "868;915;433", default 1 (915 MHz)
    TextSelection {
        /// Semicolon-delimited options string.
        ///
        /// Parse with `options.split(';')` to get individual choices.
        options: String<MAX_OPTIONS>,
        /// Index of currently selected option.
        value: u8,
        /// Minimum valid index (typically 0).
        min: u8,
        /// Maximum valid index (typically options_count - 1).
        max: u8,
        /// Default selection index.
        default: u8,
        /// Unit label (typically empty for enums).
        unit: String<MAX_STRING_LEN>,
    },
    /// Free-form text string.
    ///
    /// Used for user-editable text with length constraints.
    ///
    /// **Encoding for [ParameterWrite]:** UTF-8 bytes (no null terminator)
    /// ```rust,no_run
    /// # let text = "MyRadio";
    /// let data = text.as_bytes();  // Send as UTF-8 bytes
    /// ```
    ///
    /// **Example ExpressLRS parameters:**
    /// - Device Name: Max 16 chars, default "ELRS"
    String {
        /// Current string value.
        value: String<MAX_STRING_LEN>,
        /// Maximum character limit.
        max_length: u8,
    },
    /// Organizational folder containing child parameters.
    ///
    /// Folders define the parameter hierarchy. The children list contains
    /// parameter IDs that belong to this folder. Navigation is typically
    /// recursive: start at root (ID 0), read children, traverse down.
    ///
    /// **Not writable** - folders only provide structure.
    ///
    /// **Navigation example:**
    /// ```rust,no_run
    /// # use heapless::Vec;
    /// # let children: Vec<u8, 32> = Vec::new();
    /// for &child_id in &children {
    ///     // Load parameter via DeviceManager
    ///     if let Some(child) = device.get_parameter(child_id) {
    ///         if child.is_folder() {
    ///             // Recurse into folder
    ///         } else {
    ///             // Display parameter
    ///         }
    ///     }
    /// }
    /// ```
    Folder { children: Vec<u8, MAX_CHILDREN> },
    /// Read-only informational display.
    ///
    /// Contains text to display to users, typically device status or version info.
    ///
    /// **Not writable** - info is for display only.
    ///
    /// **Example ExpressLRS parameters:**
    /// - Link Quality: "RSSI: -45dBm, SNR: 20, LQ: 98%"
    /// - Firmware: "v3.3.1"
    /// - Device Info: "ESP32-S3 @ 240MHz"
    Info { info: String<MAX_STRING_LEN> },
    /// Command or action trigger.
    ///
    /// Commands are triggered by writing any value to this parameter.
    /// The status field indicates if command is currently executing.
    /// Commands have a timeout to prevent indefinite blocking.
    ///
    /// **Encoding for [ParameterWrite]:** Any byte (e.g., `[0]`)
    /// ```rust,no_run
    /// let data = [0];  // Triggers command
    /// ```
    ///
    /// **Command status values:**
    /// - 0: Not started / completed
    /// - 1: In progress
    /// - 2: Executing
    ///
    /// **Example ExpressLRS parameters:**
    /// - Bind: Trains RX binding (timeout ~10s)
    /// - VTX Save: Applies VTX settings (timeout ~2s)
    /// - Reset: Factory reset (timeout ~5s)
    Command {
        /// Current execution status.
        status: u8,
        /// Timeout in seconds (after this, status resets).
        timeout: u8,
        /// Description of command action.
        info: String<MAX_STRING_LEN>,
    },
    /// Video transmitter raw binary data.
    ///
    /// VTX parameters are implementation-specific binary data, typically
    /// used by Betaflight FCs for VTX configuration.
    ///
    /// **Encoding for [ParameterWrite]:** Raw bytes as defined by VTX protocol
    ///
    /// **Format varies by VTX implementation** - refer to device documentation.
    Vtx { data: Vec<u8, 64> },
}

/// Represents a Parameter Settings Entry packet (0x2B).
///
/// Used to share parameter information between devices.
/// Can represent different parameter types: FLOAT, TEXT_SELECTION, STRING, FOLDER, INFO, COMMAND.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParameterSettingsEntry {
    /// Destination device address.
    pub dst_addr: u8,
    /// Origin device address.
    pub src_addr: u8,
    /// Parameter number (index).
    pub parameter_number: u8,
    /// Chunks remaining to be read for this parameter.
    pub chunks_remaining: u8,
    /// Parent folder parameter number (0 for root folder).
    pub parent: u8,
    /// Data type (including hidden flag in bit 7).
    pub data_type: u8,
    /// Parameter name.
    pub name: String<MAX_STRING_LEN>,
    /// Parameter-specific data.
    pub data: Option<ParameterData>,
}

impl ParameterSettingsEntry {
    /// Creates a new ParameterSettingsEntry packet.
    pub fn new(
        dst_addr: u8,
        src_addr: u8,
        parameter_number: u8,
        chunks_remaining: u8,
        parent: u8,
        data_type: u8,
        name: &str,
    ) -> Result<Self, CrsfParsingError> {
        if name.len() > MAX_STRING_LEN {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }
        let mut s = String::new();
        s.push_str(name)
            .map_err(|_| CrsfParsingError::InvalidPayloadLength)?;
        Ok(Self {
            dst_addr,
            src_addr,
            parameter_number,
            chunks_remaining,
            parent,
            data_type,
            name: s,
            data: None,
        })
    }

    pub fn add_data(self, data: ParameterData) -> Self {
        Self {
            data: Some(data),
            ..self
        }
    }

    /// Returns the data type without the hidden bit.
    pub fn get_data_type(&self) -> Result<ParameterDataType, CrsfParsingError> {
        ParameterDataType::try_from(self.data_type)
    }

    /// Returns true if the hidden bit (bit 7) is set.
    pub fn is_hidden(&self) -> bool {
        (self.data_type & 0x80) != 0
    }

    /// Helper function to find null terminator in a slice.
    fn find_null(data: &[u8]) -> Result<Option<usize>, CrsfParsingError> {
        Ok(data.iter().position(|&b| b == 0))
    }

    /// Helper function to parse a null-terminated string.
    fn parse_string(data: &[u8]) -> Result<(String<MAX_STRING_LEN>, usize), CrsfParsingError> {
        let null_pos = Self::find_null(data)?.ok_or(CrsfParsingError::InvalidPayload)?;
        let str_slice = &data[..null_pos];
        let s = core::str::from_utf8(str_slice).map_err(|_| CrsfParsingError::InvalidPayload)?;
        let mut string = String::new();
        string
            .push_str(s)
            .map_err(|_| CrsfParsingError::InvalidPayloadLength)?;
        Ok((string, null_pos + 1))
    }

    fn parse_options_string(
        data: &[u8],
    ) -> Result<(String<MAX_OPTIONS>, usize), CrsfParsingError> {
        let null_pos = Self::find_null(data)?.ok_or(CrsfParsingError::InvalidPayload)?;
        let str_slice = &data[..null_pos];
        let s = core::str::from_utf8(str_slice).map_err(|_| CrsfParsingError::InvalidPayload)?;
        let mut string = String::new();
        string
            .push_str(s)
            .map_err(|_| CrsfParsingError::InvalidPayloadLength)?;
        Ok((string, null_pos + 1))
    }

    /// Helper function to find end marker (0xFF) in a list.
    fn find_end_marker(data: &[u8]) -> Option<usize> {
        data.iter().position(|&b| b == 0xFF)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for ParameterData {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            ParameterData::Float {
                value,
                min,
                max,
                default,
                decimal_point,
                step_size,
                unit,
            } => {
                defmt::write!(
                    fmt,
                    "Float {{ value: {=i32}, min: {=i32}, max: {=i32}, default: {=i32}, decimal_point: {=u8}, step_size: {=i32}, unit: {:a} }}",
                    value, min, max, default, decimal_point, step_size, unit.as_bytes()
                )
            }
            ParameterData::TextSelection {
                options,
                value,
                min,
                max,
                default,
                unit,
            } => {
                defmt::write!(
                    fmt,
                    "TextSelection {{ options: {:a}, value: {=u8}, min: {=u8}, max: {=u8}, default: {=u8}, unit: {:a} }}",
                    options.as_bytes(), value, min, max, default, unit.as_bytes()
                )
            }
            ParameterData::String { value, max_length } => {
                defmt::write!(
                    fmt,
                    "String {{ value: {:a}, max_length: {=u8} }}",
                    value.as_bytes(),
                    max_length
                )
            }
            ParameterData::Folder { children } => {
                defmt::write!(fmt, "Folder {{ children: [..{=usize}] }}", children.len())
            }
            ParameterData::Info { info } => {
                defmt::write!(fmt, "Info {{ info: {:a} }}", info.as_bytes())
            }
            ParameterData::Command {
                status,
                timeout,
                info,
            } => {
                defmt::write!(
                    fmt,
                    "Command {{ status: {=u8}, timeout: {=u8}, info: {:a} }}",
                    status,
                    timeout,
                    info.as_bytes()
                )
            }
            ParameterData::Vtx { data } => {
                defmt::write!(fmt, "Vtx {{ data: [..{=usize}] }}", data.len())
            }
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for ParameterSettingsEntry {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "ParameterSettingsEntry {{ parent: {=u8}, data_type: {=u8}, name: {:a}, hidden: {=bool}, data: {} }}",
            self.parent,
            self.data_type,
            self.name.as_bytes(),
            self.is_hidden(),
            self.data
        )
    }
}

impl CrsfPacket for ParameterSettingsEntry {
    const PACKET_TYPE: PacketType = PacketType::ParameterSettingsEntry;
    // Minimum: dst (1) + src (1) + param# (1) + chunks (1) + parent (1) + data_type (1) + name terminator (1) = 7 bytes
    const MIN_PAYLOAD_SIZE: usize = 7;

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, CrsfParsingError> {
        let name_bytes = self.name.as_bytes();
        let name_len = name_bytes.len();
        let mut offset = 0;

        // Destination address
        if offset + 1 > buffer.len() {
            return Err(CrsfParsingError::BufferOverflow);
        }
        buffer[offset] = self.dst_addr;
        offset += 1;

        // Origin address
        if offset + 1 > buffer.len() {
            return Err(CrsfParsingError::BufferOverflow);
        }
        buffer[offset] = self.src_addr;
        offset += 1;

        // Parameter number
        if offset + 1 > buffer.len() {
            return Err(CrsfParsingError::BufferOverflow);
        }
        buffer[offset] = self.parameter_number;
        offset += 1;

        // Chunks remaining
        if offset + 1 > buffer.len() {
            return Err(CrsfParsingError::BufferOverflow);
        }
        buffer[offset] = self.chunks_remaining;
        offset += 1;

        // Parent folder
        if offset + 1 > buffer.len() {
            return Err(CrsfParsingError::BufferOverflow);
        }
        buffer[offset] = self.parent;
        offset += 1;

        // Data type
        if offset + 1 > buffer.len() {
            return Err(CrsfParsingError::BufferOverflow);
        }
        buffer[offset] = self.data_type;
        offset += 1;

        // Name with null terminator
        if offset + name_len + 1 > buffer.len() {
            return Err(CrsfParsingError::BufferOverflow);
        }
        buffer[offset..offset + name_len].copy_from_slice(name_bytes);
        offset += name_len;
        buffer[offset] = 0;
        offset += 1;

        // Parameter-specific data
        if let Some(ref data) = self.data {
            match data {
                ParameterData::Float {
                    value,
                    min,
                    max,
                    default,
                    decimal_point,
                    step_size,
                    unit,
                } => {
                    let unit_bytes = unit.as_bytes();
                    let unit_len = unit_bytes.len();
                    let required = 20 + unit_len + 1; // 5 * i32 + 1 * u8 + unit + null
                    if offset + required > buffer.len() {
                        return Err(CrsfParsingError::BufferOverflow);
                    }

                    buffer[offset..offset + 4].copy_from_slice(&value.to_be_bytes());
                    offset += 4;
                    buffer[offset..offset + 4].copy_from_slice(&min.to_be_bytes());
                    offset += 4;
                    buffer[offset..offset + 4].copy_from_slice(&max.to_be_bytes());
                    offset += 4;
                    buffer[offset..offset + 4].copy_from_slice(&default.to_be_bytes());
                    offset += 4;
                    buffer[offset] = *decimal_point;
                    offset += 1;
                    buffer[offset..offset + 4].copy_from_slice(&step_size.to_be_bytes());
                    offset += 4;
                    buffer[offset..offset + unit_len].copy_from_slice(unit_bytes);
                    offset += unit_len;
                    buffer[offset] = 0;
                    offset += 1;
                }
                ParameterData::TextSelection {
                    options,
                    value,
                    min,
                    max,
                    default,
                    unit,
                } => {
                    let options_bytes = options.as_bytes();
                    let options_len = options_bytes.len();
                    let unit_bytes = unit.as_bytes();
                    let unit_len = unit_bytes.len();
                    let required = options_len + 1 + 4 + unit_len + 1; // options + null + value/min/max/default + unit + null
                    if offset + required > buffer.len() {
                        return Err(CrsfParsingError::BufferOverflow);
                    }

                    buffer[offset..offset + options_len].copy_from_slice(options_bytes);
                    offset += options_len;
                    buffer[offset] = 0;
                    offset += 1;
                    buffer[offset] = *value;
                    offset += 1;
                    buffer[offset] = *min;
                    offset += 1;
                    buffer[offset] = *max;
                    offset += 1;
                    buffer[offset] = *default;
                    offset += 1;
                    buffer[offset..offset + unit_len].copy_from_slice(unit_bytes);
                    offset += unit_len;
                    buffer[offset] = 0;
                    offset += 1;
                }
                ParameterData::String { value, max_length } => {
                    let value_bytes = value.as_bytes();
                    let value_len = value_bytes.len();
                    let required = value_len + 1 + 1; // value + null + max_length
                    if offset + required > buffer.len() {
                        return Err(CrsfParsingError::BufferOverflow);
                    }

                    buffer[offset..offset + value_len].copy_from_slice(value_bytes);
                    offset += value_len;
                    buffer[offset] = 0;
                    offset += 1;
                    buffer[offset] = *max_length;
                    offset += 1;
                }
                ParameterData::Folder { children } => {
                    let required = children.len() + 1; // children + end marker
                    if offset + required > buffer.len() {
                        return Err(CrsfParsingError::BufferOverflow);
                    }

                    for (i, &child) in children.iter().enumerate() {
                        buffer[offset + i] = child;
                    }
                    offset += children.len();
                    buffer[offset] = 0xFF;
                    offset += 1;
                }
                ParameterData::Info { info } => {
                    let info_bytes = info.as_bytes();
                    let info_len = info_bytes.len();
                    let required = info_len + 1; // info + null
                    if offset + required > buffer.len() {
                        return Err(CrsfParsingError::BufferOverflow);
                    }

                    buffer[offset..offset + info_len].copy_from_slice(info_bytes);
                    offset += info_len;
                    buffer[offset] = 0;
                    offset += 1;
                }
                ParameterData::Command {
                    status,
                    timeout,
                    info,
                } => {
                    let info_bytes = info.as_bytes();
                    let info_len = info_bytes.len();
                    let required = 2 + info_len + 1; // status + timeout + info + null
                    if offset + required > buffer.len() {
                        return Err(CrsfParsingError::BufferOverflow);
                    }

                    buffer[offset] = *status;
                    offset += 1;
                    buffer[offset] = *timeout;
                    offset += 1;
                    buffer[offset..offset + info_len].copy_from_slice(info_bytes);
                    offset += info_len;
                    buffer[offset] = 0;
                    offset += 1;
                }
                ParameterData::Vtx { data } => {
                    let required = data.len();
                    if offset + required > buffer.len() {
                        return Err(CrsfParsingError::BufferOverflow);
                    }

                    buffer[offset..offset + data.len()].copy_from_slice(data);
                    offset += data.len();
                }
            }
        }

        Ok(offset)
    }

    fn from_bytes(data: &[u8]) -> Result<Self, CrsfParsingError> {
        if data.len() < Self::MIN_PAYLOAD_SIZE {
            return Err(CrsfParsingError::InvalidPayloadLength);
        }

        let dst_addr = data[0];
        let src_addr = data[1];
        let parameter_number = data[2];
        let chunks_remaining = data[3];
        let parent = data[4];
        let data_type = data[5];
        let mut offset = 6;

        // Parse name
        let (name, name_end) = Self::parse_string(&data[offset..])?;
        offset += name_end;

        // Parse parameter-specific data based on type
        let param_type = ParameterDataType::try_from(data_type)?;
        let param_data = if data.len() > offset {
            Some(match param_type {
                ParameterDataType::Float => {
                    // Need: value (4) + min (4) + max (4) + default (4) + decimal (1) + step (4) + unit + null
                    if data.len() < offset + 21 {
                        return Err(CrsfParsingError::InvalidPayloadLength);
                    }
                    let value = i32::from_be_bytes(
                        data[offset..offset + 4]
                            .try_into()
                            .map_err(|_| CrsfParsingError::InvalidPayloadLength)?,
                    );
                    offset += 4;
                    let min = i32::from_be_bytes(
                        data[offset..offset + 4]
                            .try_into()
                            .map_err(|_| CrsfParsingError::InvalidPayloadLength)?,
                    );
                    offset += 4;
                    let max = i32::from_be_bytes(
                        data[offset..offset + 4]
                            .try_into()
                            .map_err(|_| CrsfParsingError::InvalidPayloadLength)?,
                    );
                    offset += 4;
                    let default = i32::from_be_bytes(
                        data[offset..offset + 4]
                            .try_into()
                            .map_err(|_| CrsfParsingError::InvalidPayloadLength)?,
                    );
                    offset += 4;
                    let decimal_point = data[offset];
                    offset += 1;
                    let step_size = i32::from_be_bytes(
                        data[offset..offset + 4]
                            .try_into()
                            .map_err(|_| CrsfParsingError::InvalidPayloadLength)?,
                    );
                    offset += 4;
                    let (unit, _unit_end) = Self::parse_string(&data[offset..])?;
                    ParameterData::Float {
                        value,
                        min,
                        max,
                        default,
                        decimal_point,
                        step_size,
                        unit,
                    }
                }
                ParameterDataType::TextSelection => {
                    // Need: options + null + value (1) + min (1) + max (1) + default (1) + unit + null
                    if data.len() < offset + 4 {
                        return Err(CrsfParsingError::InvalidPayloadLength);
                    }
                    let (options, options_end) = Self::parse_options_string(&data[offset..])?;
                    offset += options_end;
                    if data.len() < offset + 4 {
                        return Err(CrsfParsingError::InvalidPayloadLength);
                    }
                    let value = data[offset];
                    offset += 1;
                    let min = data[offset];
                    offset += 1;
                    let max = data[offset];
                    offset += 1;
                    let default = data[offset];
                    offset += 1;
                    let (unit, _) = Self::parse_string(&data[offset..])?;
                    ParameterData::TextSelection {
                        options,
                        value,
                        min,
                        max,
                        default,
                        unit,
                    }
                }
                ParameterDataType::String => {
                    // Need: value + null + max_length (1)
                    if data.len() < offset + 2 {
                        return Err(CrsfParsingError::InvalidPayloadLength);
                    }
                    let (value, value_end) = Self::parse_string(&data[offset..])?;
                    offset += value_end;
                    if data.len() < offset + 1 {
                        return Err(CrsfParsingError::InvalidPayloadLength);
                    }
                    let max_length = data[offset];
                    ParameterData::String { value, max_length }
                }
                ParameterDataType::Folder => {
                    // Need: children list + end marker (0xFF)
                    let end_pos = Self::find_end_marker(&data[offset..])
                        .ok_or(CrsfParsingError::InvalidPayload)?;
                    let children_slice = &data[offset..offset + end_pos];
                    let mut children = Vec::new();
                    for &child in children_slice {
                        children
                            .push(child)
                            .map_err(|_| CrsfParsingError::InvalidPayloadLength)?;
                    }
                    ParameterData::Folder { children }
                }
                ParameterDataType::Info => {
                    // Need: info + null
                    let (info, _) = Self::parse_string(&data[offset..])?;
                    ParameterData::Info { info }
                }
                ParameterDataType::Command => {
                    // Need: status (1) + timeout (1) + info + null
                    if data.len() < offset + 3 {
                        return Err(CrsfParsingError::InvalidPayloadLength);
                    }
                    let status = data[offset];
                    offset += 1;
                    let timeout = data[offset];
                    offset += 1;
                    let (info, _) = Self::parse_string(&data[offset..])?;
                    ParameterData::Command {
                        status,
                        timeout,
                        info,
                    }
                }
                ParameterDataType::Vtx => {
                    // VTX-specific data: remaining bytes
                    let mut vtx_data = Vec::new();
                    for &byte in &data[offset..] {
                        vtx_data
                            .push(byte)
                            .map_err(|_| CrsfParsingError::InvalidPayloadLength)?;
                    }
                    ParameterData::Vtx { data: vtx_data }
                }
                _ => return Err(CrsfParsingError::InvalidPayload),
            })
        } else {
            None
        };

        Ok(Self {
            dst_addr,
            src_addr,
            parameter_number,
            chunks_remaining,
            parent,
            data_type,
            name,
            data: param_data,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parameter_settings_entry_float() {
        let unit_str = String::try_from("mW").unwrap();
        let data = ParameterData::Float {
            value: 2000,
            min: 0,
            max: 10000,
            default: 2000,
            decimal_point: 0,
            step_size: 100,
            unit: unit_str.clone(),
        };

        let entry = ParameterSettingsEntry::new(0xEA, 0xEE, 2, 0, 0, 0x08, "Power")
            .unwrap()
            .add_data(data);

        let mut buffer = [0u8; 64];
        let len = entry.to_bytes(&mut buffer).unwrap();

        let round_trip_entry = ParameterSettingsEntry::from_bytes(&buffer[..len]).unwrap();
        assert_eq!(round_trip_entry.dst_addr, 0xEA);
        assert_eq!(round_trip_entry.src_addr, 0xEE);
        assert_eq!(round_trip_entry.parameter_number, 2);
        assert_eq!(round_trip_entry.chunks_remaining, 0);
        assert_eq!(round_trip_entry.parent, 0);
        assert_eq!(round_trip_entry.name, "Power");
        assert!(!round_trip_entry.is_hidden());
        assert_eq!(
            round_trip_entry.get_data_type().unwrap(),
            ParameterDataType::Float
        );

        if let Some(ParameterData::Float {
            value,
            min,
            max,
            default,
            decimal_point,
            step_size,
            unit,
        }) = round_trip_entry.data
        {
            assert_eq!(value, 2000);
            assert_eq!(min, 0);
            assert_eq!(max, 10000);
            assert_eq!(default, 2000);
            assert_eq!(decimal_point, 0);
            assert_eq!(step_size, 100);
            assert_eq!(unit, unit_str);
        } else {
            panic!("Expected Float data");
        }
    }

    #[test]
    fn test_parameter_settings_entry_hidden_flag() {
        let data = ParameterData::Info {
            info: String::try_from("Hidden parameter").unwrap(),
        };
        let entry = ParameterSettingsEntry::new(0xEA, 0xEE, 5, 0, 0, 0x8C, "Secret")
            .unwrap()
            .add_data(data); // 0x0C | 0x80

        let mut buffer = [0u8; 64];
        let len = entry.to_bytes(&mut buffer).unwrap();

        let round_trip_entry = ParameterSettingsEntry::from_bytes(&buffer[..len]).unwrap();
        assert_eq!(round_trip_entry.data_type, 0x8C);
        assert!(round_trip_entry.is_hidden());
        assert_eq!(
            round_trip_entry.get_data_type().unwrap(),
            ParameterDataType::Info
        );
    }

    #[test]
    fn test_parameter_settings_entry_text_selection() {
        let options_str = String::try_from("250;500;1kHz").unwrap();
        let unit_str = String::try_from("Hz").unwrap();
        let data = ParameterData::TextSelection {
            options: options_str.clone(),
            value: 1,
            min: 0,
            max: 2,
            default: 1,
            unit: unit_str.clone(),
        };

        let entry = ParameterSettingsEntry::new(0xEA, 0xEE, 1, 0, 0, 0x09, "Rate")
            .unwrap()
            .add_data(data);

        let mut buffer = [0u8; 64];
        let len = entry.to_bytes(&mut buffer).unwrap();

        let round_trip_entry = ParameterSettingsEntry::from_bytes(&buffer[..len]).unwrap();
        assert_eq!(round_trip_entry.name, "Rate");

        if let Some(ParameterData::TextSelection {
            options,
            value,
            min,
            max,
            default,
            unit,
        }) = round_trip_entry.data
        {
            assert_eq!(options, options_str);
            assert_eq!(value, 1);
            assert_eq!(min, 0);
            assert_eq!(max, 2);
            assert_eq!(default, 1);
            assert_eq!(unit, unit_str);
        } else {
            panic!("Expected TextSelection data");
        }
    }

    #[test]
    fn test_parameter_settings_entry_folder() {
        let mut children = Vec::new();
        children.push(1).unwrap();
        children.push(2).unwrap();
        children.push(3).unwrap();

        let data = ParameterData::Folder {
            children: children.clone(),
        };

        let entry = ParameterSettingsEntry::new(0xEA, 0xEE, 0, 0, 0, 0x0B, "ROOT")
            .unwrap()
            .add_data(data);

        let mut buffer = [0u8; 64];
        let len = entry.to_bytes(&mut buffer).unwrap();

        let round_trip_entry = ParameterSettingsEntry::from_bytes(&buffer[..len]).unwrap();
        assert_eq!(round_trip_entry.name, "ROOT");

        if let Some(ParameterData::Folder {
            children: rt_children,
        }) = round_trip_entry.data
        {
            assert_eq!(rt_children.len(), 3);
            assert_eq!(rt_children[0], 1);
            assert_eq!(rt_children[1], 2);
            assert_eq!(rt_children[2], 3);
        } else {
            panic!("Expected Folder data");
        }
    }

    #[test]
    fn test_parameter_settings_entry_command() {
        let info_str = String::try_from("Binding...").unwrap();
        let data = ParameterData::Command {
            status: 2, // lcsExecuting
            timeout: 200,
            info: info_str.clone(),
        };

        let entry = ParameterSettingsEntry::new(0xEA, 0xEE, 10, 0, 0, 0x0D, "Bind")
            .unwrap()
            .add_data(data);

        let mut buffer = [0u8; 64];
        let len = entry.to_bytes(&mut buffer).unwrap();

        let round_trip_entry = ParameterSettingsEntry::from_bytes(&buffer[..len]).unwrap();
        assert_eq!(round_trip_entry.name, "Bind");

        if let Some(ParameterData::Command {
            status,
            timeout,
            info,
        }) = round_trip_entry.data
        {
            assert_eq!(status, 2);
            assert_eq!(timeout, 200);
            assert_eq!(info, info_str);
        } else {
            panic!("Expected Command data");
        }
    }

    #[test]
    fn test_parameter_settings_entry_string() {
        let value_str = String::try_from("MyDevice").unwrap();
        let data = ParameterData::String {
            value: value_str.clone(),
            max_length: 16,
        };

        let entry = ParameterSettingsEntry::new(0xEA, 0xEE, 4, 0, 0, 0x0A, "Name")
            .unwrap()
            .add_data(data);

        let mut buffer = [0u8; 64];
        let len = entry.to_bytes(&mut buffer).unwrap();

        let round_trip_entry = ParameterSettingsEntry::from_bytes(&buffer[..len]).unwrap();
        assert_eq!(round_trip_entry.name, "Name");

        if let Some(ParameterData::String { value, max_length }) = round_trip_entry.data {
            assert_eq!(value, value_str);
            assert_eq!(max_length, 16);
        } else {
            panic!("Expected String data");
        }
    }

    #[test]
    fn test_parameter_settings_entry_minimal() {
        // Only name, no data
        let entry = ParameterSettingsEntry::new(0xEA, 0xEE, 4, 0, 0, 0x08, "Test").unwrap();

        let mut buffer = [0u8; 64];
        let len = entry.to_bytes(&mut buffer).unwrap();

        let round_trip_entry = ParameterSettingsEntry::from_bytes(&buffer[..len]).unwrap();
        assert_eq!(round_trip_entry.parent, 0);
        assert_eq!(round_trip_entry.name, "Test");
        assert!(round_trip_entry.data.is_none());
    }

    #[test]
    fn test_parameter_settings_entry_invalid_data_type() {
        // Test with valid parent and type but invalid name (no null terminator)
        let data: [u8; 7] = [0xEA, 0xEE, 0, 0, 0, 0xFF, 0xAA]; // Invalid type, no null-terminated name
        let result = ParameterSettingsEntry::from_bytes(&data);
        assert!(matches!(result, Err(CrsfParsingError::InvalidPayload)));
    }

    #[test]
    fn test_parameter_data_type_from_u8() {
        assert_eq!(
            ParameterDataType::try_from(0x08).unwrap(),
            ParameterDataType::Float
        );
        assert_eq!(
            ParameterDataType::try_from(0x09).unwrap(),
            ParameterDataType::TextSelection
        );
        assert_eq!(
            ParameterDataType::try_from(0x0A).unwrap(),
            ParameterDataType::String
        );
        assert_eq!(
            ParameterDataType::try_from(0x0B).unwrap(),
            ParameterDataType::Folder
        );
        assert_eq!(
            ParameterDataType::try_from(0x0C).unwrap(),
            ParameterDataType::Info
        );
        assert_eq!(
            ParameterDataType::try_from(0x0D).unwrap(),
            ParameterDataType::Command
        );
    }

    #[test]
    fn test_parameter_data_type_hidden_bit() {
        // 0x0C with hidden bit (0x80) set = 0x8C
        assert_eq!(
            ParameterDataType::try_from(0x8C).unwrap(),
            ParameterDataType::Info
        );
    }

    #[test]
    fn test_parameter_settings_entry_from_bytes_too_short() {
        let data: [u8; 6] = [0xEA, 0xEE, 0, 0, 0, 0x08];
        let result = ParameterSettingsEntry::from_bytes(&data);
        assert!(matches!(
            result,
            Err(CrsfParsingError::InvalidPayloadLength)
        ));
    }

    #[test]
    fn test_parameter_settings_entry_name_too_long() {
        let long_name = "This name is way too long and should fail validation";
        let result = ParameterSettingsEntry::new(0xEA, 0xEE, 0, 0, 0, 0x08, long_name);
        // With MAX_STRING_LEN = 128, this should now pass
        assert!(result.is_ok());
    }

    #[test]
    fn test_parameter_data_type_vtx() {
        assert_eq!(
            ParameterDataType::try_from(0x0F).unwrap(),
            ParameterDataType::Vtx
        );
    }

    #[test]
    fn test_parameter_settings_entry_vtx() {
        let mut vtx_data = Vec::new();
        vtx_data.push(0x01).unwrap();
        vtx_data.push(0x02).unwrap();
        vtx_data.push(0x03).unwrap();

        let data = ParameterData::Vtx {
            data: vtx_data.clone(),
        };

        let entry = ParameterSettingsEntry::new(0xEA, 0xEE, 12, 0, 0, 0x0F, "VTX Param")
            .unwrap()
            .add_data(data);

        let mut buffer = [0u8; 64];
        let len = entry.to_bytes(&mut buffer).unwrap();

        let round_trip_entry = ParameterSettingsEntry::from_bytes(&buffer[..len]).unwrap();
        assert_eq!(round_trip_entry.name, "VTX Param");
        assert!(!round_trip_entry.is_hidden());
        assert_eq!(
            round_trip_entry.get_data_type().unwrap(),
            ParameterDataType::Vtx
        );

        if let Some(ParameterData::Vtx { data }) = round_trip_entry.data {
            assert_eq!(data.len(), 3);
            assert_eq!(data.as_slice(), [0x01, 0x02, 0x03]);
        } else {
            panic!("Expected Vtx data");
        }
    }

    #[test]
    fn test_parameter_settings_entry_vtx_with_hidden_flag() {
        let mut vtx_data = Vec::new();
        vtx_data.push(0xFF).unwrap();
        vtx_data.push(0xFE).unwrap();

        let data = ParameterData::Vtx {
            data: vtx_data.clone(),
        };

        let entry = ParameterSettingsEntry::new(0xEA, 0xEE, 12, 0, 0, 0x8F, "Hidden VTX")
            .unwrap()
            .add_data(data); // 0x0F | 0x80

        let mut buffer = [0u8; 64];
        let len = entry.to_bytes(&mut buffer).unwrap();

        let round_trip_entry = ParameterSettingsEntry::from_bytes(&buffer[..len]).unwrap();
        assert_eq!(round_trip_entry.data_type, 0x8F);
        assert!(round_trip_entry.is_hidden());
        assert_eq!(
            round_trip_entry.get_data_type().unwrap(),
            ParameterDataType::Vtx
        );
    }

    // -----------------------------------------------------------------------
    // Chunked parameter parsing tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_parameter_chunk_single() {
        // A single-chunk parameter (no actual chunking needed)
        let data: [u8; 10] = [
            0xEA, 0xEE, // dst, src
            0x05, 0x00, // param_number=5, chunks_remaining=0
            0x00, // parent=0
            0x08, // data_type=Float
            0x50, 0x6F, 0x77, 0x00, // "Pow\0"
        ];
        let chunk = ParameterChunk::from_bytes(&data).unwrap();
        assert_eq!(chunk.dst_addr, 0xEA);
        assert_eq!(chunk.src_addr, 0xEE);
        assert_eq!(chunk.param_number, 5);
        assert_eq!(chunk.chunks_remaining, 0);
        assert_eq!(
            chunk.payload.as_slice(),
            &[0x00, 0x08, 0x50, 0x6F, 0x77, 0x00]
        );
    }

    #[test]
    fn test_parameter_chunk_too_short() {
        let data: [u8; 3] = [0xEA, 0xEE, 0x05];
        let result = ParameterChunk::from_bytes(&data);
        assert!(matches!(
            result,
            Err(CrsfParsingError::InvalidPayloadLength)
        ));
    }

    #[test]
    fn test_parameter_chunk_payload_overflow() {
        // Create a chunk with more than MAX_CHUNK_PAYLOAD_SIZE bytes of payload
        let mut data = Vec::<u8, 128>::new();
        data.push(0xEA).unwrap(); // dst
        data.push(0xEE).unwrap(); // src
        data.push(0x01).unwrap(); // param_number
        data.push(0x00).unwrap(); // chunks_remaining
                                  // Fill payload with MAX_CHUNK_PAYLOAD_SIZE + 1 bytes
        for _ in 0..=MAX_CHUNK_PAYLOAD_SIZE {
            data.push(0xAA).unwrap();
        }
        let result = ParameterChunk::from_bytes(data.as_slice());
        assert!(matches!(
            result,
            Err(CrsfParsingError::InvalidPayloadLength)
        ));
    }

    #[test]
    fn test_reassembler_idle_initial_state() {
        let reassembler = ParameterChunkReassembler::new();
        assert!(reassembler.is_idle());
        assert!(!reassembler.is_complete());
        assert_eq!(reassembler.chunks_received(), 0);
    }

    #[test]
    fn test_reassembler_single_chunk_float() {
        // A complete Float parameter that fits in one chunk
        let mut reassembler = ParameterChunkReassembler::new();

        // Build a chunk payload: parent(0) + data_type(0x08) + "Test\0" + float data + "mW\0"
        let mut payload_buf = [0u8; 64];
        let entry = ParameterSettingsEntry::new(0xEA, 0xEE, 2, 0, 0, 0x08, "Test")
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
        let entry_len = entry.to_bytes(&mut payload_buf).unwrap();

        // Feed the full entry as a single chunk
        let chunk = ParameterChunk::from_bytes(&payload_buf[..entry_len]).unwrap();
        let result = reassembler.push(chunk).unwrap();

        assert!(result.is_some());
        let assembled = result.unwrap();
        assert_eq!(assembled.parameter_number, 2);
        assert_eq!(assembled.name, "Test");
        assert!(!assembled.is_hidden());
        assert_eq!(assembled.get_data_type().unwrap(), ParameterDataType::Float);
        if let Some(ParameterData::Float { value, unit, .. }) = &assembled.data {
            assert_eq!(*value, 2000);
            assert_eq!(*unit, "mW");
        } else {
            panic!("Expected Float data");
        }
        assert!(reassembler.is_complete());
        assert!(!reassembler.is_idle());
        assert_eq!(reassembler.chunks_received(), 1);
        assert_eq!(reassembler.total_chunks(), 1);
        assert_eq!(reassembler.param_number(), 2);
    }

    #[test]
    fn test_reassembler_multi_chunk_text_selection() {
        // Create a TextSelection parameter large enough to need 2 chunks.
        // The entry payload must exceed MAX_CHUNK_PAYLOAD_SIZE (56).
        //
        // Entry layout: parent(1) + data_type(1) + name(N+1) + options(O+1) +
        //               value(1) + min(1) + max(1) + default(1) + unit(U+1)
        // We use a 50-char options string + null = 51 bytes, name="P" + null = 2 bytes,
        // unit="U" + null = 2 bytes.
        // Total = 1 + 1 + 2 + 51 + 4 + 2 = 61 bytes (overflows 56).

        // 50 'x' chars to make the entry payload overflow 56 bytes
        let options_body = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"; // 50 chars
        let options_str: String<MAX_OPTIONS> = String::try_from(options_body).unwrap();
        let unit_str: String<MAX_STRING_LEN> = String::try_from("U").unwrap();

        // Build full entry bytes
        let mut full_buf = [0u8; 128];
        let entry = ParameterSettingsEntry::new(0xEA, 0xEE, 3, 0, 0, 0x09, "P")
            .unwrap()
            .add_data(ParameterData::TextSelection {
                options: options_str.clone(),
                value: 1,
                min: 0,
                max: 50,
                default: 0,
                unit: unit_str.clone(),
            });
        let entry_len = entry.to_bytes(&mut full_buf).unwrap();

        // Verify it actually needs chunking
        let entry_payload_len = entry_len - 4; // minus dst/src/param/chunks header
        assert!(
            entry_payload_len > MAX_CHUNK_PAYLOAD_SIZE,
            "Test data must exceed chunk size ({} > {})",
            entry_payload_len,
            MAX_CHUNK_PAYLOAD_SIZE
        );

        // Split into two chunks
        // Chunk 0: 4-byte header (dst/src/param/chunks) + first MAX_CHUNK_PAYLOAD_SIZE entry bytes
        let chunk0_size = 4 + MAX_CHUNK_PAYLOAD_SIZE;
        let mut chunk0_data = [0u8; 64];
        chunk0_data[0..4].copy_from_slice(&full_buf[0..4]); // dst, src, param#, chunks_remaining
        chunk0_data[4..chunk0_size].copy_from_slice(&full_buf[4..chunk0_size]);
        chunk0_data[3] = 1; // chunks_remaining = 1 (1 more chunk after this)

        // Chunk 1: 4-byte header + remaining entry bytes
        let remaining = entry_len - chunk0_size;
        let chunk1_size = 4 + remaining;
        let mut chunk1_data = [0u8; 64];
        chunk1_data[0..4].copy_from_slice(&full_buf[0..4]); // dst, src, same param#
        chunk1_data[3] = 0; // chunks_remaining = 0 (last chunk)
        chunk1_data[4..chunk1_size].copy_from_slice(&full_buf[chunk0_size..entry_len]);

        // Reassemble
        let mut reassembler = ParameterChunkReassembler::new();
        let chunk0 = ParameterChunk::from_bytes(&chunk0_data[..chunk0_size]).unwrap();
        let result0 = reassembler.push(chunk0).unwrap();
        assert!(result0.is_none(), "Expected more chunks");
        assert!(!reassembler.is_complete());
        assert_eq!(reassembler.chunks_received(), 1);
        assert_eq!(reassembler.total_chunks(), 2);

        let chunk1 = ParameterChunk::from_bytes(&chunk1_data[..chunk1_size]).unwrap();
        let result1 = reassembler.push(chunk1).unwrap();
        assert!(result1.is_some(), "Expected completion");

        let assembled = result1.unwrap();
        assert_eq!(assembled.parameter_number, 3);
        assert_eq!(assembled.name, "P");
        assert_eq!(
            assembled.get_data_type().unwrap(),
            ParameterDataType::TextSelection
        );
        if let Some(ParameterData::TextSelection {
            options,
            value,
            min,
            max,
            default,
            unit,
        }) = &assembled.data
        {
            assert_eq!(*options, options_str);
            assert_eq!(*value, 1);
            assert_eq!(*min, 0);
            assert_eq!(*max, 50);
            assert_eq!(*default, 0);
            assert_eq!(*unit, unit_str);
        } else {
            panic!("Expected TextSelection data");
        }

        assert!(reassembler.is_complete());
        assert_eq!(reassembler.chunks_received(), 2);
    }

    #[test]
    fn test_reassembler_auto_reset_on_new_parameter() {
        // Start assembling param 5, then receive param 6 before completion
        let mut reassembler = ParameterChunkReassembler::new();

        // Chunk 0 of param 5 (incomplete)
        let chunk5 = ParameterChunk {
            dst_addr: 0xEA,
            src_addr: 0xEE,
            param_number: 5,
            chunks_remaining: 1,
            payload: {
                let mut p: Vec<u8, MAX_CHUNK_PAYLOAD_SIZE> = Vec::new();
                p.push(0x00).unwrap(); // parent
                p.push(0x0C).unwrap(); // data_type = Info
                p.extend_from_slice(b"P5\0").unwrap(); // name
                p
            },
        };
        let r5 = reassembler.push(chunk5).unwrap();
        assert!(r5.is_none());
        assert_eq!(reassembler.param_number(), 5);
        assert_eq!(reassembler.chunks_received(), 1);

        // Now a single-chunk param 6 should auto-reset and complete
        let info_str: String<MAX_STRING_LEN> = String::try_from("Hello").unwrap();
        let mut buf = [0u8; 64];
        let entry6 = ParameterSettingsEntry::new(0xEA, 0xEE, 6, 0, 0, 0x0C, "P6")
            .unwrap()
            .add_data(ParameterData::Info { info: info_str });
        let len6 = entry6.to_bytes(&mut buf).unwrap();
        let chunk6 = ParameterChunk::from_bytes(&buf[..len6]).unwrap();
        let r6 = reassembler.push(chunk6).unwrap();

        assert!(r6.is_some());
        let assembled = r6.unwrap();
        assert_eq!(assembled.parameter_number, 6);
        assert_eq!(assembled.name, "P6");
        if let Some(ParameterData::Info { info }) = &assembled.data {
            assert_eq!(*info, "Hello");
        } else {
            panic!("Expected Info data");
        }
    }

    #[test]
    fn test_reassembler_too_many_chunks() {
        // Push more than MAX_CHUNKS chunks without completion
        let mut reassembler = ParameterChunkReassembler::new();

        for i in 0..=MAX_CHUNKS {
            // u8 conversion is safe because MAX_CHUNKS <= 8
            let remaining = (MAX_CHUNKS - i) as u8;
            let chunk = ParameterChunk {
                dst_addr: 0xEA,
                src_addr: 0xEE,
                param_number: 1,
                chunks_remaining: remaining,
                payload: {
                    let mut p: Vec<u8, MAX_CHUNK_PAYLOAD_SIZE> = Vec::new();
                    p.push(0xAA).unwrap();
                    // Fill to max but not beyond
                    for _ in 1..MAX_CHUNK_PAYLOAD_SIZE {
                        let _ = p.push(0xAA);
                    }
                    p
                },
            };

            if i < MAX_CHUNKS {
                // Should accept
                let result = reassembler.push(chunk).unwrap();
                // Since chunks_remaining > 0 for first MAX_CHUNKS-1 pushes...
                if i < MAX_CHUNKS - 1 {
                    assert!(result.is_none());
                }
            } else {
                // The MAX_CHUNKS-th chunk should overflow
                let result = reassembler.push(chunk);
                assert!(result.is_err());
                // After error, reassembler should be reset to idle
                assert!(reassembler.is_idle());
            }
        }
    }

    #[test]
    fn test_reassembler_full_round_trip_via_entry() {
        // Verify reassembly produces the same result as a single-chunk from_bytes
        // by comparing a chunked-then-reassembled entry with a directly-parsed entry.

        let options_str: String<MAX_OPTIONS> =
            String::try_from("250;500;1kHz;2kHz;5kHz;10kHz;25kHz;50kHz;100kHz;200kHz").unwrap();
        let unit_str: String<MAX_STRING_LEN> = String::try_from("Hz").unwrap();

        // Build full entry
        let mut full_buf = [0u8; 128];
        let entry = ParameterSettingsEntry::new(0xEA, 0xEE, 7, 0, 0, 0x09, "Rate")
            .unwrap()
            .add_data(ParameterData::TextSelection {
                options: options_str.clone(),
                value: 2,
                min: 0,
                max: 9,
                default: 1,
                unit: unit_str.clone(),
            });
        let entry_len = entry.to_bytes(&mut full_buf).unwrap();

        // Direct parse (reference)
        let direct = ParameterSettingsEntry::from_bytes(&full_buf[..entry_len]).unwrap();

        // Now split into chunks and reassemble
        // Put the split point at 50 to ensure chunking
        let split = 4 + 50; // 4-byte header + 50 bytes of entry payload in chunk 0

        let mut chunk0_buf = [0u8; 64];
        chunk0_buf[0..4].copy_from_slice(&full_buf[0..4]);
        chunk0_buf[4..split].copy_from_slice(&full_buf[4..split]);
        chunk0_buf[3] = 1; // chunks_remaining = 1

        let remaining = entry_len - split;
        let mut chunk1_buf = [0u8; 64];
        chunk1_buf[0..4].copy_from_slice(&full_buf[0..4]);
        chunk1_buf[3] = 0; // last chunk
        chunk1_buf[4..4 + remaining].copy_from_slice(&full_buf[split..entry_len]);

        let mut reassembler = ParameterChunkReassembler::new();
        let c0 = ParameterChunk::from_bytes(&chunk0_buf[..split]).unwrap();
        assert!(reassembler.push(c0).unwrap().is_none());
        let c1 = ParameterChunk::from_bytes(&chunk1_buf[..4 + remaining]).unwrap();
        let result = reassembler.push(c1).unwrap();
        assert!(result.is_some());

        let chunked = result.unwrap();
        assert_eq!(
            direct, chunked,
            "Chunked reassembly must match direct parse"
        );
    }

    #[test]
    fn test_reassembler_reset_mid_assembly() {
        let mut reassembler = ParameterChunkReassembler::new();
        assert!(reassembler.is_idle());

        // Push one chunk of a multi-chunk parameter
        let chunk = ParameterChunk {
            dst_addr: 0xEA,
            src_addr: 0xEE,
            param_number: 10,
            chunks_remaining: 2,
            payload: {
                let mut p: Vec<u8, MAX_CHUNK_PAYLOAD_SIZE> = Vec::new();
                p.push(0x00).unwrap();
                p.push(0x08).unwrap();
                p.extend_from_slice(b"ABC\0").unwrap();
                p
            },
        };
        reassembler.push(chunk).unwrap();
        assert!(!reassembler.is_idle());
        assert_eq!(reassembler.chunks_received(), 1);
        assert_eq!(reassembler.param_number(), 10);

        // Reset
        reassembler.reset();
        assert!(reassembler.is_idle());
        assert_eq!(reassembler.chunks_received(), 0);
        assert!(!reassembler.is_complete());

        // After reset, can start a new parameter
        let mut buf = [0u8; 64];
        let entry = ParameterSettingsEntry::new(0xEA, 0xEE, 20, 0, 0, 0x0C, "New")
            .unwrap()
            .add_data(ParameterData::Info {
                info: String::try_from("Fresh start").unwrap(),
            });
        let len = entry.to_bytes(&mut buf).unwrap();
        let new_chunk = ParameterChunk::from_bytes(&buf[..len]).unwrap();
        let result = reassembler.push(new_chunk).unwrap();
        assert!(result.is_some());
        assert_eq!(result.unwrap().parameter_number, 20);
    }

    #[test]
    fn test_parameter_chunk_round_trip() {
        // Round-trip: data -> ParameterChunk -> to_bytes (if available)
        // ParameterChunk doesn't have to_bytes, but verify from_bytes -> fields consistency
        let raw: [u8; 10] = [
            0xEA, 0xEE, // dst, src
            0x0A, 0x03, // param_number=10, chunks_remaining=3
            0x01, 0x0B, 0x52, 0x4F, 0x4F, 0x54, // parent=1, data_type=Folder, "ROOT\0"
        ];
        let chunk = ParameterChunk::from_bytes(&raw).unwrap();
        assert_eq!(chunk.dst_addr, 0xEA);
        assert_eq!(chunk.src_addr, 0xEE);
        assert_eq!(chunk.param_number, 10);
        assert_eq!(chunk.chunks_remaining, 3);
        assert_eq!(
            chunk.payload.as_slice(),
            &[0x01, 0x0B, 0x52, 0x4F, 0x4F, 0x54]
        );
    }

    #[test]
    fn test_parameter_chunk_zero_length_payload() {
        // A chunk with just the 4-byte header and no payload is valid
        let data: [u8; 4] = [0xEA, 0xEE, 0x01, 0x00];
        let chunk = ParameterChunk::from_bytes(&data).unwrap();
        assert!(chunk.payload.is_empty());
        assert_eq!(chunk.param_number, 1);
        assert_eq!(chunk.chunks_remaining, 0);
    }

    #[test]
    fn test_parameter_chunk_reassembler_new_const() {
        // Verify that `new` works as a const fn
        const _REASSEMBLER: ParameterChunkReassembler = ParameterChunkReassembler::new();
        let r = ParameterChunkReassembler::new();
        assert!(r.is_idle());
    }

    #[test]
    fn test_reassembler_same_param_different_chunks() {
        // Two chunks for the same parameter arriving in sequence should work
        let mut reassembler = ParameterChunkReassembler::new();

        let chunk0 = ParameterChunk {
            dst_addr: 0xEA,
            src_addr: 0xEE,
            param_number: 5,
            chunks_remaining: 1,
            payload: {
                let mut p: Vec<u8, MAX_CHUNK_PAYLOAD_SIZE> = Vec::new();
                p.push(0x00).unwrap(); // parent
                p.push(0x0A).unwrap(); // data_type = String
                p.extend_from_slice(b"Name\0").unwrap(); // name
                p.extend_from_slice(b"Hello").unwrap(); // first part of value
                p
            },
        };

        let chunk1 = ParameterChunk {
            dst_addr: 0xEA,
            src_addr: 0xEE,
            param_number: 5,
            chunks_remaining: 0,
            payload: {
                let mut p: Vec<u8, MAX_CHUNK_PAYLOAD_SIZE> = Vec::new();
                p.extend_from_slice(b" World\0").unwrap(); // rest of value
                p.push(20).unwrap(); // max_length
                p
            },
        };

        assert!(reassembler.push(chunk0).unwrap().is_none());
        assert_eq!(reassembler.chunks_received(), 1);
        assert_eq!(reassembler.param_number(), 5);

        let result = reassembler.push(chunk1).unwrap();
        assert!(result.is_some());
        let entry = result.unwrap();
        assert_eq!(entry.parameter_number, 5);
        assert_eq!(entry.name, "Name");
        if let Some(ParameterData::String { value, max_length }) = &entry.data {
            assert_eq!(*value, "Hello World");
            assert_eq!(*max_length, 20);
        } else {
            panic!("Expected String data");
        }
        assert!(reassembler.is_complete());
    }

    #[test]
    fn test_long_text_selection_options_over_128_chars() {
        // Real ELRS "Packet Rate" options exceed 128 bytes — ensure they parse without error.
        // Total options string is ~214 chars, well over the old MAX_OPTIONS=128 limit.
        let long_options =
            "D50Hz(-112dBm);25Hz(-123dBm);50Hz(-120dBm);100Hz(-117dBm);\
             100Hz Full(-112dBm);200Hz(-111dBm);200Hz(-111dBm);\
             200Hz Full(-111dBm);250Hz(-111dBm);K1000 Full(-101dBm)";
        assert!(
            long_options.len() > 128,
            "options must be longer than old limit to be a useful regression test"
        );
        let options_str = String::<MAX_OPTIONS>::try_from(long_options).unwrap();
        let unit_str = String::<MAX_STRING_LEN>::try_from("").unwrap();

        let entry = ParameterSettingsEntry::new(0xEA, 0xEE, 2, 0, 0, 0x09, "Packet Rate")
            .unwrap()
            .add_data(ParameterData::TextSelection {
                options: options_str,
                value: 9,
                min: 0,
                max: 9,
                default: 0,
                unit: unit_str,
            });

        let mut buf = [0u8; 512];
        let len = entry.to_bytes(&mut buf).unwrap();
        let parsed = ParameterSettingsEntry::from_bytes(&buf[..len]).unwrap();
        if let Some(ParameterData::TextSelection { options, value, .. }) = &parsed.data {
            assert_eq!(options.as_str(), long_options);
            assert_eq!(*value, 9);
        } else {
            panic!("Expected TextSelection data");
        }
    }
}
