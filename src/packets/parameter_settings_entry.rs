use crate::packets::{CrsfPacket, PacketType};
use crate::CrsfParsingError;
use heapless::String;
use heapless::Vec;

const MAX_STRING_LEN: usize = 128;
const MAX_CHILDREN: usize = 32;
const MAX_OPTIONS: usize = 128;

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
                    let (options, options_end) = Self::parse_string(&data[offset..])?;
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

        let entry =
            ParameterSettingsEntry::new(0xEA, 0xEE, 2, 0, 0, 0x08, "Power", Some(data)).unwrap();

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
        let entry =
            ParameterSettingsEntry::new(0xEA, 0xEE, 5, 0, 0, 0x8C, "Secret", Some(data)).unwrap(); // 0x0C | 0x80

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

        let entry =
            ParameterSettingsEntry::new(0xEA, 0xEE, 1, 0, 0, 0x09, "Rate", Some(data)).unwrap();

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

        let entry =
            ParameterSettingsEntry::new(0xEA, 0xEE, 0, 0, 0, 0x0B, "ROOT", Some(data)).unwrap();

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

        let entry =
            ParameterSettingsEntry::new(0xEA, 0xEE, 10, 0, 0, 0x0D, "Bind", Some(data)).unwrap();

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

        let entry =
            ParameterSettingsEntry::new(0xEA, 0xEE, 4, 0, 0, 0x0A, "Name", Some(data)).unwrap();

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
        let entry = ParameterSettingsEntry::new(0xEA, 0xEE, 4, 0, 0, 0x08, "Test", None).unwrap();

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
        let result = ParameterSettingsEntry::new(0xEA, 0xEE, 0, 0, 0, 0x08, long_name, None);
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

        let entry =
            ParameterSettingsEntry::new(0xEA, 0xEE, 12, 0, 0, 0x0F, "VTX Param", Some(data))
                .unwrap();

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

        let entry =
            ParameterSettingsEntry::new(0xEA, 0xEE, 12, 0, 0, 0x8F, "Hidden VTX", Some(data))
                .unwrap(); // 0x0F | 0x80

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
}
