# Comprehensive Test Cases for CRSF Parameter Settings Protocol

This document provides a comprehensive test suite for the CRSF (Crossfire) protocol parameter settings implementation, covering both the Lua script (`elrsV3.lua`) and the firmware-side communication.

______________________________________________________________________

## Table of Contents

1. [Overview](#1-overview)
1. [Parameter Read Operations](#2-parameter-read-operations)
1. [Parameter Write Operations](#3-parameter-write-operations)
1. [Device Discovery and Enumeration](#4-device-discovery-and-enumeration)
1. [Field Type Handling](#5-field-type-handling)
1. [Chunked Data Transfer](#6-chunked-data-transfer)
1. [Error Handling and Edge Cases](#7-error-handling-and-edge-cases)
1. [Timeout and Retry Logic](#8-timeout-and-retry-logic)
1. [UI Interaction Scenarios](#9-ui-interaction-scenarios)
1. [Firmware-Side Protocol Tests](#10-firmware-side-protocol-tests)
1. [Integration Test Templates](#11-integration-test-templates)

______________________________________________________________________

## 1. Overview

### 1.1 Protocol Frame Types

The CRSF protocol uses the following frame types for parameter communication:

| Frame Type | Hex Value | Direction | Purpose |
| ------------------------ | --------- | -------------- | ------------------------------------------ |
| DEVICE_PING | 0x28 | Bidirectional | Device discovery broadcast |
| DEVICE_INFO | 0x29 | Device→Handset | Device identification and field count |
| PARAMETER_SETTINGS_ENTRY | 0x2B | Device→Handset | Parameter data in response to read request |
| PARAMETER_READ | 0x2C | Handset→Device | Request specific parameter data |
| PARAMETER_WRITE | 0x2D | Handset→Device | Write parameter value to device |
| ELRS_STATUS | 0x2E | Device→Handset | Link statistics and good/bad packet counts |

### 1.2 Address Constants

```json
{
  "addresses": {
    "CRSF_ADDRESS_BROADCAST": "0x00",
    "CRSF_ADDRESS_RADIO_TRANSMITTER": "0xEA",
    "CRSF_ADDRESS_CRSF_RECEIVER": "0xEC",
    "CRSF_ADDRESS_CRSF_TRANSMITTER": "0xEE",
    "CRSF_ADDRESS_ELRS_LUA": "0xEF"
  }
}
```

### 1.3 Parameter Value Types

```json
{
  "field_types": [
    { "type": 0, "name": "CRSF_UINT8", "size_bytes": 1, "signed": false },
    { "type": 1, "name": "CRSF_INT8", "size_bytes": 1, "signed": true },
    { "type": 2, "name": "CRSF_UINT16", "size_bytes": 2, "signed": false },
    { "type": 3, "name": "CRSF_INT16", "size_bytes": 2, "signed": true },
    { "type": 8, "name": "CRSF_FLOAT", "size_bytes": 4, "signed": true },
    {
      "type": 9,
      "name": "CRSF_TEXT_SELECTION",
      "size_bytes": 1,
      "options": true
    },
    {
      "type": 10,
      "name": "CRSF_STRING",
      "size_bytes": "variable",
      "null_terminated": true
    },
    { "type": 11, "name": "CRSF_FOLDER", "size_bytes": 0, "container": true },
    {
      "type": 12,
      "name": "CRSF_INFO",
      "size_bytes": "variable",
      "read_only": true
    },
    { "type": 13, "name": "CRSF_COMMAND", "size_bytes": 2, "action": true },
    {
      "type": 15,
      "name": "CRSF_VTX",
      "size_bytes": "variable",
      "video_transmitter": true
    }
  ]
}
```

______________________________________________________________________

## 2. Parameter Read Operations

### 2.1 Single Field Read Request

**Test Objective**: Verify that the Lua script correctly requests and parses a single parameter field from the device.

**Prerequisite Flow**:

1. Device is connected and enumerated
1. Device information has been received (frame 0x29)
1. Field count is known

**Test Case 2.1.1: Basic UINT8 Field Read**

```json
{
  "test_id": "TC-READ-UINT8-001",
  "name": "Basic UINT8 Field Read",
  "category": "Parameter Read",
  "description": "Verify single UINT8 field can be read and parsed correctly",
  "preconditions": [
    "Device ID: 0xEE (ELRS TX Module)",
    "Handset ID: 0xEF (ELRS LUA)",
    "Device has transmitted DEVICE_INFO frame",
    "Field count > 0"
  ],
  "steps": {
    "lua_script": {
      "action": "crossfireTelemetryPop() returns PARAMETER_READ request",
      "expected_behavior": [
        "Extract field_id from loadQ queue",
        "Extract fieldChunk (should be 0 for first chunk)",
        "Push PARAMETER_READ frame (0x2C)"
      ],
      "frame_sent": {
        "type": "0x2C",
        "payload": {
          "device_addr": "0xEE",
          "handset_addr": "0xEF",
          "field_id": 4,
          "field_chunk": 0
        }
      }
    },
    "device_response": {
      "action": "Device responds with PARAMETER_SETTINGS_ENTRY frame (0x2B)",
      "frame_received": {
        "type": "0x2B",
        "payload": {
          "device_addr": "0xEE",
          "handset_addr": "0xEF",
          "field_id": 4,
          "chunks_remain": 0,
          "parent": 0,
          "type": "0x00", // UINT8 with no hidden flag
          "name": "TX Power",
          "name_terminator": "0x00",
          "value": 2,
          "min": 0,
          "max": 10,
          "unit": "mW",
          "unit_terminator": "0x00"
        }
      }
    },
    "lua_parsing": {
      "action": "parseParameterInfoMessage() processes the response",
      "expected_results": {
        "field.id": 4,
        "field.parent": 0,
        "field.type": 0,
        "field.name": "TX Power",
        "field.value": 2,
        "field.min": 0,
        "field.max": 10,
        "field.unit": "mW"
      }
    }
  },
  "validation_criteria": [
    "Field value correctly parsed as integer 2",
    "Min/max values correctly interpreted",
    "Unit string extracted correctly",
    "loadQ entry removed after successful parse"
  ]
}
```

**Test Case 2.1.2: INT16 Field with Negative Values**

```json
{
  "test_id": "TC-READ-INT16-001",
  "name": "INT16 Field with Negative Values",
  "category": "Parameter Read",
  "description": "Verify INT16 fields with negative min/max values are parsed correctly",
  "preconditions": ["Device connected, field ID known for INT16 type"],
  "steps": {
    "device_response": {
      "frame_received": {
        "type": "0x2B",
        "payload": {
          "field_id": 18,
          "chunks_remain": 0,
          "parent": 0,
          "type": "0x03", // INT16
          "name": "Offset",
          "value": -15, // Stored as 2's complement
          "min": -50,
          "max": 50
        }
      }
    },
    "lua_parsing": {
      "expected_results": {
        "field.type": 3,
        "field.value": -15,
        "field.min": -50,
        "field.max": 50,
        "field.size": -2 // Negative indicates signed type
      }
    }
  },
  "validation_criteria": [
    "2's complement conversion applied correctly",
    "field.size is negative (-2) indicating signed type"
  ]
}
```

### 2.2 Chunked Field Read

**Test Objective**: Verify handling of fields that require multiple chunks due to size limits.

**Test Case 2.2.1: Multi-Chunk String Field**

```json
{
  "test_id": "TC-READ-CHUNK-001",
  "name": "Multi-Chunk String Field Read",
  "category": "Chunked Transfer",
  "description": "Verify fields spanning multiple 64-byte chunks are reassembled correctly",
  "preconditions": ["Device has string field exceeding 64-byte chunk limit"],
  "steps": {
    "lua_script": {
      "action": "Initial PARAMETER_READ request",
      "frame_sent": {
        "type": "0x2C",
        "field_id": 99,
        "field_chunk": 0
      }
    },
    "device_responses": [
      {
        "chunk_number": 0,
        "chunks_remain": 2,
        "frame": {
          "type": "0x2B",
          "field_id": 99,
          "chunks_remain": 2,
          "data_bytes": "Name part 1\x00...rest continues"
        }
      },
      {
        "chunk_number": 1,
        "chunks_remain": 1,
        "frame": {
          "type": "0x2B",
          "field_id": 99,
          "chunks_remain": 1,
          "data_bytes": "...continued data..."
        }
      },
      {
        "chunk_number": 2,
        "chunks_remain": 0,
        "frame": {
          "type": "0x2B",
          "field_id": 99,
          "chunks_remain": 0,
          "data_bytes": "...final chunk with terminator"
        }
      }
    ],
    "lua_parsing": {
      "action": "fieldData buffer accumulates chunks until chunks_remain = 0",
      "expected_results": {
        "fieldData.length": 180, // Combined length of all chunks
        "field.name": "Complete reassembled string",
        "field.value": "Full string value"
      }
    }
  },
  "validation_criteria": [
    "fieldData buffer persists across multiple parseParameterInfoMessage() calls",
    "expectChunksRemain tracks remaining chunks",
    "Final parse triggers field population only when chunks_remain = 0"
  ],
  "edge_cases": [
    "Chunk arrives out of order (should not happen but buffer could corrupt)",
    "Intermediate chunk missing (timeout should trigger retry)",
    "Duplicate chunk received (should be ignored or handled gracefully)"
  ]
}
```

______________________________________________________________________

## 3. Parameter Write Operations

### 3.1 Single Value Write

**Test Objective**: Verify that parameter values can be written to the device successfully.

**Test Case 3.1.1: Write UINT8 Selection Value**

```json
{
  "test_id": "TC-WRITE-UINT8-001",
  "name": "Write UINT8 Selection Value",
  "category": "Parameter Write",
  "description": "Verify user can change a UINT8 field value and device receives correct write command",
  "preconditions": [
    "Field loaded with current value",
    "User selects new value via UI"
  ],
  "steps": {
    "user_action": {
      "action": "User presses INC/DEC to change field value",
      "before": { "field.value": 0 },
      "after": { "field.value": 1 }
    },
    "ui_event": {
      "event": "EVT_VIRTUAL_ENTER",
      "action": "Toggles edit mode on field",
      "expected_behavior": [
        "edit flag set to true",
        "Field displayed with BLINK attribute"
      ]
    },
    "value_change": {
      "action": "User presses EVT_VIRTUAL_NEXT",
      "expected_behavior": [
        "incrField(1) called",
        "field.value incremented by step (typically 1)",
        "Value wrapped at max boundary"
      ]
    },
    "save_action": {
      "event": "EVT_VIRTUAL_ENTER (second press)",
      "expected_behavior": [
        "edit flag set to false",
        "functions[field.type+1].save(field) called",
        "reloadRelatedFields(field) called for dependent fields"
      ],
      "frame_sent": {
        "type": "0x2D",
        "payload": {
          "device_addr": "0xEE",
          "handset_addr": "0xEF",
          "field_id": 4,
          "value_bytes": [2] // Little-endian UINT8
        }
      }
    },
    "device_response": {
      "action": "Device may send updated field data or acknowledgment",
      "expected": "New value persisted, dependent fields queued for reload"
    }
  },
  "validation_criteria": [
    "Write frame correctly formatted with little-endian byte order",
    "fieldIntSave() uses correct size (1 byte for UINT8)",
    "Related fields marked for reload (nc = true)",
    "fieldTimeout set for EEPROM commit delay"
  ]
}
```

**Test Case 3.1.2: Write UINT16 Value**

```json
{
  "test_id": "TC-WRITE-UINT16-001",
  "name": "Write UINT16 Channel Value",
  "category": "Parameter Write",
  "description": "Verify 16-bit parameter values are written correctly with proper byte ordering",
  "preconditions": ["Field type UINT16, current value known"],
  "steps": {
    "save_action": {
      "action": "fieldIntSave() processes UINT16 field",
      "field_properties": {
        "type": 2,
        "size": 2,
        "value": 1500
      },
      "frame_sent": {
        "type": "0x2D",
        "payload": {
          "field_id": 10,
          "value_bytes": [0xdc, 0x05] // 1500 little-endian: 0x05DC
        }
      }
    }
  },
  "validation_criteria": [
    "Value 1500 → bytes [0xDC, 0x05] (little-endian)",
    "frame.length = 6 bytes (device_id + handset_id + field_id + 2 value bytes)",
    "bit32.rshift(value, 8*i) used for byte extraction"
  ],
  "additional_tests": [
    { "value": 0, "bytes": [0x00, 0x00] },
    { "value": 255, "bytes": [0xff, 0x00] },
    { "value": 65535, "bytes": [0xff, 0xff] }
  ]
}
```

### 3.2 Signed Value Write

**Test Objective**: Verify negative values are correctly converted to 2's complement for writing.

**Test Case 3.2.1: Write INT16 Negative Value**

```json
{
  "test_id": "TC-WRITE-INT16-NEG-001",
  "name": "Write INT16 Negative Value (2's Complement)",
  "category": "Parameter Write",
  "description": "Verify negative INT16 values are correctly converted for transmission",
  "preconditions": ["Field type INT16, size = -2"],
  "steps": {
    "value_conversion": {
      "input_value": -25,
      "field_size": 2,
      "expected_bytes": [0xe7, 0xff], // -25 in 16-bit 2's complement
      "calculation": "0x100 + (-25) = 231 = 0xE7, high byte = 0xFF"
    },
    "save_action": {
      "frame_sent": {
        "type": "0x2D",
        "payload": {
          "field_id": 18,
          "value_bytes": [0xe7, 0xff]
        }
      }
    }
  },
  "validation_criteria": [
    "Negative value converted to unsigned 2's complement representation",
    "Correct byte ordering maintained",
    "field.size remains negative indicating signed type"
  ],
  "test_values": [
    { "value": -1, "bytes": [0xff, 0xff] },
    { "value": -128, "bytes": [0x80, 0xff] },
    { "value": -32768, "bytes": [0x00, 0x80] },
    { "value": 0, "bytes": [0x00, 0x00] },
    { "value": 32767, "bytes": [0xff, 0x7f] }
  ]
}
```

### 3.3 Command Field Execution

**Test Objective**: Verify command-type fields (Bind, WiFi Update, etc.) are executed correctly.

**Test Case 3.3.1: Execute Bind Command**

```json
{
  "test_id": "TC-WRITE-COMMAND-001",
  "name": "Execute Bind Command",
  "category": "Parameter Write",
  "description": "Verify command-type fields trigger proper execution with status tracking",
  "preconditions": ["Field type COMMAND (13), Bind button pressed"],
  "steps": {
    "command_initiation": {
      "action": "fieldCommandSave() called",
      "expected_behavior": [
        "reloadCurField() - reloads command status",
        "field.status set to 1 (executing)",
        "fieldPopup set to this field",
        "fieldPopup.lastStatus set to 0",
        "fieldTimeout set based on field.timeout"
      ],
      "frame_sent": {
        "type": "0x2D",
        "payload": {
          "device_addr": "0xEE",
          "handset_addr": "0xEF",
          "field_id": 14,
          "status": 1 // lcsStart
        }
      }
    },
    "command_status_poll": {
      "action": "refreshNext() polls for command status",
      "frame_sent": {
        "type": "0x2D",
        "payload": {
          "device_addr": "0xEE",
          "handset_addr": "0xEF",
          "field_id": 14,
          "status": 6 // lcsQuery
        }
      }
    },
    "device_response": {
      "frame_received": {
        "type": "0x2B",
        "payload": {
          "field_id": 14,
          "status": 2, // lcsExecuting
          "timeout": 200, // poll interval in 10ms units
          "info": "Binding..."
        }
      }
    },
    "lua_handling": {
      "action": "fieldCommandLoad() processes status",
      "expected_behavior": [
        "fieldPopup.status = 2",
        "fieldPopup.info = 'Binding...'",
        "commandRunningIndicator cycles through / - \\ |",
        "Popup page displays with spinner animation"
      ]
    }
  },
  "command_status_values": {
    "0": "lcsIdle - Command not started",
    "1": "lcsStart - Initiate command",
    "2": "lcsExecuting - Command in progress",
    "3": "lcsConfirmed - User confirmed (if confirmation required)",
    "4": "lcsConfirmed - Confirmation sent",
    "5": "lcsCancel - User cancelled",
    "6": "lcsQuery - Query status"
  },
  "validation_criteria": [
    "Popup displays command status with animated indicator",
    "Polling continues until status changes from 2",
    "User can cancel with EVT_VIRTUAL_EXIT"
  ]
}
```

______________________________________________________________________

## 4. Device Discovery and Enumeration

### 4.1 Initial Device Detection

**Test Objective**: Verify the Lua script can detect and enumerate devices on the CRSF bus.

**Test Case 4.1.1: Device Discovery Broadcast**

```json
{
  "test_id": "TC-DISCOVERY-001",
  "name": "Device Discovery Broadcast",
  "category": "Device Discovery",
  "description": "Verify initial device discovery populates device list correctly",
  "preconditions": [
    "Script just started",
    "No devices in devices[] array",
    "devicesRefreshTimeout triggered"
  ],
  "steps": {
    "broadcast": {
      "action": "refreshNext() sends device ping",
      "frame_sent": {
        "type": "0x28",
        "payload": {
          "dest_addr": "0x00", // Broadcast
          "origin_addr": "0xEF" // ELRS LUA
        }
      }
    },
    "device_response": {
      "action": "ELRS TX Module responds with DEVICE_INFO",
      "frame_received": {
        "type": "0x29",
        "payload": {
          "device_addr": "0xEE",
          "origin_addr": "0xEC",
          "device_name": "ExpressLRS TX",
          "serial_no": "ELRS",
          "field_count": 18,
          "parameter_version": 1
        }
      }
    },
    "lua_handling": {
      "action": "parseDeviceInfoMessage() processes response",
      "expected_results": {
        "devices[1].id": "0xEE",
        "devices[1].name": "ExpressLRS TX",
        "devices[1].fldcnt": 18,
        "devices[1].isElrs": true,
        "deviceId": "0xEE",
        "fields_count": 18,
        "deviceIsELRS_TX": true
      }
    }
  },
  "validation_criteria": [
    "Serial number 'ELRS' (0x454C5253) detected as ELRS device",
    "deviceId changed to discovered device ID",
    "fields array allocated with correct count",
    "handsetId set to 0xEF for ELRS TX"
  ]
}
```

### 4.2 Multiple Device Handling

**Test Objective**: Verify the script can handle multiple CRSF devices on the bus.

**Test Case 4.2.1: Multiple Device Discovery**

```json
{
  "test_id": "TC-DISCOVERY-MULTI-001",
  "name": "Multiple Device Discovery",
  "category": "Device Discovery",
  "description": "Verify script correctly handles multiple responding devices",
  "preconditions": ["Multiple CRSF devices connected"],
  "steps": {
    "broadcast": {
      "action": "Ping broadcast sent",
      "frame_sent": { "type": "0x28", "dest": "0x00" }
    },
    "device_responses": [
      {
        "device_name": "ExpressLRS TX",
        "device_id": "0xEE",
        "field_count": 18,
        "isElrs": true
      },
      {
        "device_name": "Betaflight",
        "device_id": "0xC8",
        "field_count": 45,
        "isElrs": false
      },
      {
        "device_name": "GPS",
        "device_id": "0xC2",
        "field_count": 8,
        "isElrs": false
      }
    ],
    "lua_handling": {
      "expected_results": {
        "#devices": 3,
        "devices[1].id": "0xEE",
        "devices[2].id": "0xC8",
        "devices[3].id": "0xC2",
        "Other Devices folder created": true,
        "Device selection menu available": true
      }
    },
    "device_switching": {
      "action": "User selects 'Other Devices' folder",
      "expected_behavior": [
        "fieldFolderDeviceOpen() called",
        "createDeviceFields() populates device list",
        "Each device creates selectable menu item"
      ],
      "device_selection": {
        "action": "User selects Betaflight device",
        "expected_results": {
          "deviceId": "0xC8",
          "deviceName": "Betaflight",
          "fields_count": 45,
          "deviceIsELRS_TX": false,
          "handsetId": "0xEA" // RADIO_TRANSMITTER for non-ELRS
        }
      }
    }
  },
  "validation_criteria": [
    "Non-ELRS devices use handsetId 0xEA instead of 0xEF",
    "Device name correctly displayed",
    "Field count matches device response"
  ]
}
```

______________________________________________________________________

## 5. Field Type Handling

### 5.1 Text Selection Fields

**Test Objective**: Verify text selection fields with option lists are parsed and displayed correctly.

**Test Case 5.1.1: Text Selection with Options**

```json
{
  "test_id": "TC-FIELD-TEXTSELECT-001",
  "name": "Text Selection Field Parsing",
  "category": "Field Type Handling",
  "description": "Verify text selection fields with semicolon-delimited options parse correctly",
  "preconditions": ["Field type TEXT_SELECTION (9)"],
  "steps": {
    "device_response": {
      "frame_received": {
        "type": "0x2B",
        "payload": {
          "field_id": 0,
          "chunks_remain": 0,
          "parent": 0,
          "type": "0x09",
          "name": "Packet Rate",
          "options": "250Hz(-108dBm);500Hz(-105dBm);1kHz(-100dBm)",
          "value": 1,
          "unit": "Hz"
        }
      }
    },
    "parsing": {
      "action": "fieldTextSelLoad() processes options",
      "expected_results": {
        "field.values[0]": "250Hz(-108dBm)",
        "field.values[1]": "500Hz(-105dBm)",
        "field.values[2]": "1kHz(-100dBm)",
        "field.value": 1,
        "field.unit": "Hz",
        "field.grey": false
      }
    },
    "display": {
      "action": "fieldTextSelDisplay_color() renders selection",
      "expected_output": "500Hz(-105dBm) Hz",
      "expected_behavior": [
        "Selected option displayed at COL2",
        "Unit string displayed after option",
        "Value constrained to valid range (0-2)"
      ]
    },
    "value_change": {
      "action": "incrField(1) when value = 1",
      "expected_results": {
        "field.value": 2,
        "display": "1kHz(-100dBm) Hz"
      }
    }
  },
  "edge_cases": [
    {
      "description": "Single option available",
      "options": "Off",
      "expected": "field.grey = true"
    },
    {
      "description": "Empty option (blank in list)",
      "options": "On;;Off",
      "expected": "Skip blank options during incrField()"
    }
  ]
}
```

### 5.2 Float Field Handling

**Test Objective**: Verify floating-point fields with precision are parsed and displayed correctly.

**Test Case 5.2.1: Float Field with Precision**

```json
{
  "test_id": "TC-FIELD-FLOAT-001",
  "name": "Float Field Parsing with Precision",
  "category": "Field Type Handling",
  "description": "Verify FLOAT fields with precision and step values parse correctly",
  "preconditions": ["Field type FLOAT (8)"],
  "steps": {
    "device_response": {
      "frame_received": {
        "type": "0x2B",
        "payload": {
          "field_id": 18,
          "chunks_remain": 0,
          "parent": 0,
          "type": "0x08",
          "name": "Offset",
          "raw_value": -15000, // Stored as fixed-point
          "min": -50000,
          "max": 50000,
          "precision": 3, // 3 decimal places
          "step": 50,
          "unit": "dB"
        }
      }
    },
    "parsing": {
      "action": "fieldFloatLoad() processes float data",
      "expected_results": {
        "field.value": -15,
        "field.prec": 1000,
        "field.step": 50,
        "field.fmt": "%.3fdB",
        "field.min": -50,
        "field.max": 50
      }
    },
    "display": {
      "action": "fieldFloatDisplay() formats value",
      "expected_output": "-15.000dB",
      "calculation": "field.value / field.prec = -15000 / 1000 = -15.000"
    }
  },
  "precision_values": {
    "0": { "divisor": 1, "format": "%.0f" },
    "1": { "divisor": 10, "format": "%.1f" },
    "2": { "divisor": 100, "format": "%.2f" },
    "3": { "divisor": 1000, "format": "%.3f" }
  },
  "validation_criteria": [
    "Precision clamped to max 3 if > 3 received",
    "Step value used for incrField() increments",
    "Format string precomputed for display"
  ]
}
```

### 5.3 Folder/Info Fields

**Test Objective**: Verify folder and info field types are handled correctly.

**Test Case 5.3.1: Folder Field Navigation**

```json
{
  "test_id": "TC-FIELD-FOLDER-001",
  "name": "Folder Field Navigation",
  "category": "Field Type Handling",
  "description": "Verify folder fields allow navigation into sub-menus",
  "preconditions": ["Field type FOLDER (11)"],
  "steps": {
    "initial_state": {
      "field": {
        "id": 4,
        "name": "TX Power",
        "type": 11,
        "parent": 0
      },
      "display": "> TX Power"
    },
    "enter_folder": {
      "action": "User presses EVT_VIRTUAL_ENTER on folder",
      "expected_behavior": [
        "fieldFolderOpen(field) called",
        "currentFolderId = 4",
        "Back button added with name '----BACK----'",
        "lineIndex reset to 1",
        "pageOffset reset to 0"
      ]
    },
    "folder_contents": {
      "action": "getField() now filters by parent = 4",
      "expected_results": {
        "Fields displayed": [
          "Max Power (id=5)",
          "Dynamic (id=6)",
          "Fan Thresh (id=7)"
        ],
        "Back button at bottom": true
      }
    },
    "exit_folder": {
      "action": "User presses EVT_VIRTUAL_EXIT or selects Back",
      "expected_behavior": [
        "fieldBackExec() called",
        "currentFolderId = nil",
        "lineIndex restored from backFld.li",
        "pageOffset restored from backFld.po"
      ]
    }
  },
  "validation_criteria": [
    "Back button stores navigation state before folder entry",
    "Folder fields display with '>' prefix",
    "getField() correctly filters by parent ID"
  ]
}
```

______________________________________________________________________

## 6. Chunked Data Transfer

### 6.1 Chunk Protocol Details

```json
{
  "chunk_protocol": {
    "description": "Fields exceeding ~58 bytes are split into multiple frames",
    "frame_format": {
      "field_id": "1 byte - Index of field being transferred",
      "chunks_remain": "1 byte - Number of additional chunks after this one",
      "data": "up to 58 bytes - Portion of field data"
    },
    "example": {
      "field_size": 200,
      "chunk_size": 58,
      "chunks_needed": 4,
      "chunks_sent": [
        { "chunk": 0, "chunks_remain": 3, "data": "bytes 0-57" },
        { "chunk": 1, "chunks_remain": 2, "data": "bytes 58-115" },
        { "chunk": 2, "chunks_remain": 1, "data": "bytes 116-173" },
        { "chunk": 3, "chunks_remain": 0, "data": "bytes 174-199" }
      ]
    }
  }
}
```

### 6.2 Chunk Reassembly Test

**Test Case 6.2.1: Large String Field Chunking**

```json
{
  "test_id": "TC-CHUNK-REASSEMBLY-001",
  "name": "Large String Field Chunk Reassembly",
  "category": "Chunked Transfer",
  "description": "Verify large string fields are correctly reassembled from chunks",
  "preconditions": [
    "String field exceeds single frame capacity",
    "fieldData buffer is nil initially"
  ],
  "test_sequence": {
    "step_1": {
      "action": "PARAMETER_READ sent for field ID 99",
      "frame_sent": {"type": "0x2C", "field_id": 99, "chunk": 0}
    },
    "step_2": {
      "action": "First chunk received (chunks_remain = 2)",
      "frame_received": {
        "type": "0x2B",
        "field_id": 99,
        "chunks_remain": 2,
        "data": "First 58 bytes of string data..."
      },
      "state_after": {
        "fieldData": ["First 58 bytes"],
        "fieldChunk": 1,
        "expectChunksRemain": 2
      }
    },
    "step_3": {
      "action": "Second chunk received (chunks_remain = 1)",
      "frame_received": {
        "type": "0x2B",
        "field_id": 99,
        "chunks_remain": 1,
        "data": "Next 58 bytes of string data..."
      },
      "state_after": {
        "fieldData": ["First 58 bytes", "Next 58 bytes"],
        "fieldChunk": 2,
        "expectChunksRemain": 1
      }
    },
    "step_4": {
      "action": "Final chunk received (chunks_remain = 0)",
      "frame_received": {
        "type": "0x2B",
        "field_id": 99,
        "chunks_remain": 0,
        "data": "Final bytes\x00"
      },
      "state_after": {
        "fieldData": nil,
        "fieldChunk": 0,
        "field_processed": true
      }
    },
    "step_5": {
      "action": "Final field processing",
      "expected_results": {
        "field.name": "Complete reassembled string",
        "field.value": "Full string content with null terminator stripped",
        "loadQ[#loadQ]": nil
      }
    }
  },
  "error_injection_tests": [
    {
      "scenario": "Unexpected chunks_remain value",
      "description": "chunks_remain doesn't match expected",
      "expected_behavior": "Parse function returns early, field not updated"
    },
    {
      "scenario": "Field ID mismatch",
      "description": "Response has different field_id than request",
      "expected_behavior": "fieldData cleared, chunk reset"
    },
    {
      "scenario": "Timeout between chunks",
      "description": "fieldTimeout expires before all chunks arrive",
      "expected_behavior": "New PARAMETER_READ sent for chunk 0"
    }
  ]
}
```

______________________________________________________________________

## 7. Error Handling and Edge Cases

### 7.1 Malformed Frame Handling

**Test Case 7.1.1: Invalid Field ID Response**

```json
{
  "test_id": "TC-ERROR-INVALID-ID-001",
  "name": "Invalid Field ID Response Handling",
  "category": "Error Handling",
  "description": "Verify Lua script handles responses for non-existent or wrong field IDs",
  "preconditions": ["Waiting for field 5, device sends field 99"],
  "test_sequence": {
    "request": {
      "frame_sent": {
        "type": "0x2C",
        "field_id": 5
      }
    },
    "invalid_response": {
      "frame_received": {
        "type": "0x2B",
        "field_id": 99,
        "data": "Some data"
      }
    },
    "handling": {
      "check": "data[3] != fieldId in parseParameterInfoMessage()",
      "expected_behavior": [
        "fieldData = nil",
        "fieldChunk = 0",
        "Return early without processing"
      ]
    }
  }
}
```

### 7.2 Field Type Mismatch

**Test Case 7.2.1: Unexpected Field Type**

```json
{
  "test_id": "TC-ERROR-TYPE-MISMATCH-001",
  "name": "Unexpected Field Type Handling",
  "category": "Error Handling",
  "description": "Verify handling when field type in response differs from expected",
  "preconditions": ["Field expected as UINT8, device sends FLOAT"],
  "test_sequence": {
    "expected_type": 0,
    "received_type": 8,
    "handling": {
      "action": "functions[field.type+1].load() called",
      "expected_behavior": [
        "Type 9 (FLOAT + 1) indexes into functions[9]",
        "fieldFloatLoad() called",
        "Field incorrectly typed but processing continues"
      ]
    },
    "note": "Current implementation doesn't validate type consistency"
  }
}
```

### 7.3 Hidden Field Handling

**Test Case 7.3.1: Hidden Field Flag Processing**

```json
{
  "test_id": "TC-ERROR-HIDDEN-001",
  "name": "Hidden Field Flag Processing",
  "category": "Error Handling",
  "description": "Verify hidden fields (0x80 flag) are correctly marked and filtered",
  "preconditions": ["Field has hidden flag set"],
  "test_sequence": {
    "device_response": {
      "frame_received": {
        "type": "0x2B",
        "type_byte": "0x80", // UINT8 (0) + HIDDEN (0x80)
        "field_id": 99
      }
    },
    "parsing": {
      "action": "bit32.btest(fieldData[offset+1], 0x80)",
      "expected_results": {
        "field.type": 0, // Masked type
        "field.hidden": true
      }
    },
    "display": {
      "action": "getField() filters hidden fields",
      "expected_behavior": [
        "field.hidden is truthy (true or non-nil)",
        "field not returned by getField()",
        "Field not displayed in UI"
      ]
    }
  }
}
```

______________________________________________________________________

## 8. Timeout and Retry Logic

### 8.1 Field Load Timeout

**Test Case 8.1.1: Field Load Timeout and Retry**

```json
{
  "test_id": "TC-TIMEOUT-FIELD-001",
  "name": "Field Load Timeout and Retry",
  "category": "Timeout Logic",
  "description": "Verify timeout triggers retry when device doesn't respond to parameter read",
  "preconditions": ["loadQ has pending field ID", "fieldTimeout expired"],
  "test_sequence": {
    "initial_request": {
      "action": "PARAMETER_READ sent for field 5",
      "timestamp": "T0",
      "fieldTimeout": "T0 + 500ms"
    },
    "no_response": {
      "action": "Time advances past fieldTimeout",
      "expected_behavior": [
        "refreshNext() detects time > fieldTimeout",
        "crossfireTelemetryPush(0x2C) called again",
        "fieldTimeout reset"
      ]
    },
    "retry_count": {
      "description": "Script continues retrying until response received",
      "max_retries": "Unlimited (until user exits)"
    },
    "device_response": {
      "action": "Device responds to retry request",
      "expected_behavior": [
        "parseParameterInfoMessage() processes response",
        "loadQ[#loadQ] = nil",
        "fieldTimeout extended (no immediate retry)"
      ]
    }
  },
  "timeout_values": {
    "deviceIsELRS_TX": {
      "timeout_ms": 50,
      "reason": "Local device, fast response expected"
    },
    "other_device": {
      "timeout_ms": 500,
      "reason": "Remote device, slower response"
    }
  }
}
```

### 8.2 Device Refresh Timeout

**Test Case 8.2.1: Device List Refresh**

```json
{
  "test_id": "TC-TIMEOUT-DEVICE-001",
  "name": "Device List Periodic Refresh",
  "category": "Timeout Logic",
  "description": "Verify device list is periodically refreshed when empty",
  "preconditions": ["#devices == 0", "devicesRefreshTimeout expired"],
  "test_sequence": {
    "refresh": {
      "action": "refreshNext() detects time > devicesRefreshTimeout",
      "expected_behavior": [
        "forceRedraw = true",
        "devicesRefreshTimeout = time + 100 (1 second)",
        "crossfireTelemetryPush(0x28, {0x00, 0xEA})"
      ]
    }
  }
}
```

### 8.3 Link Statistics Timeout

**Test Case 8.3.1: Link Stats Request Timeout**

```json
{
  "test_id": "TC-TIMEOUT-LINK-001",
  "name": "Link Statistics Request",
  "category": "Timeout Logic",
  "description": "Verify periodic link statistics requests from ELRS TX module",
  "preconditions": ["deviceIsELRS_TX == true"],
  "test_sequence": {
    "periodic_request": {
      "action": "Time > linkstatTimeout",
      "interval_ms": 1000,
      "frame_sent": {
        "type": "0x2D",
        "payload": {
          "device_addr": "0xEE",
          "handset_addr": "0xEF",
          "field_id": 0x00,
          "value": 0x00
        }
      },
      "note": "field_id=0, value=0 indicates link stats request"
    },
    "response": {
      "action": "Device responds with 0x2E (ELRS_STATUS)",
      "expected_behavior": [
        "parseElrsInfoMessage() processes",
        "goodBadPkt string updated",
        "elrsFlags parsed for warning state"
      ]
    }
  }
}
```

______________________________________________________________________

## 9. UI Interaction Scenarios

### 9.1 Field Navigation

**Test Case 9.1.1: Field Selection Navigation**

```json
{
  "test_id": "TC-UI-NAV-001",
  "name": "Field Selection Navigation",
  "category": "UI Interaction",
  "description": "Verify field navigation with NEXT/PREV events",
  "preconditions": ["Multiple fields loaded", "lineIndex = 1, pageOffset = 0"],
  "test_sequence": {
    "next_field": {
      "action": "EVT_VIRTUAL_NEXT",
      "expected_behavior": [
        "selectField(1) called",
        "lineIndex increments",
        "pageOffset adjusted if needed"
      ]
    },
    "wrapping": {
      "description": "Navigation wraps at list boundaries",
      "at_end": "NEXT from last field wraps to first",
      "at_start": "PREV from first field wraps to last"
    },
    "folder_filtering": {
      "action": "getField() filters by currentFolderId",
      "expected_behavior": [
        "Only fields matching parent displayed",
        "Back button always visible at end"
      ]
    }
  },
  "page_scrolling": {
    "description": "Page offset adjusts when selection moves off screen",
    "scroll_down": {
      "condition": "lineIndex > maxLineIndex + pageOffset",
      "action": "pageOffset = lineIndex - maxLineIndex"
    },
    "scroll_up": {
      "condition": "lineIndex <= pageOffset",
      "action": "pageOffset = lineIndex - 1"
    }
  }
}
```

### 9.2 Value Editing

**Test Case 9.2.1: Value Editing with Constraints**

```json
{
  "test_id": "TC-UI-EDIT-001",
  "name": "Value Editing with Constraints",
  "category": "UI Interaction",
  "description": "Verify value editing respects min/max boundaries and blank options",
  "preconditions": ["Field in edit mode (edit = true)"],
  "test_sequence": {
    "increment": {
      "action": "EVT_VIRTUAL_NEXT",
      "expected_behavior": [
        "incrField(1) called",
        "field.value = field.value + field.step",
        "Value capped at field.max"
      ]
    },
    "decrement": {
      "action": "EVT_VIRTUAL_PREV",
      "expected_behavior": [
        "incrField(-1) called",
        "field.value = field.value - field.step",
        "Value floored at field.min"
      ]
    },
    "blank_option_handling": {
      "description": "Skips blank options in selection lists",
      "field.values": ["Option1", "", "Option3"],
      "action": "incrField() from value=0",
      "expected": "Skips to value=2 (Option3), not value=1 (blank)"
    },
    "save": {
      "action": "EVT_VIRTUAL_ENTER",
      "expected_behavior": [
        "edit = nil",
        "functions[field.type+1].save(field) called",
        "reloadRelatedFields(field) called"
      ]
    },
    "cancel": {
      "action": "EVT_VIRTUAL_EXIT while editing",
      "expected_behavior": [
        "edit = nil",
        "reloadCurField() called",
        "Original value restored"
      ]
    }
  }
}
```

______________________________________________________________________

## 10. Firmware-Side Protocol Tests

### 10.1 Parameter Serialization

**Test Case 10.1.1: Field Structure to Byte Array**

```json
{
  "test_id": "TC-FW-SERIALIZE-001",
  "name": "Field Structure Serialization",
  "category": "Firmware Protocol",
  "description": "Verify firmware correctly serializes field structures to CRSF frames",
  "preconditions": ["LUA parameter request received for field ID"],
  "test_sequence": {
    "request_received": {
      "frame": {
        "type": "0x2C",
        "payload": {
          "device_addr": "0xEE",
          "handset_addr": "0xEF",
          "field_id": 4,
          "chunk_number": 0
        }
      }
    },
    "serialization": {
      "action": "sendCRSFparam() converts field to byte array",
      "expected_output": {
        "chunkBuffer[2]": "parent ID (0x00)",
        "chunkBuffer[3]": "type (0x09 for TEXT_SELECTION) + hidden flags",
        "chunkBuffer[4:]": "field name with null terminator",
        "after_name": "options;value;min;max;default;units"
      }
    },
    "frame_queued": {
      "action": "CRSFHandset::packetQueueExtended(0x2B, ...)",
      "expected": "PARAMETER_SETTINGS_ENTRY frame sent to handset"
    }
  }
}
```

### 10.2 Parameter Write Processing

**Test Case 10.2.1: Parameter Write Handling**

```json
{
  "test_id": "TC-FW-WRITE-001",
  "name": "Parameter Write Processing",
  "category": "Firmware Protocol",
  "description": "Verify firmware correctly processes parameter write commands",
  "preconditions": ["PARAMETER_WRITE frame received from handset"],
  "test_sequence": {
    "write_request": {
      "frame": {
        "type": "0x2D",
        "payload": {
          "device_addr": "0xEE",
          "handset_addr": "0xEF",
          "field_id": 4,
          "value_bytes": [2] // UINT8 value
        }
      }
    },
    "processing": {
      "action": "HandleParameterWrite() or similar function",
      "expected_behavior": [
        "Extract field_id and value bytes",
        "Convert little-endian to native integer",
        "Update field value in parameter structure",
        "Persist to EEPROM if applicable"
      ]
    },
    "response": {
      "description": "Firmware may send updated field data or no response",
      "option_1": "Send PARAMETER_SETTINGS_ENTRY with new value",
      "option_2": "No response (handset will re-read if needed)"
    }
  }
}
```

### 10.3 Command Status Reporting

**Test Case 10.3.1: Command Execution and Status**

```json
{
  "test_id": "TC-FW-COMMAND-001",
  "name": "Command Execution Status Reporting",
  "category": "Firmware Protocol",
  "description": "Verify firmware correctly reports command execution status",
  "preconditions": ["Command field write received with status=1 (start)"],
  "test_sequence": {
    "command_received": {
      "frame": {
        "type": "0x2D",
        "payload": {
          "field_id": 14, // Bind command
          "status": 1 // lcsStart
        }
      }
    },
    "execution": {
      "action": "Initiate command (e.g., enter bind mode)",
      "expected_behavior": [
        "Set field.status = 2 (executing)",
        "Set field.timeout = poll interval",
        "Begin sending status updates"
      ]
    },
    "status_query": {
      "frame_received": {
        "type": "0x2D",
        "payload": {
          "field_id": 14,
          "status": 6 // lcsQuery
        }
      },
      "response": {
        "type": "0x2B",
        "payload": {
          "field_id": 14,
          "status": 2,
          "timeout": 200,
          "info": "Binding..."
        }
      }
    },
    "completion": {
      "action": "Command finishes",
      "response": {
        "type": "0x2B",
        "payload": {
          "field_id": 14,
          "status": 0, // lcsIdle - complete
          "info": "Bind successful"
        }
      }
    }
  }
}
```

______________________________________________________________________

## 11. Integration Test Templates

### 11.1 Complete Parameter Read Flow

```json
{
  "template": "INTEGRATION-READ-FLOW",
  "description": "Template for complete end-to-end parameter read test",
  "parameters": [
    {
      "name": "field_type",
      "type": "enum",
      "values": [
        "UINT8",
        "INT8",
        "UINT16",
        "INT16",
        "FLOAT",
        "TEXT_SELECTION",
        "STRING",
        "COMMAND"
      ]
    },
    {
      "name": "field_value",
      "type": "mixed",
      "description": "Appropriate value for field type"
    },
    {
      "name": "chunk_count",
      "type": "int",
      "description": "1 for single chunk, >1 for multi-chunk"
    }
  ],
  "generated_test_case": {
    "name": "Read {field_type} Field with Value {field_value}",
    "steps": [
      "1. Lua sends PARAMETER_READ request for field ID",
      "2. Firmware serializes field data ({chunk_count} chunk(s))",
      "3. Firmware sends PARAMETER_SETTINGS_ENTRY response(s)",
      "4. Lua parses and validates field data",
      "5. Lua displays field with correct value"
    ],
    "assertions": [
      "field.value == field_value",
      "field.type matches expected type",
      "field.name correctly extracted",
      "field.unit correctly extracted"
    ]
  }
}
```

### 11.2 Complete Parameter Write Flow

```json
{
  "template": "INTEGRATION-WRITE-FLOW",
  "description": "Template for complete end-to-end parameter write test",
  "parameters": [
    {
      "name": "field_type",
      "type": "enum",
      "values": ["UINT8", "INT8", "UINT16", "INT16", "TEXT_SELECTION"]
    },
    {
      "name": "old_value",
      "type": "mixed",
      "description": "Current field value"
    },
    {
      "name": "new_value",
      "type": "mixed",
      "description": "Value to write"
    }
  ],
  "generated_test_case": {
    "name": "Write {field_type} from {old_value} to {new_value}",
    "steps": [
      "1. Verify field shows old_value",
      "2. User enters edit mode",
      "3. User changes value to new_value",
      "4. User saves changes",
      "5. Lua sends PARAMETER_WRITE frame",
      "6. Firmware processes write and updates value",
      "7. Firmware optionally sends updated field data",
      "8. Lua reloads field and displays new_value"
    ],
    "assertions": [
      "Write frame correctly formatted",
      "New value persisted by firmware",
      "UI reflects new value after reload"
    ]
  }
}
```

### 11.3 Error Recovery Flow

```json
{
  "template": "INTEGRATION-ERROR-RECOVERY",
  "description": "Template for error detection and recovery tests",
  "parameters": [
    {
      "name": "error_type",
      "type": "enum",
      "values": [
        "timeout",
        "invalid_response",
        "type_mismatch",
        "device_disconnect"
      ]
    }
  ],
  "generated_test_case": {
    "name": "Recovery from {error_type}",
    "steps": [
      "1. Normal operation established",
      "2. Inject {error_type} error",
      "3. Verify error detection",
      "4. Verify retry/recovery mechanism",
      "5. Verify system returns to normal operation"
    ],
    "assertions": [
      "Error detected within expected timeframe",
      "Recovery initiated automatically",
      "System returns to operational state"
    ]
  }
}
```

______________________________________________________________________

## Test Execution Guidelines

### Running Tests

```bash
# C++ firmware tests
cd src/test
platformio test -e native

# Lua script testing (manual in simulator)
# 1. Copy elrsV3.lua to OpenTX Companion SCRIPTS/TOOLS/
# 2. Copy mockup/elrsmock.lua to SCRIPTS/TOOLS/mockup/
# 3. Run in simulator with -simu flag

# Protocol testing (requires hardware)
# 1. Connect ELRS TX module
# 2. Use ELRS Lua script on radio
# 3. Monitor CRSF frames with logic analyzer
```

### Expected Coverage

| Category | Coverage Target |
| ----------------- | ------------------------------ |
| Parameter Read | 100% of field types |
| Parameter Write | 100% of writable field types |
| Chunked Transfer | Multi-chunk scenarios |
| Error Handling | Invalid frames, timeouts |
| UI Interactions | Navigation, editing, folders |
| Firmware Protocol | Serialization, deserialization |

______________________________________________________________________

## Appendix: Complete Frame Examples

### A.1 Device Info Frame (0x29)

```json
{
  "example": {
    "description": "Device identification response",
    "bytes": [
      "0xEE", // device_addr
      "0x20", // frame_size = 32
      "0x29", // frame_type = DEVICE_INFO
      "0xEE", // dest_addr
      "0xEC", // origin_addr
      "0x45", // 'E'
      "0x78", // 'x'
      "0x70", // 'p'
      "0x72", // 'r'
      "0x65", // 'e'
      "0x73", // 's'
      "0x4C", // 'L'
      "0x52", // 'R'
      "0x53", // 'S'
      "0x20", // ' '
      "0x54", // 'T'
      "0x58", // 'X'
      "0x00", // name terminator
      "0x53", // serial_no[0] 'S'
      "0x45", // serial_no[1] 'E'
      "0x4C", // serial_no[2] 'L'
      "0x52", // serial_no[3] 'R'
      "0x00",
      "0x00",
      "0x00",
      "0x00", // hardware_ver
      "0x00",
      "0x30",
      "0x00",
      "0x00", // software_ver = 3.0.0
      "0x12", // field_count = 18
      "0x01", // parameter_version
      "0xXX" // CRC
    ]
  }
}
```

### A.2 Parameter Settings Entry (0x2B) - Text Selection

```json
{
  "example": {
    "description": "Text selection field response",
    "bytes": [
      "0xEE", // device_addr
      "0xXX", // frame_size (variable)
      "0x2B", // frame_type = PARAMETER_SETTINGS_ENTRY
      "0xEE", // dest_addr
      "0xEC", // origin_addr
      "0x00", // field_id
      "0x00", // chunks_remain = 0 (single chunk)
      "0x00", // parent_id
      "0x09", // type = TEXT_SELECTION
      "0x50", // 'P'
      "0x61", // 'a'
      "0x63", // 'c'
      "0x6B", // 'k'
      "0x65", // 'e'
      "0x74", // 't'
      "0x20", // ' '
      "0x52", // 'R'
      "0x61", // 'a'
      "0x74", // 't'
      "0x65", // 'e'
      "0x00", // name terminator
      "0x32", // '2'
      "0x35", // '5'
      "0x30", // '0'
      "0x28", // '('
      "0x2D", // '-'
      "0x31", // '1'
      "0x30", // '0'
      "0x38", // '8'
      "0x64", // 'd'
      "0x42", // 'B'
      "0x6D", // 'm'
      "0x29", // ')'
      "0x3B", // ';' option separator
      "0x35", // '5'
      "0x30", // '0'
      "0x30", // '0'
      "0x28", // '('
      "0x2D", // '-'
      "0x31", // '1'
      "0x30", // '1'
      "0x30", // '0'
      "0x64", // 'd'
      "0x42", // 'B'
      "0x6D", // 'm'
      "0x29", // ')'
      "0x00", // options terminator
      "0x01", // value = 1 (second option)
      "0x00", // min = 0
      "0x01", // max = 1
      "0x00", // default
      "0x48", // 'H'
      "0x7A", // 'z'
      "0x00", // unit terminator
      "0xXX" // CRC
    ]
  }
}
```

### A.3 Parameter Write (0x2D)

```json
{
  "example": {
    "description": "Write UINT8 value",
    "bytes": [
      "0xEE", // device_addr
      "0x06", // frame_size = 6
      "0x2D", // frame_type = PARAMETER_WRITE
      "0xEE", // dest_addr
      "0xEF", // origin_addr
      "0x04", // field_id
      "0x02", // value = 2
      "0xXX" // CRC
    ]
  }
}
```
