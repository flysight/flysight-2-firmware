# FlySight 2 BLE Interface Developer Documentation

## Overview

This document describes the Bluetooth Low Energy (BLE) interface for the FlySight 2 device. This interface allows applications to interact with the FlySight 2 for file access (reading, writing, deleting files and directories), accessing real-time sensor data, and managing device state.

The interface utilizes standard BLE GATT services where applicable, but primarily relies on custom services for its core functionality. Familiarity with BLE concepts (GATT, Services, Characteristics, UUIDs, Notifications, Writes, Bonding, Security Modes) is recommended.

**Note:** This interface is under active development. Details may change in future firmware versions. This documentation reflects the state based on developer discussions and firmware code (`v2024.11.11.release_candidate` or similar, check `flysight.txt` for exact version).

## Prerequisites

*   **Firmware:** A FlySight 2 unit running firmware version `v2024.11.11.release_candidate` or later is recommended to ensure compatibility with the features described here, especially the pairing mechanism and connection timeout. Firmware updates can be performed via the [online tool](https://flysight.ca/firmware/?include_beta=true). Note that different hardware batches (B1-B5, identifiable via `flysight.txt` or serial number) require specific firmware builds.
*   **BLE Client:** A BLE central device (e.g., smartphone, computer) with a compatible BLE stack and application, supporting secure connections (pairing/bonding) and configurable MTU.

## Core Concepts

### Pairing, Bonding, and Security

FlySight 2 employs BLE security features (bonding and whitelisting) to prevent unauthorized access.

1.  **Pairing Request Mode:**
    *   To initiate pairing with a *new* central device, the FlySight 2 must be put into "Pairing Request Mode".
    *   **Activation:** While the unit is off (Idle Mode, LED is off), perform **two short presses** of the power button.
    *   **Indication:** The **green LED** will pulse slowly (2-second period).
    *   **Advertising:** In this mode, the FlySight advertises as "limited discoverable" and includes specific manufacturer data indicating pairing mode is active (`0xDB0901`, see Advertising section).
    *   **Duration:** Pairing request mode lasts for 30 seconds, or until a BLE connection is established, or until the button is pressed again (which cancels it and returns to Idle).
    *   **Functionality:** While in this mode, *any* central device can connect and initiate pairing/bonding. The whitelist filter is temporarily disabled.

2.  **Whitelisting (Filter Accept List):**
    *   FlySight 2 uses the BLE "filter accept list" (based on the bonded device list). Outside of Pairing Request Mode, it only accepts connections from devices on this list (i.e., devices it has previously bonded with).
    *   **Adding Devices:** When a central device successfully completes the bonding process initiated during Pairing Request Mode, its address/identity is added to the FlySight's internal bonded device list, effectively whitelisting it for future connections.
    *   **Storage:** The bonded device list (including security keys like IRK/ERK) is stored persistently in the FlySight 2's non-volatile memory.

3.  **Bonding Process:**
    *   **Initiation:** After connecting to a FlySight 2 in Pairing Request Mode, the central device *must* initiate bonding to establish a secure, encrypted link and be added to the whitelist.
    *   **Security:** FlySight 2 requests **Security Mode 1, Level 2** (Authenticated pairing with encryption, No MITM protection). It uses the **Just Works** association model (as it has NoInput/NoOutput capabilities).
    *   **Triggering Pairing:** Since the peripheral (FlySight) cannot force pairing, the central device must trigger it. A reliable method is to attempt to **read a secure characteristic** (e.g., `FT_Packet_In` UUID `00000002-8e22-4541-9d4c-21edae82ed19`) immediately after connecting. FlySight 2 requires encryption for accessing user data characteristics. This read attempt will fail with an "insufficient authentication/encryption" error from the FlySight's BLE stack, which typically prompts the central device's OS/BLE stack to initiate the pairing/bonding procedure (often showing a system dialog to the user like "Pair with FlySight?").
    *   **Completion:** Once pairing is complete, the central device should store the bonding information (keys) provided by the BLE stack for future use. The FlySight stores the central's identity.

4.  **Reconnecting:**
    *   Once a central device is bonded and whitelisted, it can reconnect to the FlySight 2 even when the FlySight is *not* in Pairing Request Mode (e.g., when the LED is off or solid green), provided the FlySight is advertising (typically in slow advertising mode).
    *   The connection attempt will use the stored security keys to establish an encrypted link automatically. No user interaction (like button presses on the FlySight) is needed for reconnection.

5.  **Resetting BLE Security:**
    *   **Purpose:** To clear existing bonds if connection issues arise or if the user wants to remove access for previously paired devices.
    *   **FlySight Side:**
        *   Connect the FlySight 2 via USB.
        *   Edit the `FLYSIGHT.TXT` file and set `Reset_BLE = 1`. Save the file.
        *   Eject and unplug the FlySight. The BLE bond list and keys (IRK/ERK) are cleared when the device restarts after unplugging. The `Reset_BLE` value will be automatically set back to `0` in `FLYSIGHT.TXT`.
        *   Alternatively, deleting `flysight.txt` entirely will cause it to be recreated with defaults, including `Reset_BLE = 1`, triggering a reset on the next boot.
    *   **Central Device Side:** Crucially, you must also **"forget" or "unpair"** the FlySight device in the central device's system Bluetooth settings. This clears the stored keys on the central side. Simply clearing app cache or restarting Bluetooth may not be sufficient.
    *   **Hardware Reset (10s Hold):** This forces a processor reset but does *not* clear the BLE bonding information on its own. It's useful for recovering from a stuck state but should be used *in conjunction* with the `Reset_BLE` flag method for a full security reset.

6.  **Connection Timeout & Ping:**
    *   **Timeout:** FlySight 2 implements a **30-second** inactivity timeout (`TIMEOUT_MSEC`). If no BLE *write* activity occurs on the `FT_Packet_In` characteristic for this duration, the FlySight will terminate the connection (disconnect reason `0x13`, Remote User Terminated Connection).
    *   **Keep-Alive:** To maintain a connection during periods of inactivity (e.g., user browsing files in the app, waiting between commands), the central application **must** send a command to the `FT_Packet_In` characteristic at least once every 30 seconds.
    *   **Ping Command:** A dedicated "ping" command (`0xfe`) can be used for this. Writing `0xfe` to `FT_Packet_In` resets the timer and elicits an ACK (`0xf1 fe`) response on `FT_Packet_Out`. A recommended interval is every 15 seconds.
    *   **Implicit Reset:** *Any* successful write to `FT_Packet_In` (including file transfer commands) also resets the 30-second timeout timer.

### Device States and BLE Availability

The FlySight 2's operational mode affects BLE service availability, primarily due to resource contention (SD card access). BLE connection itself is generally possible in most states if the device is whitelisted and advertising. The current device mode can be read (and indicated) via the `DS_Mode` characteristic in the `Device_State` service.

*   **Idle Mode (LED Off):** Default low-power state. BLE is active (slow advertising). Full BLE functionality (File Transfer, Real-time Data subscription, State Control) is available. This is the **required** mode for file system operations via BLE.
*   **Active Mode (Green LED):** GNSS active, logging, audio feedback. Entered via a ~1s power button press from Idle.
    *   Real-time data (`SD_GNSS_Measurement`) notifications are available if subscribed.
    *   **File Transfer (File_Transfer service) is UNAVAILABLE** due to potential SD card conflicts with logging. Commands will likely return NAK (`0xf0`).
*   **Config Selection Mode (Orange LED):** Entered via short press then long press from Idle. BLE connection may be possible, but file transfer is unavailable.
*   **USB Mode (Red/Green LED):** Mass Storage Device mode. BLE connection may be possible, but **File Transfer (File_Transfer service) is UNAVAILABLE** due to USB using the file system.
*   **Firmware Update Mode (Orange LED when plugged in):** Bootloader mode. BLE is inactive.
*   **Pairing Request Mode (Pulsing Green LED):** Temporary mode (30s) entered via double-press from Idle. Allows connection from *any* device to initiate pairing. File transfer might be available after pairing is complete *within* this mode, but it's generally expected the app will proceed after pairing without extensive file ops immediately.
*   **Start Mode (Orange LED):** Special mode for synchronized start timing. Entered via a ~1s power button press from Idle if `Active_Mode` in `flysight.txt` is set to `1`.
    *   Real-time data (`SD_GNSS_Measurement`) notifications are available if subscribed.
    *   **File Transfer (File_Transfer service) is UNAVAILABLE.**
    *   Starter Pistol service (`SP_Control_Point`, `SP_Result`) is active.

### Advertising Details

*   **Modes:** Fast advertising (100ms interval for 30s after reset/button press), then Slow advertising (2.5s interval). Uses Limited Discoverable in Pairing Request Mode, Undirected Connectable otherwise.
*   **Address:** Uses Random Private Resolvable Addresses (RPA). Central devices should support RPAs. Issues were observed when a central tried using Random Static Address after pairing.
*   **Manufacturer Specific Data (Adv. Packet Payload):**
    *   Format: `[Length: 0x04] [Type: 0xFF] [Manuf. ID LSB: 0xDB] [Manuf. ID MSB: 0x09] [Status Byte: 0x00 or 0x01]`
    *   Manufacturer ID: `0x09DB` (Bionic Avionics Inc.)
    *   **Status Byte:**
        *   `0x00`: Normal operation (not in pairing request mode).
        *   `0x01`: **In Pairing Request Mode.**
    *   Use this data to filter scans specifically for FlySight 2 devices and identify those ready for initial pairing.

## BLE Services and Characteristics

### Control Point Protocol (Common for SD, SP, DS Control Points)

Control Point characteristics in the Sensor_Data, Starter_Pistol, and Device_State services follow a common request/response pattern.

*   **Request (Central to FlySight):**
    *   Write to the respective Control Point characteristic.
    *   Format: `[Command Opcode (uint8)] [Optional Parameters...]`
*   **Response (FlySight to Central):**
    *   Sent via an **Indication** on the same Control Point characteristic (client must enable indications and send an ACK for each indication received).
    *   Format: `[Response ID (0xF0)] [Request Opcode (echoed)] [Status (uint8)] [Optional Response Data...]`
    *   **Status Codes (`CP_STATUS_*`):**
        *   `0x01`: Success (`CP_STATUS_SUCCESS`)
        *   `0x02`: Command Not Supported (`CP_STATUS_CMD_NOT_SUPPORTED`)
        *   `0x03`: Invalid Parameter (`CP_STATUS_INVALID_PARAMETER`)
        *   `0x04`: Operation Failed (`CP_STATUS_OPERATION_FAILED`)
        *   `0x05`: Operation Not Permitted (`CP_STATUS_OPERATION_NOT_PERMITTED`)
        *   `0x06`: Busy (`CP_STATUS_BUSY`)
        *   Other values: Error Unknown or specific error.
    *   Max length of `Optional Response Data` is 17 bytes (`MAX_CP_OPTIONAL_RESPONSE_DATA_LEN`).

### 1. File_Transfer Service

Provides serial-like file system operations. Requires bonding. File operations only functional in Idle Mode.

*   **Service UUID:** `00000000-cc7a-482a-984a-7f2ed5b3e58f`
*   **Characteristics:**

    *   **`FT_Packet_Out` (Transmit from FlySight)**
        *   UUID: `00000001-8e22-4541-9d4c-21edae82ed19`
        *   Properties: **Notify**
        *   Permissions: Encrypted Read/Write required for CCCD (Client Characteristic Configuration Descriptor) to enable notifications.
        *   Usage: Sends data packets (`0x10`), file info packets (`0x11`), and ACK/NAK responses (`0xf1`/`0xf0`, `0x12`) *from* FlySight *to* the central device via notifications. Central must enable notifications on this characteristic after connecting and bonding.
        *   Max Length: 244 bytes (`SizeFt_Packet_Out`) - Note: Effective length depends on negotiated MTU.

    *   **`FT_Packet_In` (Receive by FlySight)**
        *   UUID: `00000002-8e22-4541-9d4c-21edae82ed19`
        *   Properties: **WriteWithoutResponse, Read**
        *   Permissions: Encrypted Read/Write required.
        *   Usage: Receives commands and file data *from* the central device *to* the FlySight.
            *   Uses WriteWithoutResponse for speed. Reliability via application-layer ACKs and GBN ARQ.
            *   Reading this characteristic can be used to trigger the pairing process after initial connection in Pairing Request Mode.
            *   *Any* write to this characteristic resets the 30s connection inactivity timer.
        *   Max Length: 244 bytes (`SizeFt_Packet_In`) - Note: Effective length depends on negotiated MTU.

#### File Transfer Command Protocol

Packets sent over `FT_Packet_In` (WriteWithoutResponse) and received via notifications on `FT_Packet_Out`.

**Packet Format:** `[Command Byte (uint8)] [Payload...]`

**Commands from Central to FlySight (Write to `FT_Packet_In`):**

| Opcode | Command Name         | Payload Format                                                        | Notes                                                                              |
| :----- | :------------------- | :-------------------------------------------------------------------- | :--------------------------------------------------------------------------------- |
| `0x01` | Delete File/Dir    | `Path (null-terminated string)`                                       |                                                                                    |
| `0x02` | Read File            | `Offset_multiplier (uint32_t), Stride_minus_1_multiplier (uint32_t), Path (null-terminated string)` | Opens file for reading. `Offset = Offset_multiplier * FRAME_LENGTH`. `Stride = (Stride_minus_1_multiplier + 1) * FRAME_LENGTH`. Path follows stride. |
| `0x03` | Write File (Open)    | `Path (null-terminated string)`                                       | Opens/Creates (overwrites) file for writing. FlySight will be ready to receive file data chunks via `0x10` command. |
| `0x04` | Create Directory     | `Path (null-terminated string)`                                       |                                                                                    |
| `0x05` | List Directory       | `Path (null-terminated string)`                                       |                                                                                    |
| `0x10` | File Data (Write Chunk)| `PacketCounter (uint8), DataBytes (...)`                              | Chunk of file data sent by Central to FS during write. Max data length = MTU - 3. Requires `0x12` ACK from FS. |
| `0x12` | ACK File Data Packet | `PacketCounter (uint8)`                                               | Acknowledges receipt of packet `0x10` (during read from FS).                       |
| `0xfe` | Ping                 | (None)                                                                | Keeps connection alive. Resets 30s timeout.                                        |
| `0xff` | Cancel Transfer      | (None)                                                                | Cancels ongoing Read/Write/List operation.                                         |

*Deprecated Commands (since ~v2024.09.29):*
*   `0x00`: Create File. Use Write File (`0x03` with path) instead; it now creates/overwrites. Sending `0x00` may return NAK.

**Responses/Data from FlySight to Central (Notify on `FT_Packet_Out`):**

| Opcode | Response Name         | Payload Format                                                            | Notes                                                                                    |
| :----- | :-------------------- | :------------------------------------------------------------------------ | :--------------------------------------------------------------------------------------- |
| `0x10` | File Data (Read Chunk)| `PacketCounter (uint8), DataBytes (...)`                                  | Chunk of file data sent by FS *to* Central during read. Max data length = MTU - 3. Zero-length data signals EOF. Requires `0x12` ACK from Central. |
| `0x11` | File Info             | `PacketCounter (uint8), Size (uint32_t), Date (uint16), Time (uint16), Attributes (uint8), Name (13 bytes, null-padded)` | One packet per dir entry. `Name[0]==0` signals end of list. Date/Time format per FATFS standard. |
| `0x12` | ACK File Data Packet  | `PacketCounter (uint8)`                                                   | Acknowledges receipt of packet `0x10` (during write to FS).                               |
| `0xf0` | NAK                   | `OriginalCommand (uint8)`                                                 | Command failed (e.g., file busy, not found, invalid).                                      |
| `0xf1` | ACK                   | `OriginalCommand (uint8)`                                                 | Command initiation successful (e.g., file opened, ping received, dir opened, delete OK). |

*(Note: All multi-byte integers are Little Endian unless specified otherwise. `FRAME_LENGTH` is 242 from `crs.c`)*

#### Go-Back-N ARQ for Reliable File Transfer

*   **Purpose:** Ensure reliable transfer over unreliable `WriteWithoutResponse` and `Notify`.
*   **Applies To:**
    *   Reading from FlySight: Central sends `0x02` (Read File), FlySight sends `0x10` (File Data Read Chunk) packets.
    *   Writing to FlySight: Central sends initial `0x03` (Write File Open), then Central sends `0x10` (File Data Write Chunk) packets.
*   **Window Size:** 8 packets (`FS_CRS_WINDOW_LENGTH`). The sender can send up to 8 data packets before needing an ACK for the first one.
*   **Packet Counter:** A **1-byte** counter included in each `0x10` data packet (both directions). Increments modulo 256 (wraps 0xff -> 0x00).
*   **Acknowledgement (`0x12`):** The **receiver** of `0x10` data packets sends an `0x12` ACK (containing the received packet's counter) back to the **sender** for *each* `0x10` data packet successfully received *in the expected sequence*.
*   **Timeout & Retransmission:**
    *   The **sender** of `0x10` data packets uses a timeout (`TX_TIMEOUT_MSEC` = 200ms for reads from FS; `RX_TIMEOUT_MSEC` = 10000ms for FS waiting for data from central during a write).
    *   If the sender doesn't receive the expected `0x12` ACK within its timeout, it assumes the data packet or its ACK was lost.
    *   It **goes back** and retransmits all packets starting from the first unacknowledged one.
    *   The receiver discards any duplicate or out-of-order packets based on the packet counter.
*   **End of Transfer:**
    *   **Reading from FS:** FlySight signals EOF by sending a final `0x10` data packet with a payload containing only the packet counter (data length of 1).
    *   **Writing to FS:** Central signals EOF by sending a final `0x10` data packet with a payload containing only the packet counter (data length of 1, indicating zero actual data bytes).
    *   The receiver still ACKs this final zero-data-length packet.
*   **Cancel:** Sending `0xff` to `FT_Packet_In` aborts the current transfer state on FlySight.

**Revised Flow (Writing to FlySight):**
1.  Central sends `0x03 [Path]` to `FT_Packet_In` to open/create the file.
2.  FlySight, if successful, sends `0xf1 03` (ACK) on `FT_Packet_Out`. FlySight is now ready to receive file data chunks.
3.  Central sends a window of `0x10 [PacketCounter] [DataBytes]` packets to `FT_Packet_In`.
4.  FlySight receives `0x10` packets. For each valid sequential packet, it writes data and sends `0x12 [PacketCounter]` ACK on `FT_Packet_Out`.
5.  Repeat until entire file is sent, ending with a `0x10 [PacketCounter]` (zero data bytes) packet.

### 2. Sensor_Data Service

Provides live GNSS data when FlySight is in Active Mode or Start Mode. Requires bonding.

*   **Service UUID:** `00000001-cc7a-482a-984a-7f2ed5b3e58f`
*   **Characteristics:**

    *   **`SD_GNSS_Measurement` (Position/Velocity/etc.)**
        *   UUID: `00000000-8e22-4541-9d4c-21edae82ed19`
        *   Properties: **Read, Notify**
        *   Permissions: Encrypted Read/Write required.
        *   Usage: Streams selected GNSS data fields. Updates at rate set in `config.txt` *only when FlySight is in Active Mode or Start Mode*. Central must enable notifications. Reading can trigger pairing. The specific fields included are controlled by a bitmask set via `SD_Control_Point`.
        *   Max Length: 44 bytes (`SizeSd_Gnss_Measurement`). Actual length depends on the fields enabled by the mask.
        *   **Data Format (Variable Length, Little Endian):**
            *   Byte 0: `current_mask` (uint8). A bitmask indicating which fields are *actually present* in this specific notification packet. This mask is a subset of or equal to the mask set by `SD_Control_Point`.
                *   `0x80` (`GNSS_BLE_BIT_TOW`): Time of Week included.
                *   `0x40` (`GNSS_BLE_BIT_WEEK`): Week Number included (Not yet implemented in firmware).
                *   `0x20` (`GNSS_BLE_BIT_POSITION`): Position (Lon, Lat, HAE) included.
                *   `0x10` (`GNSS_BLE_BIT_VELOCITY`): Velocity (VelN, VelE, VelD) included.
                *   `0x08` (`GNSS_BLE_BIT_ACCURACY`): Accuracy (HAcc, VAcc, SAcc) included.
                *   `0x04` (`GNSS_BLE_BIT_NUM_SV`): Number of Satellites included.
            *   **Following Bytes (order matters if present, based on `current_mask`):**
                *   If `GNSS_BLE_BIT_TOW` is set: `iTOW` (uint32_t). GNSS Time of Week (ms). (4 bytes)
                *   If `GNSS_BLE_BIT_WEEK` is set: (Data for week number - TBD)
                *   If `GNSS_BLE_BIT_POSITION` is set:
                    *   `Longitude` (int32_t). Degrees * 1e7. (4 bytes)
                    *   `Latitude` (int32_t). Degrees * 1e7. (4 bytes)
                    *   `hMSL` (int32_t). Height above Mean Sea Level (mm). (4 bytes)
                *   If `GNSS_BLE_BIT_VELOCITY` is set:
                    *   `velN` (int32_t). Velocity North (mm/s). (4 bytes)
                    *   `velE` (int32_t). Velocity East (mm/s). (4 bytes)
                    *   `velD` (int32_t). Velocity Down (mm/s). (4 bytes)
                *   If `GNSS_BLE_BIT_ACCURACY` is set:
                    *   `hAcc` (uint32_t). Horizontal Accuracy Estimate (mm). (4 bytes)
                    *   `vAcc` (uint32_t). Vertical Accuracy Estimate (mm). (4 bytes)
                    *   `sAcc` (uint32_t). Speed Accuracy Estimate (mm/s). (4 bytes)
                *   If `GNSS_BLE_BIT_NUM_SV` is set:
                    *   `numSV` (uint8_t). Number of satellites in solution. (1 byte)
        *   Default Mask: `0xB0` (iTOW, Position, Velocity).

    *   **`SD_Control_Point`**
        *   UUID: `00000006-8e22-4541-9d4c-21edae82ed19`
        *   Properties: **Write, Indicate**
        *   Permissions: Encrypted Read/Write required.
        *   Usage: Used to control which data fields are included in the `SD_GNSS_Measurement` characteristic notifications. Also used to retrieve the current mask. Central enables indications if responses are desired.
        *   Max Length: 20 bytes (`SizeSd_Control_Point`). Variable length.
        *   **Write Operations (Central to FlySight):**
            *   Byte 0: Opcode
                *   `0x01` (`SD_CMD_SET_GNSS_BLE_MASK`): Set the data mask for `SD_GNSS_Measurement`.
                    *   Payload: `[new_mask (uint8)]`. (Total length: 2 bytes)
                *   `0x02` (`SD_CMD_GET_GNSS_BLE_MASK`): Request the current data mask.
                    *   Payload: (None). (Total length: 1 byte)
        *   **Indication Responses (FlySight to Central, if indications enabled):**
            *   Format: `[0xF0 (CP_RESPONSE_ID)] [Request Opcode] [Status] [Optional Data]`
            *   For `SD_CMD_SET_GNSS_BLE_MASK (0x01)`:
                *   Success: `[0xF0] [0x01] [0x01 (CP_STATUS_SUCCESS)]`
                *   Invalid Param: `[0xF0] [0x01] [0x03 (CP_STATUS_INVALID_PARAMETER)]`
            *   For `SD_CMD_GET_GNSS_BLE_MASK (0x02)`:
                *   Success: `[0xF0] [0x02] [0x01 (CP_STATUS_SUCCESS)] [current_mask (uint8)]`
                *   Invalid Param: `[0xF0] [0x02] [0x03 (CP_STATUS_INVALID_PARAMETER)]`
            *   For unknown command:
                *   `[0xF0] [Received Opcode] [0x02 (CP_STATUS_CMD_NOT_SUPPORTED)]`

### 3. Starter_Pistol Service

Used for synchronized start timing in specific scenarios (e.g., BASE race). Only active in Start Mode. Requires bonding.

*   **Service UUID:** `00000002-cc7a-482a-984a-7f2ed5b3e58f`
*   **Characteristics:**

    *   **`SP_Control_Point`**
        *   UUID: `00000003-8e22-4541-9d4c-21edae82ed19`
        *   Properties: **Write, Indicate**
        *   Permissions: Encrypted Read/Write required.
        *   Usage: Sends control commands to the start pistol feature. Indications provide feedback.
        *   Max Length: 20 bytes (`SizeSp_Control_Point`). Variable length.
        *   **Write Operations (Central to FlySight):**
            *   Byte 0: Opcode
                *   `0x01` (`SP_CMD_START_COUNTDOWN`): Initiate the start sequence (countdown and tone).
                    *   Payload: (None). (Total length: 1 byte)
                *   `0x02` (`SP_CMD_CANCEL_COUNTDOWN`): Cancel an ongoing start sequence.
                    *   Payload: (None). (Total length: 1 byte)
        *   **Indication Responses (FlySight to Central, if indications enabled):**
            *   Format: `[0xF0 (CP_RESPONSE_ID)] [Request Opcode] [Status]` (No optional data for these commands)
            *   For `SP_CMD_START_COUNTDOWN (0x01)`:
                *   Success: `[0xF0] [0x01] [0x01 (CP_STATUS_SUCCESS)]`
                *   Busy: `[0xF0] [0x01] [0x06 (CP_STATUS_BUSY)]`
                *   Not Permitted: `[0xF0] [0x01] [0x05 (CP_STATUS_OPERATION_NOT_PERMITTED)]`
                *   Invalid Param (if payload was not empty): `[0xF0] [0x01] [0x03 (CP_STATUS_INVALID_PARAMETER)]`
            *   For `SP_CMD_CANCEL_COUNTDOWN (0x02)`:
                *   Success: `[0xF0] [0x02] [0x01 (CP_STATUS_SUCCESS)]`
                *   Not Permitted: `[0xF0] [0x02] [0x05 (CP_STATUS_OPERATION_NOT_PERMITTED)]`
                *   Invalid Param (if payload was not empty): `[0xF0] [0x02] [0x03 (CP_STATUS_INVALID_PARAMETER)]`
            *   For unknown command:
                *   `[0xF0] [Received Opcode] [0x02 (CP_STATUS_CMD_NOT_SUPPORTED)]`

    *   **`SP_Result`**
        *   UUID: `00000004-8e22-4541-9d4c-21edae82ed19`
        *   Properties: **Read, Indicate**
        *   Permissions: Encrypted Read/Write required.
        *   Usage: Provides results from the start pistol function (e.g., precise start time captured by GNSS EXTINT).
        *   Length: 9 bytes (`SizeSp_Result`). Data format (from `Custom_Start_Update`): `Year (u16), Month (u8), Day (u8), Hour (u8), Min (u8), Sec (u8), ms (u16)`. All Little Endian.

### 4. Device_State Service

Provides information about the device's current operational state and allows for state control. Requires bonding.

*   **Service UUID:** `00000003-cc7a-482a-984a-7f2ed5b3e58f`
*   **Characteristics:**

    *   **`DS_Mode`**
        *   UUID: `00000005-8e22-4541-9d4c-21edae82ed19`
        *   Properties: **Read, Indicate**
        *   Permissions: Encrypted Read required.
        *   Usage: Exposes the current operational mode of the FlySight 2. Central can read this or subscribe to indications to be informed of mode changes.
        *   Length: 1 byte (`SizeDs_Mode`).
        *   **Mode Values (corresponds to `FS_Mode_State_t` enum):**
            | Value | Mode Name        | Description                                     |
            | :---- | :--------------- | :---------------------------------------------- |
            | `0x00`| SLEEP            | Idle, low power. File access available.         |
            | `0x01`| ACTIVE           | GNSS logging, audio feedback. No file access.   |
            | `0x02`| CONFIG           | Configuration selection mode. No file access.   |
            | `0x03`| USB              | USB Mass Storage mode. No file access.          |
            | `0x04`| PAIRING          | BLE pairing request mode.                       |
            | `0x05`| START            | Starter pistol mode. No file access.            |

    *   **`DS_Control_Point`**
        *   UUID: `00000007-8e22-4541-9d4c-21edae82ed19`
        *   Properties: **Write, Indicate**
        *   Permissions: Encrypted Read/Write required.
        *   Usage: This characteristic is intended for future device state control commands. Currently, no specific commands are implemented. Writing any command will likely result in a `CP_STATUS_CMD_NOT_SUPPORTED` response.
        *   Max Length: 20 bytes (`SizeDs_Control_Point`). Variable length.
        *   **Write Operations (Central to FlySight):**
            *   Currently, no commands are defined.
        *   **Indication Responses (FlySight to Central, if indications enabled):**
            *   Format: `[0xF0 (CP_RESPONSE_ID)] [Request Opcode] [Status]`
            *   For any command written:
                *   `[0xF0] [Received Opcode] [0x02 (CP_STATUS_CMD_NOT_SUPPORTED)]`

### 5. Standard BLE Services

FlySight 2 also implements standard BLE services:

*   **Device Information Service (DIS - 0x180A):** Provides Manufacturer Name ("Bionic Avionics Inc."), Model Number ("FlySight 2"), Firmware Revision (`GIT_TAG`), etc.
*   **Battery Service (BAS - UUID `0x180F`):**
    *   FlySight 2 exposes its own battery level through this standard service.
    *   **Battery Level Characteristic (UUID `0x2A19`):**
        *   **Properties:** Read, Notify.
        *   **Permissions:** Encrypted Read.
        *   **Format:** `uint8` representing the battery level as a percentage (0-100%).
        *   **Usage:** Client applications can read this characteristic to get the current battery level of the FlySight 2. Notifications can be enabled on this characteristic to receive updates once per second.

## Implementation Notes & Best Practices

*   **Error Handling:** Expect NAKs (`0xf0`) for commands sent when the FlySight is in an incompatible state (e.g., file transfer during Active mode or USB connection). Handle timeouts appropriately, especially for the GBN ARQ.
*   **State Awareness:** Be mindful of the FlySight's operational mode (Idle, Active, USB), which can be obtained via the `DS_Mode` characteristic. File transfer is generally only reliable in Idle mode. Real-time data (`SD_GNSS_Measurement`) is only sent in Active or Start mode.
*   **MTU Negotiation:** Request a larger MTU (e.g., 247 bytes) after connection for efficient file transfer. Handle potential failures where the MTU remains small (e.g., 23 bytes with BLE 4.0/4.1 devices), potentially by fragmenting File Transfer data packets if necessary (though the current File Transfer implementation seems to assume MTU is large enough for its framing).
*   **Pairing Flow:** Implement the connect-then-read-secure-characteristic flow to reliably trigger pairing, especially if developing a cross-platform application.
*   **Connection Management:** Use the ping command (`0xfe` on `FT_Packet_In`) to keep connections alive during inactivity. Handle disconnects gracefully. Be aware of the Android system pairing behavior possibly leaving connections open.
*   **Code Examples:** Refer extensively to the provided Python, iOS, and Android examples for practical implementation details, especially for the GBN ARQ logic.

## Appendix

### Known UUIDs

| Feature                 | Characteristic Name       | UUID                                         | Service            | Properties             | Max Length (Bytes) |
| :---------------------- | :------------------------ | :------------------------------------------- | :----------------- | :--------------------- | :----------------- |
| File Transfer Service   |                           | `00000000-cc7a-482a-984a-7f2ed5b3e58f`       | File_Transfer      |                        | N/A                |
| File Transfer TX        | `FT_Packet_Out`           | `00000001-8e22-4541-9d4c-21edae82ed19`       | File_Transfer      | Notify                 | 244 (Var)          |
| File Transfer RX        | `FT_Packet_In`            | `00000002-8e22-4541-9d4c-21edae82ed19`       | File_Transfer      | WriteWithoutResponse, Read | 244 (Var)          |
| Sensor Data Service     |                           | `00000001-cc7a-482a-984a-7f2ed5b3e58f`       | Sensor_Data        |                        | N/A                |
| GNSS Measurement        | `SD_GNSS_Measurement`     | `00000000-8e22-4541-9d4c-21edae82ed19`       | Sensor_Data        | Read, Notify           | 44 (Var)           |
| Sensor Data Control     | `SD_Control_Point`        | `00000006-8e22-4541-9d4c-21edae82ed19`       | Sensor_Data        | Write, Indicate        | 20 (Var)           |
| Starter Pistol Service  |                           | `00000002-cc7a-482a-984a-7f2ed5b3e58f`       | Starter_Pistol     |                        | N/A                |
| Starter Pistol Control  | `SP_Control_Point`        | `00000003-8e22-4541-9d4c-21edae82ed19`       | Starter_Pistol     | Write, Indicate        | 20 (Var)           |
| Starter Pistol Result   | `SP_Result`               | `00000004-8e22-4541-9d4c-21edae82ed19`       | Starter_Pistol     | Read, Indicate         | 9 (Const)          |
| Device State Service    |                           | `00000003-cc7a-482a-984a-7f2ed5b3e58f`       | Device_State       |                        | N/A                |
| Device Mode             | `DS_Mode`                 | `00000005-8e22-4541-9d4c-21edae82ed19`       | Device_State       | Read, Indicate         | 1 (Const)          |
| Device State Control    | `DS_Control_Point`        | `00000007-8e22-4541-9d4c-21edae82ed19`       | Device_State       | Write, Indicate        | 20 (Var)           |
| Battery Service         |                           | `0x180F`                                     | Battery            |                        | N/A                |
| Battery Level           | `Battery_Level`           | `0x2A19`                                     | Battery            | Read, Notify           | 1 (Const)          |

*(Note: "Var" indicates variable length up to the specified maximum. "Const" indicates fixed length.)*

### File Transfer Command Opcodes

| Opcode | Command Name           | Direction      | Payload Description                            | Response (on `FT_Packet_Out`)                      |
| :----- | :--------------------- | :------------- | :--------------------------------------------- | :------------------------------------------------- |
| `0x01` | Delete File/Dir      | Central -> FS  | `[Path]`                                       | `0xf1 01` (ACK) / `0xf0 01` (NAK)                  |
| `0x02` | Read File              | Central -> FS  | `[Offset_mult (u32)] [Stride-1_mult (u32)] [Path]` | `0xf1 02` (ACK) / `0xf0 02` (NAK), then `0x10` data packets |
| `0x03` | Write File (Open)      | Central -> FS  | `[Path]`                                       | `0xf1 03` (ACK) / `0xf0 03` (NAK)                  |
| `0x04` | Create Directory       | Central -> FS  | `[Path]`                                       | `0xf1 04` (ACK) / `0xf0 04` (NAK)                  |
| `0x05` | List Directory         | Central -> FS  | `[Path]`                                       | `0xf1 05` (ACK) / `0xf0 05` (NAK), then `0x11` info packets |
| `0x10` | File Data (Chunk)      | Bidirectional  | `[Counter (u8)] [Data...]`                     | Receiver sends `0x12 [Counter]` ACK                |
| `0x11` | File Info              | FS -> Central  | `[Counter (u8)] [Size] [Date] [Time] [Attr] [Name]` | N/A                                                |
| `0x12` | ACK File Data Packet   | Receiver -> Sender | `[Counter (u8)]`                           | N/A                                                |
| `0xf0` | NAK                    | FS -> Central  | `[Original Command (u8)]`                    | N/A                                                |
| `0xf1` | ACK                    | FS -> Central  | `[Original Command (u8)]`                    | N/A                                                |
| `0xfe` | Ping                   | Central -> FS  | (None)                                         | `0xf1 fe` (ACK)                                    |
| `0xff` | Cancel Transfer        | Central -> FS  | (None)                                         | (None expected, state just resets)                 |

*(Note: Path is a null-terminated string. `0x10` is used by Central to send data chunks to FS (via `FT_Packet_In`) during a write operation (after `0x03` has opened the file), and by FS to send data chunks to Central (via `FT_Packet_Out`) during a read operation. `FRAME_LENGTH` is 242.)*
