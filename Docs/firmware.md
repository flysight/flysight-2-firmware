# FlySight Firmware API Developer Documentation

## Overview

This document describes the JSON firmware endpoint for FlySight devices. Third-party applications can use this endpoint to determine which encrypted firmware builds are compatible with a particular FlySight, whether a wireless stack update is required, and where to download the required files.

The endpoint is intended to replace screen-scraping or hard-coded batch tables. A client provides the raw contents of the device's `FLYSIGHT.TXT` file, and the server returns only firmware files that are compatible with that device.

**Note:** This API is provided by the FlySight firmware download WordPress plugin (`wp-firmware-download`). Details may change in future versions. Clients should ignore unknown JSON fields and handle documented error responses.

## Prerequisites

*   **Device State File:** The client must be able to read the raw contents of `FLYSIGHT.TXT` from the root of the FlySight storage. This can be done over USB mass storage or over the FlySight 2 BLE file transfer service when the device is in Idle Mode.
*   **HTTPS Client:** The client must be able to send JSON over HTTPS and download binary files from returned URLs.
*   **Checksum Verification:** The client should verify downloaded files using the returned `size_bytes` and `sha256` metadata before using them.
*   **Firmware Installer:** To install downloaded files, the client must be able to copy them to the returned `target_path` values on the FlySight and start the firmware install flow. For BLE-based installation details, see [`ble.md`](ble.md).

## Core Concepts

### Device Identity and Compatibility

FlySight firmware files are encrypted for specific device keys. A firmware file that works on one FlySight may not install on another FlySight from a different hardware batch or provisioning group.

The endpoint uses fields in `FLYSIGHT.TXT` to identify the device and filter the firmware catalog:

*   **`Firmware_Ver`**: The currently installed application firmware version. This is returned in the response for display and logging.
*   **`Device_ID` and `Session_ID`**: Device identifiers used to validate that the submitted text is actually `FLYSIGHT.TXT`.
*   **`Pubkey_X`**: The public-key identifier used to select compatible encrypted firmware files. This is the primary compatibility field for modern FlySight firmware.
*   **`Stack_Ver`**: The installed wireless stack version. This is used to decide whether a stack update must be downloaded before installing a selected application firmware.

For a modern state file, `Device_ID` and `Session_ID` must be 24-character hexadecimal strings, and `Pubkey_X` must be a 64-character hexadecimal string. `Stack_Ver` should use `major.minor.patch` format, such as `1.19.0`; if it is missing or invalid, the endpoint treats the installed stack version as unknown.

Clients should not maintain their own `Pubkey_X` to batch mapping unless they are building release tooling. For update selection, treat the endpoint response as authoritative: if a firmware option is returned, it is compatible with the submitted `FLYSIGHT.TXT`; if it is not returned, do not offer it for that device.

### Legacy Devices

Modern `FLYSIGHT.TXT` files include `Pubkey_X`. Older firmware before `v2023.09.22` may not include it. The endpoint can still recognize legacy state files when the file contains valid device identity fields plus legacy state fields such as `Config_File` and `Temp_Folder`.

For legacy devices, the response sets:

*   `device.legacy` to `true`
*   `device.stack_version` to `0.0.0`
*   `device.stack_version_unknown` to `true`

When `stack_version_unknown` is true, clients should assume that a returned stack update is intentional and should not suppress it just because the device did not report a stack version.

### Firmware and Stack Files

The endpoint may return two different update files for a firmware option:

*   **Application firmware:** Always present as `firmware`. This is the encrypted `.sfb` application update file.
*   **Wireless stack:** Present as `stack.file` only when `stack.update_required` is true. This is the STM32 wireless stack update file required by the selected application firmware.

Each file object includes:

*   `url`: Download URL for the binary file.
*   `target_path`: Path where the file should be written on the FlySight before installation.
*   `size_bytes`: Expected byte count.
*   `sha256`: Expected SHA-256 digest as lowercase hexadecimal.

Current target paths are:

| File | `target_path` |
| :--- | :------------ |
| Application firmware | `FW/APP.SFB` |
| Wireless stack | `FW/STACK.BIN` |

Clients should use the `target_path` values from the response instead of hard-coding them.

The binary download response may use a generic attachment filename such as `APP.SFB` or `STACK.BIN`. Do not infer compatibility or install location from the URL filename or attachment filename; use the metadata in the manifest.

### Recommended Firmware

The response includes both `recommended` and `firmwares`.

*   `firmwares` contains every compatible firmware option returned by the server, sorted newest first.
*   `recommended` is the first entry in `firmwares`.
*   Beta firmware is excluded by default. Add `include_beta=true` to include beta releases.

Applications may offer the whole `firmwares` list to advanced users, but the default update path should normally use `recommended`.

The endpoint returns compatible firmware options, not only upgrades. A returned option may match the installed `device.firmware_version` or may be older than the installed version. Clients that want to present "update available" UI should compare the selected option against `device.firmware_version`.

## Firmware Manifest Endpoint

### Request

*   **URL:** `https://flysight.ca/wp-json/flysight/v1/firmware`
*   **Method:** `POST`
*   **Content-Type:** `application/json`
*   **Authentication:** None

Optional query parameters:

| Parameter | Values | Description |
| :-------- | :----- | :---------- |
| `include_beta` | `true` | Include beta firmware options in the returned list. Any other value is treated as false. |

Request body:

```json
{
  "flysight_txt": "; FlySight - http://flysight.ca\n\nFirmware_Ver: v2024.12.30.8\nDevice_ID: 0050002f343050072036314b\nSession_ID: bc394de061ae30f6d101a6f0\nStack_Ver: 1.19.0\nPubkey_X: 728168718da35f7de7cf9ab6b10eb6d74f2cb31614daa2cc4e7963c2ab4e5b11\n"
}
```

The `flysight_txt` value must be the raw text contents of `FLYSIGHT.TXT`. Do not send `CONFIG.TXT`, parsed fields only, or a multipart upload to this JSON endpoint.

Example request:

```bash
curl -X POST 'https://flysight.ca/wp-json/flysight/v1/firmware?include_beta=true' \
  -H 'Content-Type: application/json' \
  --data '{"flysight_txt":"Firmware_Ver: v2024.12.30.8\nDevice_ID: 0050002f343050072036314b\nSession_ID: bc394de061ae30f6d101a6f0\nStack_Ver: 1.19.0\nPubkey_X: 728168718da35f7de7cf9ab6b10eb6d74f2cb31614daa2cc4e7963c2ab4e5b11\n"}'
```

### Successful Response

A successful response has HTTP status `200` and returns a firmware manifest:

```json
{
  "device": {
    "firmware_version": "v2024.12.30.8",
    "stack_version": "1.19.0",
    "stack_version_unknown": false,
    "pubkey_x": "728168718da35f7de7cf9ab6b10eb6d74f2cb31614daa2cc4e7963c2ab4e5b11",
    "legacy": false
  },
  "include_beta": true,
  "recommended": {
    "version": "v2026.03.06",
    "release_date": "2026-03-06",
    "is_beta": false,
    "release_notes_url": "https://flysight.ca/wp-content/uploads/release-notes.html",
    "firmware": {
      "url": "https://flysight.ca/wp-admin/admin-post.php?action=download_firmware&firmware_file=B6_v2026.03.06.sfb",
      "target_path": "FW/APP.SFB",
      "size_bytes": 524288,
      "sha256": "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
    },
    "stack": {
      "required_version": "1.19.0",
      "current_version": "1.19.0",
      "current_version_unknown": false,
      "update_required": false,
      "file": null
    }
  },
  "firmwares": [
    {
      "version": "v2026.03.06",
      "release_date": "2026-03-06",
      "is_beta": false,
      "release_notes_url": "https://flysight.ca/wp-content/uploads/release-notes.html",
      "firmware": {
        "url": "https://flysight.ca/wp-admin/admin-post.php?action=download_firmware&firmware_file=B6_v2026.03.06.sfb",
        "target_path": "FW/APP.SFB",
        "size_bytes": 524288,
        "sha256": "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      },
      "stack": {
        "required_version": "1.19.0",
        "current_version": "1.19.0",
        "current_version_unknown": false,
        "update_required": false,
        "file": null
      }
    }
  ]
}
```

### Response Fields

#### `device`

| Field | Type | Description |
| :---- | :--- | :---------- |
| `firmware_version` | string | Current application firmware version reported by the submitted `FLYSIGHT.TXT`. |
| `stack_version` | string | Current wireless stack version, or `0.0.0` when unknown. |
| `stack_version_unknown` | boolean | True when `Stack_Ver` was missing or invalid in `FLYSIGHT.TXT`. |
| `pubkey_x` | string | Public-key identifier used for compatibility matching. Legacy devices may use a server-side fallback value. |
| `legacy` | boolean | True when the endpoint recognized an older `FLYSIGHT.TXT` format without `Pubkey_X`. |

#### `firmwares[]` and `recommended`

| Field | Type | Description |
| :---- | :--- | :---------- |
| `version` | string | Firmware version label. |
| `release_date` | string | Release date in `YYYY-MM-DD` format. |
| `is_beta` | boolean | True for beta firmware. |
| `release_notes_url` | string or null | URL for release notes, when available. |
| `firmware` | file object | Application firmware download metadata. |
| `stack` | object | Wireless stack requirement and optional download metadata. |

#### File Object

| Field | Type | Description |
| :---- | :--- | :---------- |
| `url` | string | HTTPS URL for downloading the binary file. |
| `target_path` | string | Path where the file should be written on the FlySight before installation. |
| `size_bytes` | integer | Exact expected file size. |
| `sha256` | string | SHA-256 digest of the file as hexadecimal. |

#### `stack`

| Field | Type | Description |
| :---- | :--- | :---------- |
| `required_version` | string | Minimum wireless stack version required by this firmware option. |
| `current_version` | string | Stack version parsed from the submitted `FLYSIGHT.TXT`, or `0.0.0` when unknown. |
| `current_version_unknown` | boolean | True when the endpoint could not determine the installed stack version. |
| `update_required` | boolean | True when a stack update should be staged with the application firmware for installation. |
| `file` | file object or null | Stack update file metadata. Present only when `update_required` is true. |

## Error Responses

Error responses use the standard WordPress REST error shape:

```json
{
  "code": "not_flysight_txt",
  "message": "This does not look like FLYSIGHT.TXT.",
  "data": {
    "status": 400
  }
}
```

Known error codes:

| HTTP Status | Code | Meaning | Client Action |
| :---------- | :--- | :------ | :------------ |
| `400` | `missing_flysight_txt` | The JSON body was missing a string `flysight_txt` value. | Fix the request body. |
| `400` | `config_txt` | The submitted text looks like `CONFIG.TXT`. | Ask the user or device integration to provide `FLYSIGHT.TXT` from the FlySight root directory. |
| `400` | `not_flysight_txt` | The submitted text does not contain the expected FlySight state fields. | Re-read `FLYSIGHT.TXT` and retry. |
| `404` | `unsupported_device` | The device key is not registered for firmware downloads. | Do not offer firmware updates for this device. |
| `404` | `no_firmware` | The device key is registered, but no compatible files are available. | Treat as no update options available. |
| `500` | `missing_file` | Server firmware or stack metadata points to a file that is missing or unreadable. | Treat as a temporary server-side catalog error. |

Clients should display the server-provided `message` when it is useful to the user, but should branch behavior using `code` rather than matching the message text.

## Download and Verification Flow

Use the following flow to determine and download compatible firmware:

1.  Read the full text of `FLYSIGHT.TXT` from the target FlySight.
2.  Send it to `POST /wp-json/flysight/v1/firmware`.
3.  If the response is an error, handle the error code and stop.
4.  Choose a firmware option. Use `recommended` for the default path, or let the user choose from `firmwares`.
5.  If `option.stack.update_required` is true, download `option.stack.file.url`.
6.  Download `option.firmware.url`.
7.  For each downloaded file, verify both:
    *   byte count equals `size_bytes`
    *   SHA-256 digest equals `sha256`
8.  Use each file's `target_path` when copying the file to the FlySight for installation.

The endpoint only tells the client which files are compatible and where to download them. It does not install the firmware. Installation is performed by the client through USB mass storage or BLE file transfer; over BLE, the firmware install is requested with `DS_CMD_INSTALL_UPLOADED_FIRMWARE` after the files have been written.

## Implementation Notes

*   Preserve the exact `FLYSIGHT.TXT` text when sending `flysight_txt`. The parser accepts normal whitespace, comments, and semicolon-delimited field comments.
*   Do not require `Stack_Ver` to be present before calling the endpoint. Missing or invalid stack versions are represented as `0.0.0` with `stack_version_unknown` set to true.
*   Do not assume a stack update is unnecessary because the application firmware download is present. Check `stack.update_required` for the selected firmware option.
*   If `stack.update_required` is true, `stack.file` is expected to be present. Treat a missing `stack.file` as an invalid manifest.
*   Download URLs are generated by the server and may include query parameters. Treat them as opaque URLs.
*   `release_notes_url` may be null.
*   Unknown response fields should be ignored for forward compatibility.
