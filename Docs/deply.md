# FlySight Firmware Deployment Process

## Overview

This document describes the process for preparing and deploying encrypted firmware updates for FlySight devices. The primary tool for this process is the `deploy_firmware.py` Python script, which encrypts firmware binaries (`.bin`) and packages them into a secure firmware update format (`.sfb`). This script and its associated folders are typically located in the `/Deploy` folder of the firmware distribution.

This documentation is intended for developers involved in building and releasing FlySight firmware. It covers the necessary tools, file organization, script configuration, and the step-by-step procedure for generating deployment-ready firmware files.

## Prerequisites

Before you can deploy FlySight firmware, ensure you have the following:

*   **Python Environment:** Python 3.6 or newer.
*   **Required Python Libraries:**
    *   `coincurve`
    *   `ecdh_aesgcm`
    You can install these libraries using pip:
    ```bash
    pip install coincurve ecdh_aesgcm
    ```
*   **Deployment Script:** The `deploy_firmware.py` script (from the `/Deploy` folder).
*   **Firmware Binaries:** Compiled firmware files (e.g., `UserApp.bin`) ready for deployment.
*   **Public Key Files:** Batch-specific public key files (e.g., `pub_key_b2.bin`) corresponding to the target FlySight units.

## Key Components

### Directory Structure

Within the `/Deploy` folder of the firmware distribution, a specific directory structure is expected by the `deploy_firmware.py` script:

*   `Firmware_As_Built/`: This directory is used to store the compiled firmware binaries that you intend to deploy. You will typically copy your `UserApp.bin` (or other named firmware versions) into this folder.
*   `Public_Keys/`: This directory should contain the public key files (`.bin`) for the different production batches of FlySight devices.
*   `Firmware_To_Deploy/`: This is the output directory where the script will place the generated encrypted firmware update files (`.sfb`). The script will create this directory if it does not already exist.

### Input Files

#### Firmware Binaries

*   The primary firmware file is typically named `UserApp.bin`. After a successful firmware build, this file can usually be found in the `/Release` folder of the firmware project.
*   These files should be placed in the `Deploy/Firmware_As_Built/` directory.
*   You can place multiple firmware files in this directory if you intend to deploy different versions or types of firmware. Each will be configured in the deployment script.

#### Public Key Files

*   Public keys are stored as binary files (e.g., `pub_key_b2.bin`, `pub_key_b3.bin`) in the `Deploy/Public_Keys/` directory.
*   Each public key corresponds to a specific manufacturing batch of FlySight devices (e.g., B2, B3). Firmware encrypted with a particular public key can only be installed on devices from the corresponding batch.

### Identifying the Correct Public Key for a Device

FlySight devices are manufactured in batches, and each batch is provisioned with a specific public key. To determine which public key was used for a particular FlySight unit:

1.  Connect the FlySight device to a computer via USB. It will mount as a USB mass storage device (a drive).
2.  Open the `flysight.txt` file located in the root directory of the FlySight's storage.
3.  Look for the `Pubkey_X` value. This hexadecimal string is a hash of the public key used by the device.
4.  Use the table below to map this `Pubkey_X` value to the corresponding public key file and batch name.

**Public Key Index:**

| `Pubkey_X` (from `flysight.txt`)                             | Public Key File      | Batch Name |
| :--------------------------------------------------------- | :------------------- | :--------- |
| `486bee2d3dd60fd0de481a072d12ae5492c52c0cb2bbdde697b16aad1ca903f3` | `pub_key_b2.bin`     | B2         |
| `211d721dbd9114a6bbe9ba1256ccbbfbf2c4015b139126ab56c331935e29c016` | `pub_key_b3.bin`     | B3         |
| `dac40d2597e371390a84f3196d4256b5c96a89373afb3584dcbc226b74bcbc0c` | `pub_key_b4.bin`     | B4         |
| `3157e02846ddfd2a78347e099ba1d86c92920b101137f28091ed0dec3b4589c2`  | `pub_key_b5.bin`     | B5         |
| `8ee7870905b3e792df0ed796b996c9ff743d196c2e8abbf1a39f39733cceac80` | `pub_key_b6.bin`     | B6         |

## The `deploy_firmware.py` Script

The `deploy_firmware.py` script automates the process of encrypting firmware files for secure updates.

### Core Functionality

The script performs the following actions for each specified firmware file and public key combination:
1.  Reads the target device batch's public key.
2.  Generates a new, random ephemeral key pair for this encryption session.
3.  Reads the firmware binary data.
4.  Encrypts the firmware data using an ECDH-derived shared secret and AES-GCM.
5.  Constructs a header (`.sfb` header) containing metadata, cryptographic information (like the ephemeral public key, nonce, and firmware tag), and a MAC for header integrity.
6.  Combines the header and the encrypted firmware into a single `.sfb` file.

### Configuration

At the bottom of the `deploy_firmware.py` script, within the `main()` function, two Python lists control the deployment process:

*   **`fw_files`**: A list of tuples, where each tuple specifies a firmware binary to be deployed.
    *   Format: `('path/to/firmware.bin', 'FirmwareBaseName')`
    *   Example: `('Firmware_As_Built/UserApp_v1.2.3.bin', 'UserApp_v1.2.3')`
*   **`pk_files`**: A list of tuples, where each tuple specifies a public key to be used for encryption.
    *   Format: `('path/to/public_key.bin', 'BatchName')`
    *   Example: `('Public_Keys/pub_key_b2.bin', 'B2')`

The script will iterate through each firmware file in `fw_files` and encrypt it with each public key in `pk_files`.

### Running the Script

To execute the deployment process, run the script from your terminal. Navigate to the `/Deploy` folder of the firmware distribution (where the script and its associated folders `Firmware_As_Built`, `Public_Keys` are located):

```bash
python deploy_firmware.py
```

The script will print progress information for each encryption operation to the console.

### Output Files

*   The encrypted firmware update files are saved in the `Firmware_To_Deploy/` directory.
*   The output files are named using a combination of the batch name and the firmware base name, with an `.sfb` extension.
    *   Format: `{BatchName}_{FirmwareBaseName}.sfb`
    *   Example: `B2_UserApp_v1.2.3.sfb`

## Deployment Workflow (Step-by-Step)

Follow these steps to generate encrypted firmware update files:

1.  **Ensure Prerequisites:** Verify that Python 3 and the `coincurve` and `ecdh_aesgcm` libraries are installed.
2.  **Obtain Firmware:** Compile your FlySight firmware. This typically results in a `UserApp.bin` file located in the `/Release` folder of your firmware project.
3.  **Place Firmware:** Copy the `UserApp.bin` (or your specifically named firmware binary) from the `/Release` folder into the `Deploy/Firmware_As_Built/` directory. For example, you might rename it to `UserApp_vX.Y.Z.bin`.
4.  **Verify Public Keys:** Ensure that the public key files for your target batches (e.g., `pub_key_b2.bin`, `pub_key_b3.bin`) are present in the `Deploy/Public_Keys/` directory.
5.  **Configure Script:**
    *   Open `Deploy/deploy_firmware.py` in a text editor.
    *   Locate the `fw_files` list near the end of the script. Modify it to point to the firmware binary you placed in `Firmware_As_Built/` and assign it a descriptive base name.
        ```python
        fw_files = [
            ('Firmware_As_Built/UserApp_vX.Y.Z.bin', 'UserApp_vX.Y.Z'),
            # Add other firmware files if needed
        ]
        ```
    *   Locate the `pk_files` list. Modify it to include the public keys and batch names for all target batches.
        ```python
        pk_files = [
            ('Public_Keys/pub_key_b2.bin', 'B2'),
            ('Public_Keys/pub_key_b3.bin', 'B3'),
            # Add other public keys if needed
        ]
        ```
    *   Save the changes to `deploy_firmware.py`.
6.  **Run Deployment Script:** Open your terminal, navigate to the `/Deploy` directory within your firmware distribution, and run:
    ```bash
    python deploy_firmware.py
    ```
7.  **Collect Output Files:** After the script completes, find the generated `.sfb` files in the `Deploy/Firmware_To_Deploy/` directory. For the example configuration above, you would find files like `B2_UserApp_vX.Y.Z.sfb` and `B3_UserApp_vX.Y.Z.sfb`.
8.  **Distribute:** These `.sfb` files are now ready for distribution and use in the FlySight firmware update process for the corresponding device batches.
