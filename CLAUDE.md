# FlySight 2 Firmware

## Project Overview

FlySight 2 is a high-precision GPS data logger for skydiving and BASE jumping by Bionic Avionics Inc. The firmware runs on an STM32WB5MMGHx dual-core MCU (Cortex-M4 application + Cortex-M0+ BLE radio). It provides real-time audio feedback during freefall, multi-sensor data logging, BLE connectivity, USB mass storage, and an ActiveLook HUD display on Engo smart glasses.

### Key Features
- GNSS/GPS tracking at up to 25 Hz (u-blox MAX-M8)
- 6-axis IMU (LSM6DSO), barometer, magnetometer, humidity sensor
- Real-time audio tone/speech feedback based on flight parameters
- Data logging to SD card via FatFS
- BLE connectivity (STM32 WPAN stack) for phone app and ActiveLook glasses
- USB mass storage for file transfer
- ActiveLook HUD display: 2x2 grid layout with icons, B612 aviation font

### Build System
- **IDE**: STM32CubeIDE (Eclipse-based, ARM GCC toolchain)
- **MCU**: STM32WB5MMGHx (dual-core M4+M0+, BLE 5.x)
- **No Makefile/CMake** -- build is defined in `.cproject`/`.project` files
- **Pre-build**: `prebuild.sh` generates `version.h` from git tag
- **Post-build**: generates `UserApp.bin` and copies to `Deploy/Firmware_As_Built/`

### Branch Structure
- `master` -- stable releases
- `develop` -- active development (ActiveLook integration, new features)

## Rules for Claude Code

### Language Server Protocol (LSP)
- **ALWAYS use LSP** (documentSymbol, hover, goToDefinition, findReferences, callHierarchy) when analyzing or reading C/C++ source files
- Prefer LSP over Read tool or grep for understanding code structure
- Only fall back to Read for non-code files (configs, scripts, docs) or when LSP can't resolve symbols due to missing STM32 HAL headers
- Use LSP for Python files as well when available

### ActiveLook Development
- **ALWAYS consult the ActiveLook API documentation** before any implementation: https://github.com/ActiveLook/Activelook-API-Documentation/blob/main/ActiveLook_API.md
- **Never guess coordinates or protocol details** -- read the documentation first
- ActiveLook display coordinate system: (0,0) = bottom-right visually (optical mirror flips both axes)
  - High X = visually LEFT, Low X = visually RIGHT
  - High Y = visually TOP, Low Y = visually BOTTOM
  - Rotation 4 = normal readable text with centered anchor
- Page coordinates are NOT mirrored (standard: x=0 left, higher y = higher on screen)
- Layout-internal coordinates (extra sub-commands) ARE mirrored
- Images must be rotated 180 degrees before upload to compensate for optical mirror
- Config names use NUL-terminated strings (NOT padded to 12 bytes) when used with Config-Generator, but FlySight firmware pads to 12 bytes
- `cfgWrite` requires the correct password if config already exists (FlySight uses password `0x01020304`)
- `cfgSet` must be called to activate a config before its fonts/images are available
- Font upload uses `fontSave` with header `[id, size_hi, size_lo]` then data chunks
- Image upload uses `imgSave` with header `[id, size_u32, width_u16, format]` then data chunks
- Maximum BLE write payload = MTU - 3 (currently 253 bytes with MTU=256)
- Reference the ActiveLook Visual Assets for pre-loaded icons: https://github.com/ActiveLook/Activelook-Visual-Assets
- Reference the ActiveLook Config-Generator for font/image conversion: https://github.com/ActiveLook/Config-Generator

### STM32/Embedded Best Practices
- Follow existing code patterns -- the project uses STM32 HAL, WPAN middleware, and a cooperative scheduler (UTIL_SEQ), not FreeRTOS
- One BLE command per scheduler task invocation -- never send multiple BLE writes in a tight loop
- Use `UTIL_SEQ_SetTask()` to schedule the next step, not blocking delays
- Respect the watchdog timer (IWDG, 3-second timeout)
- All sensor I/O is DMA-based and interrupt-driven -- never block in ISR context
- Stack is limited (0x400 bytes) -- avoid large local arrays; use static buffers
- Flash is limited to 454 KB for the application (bootloader occupies lower flash)
- RAM1 is 256 KB, RAM_SHARED is 10 KB (BLE radio)
- Use `static` for file-scope variables and functions
- The project uses `newlib-nano` with float printf enabled (`-u _printf_float`)

### C Language Style
- Follow existing project conventions (Bionic Avionics coding style)
- Prefix all public functions with `FS_` (FlySight namespace)
- Use `typedef enum` and `typedef struct` with `_t` suffix
- State machines use function pointer tables (see `mode.c`, `crs.c`, `activelook.c`)
- Ring buffers for producer/consumer patterns (see `log.c`, `sensor.c`)
- No dynamic memory allocation (`malloc`/`free`) -- use static arrays
- Error handling: log events via `FS_Log_WriteEvent()`, set LED color for health status
- Configuration is file-based (text config on SD card, parsed at startup)

### Code Quality
- Do not add unnecessary comments -- the code should be self-documenting
- Do not add features beyond what was requested
- Do not guess at protocol details -- verify against documentation or test empirically
- When modifying ActiveLook layout code, make one change at a time and test
- The clangd LSP will show false errors for STM32 HAL symbols (can't resolve `main.h`) -- these are expected and don't affect the actual STM32CubeIDE build

### Deployment
- Build with STM32CubeIDE (Debug or Release configuration)
- Deploy via encrypted `.sfb` firmware update files (see `Deploy/deploy_firmware.py`)
- Each device batch has a specific public key -- check `flysight.txt` on device for `Pubkey_X`
- ActiveLook config (fonts/images) uploaded separately via `Config-Generator/upload_ble.py`

### Key Directories
- `FlySight/` -- main application code (sensors, audio, logging, modes, ActiveLook)
- `Core/Src/` -- STM32 HAL initialization, main loop, interrupt handlers
- `STM32_WPAN/App/` -- BLE application layer (app_ble.c, custom services, ActiveLook client)
- `Deploy/` -- firmware deployment scripts and tools
- `Config-Generator/` (external, at `~/Projects/Config-Generator/`) -- ActiveLook font/image preparation tool
