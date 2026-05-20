#!/usr/bin/env python3
"""Convert Config Generator hex output to a C header for firmware embedding.

Usage: python3 hex_to_c_header.py <input.txt> <output.h>

The input file has one hex-encoded BLE command per line.
The output is a C header with byte arrays that the firmware sends to the glasses.

Commands starting with FFD0 (cfgWrite) and FFD2 (cfgSet) are skipped —
the firmware handles those in its own state machine.

Large commands (>240 bytes) are split into BLE-MTU-sized chunks for
transmission via aci_gatt_write_without_resp.
"""

import sys
import os

MAX_BLE_CHUNK = 245  # already pre-chunked by gen_ble_config.py

def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <input.txt> <output.h>")
        sys.exit(1)

    input_path = sys.argv[1]
    output_path = sys.argv[2]

    with open(input_path) as f:
        raw_lines = [line.strip() for line in f if line.strip()]

    # Filter out cfgWrite (FFD0) and cfgSet (FFD2) commands
    filtered = []
    for line in raw_lines:
        cmd_id = line[2:4].upper()
        if cmd_id in ('D0', 'D2'):
            continue
        filtered.append(bytes.fromhex(line))

    # Split large commands into BLE-MTU-sized chunks
    commands = []
    for cmd in filtered:
        if len(cmd) <= MAX_BLE_CHUNK:
            commands.append(cmd)
        else:
            off = 0
            while off < len(cmd):
                sz = min(len(cmd) - off, MAX_BLE_CHUNK)
                commands.append(cmd[off:off+sz])
                off += sz

    guard = os.path.basename(output_path).upper().replace('.', '_').replace('-', '_')

    with open(output_path, 'w') as out:
        out.write(f'#ifndef {guard}\n')
        out.write(f'#define {guard}\n\n')
        out.write('#include <stdint.h>\n\n')
        out.write('#define AL_CONFIG_VERSION 4\n')
        out.write(f'#define AL_CONFIG_CMD_COUNT {len(commands)}\n\n')

        # Lengths array
        out.write('static const uint16_t al_config_cmd_lengths[AL_CONFIG_CMD_COUNT] = {\n    ')
        out.write(', '.join(str(len(cmd)) for cmd in commands))
        out.write('\n};\n\n')

        # Offsets array
        out.write('static const uint16_t al_config_cmd_offsets[AL_CONFIG_CMD_COUNT] = {\n    ')
        offset = 0
        offsets = []
        for cmd in commands:
            offsets.append(str(offset))
            offset += len(cmd)
        out.write(', '.join(offsets))
        out.write('\n};\n\n')

        # Data blob
        total_size = sum(len(cmd) for cmd in commands)
        out.write(f'// Total config data: {total_size} bytes, {len(commands)} commands\n')
        out.write(f'// (large protocol frames pre-split into {MAX_BLE_CHUNK}-byte BLE chunks)\n')
        out.write(f'static const uint8_t al_config_cmd_data[{total_size}] = {{\n')
        for i, cmd in enumerate(commands):
            cmd_type = "font" if cmd[0] == 0xFF and len(cmd) > 1 and cmd[1] == 0x51 else \
                       "image" if cmd[0] == 0xFF and len(cmd) > 1 and cmd[1] == 0x41 else \
                       "chunk"
            out.write(f'    // [{i}] {cmd_type} ({len(cmd)} bytes)\n    ')
            out.write(', '.join(f'0x{b:02X}' for b in cmd))
            out.write(',\n')
        out.write('};\n\n')
        out.write(f'#endif // {guard}\n')

    max_chunk = max(len(cmd) for cmd in commands)
    print(f"Generated {output_path}:")
    print(f"  {len(filtered)} protocol frames → {len(commands)} BLE chunks")
    print(f"  {total_size} bytes total, max chunk {max_chunk} bytes")

if __name__ == '__main__':
    main()
