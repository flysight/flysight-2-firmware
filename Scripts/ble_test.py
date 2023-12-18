import os
import argparse
import asyncio
from bleak import BleakScanner, BleakClient
from struct import unpack
from datetime import datetime
from tqdm import tqdm
import random

# UUIDs for the CRS characteristics
CRS_RX_UUID = "00000002-8e22-4541-9d4c-21edae82ed19"
CRS_TX_UUID = "00000001-8e22-4541-9d4c-21edae82ed19"

# File attribute definitions
attrib_description = {
    0: 'r',
    1: 'h',
    2: 's',
    3: 'a',
    4: 'd'
}

# Bluetooth adapter
#ble_adapter = 'hci0'
ble_adapter = None

# Flow control parameters
WINDOW_LENGTH = 8
FRAME_LENGTH = 242
TX_TIMEOUT = 1
RX_TIMEOUT = 1

async def mkdir(address, directory_name):
    async with BleakClient(address, adapter=ble_adapter) as client:
        await client.write_gatt_char(CRS_RX_UUID, b'\x04' + directory_name.encode(), response=False)

async def create_file(address, file_name):
    async with BleakClient(address, adapter=ble_adapter) as client:
        await client.write_gatt_char(CRS_RX_UUID, b'\x00' + file_name.encode(), response=False)

async def delete_file(address, file_name):
    async with BleakClient(address, adapter=ble_adapter) as client:
        await client.write_gatt_char(CRS_RX_UUID, b'\x01' + file_name.encode(), response=False)

async def write_file(address, local_filename, remote_filename):
    with tqdm(desc="Sending Bytes", unit="B", unit_scale=True) as pbar:
        async with BleakClient(address, adapter=ble_adapter) as client:
            with open(local_filename, "rb") as f:
                data = f.read()

            next_packet_num = 0
            next_ack_num = 0
            last_packet_num = -1
            next_ack_bytes = 0
            ack_received = asyncio.Event()

            def file_notification_handler(sender, data):
                nonlocal next_ack_num, next_ack_bytes, ack_received
                if data[0] == 0x12:
                    ack_num = int.from_bytes(data[1:2], byteorder='little', signed=False)
                    if ack_num == (next_ack_num & 0xff):
                        next_ack_num = next_ack_num + 1
                        ack_received.set()
                        pbar.update(next_ack_bytes)  # Update progress bar based on acknowledged bytes

            await client.start_notify(CRS_TX_UUID, file_notification_handler)
            await client.write_gatt_char(CRS_RX_UUID, b'\x03' + remote_filename.encode())

            while next_ack_num != last_packet_num:
                while (next_packet_num < next_ack_num + WINDOW_LENGTH) and (next_packet_num != last_packet_num):
                    # Read from file
                    i = next_packet_num * FRAME_LENGTH
                    next_packet_num_bytes = (next_packet_num & 0xff).to_bytes(1, byteorder='little')
                    if i < len(data):
                        chunk = data[i:i+FRAME_LENGTH]
                        next_ack_bytes = len(chunk)
                        await client.write_gatt_char(CRS_RX_UUID, b'\x10' + next_packet_num_bytes + chunk, response=False)
                    else:
                        next_ack_bytes = 0
                        await client.write_gatt_char(CRS_RX_UUID, b'\x10' + next_packet_num_bytes, response=False)
                        last_packet_num = next_packet_num + 1;

                    next_packet_num += 1

                try:
                    await asyncio.wait_for(ack_received.wait(), TX_TIMEOUT)
                    ack_received.clear()
                except asyncio.TimeoutError:
                    next_packet_num = next_ack_num

            await client.stop_notify(CRS_TX_UUID)

async def read_file(address, offset, stride, remote_filename, local_filename, test_mode=False):
    with tqdm(desc="Receiving Bytes", unit="B", unit_scale=True) as pbar:
        async with BleakClient(address, adapter=ble_adapter) as client:
            file_data = bytearray()
            transfer_complete = asyncio.Event()
            packet_received = asyncio.Event()
            next_packet_num = 0

            async def file_notification_handler(sender, data):
                nonlocal file_data, packet_received, next_packet_num
                if data[0] == 0x10:
                    packet_num = int.from_bytes(data[1:2], byteorder='little')
                    if (packet_num == (next_packet_num & 0xff)) and (not test_mode or random.random() >= 0.3):
                        if len(data) > 2:
                            file_data.extend(data[2:])
                            pbar.update(len(data)-2)  # Update the progress bar with the number of bytes received
                        else:
                            transfer_complete.set()

                        next_packet_num += 1
                        ack_packet = b'\x12' + packet_num.to_bytes(1, byteorder='little')
                        await client.write_gatt_char(CRS_RX_UUID, ack_packet, response=False)
                        packet_received.set()

            await client.start_notify(CRS_TX_UUID, file_notification_handler)
            offset_bytes = offset.to_bytes(4, byteorder='little')
            stride_bytes = stride.to_bytes(4, byteorder='little')
            await client.write_gatt_char(CRS_RX_UUID, b'\x02' + offset_bytes + stride_bytes + remote_filename.encode(), response=False)

            try:
                while not transfer_complete.is_set():
                    packet_received.clear()
                    await asyncio.wait_for(packet_received.wait(), RX_TIMEOUT)  # timeout for each packet
            except asyncio.TimeoutError:
                print(f"Timeout: No data received for {RX_TIMEOUT} seconds.")
                return

            await client.stop_notify(CRS_TX_UUID)

            # Process or save the file data
            with open(local_filename, "wb") as f:
                f.write(file_data)

def get_attrib_text(fattrib):
    descriptions = []
    for bit in attrib_description:
        if fattrib & (1 << bit):
            descriptions.append(attrib_description[bit])
        else:
            descriptions.append('-')
    return ''.join(descriptions)

def parse_filinfo(data):
    fsize, fdate, ftime, fattrib, fname = unpack("<IHHB13s", data)
    
    # Extracting filename    
    fname = fname.split(b'\0', 1)[0].decode('ascii')

    # Check if the filename is empty
    if not fname:
    	return None

    # Extracting date fields
    year = ((fdate >> 9) & 0b1111111) + 1980  # Adjust for the year starting from 1980
    month = (fdate >> 5) & 0b1111
    day = fdate & 0b11111

    # Extracting time fields
    hour = (ftime >> 11) & 0b11111
    minute = (ftime >> 5) & 0b111111
    second = (ftime & 0b11111) * 2  # Multiply by 2 to get the actual seconds

    # Extracting file attributes
    attrib_text = get_attrib_text(fattrib)

    return f"{fsize} {year}-{month}-{day} {hour}:{minute}:{second} {attrib_text} {fname}"

async def list_directory(address, directory):
    async with BleakClient(address, adapter=ble_adapter) as client:
        async def dir_notification_handler(sender, data):
            if data[0] == 0x11:
                parsed_info = parse_filinfo(data[1:])
                if parsed_info:
                    print(parsed_info)

        await client.start_notify(CRS_TX_UUID, dir_notification_handler)
        await client.write_gatt_char(CRS_RX_UUID, b'\x05' + directory.encode(), response=False)
        await asyncio.sleep(5)  # Wait for notifications
        await client.stop_notify(CRS_TX_UUID)

async def list_devices():
    devices = await BleakScanner.discover()
    for device in devices:
        print(f"Device {device.name} found with address {device.address}")

def main():
    parser = argparse.ArgumentParser(description="BLE Utility")
    
    parser.add_argument('--list', action='store_true', help='List all available BLE devices.')
    parser.add_argument('--address', type=str, metavar='ADDRESS', help='Specify the BLE device address.')
    parser.add_argument('--dir', type=str, metavar='DIRECTORY', help='List contents of the specified directory.')
    parser.add_argument('--read', nargs='+', help='Read a file from the device. Provide the offset, (stride-1), remote file path, and an optional local file path.')
    parser.add_argument('--write', nargs=2, metavar=('LOCAL_FILE', 'REMOTE_FILE'), help='Write a file to the device. Provide the local file path, and remote file path.')
    parser.add_argument('--create', type=str, metavar='FILE_NAME', help='Create a new file on the device with the specified name.')
    parser.add_argument('--delete', type=str, metavar='FILE_NAME', help='Delete the specified file from the device.')
    parser.add_argument('--mkdir', type=str, metavar='DIRECTORY_NAME', help='Create a new directory on the device with the specified name.')
    parser.add_argument('--test-mode', action='store_true', help='Enable test mode with packet dropping')
    args = parser.parse_args()

    if args.list:
        asyncio.run(list_devices())
    elif args.address and args.dir:
        asyncio.run(list_directory(args.address, args.dir))
    elif args.address and args.read:
        offset, stride, remote_filename, *local_filename = args.read
        local_filename = local_filename[0] if local_filename else os.path.basename(remote_filename)
        asyncio.run(read_file(args.address, int(offset), int(stride), remote_filename, local_filename, args.test_mode))
    elif args.address and args.write:
        local_filename, remote_filename = args.write
        asyncio.run(write_file(args.address, local_filename, remote_filename))
    elif args.address and args.create:
        asyncio.run(create_file(args.address, args.create))
    elif args.address and args.delete:
        asyncio.run(delete_file(args.address, args.delete))
    elif args.address and args.mkdir:
        asyncio.run(mkdir(args.address, args.mkdir))
    else:
        print("Invalid arguments. Use --help for usage information.")

if __name__ == "__main__":
    main()

