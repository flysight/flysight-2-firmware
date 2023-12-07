import os
import argparse
import asyncio
from bleak import BleakScanner, BleakClient
from struct import unpack
from datetime import datetime
from tqdm import tqdm

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

async def mkdir(address, directory_name):
    async with BleakClient(address, adapter=ble_adapter) as client:
        await client.write_gatt_char(CRS_RX_UUID, b'\x04' + directory_name.encode())

async def create_file(address, file_name):
    async with BleakClient(address, adapter=ble_adapter) as client:
        await client.write_gatt_char(CRS_RX_UUID, b'\x00' + file_name.encode())

async def delete_file(address, file_name):
    async with BleakClient(address, adapter=ble_adapter) as client:
        await client.write_gatt_char(CRS_RX_UUID, b'\x01' + file_name.encode())

async def write_file(address, local_filename, remote_filename):
    async with BleakClient(address, adapter=ble_adapter) as client:
        with open(local_filename, "rb") as f:
            data = f.read()

        await client.write_gatt_char(CRS_RX_UUID, b'\x03' + remote_filename.encode())

        with tqdm(desc="Sending Bytes", unit="B", unit_scale=True) as pbar:
            for i in range(0, len(data), 243):
                chunk = data[i:i+243]
                await client.write_gatt_char(CRS_RX_UUID, b'\x10' + chunk)
                pbar.update(len(chunk))

        # Sending packet to signal completion
        await client.write_gatt_char(CRS_RX_UUID, b'\x10')

async def read_file(address, offset, stride, remote_filename, local_filename, timeout=1):
    with tqdm(desc="Receiving Bytes", unit="B", unit_scale=True) as pbar:
        async with BleakClient(address, adapter=ble_adapter) as client:
            file_data = bytearray()
            transfer_complete = asyncio.Event()
            packet_received = asyncio.Event()

            def file_notification_handler(sender, data):
                nonlocal file_data
                if data[0] == 0x10:
                    if len(data) > 1:
                        file_data.extend(data[1:])
                        pbar.update(len(data)-1)  # Update the progress bar with the number of bytes received
                    else:
                        # Signal that the transfer is complete
                        transfer_complete.set()
                    packet_received.set()

            await client.start_notify(CRS_TX_UUID, file_notification_handler)
            offset_bytes = offset.to_bytes(4, byteorder='little')
            stride_bytes = stride.to_bytes(4, byteorder='little')
            await client.write_gatt_char(CRS_RX_UUID, b'\x02' + offset_bytes + stride_bytes + remote_filename.encode())

            try:
                while not transfer_complete.is_set():
                    packet_received.clear()
                    await asyncio.wait_for(packet_received.wait(), 5)  # 5-second timeout for each packet
            except asyncio.TimeoutError:
                print(f"Timeout: No data received for {timeout} seconds.")
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

async def notification_handler(sender, data):
    if data[0] == 0x11:
        parsed_info = parse_filinfo(data[1:])
        if parsed_info:
            print(parsed_info)

async def list_directory(address, directory):
    async with BleakClient(address, adapter=ble_adapter) as client:
        await client.start_notify(CRS_TX_UUID, notification_handler)
        await client.write_gatt_char(CRS_RX_UUID, b'\x05' + directory.encode())
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
    args = parser.parse_args()

    if args.list:
        asyncio.run(list_devices())
    elif args.address and args.dir:
        asyncio.run(list_directory(args.address, args.dir))
    elif args.address and args.read:
        offset, stride, remote_filename, *local_filename = args.read
        local_filename = local_filename[0] if local_filename else os.path.basename(remote_filename)
        asyncio.run(read_file(args.address, int(offset), int(stride), remote_filename, local_filename))
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

