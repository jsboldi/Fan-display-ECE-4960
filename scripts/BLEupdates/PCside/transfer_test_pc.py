# transfer_test_pc.py

import asyncio
from bleak import BleakClient, BleakScanner

DEVICE_NAME  = "FanDisplay"
UART_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
CHUNK_SIZE   = 243
IMAGE_SIZE   = 21600

CMD_IMAGE_START = bytes([0x04])
CMD_IMAGE_END   = bytes([0x05])
CMD_IMAGE_DATA  = 0xFF

def make_test_binary():
    data = bytearray()
    for s in range(90):
        for l in range(80):
            if s % 2 == 0:
                data.extend([220, 100, 30])
            else:
                data.extend([0, 0, 0])
    print(f"Test binary: {len(data)} bytes")
    return bytes(data)

async def main():
    print(f"Scanning for '{DEVICE_NAME}'...")
    devices = await BleakScanner.discover(timeout=8.0)
    address = None
    for d in devices:
        if d.name == DEVICE_NAME:
            address = d.address
            print(f"Found at {address}")
            break

    if not address:
        print("Device not found — is Pico W running?")
        return

    data = make_test_binary()

    async with BleakClient(address) as client:
        print("Connected")

        await client.write_gatt_char(UART_RX_UUID, CMD_IMAGE_START, response=False)
        await asyncio.sleep(0.2)   # longer delay after start

        total = (len(data) + CHUNK_SIZE - 1) // CHUNK_SIZE
        for i in range(total):
            start = i * CHUNK_SIZE
            end   = min(start + CHUNK_SIZE, len(data))
            chunk = bytes([CMD_IMAGE_DATA]) + data[start:end]
            await client.write_gatt_char(UART_RX_UUID, chunk, response=False)
            await asyncio.sleep(0.05)   # was 0.02, now 0.05
            print(f"Chunk {i+1}/{total}")

        await asyncio.sleep(0.2)   # longer delay before end
        await client.write_gatt_char(UART_RX_UUID, CMD_IMAGE_END, response=False)
        await asyncio.sleep(0.5)
        print("Done")

asyncio.run(main())
