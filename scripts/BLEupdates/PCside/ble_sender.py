# ble_sender.py
# PC Side - sends image and commands to Pico W over BLE
# Uses the same Nordic UART Service UUIDs as the tutorial
# Install: pip install bleak

import asyncio
from bleak import BleakClient, BleakScanner
from config import (
    BLE_DEVICE_NAME, BLE_CHUNK_SIZE, IMAGE_SIZE,
    CMD_LED_ON, CMD_LED_OFF, CMD_SET_COLOR,
    CMD_IMAGE_START, CMD_IMAGE_END, CMD_SET_BRIGHTNESS
)

# Nordic UART Service UUIDs — must match ble_receiver.py
UART_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # PC writes to this
UART_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # PC reads from this

# ── Find Device ────────────────────────────────────────────
async def find_device():
    print(f"[BLE] Scanning for '{BLE_DEVICE_NAME}'...")
    devices = await BleakScanner.discover(timeout=5.0)
    for d in devices:
        if d.name == BLE_DEVICE_NAME:
            print(f"[BLE] Found at {d.address}")
            return d.address
    print("[BLE] Not found. Is the fan powered on?")
    return None

# ── Send Raw Bytes ─────────────────────────────────────────
async def send(client, data):
    await client.write_gatt_char(UART_RX_UUID, data, response=False)
    await asyncio.sleep(0.02)

# ── Control Commands ───────────────────────────────────────
async def cmd_led_on(client):
    print("[CMD] LED ON")
    await send(client, bytes([CMD_LED_ON]))

async def cmd_led_off(client):
    print("[CMD] LED OFF")
    await send(client, bytes([CMD_LED_OFF]))

async def cmd_set_color(client, r, g, b):
    print(f"[CMD] SET COLOR ({r},{g},{b})")
    await send(client, bytes([CMD_SET_COLOR, r, g, b]))

async def cmd_set_brightness(client, level):
    # level: 0.0 - 1.0
    val = int(level * 255)
    print(f"[CMD] SET BRIGHTNESS {level:.2f}")
    await send(client, bytes([CMD_SET_BRIGHTNESS, val]))

# ── Send Image ─────────────────────────────────────────────
async def send_image(client, bin_path):
    with open(bin_path, "rb") as f:
        data = f.read()

    if len(data) != IMAGE_SIZE:
        print(f"[ERR] Wrong size: expected {IMAGE_SIZE}, got {len(data)}")
        print("[ERR] Run image_converter.py first")
        return False

    print(f"[IMG] Sending {bin_path} ({len(data)} bytes)")

    # Signal start
    await send(client, bytes([CMD_IMAGE_START]))
    await asyncio.sleep(0.1)

    # Send in chunks
    total = (len(data) + BLE_CHUNK_SIZE - 1) // BLE_CHUNK_SIZE
    for i in range(total):
        start = i * BLE_CHUNK_SIZE
        end   = min(start + BLE_CHUNK_SIZE, len(data))
        await send(client, data[start:end])
        print(f"[IMG] Chunk {i+1}/{total}")

    # Signal end
    await asyncio.sleep(0.1)
    await send(client, bytes([CMD_IMAGE_END]))
    print("[IMG] Transfer complete")
    return True

# ── Main ───────────────────────────────────────────────────
async def main():
    address = await find_device()
    if not address:
        return

    async with BleakClient(address) as client:
        print(f"[BLE] Connected to {address}")

        # ── Use whichever you need ─────────────────────────
        await send_image(client, "tiger_paw.bin")
        # await cmd_led_on(client)
        # await cmd_led_off(client)
        # await cmd_set_color(client, 255, 100, 0)
        # await cmd_set_brightness(client, 0.5)

if __name__ == "__main__":
    asyncio.run(main())
