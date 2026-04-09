# ==============================================================================
# pc_sender.py — PC-side BLE image sender
# Requires: Python 3.8+, bleak (pip install bleak)
#
# Usage:
#   python pc_sender.py fan_image.bin
#   python pc_sender.py fan_image.bin --device "POV-Fan"
#   python pc_sender.py fan_image.bin --chunk-size 200
#
# The script:
#   1. Scans for a BLE peripheral advertising as POV-Fan (or specified name).
#   2. Connects and negotiates the best available MTU.
#   3. Sends TRANSFER_MAGIC followed by the raw 15360-byte image in chunks.
#   4. Waits for the Pico to reply "OK" on the TX characteristic.
# ==============================================================================

import asyncio
import argparse
import sys
import struct
import time
from pathlib import Path

try:
    from bleak import BleakScanner, BleakClient
    from bleak.backends.characteristic import BleakGATTCharacteristic
except ImportError:
    print("ERROR: 'bleak' is not installed.  Run: pip install bleak")
    sys.exit(1)

from config import (
    NUS_SERVICE_UUID, NUS_TX_UUID, NUS_RX_UUID,
    IMAGE_SIZE, TRANSFER_MAGIC, BLE_CHUNK_SIZE,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def load_image(path: str) -> bytes:
    data = Path(path).read_bytes()
    if len(data) != IMAGE_SIZE:
        raise ValueError(
            f"Image file is {len(data)} bytes; expected {IMAGE_SIZE}.\n"
            "Run image_converter.py first to generate the correct binary."
        )
    return data


async def find_device(target_name: str, timeout: float = 10.0):
    """Scan and return the BLEDevice matching target_name."""
    print(f"[SCAN] Looking for '{target_name}' ({timeout}s timeout)…")
    devices = await BleakScanner.discover(timeout=timeout, return_adv=False)
    for d in devices:
        if d.name and target_name.lower() in d.name.lower():
            print(f"[SCAN] Found: {d.name}  addr={d.address}")
            return d
    return None


# ---------------------------------------------------------------------------
# Transfer
# ---------------------------------------------------------------------------

async def send_image(device, image_data: bytes, chunk_size: int,
                     inter_chunk_ms: float = 20.0) -> bool:
    """
    Connect to *device* and stream *image_data* over NUS.
    Returns True on success.

    Flow control strategy
    ---------------------
    The Pico W's BLE stack has a small internal ATT write queue (~4 slots at
    most).  Firing chunks with response=False back-to-back at USB speeds
    overflows that queue and causes the Pico to drop the connection.

    Two complementary guards:
      1. response=True  — Write With Response.  Each chunk waits for an ATT
                          acknowledgement from the Pico before the next is sent.
                          This is the primary throttle.
      2. inter_chunk_ms — Additional async sleep between chunks.  Gives the
                          Pico's MicroPython IRQ handler time to drain the GATT
                          write callback and copy bytes into background_buffer
                          before the next chunk arrives.  20 ms is conservative
                          but safe; lower it to 10 ms once verified stable.
    """
    ack_event = asyncio.Event()

    def on_notify(_char: BleakGATTCharacteristic, data: bytearray):
        msg = bytes(data).strip()
        print(f"[RX ] Pico replied: {msg}")
        if msg == b"OK":
            ack_event.set()

    async with BleakClient(device) as client:
        print(f"[BLE] Connected  MTU={client.mtu_size}")

        # Subscribe to TX characteristic for acknowledgement
        await client.start_notify(NUS_TX_UUID, on_notify)

        # ── Build payload: magic header + image data ────────────────────────
        payload = TRANSFER_MAGIC + image_data
        total   = len(payload)
        chunks  = (total + chunk_size - 1) // chunk_size

        print(f"[TX ] Sending {total} bytes in {chunks} chunks of {chunk_size}…")
        print(f"[TX ] Flow control: Write-With-Response + {inter_chunk_ms}ms inter-chunk delay")
        sent    = 0
        t_start = time.monotonic()

        while sent < total:
            end   = min(sent + chunk_size, total)
            chunk = payload[sent:end]

            # response=True: blocks until Pico sends ATT acknowledgement.
            # This is the primary flow-control mechanism.
            await client.write_gatt_char(NUS_RX_UUID, chunk, response=True)
            sent += len(chunk)

            # Secondary throttle: yield control so the Pico's BLE stack can
            # process the write callback before the next chunk arrives.
            await asyncio.sleep(inter_chunk_ms / 1000.0)

            # Progress every ~10 chunks
            if (sent // chunk_size) % 10 == 0 or sent >= total:
                pct = sent * 100 // total
                print(f"[TX ] {pct:3d}%  ({sent}/{total} bytes)")

        elapsed = time.monotonic() - t_start
        kbps    = (total / elapsed / 1024) if elapsed > 0 else 0
        print(f"[TX ] Transfer complete in {elapsed:.2f}s  ({kbps:.1f} kB/s)")

        # ── Wait for ACK ────────────────────────────────────────────────────
        print("[TX ] Waiting for Pico acknowledgement…")
        try:
            await asyncio.wait_for(ack_event.wait(), timeout=5.0)
            print("[TX ] ✓  Image accepted by Pico")
            return True
        except asyncio.TimeoutError:
            print("[TX ] ✗  Timeout waiting for ACK — image may still be valid")
            return False


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

async def main():
    parser = argparse.ArgumentParser(
        description="Send a POV fan image to the Pico W over BLE"
    )
    parser.add_argument("image", help="Path to fan_image.bin (output of image_converter.py)")
    parser.add_argument("--device", default="POV-Fan",
                        help="BLE device name to scan for (default: POV-Fan)")
    parser.add_argument("--chunk-size", type=int, default=BLE_CHUNK_SIZE,
                        help=f"Bytes per BLE write (default: {BLE_CHUNK_SIZE})")
    parser.add_argument("--inter-chunk-ms", type=float, default=20.0,
                        help="Delay between chunks in ms (default: 20). Lower to 10 once stable.")
    parser.add_argument("--scan-timeout", type=float, default=10.0,
                        help="BLE scan timeout in seconds (default: 10)")
    args = parser.parse_args()

    # Load and validate the binary
    try:
        image_data = load_image(args.image)
        print(f"[IMG] Loaded {len(image_data)} bytes from '{args.image}'")
    except (FileNotFoundError, ValueError) as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    # Scan
    device = await find_device(args.device, timeout=args.scan_timeout)
    if device is None:
        print(f"ERROR: Device '{args.device}' not found.  Is the Pico running and advertising?")
        sys.exit(1)

    # Send
    ok = await send_image(device, image_data,
                          chunk_size=args.chunk_size,
                          inter_chunk_ms=args.inter_chunk_ms)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    asyncio.run(main())
