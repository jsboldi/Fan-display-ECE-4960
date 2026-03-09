# ble_receiver.py
# Controller B - BLE Receiver
# Runs on Pico W - based on the same pattern as the PiCockpit tutorial
# Requires ble_advertising.py to be saved on the Pico W (from tutorial)

import bluetooth
import struct
import time
from ble_advertising import advertising_payload
from micropython import const
from config import (
    CMD_LED_ON, CMD_LED_OFF, CMD_SET_COLOR,
    CMD_IMAGE_START, CMD_IMAGE_END, CMD_SET_BRIGHTNESS,
    IMAGE_SIZE, BLE_DEVICE_NAME, BLE_CHUNK_SIZE
)

# ── IRQ Constants (same as tutorial) ──────────────────────
_IRQ_CENTRAL_CONNECT    = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE        = const(3)

# ── Flags ──────────────────────────────────────────────────
_FLAG_READ              = const(0x0002)
_FLAG_WRITE_NO_RESPONSE = const(0x0004)
_FLAG_WRITE             = const(0x0008)
_FLAG_NOTIFY            = const(0x0010)

# ── Nordic UART Service UUIDs (same as tutorial) ───────────
_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX   = (bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"), _FLAG_READ | _FLAG_NOTIFY,)
_UART_RX   = (bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"), _FLAG_WRITE | _FLAG_WRITE_NO_RESPONSE,)
_UART_SERVICE = (_UART_UUID, (_UART_TX, _UART_RX,),)


class BLEReceiver:

    def __init__(self, led_driver, image_store):
        self.led   = led_driver
        self.store = image_store
        self._ble  = bluetooth.BLE()
        self._ble.active(True)
        self._ble.irq(self._irq)

        # Register services — same pattern as tutorial
        ((self._handle_tx, self._handle_rx),) = self._ble.gatts_register_services((_UART_SERVICE,))
        self._connections = set()

        # Image transfer state
        self.receiving_image = False
        self.image_buffer    = bytearray()

        # Start advertising
        self._payload = advertising_payload(name=BLE_DEVICE_NAME, services=[_UART_UUID])
        self._advertise()
        print("[BLE] Advertising as:", BLE_DEVICE_NAME)

    # ── Advertising ────────────────────────────────────────
    def _advertise(self, interval_us=500_000):
        self._ble.gap_advertise(interval_us, adv_data=self._payload)

    # ── IRQ Handler (same structure as tutorial) ───────────
    def _irq(self, event, data):

        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            print("[BLE] Connected:", conn_handle)

        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            self._connections.discard(conn_handle)
            self.receiving_image = False
            self.image_buffer    = bytearray()
            print("[BLE] Disconnected, re-advertising")
            self._advertise()

        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            value = self._ble.gatts_read(value_handle)

            if value_handle == self._handle_rx:
                self._handle_incoming(value)

    # ── Incoming Data Router ───────────────────────────────
    # All data arrives on RX — first byte tells us what it is
    def _handle_incoming(self, data):
        if len(data) == 0:
            return

        # If we're mid image transfer, treat everything as image data
        # unless it's a CMD_IMAGE_END signal (single byte 0x05)
        if self.receiving_image and data[0] != CMD_IMAGE_END:
            self._handle_image_chunk(data)
            return

        # Otherwise treat as a command
        self._handle_command(data)

    # ── Command Handler ────────────────────────────────────
    def _handle_command(self, data):
        cmd = data[0]

        if cmd == CMD_LED_ON:
            print("[CMD] LED ON")
            self.led.all_on()

        elif cmd == CMD_LED_OFF:
            print("[CMD] LED OFF")
            self.led.all_off()

        elif cmd == CMD_SET_COLOR and len(data) == 4:
            r, g, b = data[1], data[2], data[3]
            print(f"[CMD] SET COLOR ({r},{g},{b})")
            self.led.set_all_color(r, g, b)

        elif cmd == CMD_SET_BRIGHTNESS and len(data) == 2:
            brightness = data[1] / 255.0
            print(f"[CMD] BRIGHTNESS {brightness:.2f}")
            self.led.set_brightness(brightness)

        elif cmd == CMD_IMAGE_START:
            print("[CMD] IMAGE START")
            self.receiving_image = True
            self.image_buffer    = bytearray()

        elif cmd == CMD_IMAGE_END:
            print("[CMD] IMAGE END")
            self._finalize_image()

    # ── Image Chunk Handler ────────────────────────────────
    def _handle_image_chunk(self, chunk):
        self.image_buffer.extend(chunk)
        received = len(self.image_buffer)
        print(f"[IMG] {received}/{IMAGE_SIZE} bytes")

    # ── Finalize Image ─────────────────────────────────────
    def _finalize_image(self):
        received = len(self.image_buffer)
        self.receiving_image = False

        if received != IMAGE_SIZE:
            print(f"[IMG] ERROR: expected {IMAGE_SIZE}, got {received}. Discarding.")
            self.image_buffer = bytearray()
            return

        success = self.store.save_image(bytes(self.image_buffer))
        if success:
            print("[IMG] Saved to flash successfully")
        else:
            print("[IMG] ERROR: failed to save to flash")

        self.image_buffer = bytearray()

    # ── Send (optional, for acknowledgements) ──────────────
    def send(self, data):
        for conn_handle in self._connections:
            self._ble.gatts_notify(conn_handle, self._handle_tx, data)

    def is_connected(self):
        return len(self._connections) > 0
