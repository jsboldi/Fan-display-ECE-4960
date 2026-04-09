# ==============================================================================
# ble_receiver.py — BLE peripheral for receiving image data on the Pico W
# Target: Raspberry Pi Pico W, MicroPython
#
# Implements the Nordic UART Service (NUS) peripheral role.
# The PC writes chunks to the RX characteristic; this module reassembles them
# into a complete 15360-byte image stored in a background_buffer bytearray.
#
# Protocol
# --------
# Every new transfer begins with a 4-byte TRANSFER_MAGIC header.  Bytes after
# that header are stored sequentially until IMAGE_SIZE bytes have arrived, at
# which point an internal flag is set and the caller can call swap_buffers().
#
# Robustness
# ----------
# - If a new TRANSFER_MAGIC arrives mid-transfer, the receive pointer resets.
# - Partial final chunks are handled — we never write past the buffer end.
# ==============================================================================

import bluetooth
from micropython import const
from ble_advertising import advertising_payload
from config import (
    NUS_SERVICE_UUID, NUS_TX_UUID, NUS_RX_UUID,
    IMAGE_SIZE, TRANSFER_MAGIC,
)

# BLE event codes
_IRQ_CENTRAL_CONNECT    = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE        = const(3)

# GATT characteristic flags
_FLAG_READ              = const(0x0002)
_FLAG_WRITE_NO_RESP     = const(0x0004)
_FLAG_WRITE             = const(0x0008)
_FLAG_NOTIFY            = const(0x0010)

# Build the NUS service descriptor (matches led_peripheral.py layout exactly)
_UART_UUID    = bluetooth.UUID(NUS_SERVICE_UUID)
_UART_TX_CHAR = (bluetooth.UUID(NUS_TX_UUID), _FLAG_READ | _FLAG_NOTIFY)
_UART_RX_CHAR = (bluetooth.UUID(NUS_RX_UUID), _FLAG_WRITE | _FLAG_WRITE_NO_RESP)
_UART_SERVICE = (_UART_UUID, (_UART_TX_CHAR, _UART_RX_CHAR))

MAGIC_LEN = len(TRANSFER_MAGIC)  # 4


class BLEImageReceiver:
    """
    BLE peripheral that receives a POV fan image over the Nordic UART Service.

    Typical usage (inside main.py):
        ble = bluetooth.BLE()
        receiver = BLEImageReceiver(ble, active_buf, background_buf)

        while True:
            if receiver.image_ready():
                # background_buf now holds a complete image
                receiver.clear_ready()
                swap_buffers()   # handled by display controller
            ...
    """

    def __init__(self, ble, active_buffer: bytearray, background_buffer: bytearray,
                 device_name: str = "POV-Fan"):
        if len(active_buffer) != IMAGE_SIZE or len(background_buffer) != IMAGE_SIZE:
            raise ValueError(f"Buffers must each be {IMAGE_SIZE} bytes")

        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)

        self.active_buffer     = active_buffer      # being displayed
        self.background_buffer = background_buffer  # being filled via BLE

        # Receive state
        self._rx_ptr      = 0      # write position in background_buffer
        self._in_transfer = False  # True after magic received, before image complete
        self._image_ready = False  # True when a full image has been received
        self._magic_buf   = bytearray(MAGIC_LEN)  # sliding window for magic detect
        self._magic_idx   = 0
        self._send_ack       = False  # set by _irq; consumed by tick()
        self._needs_advertise = False  # set by _irq; consumed by tick()

        # Register GATT services
        ((self._handle_tx, self._handle_rx),) = \
            self._ble.gatts_register_services((_UART_SERVICE,))

        # CRITICAL: expand the RX characteristic buffer from the 20-byte default.
        # Without this, any chunk > 20 bytes is silently truncated, causing the
        # image to never complete and the connection to drop.
        self._ble.gatts_set_buffer(self._handle_rx, 512, False)

        self._connections = set()

        # Build and start advertising
        self._payload = advertising_payload(
            name=device_name,
            services=[bluetooth.UUID(NUS_SERVICE_UUID)],
        )
        self._advertise()
        print("[BLE] Advertising as", device_name)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def image_ready(self) -> bool:
        """Returns True if a complete 15360-byte image is in background_buffer."""
        return self._image_ready

    def clear_ready(self) -> None:
        """Call after consuming background_buffer (e.g. after buffer swap)."""
        self._image_ready = False

    def is_connected(self) -> bool:
        return len(self._connections) > 0

    def send(self, data: bytes) -> None:
        """Send bytes back to all connected centrals (TX notify)."""
        for conn_handle in self._connections:
            self._ble.gatts_notify(conn_handle, self._handle_tx, data)

    def swap_buffers(self) -> None:
        """
        Atomically swap active and background buffers.
        Call only when image_ready() is True.
        The caller's display loop references self.active_buffer, so after this
        call it will automatically start showing the new image.
        """
        self.active_buffer, self.background_buffer = \
            self.background_buffer, self.active_buffer
        self._rx_ptr      = 0
        self._in_transfer = False
        self.clear_ready()
        print("[BLE] Buffers swapped — new image active")

    def tick(self) -> None:
        """
        Call this from the main loop on every iteration (NOT from IRQ).
        Handles any deferred work that is unsafe to do inside _irq():
          - Sending the OK acknowledgement after a full image is received.
        """
        if self._send_ack:
            self._send_ack = False
            self.send(b"OK")
            print("[BLE] ACK sent to PC")

        if self._needs_advertise:
            self._needs_advertise = False
            self._advertise()
            print("[BLE] Re-advertising for next connection")

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            print("[BLE] Central connected:", conn_handle)

        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            self._connections.discard(conn_handle)
            print("[BLE] Central disconnected:", conn_handle)
            self._needs_advertise = True  # deferred — unsafe to call from IRQ

        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            if value_handle == self._handle_rx:
                chunk = self._ble.gatts_read(value_handle)
                self._process_chunk(chunk)

    def _process_chunk(self, chunk: bytes) -> None:
        """
        Feed a raw BLE chunk into the receive state machine.

        Two modes:
          1. Scanning mode  — we look for TRANSFER_MAGIC in the stream.
          2. Transfer mode  — we write bytes directly into background_buffer.

        Switching back to scanning happens if:
          - Another TRANSFER_MAGIC appears (sender restarted the transfer).
          - The buffer fills completely (image_ready flag is set).
        """
        i = 0
        length = len(chunk)

        while i < length:
            if not self._in_transfer:
                # --- Scanning for magic header ---
                byte = chunk[i]
                i += 1

                # Shift byte into magic window
                self._magic_buf[self._magic_idx % MAGIC_LEN] = byte
                self._magic_idx += 1

                # Check if the last MAGIC_LEN bytes match TRANSFER_MAGIC.
                # We compare a rotated view to avoid re-slicing.
                if self._magic_idx >= MAGIC_LEN and self._check_magic():
                    self._in_transfer = True
                    self._rx_ptr      = 0
                    self._image_ready = False
                    self._magic_idx   = 0
                    print("[BLE] Magic detected — starting transfer")
            else:
                # --- Filling background_buffer ---
                remaining = IMAGE_SIZE - self._rx_ptr
                available = length - i

                # If a new TRANSFER_MAGIC starts mid-chunk, honour it.
                # Look ahead in the current chunk for the magic sequence.
                magic_pos = self._find_magic(chunk, i)
                if magic_pos is not None and magic_pos < i + remaining:
                    # There is a restart before the current image completes.
                    print("[BLE] Re-sync: new magic found mid-chunk, resetting")
                    self._rx_ptr = 0
                    i = magic_pos + MAGIC_LEN  # skip past the magic
                    continue

                write_len = min(remaining, available)
                # Slice-free copy into background_buffer
                end = i + write_len
                self.background_buffer[self._rx_ptr : self._rx_ptr + write_len] = \
                    chunk[i:end]
                self._rx_ptr += write_len
                i = end

                if self._rx_ptr >= IMAGE_SIZE:
                    self._in_transfer = False
                    self._image_ready = True
                    self._send_ack    = True  # signal main loop to ACK safely
                    print("[BLE] Full image received ({} bytes)".format(IMAGE_SIZE))
                    # NOTE: do NOT call self.send() here — inside _irq() context.
                    # gatts_notify() from IRQ causes silent connection drop on Pico W.
                    # The main loop calls tick() which sends ACK outside IRQ.

    def _check_magic(self) -> bool:
        """
        True if the circular magic_buf, starting from (magic_idx % MAGIC_LEN),
        spells out TRANSFER_MAGIC.  Avoids allocating a new bytes object.
        """
        start = self._magic_idx % MAGIC_LEN
        for k in range(MAGIC_LEN):
            if self._magic_buf[(start + k) % MAGIC_LEN] != TRANSFER_MAGIC[k]:
                return False
        return True

    def _find_magic(self, chunk: bytes, start: int):
        """
        Linear search for TRANSFER_MAGIC inside chunk[start:].
        Returns the index of the first byte of the magic sequence, or None.
        """
        end = len(chunk) - MAGIC_LEN + 1
        for j in range(start, end):
            match = True
            for k in range(MAGIC_LEN):
                if chunk[j + k] != TRANSFER_MAGIC[k]:
                    match = False
                    break
            if match:
                return j
        return None

    def _advertise(self, interval_us: int = 500_000) -> None:
        self._ble.gap_advertise(interval_us, adv_data=self._payload)
