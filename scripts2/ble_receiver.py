# ==============================================================================
# ble_receiver.py — BLE peripheral for dynamic-size image transfers
# Target: Raspberry Pi Pico W, MicroPython
#
# Protocol:
#   [4 bytes MAGIC (0xDE 0xAD 0xBE 0xEF)]
#   [4 bytes image_size as uint32 little-endian]
#   [image_size bytes of GRB image data]
#
# FIX: receive state machine is now fully reset on BLE disconnect.
# Previously, if the slip ring dropped the connection mid-transfer, _stage
# remained at _STAGE_DATA with _rx_ptr at some partial offset.  The next
# reconnect would continue filling background_buffer from the wrong position,
# silently producing a corrupted image.  Now, disconnect always returns the
# receiver to _STAGE_MAGIC so the next connection starts clean.
# ==============================================================================

import bluetooth
from micropython import const
from ble_advertising import advertising_payload
from config import (
    NUS_SERVICE_UUID, NUS_TX_UUID, NUS_RX_UUID,
    TRANSFER_MAGIC, MAX_IMAGE_SIZE,
)

_IRQ_CENTRAL_CONNECT    = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE        = const(3)

_FLAG_READ          = const(0x0002)
_FLAG_WRITE_NO_RESP = const(0x0004)
_FLAG_WRITE         = const(0x0008)
_FLAG_NOTIFY        = const(0x0010)

_UART_UUID    = bluetooth.UUID(NUS_SERVICE_UUID)
_UART_TX_CHAR = (bluetooth.UUID(NUS_TX_UUID), _FLAG_READ | _FLAG_NOTIFY)
_UART_RX_CHAR = (bluetooth.UUID(NUS_RX_UUID), _FLAG_WRITE | _FLAG_WRITE_NO_RESP)
_UART_SERVICE = (_UART_UUID, (_UART_TX_CHAR, _UART_RX_CHAR))

MAGIC_LEN = len(TRANSFER_MAGIC)   # 4

_STAGE_MAGIC = const(0)
_STAGE_SIZE  = const(1)
_STAGE_DATA  = const(2)


class BLEImageReceiver:
    """
    BLE peripheral — Nordic UART Service.
    Reads image size from 8-byte header, fills background_buffer up to that
    size, signals image_ready() for the main loop to swap buffers.
    """

    def __init__(self, ble, active_buffer: bytearray, background_buffer: bytearray,
                 device_name: str = "POV-Fan"):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)

        self.active_buffer     = active_buffer
        self.background_buffer = background_buffer

        self._magic_buf = bytearray(MAGIC_LEN)
        self._magic_idx = 0

        self._size_buf = bytearray(4)
        self._size_idx = 0

        self._rx_ptr       = 0
        self._current_size = 0

        self._stage            = _STAGE_MAGIC
        self._image_ready      = False
        self._send_ack         = False
        self._needs_advertise  = False

        ((self._handle_tx, self._handle_rx),) = \
            self._ble.gatts_register_services((_UART_SERVICE,))

        # CRITICAL: expand RX characteristic buffer beyond 20-byte default.
        self._ble.gatts_set_buffer(self._handle_rx, 512, False)

        self._connections = set()
        self._payload = advertising_payload(
            name=device_name,
            services=[bluetooth.UUID(NUS_SERVICE_UUID)],
        )
        self._advertise()
        print("[BLE] Advertising as", device_name)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def current_size(self) -> int:
        return self._current_size

    def image_ready(self) -> bool:
        return self._image_ready

    def clear_ready(self) -> None:
        self._image_ready = False

    def is_connected(self) -> bool:
        return len(self._connections) > 0

    def send(self, data: bytes) -> None:
        for conn_handle in self._connections:
            self._ble.gatts_notify(conn_handle, self._handle_tx, data)

    def swap_buffers(self) -> None:
        """
        Atomically swap active and background buffers.
        Resets receive state so background_buffer is ready for the next image.
        """
        self.active_buffer, self.background_buffer = \
            self.background_buffer, self.active_buffer
        self._reset_rx_state()
        self._image_ready = False
        print("[BLE] Buffers swapped — new image active ({} bytes)".format(
            self._current_size))

    def tick(self) -> None:
        """Called from main loop — handles BLE ops unsafe in IRQ context."""
        if self._send_ack:
            self._send_ack = False
            self.send(b"OK")
            print("[BLE] ACK sent to PC")

        if self._needs_advertise:
            self._needs_advertise = False
            self._advertise()
            print("[BLE] Re-advertising")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _reset_rx_state(self) -> None:
        """Reset the three-stage receive state machine to initial conditions."""
        self._stage     = _STAGE_MAGIC
        self._magic_idx = 0
        self._size_idx  = 0
        self._rx_ptr    = 0

    # ------------------------------------------------------------------
    # IRQ & data ingestion
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
            # FIX: reset receive state so a mid-transfer disconnect does not
            # leave _stage at _STAGE_DATA with a stale _rx_ptr.  Without this,
            # the next reconnect silently continues filling background_buffer
            # from the wrong offset, producing a corrupted image.
            self._reset_rx_state()
            self._needs_advertise = True

        elif event == _IRQ_GATTS_WRITE:
            _, value_handle = data
            if value_handle == self._handle_rx:
                chunk = self._ble.gatts_read(value_handle)
                self._ingest(chunk)

    def _ingest(self, chunk: bytes) -> None:
        i = 0
        n = len(chunk)

        while i < n:
            stage = self._stage

            if stage == _STAGE_MAGIC:
                self._magic_buf[self._magic_idx % MAGIC_LEN] = chunk[i]
                self._magic_idx += 1
                i += 1
                if self._magic_idx >= MAGIC_LEN and self._check_magic():
                    self._stage    = _STAGE_SIZE
                    self._size_idx = 0
                    print("[BLE] Magic detected")

            elif stage == _STAGE_SIZE:
                self._size_buf[self._size_idx] = chunk[i]
                self._size_idx += 1
                i += 1
                if self._size_idx == 4:
                    sz = (self._size_buf[0]        |
                          self._size_buf[1] <<  8  |
                          self._size_buf[2] << 16  |
                          self._size_buf[3] << 24)
                    buf_len = len(self.background_buffer)
                    if sz <= 0 or sz > buf_len:
                        print("[BLE] Invalid size {} (max {}) — resyncing".format(sz, buf_len))
                        self._stage     = _STAGE_MAGIC
                        self._magic_idx = 0
                        continue
                    self._current_size = sz
                    self._rx_ptr       = 0
                    self._image_ready  = False
                    self._stage        = _STAGE_DATA
                    print("[BLE] Expecting {} bytes".format(sz))

            else:  # _STAGE_DATA
                remaining = self._current_size - self._rx_ptr
                available = n - i
                write_len = remaining if remaining < available else available
                self.background_buffer[self._rx_ptr : self._rx_ptr + write_len] = \
                    chunk[i : i + write_len]
                self._rx_ptr += write_len
                i += write_len

                if self._rx_ptr >= self._current_size:
                    self._stage       = _STAGE_MAGIC
                    self._magic_idx   = 0
                    self._image_ready = True
                    self._send_ack    = True
                    print("[BLE] Image complete — {} bytes".format(self._current_size))
                    return

    def _check_magic(self) -> bool:
        start = self._magic_idx % MAGIC_LEN
        for k in range(MAGIC_LEN):
            if self._magic_buf[(start + k) % MAGIC_LEN] != TRANSFER_MAGIC[k]:
                return False
        return True

    def _advertise(self, interval_us: int = 500_000) -> None:
        self._ble.gap_advertise(interval_us, adv_data=self._payload)
