# transfer_test_pico.py

import bluetooth
from ble_advertising import advertising_payload
from micropython import const

_IRQ_CENTRAL_CONNECT    = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE        = const(3)
_FLAG_WRITE             = const(0x0008)
_FLAG_WRITE_NO_RESPONSE = const(0x0004)
_FLAG_READ              = const(0x0002)
_FLAG_NOTIFY            = const(0x0010)

_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX   = (bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"), _FLAG_READ | _FLAG_NOTIFY,)
_UART_RX   = (bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"), _FLAG_WRITE | _FLAG_WRITE_NO_RESPONSE,)
_UART_SERVICE = (_UART_UUID, (_UART_TX, _UART_RX,),)

EXPECTED_SIZE   = 21600
CMD_IMAGE_START = 0x04
CMD_IMAGE_END   = 0x05
CMD_IMAGE_DATA  = 0xFF

ble = bluetooth.BLE()
ble.active(True)

((handle_tx, handle_rx),) = ble.gatts_register_services((_UART_SERVICE,))
ble.gatts_set_buffer(handle_rx, 256)   # ← critical fix

connections     = set()
image_buffer    = bytearray()
receiving_image = False

def irq(event, data):
    global receiving_image, image_buffer

    if event == _IRQ_CENTRAL_CONNECT:
        conn_handle, _, _ = data
        connections.add(conn_handle)
        print("Connected")

    elif event == _IRQ_CENTRAL_DISCONNECT:
        conn_handle, _, _ = data
        connections.discard(conn_handle)
        receiving_image = False
        image_buffer    = bytearray()
        print("Disconnected, re-advertising")
        ble.gap_advertise(500_000, adv_data=payload)

    elif event == _IRQ_GATTS_WRITE:
        _, value_handle = data
        value = ble.gatts_read(value_handle)

        if value_handle == handle_rx:
            cmd = value[0]

            if cmd == CMD_IMAGE_DATA and receiving_image:
                image_buffer.extend(value[1:])
                print(f"Received {len(image_buffer)}/{EXPECTED_SIZE} bytes")

            elif cmd == CMD_IMAGE_START:
                print("Image transfer started")
                receiving_image = True
                image_buffer    = bytearray()

            elif cmd == CMD_IMAGE_END:
                print(f"Transfer complete — {len(image_buffer)} bytes received")
                if len(image_buffer) == EXPECTED_SIZE:
                    print("SUCCESS: correct size")
                    with open("test_image.bin", "wb") as f:
                        f.write(image_buffer)
                    print("Saved to flash as test_image.bin")
                else:
                    print(f"ERROR: expected {EXPECTED_SIZE}, got {len(image_buffer)}")
                receiving_image = False
                image_buffer    = bytearray()

ble.irq(irq)
payload = advertising_payload(name="FanDisplay", services=[_UART_UUID])
ble.gap_advertise(500_000, adv_data=payload)
print("Advertising as FanDisplay...")
print("Waiting for connection...")

while True:
    pass
