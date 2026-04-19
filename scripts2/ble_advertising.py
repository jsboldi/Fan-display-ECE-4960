# ==============================================================================
# ble_advertising.py — BLE advertising payload builder / decoder
# Target: Raspberry Pi Pico W, MicroPython
# ==============================================================================

from micropython import const
import struct
import bluetooth

_ADV_TYPE_FLAGS            = const(0x01)
_ADV_TYPE_NAME             = const(0x09)
_ADV_TYPE_UUID16_COMPLETE  = const(0x03)
_ADV_TYPE_UUID32_COMPLETE  = const(0x05)
_ADV_TYPE_UUID128_COMPLETE = const(0x07)
_ADV_TYPE_UUID16_MORE      = const(0x02)
_ADV_TYPE_UUID32_MORE      = const(0x04)
_ADV_TYPE_UUID128_MORE     = const(0x06)
_ADV_TYPE_APPEARANCE       = const(0x19)


def advertising_payload(
    limited_disc: bool = False,
    br_edr: bool = False,
    name=None,
    services=None,
    appearance: int = 0,
) -> bytearray:
    payload = bytearray()

    def _append(adv_type: int, value: bytes) -> None:
        nonlocal payload
        payload += struct.pack("BB", len(value) + 1, adv_type) + value

    _append(
        _ADV_TYPE_FLAGS,
        struct.pack(
            "B",
            (0x01 if limited_disc else 0x02) + (0x18 if br_edr else 0x04),
        ),
    )

    if name:
        _append(_ADV_TYPE_NAME, name if isinstance(name, bytes) else name.encode())

    if services:
        for uuid in services:
            b = bytes(uuid)
            if len(b) == 2:
                _append(_ADV_TYPE_UUID16_COMPLETE, b)
            elif len(b) == 4:
                _append(_ADV_TYPE_UUID32_COMPLETE, b)
            elif len(b) == 16:
                _append(_ADV_TYPE_UUID128_COMPLETE, b)

    if appearance:
        _append(_ADV_TYPE_APPEARANCE, struct.pack("<h", appearance))

    return payload


def decode_field(payload: bytes, adv_type: int) -> list:
    i = 0
    result = []
    while i + 1 < len(payload):
        length = payload[i]
        if length == 0:
            break
        if payload[i + 1] == adv_type:
            result.append(payload[i + 2 : i + 1 + length])
        i += 1 + length
    return result


def decode_name(payload: bytes) -> str:
    n = decode_field(payload, _ADV_TYPE_NAME)
    return str(n[0], "utf-8") if n else ""


def decode_services(payload: bytes) -> list:
    services = []
    for u in decode_field(payload, _ADV_TYPE_UUID16_COMPLETE):
        services.append(bluetooth.UUID(struct.unpack("<H", u)[0]))
    for u in decode_field(payload, _ADV_TYPE_UUID32_COMPLETE):
        services.append(bluetooth.UUID(struct.unpack("<I", u)[0]))
    for u in decode_field(payload, _ADV_TYPE_UUID128_COMPLETE):
        services.append(bluetooth.UUID(u))
    return services
