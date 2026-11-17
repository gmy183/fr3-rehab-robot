"""XHand 9-bit signed torque/current codec helpers."""

def decode_signed_9bit(raw: int) -> int:
    if not isinstance(raw, int):
        raise TypeError("raw must be int")
    if not 0 <= raw <= 0x1FF:
        raise ValueError("raw must be in [0, 511]")
    return raw - 0x200 if raw & 0x100 else raw


def encode_signed_9bit(value: int) -> int:
    if not isinstance(value, int):
        raise TypeError("value must be int")
    if not -256 <= value <= 255:
        raise ValueError("value must be in [-256, 255]")
    return value & 0x1FF
