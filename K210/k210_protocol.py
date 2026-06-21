# k210_protocol.py
# K210 -> STM32 二进制帧打包工具
# 帧格式：AA 55 type len payload checksum
# checksum = (type + len + payload所有字节) & 0xFF

FRAME_HEAD1 = 0xAA
FRAME_HEAD2 = 0x55

FRAME_TYPE_LINE = 0x01   # 巡线偏差
FRAME_TYPE_QR   = 0x02   # 二维码中心

def _clamp_int16(v):
    if v > 32767:
        return 32767
    if v < -32768:
        return -32768
    return int(v)

def _int16_to_le_bytes(v):
    v = _clamp_int16(v)
    if v < 0:
        v = 0x10000 + v
    return bytes([v & 0xFF, (v >> 8) & 0xFF])

def build_frame(frame_type, payload):
    if payload is None:
        payload = b""
    if len(payload) > 255:
        raise ValueError("payload too long")

    buf = bytearray()
    buf.append(FRAME_HEAD1)
    buf.append(FRAME_HEAD2)
    buf.append(frame_type & 0xFF)
    buf.append(len(payload) & 0xFF)
    buf.extend(payload)

    checksum = sum(buf[2:]) & 0xFF
    buf.append(checksum)
    return bytes(buf)

def send_line_offset(uart, offset):
    payload = _int16_to_le_bytes(offset)
    uart.write(build_frame(FRAME_TYPE_LINE, payload))

def send_qr_center(uart, x, y):
    payload = _int16_to_le_bytes(x) + _int16_to_le_bytes(y)
    uart.write(build_frame(FRAME_TYPE_QR, payload))

def send_text(uart, text):
    if isinstance(text, str):
        text = text.encode()
    uart.write(text)
