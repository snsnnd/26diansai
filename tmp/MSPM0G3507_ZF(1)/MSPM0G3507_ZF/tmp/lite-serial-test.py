"""
lite-tune 纯串口测试脚本
直接通过串口发送 DISCOVER 帧，打印 MCU 返回的注册信息
用法: python lite_serial_test.py COM13 115200
"""

import sys
import struct
import serial
import time

COBS_DELIM = 0x00

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF

def cobs_encode(data):
    result = bytearray()
    zero_idx = len(data)
    block = bytearray()
    for i, b in enumerate(data):
        if b == 0:
            result.append(len(block) + 1)
            result.extend(block)
            block.clear()
            zero_idx = i
        else:
            block.append(b)
            if len(block) == 254:
                result.append(255)
                result.extend(block)
                block.clear()
    result.append(len(block) + 1)
    result.extend(block)
    result.append(COBS_DELIM)
    return bytes(result)

def cobs_decode(data):
    if not data or data[-1] != COBS_DELIM:
        return None
    data = data[:-1]
    result = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        if code == 0:
            return None
        i += 1
        if code > 1:
            block = data[i:i+code-1]
            result.extend(block)
            i += code - 1
        if code < 255 and i < len(data):
            result.append(0)
    return bytes(result)

def build_discover_frame(frame_id):
    """构建 DISCOVER 帧: MAGIC + TYPE + FLAGS + FRAME_ID + FEATURES + CRC"""
    magic = struct.pack('<H', 0xA55A)      # LT_MAGIC
    ftype = bytes([0x01])                    # LT_TYPE_DISCOVER
    flags = bytes([0x00])
    fid = struct.pack('<Q', frame_id)
    features = struct.pack('<I', 0x1F)       # LT_FEATURE_ALL_STANDARD
    header = magic + ftype + flags + fid
    payload = features
    length = struct.pack('<H', len(payload))
    raw = header + length + payload
    csum = struct.pack('<H', crc16(raw))
    return raw + csum

def parse_frame(data):
    if len(data) < 13:
        return None
    magic = struct.unpack('<H', data[0:2])[0]
    if magic != 0xA55A:
        return None
    ftype = data[2]
    flags = data[3]
    fid = struct.unpack('<Q', data[4:12])[0]
    plen = struct.unpack('<H', data[12:14])[0]
    if len(data) < 14 + plen + 2:
        return None
    payload = data[14:14+plen]
    csum = struct.unpack('<H', data[14+plen:16+plen])[0]
    if crc16(data[:14+plen]) != csum:
        return None
    return {'type': ftype, 'flags': flags, 'frame_id': fid, 'payload': payload}

TYPE_NAMES = {
    0x01: 'DISCOVER', 0x02: 'REGISTER_BEGIN', 0x03: 'REGISTER_LOG_LAYOUT',
    0x04: 'REGISTER_PARAM_DESC', 0x05: 'REGISTER_CMD_DESC', 0x06: 'REGISTER_END',
    0x07: 'STATUS', 0x11: 'LOG_REPORT', 0x12: 'LOG_TEXT',
    0x21: 'PARAM_SET', 0x22: 'PARAM_GET', 0x23: 'PARAM_REPORT',
    0x31: 'CMD_REQUEST', 0x32: 'CMD_RESPONSE',
}

def read_str8(data, offset):
    length = data[offset]
    return data[offset+1:offset+1+length].decode('utf-8', errors='replace'), offset + 1 + length

def main():
    if len(sys.argv) < 2:
        print(f"用法: python {sys.argv[0]} COM13 [115200]")
        return
    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print(f"打开 {port} @ {baud}...")
    ser = serial.Serial(port, baud, timeout=2)
    ser.reset_input_buffer()

    print("发送 DISCOVER...")
    frame = build_discover_frame(1)
    wire = cobs_encode(frame)
    ser.write(wire)
    print(f"  TX: {wire.hex()}")

    print("等待响应...")
    time.sleep(0.5)
    buf = bytearray()
    while True:
        raw = ser.read(1024)
        if not raw:
            break
        buf.extend(raw)
        # 找 COBS 帧边界 (0x00)
        while 0x00 in buf:
            end = buf.index(0x00)
            cobs_frame = buf[:end+1]
            buf = buf[end+1:]

            decoded = cobs_decode(cobs_frame)
            if not decoded:
                print(f"  COBS decode failed: {cobs_frame.hex()}")
                continue

            parsed = parse_frame(decoded)
            if not parsed:
                print(f"  Frame parse failed: {decoded.hex()}")
                continue

            tname = TYPE_NAMES.get(parsed['type'], f"0x{parsed['type']:02X}")
            print(f"\n  RX: {tname} (fid={parsed['frame_id']}, len={len(parsed['payload'])})")

            if parsed['type'] == 0x02:  # REGISTER_BEGIN
                print(f"    device: {parsed['payload'].decode('utf-8', errors='replace')}")

            elif parsed['type'] == 0x03:  # REGISTER_LOG_LAYOUT
                layout_id = parsed['payload'][0]
                period = struct.unpack('<H', parsed['payload'][1:3])[0]
                count = parsed['payload'][3]
                print(f"    layout_id={layout_id}, period={period}ms, fields={count}")
                off = 4
                for i in range(count):
                    fid = struct.unpack('<H', parsed['payload'][off:off+2])[0]
                    vtype = parsed['payload'][off+2]
                    name, off = read_str8(parsed['payload'], off+3)
                    unit, off = read_str8(parsed['payload'], off)
                    print(f"      field[{fid}]: {name} type={vtype} unit={unit}")

            elif parsed['type'] == 0x04:  # REGISTER_PARAM_DESC
                pid = struct.unpack('<H', parsed['payload'][0:2])[0]
                vtype = parsed['payload'][2]
                off = 3
                name, off = read_str8(parsed['payload'], off)
                unit, off = read_str8(parsed['payload'], off)
                print(f"    param[{pid}]: {name} type={vtype} unit={unit}")

            elif parsed['type'] == 0x05:  # REGISTER_CMD_DESC
                cid = struct.unpack('<H', parsed['payload'][0:2])[0]
                flags = parsed['payload'][2]
                off = 3
                name, off = read_str8(parsed['payload'], off)
                print(f"    cmd[{cid}]: {name} flags=0x{flags:02X}")

            elif parsed['type'] == 0x06:  # REGISTER_END
                print("    注册完成!")

            elif parsed['type'] == 0x07:  # STATUS
                protocol_ver = parsed['payload'][0]
                print(f"    protocol v{protocol_ver} connected!")

            elif parsed['type'] == 0x11:  # LOG_REPORT
                layout_id = parsed['payload'][0]
                count = parsed['payload'][1]
                print(f"    telemetry layout={layout_id}, values={count}")

    ser.close()
    print("\n完成")

if __name__ == '__main__':
    main()
