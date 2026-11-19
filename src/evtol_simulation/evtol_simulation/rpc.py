"""Small synchronous MessagePack-RPC client for AirSim's published wire protocol.

One instance per sensor process; sockets are never shared across threads.
The codec supports the MessagePack core types used by AirSim (no extensions).
No dependency on the Python-3.8-only legacy msgpack-rpc/tornado stack.
"""
import socket
import struct


class Incomplete(Exception):
    pass


class RawText(str):
    """Keep legacy MessagePack raw bytes even when they happen to be valid UTF8."""
    def __new__(cls, wire):
        value = super().__new__(cls, wire.decode('utf8'))
        value.wire = wire
        return value


def pack(value):
    if value is None:
        return b'\xc0'
    if isinstance(value, bool):
        return b'\xc3' if value else b'\xc2'
    if isinstance(value, int):
        if 0 <= value < 128:
            return bytes([value])
        if -32 <= value < 0:
            return bytes([value & 255])
        return (b'\xcf' + struct.pack('>Q', value) if value >= 0
                else b'\xd3' + struct.pack('>q', value))
    if isinstance(value, float):
        return b'\xcb' + struct.pack('>d', value)
    if isinstance(value, str):
        data = value.encode('utf8')
        return b'\xdb' + struct.pack('>I', len(data)) + data
    if isinstance(value, bytes):
        return b'\xc6' + struct.pack('>I', len(value)) + value
    if isinstance(value, (list, tuple)):
        return b'\xdd' + struct.pack('>I', len(value)) + b''.join(map(pack, value))
    if isinstance(value, dict):
        return (b'\xdf' + struct.pack('>I', len(value))
                + b''.join(pack(k) + pack(v) for k, v in value.items()))
    raise TypeError(type(value).__name__)


def unpack(data, offset=0):
    """Return (value, consumed offset), preserving invalid-UTF8 legacy raw bytes."""
    def take(n):
        nonlocal offset
        if offset + n > len(data):
            raise Incomplete()
        part = data[offset:offset+n]
        offset += n
        return part
    code = take(1)[0]
    if code < 128:
        return code, offset
    if code >= 224:
        return code - 256, offset
    if code in (192, 194, 195):
        return {192: None, 194: False, 195: True}[code], offset
    scalar = {202: '>f', 203: '>d', 204: '>B', 205: '>H', 206: '>I', 207: '>Q',
              208: '>b', 209: '>h', 210: '>i', 211: '>q'}
    if code in scalar:
        fmt = scalar[code]
        return struct.unpack(fmt, take(struct.calcsize(fmt)))[0], offset
    kind = None
    if 160 <= code <= 191:
        kind, count = 'str', code & 31
    elif 144 <= code <= 159:
        kind, count = 'array', code & 15
    elif 128 <= code <= 143:
        kind, count = 'map', code & 15
    else:
        lengths = {196: ('bin', '>B'), 197: ('bin', '>H'), 198: ('bin', '>I'),
                   217: ('str', '>B'), 218: ('str', '>H'), 219: ('str', '>I'),
                   220: ('array', '>H'), 221: ('array', '>I'),
                   222: ('map', '>H'), 223: ('map', '>I')}
        if code not in lengths:
            raise ValueError(f'Unsupported MessagePack tag 0x{code:02x}')
        kind, fmt = lengths[code]
        count = struct.unpack(fmt, take(struct.calcsize(fmt)))[0]
    if count > 256 * 1024 * 1024:
        raise ValueError('Oversize MessagePack payload')
    if kind in ('str', 'bin'):
        value = bytes(take(count))
        if kind == 'str':
            try:
                value = RawText(value)
            except UnicodeDecodeError:
                pass
        return value, offset
    items = []
    for _ in range(count * (2 if kind == 'map' else 1)):
        value, offset = unpack(data, offset)
        items.append(value)
    return (dict(zip(items[::2], items[1::2])) if kind == 'map' else items), offset


class AirSimRPC:
    def __init__(self, host='127.0.0.1', port=41451, timeout=3.0):
        self.host, self.port, self.timeout = host, int(port), float(timeout)
        self.sock = None
        self.sequence = 0
        self.buffer = bytearray()

    def call(self, method, *args):
        try:
            if self.sock is None:
                self.sock = socket.create_connection((self.host, self.port), self.timeout)
                self.sock.settimeout(self.timeout)
            self.sequence += 1
            self.sock.sendall(pack([0, self.sequence, method, list(args)]))
            while True:
                try:
                    reply, end = unpack(self.buffer)
                    del self.buffer[:end]
                    if reply[0] != 1 or reply[1] != self.sequence:
                        raise RuntimeError('Unexpected RPC response ID')
                    if reply[2] is not None:
                        raise RuntimeError(f'AirSim {method}: {reply[2]}')
                    return reply[3]
                except Incomplete:
                    chunk = self.sock.recv(1024 * 1024)
                    if not chunk:
                        raise ConnectionError('AirSim closed RPC socket')
                    self.buffer.extend(chunk)
                    if len(self.buffer) > 256 * 1024 * 1024:
                        raise ValueError('Oversize RPC response')
        except Exception:
            self.close()
            raise

    def close(self):
        if self.sock is not None:
            self.sock.close()
        self.sock = None
        self.buffer.clear()


def image_request(camera, image_type):
    return dict(camera_name=camera, image_type=image_type, pixels_as_float=False, compress=False)
