"""gs_link.py - ligação à estação de terra: UDP (Wi-Fi) ou série (USB do ESP32-S3)."""
import json
import math
import re
import socket
import zlib
from base64 import b64decode
from binascii import unhexlify
from dataclasses import dataclass

import gs_protocol as P


class Link:
    """poll() devolve eventos: ('state', State) ('vision', Vision) ('image', (id, ndarray)) ('ack', (type, seq, status))."""

    def __init__(self):
        self.parser = P.Parser()
        self.asm = P.ImageAssembler()
        self.video = P.VideoAssembler()
        self.seq = 0
        self.connected = False
        self.n_rx = self.n_tx = 0

    def next_seq(self):
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    def send_frame(self, make, *args):
        self._write(make(self.next_seq(), *args))
        self.n_tx += 1

    def _decode(self, type_, seq, payload):
        self.n_rx += 1
        try:
            if type_ == P.TM_STATE:
                return ("state", P.decode_state(payload))
            if type_ == 0x08 and len(payload) == 159 and payload[0] == 1:
                axes = tuple(P.decode_state(payload[i:i+52]) for i in (3,55,107))
                return ('state3', (payload[1],payload[2],axes))
            if type_ == P.TM_SIM3:
                return ('sim3', P.decode_sim3(payload))
            if type_ == P.TM_GIMBAL:
                return ('gimbal', P.decode_gimbal(payload))
            if type_ == P.TM_CAMERA:return ('camera',P.decode_camera(payload))
            if type_ == P.TM_VIDEO:
                video=self.video.feed(payload)
                return ('video',video) if video else None
            if type_ == P.TM_VISION:
                return ("vision", P.decode_vision(payload))
            if type_ == P.TM_IMG:
                img = self.asm.feed(payload)
                return ("image", img) if img else None
            if type_ == P.TM_ACK:
                return ("ack", tuple(payload[:3]))
        except Exception:
            return None
        return None

    def poll(self):
        out = []
        for chunk in self._read():
            for t, s, p in self.parser.feed(chunk):
                ev = self._decode(t, s, p)
                if ev:
                    out.append(ev)
        return out


class UdpLink(Link):
    def __init__(self, host="127.0.0.1", port=5000):
        super().__init__()
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1024*1024)
        self.sock.setblocking(False)
        self.connected = True

    def _write(self, data):
        self.sock.sendto(data, self.addr)

    def _read(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except (BlockingIOError, ConnectionResetError):
                return
            yield data

    def close(self):
        self.sock.close()


class SerialLink(Link):
    """Mesmo protocolo sobre série. Requer pyserial (pip install pyserial). NÃO testado em hardware."""

    read_size = 4096

    def __init__(self, port, baud=921600):
        super().__init__()
        import serial
        self.ser = serial.Serial(baudrate=baud, timeout=0)
        self.ser.port = port
        if getattr(self, 'read_only', False):
            self.ser.dtr = False
            self.ser.rts = False
        self.ser.open()
        self.connected = True

    def _write(self, data):
        self.ser.write(data)

    def _read(self):
        data = self.ser.read(self.read_size)
        if data:
            yield data

    def close(self):
        self.ser.close()


@dataclass(frozen=True)
class UsbAttitude:
    sample_us: int
    sent_us: int
    q: tuple
    rates: tuple
    state: int
    validity: int
    relative_yaw: bool = False

    @classmethod
    def from_line(cls, line):
        start = line.find(b'{')
        if start < 0:
            return None
        try:
            data = json.loads(line[start:])
            if not isinstance(data, dict) or data.get('schema') != 1:
                return None
            timestamps = data['sample_us'], data['sent_us']
            state, validity = data['state'], data['validity']
            if not all(type(value) is int and 0 <= value < 2**64 for value in (*timestamps, state, validity)):
                return None
            q, rates = tuple(data['q']), tuple(data['rates'])
            if len(q) != 4 or len(rates) != 3 or not all(
                    type(value) in (int, float) and math.isfinite(value) for value in q + rates):
                return None
        except (KeyError, TypeError, ValueError, OverflowError, RecursionError):
            return None
        return cls(*timestamps, q, rates, state, validity, data.get("yaw_reference") == "relative")

    def angles(self):
        norm = math.hypot(*self.q)
        if not math.isclose(norm, 1.0, abs_tol=0.05):
            return None
        w, x, y, z = (value / norm for value in self.q)
        roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w*y - z*x))))
        yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
        return tuple(map(math.degrees, (roll, pitch, yaw)))


_USB_ACCEL = re.compile(rb"bench_sensors: RAW t=([0-9]+) imu=[0-9]+ mag=[0-9]+ errors=[0-9]+ a=([^ ]+) (?:w=([^ ]+))?")


@dataclass(frozen=True)
class UsbAcceleration:
    sample_us: int
    body_acceleration: tuple
    body_rates: tuple | None

    @classmethod
    def from_line(cls, line):
        match = _USB_ACCEL.search(line)
        if not match:
            return None
        try:
            sample_us = int(match[1])
            x, y, z = map(float, match[2].split(b','))
        except (ValueError, OverflowError):
            return None
        if not (0 <= sample_us < 2**64) or not all(map(math.isfinite, (x, y, z))):
            return None
        # The installed cube_adcs build uses a 180° sensor-to-body rotation
        # around (0, 1, -1). RAW reports the original sensor axes.
        rates = None
        if match[3]:
            try:
                wx, wy, wz = map(float, match[3].split(b','))
                if all(map(math.isfinite, (wx, wy, wz))):
                    rates = (-wx, -wz, -wy)
            except (ValueError, OverflowError):
                pass
        return cls(sample_us, (-x, -z, -y), rates)

    def tilt(self):
        x, y, z = self.body_acceleration
        gravity = 9.80665
        norm = math.hypot(x, y, z)
        transverse = math.hypot(y, z)
        # Reject strong non-gravity acceleration and the roll singularity.
        if not 0.8 * gravity <= norm <= 1.2 * gravity or transverse < 0.10 * norm:
            return None
        return math.degrees(math.atan2(y, z)), math.degrees(math.atan2(-x, transverse))


_USB_WHEEL_PREFIX = b'BENCH_WHEEL '
_USB_CAMERA_PREFIX = b'BENCH_CAMERA '
_USB_CAMERA_DATA_PREFIX = b'BENCH_CAMERA_DATA '


@dataclass(frozen=True)
class UsbWheel:
    axis: str
    sample_us: int
    status: str
    rpm: float | None = None
    torque_nm: float | None = None
    voltage_v: float | None = None
    temperature_c: float | None = None
    position_rev: float | None = None
    mode: int | None = None
    fault: int | None = None

    @classmethod
    def from_line(cls, line):
        if not line.startswith(_USB_WHEEL_PREFIX):
            return None
        try:
            data = json.loads(line[len(_USB_WHEEL_PREFIX):])
            if not isinstance(data, dict) or type(data.get('schema')) is not int or data['schema'] != 1:
                return None
            axis, sample_us, status = data['axis'], data['sample_us'], data['status']
            if axis not in ('X', 'Y', 'Z') or type(sample_us) is not int or not 0 <= sample_us < 2**64:
                return None
            if status in ('controller_unavailable', 'query_not_sent', 'no_reply', 'queue_recovery_failed'):
                return cls(axis, sample_us, status)
            if status != 'ok':
                return None
            readings = tuple(data[field] for field in ('rpm', 'torque_nm', 'voltage_v',
                                                       'temperature_c', 'position_rev'))
            if not all(type(value) in (float, int) and math.isfinite(value) for value in readings):
                return None
            mode, fault = data['mode'], data['fault']
            if not all(type(code) is int and 0 <= code <= 255 for code in (mode, fault)):
                return None
            return cls(axis, sample_us, status, *map(float, readings), mode, fault)
        except (KeyError, TypeError, ValueError, OverflowError, RecursionError):
            return None


@dataclass(frozen=True)
class UsbSnapshot:
    width: int
    height: int
    jpeg: bytes
    sample_us: int = 0
    live: bool = False


class UsbSnapshotAssembler:
    """Reconstruct one checked JPEG from ordered USB console lines."""

    def __init__(self):
        self.header = None
        self.data = bytearray()
        self.delivered = None

    def feed(self, line):
        if line.startswith(_USB_CAMERA_PREFIX):
            self.header = None
            self.data.clear()
            try:
                fields = json.loads(line[len(_USB_CAMERA_PREFIX):])
                if not isinstance(fields, dict) or type(fields.get('schema')) is not int or fields['schema'] != 1:
                    return None
                status = fields['status']
                if status in ('initializing', 'unavailable', 'capture_failed'):
                    return 'usb_camera_status', status
                if status not in ('snapshot', 'frame'):
                    return None
                header = tuple(fields[key] for key in ('sample_us', 'width', 'height', 'size', 'crc32'))
                live = status == 'frame'
                chunk_size = fields['chunk_size'] if live else 64
                if not all(type(value) is int for value in (*header, chunk_size)):
                    return None
                stamp, width, height, size, checksum = header
                header = (*header, chunk_size, live)
                if (0 <= stamp < 2**64 and (width, height) == (160, 120) and
                        4 <= size <= 12 * 1024 and 0 <= checksum < 2**32 and
                        1 <= chunk_size <= 384 and header != self.delivered):
                    self.header = header
            except (KeyError, TypeError, ValueError, RecursionError):
                return None
            return None

        if self.header is None or not line.startswith(_USB_CAMERA_DATA_PREFIX):
            return None
        offset, _, encoded = line[len(_USB_CAMERA_DATA_PREFIX):].rstrip(b'\r').partition(b' ')
        sample_us, width, height, size, checksum, chunk_size, live = self.header
        count = min(chunk_size, size - len(self.data))
        expected = 4 * ((count + 2) // 3) if live else 2 * count
        try:
            if offset != str(len(self.data)).encode('ascii') or len(encoded) != expected:
                raise ValueError
            chunk = b64decode(encoded, validate=True) if live else unhexlify(encoded)
            if len(chunk) != count:
                raise ValueError
        except ValueError:
            self.header = None
            self.data.clear()
            return 'usb_camera_status', 'transfer_failed'
        self.data.extend(chunk)
        if len(self.data) < size:
            return None
        jpeg = bytes(self.data)
        header, self.header = self.header, None
        self.data.clear()
        if (not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9') or
                zlib.crc32(jpeg) != checksum):
            return 'usb_camera_status', 'transfer_failed'
        self.delivered = header
        return 'usb_snapshot', UsbSnapshot(width, height, jpeg, sample_us, live)


class SerialJsonLink(SerialLink):
    """Read-only attitude, reaction-wheel telemetry and camera images from the USB console."""

    read_only = True
    max_line = 2048
    read_size = 16384

    def __init__(self, port, baud=115200):
        super().__init__(port, baud)
        self.pending = b''
        self.snapshot = UsbSnapshotAssembler()

    def send_frame(self, make, *args):
        # The console has no documented command channel.
        return

    def poll(self):
        events = []
        for chunk in self._read():
            lines = (self.pending + chunk).split(b'\n')
            self.pending = lines.pop()[-self.max_line:]
            for line in lines:
                if len(line) > self.max_line:
                    continue
                if line.startswith((_USB_CAMERA_PREFIX, _USB_CAMERA_DATA_PREFIX)):
                    event = self.snapshot.feed(line)
                    if event is not None:
                        self.n_rx += 1
                        events.append(event)
                    continue
                if line.startswith(_USB_WHEEL_PREFIX):
                    wheel = UsbWheel.from_line(line)
                    if wheel is not None:
                        self.n_rx += 1
                        events.append(('usb_wheel', wheel))
                    continue
                acceleration = UsbAcceleration.from_line(line)
                if acceleration is not None:
                    self.n_rx += 1
                    events.append(('usb_acceleration', acceleration))
                    continue
                sample = UsbAttitude.from_line(line)
                if sample is not None:
                    self.n_rx += 1
                    events.append(('usb_attitude', sample))
        return events
