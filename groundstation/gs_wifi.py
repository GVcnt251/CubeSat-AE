"""Read-only WebSocket telemetry for the factory CubeSat firmware."""

import math
import struct
import time
from collections import deque
from dataclasses import dataclass

from gs_link import Link, UsbAttitude, UsbSnapshot


@dataclass(frozen=True)
class FactoryTelemetry:
    attitude: UsbAttitude
    gyro: tuple
    accel: tuple
    mag: tuple
    battery: tuple
    wheels: tuple
    can: tuple
    uptime_ms: int


def decode_factory_message(data, received_us):
    if data.startswith(b'CBTM') and len(data) >= 420:
        q = struct.unpack_from('<4f', data, 12)
        gyro = struct.unpack_from('<3f', data, 28)
        accel = struct.unpack_from('<3f', data, 40)
        mag = struct.unpack_from('<3f', data, 52)
        battery = struct.unpack_from('<4f', data, 132)
        mode, = struct.unpack_from('<H', data, 152)
        uptime_ms, = struct.unpack_from('<I', data, 168)
        wheels = tuple(struct.unpack_from('<6f2I', data, offset) for offset in (172, 204, 236))
        can = struct.unpack_from('<3I', data, 268)
        attitude = UsbAttitude(uptime_ms * 1000, received_us, q, tuple(map(math.radians, gyro)), mode, 1)
        return 'wifi_telemetry', FactoryTelemetry(attitude, gyro, accel, mag, battery, wheels, can, uptime_ms)
    if data.startswith(b'CBJ1') and 12 <= len(data) <= 128 * 1024:
        width, height = struct.unpack_from('<HH', data, 4)
        jpeg = data[8:]
        if (1 <= width <= 640 and 1 <= height <= 480 and
                jpeg.startswith(b'\xff\xd8') and jpeg.endswith(b'\xff\xd9')):
            return 'usb_snapshot', UsbSnapshot(width, height, jpeg, received_us, True)
    return None


class FactoryWiFiLink(Link):
    """Open the factory telemetry endpoint and request the JPEG stream."""

    read_only = True
    factory_wifi = True

    def __init__(self, host='192.168.4.1', port=80):
        super().__init__()
        from PyQt5 import QtCore, QtNetwork, QtWebSockets

        self.addr = (host, port)
        self.url = QtCore.QUrl(f'ws://{host}:{port}/ws')
        self.socket = QtWebSockets.QWebSocket()
        self.socket.setMaxAllowedIncomingMessageSize(128 * 1024)
        self.pending = deque(maxlen=128)
        self.closed = False
        self.unconnected = QtNetwork.QAbstractSocket.UnconnectedState
        self.reconnect = QtCore.QTimer(self.socket)
        self.reconnect.setSingleShot(True)
        self.reconnect.timeout.connect(self._connect)
        self.socket.connected.connect(self._opened)
        self.socket.disconnected.connect(self._disconnected)
        self.socket.error.connect(self._disconnected)
        self.socket.binaryMessageReceived.connect(self._receive)
        self.reconnect.start(0)

    def _connect(self):
        if not self.closed and self.socket.state() == self.unconnected:
            self.socket.open(self.url)

    def _opened(self):
        if self.closed:
            return
        self.reconnect.stop()
        self.connected = True
        # Factory SET_STREAM: enable JPEG, use the minimum frame interval, quality 60.
        request = struct.pack('<4sHH6f', b'CBC1', 6, self.next_seq(), 1, 1, 1, 60, 0, 0)
        if self.socket.sendBinaryMessage(request) > 0:
            self.n_tx += 1

    def _disconnected(self, *_):
        self.connected = False
        if not self.closed:
            self.reconnect.start(1500)

    def _receive(self, packet):
        event = decode_factory_message(bytes(packet), time.monotonic_ns() // 1000)
        if event is not None and not self.closed:
            self.n_rx += 1
            self.pending.append(event)

    def send_frame(self, make, *args):
        return

    def poll(self):
        events = list(self.pending)
        self.pending.clear()
        return events

    def close(self):
        self.closed = True
        self.connected = False
        self.reconnect.stop()
        self.socket.abort()
