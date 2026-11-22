"""gs_link.py - ligação à estação de terra: UDP (Wi-Fi) ou série (USB do ESP32-S3)."""
import socket
import time

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

    def __init__(self, port, baud=921600):
        super().__init__()
        import serial
        self.ser = serial.Serial(port, baud, timeout=0)
        self.connected = True

    def _write(self, data):
        self.ser.write(data)

    def _read(self):
        data = self.ser.read(4096)
        if data:
            yield data

    def close(self):
        self.ser.close()
