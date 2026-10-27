"""
gs_protocol.py - codificação/descodificação das tramas TMTC (ver docs/TMTC_ICD.md).
Implementação independente em Python puro; é testada contra o código C do firmware em
sim/test_protocol.py.
"""
import struct
from dataclasses import dataclass

SYNC = b"\xA5\x5A"
MAX_PAYLOAD = 240

TM_STATE, TM_VISION, TM_IMG, TM_ACK = 0x01, 0x02, 0x03, 0x7F
TC_PING, TC_SET_MODE, TC_SET_WHEEL, TC_SET_RATE = 0x80, 0x81, 0x82, 0x83
TC_SET_HEADING, TC_SET_AIM, TC_SET_GAIN, TC_STREAM_CFG, TC_RECAL = 0x84, 0x85, 0x86, 0x87, 0x88
# só no simulador (não existem no firmware)
SIM_SHOVE, SIM_COVER, SIM_TARGET, SIM_LIGHT = 0xF0, 0xF1, 0xF2, 0xF3

MODES = {0: "CALIB", 1: "IDLE", 2: "WHEEL", 3: "RATE", 4: "HEADING", 5: "TRACK"}
TRK = {0: "—", 1: "LOCKED", 2: "COAST", 3: "SEARCH"}
FLAGS = {0: "ALVO", 1: "RODA SAT", 2: "MAG OK", 3: "CALIBRADO", 4: "INCLINAÇÃO"}
GAINS = {  # id: (nome, unidade, valor por omissão)
    0: ("k_theta", "1/s", 3.0), 1: ("rate_kp", "", 5.0), 2: ("rate_ki", "", 300.0),
    3: ("w_ref_max", "°/s", 40.1), 4: ("a_ref_max", "°/s²", 229.2), 5: ("wheel_lim", "rpm", 3000.0),
    6: ("torque_max", "N·m", 0.2), 7: ("r_cam", "°", 0.4), 8: ("r_mag", "°", 2.9),
    9: ("use_mag", "0/1", 1.0), 10: ("t_search", "s", 5.0),
}

STATE_FMT = "<IBBH11f"
VISION_FMT = "<IBBBB5fII"


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def pack_frame(type_: int, seq: int, payload: bytes = b"") -> bytes:
    assert len(payload) <= MAX_PAYLOAD
    body = bytes([len(payload), type_, seq & 0xFF]) + payload
    return SYNC + body + struct.pack("<H", crc16(body))


class Parser:
    """Extrai tramas de um fluxo de bytes (série) ou de datagramas (UDP)."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes):
        self.buf += data
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                del self.buf[:max(0, len(self.buf) - 1)]
                return
            del self.buf[:i]
            if len(self.buf) < 7:
                return
            n = self.buf[2]
            if n > MAX_PAYLOAD:
                del self.buf[:2]
                continue
            if len(self.buf) < n + 7:
                return
            body = bytes(self.buf[2:5 + n])
            (crc,) = struct.unpack_from("<H", self.buf, 5 + n)
            if crc16(body) == crc:
                yield self.buf[3], self.buf[4], bytes(self.buf[5:5 + n])
                del self.buf[:n + 7]
            else:
                self.crc_errors += 1
                del self.buf[:2]


@dataclass
class State:
    t_ms: int; mode: int; trk: int; flags: int
    heading: float; rate: float; bias: float; err: float; rate_ref: float
    wheel_rpm: float; wheel_cmd_rpm: float; torque: float; vbat: float
    target_err: float; cam_age: float

    def flag(self, bit):
        return bool(self.flags & (1 << bit))


@dataclass
class Vision:
    t_capture_ms: int; found: int; accepted: int; n_cand: int
    x: float; y: float; err_x: float; err_y: float; contrast: float; area: int; proc_us: int


def decode_state(p: bytes) -> State:
    return State(*struct.unpack(STATE_FMT, p))


def decode_vision(p: bytes) -> Vision:
    v = struct.unpack(VISION_FMT, p)
    return Vision(v[0], v[1], v[2], v[3], v[5], v[6], v[7], v[8], v[9], v[10], v[11])


class ImageAssembler:
    """Reconstrói a imagem 80x60 a partir de TM_IMG; devolve (img_id, ndarray) quando completa."""

    def __init__(self):
        self.cur = None
        self.chunks = {}

    def feed(self, p: bytes):
        import numpy as np
        img_id, chunk, n, w, h = p[:5]
        if img_id != self.cur:
            self.cur, self.chunks = img_id, {}
        self.chunks[chunk] = p[5:]
        if len(self.chunks) == n:
            data = b"".join(self.chunks[i] for i in range(n))
            self.chunks = {}
            if len(data) == w * h:
                return img_id, np.frombuffer(data, np.uint8).reshape(h, w)
        return None


# ---- telecomandos
def tc_ping(seq): return pack_frame(TC_PING, seq)
def tc_mode(seq, m): return pack_frame(TC_SET_MODE, seq, bytes([m]))
def tc_wheel(seq, rpm): return pack_frame(TC_SET_WHEEL, seq, struct.pack("<f", rpm))
def tc_rate(seq, dps): return pack_frame(TC_SET_RATE, seq, struct.pack("<f", dps))
def tc_heading(seq, deg): return pack_frame(TC_SET_HEADING, seq, struct.pack("<f", deg))
def tc_aim(seq, deg): return pack_frame(TC_SET_AIM, seq, struct.pack("<f", deg))
def tc_gain(seq, gid, v): return pack_frame(TC_SET_GAIN, seq, struct.pack("<Bf", gid, v))
def tc_recal(seq): return pack_frame(TC_RECAL, seq)
def tc_stream(seq, state_hz, vision_hz, img_every): return pack_frame(TC_STREAM_CFG, seq, bytes([state_hz, vision_hz, img_every]))
def sim_shove(seq, deg): return pack_frame(SIM_SHOVE, seq, struct.pack("<f", deg))
def sim_cover(seq, on): return pack_frame(SIM_COVER, seq, bytes([1 if on else 0]))
def sim_target(seq, deg): return pack_frame(SIM_TARGET, seq, struct.pack("<f", deg))
def sim_light(seq, level): return pack_frame(SIM_LIGHT, seq, struct.pack("<f", level))
