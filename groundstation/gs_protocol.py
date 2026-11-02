"""
gs_protocol.py - codificação/descodificação das tramas TMTC (ver docs/TMTC_ICD.md).
Implementação independente em Python puro; é testada contra o código C do firmware em
sim/test_protocol.py.
"""
import struct
import time
from dataclasses import dataclass

SYNC = b"\xA5\x5A"
MAX_PAYLOAD = 240

TM_STATE, TM_VISION, TM_IMG, TM_ACK = 0x01, 0x02, 0x03, 0x7F
TC_PING, TC_SET_MODE, TC_SET_WHEEL, TC_SET_RATE = 0x80, 0x81, 0x82, 0x83
TC_SET_HEADING, TC_SET_AIM, TC_SET_GAIN, TC_STREAM_CFG, TC_RECAL = 0x84, 0x85, 0x86, 0x87, 0x88
# só no simulador (não existem no firmware)
SIM_SHOVE, SIM_COVER, SIM_TARGET, SIM_LIGHT = 0xF0, 0xF1, 0xF2, 0xF3
SIM_SHOVE_XY, TM_SIM3 = 0xF4, 0x04
SIM_GIMBAL_CFG, SIM_REST, TM_GIMBAL = 0xF5, 0xF6, 0x05
SIM_CAMERA, TM_CAMERA, TM_VIDEO = 0xF7, 0x06, 0x07
SIM_CAMERA_CENTER=0xF8

MODES = {0: "CALIB", 1: "IDLE", 2: "WHEEL", 3: "RATE", 4: "HEADING", 5: "TRACK"}
TRK = {0: "—", 1: "LOCKED", 2: "COAST", 3: "SEARCH"}
FLAGS = {0: "ALVO", 1: "RODA SAT", 2: "MAG OK", 3: "CALIBRADO", 4: "INCLINAÇÃO",5:'ALÍVIO DE MOMENTO'}
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


@dataclass
class Sim3:
    t_ms: int
    saturated: int
    attitude: tuple
    rates: tuple
    wheels: tuple
    commands: tuple
    torques: tuple
    errors: tuple
    targets: tuple

    @property
    def unloading(self):return (self.saturated>>3)&7


def decode_sim3(payload):
    values = struct.unpack('<IB21f', payload)
    return Sim3(values[0], values[1], *(tuple(values[i:i+3]) for i in range(2, 23, 3)))


@dataclass
class GimbalState:
    t_ms: int
    reference_id: int
    attitude: tuple
    reference: tuple
    relative: tuple
    planes: tuple
    torques: tuple
    body_g: tuple
    targets: tuple
    mass: float
    g: float
    com_x: float
    com_y: float
    com_z: float


def decode_gimbal(payload):
    values=struct.unpack('<IH26f',payload)
    return GimbalState(values[0],values[1],*(tuple(values[i:i+3]) for i in range(2,23,3)),*values[23:])


@dataclass
class CameraState:
    mode:int
    status:int
    found:int
    inside:int
    intent:int
    hfov:float
    virtual_fov:float
    x:float
    y:float
    yaw:float
    pitch:float
    coverage:float
    age:float
    message:str

    @property
    def centering(self):return self.intent==1

    @property
    def waiting(self):return self.intent==2


def decode_camera(payload):
    return CameraState(*struct.unpack('<BBBBB8f',payload[:37]),payload[37:].decode('utf-8',errors='replace'))


class VideoAssembler:
    def __init__(self):self.pending={}
    def feed(self,payload):
        if len(payload)<7:return None
        frame,view,chunk,count=struct.unpack('<HBHH',payload[:7])
        if view not in (0,1) or not 1<=count<=1024 or chunk>=count:return None
        now=time.monotonic()
        self.pending={k:v for k,v in self.pending.items() if now-v[0]<1.5}
        key=(view,frame)
        if key not in self.pending:
            if len(self.pending)>=4:self.pending.pop(next(iter(self.pending)))
            self.pending[key]=(now,count,{})
        stamp,n,parts=self.pending[key]
        if n!=count:return None
        parts[chunk]=payload[7:]
        if len(parts)==count:
            self.pending.pop(key)
            return view,b''.join(parts[i] for i in range(count))
        return None


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

def sim_shove_xy(seq, x_deg, y_deg): return pack_frame(SIM_SHOVE_XY, seq, struct.pack('<ff', x_deg, y_deg))

def sim_gimbal_config(seq,mass,x,y,z): return pack_frame(SIM_GIMBAL_CFG,seq,struct.pack('<4f',mass,x,y,z))

def sim_rest(seq): return pack_frame(SIM_REST,seq)

def sim_camera(seq,enabled,index=0,hfov=60,virtual_fov=40):
    return pack_frame(SIM_CAMERA,seq,struct.pack('<BBff',int(enabled),index,hfov,virtual_fov))

def sim_camera_center(seq):return pack_frame(SIM_CAMERA_CENTER,seq)
def sim_cover(seq, on): return pack_frame(SIM_COVER, seq, bytes([1 if on else 0]))
def sim_target(seq, deg): return pack_frame(SIM_TARGET, seq, struct.pack("<f", deg))
def sim_light(seq, level): return pack_frame(SIM_LIGHT, seq, struct.pack("<f", level))
