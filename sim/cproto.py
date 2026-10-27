"""cproto.py - wrappers ctypes das funções de protocolo do núcleo C (as mesmas do firmware)."""
import ctypes as C
from sil import lib, TlmT, VisionT, CmdT, ParserT, MoteusStatus, _u8

IMG_NCHUNKS = 24


def pack_frame(type_, seq, payload=b""):
    out = (C.c_uint8 * (len(payload) + 7))()
    pl = (C.c_uint8 * max(1, len(payload))).from_buffer_copy(payload or b"\0")
    n = lib().cc_frame_pack(out, type_, seq & 0xFF, pl, len(payload))
    return bytes(out[:n])


def crc16(data: bytes) -> int:
    buf = (C.c_uint8 * max(1, len(data))).from_buffer_copy(data or b"\0")
    return lib().cc_crc16(buf, len(data))


class CParser:
    def __init__(self):
        self.p = ParserT()
        lib().cc_parser_init(C.byref(self.p))

    def feed(self, data: bytes):
        """Gera (type, seq, payload) por cada trama válida."""
        for b in data:
            if lib().cc_parser_feed(C.byref(self.p), b):
                yield self.p.type, self.p.seq, bytes(self.p.payload[: self.p.len])


def pack_state(tlm: TlmT) -> bytes:
    buf = (C.c_uint8 * 240)()
    n = lib().cc_pack_tm_state(buf, C.byref(tlm))
    return bytes(buf[:n])


def pack_vision(vis: VisionT, proc_us: int) -> bytes:
    buf = (C.c_uint8 * 240)()
    n = lib().cc_pack_tm_vision(buf, C.byref(vis), int(proc_us))
    return bytes(buf[:n])


def pack_img_chunk(gray, img_id, chunk) -> bytes:
    import numpy as np
    g = np.ascontiguousarray(gray, dtype=np.uint8)
    buf = (C.c_uint8 * 240)()
    n = lib().cc_pack_tm_img_chunk(buf, g.ctypes.data_as(C.POINTER(C.c_uint8)), img_id & 0xFF, chunk)
    return bytes(buf[:n])


def pack_ack(type_, seq, status) -> bytes:
    buf = (C.c_uint8 * 8)()
    n = lib().cc_pack_tm_ack(buf, type_, seq, status)
    return bytes(buf[:n])


def tc_to_cmd(type_, payload: bytes):
    """-> (resultado, CmdT). resultado: 1 mapeado, 0 não é do núcleo, <0 erro."""
    cmd = CmdT()
    pl = (C.c_uint8 * max(1, len(payload))).from_buffer_copy(payload or b"\0")
    r = lib().cc_tc_to_cmd(type_, pl, len(payload), C.byref(cmd))
    return r, cmd


def moteus_velocity(vel_rev_s, max_tq, accel, query=True) -> bytes:
    buf = (C.c_uint8 * 64)()
    n = lib().moteus_pack_velocity(buf, vel_rev_s, max_tq, accel, int(query))
    return bytes(buf[:n])


def moteus_stop(query=True) -> bytes:
    buf = (C.c_uint8 * 64)()
    return bytes(buf[: lib().moteus_pack_stop(buf, int(query))])


def moteus_parse(data: bytes):
    st = MoteusStatus()
    pl = (C.c_uint8 * len(data)).from_buffer_copy(data)
    r = lib().moteus_parse_reply(pl, len(data), C.byref(st))
    return r, st
