"""
sil.py - Software-in-the-loop: o MESMO código C do firmware (libcubecore) a controlar o simulador.

  Core    : ligação ctypes ao núcleo (controlo, estimador, detector, protocolo, moteus)
  Runner  : malha do simulador a 1 kHz; IMU/controlo a 200 Hz, magnetómetro a 50 Hz,
            câmara a 20 fps com latência (~80 ms), driver com latência de 2 ms
"""
import ctypes as C
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CORE_DIR = os.path.join(ROOT, "core")

# ---- constantes espelhadas de cube_core.h / cc_proto.h
MODE_CALIB, MODE_IDLE, MODE_WHEEL, MODE_RATE, MODE_HEADING, MODE_TRACK = range(6)
TRK_NONE, TRK_LOCKED, TRK_COAST, TRK_SEARCH = range(4)
(CMD_NONE, CMD_MODE, CMD_WHEEL, CMD_RATE, CMD_HEADING, CMD_AIM, CMD_GAIN, CMD_RECAL) = range(8)
(GAIN_K_THETA, GAIN_RATE_KP, GAIN_RATE_KI, GAIN_W_REF_MAX, GAIN_A_REF_MAX, GAIN_WHEEL_LIM,
 GAIN_TORQUE_MAX, GAIN_R_CAM, GAIN_R_MAG, GAIN_USE_MAG, GAIN_T_SEARCH) = range(11)
FLAG_TARGET_VALID, FLAG_WHEEL_SAT, FLAG_MAG_OK, FLAG_CALIBRATED, FLAG_TILT_HIGH = 1, 2, 4, 8, 16

IMG_W, IMG_H = 320, 240


class CmdT(C.Structure):
    _fields_ = [("type", C.c_uint32), ("id", C.c_uint32), ("a", C.c_float), ("b", C.c_float)]


class MeasT(C.Structure):
    _fields_ = [("gyro", C.c_float * 3), ("accel", C.c_float * 3), ("wheel_rps", C.c_float),
                ("wheel_torque", C.c_float), ("vbat_v", C.c_float)]


class OutT(C.Structure):
    _fields_ = [("enable", C.c_uint32), ("wheel_cmd_rps", C.c_float),
                ("torque_max_nm", C.c_float), ("accel_limit_rps2", C.c_float)]


class TlmT(C.Structure):
    _fields_ = [("t_ms", C.c_uint32), ("mode", C.c_uint8), ("trk", C.c_uint8), ("flags", C.c_uint16)] + \
               [(n, C.c_float) for n in ("heading", "rate", "bias", "err", "rate_ref", "wheel_rps",
                                         "wheel_cmd_rps", "torque_nm", "vbat_v", "target_err", "cam_age_s")]


class VisionT(C.Structure):
    _fields_ = [("t_capture_ms", C.c_uint32), ("found", C.c_uint8), ("accepted", C.c_uint8),
                ("n_cand", C.c_uint8), ("pad", C.c_uint8)] + \
               [(n, C.c_float) for n in ("x", "y", "err_x", "err_y", "contrast")] + [("area", C.c_uint32)]


class MoteusStatus(C.Structure):
    _fields_ = [("have_mode", C.c_uint8), ("have_motion", C.c_uint8), ("have_aux", C.c_uint8),
                ("mode", C.c_uint8), ("fault", C.c_uint8),
                ("position_rev", C.c_float), ("velocity_rev_s", C.c_float), ("torque_nm", C.c_float),
                ("voltage_v", C.c_float), ("temp_c", C.c_float)]


class ParserT(C.Structure):
    _fields_ = [("st", C.c_uint8), ("idx", C.c_uint8), ("len", C.c_uint8), ("type", C.c_uint8),
                ("seq", C.c_uint8), ("payload", C.c_uint8 * 240), ("crc", C.c_uint16),
                ("n_crc_err", C.c_uint32)]


def _libname():
    return "libcubecore.dll" if sys.platform.startswith("win") else "libcubecore.so"


def load_lib(rebuild=False):
    path = os.path.join(CORE_DIR, _libname())
    if rebuild or not os.path.exists(path):
        subprocess.check_call(["make", "-C", CORE_DIR] if not rebuild else ["make", "-C", CORE_DIR, "-B"],
                              stdout=subprocess.DEVNULL)
    lib = C.CDLL(path)
    u8p = C.POINTER(C.c_uint8)
    lib.cc_create.restype = C.c_void_p
    lib.cc_destroy.argtypes = [C.c_void_p]
    lib.cc_command.argtypes = [C.c_void_p, C.POINTER(CmdT)]
    lib.cc_step.argtypes = [C.c_void_p, C.c_uint64, C.POINTER(MeasT), C.POINTER(OutT)]
    lib.cc_on_mag.argtypes = [C.c_void_p, C.c_uint64, C.POINTER(C.c_float)]
    lib.cc_on_image.argtypes = [C.c_void_p, C.c_uint64, C.c_uint64, u8p, C.POINTER(VisionT)]
    lib.cc_on_image.restype = C.c_int
    lib.cc_get_tlm.argtypes = [C.c_void_p, C.POINTER(TlmT)]
    for n, t in (("cmd", CmdT), ("meas", MeasT), ("out", OutT), ("tlm", TlmT), ("vision", VisionT)):
        f = getattr(lib, f"cc_sizeof_{n}")
        f.restype = C.c_uint32
        assert f() == C.sizeof(t), f"layout de cc_{n}_t difere entre C e ctypes ({f()} vs {C.sizeof(t)})"
    # protocolo
    lib.cc_crc16.argtypes = [u8p, C.c_size_t]; lib.cc_crc16.restype = C.c_uint16
    lib.cc_frame_pack.argtypes = [u8p, C.c_uint8, C.c_uint8, u8p, C.c_uint8]; lib.cc_frame_pack.restype = C.c_size_t
    lib.cc_parser_init.argtypes = [C.POINTER(ParserT)]
    lib.cc_parser_feed.argtypes = [C.POINTER(ParserT), C.c_uint8]; lib.cc_parser_feed.restype = C.c_int
    lib.cc_pack_tm_state.argtypes = [u8p, C.POINTER(TlmT)]; lib.cc_pack_tm_state.restype = C.c_size_t
    lib.cc_pack_tm_vision.argtypes = [u8p, C.POINTER(VisionT), C.c_uint32]; lib.cc_pack_tm_vision.restype = C.c_size_t
    lib.cc_pack_tm_img_chunk.argtypes = [u8p, u8p, C.c_uint8, C.c_uint8]; lib.cc_pack_tm_img_chunk.restype = C.c_size_t
    lib.cc_pack_tm_ack.argtypes = [u8p, C.c_uint8, C.c_uint8, C.c_uint8]; lib.cc_pack_tm_ack.restype = C.c_size_t
    lib.cc_tc_to_cmd.argtypes = [C.c_uint8, u8p, C.c_uint8, C.POINTER(CmdT)]; lib.cc_tc_to_cmd.restype = C.c_int
    # moteus
    lib.moteus_pack_velocity.argtypes = [u8p, C.c_float, C.c_float, C.c_float, C.c_int]; lib.moteus_pack_velocity.restype = C.c_size_t
    lib.moteus_pack_stop.argtypes = [u8p, C.c_int]; lib.moteus_pack_stop.restype = C.c_size_t
    lib.moteus_parse_reply.argtypes = [u8p, C.c_size_t, C.POINTER(MoteusStatus)]; lib.moteus_parse_reply.restype = C.c_int
    lib.moteus_can_id.argtypes = [C.c_uint8, C.c_uint8, C.c_int]; lib.moteus_can_id.restype = C.c_uint32
    return lib


LIB = None


def lib():
    global LIB
    if LIB is None:
        LIB = load_lib()
    return LIB


def _u8(buf):
    return C.cast(buf, C.POINTER(C.c_uint8))


class Core:
    """Instância do núcleo de controlo (equivalente a `static cc_t core;` no firmware)."""

    def __init__(self):
        self.L = lib()
        self.h = C.c_void_p(self.L.cc_create())
        self._meas, self._out, self._tlm, self._vis = MeasT(), OutT(), TlmT(), VisionT()

    def __del__(self):
        try:
            self.L.cc_destroy(self.h)
        except Exception:
            pass

    def command(self, type_, a=0.0, id_=0):
        c = CmdT(type_, id_, a, 0.0)
        self.L.cc_command(self.h, C.byref(c))

    def gain(self, gid, value):
        self.command(CMD_GAIN, value, gid)

    def step(self, t_us, gyro, accel, wheel_rps, torque=0.0, vbat=22.2):
        m = self._meas
        for i in range(3):
            m.gyro[i] = gyro[i]; m.accel[i] = accel[i]
        m.wheel_rps = wheel_rps; m.wheel_torque = torque; m.vbat_v = vbat
        self.L.cc_step(self.h, t_us, C.byref(m), C.byref(self._out))
        return self._out

    def on_mag(self, t_us, mag):
        arr = (C.c_float * 3)(*[float(x) for x in mag])
        self.L.cc_on_mag(self.h, t_us, arr)

    def on_image(self, t_cap_us, now_us, img):
        img = np.ascontiguousarray(img, dtype=np.uint8)
        t0 = time.perf_counter()
        self.L.cc_on_image(self.h, t_cap_us, now_us, img.ctypes.data_as(C.POINTER(C.c_uint8)), C.byref(self._vis))
        dt_us = (time.perf_counter() - t0) * 1e6
        return self._vis, dt_us

    def tlm(self):
        self.L.cc_get_tlm(self.h, C.byref(self._tlm))
        return self._tlm


# ====================================================================== runner
@dataclass
class Log:
    t: list = field(default_factory=list)
    psi: list = field(default_factory=list)       # rumo verdadeiro (rad)
    w: list = field(default_factory=list)
    Om: list = field(default_factory=list)        # roda (rad/s)
    tau_m: list = field(default_factory=list)
    target: list = field(default_factory=list)    # rumo do alvo verdadeiro (rad)
    heading_est: list = field(default_factory=list)
    err_est: list = field(default_factory=list)
    target_err_est: list = field(default_factory=list)
    mode: list = field(default_factory=list)
    trk: list = field(default_factory=list)
    flags: list = field(default_factory=list)
    rate_ref: list = field(default_factory=list)
    wheel_cmd: list = field(default_factory=list)
    bias_est: list = field(default_factory=list)
    bias_true: list = field(default_factory=list)
    vis: list = field(default_factory=list)       # tuplos por frame

    def arr(self, name):
        return np.asarray(getattr(self, name), dtype=float)


class Runner:
    def __init__(self, sim, core, ctrl_hz=200, mag_hz=50, cam_hz=20, cam_latency=0.08,
                 cam_jitter=0.008, motor_latency=0.002, dt=0.001, frame_hook=None, keep_log=True):
        self.sim, self.core = sim, core
        self.dt = dt
        self.ctrl_n = int(round(1.0 / (ctrl_hz * dt)))
        self.mag_n = int(round(ctrl_hz / mag_hz))
        self.cam_period = 1.0 / cam_hz
        self.cam_latency, self.cam_jitter = cam_latency, cam_jitter
        self.motor_latency = motor_latency
        self.log = Log()
        self.k = 0
        self.tick = 0
        self.next_cam = 0.0
        self.cam_queue = []           # (t_arrive, t_capture, img, truth_px)
        self.motor_queue = []         # (t_apply, out)
        self.rng = np.random.default_rng(123)
        self.frame_hook = frame_hook  # f(t, img, vision) - p.ex. para o servidor de GS
        self.last_out = None
        self.events = []
        self.keep_log = keep_log

    def at(self, t, fn):
        self.events.append((t, fn))
        self.events.sort(key=lambda e: e[0])

    def run(self, t_end):
        sim, core, dt = self.sim, self.core, self.dt
        L = self.log
        n_end = int(round(t_end / dt))
        while self.k < n_end:
            t = self.k * dt
            while self.events and self.events[0][0] <= t:
                self.events.pop(0)[1](sim, core)

            # driver: aplica comandos com latência
            while self.motor_queue and self.motor_queue[0][0] <= t:
                _, o = self.motor_queue.pop(0)
                sim.set_motor(o.enable, o.wheel_cmd_rps, o.torque_max_nm, o.accel_limit_rps2)

            sim.step(dt)
            self.k += 1
            t = self.k * dt

            # câmara: captura e entrega com latência
            if t >= self.next_cam:
                img = sim.render()
                lat = self.cam_latency + self.cam_jitter * self.rng.standard_normal()
                self.cam_queue.append((t + max(0.03, lat), t, img, sim.truth_px))
                self.next_cam += self.cam_period

            if self.k % self.ctrl_n == 0:
                self.tick += 1
                t_us = int(round(t * 1e6))
                gyro, acc = sim.imu_sample(dt * self.ctrl_n)
                if self.tick % self.mag_n == 0:
                    core.on_mag(t_us, sim.mag_sample())
                while self.cam_queue and self.cam_queue[0][0] <= t:
                    t_arr, t_cap, img, truth = self.cam_queue.pop(0)
                    vis, proc_us = core.on_image(int(round(t_cap * 1e6)), t_us, img)
                    if self.keep_log: L.vis.append((t_cap, vis.found, vis.accepted, vis.x, vis.y, truth[0], truth[1],
                                  vis.err_x, proc_us, vis.n_cand))
                    if self.frame_hook:
                        self.frame_hook(t_cap, img, vis, proc_us)
                out = core.step(t_us, gyro, acc, sim.wheel_speed(), sim.tau_m)
                self.last_out = out
                self.motor_queue.append((t + self.motor_latency,
                                         type("O", (), dict(enable=out.enable, wheel_cmd_rps=out.wheel_cmd_rps,
                                                            torque_max_nm=out.torque_max_nm,
                                                            accel_limit_rps2=out.accel_limit_rps2))()))
                tl = core.tlm()
                if not self.keep_log: continue
                L.t.append(t); L.psi.append(sim.psi); L.w.append(sim.w); L.Om.append(sim.Om)
                L.tau_m.append(sim.tau_m); L.target.append(sim.target_fn(t))
                L.heading_est.append(tl.heading); L.err_est.append(tl.err); L.target_err_est.append(tl.target_err)
                L.mode.append(tl.mode); L.trk.append(tl.trk); L.flags.append(tl.flags)
                L.rate_ref.append(tl.rate_ref); L.wheel_cmd.append(tl.wheel_cmd_rps)
                L.bias_est.append(tl.bias); L.bias_true.append(sim.bias)
        return L
