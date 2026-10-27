#!/usr/bin/env python3
"""
sim_server.py - o "cubo virtual": núcleo C + simulador, acessível por UDP com o protocolo TMTC.

A estação de terra liga-se a isto exactamente como se ligará ao cubo real (mesmas tramas, as geradas
pelo MESMO código C do firmware). Corre em tempo real (ou mais depressa com --speed).

  python sim_server.py --port 5000 --psi0 -30
"""
import argparse
import math
import os
import socket
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cproto as cp                      # noqa: E402
from cube_sim import CubeSim             # noqa: E402
from sil import Core, Runner, CMD_MODE   # noqa: E402

TM_STATE, TM_VISION, TM_IMG, TM_ACK = 0x01, 0x02, 0x03, 0x7F
TC_PING, TC_STREAM_CFG = 0x80, 0x87
SIM_SHOVE, SIM_COVER, SIM_TARGET, SIM_LIGHT = 0xF0, 0xF1, 0xF2, 0xF3


class SimServer:
    def __init__(self, port=5000, speed=1.0, seed=1, psi0=-30.0, host="127.0.0.1"):
        self.sim = CubeSim(seed=seed, psi0_deg=psi0)
        self.core = Core()
        self.speed = speed
        self.state_hz, self.vision_hz, self.img_every = 20, 20, 10
        self.frames = 0
        self.img_id = 0
        self.seq = 0
        self.client = None
        self.parser = cp.CParser()
        self.cover = False
        self.sim.covered_fn = lambda t: self.cover
        self.runner = Runner(self.sim, self.core, frame_hook=self.on_frame, keep_log=False)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.next_state = 0.0

    # ---- saída
    def send(self, type_, payload=b""):
        if self.client:
            self.seq = (self.seq + 1) & 0xFF
            try:
                self.sock.sendto(cp.pack_frame(type_, self.seq, payload), self.client)
            except OSError:
                pass

    def on_frame(self, t_cap, img, vis, proc_us):
        self.frames += 1
        if self.vision_hz > 0:
            self.send(TM_VISION, cp.pack_vision(vis, proc_us))
        if self.img_every > 0 and self.frames % self.img_every == 0:
            self.img_id = (self.img_id + 1) & 0xFF
            for c in range(cp.IMG_NCHUNKS):
                self.send(TM_IMG, cp.pack_img_chunk(img, self.img_id, c))

    # ---- entrada
    def handle(self, type_, seq, payload):
        status = 0
        if type_ == TC_PING:
            pass
        elif type_ == TC_STREAM_CFG and len(payload) == 3:
            self.state_hz, self.vision_hz, self.img_every = payload
        elif type_ == SIM_SHOVE and len(payload) == 4:
            self.sim.shove(struct.unpack("<f", payload)[0], 0.25)
        elif type_ == SIM_COVER and len(payload) == 1:
            self.cover = bool(payload[0])
        elif type_ == SIM_TARGET and len(payload) == 4:
            v = math.radians(struct.unpack("<f", payload)[0])
            self.sim.target_fn = lambda t, v=v: v
        elif type_ == SIM_LIGHT and len(payload) == 4:
            v = struct.unpack("<f", payload)[0]
            self.sim.light_fn = lambda t, v=v: v
        else:
            r, cmd = cp.tc_to_cmd(type_, payload)
            if r == 1:
                self.core.L.cc_command(self.core.h, cp.C.byref(cmd))
            elif r < 0:
                status = -r
            else:
                status = 2
        self.send(TM_ACK, cp.pack_ack(type_, seq, status))

    def poll(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
            except BlockingIOError:
                return
            except ConnectionResetError:
                continue
            self.client = addr
            for type_, seq, payload in self.parser.feed(data):
                self.handle(type_, seq, payload)

    def serve(self):
        print(f"cubo virtual a escutar em UDP {self.sock.getsockname()}  (Ctrl+C para sair)", flush=True)
        t0 = time.perf_counter()
        while True:
            t_target = (time.perf_counter() - t0) * self.speed
            self.poll()
            self.runner.run(t_target)
            if self.state_hz > 0 and self.sim.t >= self.next_state:
                self.send(TM_STATE, cp.pack_state(self.core.tlm()))
                self.next_state = self.sim.t + 1.0 / self.state_hz
            time.sleep(0.002)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--psi0", type=float, default=-30.0, help="rumo inicial do cubo em relação ao alvo (°)")
    a = ap.parse_args()
    try:
        SimServer(a.port, a.speed, a.seed, a.psi0, a.host).serve()
    except KeyboardInterrupt:
        pass
