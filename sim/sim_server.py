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
from three_axis import ThreeAxisSimulation
from gimbal import GimbalParams

TM_STATE, TM_VISION, TM_IMG, TM_ACK = 0x01, 0x02, 0x03, 0x7F
TC_PING, TC_STREAM_CFG = 0x80, 0x87
SIM_SHOVE, SIM_COVER, SIM_TARGET, SIM_LIGHT = 0xF0, 0xF1, 0xF2, 0xF3
SIM_SHOVE_XY, TM_SIM3 = 0xF4, 0x04
SIM_GIMBAL_CFG, SIM_REST, TM_GIMBAL = 0xF5, 0xF6, 0x05
SIM_CAMERA, TM_CAMERA, TM_VIDEO = 0xF7, 0x06, 0x07
SIM_CAMERA_CENTER=0xF8


class SimServer:
    def __init__(self, port=5000, speed=1.0, seed=1, psi0=-30.0, host="127.0.0.1",webcam_only=False):
        self.physics = ThreeAxisSimulation(seed=seed, psi0=psi0, frame_hook=self.on_frame,webcam_only=webcam_only)
        self.sim = self.physics.sim
        self.core = self.physics.cores['yaw']
        self.speed = speed
        self.state_hz, self.vision_hz, self.img_every = 20, 20, 10
        self.frames = 0
        self.img_id = 0
        self.seq = 0
        self.client = None
        self.parser = cp.CParser()
        self.cover = False
        self.sim.covered_fn = lambda t: self.cover
        self.runner = self.physics
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.next_state = 0.0
        self.next_vision = 0.0
        self.camera_error=''
        self.video_id=0

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
        if self.vision_hz > 0 and self.sim.t >= self.next_vision:
            self.send(TM_VISION, cp.pack_vision(vis, proc_us))
            self.next_vision = self.sim.t + 1.0 / self.vision_hz - 1e-9
        if self.img_every > 0 and self.frames % self.img_every == 0:
            source=self.physics.camera_source
            if source is not None:
                self.send_video(1,img)
                if source.raw is not None:self.send_video(0,source.raw)
                return
            self.img_id = (self.img_id + 1) & 0xFF
            if img.ndim==3:img=img[:,:,0]
            for c in range(cp.IMG_NCHUNKS):
                self.send(TM_IMG, cp.pack_img_chunk(img, self.img_id, c))

    def send_video(self,view,image):
        source=self.physics.camera_source
        ok,encoded=source.cv.imencode('.jpg',image,[source.cv.IMWRITE_JPEG_QUALITY,75])
        if not ok:return
        data=encoded.tobytes()
        self.video_id=(self.video_id+1)&0xFFFF
        count=(len(data)+229)//230
        for chunk in range(count):
            header=struct.pack('<HBHH',self.video_id,view,chunk,count)
            self.send(TM_VIDEO,header+data[chunk*230:(chunk+1)*230])

    def camera_telemetry(self):
        source=self.physics.camera_source
        intent=1 if self.physics.centering else (2 if self.physics.pending_webcam_track else 0)
        if source is None:
            mode=2 if self.physics.webcam_only else 0
            status=4 if self.camera_error else (5 if mode==2 else 0)
            message=self.camera_error or ('Modo webcam: liga a câmara. Não existe alvo sintético.' if mode==2 else '')
            return struct.pack('<BBBBB8f',mode,status,0,0,intent,60,40,0,0,0,0,0,-1)+message.encode('utf-8')[:180]
        floats=(source.hfov,source.virtual_fov,*source.source_xy,
                math.degrees(source.target[0]),math.degrees(source.target[1]),source.coverage,source.last_age)
        return struct.pack('<BBBBB8f',1,source.last_status,int(source.found),int(source.inside),intent,*floats)+source.message.encode('utf-8')[:180]

    def configure_camera(self,enabled,index=0,hfov=60,virtual_fov=40):
        self.camera_error=''
        if not enabled:
            self.physics.set_camera_source(None)
            return True
        if index not in range(10) or not (40<=hfov<=120 and 20<=virtual_fov<=70 and virtual_fov<hfov-5):return False
        try:
            from webcam import WebcamView
            if self.physics.camera_source is not None:self.physics.set_camera_source(None)
            source=WebcamView(index,hfov,virtual_fov)
            self.physics.set_camera_source(source)
            return True
        except Exception as exc:
            self.camera_error=str(exc)
            return False

    # ---- entrada
    def handle(self, type_, seq, payload):
        status = 0
        if type_ == TC_PING:
            status = 0 if len(payload) == 0 else 1
        elif type_ == TC_STREAM_CFG:
            if len(payload) != 3:
                status = 1
            else:
                self.state_hz, self.vision_hz, self.img_every = payload
                self.next_state = self.next_vision = 0.0
        elif type_ == SIM_SHOVE and len(payload) == 4:
            value = struct.unpack("<f", payload)[0]
            if math.isfinite(value) and abs(value) <= 360:
                self.physics.shove(value)
            else:
                status = 3
        elif type_ == SIM_SHOVE_XY:
            if len(payload) != 8:
                status = 1
            else:
                x, y = struct.unpack('<ff', payload)
                if not all(math.isfinite(v) for v in (x, y)) or abs(x) > 180 or abs(y) > 45:
                    status = 3
                else:
                    self.physics.shove(x, y)
        elif type_ == SIM_GIMBAL_CFG:
            if len(payload)!=16:
                status=1
            elif any(c.tlm().mode!=1 for c in self.physics.cores.values()):
                status=3
            else:
                mass,x,y,z=struct.unpack('<4f',payload)
                status=0 if self.physics.configure_gimbal(GimbalParams(mass_kg=mass,com_x=x,com_y=y,com_z=z)) else 3
        elif type_ == SIM_REST:
            if payload:
                status=1
            else:
                status=0 if self.physics.capture_reference() else 3
        elif type_ == SIM_CAMERA:
            if len(payload)!=10:
                status=1
            else:
                enabled,index,hfov,vfov=struct.unpack('<BBff',payload)
                status=0 if enabled in (0,1) and self.configure_camera(enabled,index,hfov,vfov) else 3
        elif type_ == SIM_CAMERA_CENTER:
            if payload:status=1
            else:status=0 if self.physics.center_on_webcam() else 3
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
                self.physics.command(cmd)
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
            # UDP: uma trama completa por datagrama; não juntar pacotes/clientes.
            parser = cp.CParser()
            frames = list(parser.feed(data))
            if len(frames) != 1 or len(data) != len(frames[0][2]) + 7:
                continue
            self.client = addr
            for type_, seq, payload in frames:
                self.handle(type_, seq, payload)

    def serve(self,stop_event=None):
        print(f"cubo virtual a escutar em UDP {self.sock.getsockname()}  (Ctrl+C para sair)", flush=True)
        t0 = time.perf_counter()
        try:
            while stop_event is None or not stop_event.is_set():
                t_target = (time.perf_counter() - t0) * self.speed
                self.poll()
                self.runner.run(t_target)
                if self.state_hz > 0 and self.sim.t >= self.next_state:
                    self.send(TM_STATE, cp.pack_state(self.core.tlm()))
                    self.send(TM_SIM3, self.physics.telemetry())
                    self.send(TM_GIMBAL, self.physics.gimbal_telemetry())
                    self.send(TM_CAMERA,self.camera_telemetry())
                    self.next_state = self.sim.t + 1.0 / self.state_hz
                time.sleep(0.002)
        finally:
            source=self.physics.camera_source
            if source is not None:source.close()
            self.sock.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--psi0", type=float, default=-30.0, help="rumo inicial do cubo em relação ao alvo (°)")
    ap.add_argument('--webcam-only',action='store_true')
    a = ap.parse_args()
    try:
        SimServer(a.port, a.speed, a.seed, a.psi0, a.host,a.webcam_only).serve()
    except KeyboardInterrupt:
        pass
