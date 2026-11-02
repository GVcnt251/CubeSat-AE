"""SIL de três rodas ortogonais; aproximação de dinâmica desacoplada por eixo.

Três instâncias do mesmo controlador C. X/Y do painel são deslocamentos horizontal/
vertical da pontaria (guinada Z / pitch Y); a roda X estabiliza roll. Não é um modelo
de corpo rígido acoplado nem uma implementação do firmware de três rodas.
"""
import ctypes as C
import math
import struct
import numpy as np

from cube_sim import CubeSim, SimParams, G
from sil import Core, DetectionT, VisionT, CmdT, CMD_MODE, CMD_AIM, CMD_RECAL
from gimbal import GimbalParams, relative_angles
from collections import deque


class ThreeAxisSimulation:
    names = ('roll', 'pitch', 'yaw')

    def __init__(self, seed=1, psi0=-30, frame_hook=None,webcam_only=False):
        self.plants = {name: CubeSim(params=SimParams(pend_noise=0), seed=seed+i*17,
                                    psi0_deg=psi0 if name == 'yaw' else 0)
                       for i, name in enumerate(self.names)}
        self.cores = {name: Core() for name in self.names}
        for core in self.cores.values():core.L.cc_set_gimbal_unload(core.h,1)
        pitch_core=self.cores['pitch']
        pitch_core.L.cc_set_search_limits(pitch_core.h,math.radians(15),math.radians(20))
        self.sim = self.plants['yaw']
        self.sim.three_axis_camera = True
        self.frame_hook = frame_hook
        self.k = 0
        self.next_cam = 0
        self.camera_queue = []
        self.motor_queue = []
        self.rng = np.random.default_rng(seed+123)
        self.gimbal = GimbalParams()
        self.reference = self.physical_angles().copy()
        self.reference_id = 0
        self.rest_since = None
        self.camera_source=None
        self.pose_history=deque(maxlen=160)
        self.webcam_only=webcam_only
        self.centering=False
        self.pending_webcam_track=False
        self.target_pitch=math.nan if webcam_only else 0.
        if webcam_only:self.sim.target_fn=lambda t:math.nan

    def physical_angles(self):
        return np.array([self.plants[name].psi for name in self.names])

    def capture_reference(self):
        if self.rest_since is None or self.sim.t-self.rest_since<.5:
            return False
        if any(p.hand is not None or abs(p.w)>math.radians(.5) for p in self.plants.values()):
            return False
        if any(core.tlm().mode!=1 for core in self.cores.values()):
            return False
        self.reference=self.physical_angles().copy()
        self.reference_id=(self.reference_id+1)&0xFFFF
        return True

    def configure_gimbal(self, parameters):
        if not parameters.valid():
            return False
        self.gimbal=parameters
        com=np.array([parameters.com_x,parameters.com_y,parameters.com_z])
        for i,name in enumerate(self.names):
            # Inércia efetiva inicial de 0,004 kg m² a 1 kg + teorema dos eixos paralelos.
            self.plants[name].p.J_b=.004*parameters.mass_kg+parameters.mass_kg*(float(com @ com)-com[i]**2)
        return True

    def shove(self, x_deg, y_deg=0, duration=0.25, roll_deg=0):
        for name, value in (('yaw', x_deg), ('pitch', -y_deg), ('roll', roll_deg)):
            if value:
                self.plants[name].shove(value, duration)
                self.plants[name].phid = self.plants[name].thd = 0

    def command(self, cmd):
        if cmd.type==CMD_MODE or cmd.type==CMD_RECAL:
            self.centering=False;self.pending_webcam_track=False
        if (cmd.type==CMD_MODE and int(cmd.a)==5 and (self.webcam_only or self.camera_source is not None)
                and (self.camera_source is None or not self.camera_source.found)):
            self.pending_webcam_track=True
            for core in self.cores.values():core.L.cc_point_absolute(core.h,0.)
            return
        for name, core in self.cores.items():
            copy = CmdT(cmd.type, cmd.id, cmd.a, cmd.b)
            if cmd.type == CMD_MODE and int(cmd.a) == 5 and name == 'roll':
                copy.a = 4  # Roll mantém a referência de calibração.
            if cmd.type == CMD_AIM and name != 'yaw':
                copy.a = 0
            core.L.cc_command(core.h, C.byref(copy))

    def center_on_webcam(self):
        if not (self.webcam_only or self.camera_source is not None):return False
        if any(c.tlm().mode==0 for c in self.cores.values()):return False
        for core in self.cores.values():
            if not core.L.cc_point_absolute(core.h,0.):return False
        self.centering=True
        self.pending_webcam_track=False
        return True

    def set_camera_source(self,source):
        old=self.camera_source
        self.camera_source=source
        if old is not None and old is not source:old.close()
        focal=source.f_virtual if source is not None else 249.
        for name in ('yaw','pitch'):
            core=self.cores[name]
            core.L.cc_set_camera_model(core.h,focal,focal)
            core.L.cc_clear_target(core.h)
        self.camera_queue.clear()
        self.next_cam=self.sim.t
        if source is None:
            self.target_pitch=math.nan if self.webcam_only else 0.
            self.sim.target_fn=(lambda t:math.nan) if self.webcam_only else (lambda t:0.)
        else:
            self.target_pitch=math.nan
            self.sim.target_fn=lambda t:math.nan
        if source is None and self.webcam_only and not self.centering and any(c.tlm().mode==5 for c in self.cores.values()):
            self.command(CmdT(CMD_MODE,0,5,0))

    def _pose_at(self,t):
        if not self.pose_history:return self.physical_angles()
        previous=self.pose_history[0]
        for stamp,angles in self.pose_history:
            if stamp>=t:
                if stamp==previous[0]:return angles
                f=np.clip((t-previous[0])/(stamp-previous[0]),0,1)
                return previous[1]+f*(angles-previous[1])
            previous=(stamp,angles)
        return previous[1]

    def _sensors(self):
        s = self.sim
        s.phi, s.phid = self.plants['roll'].psi, self.plants['roll'].w
        s.th, s.thd = self.plants['pitch'].psi, self.plants['pitch'].w
        measured = {}
        for name, p in self.plants.items():
            gyro, acc = p.imu_sample(0.005)
            # Modelo de canais angulares desacoplados, com bias/ruído/vibração reais do simulador.
            value = float(gyro[2])
            if name == 'yaw':
                value += p.w * (1-math.cos(s.phi)*math.cos(s.th))
                shared_acc = acc
            measured[name] = value
        roll = math.atan2(shared_acc[1], shared_acc[2])
        pitch = math.atan2(-shared_acc[0], math.hypot(shared_acc[1], shared_acc[2]))
        mag = s.mag_sample()
        sr, cr, sp, cp = math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch)
        mx = mag[0]*cp + mag[1]*sr*sp + mag[2]*cr*sp
        my = mag[1]*cr - mag[2]*sr
        yaw = math.atan2(-my, mx)
        return measured, dict(roll=roll, pitch=pitch, yaw=yaw)

    def _camera(self, tcap, now, image,observation=None):
        if observation is not None:
            yaw_d,pitch_d,proc=observation
            vis=VisionT();pitch_vis=VisionT()
            for name,d,out in (('yaw',yaw_d,vis),('pitch',pitch_d,pitch_vis)):
                core=self.cores[name]
                core.L.cc_on_detection(core.h,round(tcap*1e6),round(now*1e6),C.byref(d),C.byref(out))
            if self.frame_hook:self.frame_hook(tcap,image,vis,proc)
            return
        vis, proc = self.cores['yaw'].on_image(round(tcap*1e6), round(now*1e6), image)
        d = DetectionT()
        d.found, d.n_cand = vis.found, vis.n_cand
        # Mesmo ponto detectado; o eixo vertical da imagem tem sinal oposto ao pitch.
        d.x, d.y = 159.5 - (vis.y-119.5), 119.5
        d.area, d.contrast = vis.area, vis.contrast
        v = VisionT()
        core = self.cores['pitch']
        core.L.cc_on_detection(core.h, round(tcap*1e6), round(now*1e6), C.byref(d), C.byref(v))
        if self.frame_hook:
            self.frame_hook(tcap, image, vis, proc)

    def run(self, end):
        while self.k < round(end / .001):
            now = self.k * .001
            while self.motor_queue and self.motor_queue[0][0] <= now:
                _, outputs = self.motor_queue.pop(0)
                for name, output in outputs.items():
                    self.plants[name].set_motor(*output)
            if self.gimbal.com_x==0 and self.gimbal.com_y==0 and self.gimbal.com_z==0:
                gravity_torque=(0.,0.,0.) # Braço nulo: o binário é exatamente zero em qualquer atitude.
            else:gravity_torque,_=self.gimbal.loads(self.physical_angles())
            for index,name in enumerate(self.names):
                p=self.plants[name]
                p.external_torque_nm=float(gravity_torque[index])
                p.step(.001)
            self.k += 1
            now = self.k * .001
            self.sim.phi = self.plants['roll'].psi
            self.sim.th = self.plants['pitch'].psi
            self.sim.phid = self.plants['roll'].w
            self.sim.thd = self.plants['pitch'].w
            if now >= self.next_cam:
                if self.camera_source is None:
                    if self.webcam_only:
                        d=DetectionT()
                        self.camera_queue.append((now+.08,now,np.full((240,320,3),225,np.uint8),(d,d,0)))
                    else:self.camera_queue.append((now+.08, now, self.sim.render(),None))
                else:
                    state,packet=self.camera_source.acquire(now)
                    if state=='fresh':
                        tcap,d,proc=packet
                        yaw_d,pitch_d=self.camera_source.observe(self._pose_at(tcap),d)
                        image=self.camera_source.virtual
                        if self.sim.covered_fn(now):
                            yaw_d.found=pitch_d.found=0
                            image=np.full((240,320),8,np.uint8)
                        if d.found:
                            target_yaw,self.target_pitch=self.camera_source.target
                            self.sim.target_fn=lambda t,v=target_yaw:v
                            if self.pending_webcam_track:self.command(CmdT(CMD_MODE,0,5,0))
                        else:
                            self.target_pitch=math.nan;self.sim.target_fn=lambda t:math.nan
                        self.camera_queue.append((now+.08,tcap,image,(yaw_d,pitch_d,proc)))
                    elif state=='missing':
                        d=DetectionT()
                        self.target_pitch=math.nan;self.sim.target_fn=lambda t:math.nan
                        self.camera_queue.append((now+.08,now,np.full((240,320,3),225,np.uint8),(d,d,0)))
                self.next_cam += .05
            if self.k % 5:
                continue
            rates, angles = self._sensors()
            self.pose_history.append((now,self.physical_angles().copy()))
            if self.k % 20 == 0:
                for name, core in self.cores.items():
                    angle = angles[name]
                    # Adaptador: rumo magnético para yaw; inclinação gravítica para roll/pitch.
                    core.on_mag(round(now*1e6), [25.24*math.cos(angle), -25.24*math.sin(angle), 36.04])
            while self.camera_queue and self.camera_queue[0][0] <= now:
                _, tcap, image, observation = self.camera_queue.pop(0)
                self._camera(tcap, now, image,observation)
            outputs = {}
            for name, core in self.cores.items():
                p = self.plants[name]
                out = core.step(round(now*1e6), [0, 0, rates[name]], [0, 0, G], p.Om, p.tau_m)
                outputs[name] = (out.enable, out.wheel_cmd_rps, out.torque_max_nm, out.accel_limit_rps2)
            self.motor_queue.append((now+.002, outputs))
            stationary=all(core.tlm().mode==1 for core in self.cores.values()) and all(
                p.hand is None and abs(p.w)<math.radians(.5) for p in self.plants.values())
            if stationary:
                if self.rest_since is None:self.rest_since=now
            else:self.rest_since=None

    def telemetry(self):
        tlm = [self.cores[name].tlm() for name in self.names]
        sat = sum((1 << i) for i, t in enumerate(tlm) if t.flags & 2)
        sat |= sum((1 << (i+3)) for i,t in enumerate(tlm) if t.flags & 32)
        groups = [
            [math.degrees(t.heading) for t in tlm],
            [math.degrees(t.rate) for t in tlm],
            [t.wheel_rps*60/(2*math.pi) for t in tlm],
            [t.wheel_cmd_rps*60/(2*math.pi) for t in tlm],
            [t.torque_nm for t in tlm],
            [math.degrees(t.err) for t in tlm],
            [0, math.degrees(tlm[1].heading+tlm[1].target_err), math.degrees(tlm[2].heading+tlm[2].target_err)],
        ]
        return struct.pack('<IB21f', round(self.sim.t*1000), sat, *(v for group in groups for v in group))

    def gimbal_telemetry(self):
        angles=self.physical_angles()
        relative,planes=relative_angles(angles,self.reference)
        torque,body_g=self.gimbal.loads(angles)
        groups=[np.degrees(angles),np.degrees(self.reference),np.degrees(relative),
                np.degrees(planes),torque,body_g,[0,math.degrees(self.target_pitch),math.degrees(self.sim.target_fn(self.sim.t))]]
        params=self.gimbal
        values=[float(v) for group in groups for v in group]
        values += [params.mass_kg,params.g,params.com_x,params.com_y,params.com_z]
        return struct.pack('<IH26f',round(self.sim.t*1000),self.reference_id,*values)
