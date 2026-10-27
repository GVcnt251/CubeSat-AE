"""
cube_sim.py - simulador do cubo (1 eixo de guinada, roda de reacção, gimbal).

Modela o que o guião chama de "o mundo que deixa de cooperar":
  * conservação do momento angular corpo+roda, binário limitado, atrito na roda e nos rolamentos
  * driver moteus (malha de velocidade com limite de binário e de aceleração)
  * IMU (bias + ruído + vibração da roda), magnetómetro (campo terrestre, inclinação, hard-iron)
  * pêndulo em roll/pitch (a suspensão), excitado por empurrões
  * câmara sintética 320x240: poster com ponto, luz variável, sombras, ruído, barra distractora,
    taxa de 20 fps e latência de ~80 ms (feita no runner)

Convenções iguais às do núcleo C (ver cube_core.h): guinada + = direita, NED, x frente.
"""
import math
from dataclasses import dataclass

import numpy as np

G = 9.80665
IMG_W, IMG_H = 320, 240


@dataclass
class SimParams:
    # corpo e roda (kg m^2)
    J_b: float = 0.004
    J_w: float = 2.5e-4
    # motor 5208 Kv330 -> Kt = 8.27/Kv
    kt: float = 8.27 / 330.0
    i_max: float = 10.0
    wheel_visc: float = 1.5e-6
    wheel_coul: float = 3e-4
    # rolamentos do gimbal (a perturbação dominante, segundo o guião)
    gimbal_visc: float = 4e-4
    gimbal_coul: float = 1.5e-3
    # malha de velocidade do driver
    vel_kp: float = 0.05
    vel_ki: float = 2.0
    # IMU (MPU-6050 @ 200 Hz)
    gyro_noise: float = 8.7e-4       # rad/s por amostra
    gyro_bias_rw: float = 2e-5       # rad/s/sqrt(s)
    gyro_bias_max_dps: float = 0.5
    gyro_bias_ramp: float = math.radians(0.1) / 60.0   # rad/s por s: aquecimento do MPU-6050 (0.1 °/s por minuto)
    accel_noise: float = 0.05        # m/s^2
    vib_gain: float = 4e-3           # rad/s de vibração à velocidade máxima da roda
    # magnetómetro (LIS3MDL)
    mag_total_uT: float = 44.0
    mag_incl_deg: float = 55.0
    mag_noise_uT: float = 0.35
    mag_hard_iron: tuple = (0.2, -0.15, 0.25)   # resíduo pós-calibração
    mag_wheel_uT_per_krpm: float = 0.15
    # pêndulo (roll, pitch)
    pend_f_hz: float = 0.8
    pend_zeta: float = 0.04
    pend_noise: float = 0.15         # rad/s^2 rms (ruído de excitação)
    # câmara
    f_px: float = 249.0
    dot_r_px: float = 10.0
    cam_noise: float = 3.0
    poster_level: float = 215.0
    dot_level: float = 25.0
    # mão que empurra o cubo
    hand_kp: float = 10.0
    hand_kd: float = 0.3


def _min_jerk(tau):
    tau = min(max(tau, 0.0), 1.0)
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    ds = 30 * tau**2 - 60 * tau**3 + 30 * tau**4
    return s, ds


class CubeSim:
    def __init__(self, params=None, seed=1, psi0_deg=-30.0, fixed_body=False):
        self.p = params or SimParams()
        self.rng = np.random.default_rng(seed)
        self.fixed_body = fixed_body
        self.t = 0.0
        self.psi = math.radians(psi0_deg)
        self.w = 0.0
        self.Om = 0.0
        self.phi = self.th = self.phid = self.thd = 0.0
        self.bias = math.radians(self.rng.uniform(-1, 1) * self.p.gyro_bias_max_dps)
        self.rot_phase = 0.0

        # motor
        self.enable = False
        self.v_cmd = 0.0
        self.max_tq = 0.1
        self.accel = 800.0
        self.v_c = 0.0
        self.i_int = 0.0
        self.tau_m = 0.0

        # cenário (funções do tempo)
        self.target_fn = lambda t: 0.0       # rumo do alvo (rad)
        self.light_fn = lambda t: 1.0
        self.shadow_fn = lambda t: None      # dict(angle, pos, width, depth) ou None
        self.covered_fn = lambda t: False
        self.hand = None                     # (t0, T, delta_rad, theta_start)
        self.dot_visible = True
        self.truth_px = (math.nan, math.nan)

        # precálculos para renderização
        yy, xx = np.mgrid[0:IMG_H, 0:IMG_W].astype(np.float32)
        self._X, self._Y = xx, yy
        rn = ((xx - IMG_W / 2) / (IMG_W / 2)) ** 2 + ((yy - IMG_H / 2) / (IMG_H / 2)) ** 2
        self._vignette = (1.0 - 0.22 * rn).astype(np.float32)
        self._grad = (1.0 + 0.10 * (xx - IMG_W / 2) / (IMG_W / 2)).astype(np.float32)

        # campo magnético em NED
        inc = math.radians(self.p.mag_incl_deg)
        B = self.p.mag_total_uT
        self.B_ned = np.array([B * math.cos(inc), 0.0, B * math.sin(inc)])

    # ------------------------------------------------------------- actuação
    def set_motor(self, enable, wheel_cmd_rps, torque_max_nm, accel_rps2):
        self.enable = bool(enable)
        self.v_cmd = wheel_cmd_rps
        self.max_tq = torque_max_nm
        self.accel = accel_rps2

    def shove(self, delta_deg, duration=0.25):
        """Empurrão de mão: leva o corpo (min-jerk) +delta_deg em 'duration' s e larga."""
        self.hand = (self.t, duration, math.radians(delta_deg), self.psi)
        sgn = 1.0 if delta_deg >= 0 else -1.0
        self.phid += sgn * 0.25
        self.thd += 0.15

    # ----------------------------------------------------------------- física
    def step(self, dt):
        p = self.p
        # driver: rampa de aceleração + PI de velocidade com limite de binário
        tq_lim = min(self.max_tq, p.i_max * p.kt)
        if self.enable:
            step = self.accel * dt
            self.v_c += max(-step, min(step, self.v_cmd - self.v_c))
            e = self.v_c - self.Om
            self.i_int = max(-tq_lim, min(tq_lim, self.i_int + p.vel_ki * e * dt))
            self.tau_m = max(-tq_lim, min(tq_lim, p.vel_kp * e + self.i_int))
        else:
            self.v_c = self.Om
            self.i_int = 0.0
            self.tau_m = 0.0

        f_wheel = p.wheel_visc * self.Om + p.wheel_coul * math.tanh(self.Om / 0.5)
        tau_w = self.tau_m - f_wheel

        if self.fixed_body:
            wd = 0.0
            self.w = 0.0
        else:
            f_g = -(p.gimbal_visc * self.w + p.gimbal_coul * math.tanh(self.w / 0.005))
            tau_ext = 0.0
            if self.hand is not None:
                t0, T, d, th0 = self.hand
                if self.t > t0 + T:
                    self.hand = None
                else:
                    s, ds = _min_jerk((self.t - t0) / T)
                    tau_ext = p.hand_kp * (th0 + d * s - self.psi) + p.hand_kd * (d * ds / T - self.w)
            wd = (-tau_w + f_g + tau_ext) / p.J_b
        Omd = tau_w / p.J_w - wd

        self.w += wd * dt
        self.Om += Omd * dt
        self.psi += self.w * dt
        self.rot_phase += self.Om * dt

        # pêndulo roll/pitch
        wn = 2 * math.pi * p.pend_f_hz
        for name in ("phi", "th"):
            x, v = getattr(self, name), getattr(self, name + "d")
            a = -wn * wn * x - 2 * p.pend_zeta * wn * v + p.pend_noise * self.rng.standard_normal() / math.sqrt(dt * 1000)
            v += a * dt
            x += v * dt
            setattr(self, name, x)
            setattr(self, name + "d", v)

        self.t += dt

    # ---------------------------------------------------------------- sensores
    def imu_sample(self, dt_ctrl):
        p = self.p
        self.bias += p.gyro_bias_rw * math.sqrt(dt_ctrl) * self.rng.standard_normal() + p.gyro_bias_ramp * dt_ctrl
        vib = p.vib_gain * (abs(self.Om) / 314.0) * math.sin(self.rot_phase)
        cph, sph, cth, sth = math.cos(self.phi), math.sin(self.phi), math.cos(self.th), math.sin(self.th)
        gyro = np.array([self.phid, self.thd, self.w * cph * cth]) + np.array([0, 0, self.bias])
        gyro = gyro + p.gyro_noise * self.rng.standard_normal(3) + np.array([0, 0, vib])
        acc = G * np.array([-sth, sph * cth, cph * cth]) + p.accel_noise * self.rng.standard_normal(3)
        return gyro.astype(np.float32), acc.astype(np.float32)

    def _dcm(self):
        c, s = math.cos, math.sin
        y, t, r = self.psi, self.th, self.phi
        Rz = np.array([[c(y), -s(y), 0], [s(y), c(y), 0], [0, 0, 1]])
        Ry = np.array([[c(t), 0, s(t)], [0, 1, 0], [-s(t), 0, c(t)]])
        Rx = np.array([[1, 0, 0], [0, c(r), -s(r)], [0, s(r), c(r)]])
        return Rz @ Ry @ Rx

    def mag_sample(self):
        p = self.p
        b = self._dcm().T @ self.B_ned + np.array(p.mag_hard_iron)
        b[1] += p.mag_wheel_uT_per_krpm * self.Om * 9.549 / 1000.0
        b = b + p.mag_noise_uT * self.rng.standard_normal(3)
        return b.astype(np.float32)

    def wheel_speed(self):
        return self.Om

    # ----------------------------------------------------------------- câmara
    def render(self):
        """Imagem 320x240 uint8 do instante actual. Guarda em truth_px a posição real do ponto."""
        p = self.p
        t = self.t
        n = self.rng.standard_normal((IMG_H, IMG_W)).astype(np.float32)
        if self.covered_fn(t):
            self.truth_px = (math.nan, math.nan)
            return np.clip(8.0 + 2.0 * n, 0, 255).astype(np.uint8)

        L = float(self.light_fn(t))
        e = self.target_fn(t) - self.psi
        du = p.f_px * math.tan(max(-1.45, min(1.45, e)))
        dv = p.f_px * math.tan(self.th)
        cr, sr = math.cos(self.phi), math.sin(self.phi)
        u = IMG_W / 2 - 0.5 + du * cr - dv * sr
        v = IMG_H / 2 - 0.5 + du * sr + dv * cr
        self.truth_px = (u, v)

        img = p.poster_level * L * self._vignette * self._grad

        sh = self.shadow_fn(t)
        if sh is not None:
            a = sh.get("angle", 0.6)
            s = (self._X * math.cos(a) + self._Y * math.sin(a) - sh["pos"]) / sh["width"]
            img = img * (1.0 - sh["depth"] * np.exp(-s * s))

        # barra distractora (título do poster): fina e comprida, fixa em relação ao ponto
        bar = (np.abs(self._X - u) < 38) & (np.abs(self._Y - (v - 62)) < 3.5)
        img = np.where(bar, p.dot_level * 1.6 * L, img)

        # ponto com bordo suave (anti-aliasing)
        dist = np.sqrt((self._X - u) ** 2 + (self._Y - v) ** 2)
        alpha = np.clip(p.dot_r_px + 0.5 - dist, 0.0, 1.0)
        img = img * (1.0 - alpha) + (p.dot_level * L) * alpha

        img = img + p.cam_noise * n * (0.5 + img / 255.0)
        return np.clip(img, 0, 255).astype(np.uint8)
