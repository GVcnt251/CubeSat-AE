#!/usr/bin/env python3
"""
gs_app.py - Estação de terra do cubo (PyQt5 + pyqtgraph) com Modelo 3D 1U Cúbico.
Totalmente compatível com TMTC_ICD.md (TM_STATE de 52 bytes).
"""
import argparse
import csv
import math
import sys
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets

import gs_protocol as P
from gs_link import SerialLink, UdpLink

HIST = 600  # amostras por gráfico (30 s a 20 Hz)


class PoseBuffer:
    """Interpola telemetria a 20 Hz numa linha temporal com 100 ms de atraso visual."""

    def __init__(self, delay=0.10):
        self.delay = delay
        self.samples = deque(maxlen=120)
        self.clock_offset = None
        self.last_ms = None
        self.last_rx = None

    @staticmethod
    def unwrap(angle, previous):
        return previous + (angle - previous + 180.0) % 360.0 - 180.0

    def push(self, t_ms, yaw, rpm, target, now):
        if not all(math.isfinite(v) for v in (yaw, rpm)):
            return
        if target is not None and not math.isfinite(target):
            target = None
        # Um relógio que recua indica reinício/recalibração do emissor.
        if self.last_ms is not None and t_ms == self.last_ms:
            return
        if (self.last_ms is not None and t_ms < self.last_ms) or (
                self.last_rx is not None and now - self.last_rx > 0.5):
            self.samples.clear()
            self.clock_offset = None
        if self.clock_offset is None:
            self.clock_offset = now - t_ms / 1000.0
        if self.samples:
            yaw = self.unwrap(yaw, self.samples[-1][1])
            if target is not None and self.samples[-1][3] is not None:
                target = self.unwrap(target, self.samples[-1][3])
        self.samples.append((t_ms / 1000.0 + self.clock_offset, yaw, rpm, target))
        self.last_ms, self.last_rx = t_ms, now

    def pose(self, now):
        if not self.samples:
            return None
        t = now - self.delay
        while len(self.samples) > 2 and self.samples[1][0] <= t:
            self.samples.popleft()
        a = self.samples[0]
        if t <= a[0] or len(self.samples) == 1:
            return a[1:]
        b = self.samples[1]
        f = max(0.0, min(1.0, (t - a[0]) / (b[0] - a[0])))
        target = None
        if a[3] is not None and b[3] is not None:
            target = a[3] + f * (b[3] - a[3])
        elif f >= 1.0:
            target = b[3]
        return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), target)


class Cube3DWidget(QtWidgets.QWidget):
    """Renderizador 3D de um CubeSat 1U (cubo perfeito 1:1:1) em PyQt5 puro."""
    orientation_changed = QtCore.pyqtSignal(object,object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(240)
        self.yaw_deg = 0.0
        self.wheel_phase = 0.0
        self.wheel_rpm = 0.0
        self.target_yaw_deg = None
        self.roll_deg = self.pitch_deg = self.target_pitch_deg = 0.0
        self.tilts = PoseBuffer()
        self.wheel_speeds = (0.0, 0.0, 0.0)
        self.other_phases = [0.0, 0.0]
        self.has_gimbal = False
        self.gimbal_reference = (0.0,0.0,0.0)
        self.com = (0.0,0.0,0.0)
        self.gravity = 9.80665
        self.cam_elev = 26.0
        self.cam_azim = -38.0
        self.zoom = 1.0
        self._last_mouse = None
        self.poses = PoseBuffer()
        self._last_frame = time.monotonic()
        self.animation = QtCore.QTimer(self)
        self.animation.setTimerType(QtCore.Qt.PreciseTimer)
        self.animation.timeout.connect(self.animate)
        self.animation.start(16)

    def set_state(self, yaw_deg, wheel_rpm, target_yaw_deg=None, t_ms=None):
        now = time.monotonic()
        self.poses.push(now * 1000.0 if t_ms is None else t_ms,
                        float(yaw_deg), float(wheel_rpm), target_yaw_deg, now)

    def animate(self):
        now = time.monotonic()
        dt = min(0.1, max(0.0, now - self._last_frame))
        self._last_frame = now
        pose = self.poses.pose(now)
        if pose is None:
            return
        self.yaw_deg, self.wheel_rpm, self.target_yaw_deg = pose
        tilt = self.tilts.pose(now)
        if tilt:
            self.roll_deg, self.pitch_deg, self.target_pitch_deg = tilt
        if self.has_gimbal:
            self.orientation_changed.emit(*self.relative_geometry())
        # Velocidade visual comprimida para se perceber o sentido de rotação da roda
        if now - self.poses.last_rx < 0.5:
            self.wheel_phase = (self.wheel_phase + (self.wheel_rpm / 60.0) * 0.25 * 2 * math.pi * dt) % (2 * math.pi)
            for i in range(2):
                self.other_phases[i] = (self.other_phases[i] + self.wheel_speeds[i]/60 * .25*2*math.pi*dt) % (2*math.pi)
        self.update()

    def set_three_axis(self, state):
        self.wheel_speeds = state.wheels
        self.wheel_rpm=state.wheels[2]
        if self.has_gimbal:
            return
        self.set_state(state.attitude[2], state.wheels[2], state.targets[2], state.t_ms)
        self.tilts.push(state.t_ms, state.attitude[0], state.attitude[1],
                        state.targets[1], time.monotonic())
        self.wheel_speeds = state.wheels

    def set_gimbal_state(self,state):
        if not self.has_gimbal:
            self.poses=PoseBuffer();self.tilts=PoseBuffer()
        self.has_gimbal=True
        self.gimbal_reference=state.reference
        self.com=(state.com_x,state.com_y,state.com_z)
        self.gravity=state.g
        self.set_state(state.attitude[2],self.wheel_rpm,state.targets[2],state.t_ms)
        self.tilts.push(state.t_ms,state.attitude[0],state.attitude[1],state.targets[1],time.monotonic())

    def rotation_matrix(self,angles=None):
        if angles is None:angles=(self.roll_deg,self.pitch_deg,self.yaw_deg)
        r, p, y = map(math.radians, angles)
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array(
            [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])

    def relative_geometry(self):
        relative=self.rotation_matrix(self.gimbal_reference).T @ self.rotation_matrix()
        p=math.asin(float(np.clip(-relative[2,0],-1,1)))
        if abs(math.cos(p))>1e-7:
            r,y=math.atan2(relative[2,1],relative[2,2]),math.atan2(relative[1,0],relative[0,0])
        else:
            r,y=0,math.atan2(-relative[0,1],relative[1,1])
        planes=np.degrees(np.arccos(np.clip(np.abs(np.diag(relative)[[2,1,0]]),0,1)))
        return tuple(np.degrees([r,p,y])),tuple(planes)

    def _draw_path(self,qp,points,w,h,color,width=1.5,dashed=False):
        projected=self._project(np.asarray(points),w,h)
        pen=QtGui.QPen(QtGui.QColor(color),width)
        if dashed:pen.setStyle(QtCore.Qt.DashLine)
        qp.setPen(pen);qp.setBrush(QtCore.Qt.NoBrush)
        qp.drawPolyline(QtGui.QPolygonF([QtCore.QPointF(*p[:2]) for p in projected]))

    def _draw_mount(self,qp,w,h):
        # Base e apoios fixos; translação do pivô é impedida pelo suporte.
        self._draw_path(qp,[[-.55,-1.1,1.25],[.55,-1.1,1.25],[.55,1.1,1.25],[-.55,1.1,1.25],[-.55,-1.1,1.25]],w,h,'#b0bdcd',3)
        for sign in (-1,1):
            self._draw_path(qp,[[0,sign*1.05,1.25],[0,sign*1.05,0]],w,h,'#7c8fa8',4)
        a=np.linspace(0,2*math.pi,97)
        c,s=np.cos(a),np.sin(a)
        yaw=self.rotation_matrix((0,0,self.yaw_deg))
        pitch=self.rotation_matrix((0,self.pitch_deg,self.yaw_deg))
        for points,color in ((np.column_stack((1.05*c,1.05*s,np.zeros_like(c))),'#8b9bb1'),
                             (np.column_stack((.93*c,np.zeros_like(c),.93*s)) @ yaw.T,'#6f9ec4'),
                             (np.column_stack((np.zeros_like(c),.82*c,.82*s)) @ pitch.T,'#62a89f')):
            self._draw_path(qp,points,w,h,color,2)
        # Silhueta inicial e triádade de referência: não acompanham o cubo.
        reference=self.rotation_matrix(self.gimbal_reference)
        vertices=np.array([[x,y,z] for x in (-.51,.51) for y in (-.51,.51) for z in (-.51,.51)])
        for i,v in enumerate(vertices):
            for j,u in enumerate(vertices):
                if j>i and np.count_nonzero(v!=u)==1:
                    self._draw_path(qp,np.array([v,u]) @ reference.T,w,h,'#a1afc1',1,True)
        for i,color in enumerate(('#cb7777','#62a89f','#648fcd')):
            end=reference[:,i]*.75
            self._draw_path(qp,[[0,0,0],end],w,h,color,1,True)

    def _draw_gravity(self,qp,w,h):
        start=np.array([.9,-.9,-.7]);end=np.array([.9,-.9,.75])
        p=self._project(np.array([start,end]),w,h)
        qp.setPen(QtGui.QPen(QtGui.QColor('#ad68b5'),2))
        qp.drawLine(QtCore.QPointF(*p[0,:2]),QtCore.QPointF(*p[1,:2]))
        angle=math.atan2(*(p[1,:2]-p[0,:2])[::-1])
        for side in (-.5,.5):
            qp.drawLine(QtCore.QPointF(*p[1,:2]),QtCore.QPointF(p[1,0]-8*math.cos(angle+side),p[1,1]-8*math.sin(angle+side)))
        qp.drawText(QtCore.QPointF(p[0,0]+5,p[0,1]),f'g = {self.gravity:.2f} m/s²')
        # Centro geométrico (pivô) e posição física do centro de massa (1 unidade = 0,1 m).
        com=np.array(self.com)*10 @ self.rotation_matrix().T
        center=self._project(np.array([[0,0,0],com]),w,h)
        qp.setPen(QtGui.QPen(QtGui.QColor('#ad68b5'),1))
        qp.drawLine(QtCore.QPointF(*center[0,:2]),QtCore.QPointF(*center[1,:2]))
        qp.setBrush(QtGui.QColor('#ad68b5'))
        qp.drawEllipse(QtCore.QPointF(*center[1,:2]),3.5,3.5)

    def pointing_geometry(self):
        """Eixo óptico e alvo no mesmo plano e raio: erro zero => pontos coincidentes."""
        axis = np.array([[r, 0, 0] for r in (0.5, 1.15)]) @ self.rotation_matrix().T
        target = None
        if self.target_yaw_deg is not None:
            ty = math.radians(self.target_yaw_deg)
            tp = math.radians(self.target_pitch_deg or 0)
            target = np.array([[1.15 * math.cos(tp)*math.cos(ty), 1.15 * math.cos(tp)*math.sin(ty), -1.15*math.sin(tp)]])
        return axis, target

    def mousePressEvent(self, e):
        self._last_mouse = e.pos()

    def mouseMoveEvent(self, e):
        if self._last_mouse is not None:
            dx = e.x() - self._last_mouse.x()
            dy = e.y() - self._last_mouse.y()
            self.cam_azim += dx * 0.5
            self.cam_elev = max(-80.0, min(80.0, self.cam_elev + dy * 0.5))
            self._last_mouse = e.pos()
            self.update()

    def mouseReleaseEvent(self, e):
        self._last_mouse = None

    def wheelEvent(self, e):
        delta = e.angleDelta().y()
        self.zoom = max(0.5, min(2.2, self.zoom * (1.1 if delta > 0 else 0.9)))
        self.update()

    def _project(self, pts, w, h):
        # Converte NED (x frente, y direita, z baixo) para vista de câmara
        az = math.radians(self.cam_azim)
        el = math.radians(self.cam_elev)
        ca, sa = math.cos(az), math.sin(az)
        ce, se = math.cos(el), math.sin(el)

        x, y, z = pts[:, 0], pts[:, 1], -pts[:, 2]  # z para cima no ecrã
        x1 = x * ca - y * sa
        y1 = x * sa + y * ca
        z1 = z
        x2 = x1 * ce - z1 * se
        z2 = x1 * se + z1 * ce
        y2 = y1

        dist = 4.2
        scale = min(w, h) * (0.31 if self.has_gimbal else 0.42) * self.zoom
        f = dist / np.maximum(1.2, dist - x2)
        sx = w * 0.5 + y2 * f * scale
        sy = h * 0.52 - z2 * f * scale
        return np.column_stack((sx, sy, x2))

    def paintEvent(self, _):
        qp = QtGui.QPainter(self)
        qp.setRenderHint(QtGui.QPainter.Antialiasing, True)
        w, h = self.width(), self.height()
        qp.fillRect(0, 0, w, h, QtGui.QColor(245, 247, 250))
        if self.has_gimbal:
            self._draw_mount(qp,w,h)

        # 1. Anel de pontaria no mesmo plano horizontal do eixo óptico e alvo.
        n_ring = 36
        ang = np.linspace(0, 2 * np.pi, n_ring, endpoint=False)
        ring_pts = np.column_stack((1.15 * np.cos(ang), 1.15 * np.sin(ang), np.zeros(n_ring)))
        pr = self._project(ring_pts, w, h)
        qp.setPen(QtGui.QPen(QtGui.QColor(180, 190, 205), 1.5))
        poly = QtGui.QPolygonF([QtCore.QPointF(pr[i, 0], pr[i, 1]) for i in range(n_ring)])
        qp.drawPolygon(poly)

        # 2. Cubo 1U (proporção 1:1:1 -> meias-dimensões L = 0.50 em todos os eixos!)
        L = 0.50
        verts_body = np.array([
            [ L, -L, -L], [ L,  L, -L], [ L,  L,  L], [ L, -L,  L],  # Frente (+X, câmara)
            [-L, -L, -L], [-L,  L, -L], [-L,  L,  L], [-L, -L,  L],  # Trás (-X)
        ], dtype=float)

        psi = math.radians(self.yaw_deg)
        cy, sy = math.cos(psi), math.sin(psi)
        Rz = self.rotation_matrix()
        verts_w = verts_body @ Rz.T
        pv = self._project(verts_w, w, h)

        faces = [
            ([0, 1, 2, 3], QtGui.QColor(245, 140, 40, 220)),   # Frente +X (Câmara - Laranja)
            ([4, 7, 6, 5], QtGui.QColor(70, 95, 130, 210)),    # Trás -X
            ([0, 4, 5, 1], QtGui.QColor(95, 125, 165, 210)),   # Topo -Z
            ([3, 2, 6, 7], QtGui.QColor(60, 80, 110, 210)),    # Base +Z
            ([1, 5, 6, 2], QtGui.QColor(80, 110, 150, 210)),   # Direita +Y
            ([0, 3, 7, 4], QtGui.QColor(80, 110, 150, 210)),   # Esquerda -Y
        ]
        # Ordena faces por profundidade (algoritmo do pintor)
        faces_sorted = sorted(faces, key=lambda f: np.mean(pv[f[0], 2]))
        for idx_list, col in faces_sorted:
            qpoly = QtGui.QPolygonF([QtCore.QPointF(pv[i, 0], pv[i, 1]) for i in idx_list])
            qp.setBrush(QtGui.QBrush(col))
            qp.setPen(QtGui.QPen(QtGui.QColor(30, 40, 55), 1.5))
            qp.drawPolygon(qpoly)

        # 3. Roda de reação no interior/topo para visualizar rotação
        rw_r = 0.28
        n_rw = 16
        a_rw = np.linspace(0, 2 * np.pi, n_rw, endpoint=False) + self.wheel_phase
        rw_body = np.column_stack((rw_r * np.cos(a_rw), rw_r * np.sin(a_rw), np.full(n_rw, -L - 0.02)))
        prw = self._project(rw_body @ Rz.T, w, h)
        qp.setPen(QtGui.QPen(QtGui.QColor(30, 30, 30), 2))
        qp.setBrush(QtGui.QBrush(QtGui.QColor(220, 225, 230)))
        qp.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(prw[i, 0], prw[i, 1]) for i in range(n_rw)]))
        # Raios da roda
        c_top = self._project(np.array([[0.0, 0.0, -L - 0.02]]) @ Rz.T, w, h)[0]
        qp.setPen(QtGui.QPen(QtGui.QColor(70, 80, 95), 2))
        for k in range(0, n_rw, 4):
            qp.drawLine(QtCore.QPointF(c_top[0], c_top[1]), QtCore.QPointF(prw[k, 0], prw[k, 1]))
        # Duas rodas ortogonais adicionais: X (roll) e Y (pitch).
        for i, color in enumerate((QtGui.QColor('#22b7a4'), QtGui.QColor('#7d80e8'))):
            angles = np.linspace(0, 2*math.pi, 24, endpoint=False) + self.other_phases[i]
            a, b = .24*np.cos(angles), .24*np.sin(angles)
            points = np.column_stack((np.full(24, -.52), a, b)) if i == 0 else np.column_stack((a, np.full(24, .52), b))
            projected = self._project(points @ Rz.T, w, h)
            qp.setBrush(QtGui.QBrush(QtGui.QColor(color.red(), color.green(), color.blue(), 130)))
            qp.setPen(QtGui.QPen(color, 2))
            qp.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(*pt[:2]) for pt in projected]))
            qp.drawLine(QtCore.QPointF(*projected[0, :2]), QtCore.QPointF(*projected[12, :2]))

        # 4. Eixo de visão da câmara (+X do cubo)
        lens_pts, target = self.pointing_geometry()
        pl = self._project(lens_pts, w, h)
        red = QtGui.QColor(220, 40, 40)
        qp.setPen(QtGui.QPen(red, 3))
        start, tip = QtCore.QPointF(*pl[0, :2]), QtCore.QPointF(*pl[1, :2])
        qp.drawLine(start, tip)
        angle = math.atan2(tip.y() - start.y(), tip.x() - start.x())
        for side in (-0.5, 0.5):
            qp.drawLine(tip, QtCore.QPointF(tip.x() - 9 * math.cos(angle + side),
                                          tip.y() - 9 * math.sin(angle + side)))

        # 5. Alvo estimado (ponto preto no anel exterior)
        if target is not None:
            t_pt = self._project(target, w, h)[0]
            qp.setBrush(QtGui.QBrush(QtGui.QColor(255, 255, 255)))
            qp.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0), 2))
            qp.drawEllipse(QtCore.QPointF(t_pt[0], t_pt[1]), 9, 9)
            qp.setBrush(QtGui.QBrush(QtGui.QColor(0, 0, 0)))
            qp.drawEllipse(QtCore.QPointF(t_pt[0], t_pt[1]), 4.5, 4.5)

        qp.setPen(QtGui.QColor(50, 60, 75))
        qp.setFont(QtGui.QFont("monospace", 8))
        qp.drawText(8, 16, f"Rumo {self.yaw_deg:+.1f}° | Pitch {self.pitch_deg:+.1f}° | Roll {self.roll_deg:+.1f}°")
        if self.has_gimbal:
            self._draw_gravity(qp,w,h)
        qp.setPen(QtGui.QColor(50,60,75))
        qp.drawText(8, h - 9, 'Tracejado: repouso | Roxo: gravidade / CM' if self.has_gimbal else 'Seta vermelha: câmara | Ponto preto: alvo')


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, link, title="Cubo - estação de terra", img_every=10, auto_track=False,webcam_only=False):
        super().__init__()
        self.link = link
        self.img_every = img_every
        self.setWindowTitle(title)
        self.resize(1440, 920)
        self.setMinimumSize(1120, 740)
        self.setStyleSheet('''
            QMainWindow, QWidget#dashboard { background: #f2f5fa; color: #1c2d43; }
            QWidget { font-family: "Segoe UI"; font-size: 12px; color: #24364d; }
            QGroupBox { background: white; border: 1px solid #dce4ef; border-radius: 10px;
                        margin-top: 14px; padding: 16px 10px 10px; font-weight: 600; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; color: #50647e; }
            QPushButton { background: #edf2f9; border: 1px solid #d9e3ef; border-radius: 7px;
                          padding: 9px 10px; font-weight: 600; }
            QPushButton:hover { background: #dfe9f7; border-color: #9fbbe0; }
            QPushButton:pressed, QPushButton:checked { background: #cedef3; }
            QPushButton#primary { background: #2365df; color: white; border: none; padding: 12px; }
            QPushButton#primary:hover { background: #1955c1; }
            QDoubleSpinBox { background: #f8fafd; border: 1px solid #ced9e8; border-radius: 5px; padding: 6px; }
            QTabWidget::pane { border: none; }
            QTabBar::tab { min-width: 170px; padding: 10px 16px; border-bottom: 3px solid transparent; color: #748398; font-weight: 600; }
            QTabBar::tab:selected { color: #2365df; border-bottom-color: #2365df; font-weight: 600; }
            QFrame#metric { background: white; border: 1px solid #dce4ef; border-radius: 9px; }
            QTableWidget { background: white; border: 1px solid #dce4ef; gridline-color: #edf1f7; }
            QHeaderView::section { background: #edf2f9; border: none; padding: 6px; }
            QScrollArea { border: none; background: transparent; }
            QCheckBox { spacing: 8px; }
        ''')
        self.t0 = None
        self.buf = {k: deque(maxlen=HIST) for k in
                    ("t", "heading", "setpoint", "err", "target_err", "rate", "rate_ref", "wheel", "wheel_cmd")}
        self.last_state = None
        self.heading_sp = 0.0
        self.vis_times = deque(maxlen=40)
        self.last_vis = None
        self.pending_vis = None
        self.csv_file = self.csv_writer = None
        self.last_rx = None
        self.next_ping = time.monotonic() + 1.0
        self.track_pending = auto_track
        self.track_next_send = 0.0
        self.last_sim3 = None
        self.last_sim3_rx = None
        self.last_gimbal = None
        self.last_gimbal_rx = None
        self.gimbal_synced = False
        self.last_camera=None
        self.camera_pending=False
        self.camera_settings_error=''
        self.webcam_only=webcam_only
        self.center_pending=False
        self.center_next_send=0.
        self.sim_buf = {k: deque(maxlen=HIST) for k in ('t', 'err_y', 'err_roll', 'wheel_x', 'wheel_y', 'cmd_x', 'cmd_y')}
        self.curves, self.sim_curves = {}, {}

        root = QtWidgets.QWidget()
        root.setObjectName('dashboard')
        self.setCentralWidget(root)
        outer = QtWidgets.QVBoxLayout(root)
        outer.setContentsMargins(20, 14, 20, 14); outer.setSpacing(12)
        header = QtWidgets.QHBoxLayout()
        heading = QtWidgets.QLabel('CUBESAT  /  MISSION CONTROL')
        heading.setStyleSheet('font-size: 21px; font-weight: 700; color: #183251;')
        header.addWidget(heading); header.addStretch()
        self.chk_log = QtWidgets.QCheckBox('Registar dados em CSV')
        self.chk_log.toggled.connect(self._toggle_log)
        header.addWidget(self.chk_log); outer.addLayout(header)
        self.lbl = {k: QtWidgets.QLabel('—') for k in
                    ('link','mode','trk','flags','vbat','vision','cam_age','rates','control','motor')}
        metrics = QtWidgets.QHBoxLayout()
        self.metric_error = QtWidgets.QLabel('—')
        self.metric_wheel = QtWidgets.QLabel('—')
        self.saturation = QtWidgets.QLabel('Sem telemetria das rodas')
        for name, value in (('LIGAÇÃO',self.lbl['link']), ('SEGUIMENTO',self.lbl['control']),
                            ('ERRO HORIZONTAL / VERTICAL',self.metric_error), ('RODAS DE REAÇÃO', self.saturation)):
            card = QtWidgets.QFrame(); card.setObjectName('metric')
            col = QtWidgets.QVBoxLayout(card); col.setContentsMargins(14,10,14,10)
            caption = QtWidgets.QLabel(name); caption.setStyleSheet('font-size: 10px; color: #7c8ca1; font-weight: 600;')
            value.setWordWrap(True); value.setStyleSheet('font-size: 14px; font-weight: 600;')
            col.addWidget(caption); col.addWidget(value); metrics.addWidget(card,1)
        outer.addLayout(metrics)
        self.tabs = QtWidgets.QTabWidget(); outer.addWidget(self.tabs, 1)
        operation = QtWidgets.QWidget(); self.tabs.addTab(operation, 'Operação')
        grid = QtWidgets.QHBoxLayout(operation); grid.setContentsMargins(0,10,0,0); grid.setSpacing(14)
        scroll = QtWidgets.QScrollArea(); scroll.setWidgetResizable(True)
        scroll.setFixedWidth(282); scroll.setWidget(self._build_controls()); grid.addWidget(scroll)
        right = QtWidgets.QVBoxLayout(); right.setSpacing(12)
        right.addWidget(self._build_side(), 3)
        right.addWidget(self._build_plots(), 2)
        grid.addLayout(right,1)
        advanced = QtWidgets.QScrollArea(); advanced.setWidgetResizable(True)
        advanced.setWidget(self._build_advanced()); self.tabs.addTab(advanced, 'Definições avançadas')

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(30)
        self.send(P.tc_ping)
        self.send(P.tc_stream, 20, 20, self.img_every)

    def send(self, make, *args):
        self.link.send_frame(make, *args)

    def fix_target(self):
        self.center_pending=False
        self.track_pending = True
        self.track_next_send = 0.0
        self.sp_aim.setValue(0.0)

    def stop_control(self):
        self.center_pending=False
        self.track_pending = False
        self.send(P.tc_mode, 1)

    def recalibrate(self):
        self.center_pending=False
        self.track_pending = False
        self.send(P.tc_recal)

    def _spin(self, lo, hi, val, dec=1, step=1.0, suffix=""):
        s = QtWidgets.QDoubleSpinBox()
        s.setRange(lo, hi); s.setDecimals(dec); s.setValue(val); s.setSingleStep(step); s.setSuffix(suffix)
        return s

    def _build_controls(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(0,0,7,0); lay.setSpacing(12)

        g = QtWidgets.QGroupBox("Modo"); gl = QtWidgets.QGridLayout(g)
        fix = QtWidgets.QPushButton("FIXAR ALVO — controlo automático")
        fix.setText('FIXAR ALVO'); fix.setObjectName('primary')
        fix.clicked.connect(self.fix_target)
        gl.addWidget(fix, 0, 0, 1, 3)
        self.center_button=QtWidgets.QPushButton('CENTRAR NA WEBCAM')
        self.center_button.setEnabled(self.webcam_only)
        self.center_button.setToolTip('Volta à direção central da webcam e mantém-na. FIXAR ALVO retoma o seguimento do papel.')
        self.center_button.clicked.connect(self.center_camera)
        gl.addWidget(self.center_button,1,0,1,3)
        idle = QtWidgets.QPushButton("IDLE — desligar controlo")
        idle.clicked.connect(self.stop_control)
        gl.addWidget(idle, 2, 0, 1, 3)
        rb = QtWidgets.QPushButton("RECALIBRAR (cubo parado!)")
        rb.clicked.connect(self.recalibrate)
        gl.addWidget(rb, 3, 0, 1, 3)
        lay.addWidget(g)

        g = QtWidgets.QGroupBox('Empurrões'); gl = QtWidgets.QGridLayout(g)
        for row, (axis, magnitude) in enumerate((('X',20),('Y',10))):
            for col, sign in enumerate((-1,1)):
                value = sign*magnitude
                b = QtWidgets.QPushButton(f'{axis}  {value:+}°')
                b.clicked.connect(lambda _, a=axis, v=value: self.send(P.sim_shove_xy, v if a=='X' else 0, v if a=='Y' else 0))
                gl.addWidget(b,row,col)
        note = QtWidgets.QLabel('X: horizontal  ·  Y: vertical')
        note.setStyleSheet('color: #748398; font-size: 11px;'); gl.addWidget(note,2,0,1,2)
        self.shove_x = self._spin(-90,90,20,1,5,' °')
        self.shove_y = self._spin(-30,30,10,1,5,' °')
        gl.addWidget(QtWidgets.QLabel('Ângulo X'),3,0); gl.addWidget(self.shove_x,3,1)
        gl.addWidget(QtWidgets.QLabel('Ângulo Y'),4,0); gl.addWidget(self.shove_y,4,1)
        custom = QtWidgets.QPushButton('Aplicar X + Y')
        custom.clicked.connect(lambda: self.send(P.sim_shove_xy,self.shove_x.value(),self.shove_y.value()))
        gl.addWidget(custom,5,0,1,2); lay.addWidget(g)
        g = QtWidgets.QGroupBox('Câmara'); vl = QtWidgets.QVBoxLayout(g)
        self.webcam_button=QtWidgets.QPushButton('Ligar webcam');self.webcam_button.setCheckable(True)
        self.webcam_button.toggled.connect(self._request_camera)
        vl.addWidget(self.webcam_button)
        self.cover_button = QtWidgets.QPushButton('Tapar câmara'); self.cover_button.setCheckable(True)
        self.cover_button.toggled.connect(self._cover_camera)
        vl.addWidget(self.cover_button); lay.addWidget(g)
        g = QtWidgets.QGroupBox('Atuadores'); vl = QtWidgets.QVBoxLayout(g)
        self.metric_wheel.setWordWrap(True); vl.addWidget(self.metric_wheel)
        vl.addWidget(self.lbl['motor']); vl.addWidget(self.lbl['vbat']); lay.addWidget(g)
        lay.addStretch()
        return w

    def _cover_camera(self, covered):
        self.send(P.sim_cover, covered)
        self.cover_button.setText('Destapar câmara' if covered else 'Tapar câmara')

    def center_camera(self):
        self.track_pending=False
        self.center_pending=True
        self.center_next_send=0.
        self.sp_aim.setValue(0.)

    def _build_advanced(self):
        w = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(w)
        intro = QtWidgets.QLabel('Afinação do controlador e diagnóstico. As alterações dos ganhos aplicam-se às três rodas.')
        intro.setWordWrap(True); layout.addWidget(intro)
        top = QtWidgets.QHBoxLayout()
        g = QtWidgets.QGroupBox('Pontaria e iluminação'); form = QtWidgets.QFormLayout(g)
        self.sp_aim = self._spin(-25,25,0,1,1,' °')
        row = QtWidgets.QHBoxLayout(); row.addWidget(self.sp_aim)
        b=QtWidgets.QPushButton('Aplicar pontaria'); b.clicked.connect(lambda:self.send(P.tc_aim,self.sp_aim.value()))
        row.addWidget(b); form.addRow('Desvio horizontal',row)
        self.light = self._spin(.2,1.5,1,2,.1)
        b=QtWidgets.QPushButton('Aplicar luz'); b.clicked.connect(lambda:self.send(P.sim_light,self.light.value()))
        row=QtWidgets.QHBoxLayout();row.addWidget(self.light);row.addWidget(b);form.addRow('Luz (cena sintética)',row)
        hint=QtWidgets.QLabel('0° centra o alvo. FIXAR ALVO repõe a pontaria em zero.\nO alvo permanece fixo no cenário.')
        hint.setWordWrap(True);form.addRow(hint);top.addWidget(g,1)
        g=QtWidgets.QGroupBox('Diagnóstico'); form=QtWidgets.QFormLayout(g)
        for key,name in (('mode','Modo'),('trk','Seguimento'),('flags','Indicadores'),('vision','Deteção'),('cam_age','Idade da câmara'),('rates','Pacotes recebidos / enviados')):
            self.lbl[key].setWordWrap(True);form.addRow(name,self.lbl[key])
        top.addWidget(g,1);layout.addLayout(top)
        g=QtWidgets.QGroupBox('Webcam do computador'); form=QtWidgets.QFormLayout(g)
        self.webcam_index=QtWidgets.QSpinBox();self.webcam_index.setRange(0,9)
        self.webcam_hfov=self._spin(40,120,60,1,5,' °')
        self.webcam_virtual_fov=self._spin(20,70,40,1,5,' °')
        form.addRow('Número da webcam',self.webcam_index)
        form.addRow('Campo horizontal real (aproximado)',self.webcam_hfov)
        form.addRow('Campo da vista virtual',self.webcam_virtual_fov)
        b=QtWidgets.QPushButton('Ligar / reaplicar webcam')
        b.clicked.connect(lambda:self._request_camera(True));form.addRow(b)
        self.camera_hint=QtWidgets.QLabel('Mostra um círculo preto num papel branco, sem outros círculos, ocupando boa parte da imagem. '
                                         'O vídeo fica no computador. Os ângulos dependem do campo óptico configurado.')
        self.camera_hint.setWordWrap(True);form.addRow(self.camera_hint);layout.addWidget(g)
        g=QtWidgets.QGroupBox('Gimbal · gravidade e referência'); grid=QtWidgets.QGridLayout(g)
        form=QtWidgets.QFormLayout()
        self.mass=self._spin(.1,10,1,3,.1,' kg')
        self.mass.setToolTip('1 kg é um valor de exemplo. Substitui pela massa medida do conjunto móvel.')
        form.addRow('Massa do conjunto',self.mass)
        self.com_fields=[self._spin(-50,50,0,2,1,' mm') for _ in range(3)]
        for axis,field in zip('XYZ',self.com_fields):form.addRow('Centro de massa '+axis,field)
        apply=QtWidgets.QPushButton('Aplicar massa e centro de massa (IDLE)')
        apply.clicked.connect(self._apply_gimbal);form.addRow(apply)
        grid.addLayout(form,0,0)
        side=QtWidgets.QVBoxLayout()
        self.gimbal_info=QtWidgets.QLabel('A aguardar parâmetros do gimbal');self.gimbal_info.setWordWrap(True)
        side.addWidget(self.gimbal_info)
        remember=QtWidgets.QPushButton('Memorizar posição de repouso')
        remember.clicked.connect(self._capture_rest);side.addWidget(remember)
        self.gimbal_status=QtWidgets.QLabel('Alterar parâmetros em IDLE. Para memorizar repouso, fica parado pelo menos 0,5 s.')
        self.gimbal_status.setWordWrap(True);side.addWidget(self.gimbal_status)
        note=QtWidgets.QLabel('CM relativo ao pivô, no centro geométrico. +X: frente; +Y: direita; +Z: baixo. '
                             'CM = (0, 0, 0): peso sustentado, sem binário gravítico. '
                             'A referência de repouso é independente da calibração dos sensores.')
        note.setWordWrap(True);side.addWidget(note)
        grid.addLayout(side,0,1);grid.setColumnStretch(0,1);grid.setColumnStretch(1,1)
        layout.addWidget(g)
        g = QtWidgets.QGroupBox('Ganhos e limites'); vl = QtWidgets.QVBoxLayout(g)
        self.gains = QtWidgets.QTableWidget(len(P.GAINS), 2)
        self.gains.setHorizontalHeaderLabels(["valor", "unid."])
        self.gains.setVerticalHeaderLabels([v[0] for v in P.GAINS.values()])
        for r, (gid, (name, unit, val)) in enumerate(P.GAINS.items()):
            self.gains.setItem(r, 0, QtWidgets.QTableWidgetItem(f"{val:g}"))
            it = QtWidgets.QTableWidgetItem(unit); it.setFlags(QtCore.Qt.ItemIsEnabled); self.gains.setItem(r, 1, it)
        self.gains.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.gains.setMinimumHeight(355)
        descriptions = ('Rapidez de correção da orientação','Reação imediata ao erro de velocidade',
                        'Correção do erro de velocidade acumulado','Limite de velocidade do corpo',
                        'Limite de aceleração da referência','Limite de velocidade de cada roda',
                        'Limite de binário de cada motor','Incerteza atribuída à câmara',
                        'Incerteza atribuída à referência de atitude','Usar a referência de atitude',
                        'Tempo sem câmara antes de procurar o alvo')
        for r,description in enumerate(descriptions):
            self.gains.item(r,0).setToolTip(description)
            self.gains.verticalHeaderItem(r).setToolTip(description)
        vl.addWidget(self.gains)
        gb = QtWidgets.QPushButton("Enviar linha seleccionada"); gb.clicked.connect(self._send_gain)
        vl.addWidget(gb); layout.addWidget(g)
        layout.addWidget(self._build_plots(advanced=True))
        explanation = QtWidgets.QLabel('Três rodas ortogonais: X estabiliza roll, Y controla a inclinação, Z controla a guinada. '
                                      'Nos empurrões, X/Y designam as direções horizontal/vertical da imagem. '
                                      'A dinâmica usa uma aproximação desacoplada por eixo.')
        explanation.setWordWrap(True);layout.addWidget(explanation)
        return w

    def _apply_gimbal(self):
        self.send(P.sim_gimbal_config,self.mass.value(),*(field.value()/1000 for field in self.com_fields))
        self.gimbal_status.setText('A aguardar confirmação dos parâmetros…')

    def _request_camera(self,enabled):
        self.camera_settings_error=''
        if enabled and self.webcam_virtual_fov.value()>=self.webcam_hfov.value()-5:
            self.camera_settings_error='O campo virtual deve ser mais de 5° menor que o campo real. Ajusta os valores e reaplica.'
            self.camera_hint.setText(self.camera_settings_error)
            return
        self.camera_pending=True
        self.send(P.sim_camera,enabled,self.webcam_index.value(),self.webcam_hfov.value(),self.webcam_virtual_fov.value())
        self.camera_hint.setText('A abrir a webcam…' if enabled else 'A desligar a webcam…')

    def _capture_rest(self):
        self.send(P.sim_rest)
        self.gimbal_status.setText('A aguardar confirmação da referência…')

    def _update_reference_labels(self,relative,planes):
        for label,name,value in zip(self.relative_labels,('Roll','Pitch','Yaw'),relative):
            label.setText(f'{name} {value:+.2f}°')
        for label,name,value in zip(self.plane_labels,('XY','XZ','YZ'),planes):
            label.setText(f'{name} {value:.2f}°')

    def _send_gain(self):
        r = self.gains.currentRow()
        if r < 0:
            return
        try:
            v = float(self.gains.item(r, 0).text())
        except ValueError:
            return
        self.send(P.tc_gain, list(P.GAINS)[r], v)

    def _build_plots(self, advanced=False):
        glw = pg.GraphicsLayoutWidget(); glw.setBackground('w')
        glw.setMinimumHeight(240)
        defs = (("Rumo horizontal (°)", "heading", "setpoint"), ("Velocidade horizontal (°/s)", "rate", "rate_ref")) if advanced else (
                ('Erro de pontaria (°)', 'err', None), ('Rodas · medida / pedida (rpm)', 'wheel', 'wheel_cmd'))
        first = None
        for i, (title, a, b) in enumerate(defs):
            p = glw.addPlot(row=0, col=i)
            p.setTitle(title,color='#50647e',size='11pt')
            p.addLegend(offset=(8,8), labelTextSize='9pt',labelTextColor='#50647e')
            for axis in ('left','bottom'):
                p.getAxis(axis).setTextPen('#61748a')
                p.getAxis(axis).setPen('#bdc9d9')
            p.setLabel('bottom','Tempo',units='s')
            p.showGrid(x=True, y=True, alpha=0.3)
            if first is None: first = p
            else: p.setXLink(first)
            self.curves[a] = p.plot(pen=pg.mkPen('#2365df',width=2),name='Horizontal' if a=='err' else ('Z medida' if a=='wheel' else 'Medida'))
            if b:
                self.curves[b] = p.plot(pen=pg.mkPen('#2365df',width=1,style=QtCore.Qt.DashLine),name='Z pedida' if a=='wheel' else 'Referência')
            if a=='err':
                self.sim_curves['err_y']=p.plot(pen=pg.mkPen('#16a394',width=2),name='Vertical')
                self.sim_curves['err_roll']=p.plot(pen=pg.mkPen('#8c93a3',style=QtCore.Qt.DotLine),name='Roll')
            if a=='wheel':
                for axis,color in (('x','#8b6bd9'),('y','#16a394')):
                    self.sim_curves['wheel_'+axis]=p.plot(pen=pg.mkPen(color,width=2),name=axis.upper()+' medida')
                    self.sim_curves['cmd_'+axis]=p.plot(pen=pg.mkPen(color,style=QtCore.Qt.DashLine),name=axis.upper()+' pedida')
            if a == "err":
                for y in (-2, 2): p.addItem(pg.InfiniteLine(pos=y, angle=0, pen=pg.mkPen("g", style=QtCore.Qt.DotLine)))
        return glw

    def _build_side(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QHBoxLayout(w); lay.setContentsMargins(0,0,0,0);lay.setSpacing(12)

        g3d = QtWidgets.QGroupBox("Gimbal 3D · arrastar para rodar, roda para zoom")
        v3d = QtWidgets.QVBoxLayout(g3d)
        self.cube3d = Cube3DWidget()
        v3d.addWidget(self.cube3d,1)
        self.reference_status=QtWidgets.QLabel('A aguardar referência inicial')
        self.reference_status.setStyleSheet('font-size: 11px; color: #748398;')
        v3d.addWidget(self.reference_status)
        angles=QtWidgets.QGridLayout()
        self.relative_labels=[QtWidgets.QLabel(name+' —') for name in ('Roll','Pitch','Yaw')]
        self.plane_labels=[QtWidgets.QLabel(name+' —') for name in ('XY','XZ','YZ')]
        for i,label in enumerate(self.relative_labels):
            label.setStyleSheet('font-weight: 600; color: #2365df;')
            label.setToolTip('Rotação relativa à orientação de repouso, em graus com sinal.')
            angles.addWidget(label,0,i)
        for i,label in enumerate(self.plane_labels):
            label.setToolTip('Ângulo entre o plano atual e o mesmo plano em repouso: 0° a 90°.')
            angles.addWidget(label,1,i)
        v3d.addLayout(angles)
        self.cube3d.orientation_changed.connect(self._update_reference_labels)
        lay.addWidget(g3d, 1)

        g = QtWidgets.QGroupBox('Vídeo e deteção'); vl = QtWidgets.QVBoxLayout(g)
        self.camera_tabs=QtWidgets.QTabWidget()
        self.camera_tabs.setStyleSheet('QTabBar::tab { min-width: 90px; padding: 6px; font-size: 11px; }')
        virtual_page=QtWidgets.QWidget();virtual_layout=QtWidgets.QVBoxLayout(virtual_page);virtual_layout.setContentsMargins(0,0,0,0)
        self.cam = pg.GraphicsLayoutWidget(); self.cam.setBackground('#101d2e'); self.cam.setMinimumHeight(200)
        vb = self.cam.addViewBox(lockAspect=True, invertY=True)
        self.img_item = pg.ImageItem(np.full((60, 80), 128, np.uint8).T, levels=(0, 255))
        self.img_item.setRect(QtCore.QRectF(0,0,320,240))
        vb.addItem(self.img_item)
        vb.addItem(pg.PlotCurveItem([144,176],[120,120],pen=pg.mkPen('#30baa8',width=1)))
        vb.addItem(pg.PlotCurveItem([160,160],[104,136],pen=pg.mkPen('#30baa8',width=1)))
        self.dot = pg.ScatterPlotItem(size=14, pen=pg.mkPen("y", width=2), brush=None, symbol="o")
        vb.addItem(self.dot)
        vb.setRange(xRange=(0,320),yRange=(0,240),padding=0);vb.disableAutoRange()
        virtual_layout.addWidget(self.cam)
        self.camera_tabs.addTab(virtual_page,'Câmara virtual')
        self.raw_cam=pg.GraphicsLayoutWidget();self.raw_cam.setBackground('#101d2e')
        raw_view=self.raw_cam.addViewBox(lockAspect=True,invertY=True)
        self.raw_item=pg.ImageItem(np.full((320,240),128,np.uint8),levels=(0,255))
        self.raw_item.setRect(QtCore.QRectF(0,0,320,240));raw_view.addItem(self.raw_item)
        self.raw_dot=pg.ScatterPlotItem(size=16,pen=pg.mkPen('#30baa8',width=2),brush=None,symbol='o')
        raw_view.addItem(self.raw_dot)
        raw_view.setRange(xRange=(0,320),yRange=(0,240),padding=0);raw_view.disableAutoRange()
        self.camera_tabs.addTab(self.raw_cam,'Webcam completa')
        self.camera_status = QtWidgets.QLabel('A aguardar imagem da câmara')
        self.camera_status.setStyleSheet('color: #748398; font-size: 11px;')
        self.camera_status.setWordWrap(True)
        vl.addWidget(self.camera_tabs,1); vl.addWidget(self.camera_status)
        lay.addWidget(g,1)
        return w

    def _toggle_log(self, on):
        if on:
            self.csv_file = open(time.strftime("cubo_%Y%m%d_%H%M%S.csv"), "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['t_ms','mode','trk','vbat_v'] +
                [f'{axis}_{field}' for axis in ('roll','pitch','yaw') for field in
                 ('angle_deg','rate_dps','error_deg','wheel_rpm','command_rpm','torque_nm')] +
                ['gimbal_t_ms']+[f'physical_{axis}_deg' for axis in ('roll','pitch','yaw')]+
                [f'relative_{axis}_deg' for axis in ('roll','pitch','yaw')]+
                [f'plane_{plane}_deg' for plane in ('xy','xz','yz')]+['reference_id']+
                [f'reference_{axis}_deg' for axis in ('roll','pitch','yaw')]+
                ['mass_kg','com_x_m','com_y_m','com_z_m']+
                [f'gravity_torque_{axis}_nm' for axis in ('roll','pitch','yaw')])
        elif self.csv_file:
            self.csv_file.close(); self.csv_file = self.csv_writer = None

    def on_state(self, s):
        self.last_state = s
        tgt_yaw = (s.heading + s.target_err) if s.flag(0) else None
        if not self.cube3d.has_gimbal and (self.last_sim3_rx is None or time.monotonic()-self.last_sim3_rx > .5):
            self.cube3d.set_state(s.heading, s.wheel_rpm, tgt_yaw, t_ms=s.t_ms)
        if self.t0 is None:
            self.t0 = s.t_ms
        b = self.buf
        b["t"].append((s.t_ms - self.t0) / 1000.0)
        b["heading"].append(s.heading)
        b["setpoint"].append(self.heading_sp if s.mode == 4 else np.nan)
        b["err"].append(s.err); b["target_err"].append(s.target_err if s.mode == 5 else np.nan)
        b["rate"].append(s.rate); b["rate_ref"].append(s.rate_ref)
        b["wheel"].append(s.wheel_rpm); b["wheel_cmd"].append(s.wheel_cmd_rpm)
        if self.csv_writer and self.last_sim3 is None:
            self.csv_writer.writerow([s.t_ms,s.mode,s.trk,s.vbat]+['']*12+
                                     [s.heading,s.rate,s.err,s.wheel_rpm,s.wheel_cmd_rpm,s.torque]+self._gimbal_csv_fields())

    def on_sim3(self, obj):
        self.last_sim3 = obj
        self.last_sim3_rx = time.monotonic()
        self.cube3d.set_three_axis(obj)
        if self.t0 is None:
            self.t0 = obj.t_ms
        self.sim_buf['t'].append((obj.t_ms-self.t0)/1000)
        for key,value in (('err_y',obj.errors[1]),('err_roll',obj.errors[0]),
                          ('wheel_x',obj.wheels[0]),('wheel_y',obj.wheels[1]),
                          ('cmd_x',obj.commands[0]),('cmd_y',obj.commands[1])):
            self.sim_buf[key].append(value)
        if self.csv_writer and self.last_state:
            state=self.last_state
            self.csv_writer.writerow([obj.t_ms,state.mode,state.trk,state.vbat] +
                [group[i] for i in range(3) for group in
                 (obj.attitude,obj.rates,obj.errors,obj.wheels,obj.commands,obj.torques)]+self._gimbal_csv_fields())

    def _gimbal_csv_fields(self):
        obj=self.last_gimbal
        if obj is None:return ['']*21
        return ([obj.t_ms]+list(obj.attitude)+list(obj.relative)+list(obj.planes)+[obj.reference_id]+
                list(obj.reference)+[obj.mass,obj.com_x,obj.com_y,obj.com_z]+list(obj.torques))

    def on_gimbal(self,obj):
        self.last_gimbal=obj;self.last_gimbal_rx=time.monotonic()
        self.cube3d.set_gimbal_state(obj)
        self.reference_status.setText(f'Ângulos físicos relativos ao repouso · referência #{obj.reference_id}')
        if not self.gimbal_synced:
            self.mass.setValue(obj.mass)
            for field,value in zip(self.com_fields,(obj.com_x,obj.com_y,obj.com_z)):field.setValue(value*1000)
            self.gimbal_synced=True
        ref=' / '.join(f'{v:+.1f}°' for v in obj.reference)
        torque=' / '.join(f'{v:+.4f}' for v in obj.torques)
        self.gimbal_info.setText(f'Massa aplicada = {obj.mass:.3f} kg · g = {obj.g:.5f} m/s² · Peso = {obj.mass*obj.g:.2f} N\n'
                                 f'CM aplicado: ({obj.com_x*1000:.1f}, {obj.com_y*1000:.1f}, {obj.com_z*1000:.1f}) mm\n'
                                 f'Binário gravítico roll / pitch / yaw: {torque} N·m\n'
                                 f'Repouso no referencial fixo (R / P / Y): {ref}')

    def on_camera(self,obj):
        old_mode=self.last_camera.mode if self.last_camera else 0
        old_status=self.last_camera.status if self.last_camera else -1
        self.last_camera=obj
        self.center_button.setEnabled(self.webcam_only or obj.mode in (1,2))
        if not self.camera_pending:
            self.webcam_button.blockSignals(True);self.webcam_button.setChecked(obj.mode==1);self.webcam_button.blockSignals(False)
        self.webcam_button.setText('Desligar webcam' if obj.mode==1 else 'Ligar webcam')
        if obj.mode in (1,2) and old_mode==0:self.camera_tabs.setCurrentIndex(1)
        if obj.mode==0 and old_mode in (1,2):self.camera_tabs.setCurrentIndex(0)
        if obj.status in (4,5) and (old_status!=obj.status or old_mode!=obj.mode):
            for item in (self.raw_item,self.img_item):
                item.setImage(np.full((320,240,3),225,np.uint8),levels=(0,255));item.setRect(QtCore.QRectF(0,0,320,240))
        self.camera_hint.setText(self.camera_settings_error or (obj.message if obj.mode or obj.status in (4,5) else 'Câmara sintética. Liga a webcam para seguir o papel real.'))
        self.raw_dot.setData([obj.x],[obj.y]) if obj.found and np.isfinite(obj.x) and np.isfinite(obj.y) else self.raw_dot.setData([],[])

    def on_video(self,view,jpeg):
        image=QtGui.QImage.fromData(jpeg)
        if image.isNull() or image.width()>1280 or image.height()>960:return
        image=image.convertToFormat(QtGui.QImage.Format_RGB888)
        pointer=image.bits();pointer.setsize(image.byteCount())
        array=np.frombuffer(pointer,np.uint8).reshape(image.height(),image.bytesPerLine())[:,:image.width()*3].reshape(image.height(),image.width(),3).copy()
        item=self.raw_item if view==0 else self.img_item
        item.setImage(array.transpose(1,0,2),levels=(0,255));item.setRect(QtCore.QRectF(0,0,320,240))
        if view==1 and self.pending_vis is not None:self.last_vis=self.pending_vis

    def tick(self):
        now = time.monotonic()
        if now >= self.next_ping:
            self.send(P.tc_ping)
            if self.last_rx is None or now - self.last_rx > 2.0:
                self.send(P.tc_stream, 20, 20, self.img_every)
            self.next_ping = now + 1.0
        got_image = False
        for kind, obj in self.link.poll():
            self.last_rx = now
            if kind == "state":
                self.on_state(obj)
            elif kind == 'sim3':
                self.on_sim3(obj)
            elif kind == 'gimbal':
                self.on_gimbal(obj)
            elif kind=='camera':self.on_camera(obj)
            elif kind=='video':self.on_video(*obj);got_image=True
            elif kind=='ack' and len(obj)==3 and obj[0]==P.SIM_CAMERA:
                self.camera_pending=False
                if obj[2]:self.camera_hint.setText('Não foi possível mudar de câmara. Verifica OpenCV, o número da webcam e os campos ópticos.')
            elif kind=='ack' and len(obj)==3 and obj[0]==P.SIM_CAMERA_CENTER:
                if obj[2]:
                    self.center_pending=False
                    self.camera_hint.setText('Aguarda o fim da calibração e usa este comando no modo webcam.')
            elif kind == 'ack' and len(obj)==3 and obj[0] in (P.SIM_GIMBAL_CFG,P.SIM_REST):
                if obj[2]==0:
                    self.gimbal_status.setText('Parâmetros aplicados.' if obj[0]==P.SIM_GIMBAL_CFG else 'Nova referência de repouso memorizada.')
                else:
                    self.gimbal_status.setText('Pedido recusado. Seleciona IDLE; para memorizar repouso, espera que o cubo pare. Verifica os valores.')
            elif kind == "vision":
                self.pending_vis = obj
                self.vis_times.append(time.time())
                if self.img_every == 0:
                    self.last_vis = obj
            elif kind == "image":
                self.img_item.setImage(obj[1].T, levels=(0, 255))
                self.img_item.setRect(QtCore.QRectF(0,0,320,240))
                if self.pending_vis is not None:
                    self.last_vis = self.pending_vis
                got_image = True

        # Atualiza o cursor amarelo em sincronia com a imagem (evita que o cursor se mova antes do corpo/frame)
        v = self.last_vis if self.img_every > 0 else self.pending_vis
        if v is not None and (got_image or self.img_every <= 2):
            if v.found and 0<=v.x<320 and 0<=v.y<240:
                self.dot.setData([v.x], [v.y])
            else:
                self.dot.setData([], [])

        if len(self.buf["t"]) > 1:
            t = np.fromiter(self.buf["t"], float)
            for k, c in self.curves.items():
                c.setData(t, np.fromiter(self.buf[k], float))
        if len(self.sim_buf['t']) > 1:
            t=np.asarray(self.sim_buf['t'])
            for key,curve in self.sim_curves.items():
                curve.setData(t,np.asarray(self.sim_buf[key]))

        s = self.last_state
        fps = (len(self.vis_times) - 1) / max(1e-3, self.vis_times[-1] - self.vis_times[0]) if len(self.vis_times) > 2 else 0.0
        self.lbl["link"].setText("sem dados" if self.last_rx is None or now - self.last_rx > 2.0 else "OK")
        self.lbl["rates"].setText(f"{self.link.n_rx} / {self.link.n_tx}")
        if self.center_pending and s and s.mode!=0:
            if self.last_camera and self.last_camera.centering and s.mode==4:self.center_pending=False
            elif now>=self.center_next_send:
                self.send(P.sim_camera_center)
                self.center_next_send=now+.5
        if self.track_pending and s and s.mode != 0:
            if s.mode == 5 and abs(self.sp_aim.value()) < 1e-6 and abs(s.err - s.target_err) < 0.05:
                self.track_pending = False
            elif now >= self.track_next_send:
                self.send(P.tc_aim, 0.0)
                self.send(P.tc_mode, 5)
                self.track_next_send = now + 0.5
        if s:
            self.lbl["mode"].setText("FIXAR ALVO" if s.mode == 5 else P.MODES.get(s.mode, "?"))
            self.lbl["trk"].setText(P.TRK.get(s.trk, "?"))
            self.lbl["flags"].setText(" ".join(n for b, n in P.FLAGS.items() if s.flag(b)) or "—")
            self.lbl["vbat"].setText(f"{s.vbat:.1f} V"); self.lbl["cam_age"].setText("—" if s.cam_age < 0 else f"{s.cam_age * 1000:.0f} ms")
            if s.mode != 5:
                control = "A aguardar calibração" if self.track_pending else "Desligado — carregar em FIXAR ALVO"
            elif not s.flag(0) or s.trk == 3:
                control = "À procura do alvo"
            elif abs(s.err) <= 2 and abs(s.rate) <= 1:
                control = f"Estabilizado | erro {s.err:+.2f}°"
            else:
                control = f"A corrigir | erro {s.err:+.2f}°"
            self.lbl["control"].setText(control)
            self.lbl["motor"].setText(f"{s.wheel_cmd_rpm:+.0f} rpm / {s.torque:+.3f} N·m")
            self.lbl['vbat'].setText(f'Bateria  {s.vbat:.1f} V')
            self.metric_error.setText(f'{s.err:+.2f}° / —')
        multi = self.last_sim3
        if multi and now-self.last_sim3_rx < .5:
            self.metric_error.setText(f'{multi.errors[2]:+.2f}° / {multi.errors[1]:+.2f}°')
            self.metric_wheel.setText('\n'.join(f'Roda {axis}  {multi.wheels[i]:+.0f} / {multi.commands[i]:+.0f} rpm'
                                               for i,axis in enumerate('XYZ')))
            saturated = ', '.join(axis for i,axis in enumerate('XYZ') if multi.saturated & (1<<i))
            unloading=', '.join(axis for i,axis in enumerate('XYZ') if multi.unloading & (1<<i))
            self.saturation.setText('SATURAÇÃO · '+saturated if saturated else ('Alívio de momento · '+unloading if unloading else '3 rodas · dentro dos limites'))
            color='#c2413a' if saturated else ('#aa7209' if unloading else '#138a75')
            self.saturation.setStyleSheet('font-size: 14px; font-weight: 600; color: '+color+';')
            self.lbl['motor'].setText('Binário X / Y / Z\n'+' / '.join(f'{v:+.3f}' for v in multi.torques)+' N·m')
            if s and s.mode==5 and s.flag(0) and s.trk!=3:
                stable=max(map(abs,multi.errors))<=2 and max(map(abs,multi.rates))<=1
                self.lbl['control'].setText('Estabilizado no alvo' if stable else 'A corrigir orientação')
        else:
            self.saturation.setText('Sem telemetria de 3 rodas')
            if self.last_sim3_rx is not None:
                self.metric_wheel.setText('A aguardar telemetria')
        if self.last_rx is None or now-self.last_rx>2:
            self.lbl['control'].setText('A aguardar ligação')
            self.metric_error.setText('— / —')
        if self.last_gimbal_rx is not None and now-self.last_gimbal_rx>1:
            self.reference_status.setText('Sem telemetria do gimbal · última posição retida')
        if self.pending_vis:
            pv = self.pending_vis
            self.lbl["vision"].setText(f"{'ACHOU' if pv.found else 'não vê'}  {fps:4.1f} fps  {pv.proc_us / 1000:.1f} ms  err={pv.err_x:+.1f}°")
            self.camera_status.setText(f'{"Alvo detetado" if pv.found else "Alvo não visível"}  ·  {fps:.0f} imagens/s processadas')
        camera=self.last_camera
        if camera and (camera.mode or camera.status==4):
            if camera.status in (1,4,5):
                self.camera_status.setText(camera.message)
                if not camera.centering:self.lbl['control'].setText('A aguardar vídeo recente')
            else:
                state='Ponto detetado na webcam' if camera.found else 'Ponto não detetado'
                self.camera_status.setText(f'{state} · vista virtual: {camera.coverage*100:.0f}% coberta pelo vídeo real')
            if camera.waiting:self.lbl['control'].setText('À espera do ponto da webcam')
            if camera.centering and s and s.mode==4:
                stable=multi is not None and max(map(abs,multi.errors))<1 and max(map(abs,multi.rates))<1
                self.lbl['control'].setText('Centrado na webcam' if stable else 'A centrar na webcam')
                self.lbl['mode'].setText('CENTRO DA WEBCAM')

    def closeEvent(self, e):
        self.timer.stop()
        self.cube3d.animation.stop()
        self._toggle_log(False)
        self.link.close()
        super().closeEvent(e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--serial", default=None, help="porta série (ex.: COM5) em vez de UDP")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--img-every", type=int, default=2, help="enviar 1 imagem a cada N frames")
    ap.add_argument("--auto-track", action="store_true", help="fixar alvo automaticamente após calibração")
    ap.add_argument('--webcam',action='store_true')
    ap.add_argument('--camera-index',type=int,default=0)
    a = ap.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    link = SerialLink(a.serial, a.baud) if a.serial else UdpLink(a.host, a.port)
    w = MainWindow(link, f"Cubo v9 - Proteção das rodas no gimbal  ({a.serial or f'{a.host}:{a.port}'})", img_every=a.img_every, auto_track=a.auto_track,webcam_only=a.webcam)
    w.webcam_index.setValue(max(0,min(9,a.camera_index)))
    if a.webcam:QtCore.QTimer.singleShot(200,lambda:w.webcam_button.setChecked(True))
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
