#!/usr/bin/env python3
"""
gs_app.py - Estação de terra do cubo (PyQt5 + pyqtgraph).

  python gs_app.py                      # liga a 127.0.0.1:5000 (cubo virtual: sim/sim_server.py)
  python gs_app.py --host 192.168.1.50  # cubo real por Wi-Fi (UDP)
  python gs_app.py --serial COM5        # cubo real por USB-série (requer pyserial)

Funcionalidades: telemetria em tempo real (rumo, erro, velocidades, roda), vista da câmara com o ponto
detectado, estados (modo, seguimento, flags), comandos (modo, setpoints, ganhos, recalibração),
registo CSV e, quando ligado ao simulador, perturbações (empurrão, câmara tapada, alvo, luz).
"""
import argparse
import csv
import sys
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets

import gs_protocol as P
from gs_link import SerialLink, UdpLink

HIST = 600  # amostras por gráfico (30 s a 20 Hz)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, link, title="Cubo - estação de terra"):
        super().__init__()
        self.link = link
        self.setWindowTitle(title)
        self.resize(1320, 820)
        self.t0 = None
        self.buf = {k: deque(maxlen=HIST) for k in
                    ("t", "heading", "setpoint", "err", "target_err", "rate", "rate_ref", "wheel", "wheel_cmd")}
        self.last_state = None
        self.heading_sp = 0.0
        self.vis_times = deque(maxlen=40)
        self.last_vis = None
        self.csv_file = self.csv_writer = None

        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        grid = QtWidgets.QHBoxLayout(root)
        grid.addWidget(self._build_controls(), 0)
        grid.addWidget(self._build_plots(), 1)
        grid.addWidget(self._build_side(), 0)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(30)
        self.send(P.tc_ping)
        self.send(P.tc_stream, 20, 20, 10)

    # ------------------------------------------------------------------ UI
    def send(self, make, *args):
        self.link.send_frame(make, *args)

    def _spin(self, lo, hi, val, dec=1, step=1.0, suffix=""):
        s = QtWidgets.QDoubleSpinBox()
        s.setRange(lo, hi); s.setDecimals(dec); s.setValue(val); s.setSingleStep(step); s.setSuffix(suffix)
        return s

    def _build_controls(self):
        w = QtWidgets.QWidget(); w.setFixedWidth(300)
        lay = QtWidgets.QVBoxLayout(w)

        g = QtWidgets.QGroupBox("Modo"); gl = QtWidgets.QGridLayout(g)
        for i, (name, m) in enumerate((("IDLE", 1), ("WHEEL", 2), ("RATE", 3), ("HEADING", 4), ("TRACK", 5))):
            b = QtWidgets.QPushButton(name)
            b.clicked.connect(lambda _, m=m: self.send(P.tc_mode, m))
            gl.addWidget(b, i // 3, i % 3)
        rb = QtWidgets.QPushButton("Recalibrar (cubo parado!)"); rb.clicked.connect(lambda: self.send(P.tc_recal))
        gl.addWidget(rb, 2, 0, 1, 3)
        lay.addWidget(g)

        g = QtWidgets.QGroupBox("Setpoints"); fl = QtWidgets.QFormLayout(g)
        self.sp_wheel = self._spin(-3000, 3000, 0, 0, 100, " rpm")
        self.sp_rate = self._spin(-40, 40, 0, 1, 5, " °/s")
        self.sp_head = self._spin(-720, 720, 0, 1, 15, " °")
        self.sp_aim = self._spin(-25, 25, 0, 1, 1, " °")
        for label, sp, make in (("Roda", self.sp_wheel, P.tc_wheel), ("Taxa", self.sp_rate, P.tc_rate),
                                ("Rumo", self.sp_head, P.tc_heading), ("Pontaria", self.sp_aim, P.tc_aim)):
            row = QtWidgets.QHBoxLayout(); row.addWidget(sp)
            b = QtWidgets.QPushButton("Enviar"); b.clicked.connect(lambda _, sp=sp, make=make: self._send_sp(sp, make))
            row.addWidget(b); fl.addRow(label, row)
        lay.addWidget(g)

        g = QtWidgets.QGroupBox("Ganhos (unidades do ICD)"); vl = QtWidgets.QVBoxLayout(g)
        self.gains = QtWidgets.QTableWidget(len(P.GAINS), 2)
        self.gains.setHorizontalHeaderLabels(["valor", "unid."])
        self.gains.setVerticalHeaderLabels([v[0] for v in P.GAINS.values()])
        for r, (gid, (name, unit, val)) in enumerate(P.GAINS.items()):
            self.gains.setItem(r, 0, QtWidgets.QTableWidgetItem(f"{val:g}"))
            it = QtWidgets.QTableWidgetItem(unit); it.setFlags(QtCore.Qt.ItemIsEnabled); self.gains.setItem(r, 1, it)
        self.gains.setFixedHeight(260)
        vl.addWidget(self.gains)
        gb = QtWidgets.QPushButton("Enviar linha seleccionada"); gb.clicked.connect(self._send_gain)
        vl.addWidget(gb); lay.addWidget(g)

        g = QtWidgets.QGroupBox("Simulador (perturbações)"); gl = QtWidgets.QGridLayout(g)
        for i, (txt, fn) in enumerate((("Empurrar +20°", lambda: self.send(P.sim_shove, 20.0)),
                                       ("Empurrar −20°", lambda: self.send(P.sim_shove, -20.0)),
                                       ("Tapar câmara", lambda: self.send(P.sim_cover, True)),
                                       ("Destapar", lambda: self.send(P.sim_cover, False)),
                                       ("Alvo +15°", lambda: self.send(P.sim_target, 15.0)),
                                       ("Alvo 0°", lambda: self.send(P.sim_target, 0.0)),
                                       ("Luz 0.5", lambda: self.send(P.sim_light, 0.5)),
                                       ("Luz 1.0", lambda: self.send(P.sim_light, 1.0)))):
            b = QtWidgets.QPushButton(txt); b.clicked.connect(fn); gl.addWidget(b, i // 2, i % 2)
        lay.addWidget(g)

        self.chk_log = QtWidgets.QCheckBox("Registar em CSV"); self.chk_log.toggled.connect(self._toggle_log)
        lay.addWidget(self.chk_log); lay.addStretch(1)
        return w

    def _send_sp(self, spin, make):
        v = spin.value()
        if make is P.tc_heading:
            self.heading_sp = v
        self.send(make, v)

    def _send_gain(self):
        r = self.gains.currentRow()
        if r < 0:
            return
        try:
            v = float(self.gains.item(r, 0).text())
        except ValueError:
            return
        self.send(P.tc_gain, list(P.GAINS)[r], v)

    def _build_plots(self):
        self.glw = pg.GraphicsLayoutWidget()
        self.glw.setBackground("w")
        defs = (("Rumo (°)", "heading", "setpoint"), ("Erro de pontaria (°)", "err", "target_err"),
                ("Velocidade angular (°/s)", "rate", "rate_ref"), ("Roda (rpm)", "wheel", "wheel_cmd"))
        self.curves, first = {}, None
        for i, (title, a, b) in enumerate(defs):
            p = self.glw.addPlot(row=i, col=0, title=title)
            p.showGrid(x=True, y=True, alpha=0.3)
            if first is None: first = p
            else: p.setXLink(first)
            self.curves[a] = p.plot(pen=pg.mkPen("#1f77b4", width=2))
            self.curves[b] = p.plot(pen=pg.mkPen("#ff7f0e", width=1, style=QtCore.Qt.DashLine))
            if a == "err":
                for y in (-2, 2): p.addItem(pg.InfiniteLine(pos=y, angle=0, pen=pg.mkPen("g", style=QtCore.Qt.DotLine)))
        return self.glw

    def _build_side(self):
        w = QtWidgets.QWidget(); w.setFixedWidth(320)
        lay = QtWidgets.QVBoxLayout(w)
        g = QtWidgets.QGroupBox("Câmara (80×60, a cada ~0.5 s)"); vl = QtWidgets.QVBoxLayout(g)
        self.cam = pg.GraphicsLayoutWidget(); self.cam.setFixedHeight(230)
        vb = self.cam.addViewBox(lockAspect=True, invertY=True)
        self.img_item = pg.ImageItem(np.full((60, 80), 128, np.uint8).T, levels=(0, 255))
        vb.addItem(self.img_item)
        vb.addItem(pg.InfiniteLine(pos=40, angle=90, pen=pg.mkPen("c", width=1)))
        vb.addItem(pg.InfiniteLine(pos=30, angle=0, pen=pg.mkPen("c", width=1)))
        self.dot = pg.ScatterPlotItem(size=14, pen=pg.mkPen("y", width=2), brush=None, symbol="o")
        vb.addItem(self.dot)
        vl.addWidget(self.cam); lay.addWidget(g)

        g = QtWidgets.QGroupBox("Estado"); fl = QtWidgets.QFormLayout(g)
        self.lbl = {k: QtWidgets.QLabel("—") for k in ("link", "mode", "trk", "flags", "vbat", "vision", "cam_age", "rates")}
        for k, name in (("link", "Ligação"), ("mode", "Modo"), ("trk", "Seguimento"), ("flags", "Flags"), ("vbat", "Bateria"),
                        ("vision", "Visão"), ("cam_age", "Idade da medida"), ("rates", "Pacotes rx/tx")):
            self.lbl[k].setStyleSheet("font-family: monospace"); self.lbl[k].setWordWrap(True); fl.addRow(name, self.lbl[k])
        lay.addWidget(g); lay.addStretch(1)
        return w

    # --------------------------------------------------------------- dados
    def _toggle_log(self, on):
        if on:
            self.csv_file = open(time.strftime("cubo_%Y%m%d_%H%M%S.csv"), "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["t_ms", "mode", "trk", "flags", "heading_deg", "rate_dps", "bias_dps", "err_deg",
                                      "rate_ref_dps", "wheel_rpm", "wheel_cmd_rpm", "torque_nm", "vbat_v", "target_err_deg", "cam_age_s"])
        elif self.csv_file:
            self.csv_file.close(); self.csv_file = self.csv_writer = None

    def on_state(self, s):
        self.last_state = s
        if self.t0 is None:
            self.t0 = s.t_ms
        b = self.buf
        b["t"].append((s.t_ms - self.t0) / 1000.0)
        b["heading"].append(s.heading)
        b["setpoint"].append(self.heading_sp if s.mode == 4 else np.nan)
        b["err"].append(s.err); b["target_err"].append(s.target_err if s.mode == 5 else np.nan)
        b["rate"].append(s.rate); b["rate_ref"].append(s.rate_ref)
        b["wheel"].append(s.wheel_rpm); b["wheel_cmd"].append(s.wheel_cmd_rpm)
        if self.csv_writer:
            self.csv_writer.writerow([s.t_ms, s.mode, s.trk, s.flags, s.heading, s.rate, s.bias, s.err, s.rate_ref,
                                      s.wheel_rpm, s.wheel_cmd_rpm, s.torque, s.vbat, s.target_err, s.cam_age])

    def tick(self):
        for kind, obj in self.link.poll():
            if kind == "state":
                self.on_state(obj)
            elif kind == "vision":
                self.last_vis = obj; self.vis_times.append(time.time())
            elif kind == "image":
                self.img_item.setImage(obj[1].T, levels=(0, 255))
        v = self.last_vis
        if v is not None:
            if v.found:
                self.dot.setData([v.x / 4.0], [v.y / 4.0])
            else:
                self.dot.setData([], [])
        if len(self.buf["t"]) > 1:
            t = np.fromiter(self.buf["t"], float)
            for k, c in self.curves.items():
                c.setData(t, np.fromiter(self.buf[k], float))
        s = self.last_state
        fps = (len(self.vis_times) - 1) / max(1e-3, self.vis_times[-1] - self.vis_times[0]) if len(self.vis_times) > 2 else 0.0
        self.lbl["link"].setText("sem dados" if s is None else "OK")
        self.lbl["rates"].setText(f"{self.link.n_rx} / {self.link.n_tx}")
        if s:
            self.lbl["mode"].setText(P.MODES.get(s.mode, "?")); self.lbl["trk"].setText(P.TRK.get(s.trk, "?"))
            self.lbl["flags"].setText(" ".join(n for b, n in P.FLAGS.items() if s.flag(b)) or "—")
            self.lbl["vbat"].setText(f"{s.vbat:.1f} V"); self.lbl["cam_age"].setText("—" if s.cam_age < 0 else f"{s.cam_age * 1000:.0f} ms")
        if v:
            self.lbl["vision"].setText(f"{'ACHOU' if v.found else 'não vê'}  {fps:4.1f} fps  {v.proc_us / 1000:.1f} ms  err={v.err_x:+.1f}°")

    def closeEvent(self, e):
        self._toggle_log(False)
        super().closeEvent(e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--serial", default=None, help="porta série (ex.: COM5) em vez de UDP")
    ap.add_argument("--baud", type=int, default=921600)
    a = ap.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    link = SerialLink(a.serial, a.baud) if a.serial else UdpLink(a.host, a.port)
    w = MainWindow(link, f"Cubo - estação de terra  ({a.serial or f'{a.host}:{a.port}'})")
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
