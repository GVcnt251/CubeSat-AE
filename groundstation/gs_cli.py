#!/usr/bin/env python3
"""gs_cli.py - estação de terra mínima em texto (sem Qt). Exemplo:
     python gs_cli.py --port 5000 --mode track --seconds 10
"""
import argparse
import time

import gs_protocol as P
from gs_link import UdpLink

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=5000)
ap.add_argument("--mode", choices=["idle", "wheel", "rate", "heading", "track"], default=None)
ap.add_argument("--value", type=float, default=0.0, help="rpm (wheel), °/s (rate) ou ° (heading)")
ap.add_argument("--seconds", type=float, default=10.0)
a = ap.parse_args()

link = UdpLink(a.host, a.port)
link.send_frame(P.tc_ping)
link.send_frame(P.tc_stream, 10, 10, 0)
# O núcleo ignora mudanças de modo durante CALIB. Espera por telemetria válida.
deadline = time.monotonic() + 10.0
ready = False
next_ping = 0.0
while time.monotonic() < deadline and not ready:
    if time.monotonic() >= next_ping:
        link.send_frame(P.tc_ping)
        link.send_frame(P.tc_stream, 10, 10, 0)
        next_ping = time.monotonic() + 1.0
    for kind, obj in link.poll():
        if kind == "state" and obj.mode != 0:
            ready = True
    time.sleep(0.01)
if not ready:
    link.close()
    ap.exit(1, "Sem telemetria calibrada em 10 s. Verifica o simulador, host e porta.\n")
if a.mode:
    mode = {"idle": 1, "wheel": 2, "rate": 3, "heading": 4, "track": 5}[a.mode]
    if a.mode == "wheel": link.send_frame(P.tc_wheel, a.value)
    if a.mode == "rate": link.send_frame(P.tc_rate, a.value)
    if a.mode == "heading": link.send_frame(P.tc_heading, a.value)
    link.send_frame(P.tc_mode, mode)
t_end, last = time.time() + a.seconds, 0
while time.time() < t_end:
    for kind, obj in link.poll():
        if kind == "state" and time.time() - last > 0.5:
            last = time.time()
            s = obj
            print(f"t={s.t_ms/1000:6.2f}s {P.MODES[s.mode]:8s} {P.TRK[s.trk]:7s} rumo={s.heading:7.2f}° erro={s.err:7.2f}° "
                  f"alvo_est={s.target_err:7.2f}° roda={s.wheel_rpm:7.0f}rpm flags={s.flags:#04x}")
    time.sleep(0.01)
link.close()
