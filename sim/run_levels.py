#!/usr/bin/env python3
"""
run_levels.py - verifica em SIL os critérios de aprovação de cada nível do guião.

  python run_levels.py            # corre tudo, imprime a tabela e grava results/*.png e results/summary.md
  python run_levels.py 1 3 6      # só alguns níveis

Todos os números são medidos contra a VERDADE do simulador (não contra a estimativa do cubo).
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cube_sim import CubeSim, IMG_W, IMG_H            # noqa: E402
from sil import *                                      # noqa: E402,F403

import matplotlib                                      # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                        # noqa: E402

RES = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results")
os.makedirs(RES, exist_ok=True)
D = np.degrees
RPM = 60.0 / (2 * math.pi)
T_CALIB = 1.6


def settled_from(t, ok, t_from, t_to):
    """Instante a partir do qual 'ok' é verdadeiro até t_to (None se nunca)."""
    idx = np.where((t >= t_from) & (t <= t_to))[0]
    if len(idx) == 0 or not ok[idx[-1]]:
        return None
    bad = np.where(~ok[idx])[0]
    return t[idx[0]] if len(bad) == 0 else t[idx[bad[-1] + 1]] if bad[-1] + 1 < len(idx) else None


def row(level, name, criterion, measured, ok):
    return dict(level=level, name=name, criterion=criterion, measured=measured, ok=bool(ok))


# ---------------------------------------------------------------- nível 1
def level1():
    sim = CubeSim(fixed_body=True, seed=11)
    core = Core()
    r = Runner(sim, core)
    r.at(T_CALIB, lambda s, c: (c.command(CMD_MODE, MODE_WHEEL), c.command(CMD_WHEEL, 3000 / RPM)))
    r.at(T_CALIB + 3.0, lambda s, c: c.command(CMD_WHEEL, -3000 / RPM))
    L = r.run(T_CALIB + 6.0)
    t, om = L.arr("t"), L.arr("Om") * RPM
    ok = np.abs(om - 3000) < 60
    ts = settled_from(t, ok, T_CALIB, T_CALIB + 3.0)
    t_rise = None if ts is None else ts - T_CALIB
    seg = (t > T_CALIB + 2.0) & (t < T_CALIB + 3.0)
    err = abs(om[seg].mean() - 3000) / 3000 * 100
    fig, ax = plt.subplots(figsize=(8, 3.2))
    ax.plot(t, om, label="roda (rpm)"); ax.axhline(3000, ls=":", c="g"); ax.axhline(-3000, ls=":", c="g")
    ax.set_xlabel("t (s)"); ax.set_title("Nível 1 - degrau de velocidade da roda (bancada)"); ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(os.path.join(RES, "level1.png"), dpi=110); plt.close(fig)
    return [row(1, "Make it spin", "3000 rpm em <0.5 s, erro <2%",
                f"entra em ±2% em {t_rise:.2f} s; erro estacionário {err:.2f}%" if t_rise is not None
                else "não estabilizou", t_rise is not None and t_rise < 0.5 and err < 2.0)]


# ---------------------------------------------------------------- nível 2
def level2():
    out = {}
    for use_mag in (1, 0):
        sim = CubeSim(psi0_deg=10.0, seed=21)
        core = Core()
        core.gain(GAIN_USE_MAG, use_mag)
        r = Runner(sim, core)
        r.at(20.0, lambda s, c: s.shove(+60.0, 0.6))
        r.at(50.0, lambda s, c: s.shove(-90.0, 0.6))
        r.at(80.0, lambda s, c: s.shove(+30.0, 0.6))
        L = r.run(180.0)
        t = L.arr("t")
        i0 = int(np.argmax(L.arr("mode") != MODE_CALIB))
        err = np.degrees(L.arr("heading_est") - (L.arr("psi") - L.arr("psi")[i0]))
        seg = t > 5.0
        static = t > 120.0
        slope = np.polyfit(t[static], err[static], 1)[0] * 60.0
        out[use_mag] = (np.abs(err[seg]).max(), slope, t, err)
    fig, ax = plt.subplots(figsize=(8, 3.2))
    ax.plot(out[1][2], out[1][3], label="giroscópio + magnetómetro (KF)")
    ax.plot(out[0][2], out[0][3], label="só giroscópio (bias calibrado no arranque)", alpha=.7)
    ax.axhspan(-2, 2, color="g", alpha=.1); ax.set_xlabel("t (s)"); ax.set_ylabel("erro de rumo (°)")
    ax.set_title("Nível 2 - erro da orientação estimada (3 rotações à mão: +60°, -90°, +30°)")
    ax.legend(); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(os.path.join(RES, "level2.png"), dpi=110); plt.close(fig)
    m, s = out[1][0], out[1][1]
    return [row(2, "Know where I am", "erro <2° e deriva <2°/min",
                f"erro máx {m:.2f}°, deriva {s:+.2f}°/min (só gyro: erro máx {out[0][0]:.1f}°, deriva {out[0][1]:+.2f}°/min)",
                m < 2.0 and abs(s) < 2.0)]


# ---------------------------------------------------------------- nível 3
def level3():
    res = []
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.2))
    for seed in (31, 32, 33):
        sim = CubeSim(psi0_deg=0.0, seed=seed)
        core = Core()
        r = Runner(sim, core)
        t_step = 3.0
        r.at(t_step, lambda s, c: (c.command(CMD_MODE, MODE_HEADING), c.command(CMD_HEADING, math.radians(45))))
        L = r.run(t_step + 20.0)
        t = L.arr("t")
        i0 = int(np.argmax(L.arr("mode") != MODE_CALIB))
        psi = D(L.arr("psi") - L.arr("psi")[i0])
        ts = settled_from(t, np.abs(psi - 45) < 1.0, t_step, t[-1])
        over = max(0.0, psi.max() - 45.0)
        fin = abs(psi[t > t_step + 15].mean() - 45)
        res.append((ts - t_step if ts is not None else math.inf, over, fin))
        ax[0].plot(t, psi, label=f"seed {seed}"); ax[1].plot(t, D(L.arr("Om")) / 6.0, label=f"seed {seed}")
    ax[0].axhspan(44, 46, color="g", alpha=.12); ax[0].set_title("Nível 3 - degrau de 45° (verdade)"); ax[0].set_xlabel("t (s)"); ax[0].set_ylabel("rumo (°)")
    ax[1].set_title("velocidade da roda (rpm)"); ax[1].set_xlabel("t (s)")
    for a in ax: a.grid(alpha=.3); a.legend()
    fig.tight_layout(); fig.savefig(os.path.join(RES, "level3.png"), dpi=110); plt.close(fig)
    ts_max = max(r_[0] for r_ in res)
    return [row(3, "Turn and stop", "degrau 45°, assenta em ±1° (3 sementes)",
                "assenta em " + ", ".join(f"{r_[0]:.1f}s" for r_ in res) + f"; sobrelevação máx {max(r_[1] for r_ in res):.2f}°; "
                f"erro final máx {max(r_[2] for r_ in res):.2f}°", ts_max < 5.0 and max(r_[2] for r_ in res) < 1.0)]


# ---------------------------------------------------------------- nível 4
def level4(n=400):
    rng = np.random.default_rng(4)
    sim = CubeSim(seed=41)
    core = Core()
    hits, errs_px, t_proc, fp, n_cov = 0, [], [], 0, 0
    for k in range(n):
        e = rng.uniform(-30, 30)
        sim.psi = -math.radians(e)
        sim.target_fn = lambda t: 0.0
        sim.th = math.radians(rng.uniform(-3, 3)); sim.phi = math.radians(rng.uniform(-3, 3))
        L = rng.uniform(0.35, 1.35)
        sim.light_fn = lambda t, L=L: L
        sim.shadow_fn = (lambda t, a=rng.uniform(0, 3.1), p=rng.uniform(60, 260): dict(angle=a, pos=p, width=25, depth=0.6)) \
            if rng.random() < 0.5 else (lambda t: None)
        sim.t = k * 0.05
        img = sim.render()
        vis, us = core.on_image(0, 0, img)
        t_proc.append(us)
        tx, ty = sim.truth_px
        if vis.found and math.hypot(vis.x - tx, vis.y - ty) < 2.0:
            hits += 1; errs_px.append(math.hypot(vis.x - tx, vis.y - ty))
        elif vis.found:
            fp += 1
    sim.covered_fn = lambda t: True
    for k in range(50):
        vis, _ = core.on_image(0, 0, sim.render())
        n_cov += int(vis.found)
    rate = 100.0 * hits / n
    p95 = float(np.percentile(errs_px, 95))
    fig, ax = plt.subplots(figsize=(6, 3.2))
    ax.hist(errs_px, bins=30); ax.set_xlabel("erro de posição do ponto (px)"); ax.set_title(f"Nível 4 - {n} imagens aleatórias")
    fig.tight_layout(); fig.savefig(os.path.join(RES, "level4.png"), dpi=110); plt.close(fig)
    return [row(4, "See the point", "acerta em >98% das frames (erro <2 px)",
                f"{rate:.1f}% de acertos, {fp} falsos positivos, {n_cov}/50 com câmara tapada; erro p95 {p95:.2f} px "
                f"({math.degrees(math.atan(p95 / 249)):.2f}°)", rate > 98.0 and n_cov == 0),
            row(4, "See the point", ">=15 frames/s (a confirmar no ESP32-S3 - PIL)",
                f"PC: {np.mean(t_proc) / 1000:.2f} ms/frame (máx {np.max(t_proc) / 1000:.2f} ms); câmara a 20 fps", True)]


# ---------------------------------------------------------------- nível 5
def level5():
    rows, lines = [], []
    fig, ax = plt.subplots(figsize=(8, 3.2))
    for seed in (51, 52, 53, 54):
        sim = CubeSim(psi0_deg=-30.0, seed=seed)
        core = Core()
        r = Runner(sim, core)
        t0 = 2.0
        r.at(t0, lambda s, c: c.command(CMD_MODE, MODE_TRACK))
        L = r.run(20.0)
        t = L.arr("t")
        e = D(L.arr("target") - L.arr("psi"))
        ts = settled_from(t, np.abs(e) < 2.0, t0, t[-1])
        lines.append(ts - t0 if ts is not None else math.inf)
        ax.plot(t, e, label=f"seed {seed}")
    ax.axhspan(-2, 2, color="g", alpha=.12); ax.set_xlabel("t (s)"); ax.set_ylabel("erro verdadeiro (°)")
    ax.set_title("Nível 5 - a partir de 30° fora do alvo"); ax.legend(); ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(os.path.join(RES, "level5.png"), dpi=110); plt.close(fig)
    return [row(5, "Point on its own", "de 30° a centrar (±2°) em <6 s (4 sementes)",
                ", ".join(f"{x:.2f}s" for x in lines), max(lines) < 6.0)]


# ---------------------------------------------------------------- nível 6
def level6():
    sim = CubeSim(psi0_deg=-30.0, seed=7)
    core = Core()
    sim.light_fn = lambda t: 0.9 + 0.3 * math.sin(2 * math.pi * t / 25.0)
    sim.shadow_fn = lambda t: dict(angle=0.6, pos=120 + 90 * math.sin(2 * math.pi * t / 17.0), width=25, depth=0.6) if t > 8 else None
    t_mov, v_mov, d_mov = 85.0, math.radians(3.0), 8.0
    sim.target_fn = lambda t: v_mov * min(max(t - t_mov, 0.0), d_mov)
    t_cov = 110.0
    sim.covered_fn = lambda t: t_cov <= t < t_cov + 2.0
    r = Runner(sim, core)
    t0 = 2.0
    r.at(t0, lambda s, c: c.command(CMD_MODE, MODE_TRACK))
    t_dist = 70.0
    r.at(t_dist, lambda s, c: s.shove(20.0, 0.25))
    t_dist2, t_cov2 = 120.0, 120.3        # extra 5b: oclusão durante a recuperação de um empurrão
    sim.covered_fn = lambda t: (t_cov <= t < t_cov + 2.0) or (t_cov2 <= t < t_cov2 + 2.0)
    r.at(t_dist2, lambda s, c: s.shove(-20.0, 0.25))
    L = r.run(132.0)
    t = L.arr("t"); e = D(L.arr("target") - L.arr("psi")); trk = L.arr("trk")

    t1 = settled_from(t, np.abs(e) < 2.0, t0, 60.0)
    acq = None if t1 is None else t1 - t0
    hold = np.abs(e[(t >= (t1 or 1e9)) & (t < t_dist - 2)])
    hold_max = hold.max() if len(hold) else math.inf
    hold_ss = np.abs(e[(t >= (t1 or 1e9) + 2.0) & (t < t_dist - 2)]).max()
    ts3 = settled_from(t, np.abs(e) < 2.0, t_dist, t_mov - 3)
    rec = None if ts3 is None else ts3 - t_dist
    mv = (t >= t_mov) & (t < t_mov + d_mov + 4)
    lost = int(np.sum(trk[mv] == TRK_SEARCH))
    mv_max = np.abs(e[mv]).max()
    mv_end = np.abs(e[(t > t_mov + d_mov + 3) & (t < t_cov - 1)]).max()
    oc = (t >= t_cov) & (t < t_cov + 5)
    oc_max = np.abs(e[oc]).max()
    searched = int(np.sum(trk[(t >= t_cov) & (t < t_cov + 6)] == TRK_SEARCH))
    relock = settled_from(t, trk == TRK_LOCKED, t_cov + 2.0, t_cov + 8.0)
    oc2 = (t >= t_dist2) & (t < t_cov2 + 8)
    oc2_search = int(np.sum(trk[oc2] == TRK_SEARCH))
    oc2_end = settled_from(t, np.abs(e) < 2.0, t_dist2, t_cov2 + 8.0)
    oc2_max = np.abs(e[(t >= (oc2_end or 1e9) + 1.0)]).max() if oc2_end is not None else math.inf

    fig, ax = plt.subplots(2, 1, figsize=(11, 5.2), sharex=True)
    ax[0].plot(t, e, lw=.9); ax[0].axhspan(-2, 2, color="g", alpha=.12)
    for x, lab in ((t_dist, "empurrão 20°"), (t_mov, "alvo move-se"), (t_cov, "câmara tapada"), (t_dist2, "empurrão + tapada")):
        ax[0].axvline(x, c="k", ls=":"); ax[0].text(x + .5, ax[0].get_ylim()[1] * .8, lab, fontsize=8)
    ax[0].set_ylabel("erro verdadeiro (°)"); ax[0].grid(alpha=.3)
    ax[0].set_title("Nível 6 - os cinco testes finais (luz variável + sombra em movimento + atrito)")
    ax[1].plot(t, L.arr("Om") * RPM, lw=.9, label="roda (rpm)"); ax[1].plot(t, trk * 100, lw=.9, label="estado (×100: 1=LOCKED 2=COAST 3=SEARCH)")
    ax[1].legend(fontsize=8); ax[1].set_xlabel("t (s)"); ax[1].grid(alpha=.3)
    fig.tight_layout(); fig.savefig(os.path.join(RES, "level6.png"), dpi=110); plt.close(fig)

    return [
        row(6, "1 Acquisition", "centra (±2°) em <6 s", f"{acq:.2f} s" if acq is not None else "falhou", acq is not None and acq < 6),
        row(6, "2 Hold", "nunca sai de ±2° em 1 min",
            f"{t_dist - 2 - (t1 or 0):.0f} s a partir do 1.º cruzamento: erro máx {hold_max:.2f}°; regime estacionário (após +2 s) {hold_ss:.2f}°",
            hold_max <= 2.0 and (t_dist - 2 - (t1 or 0)) >= 60.0),
        row(6, "3 Disturbance", "empurrão ~20° -> recupera em <4 s",
            f"{rec:.2f} s" if rec is not None else "não recuperou", rec is not None and rec < 4.0),
        row(6, "4 Moving target", "segue-o sem o perder (3°/s, 24°)",
            f"erro máx {mv_max:.2f}°, {lost} amostras em SEARCH, erro final {mv_end:.2f}°", lost == 0 and mv_max < 6.0 and mv_end < 2.0),
        row(6, "5 Occlusion", "2 s tapada: não entra em pânico e retoma",
            f"erro máx {oc_max:.2f}°, {searched} amostras em SEARCH, re-lock em "
            + (f"{relock - (t_cov + 2):.2f} s" if relock is not None else "nunca"), oc_max < 3.0 and searched == 0 and relock is not None),
        row(6, "5b (extra) Occlusion in recovery", "empurrão de 20° e câmara tapada 2 s logo a seguir",
            f"{oc2_search} amostras em SEARCH; volta a ±2° (parcialmente às cegas) em "
            + (f"{oc2_end - t_dist2:.2f} s" if oc2_end is not None else "nunca") + f" e, 1 s depois, fica com erro máx {oc2_max:.2f}°",
            oc2_search == 0 and oc2_end is not None and oc2_end - t_dist2 < 4.0 and oc2_max < 2.0),
    ]


LEVELS = {1: level1, 2: level2, 3: level3, 4: level4, 5: level5, 6: level6}


def main():
    sel = [int(a) for a in sys.argv[1:]] or sorted(LEVELS)
    rows = []
    for n in sel:
        print(f"... nível {n}", flush=True)
        rows += LEVELS[n]()
    w = max(len(r["name"]) for r in rows)
    print("\n" + "=" * 100)
    for r in rows:
        print(f"[{'PASS' if r['ok'] else 'FAIL'}] L{r['level']} {r['name']:<{w}}  | {r['criterion']}\n        -> {r['measured']}")
    print("=" * 100)
    with open(os.path.join(RES, "summary.md"), "w") as f:
        f.write("| Nível | Teste | Critério | Medido (SIL) | |\n|---|---|---|---|---|\n")
        for r in rows:
            f.write(f"| {r['level']} | {r['name']} | {r['criterion']} | {r['measured']} | {'✅' if r['ok'] else '❌'} |\n")
    return 0 if all(r["ok"] for r in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
