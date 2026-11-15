"""Phase 7: non-zero pointing setpoints, through TMTC -> core -> simulator.

Run from a project copy containing core/, sim/ and groundstation/:
    python core/tests/test_aim_sil.py
"""
from pathlib import Path
import ctypes as C
import sys
import numpy as np

root = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(root / 'sim'), str(root / 'groundstation')]
from cube_sim import CubeSim
from sil import Core, Runner, CMD_MODE, MODE_TRACK
import cproto
import gs_protocol as gs

for aim in (10.0, -10.0):
    sim, core = CubeSim(psi0_deg=-20, seed=71), Core()
    runner = Runner(sim, core)
    def set_aim(s, c):
        frame, = gs.Parser().feed(gs.tc_aim(1, aim))
        result, cmd = cproto.tc_to_cmd(frame[0], frame[2])
        assert result == 1
        c.L.cc_command(c.h, C.byref(cmd))
        c.command(CMD_MODE, MODE_TRACK)
    runner.at(2.0, set_aim)
    log = runner.run(14.0)
    error = np.degrees(log.arr('target') - log.arr('psi')) - aim
    worst = float(np.max(np.abs(error[log.arr('t') >= 10.0])))
    assert worst < 2.0, (aim, worst)
    print(f'PASS: AIM {aim:+.0f} deg, maximum steady error {worst:.3f} deg')
