"""Regressão do controlo: mede erro físico, não apenas a seta desenhada."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cube_sim import CubeSim
from sil import Core, Runner, CMD_MODE, MODE_TRACK, TRK_LOCKED


def error_deg(log):
    return (np.degrees(log.arr('target') - log.arr('psi')) + 180) % 360 - 180


class TrackingTests(unittest.TestCase):
    def test_shoves_in_both_directions_and_full_turns(self):
        for angle in (20, -20, 60, -60, 120, -120, 180, 360, -360):
            with self.subTest(angle=angle):
                sim, core = CubeSim(), Core()
                runner = Runner(sim, core)
                runner.at(1.6, lambda s, c: c.command(CMD_MODE, MODE_TRACK))
                runner.at(4, lambda s, c, a=angle: s.shove(a, 0.25))
                log = runner.run(16)
                t, e = log.arr('t'), error_deg(log)
                self.assertGreater(np.abs(e[(t > 4) & (t < 5)]).max(), 5)
                self.assertGreater(np.abs(log.arr('wheel_cmd')[t > 4]).max(), 5)
                self.assertLess(np.abs(e[t > 14]).max(), 2)
                self.assertEqual(log.trk[-1], TRK_LOCKED)
                print(f'PASS: {angle:+}°; erro máx nos 2 s finais = {np.abs(e[t > 14]).max():.3f}°', flush=True)

    def test_repeated_shoves_keep_the_same_physical_target(self):
        sim, core = CubeSim(seed=2), Core()
        runner = Runner(sim, core)
        runner.at(1.6, lambda s, c: c.command(CMD_MODE, MODE_TRACK))
        for when, angle in ((4, 20), (8, -60), (14, 120), (24, -180), (36, 360)):
            runner.at(when, lambda s, c, a=angle: s.shove(a, 0.25))
        log = runner.run(50)
        t, e = log.arr('t'), error_deg(log)
        self.assertTrue(np.all(log.arr('target') == 0))
        for start, end in ((6, 7.5), (12, 13.5), (22, 23.5), (34, 35.5), (48, 50)):
            self.assertLess(np.abs(e[(t > start) & (t < end)]).max(), 2)

    def test_camera_respects_full_turn_and_hidden_backside(self):
        front = CubeSim(psi0_deg=0).render()
        full_turn = CubeSim(psi0_deg=360).render()
        np.testing.assert_array_equal(front, full_turn)
        sim = CubeSim(psi0_deg=180)
        sim.render()
        self.assertTrue(np.isnan(sim.truth_px).all())


if __name__ == '__main__':
    unittest.main()
