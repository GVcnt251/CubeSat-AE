"""Verifica alinhamento geométrico e continuidade da animação sem hardware/rede."""
import os
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'groundstation'), str(ROOT / 'sim')]
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5 import QtWidgets, QtGui
from gs_app import Cube3DWidget, PoseBuffer
from cube_sim import CubeSim
from sil import Core, Runner, CMD_MODE, MODE_TRACK


class VisualTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_camera_arrow_meets_target_in_all_views(self):
        widget = Cube3DWidget()
        try:
            for yaw in (-180, -30, 0, 45, 179, 360):
                widget.yaw_deg = widget.target_yaw_deg = yaw
                axis, target = widget.pointing_geometry()
                np.testing.assert_allclose(axis[-1], target[0], atol=1e-12)
                for az in (-100, -38, 70):
                    for el in (-60, 26, 70):
                        widget.cam_azim, widget.cam_elev = az, el
                        np.testing.assert_allclose(widget._project(axis, 340, 280)[-1],
                                                   widget._project(target, 340, 280)[0], atol=1e-10)
            widget.target_yaw_deg = None
            self.assertIsNone(widget.pointing_geometry()[1])
        finally:
            widget.animation.stop()

    def test_intermediate_frames_and_shortest_angle(self):
        poses = PoseBuffer()
        poses.push(0, 179, 0, 30, 10.0)
        poses.push(50, -179, 120, 30, 10.05)
        yaw, rpm, target = poses.pose(10.125)
        self.assertAlmostEqual(yaw, 180)
        self.assertAlmostEqual(rpm, 60)
        self.assertAlmostEqual(target, 30)
        # Sem pacotes novos, a orientação pára na última medida.
        self.assertAlmostEqual(poses.pose(20)[0], 181)

    def test_restart_and_reconnect(self):
        poses = PoseBuffer()
        poses.push(5000, 30, 0, 10, 10)
        poses.push(0, -20, 0, None, 10.1)
        self.assertAlmostEqual(poses.pose(10.2)[0], -20)
        self.assertIsNone(poses.pose(10.2)[2])
        poses.push(2000, 40, 0, 20, 12.1)
        self.assertAlmostEqual(poses.pose(12.2)[0], 40)

    def test_actual_shove_is_interpolated_between_packets(self):
        sim, core = CubeSim(), Core()
        runner = Runner(sim, core)
        runner.at(1.6, lambda s, c: c.command(CMD_MODE, MODE_TRACK))
        runner.at(4.0, lambda s, c: s.shove(20, 0.25))
        log = runner.run(6)
        poses = PoseBuffer()
        times = log.arr('t')[::10]  # A estação recebe o estado a 20 Hz.
        headings = np.degrees(log.arr('heading_est')[::10])
        output, held, i = [], [], 0
        for now in np.arange(3.8, 5.8, 1 / 60):
            while i < len(times) and times[i] <= now:
                poses.push(round(times[i] * 1000), headings[i], 0, 0, times[i])
                i += 1
            output.append(poses.pose(now)[0])
            held.append(headings[max(0, i - 1)])
        # A animação tem mais posições intermédias e saltos menores que a atualização direta.
        self.assertGreater(np.count_nonzero(np.abs(np.diff(output)) > 1e-5),
                           np.count_nonzero(np.abs(np.diff(held)) > 1e-5))
        self.assertLess(np.max(np.abs(np.diff(output))), np.max(np.abs(np.diff(held))))


if __name__ == '__main__':
    unittest.main()
