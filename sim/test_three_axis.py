"""Testa dinâmica e controlo das três rodas, sem rede nem hardware."""
from pathlib import Path
import sys
import math
import unittest
import numpy as np
sys.path[:0]=[str(Path(__file__).resolve().parent), str(Path(__file__).resolve().parents[1]/'groundstation')]
from three_axis import ThreeAxisSimulation
from sil import CmdT, CMD_MODE
from gs_protocol import decode_sim3
from cube_sim import CubeSim


class ThreeAxisTests(unittest.TestCase):
    def test_combined_shoves_and_roll_control(self):
        for seed,x,y,roll in ((1,20,15,0),(2,-20,-15,0),(3,45,-25,10),(4,90,30,0),(5,-90,-30,0)):
            with self.subTest(seed=seed,x=x,y=y):
                model=ThreeAxisSimulation(seed=seed)
                model.run(1.7);model.command(CmdT(CMD_MODE,0,5,0));model.run(4)
                model.shove(x,y,roll_deg=roll);model.run(4.25)
                peaks=decode_sim3(model.telemetry())
                # Verificar atuação, sem exigir um comando mínimo arbitrário:
                # o anti-windup pode limitar a integração durante o empurrão.
                self.assertGreater(abs(peaks.commands[1]),1)
                self.assertGreater(abs(peaks.commands[2]),1)
                if roll:self.assertGreater(abs(peaks.commands[0]),20)
                model.run(12)
                state=decode_sim3(model.telemetry())
                # Medida contra a orientação física, não contra o estimador/desenho.
                body=model.sim._dcm().T @ np.array([1.,0.,0.])
                error=math.degrees(math.acos(np.clip(body[0],-1,1)))
                self.assertLess(error,2)
                self.assertLess(abs(math.degrees(model.plants['roll'].psi)),1)
                self.assertLess(max(map(abs,state.errors)),2)
                self.assertEqual(len(state.wheels),3)
                model.command(CmdT(CMD_MODE,0,1,0));model.run(12.02)
                self.assertTrue(all(not p.enable for p in model.plants.values()))
                print(f'PASS: X={x:+} Y={y:+} roll={roll:+}; erro físico final={error:.3f}°',flush=True)

    def test_clean_camera(self):
        sim=CubeSim(psi0_deg=0)
        image=sim.render()
        self.assertLess(image[120,160],80)
        self.assertGreater(image[58,160],140) # A antiga barra estava aqui.


if __name__=='__main__':unittest.main()
