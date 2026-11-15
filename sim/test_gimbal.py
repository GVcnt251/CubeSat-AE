"""Gravidade, geometria dos planos e referência inicial: validação contra física/geometria."""
from pathlib import Path
import sys
import math
import unittest
import numpy as np
sys.path[:0]=[str(Path(__file__).resolve().parent),str(Path(__file__).resolve().parents[1]/'groundstation')]
from gimbal import GimbalParams,relative_angles,rotation
from three_axis import ThreeAxisSimulation
from sil import CmdT,CMD_MODE,CMD_RECAL
from gs_protocol import decode_gimbal


class GimbalTests(unittest.TestCase):
    def test_center_of_mass_on_pivot_has_no_gravity_torque(self):
        params=GimbalParams(mass_kg=2)
        for angles in ((0,0,0),(.2,.3,.4),(-.4,.5,2)):
            torque,body_g=params.loads(angles)
            np.testing.assert_allclose(torque,0,atol=1e-12)
            self.assertAlmostEqual(np.linalg.norm(body_g),params.g)

    def test_gravity_torque_matches_negative_potential_gradient(self):
        params=GimbalParams(mass_kg=1.3,com_x=.002,com_y=-.003,com_z=.009)
        q=np.array([.25,-.32,.7]);h=1e-6
        finite=[]
        for i in range(3):
            plus=q.copy();minus=q.copy();plus[i]+=h;minus[i]-=h
            finite.append(-(params.potential(plus)-params.potential(minus))/(2*h))
        torque,_=params.loads(q)
        np.testing.assert_allclose(torque,finite,atol=1e-9)
        self.assertAlmostEqual(torque[2],0,places=12)

    def test_offset_creates_restoring_motion_in_passive_gimbal(self):
        balanced=ThreeAxisSimulation(psi0=0);offset=ThreeAxisSimulation(psi0=0)
        offset.configure_gimbal(GimbalParams(com_z=.005))
        for model in (balanced,offset):model.plants['pitch'].psi=math.radians(20)
        balanced.run(.05);offset.run(.05)
        self.assertAlmostEqual(balanced.plants['pitch'].w,0)
        self.assertLess(offset.plants['pitch'].w,-.1)
        self.assertLess(offset.plants['pitch'].psi,balanced.plants['pitch'].psi)

    def test_relative_angles_and_planes(self):
        zero=np.zeros(3)
        for q,expected in (([0,0,30],[0,30,30]),([20,0,0],[20,20,0]),([0,-25,0],[25,0,25])):
            rel,planes=relative_angles(np.radians(q),zero)
            np.testing.assert_allclose(np.degrees(rel),q,atol=1e-10)
            np.testing.assert_allclose(np.degrees(planes),expected,atol=1e-10)
        ref=np.radians([12,-18,40])
        rel,planes=relative_angles(ref,ref)
        np.testing.assert_allclose(rel,0,atol=1e-12)
        np.testing.assert_allclose(planes,0,atol=3e-8)
        # Planos não orientados: uma meia-volta não inclina o mesmo plano.
        rel,planes=relative_angles([0,0,math.pi],zero)
        np.testing.assert_allclose(planes,0,atol=1e-10)
        self.assertAlmostEqual(abs(rel[2]),math.pi)

    def test_reference_is_fixed_and_independent_of_calibration(self):
        model=ThreeAxisSimulation()
        ref=model.reference.copy();model.run(2.1)
        self.assertTrue(model.capture_reference())
        model.command(CmdT(CMD_MODE,0,5,0));model.run(4)
        self.assertFalse(model.capture_reference())
        model.command(CmdT(CMD_RECAL,0,0,0))
        np.testing.assert_allclose(model.reference,ref,atol=1e-12)
        state=decode_gimbal(model.gimbal_telemetry())
        np.testing.assert_allclose(state.reference,np.degrees(ref),atol=1e-5)
        self.assertAlmostEqual(state.relative[2],30,delta=1)

    def test_mass_inertia_and_control_with_gravity(self):
        model=ThreeAxisSimulation()
        self.assertTrue(model.configure_gimbal(GimbalParams(mass_kg=2,com_z=.005)))
        self.assertGreater(model.plants['pitch'].p.J_b,.008)
        model.run(1.7);model.command(CmdT(CMD_MODE,0,5,0));model.run(5)
        model.shove(20,15,roll_deg=8);model.run(13)
        body=model.sim._dcm().T @ np.array([1.,0.,0.])
        self.assertLess(math.degrees(math.acos(np.clip(body[0],-1,1))),2)
        self.assertFalse(model.configure_gimbal(GimbalParams(mass_kg=-1)))


if __name__=='__main__':unittest.main()
