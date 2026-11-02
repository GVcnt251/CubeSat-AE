"""Regressões de saturação no simulador sintético do gimbal (sem webcam)."""
from pathlib import Path
import sys
import math
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from three_axis import ThreeAxisSimulation
from sil import CmdT,CMD_MODE

RPM=60/(2*math.pi)


class SaturationTests(unittest.TestCase):
    def test_repeated_pushes_same_direction(self):
        model=ThreeAxisSimulation();model.run(1.7);model.command(CmdT(CMD_MODE,0,5,0))
        data=[];next_push=5.;last_push=1.7;count=0
        for t in np.arange(2,181,.05):
            if t>=next_push-1e-7 and next_push<=160:
                model.shove(20);last_push=model.sim.t;next_push+=5;count+=1
            model.run(float(t))
            tlm=model.cores['yaw'].tlm();wheel=model.plants['yaw'].Om*RPM
            error=(math.degrees(model.sim.psi)+180)%360-180
            data.append((t,wheel,error,bool(tlm.flags&32),t-last_push))
            self.assertFalse(tlm.flags&2,f'Roda saturada em {t:.2f}s')
        data=np.asarray(data)
        self.assertEqual(count,32)
        self.assertLess(np.abs(data[:,1]).max(),2800)
        self.assertTrue(data[:,3].any())
        self.assertLess(np.abs(data[data[:,4]>3,2]).max(),2)
        self.assertLess(abs(data[-1,2]),2)
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(2,1,figsize=(10,5.4),sharex=True)
        axes[0].plot(data[:,0],data[:,1],lw=1.3,color='#2365df')
        for limit in (-3000,3000):axes[0].axhline(limit,ls='--',color='#c2413a',lw=.8)
        axes[0].fill_between(data[:,0],-3000,3000,where=data[:,3]>0,color='#e6bc53',alpha=.16)
        axes[0].set_ylabel('Roda Z (rpm)');axes[0].set_ylim(-3200,3200)
        axes[0].set_title('32 empurrões de +20° no mesmo sentido — limite físico mantido em 3000 rpm')
        axes[1].plot(data[:,0],data[:,2],color='#16a394',lw=1)
        axes[1].axhspan(-2,2,color='#16a394',alpha=.13)
        axes[1].set_ylabel('Erro físico (graus)');axes[1].set_xlabel('Tempo simulado (s)')
        for ax in axes:ax.grid(alpha=.2)
        fig.tight_layout();fig.savefig(Path(__file__).resolve().parents[1]/'results/saturation_v9.png',dpi=125)
        plt.close(fig)
        print(f'PASS: 32 empurrões, pico {np.abs(data[:,1]).max():.0f} rpm, erro entre empurrões <2°',flush=True)

    def test_long_occlusion_uses_bounded_search(self):
        model=ThreeAxisSimulation();model.run(1.7);model.command(CmdT(CMD_MODE,0,5,0))
        model.sim.covered_fn=lambda t:10<t<60
        peaks=np.zeros(3);search_angles=[]
        for t in np.arange(2,85,.1):
            model.run(float(t))
            peaks=np.maximum(peaks,[abs(p.Om*RPM) for p in model.plants.values()])
            if 18<t<59:search_angles.append(np.degrees(model.physical_angles()))
            self.assertFalse(any(c.tlm().flags&2 for c in model.cores.values()))
        self.assertLess(peaks.max(),1000)
        self.assertLess(np.max(np.abs(np.asarray(search_angles)[:,2])),23)
        self.assertLess(np.max(np.abs(np.asarray(search_angles)[:,1])),18)
        self.assertLess(np.max(np.abs(np.degrees(model.physical_angles()))),.5)
        self.assertEqual(model.cores['yaw'].tlm().trk,1)
        print(f'PASS: oclusão de 50 s, pico das rodas {peaks.max():.0f} rpm e readquisição do alvo',flush=True)

    def test_unloading_uses_external_friction_not_wheel_reset(self):
        for sign,friction in ((1,True),(-1,True),(1,False)):
            with self.subTest(sign=sign,friction=friction):
                model=ThreeAxisSimulation();model.run(1.7);model.command(CmdT(CMD_MODE,0,5,0));model.run(4)
                model.command(CmdT(CMD_MODE,0,1,0));model.run(4.05)
                plant=model.plants['yaw']
                # Condição inicial do ensaio: roda já carregada; nunca imposto no código de produção.
                plant.Om=sign*2400/RPM
                if not friction:plant.p.gimbal_visc=plant.p.gimbal_coul=0
                model.run(4.15);model.command(CmdT(CMD_MODE,0,5,0))
                momentum=lambda:(plant.p.J_b+plant.p.J_w)*plant.w+plant.p.J_w*plant.Om
                initial=momentum();history=[];peak_error=0
                for t in np.arange(4.2,45,.1):
                    model.run(float(t));history.append(momentum())
                    peak_error=max(peak_error,abs(math.degrees(plant.psi)))
                if friction:
                    self.assertLess(abs(plant.Om*RPM),1700)
                    self.assertLess(abs(momentum()),abs(initial)*.8)
                    self.assertLess(peak_error,2)
                else:
                    self.assertLess(np.max(np.abs(np.asarray(history)-initial)),1e-8)
                    self.assertGreater(abs(plant.Om*RPM),2200)
        print('PASS: descarga nos dois sentidos; sem atrito externo o momento conserva-se',flush=True)


if __name__=='__main__':unittest.main()
