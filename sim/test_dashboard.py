"""Verifica comandos da interface e registo dos três eixos, sem usar a rede."""
import csv
import io
import os
from pathlib import Path
import struct
import sys
import unittest
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'groundstation'))
from PyQt5 import QtWidgets
from gs_app import MainWindow
import gs_protocol as P


class Link:
    n_rx=n_tx=0
    def __init__(self):self.frames=[]
    def send_frame(self,make,*args):self.frames.append(make(1,*args))
    def poll(self):return []
    def close(self):pass


class DashboardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    def setUp(self):self.link=Link();self.window=MainWindow(self.link)
    def tearDown(self):self.window.close()

    def click(self,text):
        next(b for b in self.window.findChildren(QtWidgets.QPushButton) if b.text()==text).click()
        return list(P.Parser().feed(self.link.frames[-1]))[0]

    def test_commands_and_advanced_settings(self):
        for text,expected in (('X  -20°',(-20,0)),('X  +20°',(20,0)),('Y  -10°',(0,-10)),('Y  +10°',(0,10))):
            ty,_,payload=self.click(text)
            self.assertEqual(ty,P.SIM_SHOVE_XY);self.assertEqual(struct.unpack('<ff',payload),expected)
        self.window.shove_x.setValue(-35);self.window.shove_y.setValue(17)
        self.assertEqual(struct.unpack('<ff',self.click('Aplicar X + Y')[2]),(-35,17))
        self.window.sp_aim.setValue(7)
        self.assertEqual(self.click('Aplicar pontaria')[0],P.TC_SET_AIM)
        self.click('FIXAR ALVO');self.assertTrue(self.window.track_pending)
        self.assertEqual(self.window.sp_aim.value(),0)
        self.click('IDLE — desligar controlo');self.assertFalse(self.window.track_pending)
        self.assertEqual(self.click('Tapar câmara')[0],P.SIM_COVER)
        self.assertEqual(self.click('Destapar câmara')[2],b'\0')
        texts=[b.text() for b in self.window.findChildren(QtWidgets.QPushButton)]
        self.assertFalse(any('Alvo +' in t or t in ('WHEEL','RATE','HEADING','TRACK') for t in texts))

    def test_three_axis_csv_and_saturation(self):
        w=self.window
        state=P.State(1000,5,1,13,30,0,0,0,0,100,100,.01,22.2,0,0.08)
        w.on_state(state)
        stream=io.StringIO();w.csv_writer=csv.writer(stream)
        multi=P.Sim3(1000,4,(0,0,30),(0,0,0),(1,2,3),(4,5,6),(.01,.02,.03),(0,0,0),(0,0,30))
        w.on_sim3(multi);w.tick()
        row=next(csv.reader(io.StringIO(stream.getvalue())))
        self.assertEqual(len(row),43)
        self.assertEqual([float(row[i]) for i in (7,13,19)],[1,2,3])
        self.assertIn('SATURAÇÃO',w.saturation.text())
        self.assertIn('Z',w.saturation.text())
        w.csv_writer=None
        multi.saturated=32
        w.on_sim3(multi);w.tick()
        self.assertIn('Alívio de momento',w.saturation.text())
        self.assertIn('Z',w.saturation.text())
        self.assertNotIn('SATURAÇÃO',w.saturation.text())
        multi.saturated=36
        w.on_sim3(multi);w.tick()
        self.assertIn('SATURAÇÃO',w.saturation.text()) # Saturação real tem prioridade sobre alívio.

    def test_gimbal_settings_reference_and_display_geometry(self):
        w=self.window
        w.mass.setValue(1.25)
        for field,value in zip(w.com_fields,(2,-3,5)):field.setValue(value)
        w._apply_gimbal()
        ty,_,payload=list(P.Parser().feed(self.link.frames[-1]))[0]
        self.assertEqual(ty,P.SIM_GIMBAL_CFG)
        np_values=struct.unpack('<4f',payload)
        self.assertAlmostEqual(np_values[0],1.25)
        self.assertAlmostEqual(np_values[3],.005,places=7)
        w._capture_rest()
        self.assertEqual(list(P.Parser().feed(self.link.frames[-1]))[0][0],P.SIM_REST)
        data=P.GimbalState(1000,0,(20,0,30),(0,0,30),(20,0,0),(20,20,0),
                           (0,0,0),(0,0,9.80665),(0,0,0),1,9.80665,0,0,0)
        w.on_gimbal(data);w.cube3d.animate()
        self.assertTrue(w.cube3d.has_gimbal)
        relative,planes=w.cube3d.relative_geometry()
        for value,expected in zip(relative,(20,0,0)):self.assertAlmostEqual(value,expected)
        for value,expected in zip(planes,(20,20,0)):self.assertAlmostEqual(value,expected,places=5)
        self.assertIn('+20.00',w.relative_labels[0].text())
        fields=w._gimbal_csv_fields()
        self.assertEqual(len(fields),21)
        self.assertEqual(fields[0],1000)

    def test_webcam_waiting_and_center_button(self):
        import time
        w=self.window
        camera=P.decode_camera(struct.pack('<BBBBB8f',2,5,0,0,2,60,40,0,0,0,0,0,-1)+b'Webcam desligada')
        w.on_camera(camera)
        self.assertTrue(w.center_button.isEnabled())
        self.assertFalse(w.webcam_button.isChecked())
        w.on_state(P.State(2000,1,0,12,0,0,0,0,0,0,0,0,22.2,0,-1))
        w.last_rx=time.monotonic();w.center_button.click();w.tick()
        self.assertFalse(w.track_pending)
        self.assertEqual(list(P.Parser().feed(self.link.frames[-1]))[0][0],P.SIM_CAMERA_CENTER)
        w.fix_target()
        self.assertFalse(w.center_pending)
        self.assertTrue(w.track_pending)


if __name__=='__main__':unittest.main()
