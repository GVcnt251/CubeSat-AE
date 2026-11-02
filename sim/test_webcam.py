"""Vídeo de teste gerado localmente: não abre a webcam nem usa imagens pessoais."""
import ctypes as C
import math
import os
from pathlib import Path
import struct
import sys
import unittest
import numpy as np
sys.path[:0]=[str(Path(__file__).resolve().parent),str(Path(__file__).resolve().parents[1]/'groundstation')]
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from webcam import WebcamView,opencv
from three_axis import ThreeAxisSimulation
from sil import CmdT,CMD_MODE,DetectionT,VisionT,lib
import gs_protocol as P


class PaperVideo:
    """Um círculo em papel, movido em duas direções; fonte substituta de VideoCapture."""
    error=''
    def __init__(self):self.closed=False;self.stale=False;self.calls=0
    @staticmethod
    def angles(now):
        t=max(0,now-3)
        return math.radians(8)*math.sin(.45*t),math.radians(5)*math.sin(.4*t)
    def snapshot(self,now):
        cv=opencv();self.calls+=1
        yaw,pitch=self.angles(now)
        f=320/math.tan(math.radians(30))
        image=np.full((480,640,3),150,np.uint8)
        cv.rectangle(image,(60,30),(580,450),(235,235,235),-1)
        x=round(319.5+f*math.tan(yaw))
        y=round(239.5-f*math.tan(pitch)/math.cos(yaw))
        cv.circle(image,(x,y),16,(15,15,15),-1,lineType=cv.LINE_AA)
        return image,int(now*20+1e-6),.6 if self.stale else 0.
    def close(self):self.closed=True


class WebcamTests(unittest.TestCase):
    def test_real_pixel_detection_and_virtual_projection(self):
        capture=PaperVideo();source=WebcamView(capture=capture)
        state,packet=source.acquire(5)
        self.assertEqual(state,'fresh');self.assertTrue(source.found)
        target_yaw,target_pitch=source.target
        yaw,pitch=source.observe([0,target_pitch,target_yaw],packet[1])
        self.assertTrue(yaw.found and pitch.found and source.inside)
        self.assertAlmostEqual(yaw.x,159.5,delta=.02)
        self.assertAlmostEqual(yaw.y,119.5,delta=.02)
        self.assertAlmostEqual(pitch.x,159.5,delta=.02)
        self.assertGreater(source.coverage,.75)
        self.assertEqual(source.acquire(5.01)[0],'duplicate')
        capture.stale=True
        self.assertEqual(source.acquire(6)[0],'missing')
        self.assertFalse(source.found)
        source.close();self.assertTrue(capture.closed)

    def test_paper_motion_closes_the_loop_through_motors(self):
        capture=PaperVideo();source=WebcamView(capture=capture)
        model=ThreeAxisSimulation(psi0=0);model.set_camera_source(source)
        model.run(1.7);model.command(CmdT(CMD_MODE,0,5,0))
        errors=[];commands=[]
        for t in np.arange(1.75,14,.05):
            model.run(float(t))
            yaw,pitch=capture.angles(float(t))
            if t>5:errors.append(np.degrees([model.plants['yaw'].psi-yaw,model.plants['pitch'].psi-pitch]))
            commands.append([model.cores[a].tlm().wheel_cmd_rps for a in ('yaw','pitch')])
        self.assertLess(np.max(np.abs(errors)),2.5)
        self.assertGreater(np.max(np.abs(commands)),3)
        self.assertGreater(np.max(np.abs(np.asarray(commands)[:,1])),2)
        capture.stale=True;model.run(14.6)
        self.assertGreater(model.cores['yaw'].tlm().cam_age_s,.35)
        self.assertFalse(source.found)
        model.set_camera_source(None)
        self.assertTrue(capture.closed)
        self.assertTrue(math.isfinite(model.sim.target_fn(15)))
        print(f'PASS: movimento do papel seguido; maior erro medido {np.max(np.abs(errors)):.2f}°',flush=True)

    def test_video_fragments_and_qt_display(self):
        from PyQt5 import QtWidgets
        from gs_app import MainWindow
        from test_dashboard import Link
        cv=opencv();image=PaperVideo().snapshot(5)[0]
        image=cv.resize(cv.cvtColor(image,cv.COLOR_BGR2GRAY),(320,240))
        ok,jpeg=cv.imencode('.jpg',image);self.assertTrue(ok)
        data=jpeg.tobytes();count=(len(data)+229)//230;assembler=P.VideoAssembler();result=None
        for i in reversed(range(count)):
            packet=struct.pack('<HBHH',4,0,i,count)+data[i*230:(i+1)*230]
            result=assembler.feed(packet) or result
        self.assertEqual(result,(0,data))
        self.assertIsNone(assembler.feed(b'x'))
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        window=MainWindow(Link())
        window.on_video(0,data);window.on_video(1,data)
        self.assertEqual(window.img_item.image.shape,(320,240,3))
        self.assertLess(window.raw_item.image.min(),30)
        window._request_camera(True)
        ty,_,payload=list(P.Parser().feed(window.link.frames[-1]))[0]
        self.assertEqual(ty,P.SIM_CAMERA)
        self.assertEqual(len(payload),10)
        window.close()

    def test_blocked_camera_without_opening_hardware(self):
        from unittest.mock import patch
        from webcam import Capture
        class ClosedDevice:
            def isOpened(self):return False
            def release(self):self.released=True
        class Backend:
            CAP_DSHOW=700;CAP_ANY=0
            def VideoCapture(self,*args):return ClosedDevice()
        with patch('webcam.opencv',return_value=Backend()):
            camera=Capture(0);camera.thread.join(timeout=2)
        self.assertIsNone(camera.snapshot())
        self.assertIn('abrir',camera.error)
        view=WebcamView(capture=camera)
        self.assertEqual(view.acquire(0)[0],'missing')
        self.assertEqual(view.last_status,4)
        view.close()

    def test_stopping_the_paper_stabilizes_at_new_direction(self):
        class HeldPaper(PaperVideo):
            @staticmethod
            def angles(now):
                return (math.radians(10),math.radians(-6)) if now>=3 else (0.,0.)
        source=WebcamView(capture=HeldPaper());model=ThreeAxisSimulation(psi0=0)
        model.set_camera_source(source);model.run(1.7);model.command(CmdT(CMD_MODE,0,5,0));model.run(10)
        error=np.degrees([model.plants['yaw'].psi,model.plants['pitch'].psi])-[10,-6]
        self.assertLess(np.max(np.abs(error)),.3)
        self.assertTrue(model.center_on_webcam())
        model.run(16)
        self.assertTrue(model.centering)
        self.assertLess(np.max(np.abs(np.degrees(model.physical_angles()))),.6)
        model.command(CmdT(CMD_MODE,0,5,0));model.run(22)
        self.assertFalse(model.centering)
        error=np.degrees([model.plants['yaw'].psi,model.plants['pitch'].psi])-[10,-6]
        self.assertLess(np.max(np.abs(error)),.3)
        model.set_camera_source(None)

    def test_webcam_only_never_creates_a_synthetic_point(self):
        frames=[]
        model=ThreeAxisSimulation(psi0=0,webcam_only=True,frame_hook=lambda t,img,vis,proc:frames.append(vis.found))
        self.assertFalse(model.center_on_webcam())
        model.run(1.7);model.command(CmdT(CMD_MODE,0,5,0));model.run(8)
        self.assertTrue(model.pending_webcam_track)
        self.assertTrue(math.isnan(model.sim.target_fn(8)))
        self.assertEqual(max(frames),0)
        self.assertLess(max(abs(v) for v in np.degrees(model.physical_angles())),.5)
        model.set_camera_source(None);model.run(8.1)
        self.assertTrue(math.isnan(model.sim.target_fn(8.1)))

    def test_colour_is_preserved_and_saturated_dark_objects_are_rejected(self):
        cv=opencv()
        class ColourPaper(PaperVideo):
            def snapshot(self,now):
                image=np.full((480,640,3),235,np.uint8)
                cv.circle(image,(320,240),18,(45,45,45),-1)
                cv.circle(image,(130,200),20,(255,0,0),-1)
                cv.rectangle(image,(420,50),(500,110),(0,0,255),-1)
                return image,int(now*20),0
        source=WebcamView(capture=ColourPaper())
        _,packet=source.acquire(1)
        self.assertEqual(source.raw.shape,(240,320,3))
        self.assertTrue(source.found)
        self.assertAlmostEqual(source.source_xy[0],160,delta=1)
        self.assertAlmostEqual(source.source_xy[1],120,delta=1)
        self.assertGreater(int(source.raw[40,230,2]),200)
        self.assertLess(int(source.raw[40,230,0]),50)
        source.observe([0,0,0],packet[1])
        self.assertEqual(source.virtual.shape,(240,320,3))
        source.close()


if __name__=='__main__':unittest.main()
