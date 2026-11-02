"""Percurso de vídeo/controlo pela rede local. Usa vídeo gerado, nunca uma webcam real."""
import os
from pathlib import Path
import sys
import threading
import time
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'sim'),str(ROOT/'groundstation')]
from PyQt5 import QtWidgets
from webcam import WebcamView
from test_webcam import PaperVideo
from sim_server import SimServer
from gs_app import MainWindow
from gs_link import UdpLink


def main():
    app=QtWidgets.QApplication([])
    source=WebcamView(capture=PaperVideo())
    server=SimServer(port=0,psi0=0,webcam_only=True)
    port=server.sock.getsockname()[1]
    server.physics.set_camera_source(source)
    stop=threading.Event()
    worker=threading.Thread(target=server.serve,args=(stop,),daemon=True);worker.start()
    window=MainWindow(UdpLink(port=port),auto_track=True,img_every=2)
    window.show()
    try:
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            app.processEvents()
            if (window.last_state and window.last_state.t_ms>6500 and window.last_camera
                and window.last_camera.mode and window.last_camera.found
                and window.last_state.mode==5 and window.last_state.cam_age<.4
                and window.img_item.image.shape==(320,240,3)
                and window.raw_item.image.min()<50):break
            time.sleep(.005)
        assert window.last_camera and window.last_camera.mode==1
        assert window.last_camera.found
        assert window.last_state.t_ms>6500 and window.last_state.mode==5
        assert window.img_item.image.shape==(320,240,3)
        assert window.raw_item.image.min()<50
        assert window.last_state.cam_age<.4
        assert abs(window.last_gimbal.attitude[2])>1
        print('PASS: vídeo completo e virtual por UDP/JPEG, deteção e resposta do CubeSat aos movimentos do papel',flush=True)
        window.center_button.click()
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            app.processEvents()
            if (window.last_camera.centering and window.last_state.mode==4 and
                max(map(abs,window.last_gimbal.attitude))<1):break
            time.sleep(.005)
        assert window.last_camera.centering and window.last_state.mode==4
        assert max(map(abs,window.last_gimbal.attitude))<1
        assert not window.track_pending
        window.fix_target()
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            app.processEvents()
            if not window.last_camera.centering and window.last_state.mode==5:break
            time.sleep(.005)
        assert not window.last_camera.centering and window.last_state.mode==5
        print('PASS: botão de centrar, manutenção do centro e retoma de FIXAR ALVO pela interface',flush=True)
    finally:
        window.close();stop.set();worker.join(timeout=5)
        assert not worker.is_alive()
        assert source.capture.closed


if __name__=='__main__':main()
