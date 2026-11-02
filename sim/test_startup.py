"""Regressão de arranque: GUI antes do servidor, imagem, TRACK e CLI em CALIB.

Usa portas UDP locais livres; não liga a hardware.
"""
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'groundstation'), str(ROOT / 'sim')]
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5 import QtWidgets, QtGui
from gs_app import MainWindow
from gs_link import UdpLink
import gs_protocol as P
from sim_server import SimServer


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def server(port, log):
    return subprocess.Popen([sys.executable, '-u', str(ROOT / 'sim/sim_server.py'),
                             '--port', str(port)], stdout=log, stderr=subprocess.STDOUT)


def stop(process):
    if process.poll() is None:
        process.terminate()
    process.wait(timeout=5)


def main():
    app = QtWidgets.QApplication([])
    font = Path('C:/Windows/Fonts/segoeui.ttf')
    if font.exists():
        index = QtGui.QFontDatabase.addApplicationFont(str(font))
        families = QtGui.QFontDatabase.applicationFontFamilies(index)
        if families:
            app.setFont(QtGui.QFont(families[0],10))
    port = free_port()
    link = UdpLink(port=port)
    window = MainWindow(link, img_every=2, auto_track=True)
    window.show()
    app.processEvents()  # O contacto inicial ocorre antes do arranque do servidor.
    with open(ROOT / 'results/test_startup.log', 'w', encoding='utf-8') as log:
        proc = server(port, log)
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                app.processEvents()
                state = window.last_state
                if state and state.mode == 5 and abs(state.err) < 2 and abs(state.rate) < 1 and window.last_vis:
                    break
                time.sleep(0.01)
            state = window.last_state
            assert state and state.mode == 5, 'GUI sem telemetria TRACK'
            assert abs(state.err) < 2, state.err
            assert window.last_vis is not None, 'Imagem não recebida'
            window.grab().save(str(ROOT / 'results/gs_corrigida.png'))
            print('PASS: GUI antes do servidor, UDP, imagem e TRACK', flush=True)
            for angle in (20, -20):
                window.send(P.sim_shove, angle)
                deadline = time.monotonic() + 7
                disturbed = False
                motor_commanded = False
                recovered = False
                while time.monotonic() < deadline:
                    app.processEvents()
                    state = window.last_state
                    disturbed |= abs(state.err) > 5
                    motor_commanded |= abs(state.wheel_cmd_rpm) > 50
                    if disturbed and abs(state.err) < 2 and abs(state.rate) < 1:
                        recovered = True
                        break
                    time.sleep(0.01)
                assert disturbed and motor_commanded and recovered, (angle, state)
                print(f'PASS: empurrão {angle:+}°, comando de motor e retorno ao alvo', flush=True)
            window.shove_x.setValue(20); window.shove_y.setValue(15)
            custom = next(b for b in window.findChildren(QtWidgets.QPushButton) if b.text()=='Aplicar X + Y')
            custom.click()
            deadline=time.monotonic()+8
            disturbed=False; recovered=False; motor_y=False
            while time.monotonic()<deadline:
                app.processEvents()
                multi=window.last_sim3
                if multi:
                    disturbed |= abs(multi.errors[1])>5 and abs(multi.errors[2])>5
                    motor_y |= abs(multi.commands[1])>50
                    if disturbed and max(map(abs,multi.errors))<2 and max(map(abs,multi.rates))<1:
                        recovered=True;break
                time.sleep(.01)
            assert disturbed and recovered and motor_y
            assert window.last_gimbal is not None
            assert abs(window.last_gimbal.mass-1)<1e-6
            assert max(abs(v) for v in (window.last_gimbal.com_x,window.last_gimbal.com_y,window.last_gimbal.com_z))==0
            assert abs(window.last_gimbal.reference[2]+30)<1e-4
            assert len(window.tabs.tabText(1))>0
            assert not any('Alvo +' in b.text() or b.text() in ('WHEEL','RATE','HEADING','TRACK')
                           for b in window.findChildren(QtWidgets.QPushButton))
            window.grab().save(str(ROOT/'results/dashboard_v5.png'))
            window.grab().save(str(ROOT/'results/gimbal_v6.png'))
            window.tabs.setCurrentIndex(1);app.processEvents()
            window.grab().save(str(ROOT/'results/advanced_v5.png'))
            window.grab().save(str(ROOT/'results/gimbal_advanced_v6.png'))
            window.tabs.setCurrentIndex(0)
            print('PASS: botão personalizado X+Y, comando na roda Y e recuperação dos dois eixos',flush=True)
            link.send_frame(P.tc_stream, 20, 5, 0)
            # Descartar pacotes já em trânsito antes de medir a nova frequência.
            settling=time.monotonic()+.35
            while time.monotonic()<settling:
                link.poll()
                time.sleep(.01)
            start = time.monotonic()
            count = 0
            while time.monotonic() - start < 2:
                count += sum(k == 'vision' for k, _ in link.poll())
                time.sleep(0.01)
            assert 7 <= count <= 12, count
            print(f'PASS: vision_hz=5 ({count} medidas em 2 s)', flush=True)
        finally:
            window.close()
            stop(proc)
        port = free_port()
        proc = server(port, log)
        try:
            result = subprocess.run(
                [sys.executable, str(ROOT / 'groundstation/gs_cli.py'), '--port', str(port),
                 '--mode', 'track', '--seconds', '3'], capture_output=True, text=True, timeout=20)
            assert result.returncode == 0, result.stderr
            assert 'TRACK' in result.stdout, result.stdout
            print('PASS: CLI aguarda calibração e entra em TRACK', flush=True)
        finally:
            stop(proc)
    # Validação do ICD sem depender da interface gráfica.
    sim = SimServer(port=0)
    sent = []
    sim.send = lambda ty, payload=b'': sent.append((ty, payload))
    try:
        sim.handle(P.TC_PING, 9, b'x')
        assert sent[-1] == (P.TM_ACK, bytes([P.TC_PING, 9, 1]))
        sim.handle(P.TC_STREAM_CFG, 10, b'xx')
        assert sent[-1] == (P.TM_ACK, bytes([P.TC_STREAM_CFG, 10, 1]))
        sim.physics.run(2.1)
        sim.handle(P.SIM_GIMBAL_CFG,11,__import__('struct').pack('<4f',1.25,0,0,.005))
        assert sent[-1]==(P.TM_ACK,bytes([P.SIM_GIMBAL_CFG,11,0]))
        assert abs(sim.physics.gimbal.com_z-.005)<1e-8
        sim.handle(P.SIM_REST,12,b'')
        assert sent[-1]==(P.TM_ACK,bytes([P.SIM_REST,12,0]))
        assert sim.physics.reference_id==1
        sim.handle(P.SIM_GIMBAL_CFG,13,b'bad')
        assert sent[-1]==(P.TM_ACK,bytes([P.SIM_GIMBAL_CFG,13,1]))
        sim.handle(P.SIM_GIMBAL_CFG,14,__import__('struct').pack('<4f',-1,0,0,0))
        assert sent[-1]==(P.TM_ACK,bytes([P.SIM_GIMBAL_CFG,14,3]))
        print('PASS: ACK de comprimento inválido em PING/STREAM_CFG', flush=True)
        print('PASS: parâmetros de massa/CM, referência de repouso e validação dos comandos do gimbal',flush=True)
    finally:
        sim.sock.close()


if __name__ == '__main__':
    (ROOT / 'results').mkdir(exist_ok=True)
    main()
