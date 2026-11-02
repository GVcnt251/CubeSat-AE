"""Abre o cubo virtual e a estação de terra; fecha ambos ao sair da janela."""
import argparse
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', type=int, default=5000)
    ap.add_argument('--psi0', type=float, default=None)
    ap.add_argument('--webcam',action='store_true')
    ap.add_argument('--camera-index',type=int,default=0)
    args = ap.parse_args()
    if args.psi0 is None:args.psi0=0.0 if args.webcam else -30.0
    try:
        import numpy
        import PyQt5.QtWidgets
        import pyqtgraph
        sys.path.insert(0, str(ROOT / 'sim'))
        from sil import Core
        core = Core()  # Diagnóstico da DLL antes de abrir processos.
        del core
    except (ImportError, OSError, subprocess.SubprocessError) as exc:
        print(f'Não foi possível iniciar: {exc}')
        print('Instala as dependências com este mesmo Python:')
        print(f'"{sys.executable}" -m pip install -r "{ROOT / "requirements.txt"}"')
        print('Se o erro for da biblioteca C: python compilar_core.py')
        return 1
    # Não assumir controlo de um servidor que já ocupa a porta.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        if os.name == 'nt':
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        try:
            probe.bind(('127.0.0.1', args.port))
        except OSError:
            print(f'Porta {args.port} ocupada. Fecha o outro simulador ou usa --port 5001.')
            return 1
    env = dict(os.environ, PYTHONUTF8='1')
    flags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
    with open(ROOT / 'simulador.log', 'w', encoding='utf-8') as log:
        server_command=[sys.executable,'-u',str(ROOT/'sim/sim_server.py'),'--port',str(args.port),'--psi0',str(args.psi0)]
        if args.webcam:server_command+=['--webcam-only']
        server = subprocess.Popen(
            server_command, cwd=ROOT, env=env, stdout=log,
            stderr=subprocess.STDOUT, creationflags=flags)
        try:
            time.sleep(0.5)
            if server.poll() is not None:
                print('O simulador não arrancou. Consulta simulador.log.')
                return 1
            command=[sys.executable,str(ROOT/'groundstation/gs_app.py'),'--port',str(args.port),'--auto-track']
            if args.webcam:command+=['--webcam','--camera-index',str(args.camera_index)]
            return subprocess.call(command,cwd=ROOT,env=env)
        finally:
            if server.poll() is None:
                server.terminate()
            server.wait(timeout=5)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
