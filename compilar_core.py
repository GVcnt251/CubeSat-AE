"""Compila o núcleo C sem depender de make ou do terminal MSYS2."""
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parent


def build():
    cc = os.environ.get('CC') or shutil.which('gcc')
    if not cc and os.name == 'nt':
        for flavor in ('mingw64', 'ucrt64'):
            candidate = Path('C:/msys64') / flavor / 'bin/gcc.exe'
            if candidate.is_file():
                cc = str(candidate)
                break
    if not cc:
        raise RuntimeError('GCC não encontrado. Instala o GCC do MSYS2 MINGW64 ou define CC com o caminho para gcc.exe.')
    core = ROOT / 'core'
    output = core / ('libcubecore.dll' if os.name == 'nt' else 'libcubecore.so')
    temporary = output.with_name(output.stem + '_build' + output.suffix)
    env = os.environ.copy()
    env['PATH'] = str(Path(cc).resolve().parent) + os.pathsep + env.get('PATH', '')
    cmd = [cc, '-O2', '-std=c99', '-Wall', '-Wextra', '-Wno-unused-parameter',
           '-fPIC', '-I', str(core / 'include'), '-shared', '-o', str(temporary)]
    cmd += [str(core / 'src' / name) for name in
            ('target_detect.c', 'cube_core.c', 'cc_proto.c', 'moteus_proto.c')]
    cmd += ['-lm']
    if os.name == 'nt':
        cmd += ['-static-libgcc']
    try:
        subprocess.run(cmd, check=True, env=env)
        temporary.replace(output)
    finally:
        if temporary.exists():
            temporary.unlink()
    print(f'Núcleo compilado: {output}')


if __name__ == '__main__':
    try:
        build()
    except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
        print(f'Falha na compilação: {exc}', file=sys.stderr)
        sys.exit(1)
