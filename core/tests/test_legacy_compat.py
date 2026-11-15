"""Build the firmware's scalar core sources, without linking cube_core3.c.

Run from the repository root: python core/tests/test_legacy_compat.py
Also checks that headers for both APIs coexist in either inclusion order.
"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

CORE = Path(__file__).resolve().parents[1]
compiler = os.environ.get('CC') or shutil.which('gcc')
if not compiler:
    compiler = next((p for p in ('C:/msys64/ucrt64/bin/gcc.exe',
                                 'C:/msys64/mingw64/bin/gcc.exe')
                     if Path(p).is_file()), None)
if not compiler:
    raise SystemExit('Host GCC not found. Set CC to the compiler executable.')
compiler = shutil.which(compiler) or compiler
env = os.environ.copy()
env['PATH'] = str(Path(compiler).resolve().parent) + os.pathsep + env.get('PATH', '')
flags = ['-O2', '-std=c99', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-Iinclude']
sources = ['src/' + name + '.c' for name in
           ('cube_core', 'target_detect', 'cc_proto', 'moteus_proto')]
tail = ['-lm'] + (['-static-libgcc'] if os.name == 'nt' else [])


def run(command, **kwargs):
    subprocess.run(command, cwd=CORE, env=env, check=True, **kwargs)


with tempfile.TemporaryDirectory(prefix='cube_legacy_') as directory:
    for test in ('test_core', 'test_imu', 'test_review'):
        output = str(Path(directory) / (test + ('.exe' if os.name == 'nt' else '')))
        run([compiler, *flags, '-o', output, 'tests/' + test + '.c', *sources, *tail])
        run([output])
    for headers in (('cube_core.h', 'cube_core3.h', 'cc_proto3.h'),
                    ('cc_proto3.h', 'cube_core3.h', 'cube_core.h')):
        source = ''.join('#include "' + h + '"\n' for h in headers)
        source += 'int main(void) { cc_t a; cc3_t b; (void)a; (void)b; return 0; }\n'
        run([compiler, *flags, '-x', 'c', '-fsyntax-only', '-'], input=source, text=True)
print('PASS: scalar firmware source set links independently; both header orders compile')
