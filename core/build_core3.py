"""Build only the three-axis core. Output: libcubecore3.dll / libcubecore3.so.

Run: python core/build_core3.py --test
Requires host GCC; no network, no installations, no firmware/repository edits.
The distinct library name leaves an existing single-axis simulator DLL alone.
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

MODULES = ('cube_core3', 'target_detect', 'cc_proto', 'cc_proto3', 'moteus_proto')
TESTS = ('test_common', 'test_core3', 'test_core3_more')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--test', action='store_true', help='run C regressions and API checks')
    args = parser.parse_args()
    core = Path(__file__).resolve().parent
    compiler = os.environ.get('CC') or shutil.which('gcc')
    if not compiler:
        candidates = ('C:/msys64/ucrt64/bin/gcc.exe', 'C:/msys64/mingw64/bin/gcc.exe')
        compiler = next((p for p in candidates if Path(p).is_file()), None)
    if not compiler:
        parser.error('Host GCC not found. Configure MSYS2 GCC or set CC to its executable.')
    compiler = shutil.which(compiler) or compiler
    env = os.environ.copy()
    env['PATH'] = str(Path(compiler).resolve().parent) + os.pathsep + env.get('PATH', '')
    flags = ['-O2', '-std=c99', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-Iinclude']
    windows = os.name == 'nt'
    if not windows:
        flags.append('-fPIC')
    tail = ['-lm'] + (['-static-libgcc'] if windows else [])
    sources = ['src/' + name + '.c' for name in MODULES]
    suffix = '.dll' if windows else '.so'
    library = 'libcubecore3' + suffix
    temporary = 'libcubecore3_build' + suffix

    def run(command):
        subprocess.run(command, cwd=core, env=env, check=True)

    # Relative paths also avoid Windows compiler encoding issues in accented paths.
    run([compiler, *flags, '-shared', '-o', temporary, *sources, *tail])
    (core / temporary).replace(core / library)
    if args.test:
        for test in TESTS:
            output = test + ('.exe' if windows else '')
            run([compiler, *flags, '-o', output, 'tests/' + test + '.c', *sources, *tail])
            run([str(core / output)])
        run([sys.executable, 'tests/test_standalone.py'])
    print('Built standalone 3-axis library:', core / library)


if __name__ == '__main__':
    main()
