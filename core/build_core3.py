"""Build the optional 3D core and unchanged legacy core with host GCC.
Run from any directory: python core/build_core3.py --test
No pip installs, no network and no hardware. DLL must not be loaded elsewhere.
"""
from pathlib import Path
import os
import shutil
import subprocess
import sys

def main():
    core=Path(__file__).resolve().parent
    cc=os.environ.get('CC') or shutil.which('gcc')
    if not cc:
        cc=next((str(p) for p in (Path('C:/msys64/ucrt64/bin/gcc.exe'),Path('C:/msys64/mingw64/bin/gcc.exe')) if p.is_file()),None)
    if not cc:raise RuntimeError('Host GCC not found; install MSYS2 GCC or set CC.')
    env=os.environ.copy();env['PATH']=str(Path(cc).resolve().parent)+os.pathsep+env.get('PATH','')
    flags=['-O2','-std=c99','-Wall','-Wextra','-Wpedantic','-Werror','-Iinclude']
    sources=['src/'+n+'.c' for n in ('cube_core','target_detect','cc_proto','moteus_proto','cube_core3','cc_proto3')]
    tail=['-lm']+(['-static-libgcc'] if os.name=='nt' else [])
    suffix='.dll' if os.name=='nt' else '.so'
    if os.name!='nt':flags+=['-fPIC']
    # Relative arguments avoid Windows toolchain path-encoding issues.
    temporary='libcubecore_build'+suffix
    def run(args):subprocess.run(args,cwd=core,env=env,check=True)
    run([cc,*flags,'-shared','-o',temporary,*sources,*tail])
    (core/temporary).replace(core/('libcubecore'+suffix))
    if '--test' in sys.argv:
        for name in ('test_core','test_imu','test_review','test_core3','test_core3_more'):
            output=name+('.exe' if os.name=='nt' else '')
            run([cc,*flags,'-o',output,'tests/'+name+'.c',*sources,*tail])
            run([str(core/output)])
    print('Built legacy + 3D library',core/('libcubecore'+suffix))

if __name__=='__main__':main()
