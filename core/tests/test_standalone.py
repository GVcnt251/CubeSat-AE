"""Assert that the standalone DLL exports the 3D API, not the retired controller."""
import ctypes as C
import sys
from pathlib import Path

core = Path(__file__).resolve().parents[1]
library = C.CDLL(str(core / ('libcubecore3.dll' if sys.platform == 'win32' else 'libcubecore3.so')))
for name in ('cc3_init', 'cc3_config_default', 'cc3_step_with_imu', 'cc3_on_detection',
             'cc3_apply_tc', 'cc3_pack_tm', 'cc3_gain_valid', 'cc_frame_pack',
             'cc_detect', 'moteus_pack_velocity'):
    assert getattr(library, name), name
for name in ('cc_create', 'cc_init', 'cc_step', 'cc_step_with_imu', 'cc_on_image',
             'cc_on_mag', 'cc_config_default', 'cc_gain_valid'):
    try:
        getattr(library, name)
    except AttributeError:
        pass
    else:
        raise AssertionError('Old controller leaked into 3D library: ' + name)
library.cc3_gain_valid.argtypes = [C.c_uint32, C.c_float]
library.cc3_gain_valid.restype = C.c_int
assert library.cc3_gain_valid(2, 0) == 1 # Ki=0 remains supported
assert library.cc3_gain_valid(7, 0) == 0 # zero camera sigma is invalid
assert library.cc3_gain_valid(9, .5) == 0 # use_mag must be boolean
assert library.cc3_gain_valid(0, float('nan')) == 0
print('PASS: standalone 3D exports and gain validation; no single-axis controller symbols')
