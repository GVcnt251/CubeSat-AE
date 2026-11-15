"""Reference codec for Person 5, DRAFT IDs, not an automatic GS deployment.
Frame with existing gs_protocol.pack_frame; this module only encodes payloads.
"""
import math
import struct
TC_SET_VECTOR3=0x89
TC_SET_GAIN3=0x8A
TM_STATE3=0x08
def vector(kind,values):
    """kind 2 WHEEL(rpm), 3 RATE(deg/s), 4 HEADING(deg), 5 AIM(deg)."""
    if kind not in (2,3,4,5) or len(values)!=3 or not all(math.isfinite(v) for v in values):raise ValueError('Invalid vector')
    return struct.pack('<BB3f',1,kind,*values)
def gain(axis,gain_id,value):
    if axis not in (0,1,2) or not 0<=gain_id<=10 or not math.isfinite(value):raise ValueError('Invalid gain')
    return struct.pack('<BBBf',1,axis,gain_id,value)
def decode_state3(payload):
    if len(payload)!=159 or payload[0]!=1:raise ValueError('Unsupported state3 payload')
    return {'imu_status':payload[1],'motor_mask':payload[2],
            'axes':[struct.unpack_from('<IBBH11f',payload,3+52*i) for i in range(3)]}
