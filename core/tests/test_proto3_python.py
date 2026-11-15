"""Independent struct-based golden payload checks against C serialization."""
import ctypes as C
import struct
import unittest
from test_core3_sil import L, Plant
from proto3_reference import vector, gain, decode_state3
L.cc3_apply_tc.argtypes=[C.c_void_p,C.c_uint8,C.c_void_p,C.c_uint8]
L.cc3_get_tlm.argtypes=[C.c_void_p,C.c_void_p]
L.cc3_pack_tm.argtypes=[C.c_void_p,C.c_size_t,C.c_void_p]
L.cc_tc_to_cmd.argtypes=[C.c_uint8,C.c_void_p,C.c_uint8,C.c_void_p]
class WireTests(unittest.TestCase):
    def test_vectors_and_roundtrip(self):
        p=Plant();p.command(2)
        payload=vector(2,[600,-1200,1800]);self.assertEqual(len(payload),14)
        self.assertEqual(L.cc3_apply_tc(p.state,0x89,payload,len(payload)),0)
        p.step(physical=False)
        state=C.create_string_buffer(L.cc3_sizeof_tlm());buf=C.create_string_buffer(159)
        L.cc3_get_tlm(p.state,state);self.assertEqual(L.cc3_pack_tm(buf,159,state),159)
        d=decode_state3(buf.raw);self.assertEqual(d['motor_mask'],7);self.assertEqual(d['imu_status'],0)
        for i,expected in enumerate([600,-1200,1800]):self.assertAlmostEqual(d['axes'][i][10],expected,places=3)
        # Old decoder does not accidentally execute the proposed new command.
        cmd=C.create_string_buffer(16);self.assertEqual(L.cc_tc_to_cmd(0x89,payload,14,cmd),0)
    def test_bad_version_lengths_nonfinite(self):
        p=Plant()
        for payload,expected in [(b'\x02'+vector(4,[0,0,0])[1:],3),(vector(4,[0,0,0])[:-1],1),(struct.pack('<BB3f',1,4,0,float('inf'),0),3)]:
            self.assertEqual(L.cc3_apply_tc(p.state,0x89,payload,len(payload)),expected)
        payload=gain(2,0,4);self.assertEqual(L.cc3_apply_tc(p.state,0x8a,payload,7),0)
if __name__=='__main__':unittest.main()
