"""Independent coupled rigid-body / three-wheel host validation. No real motor.
J*w_dot = -Jw*wheel_dot - cross(w, J*w+Jw*wheel), inertias diagonal.
J includes locked rotors; torque=Jw*(wheel_dot+w_dot). A disabled rotor
has zero motor torque, not artificially fixed speed relative to the body.
The plant has no gimbal unloading or artificial reset of wheel velocities.
"""
import ctypes as C
import math
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
L = C.CDLL(str(ROOT / 'core' / ('libcubecore3.dll' if sys.platform=='win32' else 'libcubecore3.so')))
F3=C.c_float*3;F4=C.c_float*4
class Meas(C.Structure):
    _fields_=[(n,F3) for n in ('gyro','accel','mag','wheel_rps','wheel_torque')]+[('vbat_v',C.c_float),('mag_valid',C.c_uint8),('motor_valid_mask',C.c_uint8)]
class AxisOut(C.Structure):
    _fields_=[('enable',C.c_uint32)]+[(n,C.c_float) for n in ('wheel_cmd_rps','torque_max_nm','accel_limit_rps2')]
class Out(C.Structure):_fields_=[('axis',AxisOut*3)]
class Cmd(C.Structure):_fields_=[('type',C.c_uint32),('id',C.c_uint32),('a',C.c_float),('b',C.c_float)]
class Det(C.Structure):
    _fields_=[('found',C.c_uint8),('n_cand',C.c_uint8),('x',C.c_float),('y',C.c_float),('area',C.c_int)]+[(n,C.c_float) for n in ('fill','contrast','bg','threshold')]
class Vision(C.Structure):
    _fields_=[('time',C.c_uint32),('found',C.c_uint8),('accepted',C.c_uint8),('n',C.c_uint8),('pad',C.c_uint8)]+[(n,C.c_float) for n in ('x','y','ex','ey','contrast')]+[('area',C.c_uint32)]
L.cc3_init.argtypes=[C.c_void_p,C.c_void_p]
L.cc3_step_with_imu.argtypes=[C.c_void_p,C.c_uint64,C.POINTER(Meas),C.c_int,C.POINTER(Out)]
L.cc3_command_legacy.argtypes=[C.c_void_p,C.POINTER(Cmd)]
L.cc3_command_vector.argtypes=[C.c_void_p,C.c_uint32,C.POINTER(C.c_float)]
L.cc3_get_attitude.argtypes=[C.c_void_p,C.POINTER(C.c_float)]
L.cc3_on_detection.argtypes=[C.c_void_p,C.c_uint64,C.c_uint64,C.POINTER(Det),C.POINTER(Vision)]
L.cc3_get_tlm.argtypes=[C.c_void_p,C.c_void_p]
L.cc3_pack_tm.argtypes=[C.c_void_p,C.c_size_t,C.c_void_p]
L.cc3_pack_tm.restype=C.c_size_t

def mul(a,b):
    return np.r_[a[0]*b[0]-a[1:]@b[1:],a[0]*b[1:]+b[0]*a[1:]+np.cross(a[1:],b[1:])]
def euler(a):
    r,p,y=np.array(a)/2
    return mul(mul(np.array([np.cos(y),0,0,np.sin(y)]),np.array([np.cos(p),0,np.sin(p),0])),np.array([np.cos(r),np.sin(r),0,0]))
def rotate(q,v):return v+2*np.cross(q[1:],q[0]*v+np.cross(q[1:],v))
def inv(q):return q*np.array([1,-1,-1,-1])
def integrate(q,w,dt):
    n=np.linalg.norm(w);a=n*dt/2;d=np.r_[np.cos(a),w*(np.sin(a)/n if n>1e-12 else dt/2)]
    q=mul(q,d);return q/np.linalg.norm(q)
def angle(a,b):return math.degrees(2*math.acos(np.clip(abs(a@b),0,1)))

class Plant:
    def __init__(self,pose=(0,0,0)):
        self.state=C.create_string_buffer(L.cc3_sizeof_state())
        assert L.cc3_init(self.state,None)
        self.q=euler(np.radians(pose));self.w=np.zeros(3);self.wheel=np.zeros(3)
        self.J=np.array([.004,.005,.006]);self.Jw=np.array([.00004,.00005,.00006])
        self.dt=.005;self.k=0;self.out=Out();self.m=Meas();self.queue=[];self.peak=np.zeros(3)
        self.bias=np.array([.003,-.004,.002]);self.motor_mask=7;self.torque=np.zeros(3)
        for _ in range(330):self.step(physical=False)
    def command(self,mode):assert L.cc3_command_legacy(self.state,C.byref(Cmd(1,0,mode,0)))
    def vector(self,kind,v):assert L.cc3_command_vector(self.state,kind,F3(*v))
    def estimate(self):
        q=F4();L.cc3_get_attitude(self.state,q);return np.array(q)
    def tracking_state(self):
        raw=C.create_string_buffer(L.cc3_sizeof_tlm());payload=C.create_string_buffer(159)
        L.cc3_get_tlm(self.state,raw)
        assert L.cc3_pack_tm(payload,159,raw)==159
        return payload.raw[8] # version/status/mask (3), then trk at offset 5
    def step(self,physical=True,target=None,covered=False,imu=True):
        self.k+=1;t=self.k*5000
        self.m.gyro[:]=self.w+self.bias
        self.m.accel[:]=rotate(inv(self.q),np.array([0,0,9.80665]))
        self.m.mag[:]=rotate(inv(self.q),np.array([44.,0,0]))
        self.m.mag_valid=1;self.m.motor_valid_mask=self.motor_mask
        self.m.wheel_rps[:]=self.wheel;self.m.wheel_torque[:]=self.torque;self.m.vbat_v=22.2
        status=L.cc3_step_with_imu(self.state,t,C.byref(self.m),int(imu),C.byref(self.out))
        if target is not None and self.k%10==0 and not covered:
            ray=rotate(inv(self.q),target)
            if ray[0]>0:
                x=159.5+249*ray[1]/ray[0];y=119.5+249*ray[2]/ray[0]
                if 0<=x<320 and 0<=y<240:self.queue.append((self.k+16,t,x,y))
        while self.queue and self.queue[0][0]<=self.k:
            _,cap,x,y=self.queue.pop(0);d=Det(1,1,x,y,100,.8,180,220,50);v=Vision()
            L.cc3_on_detection(self.state,cap,t,C.byref(d),C.byref(v))
        if physical:
            # Velocity-error motor torque, limited by torque/acceleration settings.
            # Resolve body and relative rotor accelerations together, including
            # free rotors when enable=0. No artificial velocity resets.
            motor_torque=np.zeros(3)
            for i,o in enumerate(self.out.axis):
                if o.enable:
                    maximum=min(self.Jw[i]*o.accel_limit_rps2,o.torque_max_nm)
                    motor_torque[i]=np.clip(self.Jw[i]*(o.wheel_cmd_rps-self.wheel[i])/.06,-maximum,maximum)
            momentum=self.J*self.w+self.Jw*self.wheel
            accel=(-motor_torque-np.cross(self.w,momentum))/(self.J-self.Jw)
            wd=motor_torque/self.Jw-accel
            self.q=integrate(self.q,self.w+accel*self.dt/2,self.dt)
            self.w+=accel*self.dt;self.wheel+=wd*self.dt;self.torque=motor_torque
            self.peak=np.maximum(self.peak,np.abs(self.wheel))
        assert np.isfinite(self.q).all() and np.isfinite(self.wheel).all()
        return status
    def run(self,seconds,**kw):
        for _ in range(round(seconds/self.dt)):self.step(**kw)

class CoupledTests(unittest.TestCase):
    def test_layout(self):
        self.assertEqual(C.sizeof(Meas),L.cc3_sizeof_meas());self.assertEqual(C.sizeof(Out),L.cc3_sizeof_out())
    def test_mixed_attitude_references(self):
        for initial,ref in [((0,0,0),(20,-15,35)),((10,15,-20),(-25,20,-40)),((0,0,0),(30,70,100))]:
            p=Plant(initial);desired=mul(p.q,euler(np.radians(ref)))
            p.vector(4,np.radians(ref));p.command(4);p.run(18)
            error=angle(p.q,desired);print('3D attitude',initial,ref,'error_deg',round(error,4),'peak_rpm',np.round(p.peak*60/(2*np.pi)),flush=True)
            self.assertLess(error,1);self.assertLess(angle(p.q,p.estimate()),1)
            self.assertLess(max(p.peak),314.17)
    def test_track_both_bearings_latency_occlusion_aim(self):
        p=Plant();target=rotate(euler(np.radians([0,-15,25])),np.array([1.,0,0]))
        p.command(5);p.run(7,target=target)
        ray=rotate(inv(p.q),target);self.assertLess(math.degrees(math.acos(np.clip(ray[0],-1,1))),2)
        p.run(2,target=target,covered=True);p.run(2,target=target)
        p.vector(5,np.radians([10,5,-7]));p.run(8,target=target)
        ray=rotate(inv(p.q),target);bearings=np.degrees(np.arctan2(ray[1:],ray[0]))
        print('3D camera with 80 ms latency: horizontal/vertical',bearings,flush=True)
        self.assertLess(abs(bearings[0]+7),1);self.assertLess(abs(bearings[1]-5),1)
    def test_motor_failure_and_imu_recovery(self):
        p=Plant();p.vector(4,np.radians([10,5,20]));p.command(4);p.run(.3)
        p.motor_mask=5;p.step();self.assertTrue(all(not o.enable for o in p.out.axis))
        p.motor_mask=7;p.run(.1);p.step(imu=False);self.assertTrue(all(not o.enable for o in p.out.axis))
        self.assertEqual(p.step(),0);self.assertTrue(all(not o.enable for o in p.out.axis))
        p.run(4);self.assertLess(angle(p.q,euler(np.radians([10,5,20]))),2)
    def test_shove_with_all_three_axes(self):
        p=Plant();target=rotate(euler(np.radians([0,-10,20])),np.array([1.,0,0]))
        p.command(5);p.run(6,target=target)
        # External angular impulse; no reset of orientation or wheels.
        p.w+=np.array([.6,-.7,.9]);p.run(4,target=target)
        ray=rotate(inv(p.q),target);error=math.degrees(math.acos(np.clip(ray[0],-1,1)))
        print('After 3-axis angular impulse: pointing error_deg',round(error,4),flush=True)
        self.assertLess(error,2);self.assertLess(max(p.peak),314.17)
    def test_quaternion_propagation_at_high_pitch(self):
        p=Plant((20,80,30));p.command(1)
        for _ in range(1200):
            p.w=np.array([.4,-.2,.3]);p.step(physical=False)
            p.q=integrate(p.q,p.w,p.dt)
        self.assertLess(angle(p.q,p.estimate()),1)
    def test_search_after_long_camera_loss(self):
        p=Plant();target=rotate(euler(np.radians([0,-10,15])),np.array([1.,0,0]))
        p.command(5);p.run(6,target=target)
        self.assertEqual(p.tracking_state(),1)
        p.run(6.5,target=target,covered=True)
        self.assertEqual(p.tracking_state(),3)
        p.run(10,target=target)
        self.assertEqual(p.tracking_state(),1)
        ray=rotate(inv(p.q),target)
        self.assertLess(math.degrees(math.acos(np.clip(ray[0],-1,1))),2)
    def test_disabled_rotors_coast_and_momentum(self):
        p=Plant();p.command(1)
        p.w=np.array([.6,-.4,.8]);p.wheel=np.array([20.,-30.,40.])
        initial=rotate(p.q,p.J*p.w+p.Jw*p.wheel)
        for _ in range(400):
            spin=p.wheel+p.w
            p.step()
            self.assertTrue(all(not o.enable for o in p.out.axis))
            np.testing.assert_allclose(p.wheel+p.w,spin,atol=1e-10)
        final=rotate(p.q,p.J*p.w+p.Jw*p.wheel)
        self.assertLess(np.linalg.norm(final-initial)/np.linalg.norm(initial),.01)


if __name__=='__main__':unittest.main()
