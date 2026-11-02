"""Geometria e carga gravítica de um cubo preso num gimbal com pivô fixo."""
from dataclasses import dataclass
import math
import numpy as np


def rotation(angles):
    r,p,y=angles
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]]) @ np.array(
        [[cp,0,sp],[0,1,0],[-sp,0,cp]]) @ np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])


def relative_angles(current, reference):
    relative=rotation(reference).T @ rotation(current)
    pitch=math.asin(float(np.clip(-relative[2,0],-1,1)))
    if abs(math.cos(pitch))>1e-7:
        roll=math.atan2(relative[2,1],relative[2,2])
        yaw=math.atan2(relative[1,0],relative[0,0])
    else:
        roll=0
        yaw=math.atan2(-relative[0,1],relative[1,1])
    # Ângulo entre planos não orientados: XY (normal Z), XZ (normal Y), YZ (normal X).
    planes=np.arccos(np.clip(np.abs(np.diag(relative)[[2,1,0]]),0,1))
    return np.array([roll,pitch,yaw]), planes


@dataclass
class GimbalParams:
    mass_kg: float = 1.0  # Valor de exemplo; substituir pela massa medida.
    g: float = 9.80665
    com_x: float = 0.0   # Metros, no corpo NED, desde o pivô (centro geométrico).
    com_y: float = 0.0
    com_z: float = 0.0   # +Z: para baixo quando o cubo está nivelado.

    def valid(self):
        return (all(math.isfinite(x) for x in (self.mass_kg,self.g,self.com_x,self.com_y,self.com_z))
                and .1<=self.mass_kg<=10 and 0<=self.g<=20
                and max(abs(self.com_x),abs(self.com_y),abs(self.com_z))<=.05000001)

    def loads(self, angles):
        body_g=rotation(angles).T @ np.array([0.,0.,self.g])
        torque=np.cross([self.com_x,self.com_y,self.com_z],self.mass_kg*body_g)
        roll,pitch,_=angles
        # Trabalho virtual: Q = E(q)^T tau, com omega_corpo = E(q) q_dot.
        sr,cr,sp,cp=math.sin(roll),math.cos(roll),math.sin(pitch),math.cos(pitch)
        e=np.array([[1,0,-sp],[0,cr,sr*cp],[0,-sr,cr*cp]])
        return e.T @ torque, body_g

    def potential(self, angles):
        position=rotation(angles) @ np.array([self.com_x,self.com_y,self.com_z])
        return -self.mass_kg*self.g*position[2]
