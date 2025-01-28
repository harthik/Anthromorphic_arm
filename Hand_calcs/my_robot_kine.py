import numpy as np
import sympy as sp
import copy



class Quaternion:
    def __init__(self, a, b, c, d):
        self.a = a
        self.b = b
        self.c = c
        self.d = d

    def __add__(self, other):
        return Quaternion(self.a + other.a, self.b + other.b, self.c + other.c, self.d + other.d)

    def __mul__(self, other):
        a, b, c, d = self.a, self.b, self.c, self.d
        e, f, g, h = other.a, other.b, other.c, other.d
        return Quaternion(a*e - b*f - c*g - d*h, a*f + b*e + c*h - d*g, a*g - b*h + c*e + d*f, a*h + b*g - c*f + d*e)

    def conj(self):
        return Quaternion(self.a, -self.b, -self.c, -self.d)

    def transl(self, vector, t):
        vt = [vector[i] + t[i] for i in range(3)]
        v = Quaternion(0, *vt)
        vr = self * v * self.conj()
        return sp.Matrix([vr.b, vr.c, vr.d])

    def rotate(self, vector):
        v = Quaternion(0, *vector)
        vr = self * v * self.conj()
        return sp.Matrix([vr.b, vr.c, vr.d])
    def angax2quat(self,vector,theta):
        v = vector
        return Quaternion(sp.cos(theta/2),sp.sin(theta/2)*v[0],sp.sin(theta/2)*v[1],sp.sin(theta/2)*v[2])

class Link(object):
    def __init__(self,alpha,a,th,d,l):
        self.alpha = alpha
        self.a = a
        self.th = th
        self.d = d
        self.l = l
    def link_trans(self,v):
        alpha = self.alpha
        aa = self.a
        th = self.th
        dd = self.d
        a = np.array([aa,0,0])
        d = np.array([0,0,dd])
        q = Quaternion(0,0,0,0)
        qz = q.angax2quat([0,0,1],th)
        qx = q.angax2quat([1,0,0],alpha)
        rvi = qx.transl(v,a)
        rv = qz.transl(rvi,d)
        return rv
    def link_orient(self,v):
        alpha = self.alpha
        th = self.th
        q = Quaternion(0,0,0,0)
        qz = q.angax2quat([0,0,1],th)
        qx = q.angax2quat([1,0,0],alpha)
        rvi = qx.rotate(v)
        rv = qz.rotate(rvi)
        return rv

class Robot(object):
    def __init__(self,link):
        self.link = link
    def fkine(self,v,l):
        link = self.link
        l = l-1
        n = len(link)
        i = l
        vi = v
        vx = np.array([1,0,0])
        vy = np.array([0,1,0])
        vz = np.array([0,0,1])
        if l > n:
            print("select valid range")
        else:
            while i >= 0:
                vi = link[i].link_trans(vi)
                vx = link[i].link_orient(vx)
                vy = link[i].link_orient(vy)
                vz = link[i].link_orient(vz)
                i -= 1
        R = np.hstack((vx, vy, vz)) #rotation matrix from T0 to T6
        R = sp.Matrix(R) 
        return R,vi
    
def forward_kine(links, n, v):
    bot = Robot(links)
    Rot, pe = bot.fkine(v, n)
    
    return Rot, pe
