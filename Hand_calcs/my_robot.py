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
    
def jacobians(links, Mp, K,v0, g):
    JPl = {}
    Jol = {}
    JPm = {}
    Jom = {}
    R_dict = {}
    R_dict['0'] = sp.eye(3)
    pe_link = {}
    pe_motor = {}
    link_in = copy.deepcopy(links)
    n = len(link_in)  # number of links
    g_f = sp.zeros(3, 1)
    for i in range(n, 0, -1):
        # Update the 'a' parameter of the current link
        link_in[i-1].a = link_in[i-1].l

        bot_in = Robot(link_in[:i])
        R_l, pe_l = bot_in.fkine(v0, i)
        z = R_l[:, 2]
        z = sp.simplify(z)
        pe_l = sp.simplify(pe_l)
        R_dict[f'{i}'] = sp.Matrix(R_l)
        pe_link[f'{i}'] = pe_l
        JPl[f'{i}'] = sp.zeros(3, n)
        JPl[f'{i}'][:, :i] = pe_l.jacobian([link_in.th for link_in in link_in[:i]])
        JPl[f'{i}'] = JPl[f'{i}'].applyfunc(sp.trigsimp)
        Jol[f'{i}'] = sp.zeros(3, n)
        Jol[f'{i}'][:, :i] = sp.Matrix.hstack(*[z for _ in range(i)])

        link_in[i-1].a = Mp[i-1] # set the positions of motors instead of 0s
        bot_in = Robot(link_in[:i])
        R_m, pe_m = bot_in.fkine(v0, i)
        z = R_m[:, 2]
        z = sp.simplify(z)
        pe_m = sp.simplify(pe_m)
        pe_motor[f'{i}'] = pe_m
        JPm[f'{i}'] = sp.zeros(3, n)
        JPm[f'{i}'][:, :i] = pe_m.jacobian([link_in.th for link_in in link_in[:i]])
        JPm[f'{i}'] = JPm[f'{i}'].applyfunc(sp.trigsimp)
        Jom[f'{i}'] = sp.zeros(3, n)
        Jom[f'{i}'][:, :i] = sp.Matrix.hstack(*[z for _ in range(i)])
        Jom[f'{i}'][:, i-1] *= K[i-1]

    return JPl, Jol, JPm, Jom, R_dict, pe_link, pe_motor

# probably have to modify this function depending on number of joints works for 3 as of now
def gravity_torques(links, Mp, ml, mm, v0, g):
    R_dict = {}
    R_dict['0'] = sp.eye(3)
    pe_link = {}
    pe_motor = {}
    link_in = copy.deepcopy(links)
    n = len(link_in)  # number of links
    L = [sp.Symbol(f'l{i+1}') for i in range(n)]
    a = [sp.Symbol(f'a{i+1}') for i in range(n)] # position of link cg
    for i in range(n, 0, -1):
        link_in[i-1].a = a[i-1]

    g_f = sp.zeros(3, 1)
    for i in range(n, 0, -1):
        # Update the 'a' parameter of the current link
        link_in[i-1].a = L[i-1]

        bot_in = Robot(link_in[:i])
        R_l, pe_l = bot_in.fkine(v0, i)
        z = R_l[:, 2]
        z = sp.simplify(z)
        pe_l = sp.simplify(pe_l)
        R_dict[f'{i}'] = sp.Matrix(R_l)
        pe_link[f'{i}'] = pe_l

        tau_gl = ml[i-1] * pe_l.cross(sp.Matrix([0, 0, g]))

        link_in[i-1].a = Mp[i-1] # set the positions of motors instead of 0s
        bot_in = Robot(link_in[:i])
        R_m, pe_m = bot_in.fkine(v0, i)
        z = R_m[:, 2]
        z = sp.simplify(z)
        pe_m = sp.simplify(pe_m)
        pe_motor[f'{i}'] = pe_m

        tau_gm = mm[i-1]* pe_m.cross(sp.Matrix([0, 0, g]))
        g_f = g_f + tau_gm + tau_gl
    tau_g = {}
    tau_g['1'] = R_dict['0']*g_f
    tau_g['2'] = R_dict['1']*g_f.subs({a[0]:0, L[0]:0})
    tau_g['3'] = R_dict['2']*g_f.subs({a[0]:0, L[0]:0, a[1]:0, L[1]:0})
    G = sp.Matrix([tau_g[f'{i}'][2] for i in range(1, n+1)])
    G = G.subs({a[0]:links[0].a, L[0]:links[0].l, a[1]:links[1].a, L[1]:links[1].l, a[2]:links[2].a, L[2]:links[2].l})
    return tau_g, G

def calc_b(n, ml, JPl, Jol, R_dict, Il, mm, JPm, Jom, Im):
    B = sp.zeros(n, n)
    for i in range(n-1, -1, -1):
        B_inter = (ml[i] * JPl[f'{i+1}'].T * JPl[f'{i+1}'] + 
                   Jol[f'{i+1}'].T * R_dict[f'{i+1}'] * Il[f'{i+1}'] * R_dict[f'{i+1}'].T * Jol[f'{i+1}'] + 
                   mm[i] * JPm[f'{i+1}'].T * JPm[f'{i+1}'] + 
                   Jom[f'{i+1}'].T * R_dict[f'{i+1}'] * Im[f'{i+1}'] * R_dict[f'{i+1}'].T * Jom[f'{i+1}'])
        B += B_inter

    # Simplify the final result
    B = B.applyfunc(sp.trigsimp)
    
    return B

def calc_c(B,n):
    q = [sp.Symbol(f'q{i+1}') for i in range(n)] # joint positions
    dq = [sp.Symbol(f'dq{i+1}') for i in range(n)] # joint velocities
    c = sp.MutableDenseNDimArray.zeros(n, n, n)
    for i in range(n):
        for j in range(n):
            for k in range(n):
                c[i, j, k] = 1/2 * (sp.diff(B[i, j], q[k]) + sp.diff(B[i, k], q[j]) - sp.diff(B[j, k], q[i]))
    C = c[:,:,0]*dq[0] + c[:,:,1]*dq[1]
    return C

def forward_kine(links, n, v):
    bot = Robot(links)
    Rot, pe = bot.fkine(v, n)
    
    return Rot, pe