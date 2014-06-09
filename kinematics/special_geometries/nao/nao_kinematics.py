'''   Header
@file:          NAO_kinematics.py
@brief:    	    Contains specific functions that define all geometric and kinematic parameters for nao robot

@author:        Nima Ramezani Taghiabadi
                PhD Researcher
                Faculty of Engineering and Information Technology
                University of Technology Sydney (UTS)
                Broadway, Ultimo, NSW 2007, Australia
                Room No.: CB10.03.512
                Phone:    02 9514 4621
                Mobile:   04 5027 4611
                Email(1): Nima.RamezaniTaghiabadi@student.uts.edu.au 
                Email(2): nima.ramezani@gmail.com
                Email(3): nima_ramezani@yahoo.com
                Email(4): ramezanitn@alum.sharif.edu
@version:	    3.0
Last Revision:  10 Jun 2014

Changes from previous version:
added functions for FK of all segment poses

'''
import packages.nima.mathematics.general as gen

import numpy, math, copy
import packages.nima.mathematics.general as genlib
import packages.nima.mathematics.trigonometry as triglib
import packages.nima.mathematics.rotation as rotlib
import packages.nima.mathematics.vectors_and_matrices as vecmatlib

drc     = math.pi/180.00

default_masses = {"torso":1.2171, "neck":0.05, "shoulder": 0.07, "head": 0.35, "upperarm": 0.15, "elbow":0.035, "lowerarm":0.2, "hip1": 0.09, "hip2": 0.125, "thigh": 0.275, "shank":0.225, "ankle":0.125, "foot":0.2}
default_dims = {"h19":90.0, "b11":98.0,"h11":75.0,"a2":120.0,"d2":5.0,"h2":40.0,"e2":10.0,"a3":100.0,"h3":45.0,"e3":10.0,"a5":50.0,"h5":40.0,"d5":30.0,"e5":30.0,
                "e14" : 50.0, "e12":20.0,"b12":10.0, "d12":9.0, "a12":90.0, "h20":5.0, "a19":60.0, "e0":10.0, "h0":115.0, "b0":55.0}

class NAO():
    '''
    '''
    def __init__(self, mass = default_masses, dim = default_dims):
        '''
        The defaults are extracted from NAO model ... :
        link
        '''

        self.mass = mass
        self.dim  = dim

        self.q = numpy.zeros(21) # Now, we have considered only the leg joints, we should add arm and head joints as well
        self.c = numpy.cos(self.q)         
        self.s = numpy.sin(self.q)         

    def set_config(self, q):
        self.q = copy.copy(q)

        self.c = numpy.cos(self.q)         
        self.s = numpy.sin(self.q)  

        self.c6  = math.cos(self.q[6] - math.pi/4)
        self.s6  = math.sin(self.q[6] - math.pi/4)
        self.c1 = math.cos(self.q[1] - math.pi/4)
        self.s1 = math.sin(self.q[1] - math.pi/4)



    def update(self):
        k       = math.cos(math.pi/4)

        (c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10) = (self.c[0], self.c[1],self.c[2],self.c[3],self.c[4],self.c[5], self.c[6], self.c[7],self.c[8],self.c[9],self.c[10]) 
        (c11, c12, c13, c14, c15, c16, c17, c18, c19, c20) = (self.c[11], self.c[12],self.c[13],self.c[14],self.c[15],self.c[16], self.c[17], self.c[18],self.c[19],self.c[20])
        (s0, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10) = (self.s[0], self.s[1],self.s[2],self.s[3],self.s[4],self.s[5], self.s[6], self.s[7],self.s[8],self.s[9],self.s[10]) 
        (s11, s12, s13, s14, s15, s16, s17, s18, s19, s20) = (self.s[11], self.s[12],self.s[13],self.s[14],self.s[15],self.s[16], self.s[17], self.s[18],self.s[19],self.s[20])

        self.frx = c0*s2 - self.c1*c2*s0
        self.fry = k*(c2*(c0*self.c1 + self.s1) + s0*s2)
        self.frz = k*(-c0*self.c1*c2 + self.s1*c2- s0*s2)

        self.flx = c0*s7 - self.c6*c7*s0
        self.fly = k*(c7*(-c0*self.c6 - self.s6) - s0*s7)
        self.flz = k*(-c0*self.c6*c7 + self.s6*c7- s0*s7)

        self.grx = c3*(c0*s2 - self.c1*c2*s0) + s3*(c0*c2 + self.c1*s0*s2)
        self.gry = k*(c3*(c0*self.c1*c2 + self.s1*c2 + s0*s2) + s3*(c2*s0 -c0*self.c1*s2 - self.s1*s2))
        self.grz = k*(-c0*self.c1*c2*c3 + self.s1*c2*c3- s0*s2*c3 - c2*s0*s3 + c0*self.c1*s2*s3 - self.s1*s2*s3)

        self.glx = c8*(c0*s7 - self.c6*c7*s0) + s8*(c0*c7 + self.c6*s0*s7)
        self.gly = k*(c8*(-c0*self.c6*c7 - self.s6*c7 - s0*s7) - s8*(c7*s0 -c0*self.c6*s7 - self.s6*s7))
        self.glz = k*(-c0*self.c6*c7*c8 + self.s6*c7*c8- s0*s7*c8 - c7*s0*s8 + c0*self.c6*s7*s8 - self.s6*s7*s8)

        self.prx = c11*c12
        self.pry = s12
        self.prz = s11*c12

        self.plx = c15*c16
        self.ply = s16
        self.plz = s15*c16
        
        self.rrx = c11*s12
        self.rry = - c12
        self.rrz = s11*s12
        
        self.rlx = - c15*s16
        self.rly =   c16
        self.rlz = - s15*s16

        self.srx = - s11
        self.sry = 0    
        self.srz = c11

        self.slx = - s15
        self.sly = 0    
        self.slz = c15
        
        self.qrx = -s14*(s11*s13 + c11*c13*s12) + c11*c12*c14
        self.qry =  c14*s12 + c12*c13*s14
        self.qrz = -s14*(-c11*s13 + c13*s11*s12) + c12*c14*s11

        self.qlx = -s18*(s15*s17 + c15*c17*s16) + c15*c16*c18
        self.qly =  c18*s16 + c16*c17*s18
        self.qlz = -s18*(-c15*s17 + c17*s15*s16) + c16*c18*s15

        self.hx = - c19*s20
        self.hy = - s19*s20
        self.hz =   c20

        '''
        self.ur  = c1*(c0 -1) + s1*(c0 + 1)
        self.upr = c1*(c0 -1) - s1*(c0 + 1)
        self.vr  = c1*(c0 + 1) - s1*(c0 -1)
        self.vpr = c1*(c0 + 1) + s1*(c0 -1)

        self.ar = -k*c2*s0 + 0.5*s2*self.ur
        self.apr = -k*c2*s0 + 0.5*s2*self.vpr
        self.br = 0.5*c2*self.ur  + k*s0*s2
        self.bpr = 0.5*c2*self.vpr + k*s0*s2

        self.Ar  = c3*self.br  - s3*self.ar
        self.Apr = c3*self.bpr - s3*self.apr
        self.Br  = c3*self.ar  + s3*self.br
        self.Bpr = c3*self.apr + s3*self.bpr

        self.fr =   c2*c0 + s2*k*s0*(c1 + s1)  
        self.gr = - s2*c0 + c2*k*s0*(c1 + s1) 

        '''
        S1 = c1 + s1
        C1 = c1 - s1

        u1 = c0*S1 - C1
        v1  = c0*C1 + S1
        p1 = c0*C1 - S1
        q1 = c0*S1 + C1

        u2 = - k*c2*s0 + 0.5*s2*u1
        v2 =   0.5*c2*u1  + k*s0*s2
        p2 = - k*c2*s0 + 0.5*s2*q1
        q2 =   0.5*c2*q1 + k*s0*s2
        f2 =   c2*c0 + s2*k*s0*S1
        g2 = - s2*c0 + c2*k*s0*S1

        f3 = c3*g2 - s3*f2
        g3 = c3*f2 + s3*g2
        f4 = c4*f3 - s4*g3

        u3 = c3*v2  - s3*u2
        v3 = c3*u2  + s3*v2
        u4 = c4*u3  - s4*v3

        p3 = c3*q2 - s3*p2
        q3 = c3*p2 + s3*q2
        p4 = c4*p3 - s4*q3

        self.sxr =   c4*v3 + s4*u3
        self.syr =   c4*g3 + s4*f3  
        self.szr =   c4*q3 + s4*p3

        self.axr =   c5*u4 + 0.5*s5*v1
        self.ayr =   c5*f4 + k*s5*s0*C1
        self.azr =   c5*p4 + 0.5*s5*p1

        self.nxr =   0.5*c5*v1  - s5*u4
        self.nyr =   c5*s0*k*C1 - s5*f4
        self.nzr =   0.5*c5*p1  - s5*p4


        S6 = c6 + s6
        C6 = c6 - s6

        u6 = c0*S6 - C6
        v6  = c0*C6 + S6
        p6 = c0*C6 - S6
        q6 = c0*S6 + C6

        u7 = - k*c7*s0 + 0.5*s7*u6
        v7 =   0.5*c7*u6  + k*s0*s7
        p7 = - k*c7*s0 + 0.5*s7*q6
        q7 =   0.5*c7*q6 + k*s0*s7
        f7 =   c7*c0 + s7*k*s0*S6
        g7 = - s7*c0 + c7*k*s0*S6

        f8 = c8*g7 - s8*f7
        g8 = c8*f7 + s8*g7
        f9 = c9*f8 - s9*g8

        u8 = c8*v7  - s8*u7
        v8 = c8*u7  + s8*v7
        u9 = c9*u8  - s9*v8

        p8 = c8*q7 - s8*p7
        q8 = c8*p7 + s8*q7
        p9 = c9*p8 - s9*q8

        self.sxl = - c9*v8 - s9*u8
        self.syl =   c9*g8 + s9*f8  
        self.szl =   c9*q8 + s9*p8

        self.axl = - c10*u9 - 0.5*s10*v6
        self.ayl =   c10*f9 + k*s10*s0*C6
        self.azl =   c10*p9 + 0.5*s10*p6

        self.nxl =   0.5*c10*v6  - s10*u9
        self.nyl = - c10*s0*k*C6 + s10*f9
        self.nzl = - 0.5*c10*p6  + s10*p9


    def torso_orientation_wrt_rfoot(self):
        (c0, c1, c2, c3, c4, c5) = (self.c[0], self.c[1],self.c[2],self.c[3],self.c[4],self.c[5]) 
        (s0, s1, s2, s3, s4, s5) = (self.s[0], self.s[1],self.s[2],self.s[3],self.s[4],self.s[5]) 
        R = numpy.zeros((3,3))
        '''
        R[0,0] =  0.5*c5*self.vr - s5*(c4*self.Ar - s4*self.Br)
        R[0,1] =  c5*s0*k*(c1 - s1) - s5*(c4*(c3*self.gr - s3*self.fr) - s4*(c3*self.fr + s3*self.gr))
        R[0,2] =  0.5*c5*self.upr - s5*(c4*self.Apr - s4*self.Bpr)
        R[1,0] =  c4*self.Br + s4*self.Ar
        R[1,1] =  c4*(c3*self.fr + s3*self.gr) + s4*(c3*self.gr - s3*self.fr)
        R[1,2] =  c4*self.Bpr + s4*self.Apr
        R[2,0] =  c5*(c4*self.Ar - s4*self.Br) + 0.5*s5*self.vr
        R[2,1] =  c5*(c4*(c3*self.gr - s3*self.fr) - s4*(c3*self.fr + s3*self.gr)) + k*s5*s0*(c1 - s1)
        R[2,2] =  c5*(c4*self.Apr - s4*self.Bpr) + 0.5*s5*self.upr
        '''

        R[0,0] =  self.nxr
        R[0,1] =  self.nyr
        R[0,2] =  self.nzr
        R[1,0] =  self.sxr
        R[1,1] =  self.syr
        R[1,2] =  self.szr
        R[2,0] =  self.axr
        R[2,1] =  self.ayr
        R[2,2] =  self.azr

        return R

    def torso_orientation_wrt_lfoot(self):
        R = numpy.zeros((3,3))

        R[0,0] =  self.nxl # nxl
        R[0,1] =  self.nyl # nyl
        R[0,2] =  self.nzl # nzl
        R[1,0] =  self.sxl  # sxl
        R[1,1] =  self.syl # syl
        R[1,2] =  self.szl # szl
        R[2,0] =  self.axl # axl
        R[2,1] =  self.ayl # ayl
        R[2,2] =  self.azl # azl

        return R

    def pos_rank_wrt_tor_in_torcs(self):
        '''
        returns the position of right ankle with respect to the torso midhip point in torso coordinate systm    
        '''
        (a2, a3, b0) = (self.dim['a2'], self.dim['a3'], self.dim['b0'])

        x = a2*self.frx + a3*self.grx
        y = a2*self.fry + a3*self.gry - b0
        z = a2*self.frz + a3*self.grz
    
        return numpy.array([x, y, z])

    def pos_lank_wrt_tor_in_torcs(self):
        '''
        returns the position of left ankle with respect to the torso midhip point in torso coordinate systm    
        '''
        (a2, a3, b0) = (self.dim['a2'], self.dim['a3'], self.dim['b0'])

        x = a2*self.flx + a3*self.glx
        y = a2*self.fly + a3*self.gly + b0
        z = a2*self.flz + a3*self.glz
    
        return numpy.array([x, y, z])

    def pos_rshankcom_wrt_tor_in_torcs(self):
        '''
        returns the position of right ankle center of mass with respect to the torso midhip point in torso coordinate systm
        '''
        (a2, a3, b0) = (self.dim['a2'], self.dim['h3'], self.dim['b0'])

        x = a2*self.frx + a3*self.grx
        y = a2*self.fry + a3*self.gry - b0
        z = a2*self.frz + a3*self.grz
    
        return numpy.array([x, y, z])

    def pos_lshankcom_wrt_tor_in_torcs(self):
        '''
        returns the position of left ankle center of mass with respect to the torso midhip point in torso coordinate systm
        '''
        (a2, a3, b0) = (self.dim['a2'], self.dim['h3'], self.dim['b0'])

        x = a2*self.flx + a3*self.glx
        y = a2*self.fly + a3*self.gly + b0
        z = a2*self.flz + a3*self.glz
    
        return numpy.array([x, y, z])

    def pos_rthighcom_wrt_tor_in_torcs(self):
        '''
        returns the position of right thigh center of mass with respect to the torso midhip point in torso coordinate systm
        '''
        (h2, b0) = (self.dim['h2'], self.dim['b0'])

        x = h2*self.frx
        y = h2*self.fry - b0
        z = h2*self.frz
    
        return numpy.array([x, y, z])

    def pos_lank_wrt_rank_in_torcs(self):
        '''
        returns the position of left ankle with respect to the right ankle in torso coordinate system
        '''
        (a2, a3, b0) = (self.dim['a2'], self.dim['a3'], self.dim['b0'])

        x = a2*(self.flx - self.frx) + a3*(self.glx - self.grx)
        y = a2*(self.fly - self.fry) + a3*(self.gly - self.gry) + 2*b0
        z = a2*(self.flz - self.frz) + a3*(self.glz - self.grz)
    
        return numpy.array([x, y, z])

    def pos_headcom_wrt_tor_in_torcs(self):
        '''
        returns the position of head center of mass with respect to the torso midhip in torso coordinate system
        '''
        (h0, h19, h20, a19, e0) = (self.dim['h0'], self.dim['h19'], self.dim['h20'], self.dim['a19'], self.dim['e0'])

        x =  h20*self.hx + e0
        y =  h20*self.hy
        z =  h20*self.hz + h0 + h19 + a19
        
        return numpy.array([x, y, z])

    def pos_rupperarmcom_wrt_tor_in_torcs(self):
        (e0, b12, e12, b11, h11, h0) = (self.dim['e0'], self.dim['b12'], self.dim['e12'], self.dim['b11'], self.dim['h11'], self.dim['h0'])

        x =  b12*self.rrx + e12*self.prx + e0
        y =  b12*self.rry + e12*self.pry - b11
        z =  b12*self.rrz + e12*self.prz + h0 + h11 
        
        return numpy.array([x, y, z])

    def pos_relbowcom_wrt_tor_in_torcs(self):
        (e0, d12, a12, b11, h11, h0) = (self.dim['e0'], self.dim['d12'], self.dim['a12'], self.dim['b11'], self.dim['h11'], self.dim['h0'])

        x =  a12*self.prx + d12*self.srx + e0
        y =  a12*self.pry - b11
        z =  a12*self.prz + d12*self.srz + h0 + h11 
        
        return numpy.array([x, y, z])

    def pos_lelbowcom_wrt_tor_in_torcs(self):
        (e0, d12, a12, b11, h11, h0) = (self.dim['e0'], self.dim['d12'], self.dim['a12'], self.dim['b11'], self.dim['h11'], self.dim['h0'])

        x =  a12*self.plx + d12*self.slx + e0
        y =  a12*self.ply + b11
        z =  a12*self.plz + d12*self.slz + h0 + h11 
        
        return numpy.array([x, y, z])

    def pos_lupperarmcom_wrt_tor_in_torcs(self):
        (e0, b12, e12, b11, h11, h0) = (self.dim['e0'], self.dim['b12'], self.dim['e12'], self.dim['b11'], self.dim['h11'], self.dim['h0'])
        
        x =  b12*self.rlx + e12*self.plx + e0
        y =  b12*self.rly + e12*self.ply + b11
        z =  b12*self.rlz + e12*self.plz + h0  + h11
        
        return numpy.array([x, y, z])

    def pos_rlowerarmcom_wrt_tor_in_torcs(self):
        (h0, e0,  b11, h11, a12, d12, e14) = ( self.dim['h0'], self.dim['e0'], self.dim['b11'], self.dim['h11'], self.dim['a12'], self.dim['d12'], self.dim['e14'])

        X =  a12*self.prx + e14*self.qrx + d12*self.srx + e0
        Y =  a12*self.pry + e14*self.qry - b11
        Z =  a12*self.prz + e14*self.qrz + d12*self.srz + h0 + h11

        return numpy.array([X, Y, Z])

    def pos_llowerarmcom_wrt_tor_in_torcs(self):
        (h0, e0,  b11, h11, a12, d12, e14) = ( self.dim['h0'], self.dim['e0'], self.dim['b11'], self.dim['h11'], self.dim['a12'], self.dim['d12'], self.dim['e14'])

        X =  a12*self.plx + e14*self.qlx + d12*self.slx + e0
        Y =  a12*self.ply + e14*self.qly + b11
        Z =  a12*self.plz + e14*self.qlz + d12*self.slz + h0 + h11

        return numpy.array([X, Y, Z])

    def pos_lank_wrt_rank_in_rfootcs(self):
        '''
        returns the position of left ankle with respect to the right ankle in left foot coordinate system
        '''
        p = numpy.dot(self.torso_orientation_wrt_rfoot(), self.pos_lank_wrt_rank_in_torcs())
            
        return p

    def pos_rshankcom_wrt_rank_in_rfootcs(self):
        '''
        returns the position of left ankle with respect to the right ankle in left foot coordinate system
        '''
        p = numpy.dot(self.torso_orientation_wrt_rfoot(), self.pos_lank_wrt_rank_in_torcs())
            
        return p


    def segment_position(self, seg_name):
        '''
        Returns the position of the given seg_name wrt the ankle of the support foot in support foot coordinate system 
        '''
        (b0, e0, h0, b11, h11, h19) = (self.dim['b0'],self.dim['e0'],self.dim['h0'],self.dim['b11'],self.dim['h11'],self.dim['h19'])

        if seg_name in ["r_hip1", "r_hip2", "r_hip"]:
            pS_T    = numpy.array([0.0, -b0, 0.0])
        elif seg_name in ["l_hip1", "l_hip2", "l_hip"]:
            pS_T    = numpy.array([0.0, b0, 0.0])
        elif seg_name == "r_thigh":
            pS_T    = self.pos_rthighcom_wrt_tor_in_torcs()
        elif seg_name == "l_thigh":
            pS_T    = self.pos_lthighcom_wrt_tor_in_torcs()
        elif seg_name == "r_shank":
            pS_T    = self.pos_rshankcom_wrt_tor_in_torcs()
        elif seg_name == "l_shank":
            pS_T    = self.pos_lshankcom_wrt_tor_in_torcs()
        elif seg_name == "r_ankle":
            pS_T    = self.pos_rank_wrt_tor_in_torcs()
        elif seg_name == "l_ankle":
            pS_T    = self.pos_lank_wrt_tor_in_torcs()
        elif seg_name == "r_shoulder":
            pS_T    = numpy.array([e0, -b11, h0 + h11])
        elif seg_name == "l_shoulder":
            pS_T    = numpy.array([e0, b11, h0 + h11])
        elif seg_name == "r_upperarm":
            pS_T    = self.pos_rupperarmcom_wrt_tor_in_torcs()
        elif seg_name == "l_upperarm":
            pS_T    = self.pos_lupperarmcom_wrt_tor_in_torcs()
        elif seg_name == "r_lowerarm":
            pS_T    = self.pos_rlowerarmcom_wrt_tor_in_torcs()
        elif seg_name == "l_lowerarm":
            pS_T    = self.pos_llowerarmcom_wrt_tor_in_torcs()
        elif seg_name == "neck":
            pS_T    = numpy.array([e0, 0.0, h0 + h19])
        if self.support == "right_foot":
            pSF_T   = self.pos_rank_wrt_tor_in_torcs()
            R_T_SF  = self.torso_orientation_wrt_rfoot()
        elif self.support == "left_foot":
            pSF_T   = self.pos_lank_wrt_tor_in_torcs()
            R_T_SF  = self.torso_orientation_wrt_lfoot()
        else : 
            assert False
        p_S_SF  = numpy.dot(R_T_SF, pS_T - p_SF_T)
        
        return p_S_SF 

    def pos_com_wrt_tor_in_torcs(self):

        m0   = self.mass['torso']
        m1   = self.mass['hip1'] + self.mass['hip2']
        m2   = self.mass['thigh']
        m3   = self.mass['shank']
        m4   = self.mass['ankle']
        m5   = self.mass['foot']

        m11  = self.mass['shoulder']
        m12  = self.mass['upperarm']
        m13  = self.mass['elbow']
        m14  = self.mass['lowerarm']

        m19  = self.mass['neck']
        m20  = self.mass['head']

        (frx,fry,frz, flx,fly,flz, grx,gry,grz, glx,gly,glz) = (self.frx,self.fry,self.frz, self.flx,self.fly,self.flz, self.grx,self.gry,self.grz, self.glx,self.gly,self.glz)
        (hx,hy,hz) = (self.hx,self.hy,self.hz) 

        (prx,pry,prz, plx,ply,plz, qrx,qry,qrz, qlx,qly,qlz) = (self.prx,self.pry,self.prz, self.plx,self.ply,self.plz, self.qrx,self.qry,self.qrz, self.qlx,self.qly,self.qlz)
        (rrx,rry,rrz, rlx,rly,rlz, srx,srz, slx,slz) = (self.rrx,self.rry,self.rrz, self.rlx,self.rly,self.rlz, self.srx,self.srz, self.slx,self.slz)
        
        (h19,b11,h11,a2,d2,h2)     = (self.dim['h19'],self.dim['b11'],self.dim['h11'],self.dim['a2'],self.dim['d2'],self.dim['h2'])
        (e2,a3,h3,e3,a5,h5,d5)     = (self.dim['e2'],self.dim['a3'],self.dim['h3'],self.dim['e3'],self.dim['a5'],self.dim['h5'],self.dim['d5'])
        (e5,e14,e2,a3,h3,e3,a5)    = (self.dim['e5'],self.dim['e14'],self.dim['e2'],self.dim['a3'],self.dim['h3'],self.dim['e3'],self.dim['a5'])
        (h5,d5,e5,e14,e12,b12,d12) = (self.dim['h5'],self.dim['d5'],self.dim['e5'],self.dim['e14'],self.dim['e12'],self.dim['b12'],self.dim['d12'])
        (a12,h20,a19,e0,h0,b0)     = (self.dim['a12'],self.dim['h20'],self.dim['a19'],self.dim['e0'],self.dim['h0'],self.dim['b0'])
        '''
        x0  =   e0                                      #(torso)
        x1  =   0                                       #(right hip)
        x2  =   h2*frx                                  #(right thigh)
        x3  =   a2*frx  + h3*grx                        #(right shank)
        x4  =   a2*frx  + a3*grx                        #(right ankle)    
        x5  =   a2*frx  + a3*grx  + e5                  #(right foot)    # This equality has a little error due to possible orientation of right foot
        x6  =   0                                       #(left hip)
        x7  =   h2*flx                                  #(left thigh)
        x8  =   a2*flx  + h3*glx                        #(left shank)
        x9  =   a2*flx  + a3*glx                        #(left ankle)    
        x10 =   a2*flx  + a3*glx  + e5                  #(left foot)    # This equality has a little error due to possible orientation of left foot
        x11 =   e0                                      #(right shoulder)   
        x12 =   b12*rrx + e12*prx + e0                  #(right upperarm)              
        x13 =   d12*srx + a12*prx + e0                  #(right elbow)   
        x14 =   d12*srx + a12*prx + e14*qrx + e0        #(right lowerarm)   
        x15 =   e0                                      #(left shoulder)
        x16 =   b12*rlx  + e12*plx + e0                 #(left upperarm)
        x17 =   d12*slx  + a12*plx + e0                 #(left elbow)
        x18 =   d12*slx  + a12*plx + e14*qlx + e0       #(left lowerarm)
        x19 =   e0                                      #(neck)
        x20 =   h20*hx  + e0                            #(head)

        y0  =   0
        y1  = - b0
        y2  =   h2*fry - b0
        y3  =   a2*fry + h3*gry - b0
        y4  =   a2*fry + a3*gry - b0
        y5  =   a2*fry + a3*gry - b0
        y6  =   b0
        y7  =   h2*fly + b0
        y8  =   a2*fly + h3*gly + b0
        y9  =   a2*fly + a3*gly + b0
        y10 =   a2*fly + a3*gly + b0
        y11 = - b11
        y12 =   e12*pry  + b12*rry  - b11
        y13 =   a12*pry  - b11
        y14 =   a12*pry  + e14*qry  - b11
        y15 =   b11
        y16 =   b12*rly  + e12*ply  + b11
        y17 =   a12*ply  + b11
        y18 =   a12*ply  + e14*qly + b11
        y19 =   0
        y20 =   h20*hy

        z0  =   h0
        z1  =   0
        z2  =   h2*frz
        z3  =   a2*frz + h3*grz
        z4  =   a2*frz + a3*grz
        z5  =   a2*frz + a3*grz - h5             # This equality has a little error due to possible orientation of right foot
        z6  =   0
        z7  =   h2*flz
        z8  =   a2*flz + h3*glz
        z9  =   a2*flz + a3*glz
        z10 =   a2*flz + a3*glz - h5             # This equality has a little error due to possible orientation of right foot
        z11 =   h0 + h11
        z12 =   e12*prz  + b12*rrz  + h0 + h11 
        z13 =   a12*prz  + d12*srz  + h0 + h11 
        z14 =   a12*prz  + d12*srz  + e14*qrz  + h0 + h11
        z15 =   h0 + h11
        z16 =   e12*plz  + b12*rlz  + h0 + h11
        z17 =   a12*plz  + d12*slz  + h0 + h11 
        z18 =   a12*plz  + d12*slz  + e14*qlz  + h0 + h11
        z19 =   h0 + h19
        z20 =   h20*hz  + h0 + h19 + a19

        '''

        M = m0 + m19 + m20 + 2*(m1 + m2 + m3 + m4 + m5 + m11 + m12 + m13 + m14)
    
        X0 = 2*m5*e5 + e0*(m0 + 2*m11 + 2*m12 + 2*m13 + 2*m14 + m19 + m20)
        Z0 = m0*h0 - 2*m5*h5 + 2*(m11+m12+m13+m14)*(h0 + h11) + (m19+m20)*(h0 + h19) + m20*a19

        F  = m2*h2 + (m3 + m4 + m5)*a2
        G  = m3*h3 + (m4 + m5)*a3
        P  = m12*e12 + (m14 + m13)*a12
        Q  = m14*e14
        R   = m12*b12
        S  = (m13 + m14)*d12
        H  = m20*h20

        X = X0 + F*(frx+flx) + G*(grx+glx) + P*(prx+plx) + Q*(qrx+qlx) + R*(rrx+rlx) + S*(srx+slx) + H*hx 

        Y = F*(fry+fly) + G*(gry+gly) + P*(pry+ply) + Q*(qry+qly) + R*(rry+rly) + H*hy 

        Z = Z0 + F*(frz+flz) + G*(grz+glz) + P*(prz+plz) + Q*(qrz+qlz) + R*(rrz+rlz) + S*(srz+slz) + H*hz

        return numpy.array([X, Y, Z])/M

    def pos_rftf_wrt_tor_in_torcs(self):

        (b0, a2, a3, a5, d5) = (self.dim['b0'], self.dim['a2'], self.dim['a3'], self.dim['a5'],self.dim['d5'])

        X = a2*self.frx  + a3*self.grx + d5*self.syr - a5*self.ayr
        Y = a2*self.fry  + a3*self.gry - d5*self.sxr + a5*self.axr - b0
        Z = a2*self.frz  + a3*self.grz + d5*self.szr - a5*self.azr

        return numpy.array([X, Y, Z])

    def pos_lftf_wrt_tor_in_torcs(self):

        (b0, a2, a3, a5, d5) = (self.dim['b0'], self.dim['a2'], self.dim['a3'], self.dim['a5'],self.dim['d5'])

        p0 = self.pos_lank_wrt_tor_in_torcs()

        X = a2*self.flx + a3*self.glx + d5*self.syl - a5*self.ayl
        Y = a2*self.fly + a3*self.gly - d5*self.sxl + a5*self.axl + b0
        Z = a2*self.flz + a3*self.glz + d5*self.szl - a5*self.azl

        return numpy.array([X, Y, Z])

    def pos_rftf_wrt_lftf_in_lfootcs(self):
        (a5, d5) = (self.dim['a5'],self.dim['d5'])
        

        p = self.pos_rftf_wrt_tor_in_torcs() - self.pos_lank_wrt_tor_in_torcs()

        X =    self.syl*p[0] - self.sxl*p[1] + self.szl*p[2] - d5
        Y =  - self.nyl*p[0] + self.nxl*p[1] - self.nzl*p[2]
        Z =  - self.axl*p[1] + self.ayl*p[0] + self.azl*p[2] + a5

        return numpy.array([X , Y , Z ])
        
    def pos_lftf_wrt_rftf_in_rfootcs(self):
        (a5, d5) = (self.dim['a5'],self.dim['d5'])
        p = self.pos_lftf_wrt_tor_in_torcs() - self.pos_rank_wrt_tor_in_torcs()

        X =    self.syr*p[0] - self.sxr*p[1] + self.szr*p[2] - d5
        Y =  - self.nyr*p[0] + self.nxr*p[1] - self.nzr*p[2]
        Z =  - self.axr*p[1] + self.ayr*p[0] + self.azr*p[2] + a5

        return numpy.array([X , Y , Z ])


