'''   Header
@file:          NAO_dynamics.py
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
@version:	    2.0
Last Revision:  06 June 2014

Changes from previous version:

added NAO_Symbolic class to see symbolic results of calculations

'''

import numpy as np
import math
from sympy import Symbol

import packages.nima.mathematics.vectors_and_matrices as vm
import packages.nima.mathematics.general as gen
import packages.nima.mathematics.rotation as rot

drc     = math.pi/180.00

class Segment(object):
    def __init__(self, name = "Unnamed", parent = "", mass = 1.0, COM = np.zeros(3), n_child = 1, JC = np.zeros(3), rot_axis = [0,0,1], q = 0.0, ql = - math.pi, qh = math.pi, symbolic = False, S = Symbol('s'), C = Symbol('c')):
        '''
        JC : A matrix of 3 rows and n columns where n is the number of children. Each column specifies the local coordinates of the Joint Center of each child segment.
        COM: Local coordinates of the Center of Mass
        '''
        self.name     = name
        self.parent   = parent
        self.mass     = mass
        self.n_child  = n_child    
        self.COM      = COM
        self.JC       = JC
        self.rot_axis = rot_axis
        self.ql       = ql
        self.qh       = qh

        self.symbolic = symbolic
        self.C        = C
        self.S        = S

        self.set_q(q)
        
        self.pos_origin = np.zeros(3)    
        self.pos_com    = COM
        self.pos_jc     = JC
    
        self.ori        = np.eye(3)
        self.updated    = False

    def pos_JC(self, i = 1):
        '''
        Returns the global coordinates of the joint center connecting the link to it's i-th child. 
        Before calling this function, self.pos_origin and self.q must be set.
        '''
        if not self.updated:
            self.update()
        if self.n_child > 1:
            return self.pos_jc[:,i-1]
        else:
            assert i == 1 
            return self.pos_jc

    def orientation(self):
        '''
        Returns the global orientation of the segment in a 3X3 rotation matrix form
        Before calling this function, self.set_parent_pose and self.set_q must be called.
        '''
        if not self.updated:
            self.update(symbolic = symbolic, C = C, S = S)
        return self.ori


    def pos_COM(self):
        '''
        Returns the global coordinates of the segment COM
        Before calling this function, self.set_parent_pose and self.set_q must be called.
        '''
        if not self.updated:
            self.update()
        return self.pos_com

    def set_q(self, qd):
        
        if qd < self.ql:
            self.q = self.ql
        elif qd > self.qh:
            self.q = self.qh
        else:
            self.q = qd

        self.c = math.cos(qd)
        self.s = math.sin(qd)

        if gen.equal(self.c, 0.0):
            self.c = 0
        elif gen.equal(self.c, 1.0):
            self.c = 1
        elif gen.equal(self.c, - 1.0):
            self.c = -1
        else:
            if self.symbolic:
                self.c = self.C 
                
        if gen.equal(self.s, 0.0):
            self.s = 0
        elif gen.equal(self.s, 1.0):
            self.s = 1
        elif gen.equal(self.s, - 1.0):
            self.s = -1
        else:
            if self.symbolic:
                self.s = self.S 
        
        self.updated = False

    def set_parent_pose(self, position, orientation):
        self.pos_origin = np.copy(position)
        self.parent_ori = np.copy(orientation)
        self.updated = False

    def update(self):
        if not self.updated:
            rv = np.append(self.q, np.array(self.rot_axis))
            R  = rot.rotation_matrix(rv, parametrization = 'angle_axis', symbolic = self.symbolic, C = self.c, S = self.s, U = self.rot_axis)
            '''
            Note: If you use cgkit tool, remember that converting a Mat3() to a numpy matrix gives the transpose of the matrix
            To convert a Mat3() into a numpy matrix use: R = numpy.array(qt.toMat3()).T
            you can easily verify this with an example !
            '''
            
            self.ori         = np.dot(self.parent_ori, R)
            self.pos_com     = self.pos_origin + np.dot(self.ori, self.COM)
            self.pos_jc      = self.pos_origin + np.dot(self.ori, self.JC)
            self.updated     = True
            
class NAO_Symbolic():
    '''
    z is upwards, x is perpendicular to the sagittal plane, y is towards forward walking direction

        
    '''
    def __init__(self, mass   = {"torso":1.2171, "neck":0.05, "shoulder": 0.07, "head": 0.35, "upperarm": 0.15, "elbow":0.035, "lowerarm":0.2, "hip1": 0.09, "hip2": 0.125, "thigh": 0.275, "shank":0.225, "ankle":0.125, "foot":0.2}):
        '''
        definition of nao segments according to this:
        simspark.sourceforge.net/wiki/images/4/42/Models_NaoBoxModel.png
        '''
        h19 = Symbol('h19') # neack offset from torso center in z direction = 90 mm
        b11 = Symbol('b11') # shoulder offset from torso center in x direction: b11 for right shoulder, -b11 for left  
        h11 = Symbol('h11') # shoulder offset from torso center in z direction = 75 mm
        b0  = Symbol('b0') # hip offset from torso center in x direction: b0 for right hip, -b0 for left hip (b0 = 55 mm)
        h0  = Symbol('h0') # torso center offset from hip JC in z direction = 115 mm
        e0  = Symbol('e0') # torso center offset from hip in y direction = 10 mm

        a19 = Symbol('a19') # head joint center offset from neck (in z direction)     = 60 mm
        h20 = Symbol('h20') # head COM offset from head joint center(in z direction)  = 5 mm

        a12 = Symbol('a12') # elbow JC offset from shoulder center in y direction when NAO is in zero state (hands pointing forward) = 90 mm
        d12 = Symbol('d12') # elbow JC offset from shoulder center in z direction when NAO is in zero state (hands pointing forward) = 9  mm

        b12 = Symbol('b12') # upperarm COM offset from shoulder JC in local x direction = 10 mm
        e12 = Symbol('e12') # upperarm COM offset from shoulder JC in local y direction = 20 mm

        e14 = Symbol('e14') # lowerarm COM offset from elbow JC in local y direction = 50 mm

        a2  = Symbol('a2') # Thigh length or hip JC offset from knee JC in z direction when NAO is in zero state = 120 mm 
        d2  = Symbol('d2') # knee JC offset from hip JC in y direction when NAO is in zero state = 5 mm 

        h2  = Symbol('a2') # hip offset from thigh COM  in z direction when NAO is in zero state = 40 mm 
        e2  = Symbol('e2') # thigh COM offset from hip JC in y direction when NAO is in zero state = 10 mm 

        a3  = Symbol('a3') # shank length or knee JC offset from ankle JC in z direction when NAO is in zero state = 100 mm 
        h3  = Symbol('h3') # knee offset from shank COM  in z direction when NAO is in zero state = 45 mm 
        e3  = Symbol('e3') # shank COM offset from knee JC in y direction when NAO is in zero state =  10 mm 

        a5  = Symbol('a5') # ankle JC offset from foot floor touch point(endeffector) in z direction when NAO is in zero state = 50 mm
        h5  = Symbol('h5') # ankle offset from foot COM  in z direction when NAO is in zero state = 40 mm 
        d5  = Symbol('d5') # foot endeffector offset from ankle JC  in y direction when NAO is in zero state = 30 mm
        e5  = Symbol('e5') # foot COM offset from ankle JC  in y direction when NAO is in zero state = 30 mm

        JCT = np.array([[0  , b11, -b11,   b0, -b0 ], 
                        [0  , 0  ,  0  , - e0, -e0 ],
                        [h19, h11,  h11, - h0, -h0]])

        self.torso = Segment(name = "torso", mass = mass['torso'], n_child = 5, JC = JCT, symbolic = True)
        '''
        Torso is the base(ground) segment, so parent, rot_axis, q, ql, qh for torso are meaningless.
        Child 1: Neck
        Child 2: right shoulder
        Child 3: left shoulder
        Child 4: right hip 1
        Child 5: left hip 1
        '''
        self.torso.updated = True

        self.neck = Segment(name = "neck", parent = "torso", mass = mass['neck'], JC = np.array([0, 0, a19]), ql = -120.0/drc, qh = 120.0/drc, symbolic = True, S = Symbol('s19'), C = Symbol('c19'))
    
        self.head = Segment(name = "head", parent = "neck", mass = mass['head'], n_child = 0, COM = np.array([0, 0, h20]), rot_axis = [1,0,0], ql = -45.0/drc, qh = 45.0/drc, symbolic = True, S = Symbol('s20'), C = Symbol('c20'))
        
        self.r_shoulder = Segment(name = "r_shoulder", parent = "torso", mass = mass['shoulder'], ql = -120.0/drc, qh = 120.0/drc, symbolic = True, S = Symbol('s11'), C = Symbol('c11'))
        self.l_shoulder = Segment(name = "l_shoulder", parent = "torso", mass = mass['shoulder'], ql = -120.0/drc, qh = 120.0/drc, symbolic = True, S = Symbol('s15'), C = Symbol('c15'))

        self.r_upperarm = Segment(name = "r_upperarm", parent = "r_shoulder", mass = mass['upperarm'], COM = np.array([ b12,e12,0]), JC = np.array([0, a12, d12]), ql = -95.0/drc, qh = 1.0/drc, symbolic = True, S = Symbol('s12'), C = Symbol('c12'))
        self.l_upperarm = Segment(name = "l_upperarm", parent = "l_shoulder", mass = mass['upperarm'], COM = np.array([-b12,e12,0]), JC = np.array([0, a12, d12]), ql = -1.0/drc, qh = 95.0/drc, symbolic = True, S = Symbol('s16'), C = Symbol('c16'))

        self.r_elbow = Segment(name = "r_elbow", parent = "r_upperarm", mass = mass['elbow'], ql = -120.0/drc, qh = 120.0/drc, symbolic = True, S = Symbol('s13'), C = Symbol('c13'))
        self.l_elbow = Segment(name = "l_elbow", parent = "l_upperarm", mass = mass['elbow'], ql = -120.0/drc, qh = 120.0/drc, symbolic = True, S = Symbol('s17'), C = Symbol('c17'))

        self.r_lowerarm = Segment(name = "r_lowerarm", parent = "r_elbow", mass = mass['lowerarm'], n_child = 0, COM = np.array([0, e14, 0]), ql = -1.0/drc, qh = 90.0/drc, symbolic = True, S = Symbol('s14'), C = Symbol('c14'))
        self.l_lowerarm = Segment(name = "l_lowerarm", parent = "l_elbow", mass = mass['lowerarm'], n_child = 0, COM = np.array([0, e14, 0]), ql = -90.0/drc, qh = 1.0/drc, symbolic = True, S = Symbol('s18'), C = Symbol('c18'))

        self.r_hip1 = Segment(name = "r_hip1", parent = "torso", mass = mass['hip1'], rot_axis = [-0.7071, 0,  0.7071], ql = -90.0/drc, qh = 1.0/drc, symbolic = True, S = Symbol('s0'), C = Symbol('c0'))
        self.l_hip1 = Segment(name = "l_hip1", parent = "torso", mass = mass['hip1'], rot_axis = [-0.7071, 0, -0.7071], ql = -90.0/drc, qh = 1.0/drc, symbolic = True, S = Symbol('s0'), C = Symbol('c0'))

        self.r_hip2 = Segment(name = "r_hip2", parent = "r_hip1", mass = mass['hip2'], rot_axis = [0,1,0], ql = -45.0/drc, qh = 25.0/drc, symbolic = True, S = Symbol('s1'), C = Symbol('c1'))
        self.l_hip2 = Segment(name = "l_hip2", parent = "l_hip1", mass = mass['hip2'], rot_axis = [0,1,0], ql = -25.0/drc, qh = 45.0/drc, symbolic = True, S = Symbol('s6'), C = Symbol('c6'))

        self.r_thigh = Segment(name = "r_thigh", parent = "r_hip2", mass = mass['thigh'], COM = np.array([0,e2,-h2]), JC = np.array([0,d2,-a2]), rot_axis = [1,0,0], ql = -25.0/drc, qh = 100.0/drc, symbolic = True, S = Symbol('s2'), C = Symbol('c2'))
        self.l_thigh = Segment(name = "l_thigh", parent = "l_hip2", mass = mass['thigh'], COM = np.array([0,e2,-h2]), JC = np.array([0,d2,-a2]), rot_axis = [1,0,0], ql = -25.0/drc, qh = 100.0/drc, symbolic = True, S = Symbol('s7'), C = Symbol('c7'))
        
        self.r_shank = Segment(name = "r_shank", parent = "r_thigh", mass = mass['shank'], COM = np.array([0,e3,-h3]), JC = np.array([0,0,-a3]), rot_axis = [1,0,0], ql = -130.0/drc, qh = 1.0/drc, symbolic = True, S = Symbol('s3'), C = Symbol('c3'))
        self.l_shank = Segment(name = "l_shank", parent = "l_thigh", mass = mass['shank'], COM = np.array([0,e3,-h3]), JC = np.array([0,0,-a3]), rot_axis = [1,0,0], ql = -130.0/drc, qh = 1.0/drc, symbolic = True, S = Symbol('s8'), C = Symbol('c8'))

        self.r_ankle = Segment(name = "r_ankle", parent = "r_shank", mass = mass['ankle'], rot_axis = [1,0,0], ql = -45.0/drc, qh = 75.0/drc, S = Symbol('s4'), C = Symbol('c4'))
        self.l_ankle = Segment(name = "l_ankle", parent = "l_shank", mass = mass['ankle'], rot_axis = [1,0,0], ql = -45.0/drc, qh = 75.0/drc, S = Symbol('s9'), C = Symbol('c9'))
        
        self.r_foot = Segment(name = "r_foot", parent = "r_ankle", mass = mass['foot'], n_child = 0, COM = np.array([0, e5, -h5]), JC = np.array([0,d5,-a5]), rot_axis = [0,1,0], ql = -25.0/drc, qh = 45.0/drc, symbolic = True, S = Symbol('s5'), C = Symbol('c5'))
        self.l_foot = Segment(name = "l_foot", parent = "l_ankle", mass = mass['foot'], n_child = 0, COM = np.array([0, e5, -h5]), JC = np.array([0,d5,-a5]), rot_axis = [0,1,0], ql = -45.0/drc, qh = 25.0/drc, symbolic = True, S = Symbol('s10'), C = Symbol('c10'))

        self.q = np.zeros(21)
        self.updated = False

    def set_config(self, qd = np.zeros(21)):

        self.r_hip1.set_q(qd[0])
        self.r_hip2.set_q(qd[1])
        self.r_thigh.set_q(qd[2])
        self.r_shank.set_q(qd[3])
        self.r_ankle.set_q(qd[4])
        self.r_foot.set_q(qd[5])

        self.l_hip1.set_q(qd[0])
        self.l_hip2.set_q(qd[6])
        self.l_thigh.set_q(qd[7])
        self.l_shank.set_q(qd[8])
        self.l_ankle.set_q(qd[9])
        self.l_foot.set_q(qd[10])

        self.r_shoulder.set_q(qd[11])
        self.r_upperarm.set_q(qd[12])
        self.r_elbow.set_q(qd[13])
        self.r_lowerarm.set_q(qd[14])

        self.r_shoulder.set_q(qd[15])
        self.r_upperarm.set_q(qd[16])
        self.r_elbow.set_q(qd[17])
        self.r_lowerarm.set_q(qd[18])

        self.neck.set_q(qd[19])
        self.head.set_q(qd[20])
    
        self.updated = False

    def update(self):
        if not self.updated:
            self.neck.set_parent_pose(self.torso.pos_JC(), self.torso.orientation()) 
            self.head.set_parent_pose(self.neck.pos_JC(), self.neck.orientation())
            self.head.update()
            
            self.r_shoulder.set_parent_pose(self.torso.pos_JC(2), self.torso.orientation())
            self.r_upperarm.set_parent_pose(self.r_shoulder.pos_JC(), self.r_shoulder.orientation())
            self.r_elbow.set_parent_pose(self.r_upperarm.pos_JC(), self.r_upperarm.orientation())
            self.r_lowerarm.set_parent_pose(self.r_elbow.pos_JC(), self.r_elbow.orientation())
            self.r_lowerarm.update()

            self.l_shoulder.set_parent_pose(self.torso.pos_JC(3), self.torso.orientation())
            self.l_upperarm.set_parent_pose(self.l_shoulder.pos_JC(), self.l_shoulder.orientation())
            self.l_elbow.set_parent_pose(self.l_upperarm.pos_JC(), self.l_upperarm.orientation())
            self.l_lowerarm.set_parent_pose(self.l_elbow.pos_JC(), self.l_elbow.orientation())
            self.l_lowerarm.update()

            self.r_hip1.set_parent_pose(self.torso.pos_JC(4), self.torso.orientation())
            self.r_hip2.set_parent_pose(self.r_hip1.pos_JC(), self.r_hip1.orientation())
            self.r_thigh.set_parent_pose(self.r_hip2.pos_JC(), self.r_hip2.orientation())
            self.r_shank.set_parent_pose(self.r_thigh.pos_JC(), self.r_thigh.orientation())
            self.r_ankle.set_parent_pose(self.r_shank.pos_JC(), self.r_shank.orientation())
            self.r_foot.set_parent_pose(self.r_ankle.pos_JC(), self.r_ankle.orientation())
            self.r_foot.update()
        
            self.l_hip1.set_parent_pose(self.torso.pos_JC(5), self.torso.orientation())
            self.l_hip2.set_parent_pose(self.l_hip1.pos_JC(), self.l_hip1.orientation())
            self.l_thigh.set_parent_pose(self.l_hip2.pos_JC(), self.l_hip2.orientation())
            self.l_shank.set_parent_pose(self.l_thigh.pos_JC(), self.l_thigh.orientation())
            self.l_ankle.set_parent_pose(self.l_shank.pos_JC(), self.l_shank.orientation())
            self.l_foot.set_parent_pose(self.l_ankle.pos_JC(), self.l_ankle.orientation())
            self.l_foot.update()

            self.updated = True

class NAO():
    '''
    The defaults are extracted from NAO model ... :
    '''
    def __init__(self, mass   = {"torso":1.2171, "neck":0.05, "shoulder": 0.07, "head": 0.35, "upperarm": 0.15, "elbow":0.035, "lowerarm":0.2, "hip1": 0.09, "hip2": 0.125, "thigh": 0.275, "shank":0.225, "ankle":0.125, "foot":0.2},
                       tibia_length = 102.9, 
                       thigh_length = 100.0, 
                       hip_offset   = [0.0, 50.0, - 85.0], 
                       foot_offset  = [0.0, 0.0, -45.19]):
        '''
        definition of nao segments according to this:
        simspark.sourceforge.net/wiki/images/4/42/Models_NaoBoxModel.png
        '''
        JCT = np.array([[0.0  , 98.0, -98.0,  55.0 , -55.0 ], 
                        [0.0  , 0.00,  0.00, -10.0 , -10.0 ],
                        [90.0 , 75.0,  75.0, -115.0, -115.0]])

        self.torso = Segment(name = "torso", mass = mass['torso'], n_child = 5, JC = JCT)
        '''
        Torso is the base(ground) segment, so parent, rot_axis, q, ql, qh for torso are meaningless.
        Child 1: Neck
        Child 2: right shoulder
        Child 3: left shoulder
        Child 4: right hip 1
        Child 5: left hip 1
        '''
        self.torso.updated = True

        self.neck = Segment(name = "neck", parent = "torso", mass = mass['neck'], JC = np.array([0.0, 0.0, 60.0]), ql = -120.0/drc, qh = 120.0/drc)
    
        self.head = Segment(name = "head", parent = "neck", mass = mass['head'], n_child = 0, COM = np.array([0.0, 0.0, 5.0]), rot_axis = [1,0,0], ql = -45.0/drc, qh = 45.0/drc)
        
        self.r_shoulder = Segment(name = "r_shoulder", parent = "torso", mass = mass['shoulder'], ql = -120.0/drc, qh = 120.0/drc)
        self.l_shoulder = Segment(name = "l_shoulder", parent = "torso", mass = mass['shoulder'], ql = -120.0/drc, qh = 120.0/drc)

        self.r_upperarm = Segment(name = "r_upperarm", parent = "r_shoulder", mass = mass['upperarm'], COM = np.array([ 10.0,20.0,0.0]), JC = np.array([0.0,90.0,9.0]), ql = -95.0/drc, qh = 1.0/drc)
        self.l_upperarm = Segment(name = "l_upperarm", parent = "l_shoulder", mass = mass['upperarm'], COM = np.array([-10.0,20.0,0.0]), JC = np.array([0.0,90.0,9.0]), ql = -1.0/drc, qh = 95.0/drc)

        self.r_elbow = Segment(name = "r_elbow", parent = "r_upperarm", mass = mass['elbow'], ql = -120.0/drc, qh = 120.0/drc)
        self.l_elbow = Segment(name = "l_elbow", parent = "l_upperarm", mass = mass['elbow'], ql = -120.0/drc, qh = 120.0/drc)

        self.r_lowerarm = Segment(name = "r_lowerarm", parent = "r_elbow", mass = mass['lowerarm'], n_child = 0, COM = np.array([0.0, 50.0, 0.0]), ql = -1.0/drc, qh = 90.0/drc)
        self.l_lowerarm = Segment(name = "l_lowerarm", parent = "l_elbow", mass = mass['lowerarm'], n_child = 0, COM = np.array([0.0, 50.0, 0.0]), ql = -90.0/drc, qh = 1.0/drc)

        self.r_hip1 = Segment(name = "r_hip1", parent = "torso", mass = mass['hip1'], rot_axis = [-0.7071, 0,  0.7071], ql = -90.0/drc, qh = 1.0/drc)
        self.l_hip1 = Segment(name = "l_hip1", parent = "torso", mass = mass['hip1'], rot_axis = [-0.7071, 0, -0.7071], ql = -90.0/drc, qh = 1.0/drc)

        self.r_hip2 = Segment(name = "r_hip2", parent = "r_hip1", mass = mass['hip2'], rot_axis = [0,1,0], ql = -45.0/drc, qh = 25.0/drc)
        self.l_hip2 = Segment(name = "l_hip2", parent = "l_hip1", mass = mass['hip2'], rot_axis = [0,1,0], ql = -25.0/drc, qh = 45.0/drc)

        self.r_thigh = Segment(name = "r_thigh", parent = "r_hip2", mass = mass['thigh'], COM = np.array([0.0,10.0,-40.0]), JC = np.array([0.0,5.0,-120.0]), rot_axis = [1,0,0], ql = -25.0/drc, qh = 100.0/drc)
        self.l_thigh = Segment(name = "l_thigh", parent = "l_hip2", mass = mass['thigh'], COM = np.array([0.0,10.0,-40.0]), JC = np.array([0.0,5.0,-120.0]), rot_axis = [1,0,0], ql = -25.0/drc, qh = 100.0/drc)
        
        self.r_shank = Segment(name = "r_shank", parent = "r_thigh", mass = mass['shank'], COM = np.array([0.0,10.0,-45.0]), JC = np.array([0.0,0.0,-100.0]), rot_axis = [1,0,0], ql = -130.0/drc, qh = 1.0/drc)
        self.l_shank = Segment(name = "l_shank", parent = "l_thigh", mass = mass['shank'], COM = np.array([0.0,10.0,-45.0]), JC = np.array([0.0,0.0,-100.0]), rot_axis = [1,0,0], ql = -130.0/drc, qh = 1.0/drc)

        self.r_ankle = Segment(name = "r_ankle", parent = "r_shank", mass = mass['ankle'], rot_axis = [1,0,0], ql = -45.0/drc, qh = 75.0/drc)
        self.l_ankle = Segment(name = "l_ankle", parent = "l_shank", mass = mass['ankle'], rot_axis = [1,0,0], ql = -45.0/drc, qh = 75.0/drc)
        
        self.r_foot = Segment(name = "r_foot", parent = "r_ankle", mass = mass['foot'], n_child = 0, COM = np.array([0.0, 30.0, -40.0]), JC = np.array([0.0, 30.0, -50.0]), rot_axis = [0,1,0], ql = -25.0/drc, qh = 45.0/drc)
        self.l_foot = Segment(name = "l_foot", parent = "l_ankle", mass = mass['foot'], n_child = 0, COM = np.array([0.0, 30.0, -40.0]), JC = np.array([0.0, 30.0, -50.0]), rot_axis = [0,1,0], ql = -45.0/drc, qh = 25.0/drc)

        self.q = np.zeros(21)
        self.updated = False
        
    def set_config(self, qd = np.zeros(21)):
        
        self.r_hip1.set_q(qd[0])
        self.r_hip2.set_q(qd[1])
        self.r_thigh.set_q(qd[2])
        self.r_shank.set_q(qd[3])
        self.r_ankle.set_q(qd[4])
        self.r_foot.set_q(qd[5])

        self.l_hip1.set_q(qd[0])
        self.l_hip2.set_q(qd[6])
        self.l_thigh.set_q(qd[7])
        self.l_shank.set_q(qd[8])
        self.l_ankle.set_q(qd[9])
        self.l_foot.set_q(qd[10])

        self.r_shoulder.set_q(qd[11])
        self.r_upperarm.set_q(qd[12])
        self.r_elbow.set_q(qd[13])
        self.r_lowerarm.set_q(qd[14])

        self.r_shoulder.set_q(qd[15])
        self.r_upperarm.set_q(qd[16])
        self.r_elbow.set_q(qd[17])
        self.r_lowerarm.set_q(qd[18])

        self.neck.set_q(qd[19])
        self.head.set_q(qd[20])
    
        self.updated = False
        
    def update(self):
        if not self.updated:
            self.neck.set_parent_pose(self.torso.pos_JC(), self.torso.orientation()) 
            self.head.set_parent_pose(self.neck.pos_JC(), self.neck.orientation())
            self.head.update()
            
            self.r_shoulder.set_parent_pose(self.torso.pos_JC(2), self.torso.orientation())
            self.r_upperarm.set_parent_pose(self.r_shoulder.pos_JC(), self.r_shoulder.orientation())
            self.r_elbow.set_parent_pose(self.r_upperarm.pos_JC(), self.r_upperarm.orientation())
            self.r_lowerarm.set_parent_pose(self.r_elbow.pos_JC(), self.r_elbow.orientation())
            self.r_lowerarm.update()

            self.l_shoulder.set_parent_pose(self.torso.pos_JC(3), self.torso.orientation())
            self.l_upperarm.set_parent_pose(self.l_shoulder.pos_JC(), self.l_shoulder.orientation())
            self.l_elbow.set_parent_pose(self.l_upperarm.pos_JC(), self.l_upperarm.orientation())
            self.l_lowerarm.set_parent_pose(self.l_elbow.pos_JC(), self.l_elbow.orientation())
            self.l_lowerarm.update()

            self.r_hip1.set_parent_pose(self.torso.pos_JC(4), self.torso.orientation())
            self.r_hip2.set_parent_pose(self.r_hip1.pos_JC(), self.r_hip1.orientation())
            self.r_thigh.set_parent_pose(self.r_hip2.pos_JC(), self.r_hip2.orientation())
            self.r_shank.set_parent_pose(self.r_thigh.pos_JC(), self.r_thigh.orientation())
            self.r_ankle.set_parent_pose(self.r_shank.pos_JC(), self.r_shank.orientation())
            self.r_foot.set_parent_pose(self.r_ankle.pos_JC(), self.r_ankle.orientation())
            self.r_foot.update()
        
            self.l_hip1.set_parent_pose(self.torso.pos_JC(5), self.torso.orientation())
            self.l_hip2.set_parent_pose(self.l_hip1.pos_JC(), self.l_hip1.orientation())
            self.l_thigh.set_parent_pose(self.l_hip2.pos_JC(), self.l_hip2.orientation())
            self.l_shank.set_parent_pose(self.l_thigh.pos_JC(), self.l_thigh.orientation())
            self.l_ankle.set_parent_pose(self.l_shank.pos_JC(), self.l_shank.orientation())
            self.l_foot.set_parent_pose(self.l_ankle.pos_JC(), self.l_ankle.orientation())
            self.l_foot.update()

            self.updated = True


    


