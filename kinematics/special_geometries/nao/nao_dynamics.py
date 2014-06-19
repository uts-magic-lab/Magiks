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
@version:	    5.0
Last Revision:  20 June 2014

Changes from previous version:

added "pos_updated", "ori_updated" and "jac_updated" flags for the segments and the robot to avoid repeating calculations
This will reduce computational costs
functions for calculating the Jacobian matrix added
'''

import numpy as np
import math, copy
from sympy import Symbol

import packages.nima.mathematics.vectors_and_matrices as vm
import packages.nima.mathematics.general as gen
import packages.nima.mathematics.rotation as rot

drc     = math.pi/180.00

dflt_mass = {"torso":1.2171, "neck":0.05, "shoulder": 0.07, "head": 0.35, "upperarm": 0.15, "elbow":0.035, "lowerarm":0.2, "hip1": 0.09, "hip2": 0.125, "thigh": 0.275, "shank":0.225, "ankle":0.125, "foot":0.2}
dflt_dims = {"h19":90.0, "b11":98.0,"h11":75.0,"a2":120.0,"d2":5.0,"h2":40.0,"e2":10.0,"a3":100.0,"h3":45.0,"e3":10.0,"a5":50.0,"h5":40.0,"d5":30.0,"e5":30.0,
                "e14" : 50.0, "e12":20.0,"b12":10.0, "d12":9.0, "a12":90.0, "h20":5.0, "a19":60.0, "e0":10.0, "h0":115.0, "b0":55.0}
dflt_mass_symb = {"torso":Symbol('m0'), "neck":Symbol('m19'), "shoulder": Symbol('m11'), "head" : Symbol('m20'), "upperarm": Symbol('m12'), "elbow":Symbol('m13'), "lowerarm":Symbol('m14'), 
                      "hip1" :Symbol('m1'), "hip2": Symbol('m2'), "thigh"   : Symbol('m3') , "shank": Symbol('m4') , "ankle"   : Symbol('m5'),  "foot" :Symbol('m6')}
dflt_dims_symb = {"h19":Symbol('h19'), "b11":Symbol('b11'),"h11":Symbol('h11'),"a2":Symbol('a2'),"d2":Symbol('d2'),"h2" :Symbol('h2') ,"e2" :Symbol('e2') ,"a3" :Symbol('a3') ,"h3" :Symbol('h3'),
                      "e3" :Symbol('e3') , "a5" :Symbol('a5') ,"h5" :Symbol('h5') ,"d5":Symbol('d5'),"e5":Symbol('e5'),"e14":Symbol('e14'),"e12":Symbol('e12'),"b12":Symbol('b12'),"d12":Symbol('d12'),              "a12":Symbol('a12'), "h20":Symbol('h20'),"a19":Symbol('a19'),"e0":Symbol('e0'),"h0":Symbol('h0'),"b0":Symbol('b0')}


class Segment(object):
    def __init__(self, name = "Unnamed", parent = "", mass = 1.0, children = [], child_number = 1, POS = [np.zeros(3), np.zeros(3)], rot_axis = [0,0,1], q = 0.0, ql = - math.pi, qh = math.pi, symb = False, S = Symbol('s'), C = Symbol('c')):
        '''
        JC : A matrix of 3 rows and n columns where n is the number of children. Each column specifies the local coordinates of the Joint Center of each child segment.
        COM: Local coordinates of the Center of Mass
        '''
        self.name     = name
        self.parent   = parent
        self.mass     = mass
        self.children = children
        self.child_number  = child_number
        self.POS      = POS  # local position of reference points (POS[0]: local position of COM, POS[1,...,n] local position of the Joint Centers of Children 1..n)
        self.rot_axis = rot_axis
        self.ql       = ql
        self.qh       = qh

        self.symb = symb
        self.C        = C
        self.S        = S

        self.set_q(q)
        
        self.pos = copy.copy(POS)  # global coordinates of reference points
    
        self.orientation  = np.eye(3)
        self.pos_updated  = [False for i in range(len(POS))]
        self.ori_updated  = False

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
            if self.symb:
                self.c = self.C 
                
        if gen.equal(self.s, 0.0):
            self.s = 0
        elif gen.equal(self.s, 1.0):
            self.s = 1
        elif gen.equal(self.s, - 1.0):
            self.s = -1
        else:
            if self.symb:
                self.s = self.S 
        
class NAO():
    '''
    The defaults are extracted from NAO model ... :
    '''
    def __init__(self, mass = dflt_mass, dims = dflt_dims, mass_symb = dflt_mass_symb, dims_symb = dflt_dims_symb, symb = False):
        '''
        definition of nao segments according to this:
        simspark.sourceforge.net/wiki/images/4/42/Models_NaoBoxModel.png

        h19 : neack offset from torso center in z direction = 90 mm
        b11 : shoulder offset from torso center in x direction = 98 mm ( b11 for right shoulder, -b11 for left)  
        h11 : shoulder offset from torso center in z direction = 75 mm
        b0  : hip offset from torso center in x direction: b0 for right hip, -b0 for left hip (b0 = 55 mm)
        h0  : torso center offset from hip JC in z direction = 115 mm
        e0  : torso center offset from hip in y direction = 10 mm

        a19 : head joint center offset from neck (in z direction)     = 60 mm
        h20 : head COM offset from head joint center(in z direction)  = 5 mm

        a12 : elbow JC offset from shoulder center in y direction when NAO is in zero state (hands pointing forward) = 90 mm
        d12 : elbow JC offset from shoulder center in z direction when NAO is in zero state (hands pointing forward) = 9  mm

        b12 : upperarm COM offset from shoulder JC in local x direction = 10 mm
        e12 : upperarm COM offset from shoulder JC in local y direction = 20 mm

        e14 : lowerarm COM offset from elbow JC in local y direction = 50 mm

        a2  : Thigh length or hip JC offset from knee JC in z direction when NAO is in zero state = 120 mm 
        d2  : knee JC offset from hip JC in y direction when NAO is in zero state = 5 mm 

        h2  : hip offset from thigh COM  in z direction when NAO is in zero state = 40 mm 
        e2  : thigh COM offset from hip JC in y direction when NAO is in zero state = 10 mm 

        a3  : shank length or knee JC offset from ankle JC in z direction when NAO is in zero state = 100 mm 
        h3  : knee offset from shank COM  in z direction when NAO is in zero state = 45 mm 
        e3  : shank COM offset from knee JC in y direction when NAO is in zero state =  10 mm 

        a5  : ankle JC offset from foot floor touch point(endeffector) in z direction when NAO is in zero state = 50 mm
        h5  : ankle offset from foot COM  in z direction when NAO is in zero state = 40 mm 
        d5  : foot endeffector offset from ankle JC  in y direction when NAO is in zero state = 30 mm
        e5  : foot COM offset from ankle JC  in y direction when NAO is in zero state = 30 mm
        '''
        self.mass      = mass
        self.mass_symb = mass_symb
        self.dims      = dims
        self.dims_symb = dims_symb

        self.q   = np.zeros(21)
        self.com = 0.0
        self.all_segments = ['torso','r_hip1', 'r_hip2', 'r_thigh', 'r_shank', 'r_ankle', 'r_foot', 
                                     'l_hip1', 'l_hip2', 'l_thigh', 'l_shank', 'l_ankle', 'l_foot', 
                            'r_shoulder', 'r_upperarm', 'r_elbow', 'r_lowerarm',
                            'l_shoulder', 'l_upperarm', 'l_elbow', 'l_lowerarm', 'neck', 'head']

        self.set_symb(symb)
        
    def generate_segments(self, mass, dim):

        '''
        Torso is the base(ground) segment, so parent, rot_axis, q, ql, qh for torso are meaningless.
        Child 1: Neck
        Child 2: right shoulder
        Child 3: left shoulder
        Child 4: right hip 1
        Child 5: left hip 1
        '''
        POST = [np.zeros(3),np.array([0,0,dim['h19']]),np.array([dim['b11'],0,dim['h11']]),np.array([-dim['b11'],0,dim['h11']]),np.array([dim['b0'],-dim['e0'],-dim['h0']]),np.array([-dim['b0'],-dim['e0'],-dim['h0']])]

        torso = Segment(name = "torso", mass = mass['torso'], POS = POST, children = ['neck', 'r_shoulder', 'l_shoulder', 'r_hip1', 'l_hip1'])
        torso.pos_updated = [True, True, True, True, True, True]
        torso.ori_updated = True

        neck = Segment(name = "neck", parent = "torso", mass = mass['neck'], POS = [np.zeros(3),np.array([0, 0, dim['a19']])], ql = -120.0/drc, qh = 120.0/drc, symb = self.symb, S = Symbol('s19'), C = Symbol('c19'), children = ['head'])
    
        head = Segment(name = "head", parent = "neck", mass = mass['head'], POS = [np.array([0, 0, dim['h20']])], rot_axis = [1,0,0], ql = -45.0/drc, qh = 45.0/drc, symb = self.symb, S = Symbol('s20'), C = Symbol('c20'))
        
        r_shoulder = Segment(name = "r_shoulder", parent = "torso", children = ['r_upperarm'], mass = mass['shoulder'], child_number = 2, ql = -120.0/drc, qh = 120.0/drc, symb = self.symb, rot_axis = [1,0,0], S = Symbol('s11'), C = Symbol('c11'))
        l_shoulder = Segment(name = "l_shoulder", parent = "torso", children = ['l_upperarm'], mass = mass['shoulder'], child_number = 3, ql = -120.0/drc, qh = 120.0/drc, symb = self.symb, rot_axis = [1,0,0], S = Symbol('s15'), C = Symbol('c15'))

        r_upperarm = Segment(name = "r_upperarm", parent = "r_shoulder", children = ['r_elbow'], mass = mass['upperarm'], POS = [np.array([ dim['b12'],dim['e12'],0]),np.array([0, dim['a12'], dim['d12']])], ql = -95.0/drc, qh = 1.0/drc, symb = self.symb, S = Symbol('s12'), C = Symbol('c12'))
        l_upperarm = Segment(name = "l_upperarm", parent = "l_shoulder", children = ['l_elbow'], mass = mass['upperarm'], POS = [np.array([-dim['b12'],dim['e12'],0]),np.array([0, dim['a12'], dim['d12']])], ql = -1.0/drc, qh = 95.0/drc, symb = self.symb, S = Symbol('s16'), C = Symbol('c16'))

        r_elbow = Segment(name = "r_elbow", parent = "r_upperarm", children = ['r_lowerarm'], mass = mass['elbow'], ql = -120.0/drc, qh = 120.0/drc, symb = self.symb, rot_axis = [0,1,0], S = Symbol('s13'), C = Symbol('c13'))
        l_elbow = Segment(name = "l_elbow", parent = "l_upperarm", children = ['l_lowerarm'], mass = mass['elbow'], ql = -120.0/drc, qh = 120.0/drc, symb = self.symb, rot_axis = [0,1,0], S = Symbol('s17'), C = Symbol('c17'))

        r_lowerarm = Segment(name = "r_lowerarm", parent = "r_elbow", mass = mass['lowerarm'], POS = [np.array([0, dim['e14'], 0])], ql = -1.0/drc, qh = 90.0/drc, symb = self.symb, S = Symbol('s14'), C = Symbol('c14'))
        l_lowerarm = Segment(name = "l_lowerarm", parent = "l_elbow", mass = mass['lowerarm'], POS = [np.array([0, dim['e14'], 0])], ql = -90.0/drc, qh = 1.0/drc, symb = self.symb, S = Symbol('s18'), C = Symbol('c18'))

        r_hip1 = Segment(name = "r_hip1", parent = "torso", children = ['r_hip2'], mass = mass['hip1'], child_number = 4, rot_axis = [-0.7071, 0,  0.7071], ql = -90.0/drc, qh = 1.0/drc, symb = self.symb, S = Symbol('s0'), C = Symbol('c0'))
        l_hip1 = Segment(name = "l_hip1", parent = "torso", children = ['l_hip2'], mass = mass['hip1'], child_number = 5, rot_axis = [-0.7071, 0, -0.7071], ql = -90.0/drc, qh = 1.0/drc, symb = self.symb, S = Symbol('s0'), C = Symbol('c0'))

        r_hip2 = Segment(name = "r_hip2", parent = "r_hip1", children = ['r_thigh'], mass = mass['hip2'], rot_axis = [0,1,0], ql = -45.0/drc, qh = 25.0/drc, symb = self.symb, S = Symbol('s1'), C = Symbol('c1'))
        l_hip2 = Segment(name = "l_hip2", parent = "l_hip1", children = ['l_thigh'], mass = mass['hip2'], rot_axis = [0,1,0], ql = -25.0/drc, qh = 45.0/drc, symb = self.symb, S = Symbol('s6'), C = Symbol('c6'))

        r_thigh = Segment(name = "r_thigh", parent = "r_hip2", children = ['r_shank'], mass = mass['thigh'], POS = [np.array([0,dim['e2'],-dim['h2']]),np.array([0,dim['d2'],-dim['a2']])], rot_axis = [1,0,0], ql = -25.0/drc, qh = 100.0/drc, symb = self.symb, S = Symbol('s2'), C = Symbol('c2'))
        l_thigh = Segment(name = "l_thigh", parent = "l_hip2", children = ['l_shank'], mass = mass['thigh'], POS = [np.array([0,dim['e2'],-dim['h2']]),np.array([0,dim['d2'],-dim['a2']])], rot_axis = [1,0,0], ql = -25.0/drc, qh = 100.0/drc, symb = self.symb, S = Symbol('s7'), C = Symbol('c7'))
        
        r_shank = Segment(name = "r_shank", parent = "r_thigh", children = ['r_ankle'], mass = mass['shank'], POS = [np.array([0,dim['e3'],-dim['h3']]),np.array([0,0,-dim['a3']])], rot_axis = [1,0,0], ql = -130.0/drc, qh = 1.0/drc, symb = self.symb, S = Symbol('s3'), C = Symbol('c3'))
        l_shank = Segment(name = "l_shank", parent = "l_thigh", children = ['l_ankle'], mass = mass['shank'], POS = [np.array([0,dim['e3'],-dim['h3']]),np.array([0,0,-dim['a3']])], rot_axis = [1,0,0], ql = -130.0/drc, qh = 1.0/drc, symb = self.symb, S = Symbol('s8'), C = Symbol('c8'))

        r_ankle = Segment(name = "r_ankle", parent = "r_shank", children = ['r_foot'], mass = mass['ankle'], rot_axis = [1,0,0], ql = -45.0/drc, qh = 75.0/drc, symb = self.symb, S = Symbol('s4'), C = Symbol('c4'))
        l_ankle = Segment(name = "l_ankle", parent = "l_shank", children = ['l_foot'], mass = mass['ankle'], rot_axis = [1,0,0], ql = -45.0/drc, qh = 75.0/drc, symb = self.symb, S = Symbol('s9'), C = Symbol('c9'))
        
        r_foot = Segment(name = "r_foot", parent = "r_ankle", mass = mass['foot'], POS = [np.array([0, dim['e5'], -dim['h5']]),np.array([0,dim['d5'],-dim['a5']])], rot_axis = [0,1,0], ql = -25.0/drc, qh = 45.0/drc, symb = self.symb, S = Symbol('s5'), C = Symbol('c5'))
        l_foot = Segment(name = "l_foot", parent = "l_ankle", mass = mass['foot'], POS = [np.array([0, dim['e5'], -dim['h5']]),np.array([0,dim['d5'],-dim['a5']])], rot_axis = [0,1,0], ql = -45.0/drc, qh = 25.0/drc, symb = self.symb, S = Symbol('s10'), C = Symbol('c10'))

        self.segment = {"torso":torso, "neck":neck, "r_shoulder": r_shoulder, "l_shoulder": l_shoulder, "head": head, "r_upperarm": r_upperarm, "l_upperarm": l_upperarm, "r_elbow":r_elbow, "l_elbow":l_elbow, "r_lowerarm":r_lowerarm,"l_lowerarm":l_lowerarm, "r_hip1": r_hip1, "l_hip1": l_hip1, "r_hip2": r_hip2, "l_hip2": l_hip2, "r_thigh": r_thigh, "l_thigh": l_thigh, "r_shank": r_shank,"l_shank": l_shank, "r_ankle":r_ankle, "l_ankle":l_ankle, "r_foot":r_foot, "l_foot":l_foot}

        for seg_name in self.all_segments:
            self.segment[seg_name].effective_joints = self.effective_joints(seg_name)
            self.segment[seg_name].partial_pos  = [{j:None for  j in self.segment[seg_name].effective_joints} for  i in range(len(self.segment[seg_name].POS))]
            self.segment[seg_name].partial_ori  = {j:None for  j in self.segment[seg_name].effective_joints}

    def set_symb(self, symb = False):
        self.symb = symb
        if self.symb:
            self.generate_segments(mass = self.mass_symb, dim = self.dims_symb)
        else:
            self.generate_segments(mass = self.mass, dim = self.dims)

        self.set_config(self.q)

    def unupdate(self, seg_name):
        if not seg_name == 'torso':
            self.segment[seg_name].pos_updated = [False for i in range(len(self.segment[seg_name].POS))]
            self.segment[seg_name].ori_updated = False        
            self.segment[seg_name].partial_pos  = [{j:None for  j in self.segment[seg_name].effective_joints} for  i in range(len(self.segment[seg_name].POS))]
            self.segment[seg_name].partial_ori  = {j:None for  j in self.segment[seg_name].effective_joints}

        for child in self.segment[seg_name].children:
            self.unupdate(child)
    
    def set_config(self, qd = np.zeros(21)):
        
        self.segment['r_hip1'].set_q(qd[0])
        self.segment['r_hip2'].set_q(qd[1])
        self.segment['r_thigh'].set_q(qd[2])
        self.segment['r_shank'].set_q(qd[3])
        self.segment['r_ankle'].set_q(qd[4])
        self.segment['r_foot'].set_q(qd[5])

        self.segment['l_hip1'].set_q(qd[0])
        self.segment['l_hip2'].set_q(qd[6])
        self.segment['l_thigh'].set_q(qd[7])
        self.segment['l_shank'].set_q(qd[8])
        self.segment['l_ankle'].set_q(qd[9])
        self.segment['l_foot'].set_q(qd[10])

        self.segment['r_shoulder'].set_q(qd[11])
        self.segment['r_upperarm'].set_q(qd[12])
        self.segment['r_elbow'].set_q(qd[13])
        self.segment['r_lowerarm'].set_q(qd[14])

        self.segment['l_shoulder'].set_q(qd[15])
        self.segment['l_upperarm'].set_q(qd[16])
        self.segment['l_elbow'].set_q(qd[17])
        self.segment['l_lowerarm'].set_q(qd[18])

        self.segment['neck'].set_q(qd[19])
        self.segment['head'].set_q(qd[20])

        self.unupdate('torso')
        self.com = None # position of center of mass with respect to the torso in torso coordinate system
        self.pl  = None # position of left foot with respect to the right foot in right foot coordinate system
        self.pr  = None # position of right foot with respect to the left foot in left foot coordinate system
        self.pcl = None # position of center of mass with respect to the left foot in left foot coordinate system
        self.pcr = None # position of center of mass with respect to the right foot in right foot coordinate system

        self.rfoot_jacobian_updated = False
        self.lfoot_jacobian_updated = False
        self.com_l_jacobian_updated = False
        self.com_r_jacobian_updated = False


    def effective_joints(self, seg_name):
        '''
        returns the list of joint names that can influence the pose of link "seg_name"
        '''
        if seg_name == 'torso':
            return []
        else:
            ej = self.effective_joints(self.segment[seg_name].parent)
            ej.insert(0, seg_name)
            return ej

    def position(self, seg_name, pos_num = 0):
        '''
        if pos_num = 0, the global coordinates of the Center of Mass is returned
        '''   

        if not self.segment[seg_name].pos_updated[pos_num]:
            p0 = self.position(self.segment[seg_name].parent, self.segment[seg_name].child_number)
            self.segment[seg_name].pos[pos_num]  = p0 + np.dot(self.orientation(seg_name), self.segment[seg_name].POS[pos_num])
            self.segment[seg_name].pos_updated[pos_num] = True
        return self.segment[seg_name].pos[pos_num]

    def orientation(self, seg_name):

        if not self.segment[seg_name].ori_updated:
            R0 = self.orientation(self.segment[seg_name].parent)
            rv = np.append(self.segment[seg_name].q, np.array(self.segment[seg_name].rot_axis))
            R  = rot.rotation_matrix(rv, parametrization = 'angle_axis', symbolic = self.symb, C = self.segment[seg_name].c, S = self.segment[seg_name].s, U = self.segment[seg_name].rot_axis)
            self.segment[seg_name].orientation = np.dot(R0, R)
            self.ori_updated = True
        return self.segment[seg_name].orientation
        

    def partial_position(self, seg_name, joint_name, pos_num = 0):


        if (seg_name == 'torso') or (not (joint_name in self.segment[seg_name].effective_joints)):
            return np.zeros(3)
        else:
            if self.segment[seg_name].partial_pos[pos_num][joint_name] == None:
                dp0 = self.partial_position(self.segment[seg_name].parent, joint_name = joint_name, pos_num = self.segment[seg_name].child_number)
                dR  = self.partial_orientation(seg_name, joint_name = joint_name)
                self.segment[seg_name].partial_pos[pos_num][joint_name]  = dp0 + np.dot(dR, self.segment[seg_name].POS[pos_num])
            return self.segment[seg_name].partial_pos[pos_num][joint_name]

    def partial_orientation(self, seg_name, joint_name):

        seg = self.segment[seg_name]
        if (seg_name == 'torso') or (not (joint_name in self.segment[seg_name].effective_joints)):
            return np.zeros((3,3))
        else:
            if self.segment[seg_name].partial_ori[joint_name] == None:
                dR0 = self.partial_orientation(seg.parent, joint_name)
                R0  = self.orientation(seg.parent)
                rv  = np.append(seg.q, np.array(seg.rot_axis))
                R   = rot.rotation_matrix(rv, parametrization = 'angle_axis', symbolic = self.symb, C = seg.c, S = seg.s, U = seg.rot_axis)
                if joint_name == seg_name:
                    drv = np.append(seg.q, np.array(seg.rot_axis))
                    dR  = rot.rotation_matrix(drv, parametrization = 'angle_axis', symbolic = self.symb, derivative = True, C = seg.c, S = seg.s, U = seg.rot_axis)
                else:
                    dR = np.zeros((3,3))
                self.segment[seg_name].partial_ori[joint_name] = np.dot(dR0, R) + np.dot(R0, dR)
            return self.segment[seg_name].partial_ori[joint_name]

    def pos_com(self):
        '''
        Returns the position of robot Center of Mass with respect to the torso in torso coordinate system
        '''
        if self.com == None:

            p = np.zeros(3)
            M = 0.0
            for seg_name in self.all_segments:
                p = p + self.segment[seg_name].mass*self.position(seg_name)
                M = M + self.segment[seg_name].mass
            self.com = p/M

        return self.com

    def pos_com_wrt_rfoot(self):
        if self.pcr == None:
            p       = self.pos_com() - self.position('r_foot', 1)
            self.pcr = np.dot(self.orientation('r_foot').T, p)
        return(self.pcr)

    def pos_com_wrt_lfoot(self):
        if self.pcl == None:
            p       = self.pos_com() - self.position('l_foot', 1)
            self.pcl = np.dot(self.orientation('l_foot').T, p)
        return(self.pcl)

    def pos_lfoot_wrt_rfoot(self):
        if self.pl == None:
            p       = self.position('l_foot', 1) - self.position('r_foot', 1)
            self.pl = np.dot(self.orientation('r_foot').T, p)
        return(self.pl)

    def pos_rfoot_wrt_lfoot(self):
        if self.pr == None:
            p       = self.position('r_foot', 1) - self.position('l_foot', 1)
            self.pr = np.dot(self.orientation('l_foot').T, p)
        return self.pr


    def partial_pos_com(self, joint_name):
        '''
        Returns the position of robot Center of Mass with respect to the torso in torso coordinate system
        
        '''
        p = np.zeros(3)
        M = 0.0
        for seg_name in self.all_segments:
            if (joint_name == 'l_hip1') and (seg_name in self.segment['r_foot'].effective_joints):
                p = p + self.segment[seg_name].mass*self.partial_position(seg_name, 'r_hip1')    
            elif (joint_name == 'r_hip1') and (seg_name in self.segment['l_foot'].effective_joints):
                p = p + self.segment[seg_name].mass*self.partial_position(seg_name, 'l_hip1')    
            else:
                p = p + self.segment[seg_name].mass*self.partial_position(seg_name, joint_name)
            M = M + self.segment[seg_name].mass
        return p/M

    def pos_rfoot_jacobian(self):
        '''
        Returns the jacobian of the right foot position wrt the left foot
        the jacobian is a 3x21 matrix. the first three rows refer to the X,Y and Z coordinates of the right foot touch floor wrt left foot touch floor.
        Segment: Right Foot/ Support Foot: Left Foot
        '''
        if not self.rfoot_jacobian_updated:
            J = []
            # joint 0
            p_S_T   = self.position('r_foot', pos_num = 1)
            p_SF_T  = self.position('l_foot', pos_num = 1)
            R_SF_T   = self.orientation('l_foot').T
            dp_S_T  = self.partial_position('r_foot', joint_name = 'r_hip1', pos_num = 1)
            dp_SF_T = self.partial_position('l_foot', joint_name = 'l_hip1', pos_num = 1)
            dR_SF_T = self.partial_orientation('l_foot', joint_name = 'l_hip1').T

            b = np.dot(dR_SF_T, p_S_T - p_SF_T) + np.dot(R_SF_T, dp_S_T - dp_SF_T)
            J = [b]

            # joints 1-5
            for segname in ['r_hip2', 'r_thigh', 'r_shank', 'r_ankle', 'r_foot']:
                dp_S_T  = self.partial_position('r_foot', joint_name = segname, pos_num = 1)
                b = np.dot(R_SF_T, dp_S_T)
                J = np.append(J, [b], axis = 0)

            # joints 6-10
            for segname in ['l_hip2', 'l_thigh', 'l_shank', 'l_ankle', 'l_foot']:
                dp_SF_T = self.partial_position('l_foot', joint_name = segname, pos_num = 1)
                dR_SF_T = self.partial_orientation('l_foot', joint_name = segname).T

                b = np.dot(dR_SF_T, p_S_T - p_SF_T) + np.dot(R_SF_T, - dp_SF_T)
                J = np.append(J, [b], axis = 0)

            # joints 11-20
            for i in range(10):
                J = np.append(J, [np.zeros(3)], axis = 0)
            
            self.JR = J.T
            self.rfoot_jacobian_updated = True

        return self.JR        
                
    def pos_lfoot_jacobian(self):
        '''
        Returns the jacobian of the left foot position wrt the right foot
        the jacobian is a 3x21 matrix. the first three rows refer to the X,Y and Z coordinates of the right foot touch floor wrt left foot touch floor.
        Segment: Left Foot/ Support Foot: Right Foot
        '''
        if not self.lfoot_jacobian_updated:
            J = []
            # joint 0
            p_S_T   = self.position('l_foot', pos_num = 1)
            p_SF_T  = self.position('r_foot', pos_num = 1)
            R_SF_T   = self.orientation('r_foot').T
            dp_S_T  = self.partial_position('l_foot', joint_name = 'l_hip1', pos_num = 1)
            dp_SF_T = self.partial_position('r_foot', joint_name = 'r_hip1', pos_num = 1)
            dR_SF_T = self.partial_orientation('r_foot', joint_name = 'r_hip1').T

            b = np.dot(dR_SF_T, p_S_T - p_SF_T) + np.dot(R_SF_T, dp_S_T - dp_SF_T)
            J = [b]

            # joints 1-5
            for segname in ['r_hip2', 'r_thigh', 'r_shank', 'r_ankle', 'r_foot']:
                dp_SF_T = self.partial_position('r_foot', joint_name = segname, pos_num = 1)
                dR_SF_T = self.partial_orientation('r_foot', joint_name = segname).T

                b = np.dot(dR_SF_T, p_S_T - p_SF_T) + np.dot(R_SF_T, - dp_SF_T)
                J = np.append(J, [b], axis = 0)

            # joints 6-10
            for segname in ['l_hip2', 'l_thigh', 'l_shank', 'l_ankle', 'l_foot']:
                dp_S_T  = self.partial_position('l_foot', joint_name = segname, pos_num = 1)
                b = np.dot(R_SF_T, dp_S_T)
                J = np.append(J, [b], axis = 0)

            # joints 11-20
            for i in range(10):
                J = np.append(J, [np.zeros(3)], axis = 0)
            
            self.JL = J.T
            self.lfoot_jacobian_updated = True

        return self.JL        

    def pos_com_wrt_lfoot_jacobian(self):
        '''
        Returns the jacobian of the com position wrt the left foot
        the jacobian is a 3x21 matrix. the first three rows refer to the X,Y and Z coordinates of the center of mass wrt left foot touch floor.
        Segment: com/ Support Foot: Left Foot
        '''
        if not self.com_l_jacobian_updated:
            J = []
            # joint 0
            p_S_T   = self.pos_com()
            p_SF_T  = self.position('l_foot', pos_num = 1)
            R_SF_T  = self.orientation('l_foot').T
            dp_S_T  = self.partial_pos_com(joint_name = 'l_hip1')
            dp_SF_T = self.partial_position('l_foot', joint_name = 'l_hip1', pos_num = 1)
            dR_SF_T = self.partial_orientation('l_foot', joint_name = 'l_hip1').T

            b = np.dot(dR_SF_T, p_S_T - p_SF_T) + np.dot(R_SF_T, dp_S_T - dp_SF_T)
            J = [b]

            # joints 1-5
            for segname in ['r_hip2', 'r_thigh', 'r_shank', 'r_ankle', 'r_foot']:
                dp_S_T  = self.partial_pos_com(joint_name = segname)
                b = np.dot(R_SF_T, dp_S_T)
                J = np.append(J, [b], axis = 0)

            # joints 6-10
            for segname in ['l_hip2', 'l_thigh', 'l_shank', 'l_ankle', 'l_foot']:
                dp_S_T  = self.partial_pos_com(joint_name = segname)
                dp_SF_T = self.partial_position('l_foot', joint_name = segname, pos_num = 1)
                dR_SF_T = self.partial_orientation('l_foot', joint_name = segname).T

                b = np.dot(dR_SF_T, p_S_T - p_SF_T) + np.dot(R_SF_T, dp_S_T - dp_SF_T)
                J = np.append(J, [b], axis = 0)

            # joints 11-20
            for segname in ['r_shoulder', 'r_upperarm', 'r_elbow', 'r_lowerarm', 'l_shoulder', 'l_upperarm', 'l_elbow', 'l_lowerarm', 'neck', 'head']:
                dp_S_T  = self.partial_pos_com(joint_name = segname)
                b = np.dot(R_SF_T, dp_S_T)
                J = np.append(J, [b], axis = 0)
            
            self.JCL = J.T
            self.com_l_jacobian_updated = True

        return self.JCL

    def pos_com_wrt_rfoot_jacobian(self):
        '''
        Returns the jacobian of the com position wrt the right foot
        the jacobian is a 3x21 matrix. the first three rows refer to the X,Y and Z coordinates of the center of mass wrt right foot touch floor.
        Segment: com/ Support Foot: Right Foot
        '''
        if not self.com_r_jacobian_updated:
            J = []
            # joint 0
            p_S_T   = self.pos_com()
            p_SF_T  = self.position('r_foot', pos_num = 1)
            R_SF_T  = self.orientation('r_foot').T
            dp_S_T  = self.partial_pos_com(joint_name = 'r_hip1')
            dp_SF_T = self.partial_position('r_foot', joint_name = 'r_hip1', pos_num = 1)
            dR_SF_T = self.partial_orientation('r_foot', joint_name = 'r_hip1').T

            b = np.dot(dR_SF_T, p_S_T - p_SF_T) + np.dot(R_SF_T, dp_S_T - dp_SF_T)
            J = [b]

            # joints 1-5
            for segname in ['r_hip2', 'r_thigh', 'r_shank', 'r_ankle', 'r_foot']:
                dp_S_T  = self.partial_pos_com(joint_name = segname)
                dp_SF_T = self.partial_position('r_foot', joint_name = segname, pos_num = 1)
                dR_SF_T = self.partial_orientation('r_foot', joint_name = segname).T

                b = np.dot(dR_SF_T, p_S_T - p_SF_T) + np.dot(R_SF_T, dp_S_T - dp_SF_T)
                J = np.append(J, [b], axis = 0)

            # joints 6-10
            for segname in ['l_hip2', 'l_thigh', 'l_shank', 'l_ankle', 'l_foot']:
                dp_S_T  = self.partial_pos_com(joint_name = segname)
                b = np.dot(R_SF_T, dp_S_T)
                J = np.append(J, [b], axis = 0)

            # joints 11-20
            for segname in ['r_shoulder', 'r_upperarm', 'r_elbow', 'r_lowerarm', 'l_shoulder', 'l_upperarm', 'l_elbow', 'l_lowerarm', 'neck', 'head']:
                dp_S_T  = self.partial_pos_com(joint_name = segname)
                b = np.dot(R_SF_T, dp_S_T)
                J = np.append(J, [b], axis = 0)
            
            self.JCR = J.T
            self.com_r_jacobian_updated = True

        return self.JCR

