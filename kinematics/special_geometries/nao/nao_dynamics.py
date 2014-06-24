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
@version:	    6.0
Last Revision:  25 June 2014

Changes from previous version:

A trajectory is added to the module.
functions step_forward() and run() are added to follow the given trajectory

'''

import numpy as np
import math, copy, time
from sympy import Symbol

import packages.nima.mathematics.vectors_and_matrices as vm
import packages.nima.mathematics.general as gen
import packages.nima.mathematics.rotation as rot
import packages.nima.robotics.kinematics.task_space.trajectory as traj


drc     = math.pi/180.00

dflt_mass = {"torso":1.2171, "neck":0.05, "shoulder": 0.07, "head": 0.35, "upperarm": 0.15, "elbow":0.035, "lowerarm":0.2, "hip1": 0.09, "hip2": 0.125, "thigh": 0.275, "shank":0.225, "ankle":0.125, "foot":0.2}
dflt_dims = {"h19":90.0, "b11":98.0,"h11":75.0,"a2":120.0,"d2":5.0,"h2":40.0,"e2":10.0,"a3":100.0,"h3":45.0,"e3":10.0,"a5":50.0,"h5":40.0,"d5":30.0,"e5":30.0,
                "e14" : 50.0, "e12":20.0,"b12":10.0, "d12":9.0, "a12":90.0, "h20":5.0, "a19":60.0, "e0":10.0, "h0":115.0, "b0":55.0}
dflt_mass_symb = {"torso":Symbol('m0'), "neck":Symbol('m19'), "shoulder": Symbol('m11'), "head" : Symbol('m20'), "upperarm": Symbol('m12'), "elbow":Symbol('m13'), "lowerarm":Symbol('m14'), 
                      "hip1" :Symbol('m1'), "hip2": Symbol('m2'), "thigh"   : Symbol('m3') , "shank": Symbol('m4') , "ankle"   : Symbol('m5'),  "foot" :Symbol('m6')}
dflt_dims_symb = {"h19":Symbol('h19'), "b11":Symbol('b11'),"h11":Symbol('h11'),"a2":Symbol('a2'),"d2":Symbol('d2'),"h2" :Symbol('h2') ,"e2" :Symbol('e2') ,"a3" :Symbol('a3') ,"h3" :Symbol('h3'),
                      "e3" :Symbol('e3') , "a5" :Symbol('a5') ,"h5" :Symbol('h5') ,"d5":Symbol('d5'),"e5":Symbol('e5'),"e14":Symbol('e14'),"e12":Symbol('e12'),"b12":Symbol('b12'),"d12":Symbol('d12'),              "a12":Symbol('a12'), "h20":Symbol('h20'),"a19":Symbol('a19'),"e0":Symbol('e0'),"h0":Symbol('h0'),"b0":Symbol('b0')}

def sum_dic(dic):
    s = 0
    for p in dic:
        s = s + dic[p]    
    return (s)
    
def inner_product(R1 = np.eye(3), R2 = np.eye(3)):
    x = 1000.0*(R1[0,0]*R2[0,0] + R1[1,0]*R2[1,0] + R1[2,0]*R2[2,0])
    z = 1000.0*(R1[0,2]*R2[0,2] + R1[1,2]*R2[1,2] + R1[2,2]*R2[2,2])
    return np.array([x,z])


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

        self.c = math.cos(self.q)
        self.s = math.sin(self.q)

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
        
class NAO(object):
    '''
    The defaults are extracted from NAO model ... :
    '''
    def __init__(self, mass = dflt_mass, dims = dflt_dims, mass_symb = dflt_mass_symb, dims_symb = dflt_dims_symb, symb = False, q0 = np.zeros(21)):
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
        self.M         = sum_dic(self.mass)
        self.mass_symb = mass_symb
        self.dims      = dims
        self.dims_symb = dims_symb
        self.tar_traj_pos_rft_wrt_lft = traj.Trajectory()
        self.tar_traj_pos_lft_wrt_rft = traj.Trajectory()
        self.tar_traj_pos_com_wrt_sft = traj.Trajectory()
        self.tar_traj_ori_rft_wrt_lft = traj.Orientation_Trajectory()
        self.tar_traj_ori_lft_wrt_rft = traj.Orientation_Trajectory()
        self.q   = q0
        self.com = 0.0
        self.all_segments = ['torso','r_hip1', 'r_hip2', 'r_thigh', 'r_shank', 'r_ankle', 'r_foot', 
                                     'l_hip1', 'l_hip2', 'l_thigh', 'l_shank', 'l_ankle', 'l_foot', 
                            'r_shoulder', 'r_upperarm', 'r_elbow', 'r_lowerarm',
                            'l_shoulder', 'l_upperarm', 'l_elbow', 'l_lowerarm', 'neck', 'head']

        self.set_symb(symb)
        self.reset_history()

        self.support_foot  = 'l_foot'
        self.com_respected = False
        self.ori_respected = False
        self.zmp_respected = False
        self.n_com_pel     = 3 # number of center of mass position elements

        
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

        self.q = copy.copy(qd)

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
            J = [b[0:self.n_com_pel]]

            # joints 1-5
            for segname in ['r_hip2', 'r_thigh', 'r_shank', 'r_ankle', 'r_foot']:
                dp_S_T  = self.partial_pos_com(joint_name = segname)
                b = np.dot(R_SF_T, dp_S_T)
                J = np.append(J, [b[0:self.n_com_pel]], axis = 0)

            # joints 6-10
            for segname in ['l_hip2', 'l_thigh', 'l_shank', 'l_ankle', 'l_foot']:
                dp_S_T  = self.partial_pos_com(joint_name = segname)
                dp_SF_T = self.partial_position('l_foot', joint_name = segname, pos_num = 1)
                dR_SF_T = self.partial_orientation('l_foot', joint_name = segname).T

                b = np.dot(dR_SF_T, p_S_T - p_SF_T) + np.dot(R_SF_T, dp_S_T - dp_SF_T)
                J = np.append(J, [b[0:self.n_com_pel]], axis = 0)

            # joints 11-20
            for segname in ['r_shoulder', 'r_upperarm', 'r_elbow', 'r_lowerarm', 'l_shoulder', 'l_upperarm', 'l_elbow', 'l_lowerarm', 'neck', 'head']:
                dp_S_T  = self.partial_pos_com(joint_name = segname)
                b = np.dot(R_SF_T, dp_S_T)
                J = np.append(J, [b[0:self.n_com_pel]], axis = 0)
            
            self.JCL = J.T
            self.com_l_jacobian_updated = True

        return self.JCL

    def pos_zmp_wrt_lfoot(self, a = np.zeros(3)):
        p  = self.pos_com_wrt_lfoot()
        g  = np.array([0.0, 0.0, -9.81])
        F  = self.M*(a+g)
        Mx = F[2]*p[1] - F[1]*p[2]
        My = F[0]*p[2] - F[2]*p[0]
        Mz = F[1]*p[0] - F[0]*p[1]
        return np.array([Mx, My])

    def pos_zmp_wrt_lfoot_jacobian(self, a = np.zeros(3)):
        J = self.pos_com_wrt_lfoot_jacobian()
        g = np.array([0.0, 0.0, -9.81])
        F = self.M*(a+g)
        JMx = F[2]*J[1,:] - F[1]*J[2,:]
        JMy = F[0]*J[2,:] - F[2]*J[0,:]
        JMz = F[1]*J[0,:] - F[0]*J[1,:]
        return np.append([JMx], [JMy], axis = 0)

    def ori_rfoot_wrt_lfoot(self):
        RL = self.orientation('l_foot')
        RR = self.orientation('r_foot')
        return np.dot(RL.T, RR)

    def ori_lfoot_wrt_rfoot(self):
        RL = self.orientation('l_foot')
        RR = self.orientation('r_foot')
        return np.dot(RR.T, RL)

    def ori_err_rfoot(self, Rd = np.eye(3)):
        Ra = self.ori_rfoot_wrt_lfoot()
        return inner_product(Ra, Rd)
        
    def ori_err_lfoot(self, Rd = np.eye(3)):
        Ra = self.ori_lfoot_wrt_rfoot()
        return inner_product(Ra, Rd)
        
    def ori_err_rfoot_jacobian(self, Rd = np.eye(3)):
        J = []
        # joint 0
        RL  = self.orientation('l_foot')
        RR  = self.orientation('r_foot')
        dRL = self.partial_orientation('l_foot', 'l_hip1')
        dRR = self.partial_orientation('r_foot', 'r_hip1')

        dR  = np.dot(dRL.T, RR) + np.dot(RL.T, dRR)
        b   = inner_product(dR, Rd)            
        J   = [b]

        # joints 1-5
        for segname in ['r_hip2', 'r_thigh', 'r_shank', 'r_ankle', 'r_foot']:
            dRR = self.partial_orientation('r_foot', segname)
            dR  = np.dot(RL.T, dRR)
            b   = inner_product(dR, Rd)            
            J   = np.append(J, [b], axis = 0)

        # joints 6-10
        for segname in ['l_hip2', 'l_thigh', 'l_shank', 'l_ankle', 'l_foot']:

            dRL = self.partial_orientation('l_foot', segname)
            dR  = np.dot(dRL.T, RR)
            b   = inner_product(dR, Rd)            
            J   = np.append(J, [b], axis = 0)

        # joints 11-20
        for i in range(10):
            J = np.append(J, [np.zeros(2)], axis = 0)

        return J.T

    def ori_err_lfoot_jacobian(self, Rd = np.eye(3)):
        J = []
        # joint 0
        RL  = self.orientation('l_foot')
        RR  = self.orientation('r_foot')
        dRL = self.partial_orientation('l_foot', 'l_hip1')
        dRR = self.partial_orientation('r_foot', 'r_hip1')

        dR  = np.dot(dRR.T, RL) + np.dot(RR.T, dRL)
        b   = inner_product(dR, Rd)            
        J   = [b]

        # joints 1-5
        for segname in ['r_hip2', 'r_thigh', 'r_shank', 'r_ankle', 'r_foot']:

            dRR = self.partial_orientation('r_foot', segname)
            dR  = np.dot(dRR.T, RL)
            b   = inner_product(dR, Rd)            
            J   = np.append(J, [b], axis = 0)

        # joints 6-10
        for segname in ['l_hip2', 'l_thigh', 'l_shank', 'l_ankle', 'l_foot']:
            dRL = self.partial_orientation('l_foot', segname)
            dR  = np.dot(RR.T, dRL)
            b   = inner_product(dR, Rd)            
            J   = np.append(J, [b], axis = 0)

        # joints 11-20
        for i in range(10):
            J = np.append(J, [np.zeros(2)], axis = 0)

        return J.T

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
            J = [b[0:self.n_com_pel]]

            # joints 1-5
            for segname in ['r_hip2', 'r_thigh', 'r_shank', 'r_ankle', 'r_foot']:
                dp_S_T  = self.partial_pos_com(joint_name = segname)
                dp_SF_T = self.partial_position('r_foot', joint_name = segname, pos_num = 1)
                dR_SF_T = self.partial_orientation('r_foot', joint_name = segname).T

                b = np.dot(dR_SF_T, p_S_T - p_SF_T) + np.dot(R_SF_T, dp_S_T - dp_SF_T)
                J = np.append(J, [b[0:self.n_com_pel]], axis = 0)

            # joints 6-10
            for segname in ['l_hip2', 'l_thigh', 'l_shank', 'l_ankle', 'l_foot']:
                dp_S_T  = self.partial_pos_com(joint_name = segname)
                b = np.dot(R_SF_T, dp_S_T)
                J = np.append(J, [b[0:self.n_com_pel]], axis = 0)

            # joints 11-20
            for segname in ['r_shoulder', 'r_upperarm', 'r_elbow', 'r_lowerarm', 'l_shoulder', 'l_upperarm', 'l_elbow', 'l_lowerarm', 'neck', 'head']:
                dp_S_T  = self.partial_pos_com(joint_name = segname)
                b = np.dot(R_SF_T, dp_S_T)
                J = np.append(J, [b[0:self.n_com_pel]], axis = 0)
            
            self.JCR = J.T
            self.com_r_jacobian_updated = True

        return self.JCR

    def err_vect(self):
        if self.support_foot == 'l_foot':
            xa = self.pos_rfoot_wrt_lfoot()
            xd = self.tar_traj_pos_rft_wrt_lft.current_position
            if self.ori_respected:
                xa = np.append(xa, self.ori_err_rfoot(Rd = self.tar_traj_ori_rft_wrt_lft.current_orientation))
                xd = np.append(xd, np.array([1000.0, 1000.0]))
            if self.com_respected:
                xa = np.append(xa, self.pos_com_wrt_lfoot()[0:self.n_com_pel])
                xd = np.append(xd, self.tar_traj_pos_com_wrt_sft.current_position[0:self.n_com_pel])
            elif self.zmp_respected:
                xa = np.append(xa, self.pos_zmp_wrt_lfoot())
                xd = np.append(xd, np.zeros(2))
        elif self.support_foot == 'r_foot':
            xa = self.pos_lfoot_wrt_rfoot()
            xd = self.tar_traj_pos_lft_wrt_rft.current_position
            if self.ori_respected:
                xa = np.append(xa, self.ori_err_lfoot(Rd = self.tar_traj_ori_lft_wrt_rft.current_orientation))
                xd = np.append(xd, np.array([1000.0, 1000.0]))
            if self.com_respected:
                xa = np.append(xa, self.pos_com_wrt_rfoot()[0:self.n_com_pel])
                xd = np.append(xd, self.tar_traj_pos_com_wrt_sft.current_position[0:self.n_com_pel])
            elif self.zmp_respected:
                xa = np.append(xa, self.pos_zmp_wrt_rft())
                xd = np.append(xd, np.zeros(2))
        else:
            print "Error from err_vect(): support_foot must be either 'r_foot' or 'l_foot'"
            return None
        return xd-xa                        

    def target_velocity(self):
        if self.support_foot == 'l_foot':
            vd = self.tar_traj_pos_rft_wrt_lft.current_velocity
            if self.ori_respected:
                vd = np.append(vd, np.zeros(2))
            if self.com_respected:
                vd = np.append(vd, self.tar_traj_pos_com_wrt_sft.current_velocity[0:self.n_com_pel])
            elif self.zmp_respected:
                vd = np.append(vd, np.zeros(2))
        elif self.support_foot == 'r_foot':
            vd = self.tar_traj_pos_lft_wrt_rft.current_velocity
            if self.ori_respected:
                vd = np.append(vd, np.zeros(2))
            if self.com_respected:
                vd = np.append(vd, self.tar_traj_pos_com_wrt_sft.current_velocity[0:self.n_com_pel])
            elif self.zmp_respected:
                vd = np.append(vd, np.zeros(2))
        else:
            print "Error from err_vect(): support_foot must be either 'r_foot' or 'l_foot'"
            return None
        return vd                        

    def err_norm(self):
        return np.linalg.norm(self.err_vect())

    def err_jacob(self):    
        if self.support_foot == 'l_foot':
            JE = self.pos_rfoot_jacobian()
            if self.ori_respected:
                JE = np.append(JE, self.ori_err_rfoot_jacobian(Rd = self.tar_traj_ori_rft_wrt_lft.current_orientation), axis = 0)
            if self.com_respected:
                JE = np.append(JE, self.pos_com_wrt_lfoot_jacobian(), axis = 0)
            elif self.zmp_respected:
                JE = np.append(JE, self.pos_zmp_wrt_lfoot_jacobian(), axis = 0)
        elif self.support_foot == 'r_foot':
            JE = self.pos_lfoot_jacobian()
            if self.ori_respected:
                JE = np.append(JE, self.ori_err_lfoot_jacobian(Rd = self.tar_traj_ori_lft_wrt_rft.current_orientation), axis = 0)
            if self.com_respected:
                JE = np.append(JE, self.pos_com_wrt_rfoot_jacobian(), axis = 0)
            elif self.zmp_respected:
                JE = np.append(JE, self.pos_zmp_wrt_rfoot_jacobian(), axis = 0)
        else:
            print "Error from err_jacob(): support_foot must be either 'r_foot' or 'l_foot'"
            return None
        return JE

    def in_target(self):
        if self.support_foot == 'l_foot':
            intarg = vm.equal(self.pos_rfoot_wrt_lfoot(), self.tar_traj_pos_rft_wrt_lft.current_position, epsilon = 0.1)
            if self.ori_respected:
                intarg = intarg and vm.equal(self.ori_err_rfoot(Rd = self.tar_traj_ori_rft_wrt_lft.current_orientation),np.array([1000.0, 1000.0]), epsilon = 0.1) 
            if self.com_respected:
                intarg = intarg and vm.equal(self.pos_com_wrt_lfoot()[0:self.n_com_pel],self.tar_traj_pos_com_wrt_sft.current_position[0:self.n_com_pel], epsilon = 1.0)    
            elif self.zmp_respected:
                intarg = intarg and vm.equal(self.pos_zmp_wrt_lfoot(), np.zeros(2), epsilon = 10.0)
        elif self.support_foot == 'r_foot':
            intarg = vm.equal(self.pos_lfoot_wrt_rfoot(), self.tar_traj_pos_lft_wrt_rft.current_position, epsilon = 0.1)
            if self.ori_respected:
                intarg = intarg and vm.equal(self.ori_err_lfoot(Rd = self.tar_traj_ori_lft_wrt_rft.current_orientation),np.array([1000.0, 1000.0]), epsilon = 0.1) 
            if self.com_respected:
                intarg = intarg and vm.equal(self.pos_com_wrt_rfoot()[0:self.n_com_pel],self.tar_traj_pos_com_wrt_sft.current_position[0:self.n_com_pel], epsilon = 1.0)    
            elif self.zmp_respected:
                intarg = intarg and vm.equal(self.pos_zmp_wrt_rfoot(), np.zeros(2), epsilon = 10.0)
        else:
            print "Error from err_jacob(): support_foot must be either 'r_foot' or 'l_foot'"
            return None
        return intarg
    def update_step(self, step_size):
        '''
        Implements an update rule for the joint configuration. This function performs one iteration of the algorithm.
        '''
        # Pose Error is multiplied by step_size
        err = step_size*self.err_vect()
        # Error jacobian is placed in Je
        Je  = self.err_jacob()
        # right pseudo inverse of error jacobian is calculated and placed in Je_dagger
        Je_dagger = np.linalg.pinv(Je)
        # Joint Correction is calculated

        dq = np.dot(Je_dagger, err)

        self.set_config(self.q + dq)

    def self_copy(self):
        nao = NAO()
        nao.set_config(self.q)

        nao.tar_traj_pos_rft_wrt_lft = copy.copy(self.tar_traj_pos_rft_wrt_lft)
        nao.tar_traj_pos_lft_wrt_rft = copy.copy(self.tar_traj_pos_lft_wrt_rft)
        nao.tar_traj_pos_com_wrt_sft = copy.copy(self.tar_traj_pos_com_wrt_sft)
        nao.com_respected = self.com_respected
        nao.ori_respected = self.ori_respected
        nao.support_foot = self.support_foot
        nao.n_com_pel    = self.n_com_pel
        return nao        
    
    def reach_target(self):
        number_of_steps = 20

        # nao = copy.copy(self)

        nao  = self.self_copy()
        nao2 = self.self_copy()
        counter = 0

        not_yet     = not nao2.in_target()
        have_time   = (counter < number_of_steps)
        err_reduced = False

        while not_yet and (have_time or err_reduced):   
            try:
                nao.update_step(step_size = 1.0)
                counter     += 1
            except np.linalg.linalg.LinAlgError:

                print "********************************************************Singular Matrix ***********************************************************";
                break
    
            not_yet     = not nao2.in_target()
            have_time   = (counter < number_of_steps)
            err_reduced = (nao.err_norm() < nao2.err_norm())
    
            if err_reduced:
                nao2.set_config(nao.q)

        assert nao2.in_target()
        self.set_config(nao2.q)

    def joint_speed(self, k = 1.0):

        e  = self.err_vect()
        JA = self.err_jacob()
        K  = k*np.eye(len(e))
        vd = self.target_velocity()

        JA_dag = np.linalg.pinv(JA)
        q_dot  = np.dot(JA_dag, vd + np.dot(K,e)) 

        return q_dot        

    def run(self, duration, k = 1.0, verbose = False):
        t_s    = time.time()        
        t      = 0.0
        self.t = t_s - self.t_s

        while t < duration:
            t0  = t
            if self.com_respected:     
                self.tar_traj_pos_com_wrt_sft.set_phi(t)
            if self.support_foot == 'l_foot':
                self.tar_traj_pos_rft_wrt_lft.set_phi(t)
            elif self.support_foot == 'r_foot':
                self.tar_traj_pos_lft_wrt_rft.set_phi(t)
            else:
                print "Error from run(): support_foot must be either 'r_foot' or 'l_foot'"
                return None

            q_dot  = self.joint_speed(k = k)
            t      = time.time() - t_s
            self.t = t + t_s - self.t_s
            dt     = t - t0
            self.set_config(self.q + dt*q_dot)

            '''
            self.reach_target()
            t     = time.time() - t_s
            '''
    
            self.history_time.append(self.t)
            if self.support_foot == 'l_foot':
                xa = self.pos_rfoot_wrt_lfoot()
                xd = self.tar_traj_pos_rft_wrt_lft.current_position
                if self.ori_respected:
                    xa = np.append(xa, self.ori_err_rfoot())
                    xd = np.append(xd, np.array([1000.0, 1000.0]))
                if self.com_respected:
                    xa = np.append(xa, self.pos_com_wrt_lfoot())
                    xd = np.append(xd, self.tar_traj_pos_com_wrt_sft.current_position)
                elif self.zmp_respected:
                    xa = np.append(xa, self.pos_zmp_wrt_lfoot())
                    xd = np.append(xd, np.zeros(2))
            elif self.support_foot == 'r_foot':
                xa = self.pos_lfoot_wrt_rfoot()
                xd = self.tar_traj_pos_lft_wrt_rft.current_position
                if self.ori_respected:
                    xa = np.append(xa, self.ori_err_lfoot())
                    xd = np.append(xd, np.array([1000.0, 1000.0]))
                if self.com_respected:
                    xa = np.append(xa, self.pos_com_wrt_rfoot())
                    xd = np.append(xd, self.tar_traj_pos_com_wrt_sft.current_position)
                elif self.zmp_respected:
                    xa = np.append(xa, self.pos_zmp_wrt_rfoot())
                    xd = np.append(xd, np.zeros(2))

            self.history_target.append(xd) 
            self.history_actual.append(xa) 
            if verbose:
                print
                print "t = ", t
                print "Target Position = ", xd
                print "Actual Position = ", xa

    def run_ik(self, duration, dt = 0.5, k = 1.0, verbose = False, delay = False):
        t      = 0.0

        while t < duration + 0.001:
            if self.com_respected:     
                self.tar_traj_pos_com_wrt_sft.set_phi(t)
            if self.support_foot == 'l_foot':
                self.tar_traj_pos_rft_wrt_lft.set_phi(t)
            elif self.support_foot == 'r_foot':
                self.tar_traj_pos_lft_wrt_rft.set_phi(t)
            else:
                print "Error from run(): support_foot must be either 'r_foot' or 'l_foot'"
                return None

            if delay:
                '''
                In python 3.x, you should use: input("...")
                '''
                # raw_input("Be careful! I am going to change the joint angles. Press Enter to continue...")
                print("Be careful! I am going to change the joint angles in 5 seconds")
                time.sleep(5.0)
                    
            self.reach_target()
    
            self.history_time.append(self.t)
            if self.support_foot == 'l_foot':
                xa = self.pos_rfoot_wrt_lfoot()
                xd = self.tar_traj_pos_rft_wrt_lft.current_position
                if self.ori_respected:
                    xa = np.append(xa, self.ori_err_rfoot())
                    xd = np.append(xd, np.array([1000.0, 1000.0]))
                if self.com_respected:
                    xa = np.append(xa, self.pos_com_wrt_lfoot())
                    xd = np.append(xd, self.tar_traj_pos_com_wrt_sft.current_position)
                elif self.zmp_respected:
                    xa = np.append(xa, self.pos_zmp_wrt_lfoot())
                    xd = np.append(xd, np.zeros(2))
            elif self.support_foot == 'r_foot':
                xa = self.pos_lfoot_wrt_rfoot()
                xd = self.tar_traj_pos_lft_wrt_rft.current_position
                if self.ori_respected:
                    xa = np.append(xa, self.ori_err_lfoot())
                    xd = np.append(xd, np.array([1000.0, 1000.0]))
                if self.com_respected:
                    xa = np.append(xa, self.pos_com_wrt_rfoot())
                    xd = np.append(xd, self.tar_traj_pos_com_wrt_sft.current_position)
                elif self.zmp_respected:
                    xa = np.append(xa, self.pos_zmp_wrt_rfoot())
                    xd = np.append(xd, np.zeros(2))

            self.history_target.append(xd) 
            self.history_actual.append(xa) 
            if verbose:
                print
                print "t = ", t
                print "Target Position = ", xd
                print "Actual Position = ", xa

            t      = t + dt
            self.t = self.t + dt

    def reset_history(self):
        self.t_s = time.time()
        self.t   = 0.0
        self.history_time   = []
        self.history_target = []
        self.history_actual = []

    '''
    def set_walk_step_trajectory(self, duration, Dx, h):

        phi = [0.0, 0.5*duration, duration]
        if self.support_foot == 'l_foot':
            p_start = self.pos_rfoot_wrt_lfoot()
        elif self.support_foot == 'r_foot':
            p_start = self.pos_lfoot_wrt_rfoot()
       
        p_end = copy.copy(p_start)
        p_end[1] = Dx
        if self.support_foot == 'l_foot':
            p_end[0] = 110.0
        elif self.support_foot == 'r_foot':
            p_end[0] = - 110.0
        

        p_middle = 0.5*(p_start+p_end)
        p_middle[2] = p_middle[2] + h
        pos      = [p_start, p_middle, p_end]

        v_start  = np.zeros(3)
        v_middle = np.array([None,None,None])
        v_end    = np.zeros(3)
        vel      = [v_start, v_middle, v_end]

        a_start  = np.zeros(3)
        a_middle = np.array([None,None,None])
        a_end    = np.zeros(3)
        acc      = [a_start, a_middle, a_end]
    
        if self.support_foot == 'l_foot':
            self.tar_traj_pos_rft_wrt_lft.interpolate(phi = phi, positions = pos, velocities = vel, accelerations = acc)
        elif self.support_foot == 'r_foot':
            self.tar_traj_pos_lft_wrt_rft.interpolate(phi = phi, positions = pos, velocities = vel, accelerations = acc)

        if self.com_respected:
            if self.support_foot == 'l_foot':
                p_s_com = self.pos_com_wrt_lfoot()
            elif self.support_foot == 'r_foot':
                p_s_com = self.pos_com_wrt_rfoot()

            p_e_com = copy.copy(p_s_com)
            p_e_com[1] = 0.5*Dx
            p_e_com[0] = 0.5*p_end[0]

            p_m_com = 0.5*(p_s_com+p_e_com)
            pos      = [p_s_com, p_m_com, p_e_com]

            self.tar_traj_pos_com_wrt_sft.interpolate(phi = phi, positions = pos, velocities = vel, accelerations = acc)
    '''

    def gen_walk_trajectories(self, T, Ts, L, h, b, w):
        # Left foot is the support foot
        self.part0_rft = traj.Path()
        self.part0_com = traj.Path()

        # Left foot is the support foot
        self.part1_rft = traj.Path()
        self.part1_com = traj.Path()

        # Both feet are on the floor (support) but left foot is set as the support foot (Reference)
        self.part2_rft = traj.Path()
        self.part2_com = traj.Path()

        # Right foot is the support foot
        self.part3_lft = traj.Path()
        self.part3_com = traj.Path()

        # Both feet are on the floor (support) but right foot is set as the support foot (Reference)
        self.part4_lft = traj.Path()
        self.part4_com = traj.Path()
        '''
        totally, COM slides by 2*L in time T
        '''    
        t0 = 0.0
        # middle 1: t1/2
        t1 = 0.5*(T - 2*Ts)
        t2 = t1 + Ts
        # middle 2: t2 + t1/2
        t3 = t2 + t1
        t4 = T

        # traj part 0 for right Foot:
        p0    = self.pos_rfoot_wrt_lfoot()        

        p2    = np.zeros(3)
        p2[0] = 110.0
        p2[1] = - L

        v0    = np.zeros(3)
        v2    = np.zeros(3)
            
        phi = [0.0, 1.0]
        p   = [p0, p2]
        v   = [v0, v2]

        self.part0_rft.interpolate(phi, positions = p, velocities = v)

        # traj part 0 for COM:
        p0    = self.pos_com_wrt_lfoot()        

        p2    = np.zeros(3)
        p2[0] =   b/2
        p2[1] = - w/2
        p2[2] = p0[2]

        v2[1] = w/t1

        phi = [0.0, 1.0]
        p   = [p0, p2]
        v   = [v0, v2]
        self.part0_com.interpolate(phi, positions = p, velocities = v)

        # traj part 1 for right Foot: (starting from t0) Right foot moves
        
        p0    = np.array([110.0, - L, 0.0])
        p1    = np.array([110.0, 0.0, h  ])
        p2    = np.array([110.0,   L, 0.0])

        v0    = np.array([0.0, 0.0   , 0.0])
        v1    = np.array([0.0, None  , 0.0])
        v2    = np.array([0.0, 0.0   , 0.0])

        a0    = np.zeros(3)
        a1    = np.array([None, None, None])
        a2    = np.zeros(3)
            
        phi = [t0, t1/2, t1]
        p   = [p0, p1, p2]
        v   = [v0, v1, v2]
        a   = [a0, a1, a2]    
        
        self.part1_rft.interpolate(phi, p, velocities = v, accelerations = a)

        # traj part 1 for COM:  
        self.part0_com.set_phi(1.0)
        p0    = self.part0_com.current_position
        p2    = np.array([b/2, w/2, p0[2]])

        v0    = self.part0_com.current_velocity
        v2    = np.array([None, (L-w)/Ts, None])

        phi = [t0, t1]
        p   = [p0, p2]
        v   = [v0, v2]

        self.part1_com.interpolate(phi, p, velocities = v)

        # traj part 2 for right foot:  (from t1 --> t2) Double Support
        self.part1_rft.set_phi(t1)
        p0    = self.part1_rft.current_position

        phi = [t0]
        p   = [p0]

        self.part2_rft.interpolate(phi, p)

        # traj part 2 for COM:
        self.part1_com.set_phi(t1)
        p0    = self.part1_com.current_position
        p2    = np.array([110.0 - b/2, L - w/2, p0[2]])

        v0    = self.part1_com.current_velocity
        v2    = np.array([0.0, w/t1, None])

        phi = [t0, Ts]
        p   = [p0, p2]
        v   = [v0, v2]

        self.part2_com.interpolate(phi, p, velocities = v)

        # traj part 3 for left Foot:  (from t2 --> t3) Left leg moves
        
        p0    = np.array([-110.0, - L, 0.0])
        p1    = np.array([-110.0, 0.0, h  ])
        p2    = np.array([-110.0,   L, 0.0])

        v0    = np.array([0.0, 0.0   , 0.0])
        v1    = np.array([0.0, None  , 0.0])
        v2    = np.array([0.0, 0.0   , 0.0])

        a0    = np.zeros(3)
        a1    = np.array([None, None, None])
        a2    = np.zeros(3)
            
        phi = [t0, t1/2, t1]
        p   = [p0, p1, p2]
        v   = [v0, v1, v2]
        a   = [a0, a1, a2]    
        
        self.part3_lft.interpolate(phi, p, velocities = v, accelerations = a)

        # traj part 3 for COM:
        self.part2_com.set_phi(Ts)
        p0    = self.part2_com.current_position
        p0[0] = -b/2
        p0[1] = -w/2

        p2    = copy.copy(p0)
        p2[1] = w/2

        v0  = self.part2_com.current_velocity
        v2    = np.array([0.0, (L-w)/Ts, None])  # prepares speed for the next stage
        
        phi = [t0, t1]
        p   = [p0, p2]
        v   = [v0, v2]

        self.part3_com.interpolate(phi, p, velocities = v)

        # traj part 4 for left Foot:  (from t3 --> t4) Double Support
        
        self.part3_lft.set_phi(t1)
        p0    = self.part3_lft.current_position

        phi = [t0]
        p   = [p0]
        
        self.part4_lft.interpolate(phi, p)

        # traj part 4 for COM:
        self.part3_com.set_phi(t1)
        p0    = self.part3_com.current_position
        p2    = np.array([- 110.0 + b/2, L - w/2, p0[2]])

        v0    = self.part3_com.current_velocity
        v2    = np.array([0.0, w/t1, None])

        phi = [t0, Ts]
        p   = [p0, p2]
        v   = [v0, v2]

        self.part4_com.interpolate(phi, p, velocities = v)

    def change_support_foot(self):
        if self.support_foot == 'l_foot':
            self.support_foot == 'r_foot'            
        elif self.support_foot == 'r_foot':
            self.support_foot == 'l_foot'            

            

