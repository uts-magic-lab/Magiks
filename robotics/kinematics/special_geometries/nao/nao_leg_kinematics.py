'''   Header
@file:          NAO_leg_kinematics.py
@brief:    	    Contains specific functions that define all geometric and kinematic parameters for nao robot right and left legs

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
Last Revision:  26 May 2014

Changes from previous version:
After forward kinematics was finalized, we developed the unit for inverse kinematics functions

'''
import sympy, copy
import packages.nima.mathematics.general as gen
from sympy import Symbol, simplify
from interval import interval, inf, imath

'''
The most convenient way to install "pyinterval" library is by means of easy_install:
sudo easy_install pyinterval
Alternatively, it is possible to download the sources from PyPI and invoking
python setup.py install
from sets import Set
Please refer to:
http://pyinterval.googlecode.com/svn-history/r25/trunk/html/index.html
'''
import numpy, math
import packages.nima.robotics.kinematics.kinematicpy.general as genkinlib
import packages.nima.robotics.kinematics.joint_space.configuration as configlib
import packages.nima.mathematics.general as genlib
import packages.nima.mathematics.trigonometry as triglib
import packages.nima.mathematics.rotation as rotlib
import packages.nima.mathematics.vectors_and_matrices as vecmatlib

drc     = math.pi/180.00

class NAO_Leg():
    '''
    '''
    def __init__(self, tibia_length = 102.9, thigh_length = 100.0, hip_offset = [0.0, 50.0, - 85.0], foot_offset = [0.0, 0.0, -45.19]):
        '''
        The defaults are extracted from NAO model ... :
        link
        '''
        self.tibia_length = tibia_length
        self.thigh_length = thigh_length
        self.foot_offset  = foot_offset
        self.hip_offset   = hip_offset # Specifies the offset of hip JC(Joint Center) w.r.t. the torso in torso CS(Coordinate System)
        # DH parameters:
        self.a = [0.0,  0.0 ,  0.0, thigh_length, tibia_length, 0.0]
        self.d = [0.0,  0.0 ,  0.0, 0.0         , 0.0         , 0.0]

        self.theta = [    math.pi/2,  - math.pi/4, 0.0      , 0.0, 0.0,   0.0      ]
        self.alpha = [  - math.pi/4,  math.pi/2, math.pi/2, 0.0, 0.0, math.pi/2]    

        self.fk_updated = False
        self.q = numpy.zeros(7)        
        
    def update_fk(self):
        '''
        updates the forward kinematics according to the current joint values    
        '''
        if not self.fk_updated:
            n = 6

            # A_0B: translation_matrix from torso center(base) to the hip joint center(origin of frame 0)
            A_0B = rotlib.trans_hemogeneous(self.hip_offset)
            # A_E6: translation_matrix from Ankle joint center(origin frame 6) to the Endeffector (Bottom of foot)
            A_E6 = rotlib.trans_hemogeneous(self.foot_offset)

            self.c = [math.cos(self.q[i]+ self.theta[i]) for i in range(n)]
            self.s = [math.sin(self.q[i]+ self.theta[i]) for i in range(n)]

            self.ca    = [math.cos(self.alpha[i]) for i in range(n)]
            self.sa    = [math.sin(self.alpha[i]) for i in range(n)]

            # A[i] transfer matrix from link i-1 to i  according to standard DH parameters:

            self.A = [numpy.zeros((4,4)) for i in range(n)]

            for i in range(n):
                Rx_alpha = rotlib.rot_x(self.alpha[i], hemogeneous = True) 
                Tx_a     = rotlib.trans_hemogeneous([self.a[i], 0.0, 0.0]) 
                Ty_d     = rotlib.trans_hemogeneous([0.0, self.d[i], 0.0 ]) 
                Rz_theta = rotlib.rot_z(self.q[i]+ self.theta[i], hemogeneous = True) 
                self.A[i]     = numpy.dot(numpy.dot(Rx_alpha, Tx_a), numpy.dot(Rz_theta, Ty_d))
                

            # T[i] transfer matrix from link -1(Torso) to i
            self.T = numpy.copy(self.A)
            for i in range(n-1):
                self.T[i + 1] = numpy.dot(self.T[i],self.A[i + 1])


            Rz   = rotlib.rot_z(  math.pi  , hemogeneous = True)
            Ry   = rotlib.rot_y(  math.pi/2  , hemogeneous = True)
            R    = numpy.dot(Rz, Ry)
            RAE6 = numpy.dot(R, A_E6)
            A0BT = numpy.dot(A_0B, self.T[5])

            # Transfer matrix of the Endeffector(foot bottom) with respect to the base(torso center):
            self.TEB = numpy.dot(A0BT, RAE6) 
            
            self.fk_updated = True
        

    def ankle_position(self):
        '''
        FK function. Returns the ankle pose w.r.t. the torso
        '''
        if not self.fk_updated:
            self.update_fk()
        return self.TEB[0:3, 3]
    
    def ankle_orientation(self):
        '''
        FK function. Returns the ankle rotation matrix w.r.t. the torso
        '''
        if not self.fk_updated:
            self.update_fk()
        return self.TEB[0:3, 0:3]

    


