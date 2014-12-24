'''   Header
@file:          NAO_leg_symbolics.py
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
@version:	    1.0
Last Revision:  24 March 2014


'''

import sympy, math, numpy

import packages.nima.mathematics.general as gen
import packages.nima.mathematics.rotation as rotlib

from sympy import Symbol, simplify


class NAO_Leg_Symbolic():
    '''
    This class provides a parametric representation of the kinematics of NAO leg
    The position, orientation and the Jacobian can be expressed in terms of DH parameters and sine and cosine of the joint angles
    It uses sympy as a tool for symbols algebra
    '''
    def __init__(self):

        n = 6

        self.c = [Symbol('c' + str(i)) for i in range(n)]
        self.s = [Symbol('s' + str(i)) for i in range(n)]
        self.a = [0, 0, 0, Symbol('a3'), Symbol('a4'), 0]
        self.d = [0, 0, 0, 0         , 0         , 0]
        self.alpha = [  - math.pi/4,  math.pi/2, math.pi/2, 0, 0, - math.pi/2]  

        # Because theta[0] is pi/2 in DH parameters:
        self.c[0] = - Symbol('s0')  
        self.s[0] =   Symbol('c0')

        # Because theta[1] is -pi/4 in DH parameters:
        #self.c[1] = 0.5*math.sqrt(2)*(Symbol('s1') + Symbol('c1'))
        #self.c[1] = 0.5*math.sqrt(2)*(Symbol('s1') - Symbol('c1'))
        
        ca    = [math.cos(self.alpha[i]) for i in range(n)]
        sa    = [math.sin(self.alpha[i]) for i in range(n)]

        for i in range(len(ca)):
            ca[i] = gen.round(ca[i])
            sa[i] = gen.round(sa[i])
        
        # A[i] transfer matrix from link i-1 to i  according to standard DH parameters:

        self.A = [numpy.zeros((4,4)) for i in range(n)]

        for i in range(n):
            Rx_alpha = numpy.array([[ 1 ,  0     , 0     , 0 ], 
                                   [  0 ,  ca[i] , sa[i] , 0 ],
                                   [  0 , -sa[i] , ca[i] , 0 ],
                                   [  0 ,  0     , 0     , 1 ]]) 
            Tx_a     = rotlib.trans_hemogeneous([self.a[i], 0, 0]) 
            Ty_d     = rotlib.trans_hemogeneous([0, self.d[i], 0 ]) 
            Rz_theta = rotlib.rot_z(0, hemogeneous = True, symbolic = True, s = self.s[i], c = self.c[i]) 

            self.A[i]     = numpy.dot(numpy.dot(Rx_alpha, Tx_a), numpy.dot(Rz_theta, Ty_d))

            # T[i] transfer matrix from link -1(Torso) to i
            self.T = numpy.copy(self.A)
            for i in range(n-1):
                self.T[i + 1] = numpy.dot(self.T[i],self.A[i + 1])


            hip_offset  = [0.0,   Symbol('h'),    0.0]
            foot_offset = [0.0, 0.0, 0.0]

            # A_0B: translation_matrix from torso center(base) to the hip joint center(origin of frame 0)
            A_0B = rotlib.trans_hemogeneous(hip_offset)
            # A_E6: translation_matrix from Ankle joint center(origin frame 6) to the Endeffector (Bottom of foot)
            A_E6 = rotlib.trans_hemogeneous(foot_offset)

            self.T[5][0,3] = Symbol('x')
            self.T[5][1,3] = Symbol('y')
            self.T[5][2,3] = Symbol('z')

            Rz   = rotlib.rot_z(  math.pi  , hemogeneous = True)
            Ry   = rotlib.rot_y(  math.pi/2  , hemogeneous = True)
            R    = numpy.dot(Rz, Ry)
            RAE6 = numpy.dot(R, A_E6)
            A0BT = numpy.dot(A_0B, self.T[5])

            # Transfer matrix of the Endeffector(foot bottom) with respect to the base(torso center):
            self.TEB = numpy.dot(A0BT, RAE6) 

  
        '''
        x = a3*c0*s2 + 0.7071*a3*c2*s0*(c1 - s1) + a4*(c3*(c0*s2 + 0.7071*c2*s0*(c1 - s1)) + s3*(c0*c2 - 0.7071*s0*s2*(c1 - s1)))
        '''      

            
