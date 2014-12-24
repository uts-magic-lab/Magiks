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



"""
class NAO_Leg_Symbolic():
    '''
    This class provides a parametric representation of the kinematics of PR2 arm
    The position, orientation and the Jacobian can be expressed in terms of DH parameters and sine and cosine of the joint angles
    It uses sympy as a tool for symbols algebra
    '''
    def __init__(self):

        n = 6

        c = [Symbol('c' + str(i)) for i in range(n)]
        s = [Symbol('s' + str(i)) for i in range(n)]
        a = [0.0, 0.0, 0.0, Symbol('a3'), Symbol('a4'), 0.0]
        d = [0.0, 0.0, 0.0, 0.0         , 0.0         , 0.0]
        alpha = [- 3*math.pi/4, - math.pi/2, math.pi/2, 0.0, 0, - math.pi/2]    
        
        ca    = [math.cos(alpha[i]) for i in range(n)]
        sa    = [math.sin(alpha[i]) for i in range(n)]

        for i in range(len(ca)):
            ca[i] = gen.round(ca[i])
            sa[i] = gen.round(sa[i])
        
        # A[i] transfer matrix from link i-1 to i  according to standard DH parameters:
        self.A = [numpy.array([[ c[i],  -ca[i]*s[i],  sa[i]*s[i], a[i]*c[i] ], 
                               [ s[i],   ca[i]*c[i], -sa[i]*c[i], a[i]*s[i] ],
                               [ 0   ,   sa[i]     ,  ca[i]     , d[i]      ],
                               [ 0   ,   0         ,  0         , 1         ]]) for i in range(n)]
                               
        # B[i] transfer matrix from link i to i-1 or B[i] = inv(A[i])
        self.B = [numpy.array([[        c[i],         s[i], 0     ,  0          ], 
                               [ -ca[i]*s[i],   ca[i]*c[i], sa[i] , -d[i]*sa[i] ],
                               [  ca[i]*s[i],  -sa[i]*c[i], ca[i] , -d[i]*ca[i] ],
                               [  0         ,   0         , 0     ,  1          ]]) for i in range(n)]

        # R[i] rotation matrix from link i-1 to i  according to standard DH parameters:
        self.R = [numpy.array([[ c[i],  -ca[i]*s[i],  sa[i]*s[i]], 
                               [ s[i],   ca[i]*c[i], -sa[i]*c[i]],
                               [ 0   ,   sa[i]     ,  ca[i]     ]]) for i in range(n)]

        # T[i] transfer matrix from link -1(Torso) to i
        self.T = numpy.copy(self.A)
        for i in range(n-1):
            self.T[i + 1] = numpy.dot(self.T[i],self.A[i + 1])

        # Desired Pose:
        T_d = numpy.array([[ Symbol('r11'), Symbol('r12') , Symbol('r13') , Symbol('px') ], 
                           [ Symbol('r21'), Symbol('r22') , Symbol('r23') , Symbol('py') ],
                           [ Symbol('r31'), Symbol('r32') , Symbol('r33') , Symbol('pz') ],
                           [ 0           , 0            ,  0           , 1            ]])

        self.x_d = T_d[0:3, 3]
        self.R_d = T_d[0:3, 0:3]



        self.T_F = self.T[5]
        self.p_F = self.T_F[0:3, 3]
        self.R_F = self.T_F[0:3, 0:3]

        '''
        "p_EF_W" denotes the position of the arm endeffector (end of the gripper) relative to the wrist joint center 
        in the coordinates system of the right gripper.
        '''

        self.p_EF_W = numpy.array([0.0, 0.0, Symbol('d7')])
    
        self.H = numpy.copy(self.T)
        self.H[n - 1] = numpy.copy(T_d)
        
        # H[i] transfer matrix from link -1(ground) to i calculated from inverse transform
        for i in range(n-1):
            self.H[n - i - 2] = numpy.dot(self.H[n - i - 1], self.B[n - i - 1])
"""
