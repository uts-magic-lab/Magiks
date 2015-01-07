## @file        	PR2_symbolics.py
#  @brief     		Contains functions to represent symbolic formulations of pr2 kinematics
#  @author      	Nima Ramezani Taghiabadi 
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. :   04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	2.0
# 
#  Last Revision:  	03 January 2015 
 
from sympy import Symbol, simplify

class PR2_Arm_Symbolic():
    '''
    This class provides a parametric representation of the kinematics of PR2 arm
    The position, orientation and the Jacobian can be expressed in terms of DH parameters and sine and cosine of the joint angles
    It uses sympy as a tool for symbols algebra
    '''
    def __init__(self):

        n = 7

        c = [Symbol('c' + str(i)) for i in range(n)]
        s = [Symbol('s' + str(i)) for i in range(n)]
        a = [Symbol('a0'), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        d = [0.0, 0.0, Symbol('d2'), 0.0, Symbol('d4'), 0.0, 0.0]
        alpha = [- math.pi/2, math.pi/2, - math.pi/2, math.pi/2, - math.pi/2, math.pi/2, 0.0]    
        
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

        # T[i] transfer matrix from link -1(ground) to i
        self.T = numpy.copy(self.A)
        for i in range(n-1):
            self.T[i + 1] = numpy.dot(self.T[i],self.A[i + 1])

        # Desired Pose:
        T_d = numpy.array([[ Symbol('r11'), Symbol('r12') , Symbol('r13') , Symbol('px') ], 
                           [ Symbol('r21'), Symbol('r22') , Symbol('r23') , Symbol('py') ],
                           [ Symbol('r31'), Symbol('r32') , Symbol('r33') , Symbol('pz') ],
                           [ 0           , 0            ,  0           , 1               ]])

        self.x_d = T_d[0:3, 3]
        self.R_d = T_d[0:3, 0:3]

        '''
        "p_W" denotes the position vector of the wrist joint center
        "R_W" denotes the orientation of the gripper (Endeffector Orientation) 
        '''
        self.T_W = self.T[6]
        self.p_W = self.T_W[0:3, 3]
        self.R_W = self.T_W[0:3, 0:3]

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

        self.div_theta_err = sympy.Matrix([
        [ 0 , 0 , -2*d[2]*d[4]*s[3] , 0 , 0 , 0   ],
        [0 , - 2*c[2]*s[2]*s[3]**2 , Symbol('e23') , 0 , 0 , 0   ],
        [Symbol('e31') , d[4]*Symbol('s321') , Symbol('e33') , 0 , 0 , 0   ],
        [Symbol('e41') , Symbol('e42') , Symbol('e43') , 0 , - s[5] , 0   ],
        [Symbol('e51') , Symbol('e52') , 0 , c[4]*s[5] , c[5]*s[4] , 0   ],
        [Symbol('e61') , Symbol('e62') , Symbol('e63') , 0 , Symbol('c65') , - Symbol('s65') ]]) 

        self.div_phi_err = sympy.Matrix([Symbol('e10') , 0 , 0 , Symbol('e40') , Symbol('e50') , Symbol('e60')   ])

        e = self.div_theta_err
        J3 = - Symbol('e10')/e[0,2] 
        J2 = - e[1,2]*J3/e[1,1]
        J1 = - (e[2,1]*J2 + e[2,2]*J3)/e[2,0]
        J5 = - (Symbol('e40') + e[3,0]*J1 + e[3,1]*J2 + e[3,2]*J3)/e[3,4]
        J4 = - (Symbol('e50') + e[4,0]*J1 + e[4,1]*J2 + e[4,4]*J5)/e[4,3]
        J6 = - (Symbol('e60') + e[5,0]*J1 + e[5,1]*J2 + e[5,2]*J3 + e[5,3]*J4 + e[5,4]*J5)/ e[5,5]

        self.RJ = sympy.Matrix([J1,J2,J3,J4,J5,J6])
        #self.redundancy_jacobian = self.div_theta_err.inv().transpose()*self.div_phi_err

class PR2_Symbolic():
    '''
    This class provides a parametric representation of the kinematics of PR2 robot
    The position, orientation and the Jacobian can be expressed in terms of the kinematic parameters of the robot
    It uses sympy as a tool for symbols algebra
    '''
    def __init__(self):

        c = [Symbol('c' + str(i)) for i in range(7)]
        s = [Symbol('s' + str(i)) for i in range(7)]
        S = [Symbol('S' + str(i)) for i in range(7)]
        C = [Symbol('C' + str(i)) for i in range(7)]
        phi = [Symbol('p' + str(i)) for i in range(5)]
        
        a = [Symbol('a0'), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        d = [0.0, 0.0, Symbol('d2'), 0.0, Symbol('d4'), 0.0, 0.0]

        self.right_arm = armlib.PR2_Arm_Symbolic()
                
        '''
        we compromize that capital letters are used for the base
        so X,Y,Z represent the position of the base
        and C,S represent sin(theta) and cos(theta) where theta is the rotation of base around z axis 
        '''
        self.p_EFR_WR = numpy.array([0.0  ,  0.0 , Symbol('d7')])
        self.p_BO    = numpy.array([Symbol('X')  ,  Symbol('Y') , Symbol('Z')])
        self.p_BR_BO = numpy.array([Symbol('b0') ,  Symbol('l0'), Symbol('h')])        
        self.p_BL_BO = numpy.array([Symbol('b0') ,- Symbol('l0'), Symbol('h')])     
        self.R_B     = numpy.array([[ Symbol('C'),  Symbol('S') , 0.0 ], 
                                    [-Symbol('S'),  Symbol('C') , 0.0 ],
                                    [ 0.0        ,  0.0         , 1.0]]) 
        '''
        RRd and LRd represent the desired orientation of the right and left arm grippers respectively
        '''

        T_EFR = numpy.array([[ Symbol('nx'), Symbol('sx') , Symbol('ax') , Symbol('xd') ], 
                             [ Symbol('ny'), Symbol('sy') , Symbol('ay') , Symbol('yd') ],
                             [ Symbol('nz'), Symbol('sz') , Symbol('az') , Symbol('zd') ],
                             [ 0.0           , 0.0            , 0.0            , 1.0           ]])

        self.p_EFR = T_EFR[0:3, 3]
        self.R_EFR = T_EFR[0:3, 0:3]

        self.p_WR_BR = - self.p_BR_BO + numpy.dot(self.R_B.T,(self.p_EFR - self.p_BO - numpy.dot(self.R_EFR, self.right_arm.p_EF_W)))
        self.R_WR_B  = numpy.dot(self.R_B.T, self.R_EFR)

        import sympy

        self.div_theta_e = sympy.Matrix([
        [0 , 0 , -2*Symbol('d42')*s[3] , 0 , 0 , 0 , 0 , 0 , 0  ],
        [0 , Symbol('e22') , Symbol('e23') , 0 , 0 , 0 , 0 , 0 , 0  ],
        [Symbol('e31') , d[4]*Symbol('s321') , Symbol('e33') , 0 , 0 , 0 , 1 , 0 , 0  ],
        [Symbol('e41') , Symbol('e42') , Symbol('e43') , 0 , - s[5] , 0  , 0 , 0 , 0 ],
        [Symbol('e51') , Symbol('e52') , 0 , c[4]*s[5] , c[5]*s[4] , 0  , 0 , 0 , 0 ],
        [Symbol('e61') , Symbol('e62') , Symbol('e63') , 0 , Symbol('c65') , - Symbol('s65')  , 0 , 0 , 0 ],
        [0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 0  ],
        [0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 0  ],
        [0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1   ]])
                
        self.div_phi_e = sympy.Matrix([
        [ Symbol('z11') , Symbol('z12') , -2*phi[3]     , Symbol('z14')     , 0   ],
        [ 0             , Symbol('z22') , Symbol('z23') , 0                 , 0   ],
        [ 0             , 0             , 0             , 0                 , 0   ],
        [ Symbol('z41') , 0             , 0             , 0                 , - Symbol('z41')   ],
        [ Symbol('z51') , 0             , 0             , 0                 , - Symbol('z51')    ],
        [ Symbol('z61') , 0             , 0             , 0                 , - Symbol('z61')    ],
        [ 0             , 0             , 1             , 0                 , 0   ],
        [ 0             , Symbol('z82') , 0             , Symbol('z84')     , Symbol('z85')  ],
        [ 0             , Symbol('z92') , 0             , Symbol('z94')     , Symbol('z95')  ]])

        self.redundancy_jacobian = sympy.Matrix([
        [Symbol('J' + str(j+1) + str(i+1)) for i in range(5)] for j in range(9)])

