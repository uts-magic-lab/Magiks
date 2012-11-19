'''   Header
@file:          PR2_kinematics.py
@brief:    	    Contains specific functions that define all geometric and kinematic parameters for PR2 robot

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
@version:	    0.7
Last Revision:  20 November 2012
References:     
                [1] Jabref Key: 2008_arti_Shimizu.Kakuya.ea_Analyticala
'''

import __init__
__init__.set_file_path( False )


from sympy import Symbol, simplify
import numpy, math
import packages.nima.robotics.kinematics.kinematicpy.manipulator_library as mnlib
import packages.nima.robotics.kinematics.kinematicpy.inverse_kinematics  as iklib 
import packages.nima.mathematics.general as gen
import packages.nima.mathematics.trigonometry as trig
import packages.nima.mathematics.vectors_and_matrices as vecmat


epsilon = 0.0000001
drc     = math.pi/180.00


def link_T(theta,alpha,a,d):

    T_theta = numpy.array([ [ math.cos(theta), -math.sin(theta), 0.0, 0.0 ],
                       [ math.sin(theta),  math.cos(theta), 0.0, 0.0 ],
                       [    0.0,             0.0,     1.0, 0.0 ],
                       [    0.0,             0.0,     0.0, 1.0 ] ])

    T_alpha = numpy.array([ [ 1.0,     0.0,            0.0,     0.0 ],
                       [ 0.0, math.cos(alpha), -math.sin(alpha), 0.0 ],
                       [ 0.0, math.sin(alpha),  math.cos(alpha), 0.0 ],
                       [ 0.0,     0.0,            0.0,     1.0 ] ])

    T_trans = numpy.array([ [ 1.0, 0.0, 0.0, a ],
                       [ 0.0, 1.0, 0.0, 0.0 ],
                       [ 0.0, 0.0, 1.0, d ],
                       [ 0.0, 0.0, 0.0, 1.0 ] ])

    return numpy.dot(numpy.dot(T_theta, T_trans), T_alpha)


def round(x):
    
    if abs(x) < epsilon:
        y = 0
    elif abs(x - 1) < epsilon:
        y = 1
    elif abs(x + 1) < epsilon:
        y = -1
    else:
        y = x    
    return y    

def assert_equality(x,y):
    '''
    checks if x and y are equal removing the machine error
    '''
    assert (abs(x-y) < epsilon)


def transfer_DH_standard(theta, alpha, a, d, q):
    s  = math.sin(theta + q)
    c  = math.cos(theta + q)
    sa = math.sin(alpha)
    ca = math.cos(alpha)
    
    return numpy.array([[  c, -ca*s,  sa*s, a*c ],
                        [  s,  ca*c, -sa*c, a*s ],
                        [  0,  sa  ,  ca  , d   ],
                        [  0,  0   ,  0   , 1   ]])

def rot_z(q):
    # Return a transfer matrix corresponding to rotation around z axis
    c = math.cos(q)
    s = math.sin(q)
    return numpy.array([[  c, -s, 0, 0 ],
                        [  s,  c, 0, 0 ],
                        [  0,  0, 1, 0 ],
                        [  0,  0, 0, 1 ]])





class PR2_ARM():

            
    def forward_update(self):        
        '''
        This function, finds all kinematics of the PR2 Arm directly from the formulations
        '''    

        self.c = [math.cos(self.qr[i]) for i in range(0,7)]
        self.s = [math.sin(self.qr[i]) for i in range(0,7)]

        [s0, s1, s2, s3, s4, s5, s6] = self.s
        [c0, c1, c2, c3, c4, c5, c6] = self.c

        [s10, s20, s30] = s0*numpy.array([s1, s2, s3])
        [c10, c20, c30] = c0*numpy.array([c1, c2, c3])
        
        [c1s10, c1s20, c1s30] = c1*numpy.array([s10, s20, s30])

        b = numpy.array([s1, c1])
        [s21, c1s2] = s2*b
        [c2s1, c21] = c2*b
        [s31, c1s3] = s3*b
        [c3s1, c31] = c3*b

        [c21s0, c21s1, c21s2, c21s3] = c21*numpy.array([s0, s1, s2, s3])

        [s31, s32]    = s3*numpy.array([s1, s2])
        [c31, c32]    = c3*numpy.array([c1, c2])

        [c0s0, c0s1, c0s2, c0s3, c210] = c0*numpy.array([s0, s1, s2, s3, c21])
        [c0s30, c0s31, c0s32] = c0*numpy.array([s30, s31, s32])

        b = numpy.array([c10, c20, c30])
        [c10s0, c20s0, c30s0]  = s0*b
        [c10s1, c20s1, c30s1]  = s1*b
        [c10s2, c20s2, c30s2]  = s2*b
        [c10s3, c20s3, c30s3]  = s3*b

        c30s1  = c30*s1
        c210s3 = c2*c10s3
        c21s30  = c21*s30
        [c2s31, c2s0]  = c2*numpy.array([s31,s0])
        s320   = s3*s20
        c3s10  = c3*s10


        X =  c0*self.a0 + c0s1*self.d2 + (c30s1 + c210s3 - s320)*self.d4

        Y =  s0*self.a0 + s10*self.d2 + (c3s10 + c0s32 + c21s30)*self.d4

        Z =  c1*self.d2 + (c31 - c2s31)*self.d4
        
        self.end_right_arm_position = [X, Y, Z]

         
        R03 = numpy.zeros((3,3))      

        R03[0,0] =  -s20 + c210
        R03[0,1] =  -c0s1
        R03[0,2] =  -c2s0 - c10s2

        R03[1,0] =  c0s2 + c21s0
        R03[1,1] =  -s10
        R03[1,2] =  c20 - c1s20

        R03[2,0] =  -c2s1
        R03[2,1] =  -c1
        R03[2,2] =  s21

        s54 = s5*s4
        [c54, c5s4, c5s5] = c5*numpy.array([c4, s4, s5])

        [s64, s65, c4s6, c5s6, c54s6, s654] = s6*numpy.array([s4, s5, c4, c5, c54, s54])
        [c64, c65, c6s4, c6s5, c654, C6s54] = s6*numpy.array([s4, s5, c4, c5, c54, s54])

        R47 = numpy.zeros((3,3))      

        R47[0,0] =  -s64 + c654
        R47[0,1] =  -c6s4 - c54s6
        R47[0,2] =  c4*s5

        R47[1,0] =  c4s6 + c65*s4
        R47[1,1] =  c64 - c5*s64
        R47[1,2] =  s54

        R47[2,0] =  -c6s5
        R47[2,1] =  s65
        R47[2,2] =  c5
        
        R34 =  numpy.array([[  c3,     0,     s3 ],
                            [  s3,     0,    -c3 ],
                            [  0,      1,     0  ]])

        self.end_right_arm_orientation = numpy.dot(numpy.dot(R03, R34), R47)

        """
        self.b = [s0, c0, c0s0]
        '[s0, c0, c0s0] = b[0:3]'
    
        bb = s1*self.b
        [s10, c0s1, c0s10] = bb
        self.b = self.b + bb
        #[s10, c0s1, c0s10] = b[3:6]

        bb = c1*self.b
        [c1s0, c10, c10s0, c1s10, c10s1, c10s10] = bb
        self.b = self.b + bb
        #[c1s0, c10, c10s0, c1s10, c10s1, c10s10] = b[6:12]

        bb = s2*self.b
        [s20, c0s2, c0s20, s210, c0s21, c0s210] = bb[0:6]
        [c1s20, c10s2, c10s20, c1s210, c10s21, c10s210] = bb[6:12]
        self.b = self.b + bb
        #[s20, c0s2, c0s20, s210, c0s21, c0s210] = b[12:18]
        #[c1s20, c10s2, c10s20, c1s210, c10s21, c10s210] = b[18:24]

        bb = c2*self.b
        [c2s0, c20, c20s0] = bb[0:3]
        [c2s10, c20s1, c20s10] = bb[3:6]
        [c21s0, c210, c210s0, c21s10, c210s1, c210s10] = bb[6:12]
        [c2s20, c20s2, c20s20, c2s210, c20s21, c20s210] = bb[12:18]
        [c21s20, c210s2, c210s20, c21s210, c210s21, c210s210] = bb[18:24]

        self.b = self.b + bb
        """






    def forward_update_slow(self):        
        self.A = []
        '''        
        self.A.append(transfer_DH_standard( 0.0       , - math.pi/2, 0.0, 0.0       ,   self.qr[0]))
        self.A.append(transfer_DH_standard( 0.0       ,   math.pi/2, 0.0, 0.0       ,   self.qr[1]))
        self.A.append(transfer_DH_standard( 0.0       , - math.pi/2, 0.0, self.d2   ,   self.qr[2]))
        self.A.append(transfer_DH_standard( 0.0       ,   math.pi/2, 0.0, 0.0       ,   self.qr[3]))
        self.A.append(transfer_DH_standard( 0.0       , - math.pi/2, 0.0, self.d4   ,   self.qr[4]))
        self.A.append(transfer_DH_standard( 0.0       ,   math.pi/2, 0.0, 0.0       ,   self.qr[5]))
        self.A.append(transfer_DH_standard( 0.0       ,   0.0      , 0.0, 0.0       ,   self.qr[6]))
        '''        

        self.A.append(link_T( self.qr[0]       , - math.pi/2, self.a0, 0.0 ))
        self.A.append(link_T( self.qr[1]       ,   math.pi/2, 0.0, 0.0       ))
        self.A.append(link_T( self.qr[2]       , - math.pi/2, 0.0, self.d2   ))
        self.A.append(link_T( self.qr[3]       ,   math.pi/2, 0.0, 0.0       ))
        self.A.append(link_T( self.qr[4]       , - math.pi/2, 0.0, self.d4   ))
        self.A.append(link_T( self.qr[5]       ,   math.pi/2, 0.0, 0.0       ))
        self.A.append(link_T( self.qr[6]       ,   0.0      , 0.0, 0.0       ))

        T = numpy.eye(4)
        # T Transfer matrix of the Arm (from base of arm to tip)
        for X in self.A:
            T = numpy.dot(T, X)
        
        self.end_right_arm = numpy.copy(T)


    def inverse_update(self, T_d):
        '''
        Corrects the joint angles to fulfil kinematic constraints (T6 = T_d) 
        The new joint angles will be as close as possible to the current joint angles: (self.qr)
        '''
        tt0 = self.qr[0] 
        tt3 = self.qr[3] 

    def test_kinematic_equations(self): 
        
        x_e = self.end_right_arm_position

        A = self.d2 + math.cos(self.qr[3])*self.d4
        B = self.d4*math.sin(self.qr[3])

        s0 = math.sin(self.qr[0])
        c0 = math.cos(self.qr[0])
        s1 = math.sin(self.qr[1])
        c1 = math.cos(self.qr[1])
        s2 = math.sin(self.qr[2])
        c2 = math.cos(self.qr[2])
        s3 = math.sin(self.qr[3])
        c3 = math.cos(self.qr[3])

        xp = x_e[0] - c0*self.a0 
        yp = x_e[1] - s0*self.a0 
        zp = x_e[2]

        U    = [xp,yp,zp]/numpy.linalg.norm([xp,yp,zp])
        U_X  = vecmat.skew(U)

        s22 = s2**2
        c32 = c3**2

        E = B*s2
        F = B*c2

        E2 = E**2

        RAB2 = A**2 + B**2

        rxy2  = x_e[0]**2 + x_e[1]**2
        rxy   = math.sqrt(rxy2)
        r2    = rxy2 + x_e[2]**2
        a02   = self.a0**2
        d42   = self.d4**2

        R2 = r2 - self.d2**2 - self.d4**2 + a02
        R4 = R2**2
        Q  = -2*self.d2*self.d4
        Q2 = Q**2
        P2 = 4*a02*rxy2
        T2 = R4 - 4*a02*rxy2

        print " I must be zero (0): ", xp**2 + yp**2 + zp**2 - RAB2
        print " I must be zero (1): ", 4*a02*E2 + (R2+Q*c3)**2 - P2

        alpha = Q2 - 4*a02*d42*s22
        betta = 2*Q*R2
        gamma = R4 + 4*a02*(s22*d42 - rxy2)

        print " I must be zero (2): ", alpha*c32 + betta*c3 + gamma

        print " I must be zero (3): ", T2 + Q2*c32 + 2*R2*Q*c3  + 4*a02*(1-c32)*s22*d42
        print " I must be zero (4): ", (T2 + Q2*c32 + 2*R2*Q*c3)/(4*a02*(c32 -1)*d42) - s22
        
        print " I must be zero (5): ", R2 + Q*c3 - 2*self.a0*(c0*x_e[0]+s0*x_e[1])

        sai = math.atan2(x_e[1],x_e[0]) 

        print " I must be zero (6): ", R2 + Q*c3 - 2*self.a0*rxy*math.sin(self.qr[0] + sai)

        u = math.sin(self.qr[0] + sai)
        a = - 2*self.a0*rxy
        b = Q
        c = R2
        v = - (c + a*u)/b
        v2 = v**2

        print 'I must be zero (7):', c3 - v 

        w = math.sqrt((Q2*v2 + 2*R2*Q*v + T2)/(4*a02*(v2 -1)*d42))
        print 'I must be zero (8):', s2 - w 
        
        print 'I must be zero (9):', A*c1 - F*s1 - x_e[2]

    def solve_inverse_kinematics_explained(self, position, orientation, tt0): 
     
        '''
        Finds all the solutions of the Inverse Kinematic problem for given phi

        The code specifies which joint angle must be equal to tt
        Provides a solution set for PR2 arm for the desired pose given by T_d
        tt is the redundancy parameter and can be arbitrarily chosen.
        ''' 
        solution_set = []

        x_d = T_d[0:3,3]
        R_d = T_d[0:3,0:3]

        rxy2  = x_d[0]**2 + x_d[1]**2
        rxy   = math.sqrt(rxy2)
        r2    = rxy2 + x_d[2]**2
        a02   = self.a0**2
        d42   = self.d4**2

        r2    = rxy2 + x_d[2]**2
        a02   = self.a0**2
        d42   = self.d4**2

        R2 = r2 - self.d2**2 - self.d4**2 + a02
        R4 = R2**2
        Q  = -2*self.d2*self.d4
        Q2 = Q**2
        P2 = 4*a02*rxy2
        T2 = R4 - 4*a02*rxy2

        '''
        code = 1:  theta0 = tt
        '''
        '''
        a, b, c are selected so that: a*u + b*v + c = 0
        '''   
        c0 = math.cos(tt0)
        s0 = math.sin(tt0)

 
        u  = c0*x_d[0] + s0*x_d[1]
        a  = - 2*self.a0
        b  = Q
        c  = R2
        v  = - (c + a*u)/b
        v2 = v**2
        A  = self.d2 + v*self.d4
        #print " I must be zero (0): ", v - math.cos(self.qr[3])

        i  = 0
        while i < 2:
            theta3 = (2*i - 1)*trig.arccos(v)
            s3     = math.sin( theta3)
            c3     = v
            c30    = c3*c0
            B      = self.d4*s3

            T34 = link_T( theta3       ,   math.pi/2, 0.0, 0.0       )
            R34 = T34[0:3,0:3]

            '''
            alpha, betta and gamma are selected so that w^2 * (1 - v^2) = alpha*v^2 + betta*v + gamma
            '''
            foura02d42 = 4*a02*d42 
            alpha = - Q2/foura02d42
            betta = - 2*R2*Q/foura02d42
            gamma = - T2/foura02d42
    
            if gen.equal(v2, 1.0):
                print "Singular Point"
                return []
                '''
                In this case, a singularity happens
                '''
            else: 
                #print " I must be zero (1): ", (gamma + alpha*v2 + betta*v)/(1 - v2) - (math.sin(self.qr[2])**2)
                w2  = (alpha*v2 + betta*v + gamma)/(1 - v2)
                w   = math.sqrt(w2)
                s2  = w
                s20 = s2*s0 
                c0s2= c0*s2  

                j = 0
                while j < 2:
                    theta2 = math.pi*(1 - j) + (2*j - 1)*trig.arcsin(w)
                    c2     = math.cos(theta2)
                    c20    = c2*c0
                    c2s0   = c2*s0
                    E      = B*s2
                    F      = B*c2
                    R1     = math.sqrt(A**2 + F**2)
                    sai1   = math.atan2(F,A)

                    k = 0                
                    while k < 2:
                        theta1 = (2*k - 1)*trig.arccos(x_d[2]/R1) - sai1 
                        s1   = math.sin(theta1)
                        c1   = math.cos(theta1)
                        
                        #print " I must be zero (2): ", R1*math.cos(theta1 + sai1) - x_d[2] 
                        #print " I must be zero (3): ", A*c1 - F*s1 - x_d[2] 

                        As1Fc1 = self.a0 + A*s1 + F*c1
                        X      = c0*As1Fc1 - E*s0
                        Y      = s0*As1Fc1 + E*c0
                        Z      = A*c1 - F*s1 

                        if vecmat.equal(x_d, [X,Y,Z]):
                            R03 = numpy.array([[c20*c1 - s20, -c0*s1, -c2s0 - c1*c0*s2],
                                               [c0s2 + c1*c2*s0, -s1*s0, c20 - c1*s20],   
                                               [-c2*s1, -c1, s2*s1 ]])
                            R04 = numpy.dot(R03, R34)
                            R47 = numpy.dot(R04.T, R_d)

                            l = 0
                            while l < 2:
                                theta5 = (2*l - 1)*trig.arccos(R47[2,2])
                                s5     = math.sin(theta5)
                                c5     = math.cos(theta5)
                                if gen.equal(s5,0):
                                    assert gen.equal(R47[2,0], 0)
                                    print "Singular Point"
                                    return []
                                    '''
                                    In this case, only sum of theta4 + theta6 is known 
                                    '''
                                else:
                                    c6     = - R47[2,0]/s5
                                    s6     =   R47[2,1]/s5
                                    c4     =   R47[0,2]/s5
                                    s4     =   R47[1,2]/s5

                                    theta6 =   gen.sign(s6)*trig.arccos(c6)
                                    assert gen.equal(s6, math.sin(theta6))

                                    theta4 =   gen.sign(s4)*trig.arccos(c4)
                                    assert gen.equal(s4, math.sin(theta4))

                                    assert gen.equal(R47[1,0] ,  c4*s6 + c5*c6*s4)
                                    assert gen.equal(R47[1,1] ,  c4*c6 - c5*s4*s6)
                                    assert gen.equal(R47[0,0] ,  -s4*s6 + c4*c5*c6)
                                    assert gen.equal(R47[0,1] ,  -c6*s4 - c4*c5*s6)
    
                                    solution_set.append([tt0, theta1, theta2, theta3, theta4, theta5, theta6])
                                l = l + 1
                        k = k + 1
                    j = j + 1
            i = i + 1    

        return solution_set


    def __init__(self, a0 = 0.1, d2 = 0.4, d4 = 0.321):

        self.qr = numpy.zeros((7))
    
        self.a0 = a0
        self.d2 = d2
        self.d4 = d4

        self.l_se = [0.0, - self.d2, 0.0]
        self.l_ew = [0.0, 0.0, self.d4]

       
class PR2_Arm_Symbolic():
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
            ca[i] = round(ca[i])
            sa[i] = round(sa[i])
        
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
        T_d = numpy.array([[ Symbol('nx'), Symbol('sx') , Symbol('ax') , Symbol('px') ], 
                           [ Symbol('ny'), Symbol('sy') , Symbol('ay') , Symbol('py') ],
                           [ Symbol('nz'), Symbol('sz') , Symbol('az') , Symbol('pz') ],
                           [ 0           , 0            ,  0           , 1            ]])

        self.x_d = T_d[0:3, 3]
        self.R_d = T_d[0:3, 0:3]

        self.T7 = self.T[6]
        self.x7 = self.T7[0:3, 3]
        self.R7 = self.T7[0:3, 0:3]
    
        self.H = numpy.copy(self.T)
        self.H[n - 1] = numpy.copy(T_d)
        
        # H[i] transfer matrix from link -1(ground) to i calculated from inverse transform
        for i in range(n-1):
            self.H[n - i - 2] = numpy.dot(self.H[n - i - 1], self.B[n - i - 1])

        
def symbolic_help(code):
    '''
    This function applies symbolic PR2 Arm to state (Reference[1] eq. 14) in terms of geometric papameters.
    This helps to find a parametric formulation for the reference joint angles theta1 and theta2  
    '''
    arm = PR2_Arm_Symbolic()

    #x_sw1 = arm.x_d - arm.l_bs - numpy.dot(arm.R_d, arm.l_wt)

    R23 = arm.R[2]

    if code == 1:
        # substituting theta3 = 0 to find the reference rotation matrix R23
        R23[0,0] = R23[0,0].subs('c2',1)
        R23[0,2] = R23[0,2].subs('s2',0)
        R23[1,0] = R23[1,0].subs('s2',0)
        R23[1,2] = R23[1,2].subs('c2',1)
        
        R02 = numpy.dot(arm.R[0],arm.R[1])
        R03 = numpy.dot(R02, R23)
        x_sw2 = numpy.dot(R03, arm.l_se + numpy.dot(arm.R[3], arm.l_ew))

        print 
        print " The following equations help you to find theta0 and theta1 in the reference posture."
        print " s0,c0,s1 and c1 represent sine and cosine of theta0 and theta1 respectively "
        print 

        print x_sw1[0]," = ", x_sw2[0]
        print x_sw1[1]," = ", x_sw2[1]
        print x_sw1[2]," = ", x_sw2[2]

    if code == 2:

        # This code gives you matricres As, Bs and Cs

        R23[0,0] = R23[0,0].subs('c2',1)
        R23[0,2] = R23[0,2].subs('s2',0)
        R23[1,0] = R23[1,0].subs('s2',0)
        R23[1,2] = R23[1,2].subs('c2',1)
        
        R02 = numpy.dot(arm.R[0],arm.R[1])
        R03 = numpy.dot(R02, R23)
        x_sw2 = numpy.dot(R03, arm.l_se + numpy.dot(arm.R[3], arm.l_ew))

        print 
        print " The following equations help you to find As, Bs and Cs matrices according to Ref.1.eq.15"
        print " as a symbolic formulation in terms of the input parameters"
        print 
        u_sw = numpy.array([Symbol('ux'), Symbol('uy'), Symbol('uz')])
        
        uX = vecmat.skew(u_sw)
        # According to Ref.1.eq.15
        As =   numpy.dot(uX, R03)   
        Bs = - numpy.dot(numpy.dot(uX,uX), R03)
        Cs = - Bs + R03
        print 
        print 'As[1,1] = ', As[1,1]
        print 'Bs[1,1] = ', Bs[1,1]
        print 'Cs[1,1] = ', Cs[1,1]
        print 
        print 'As[0,1] = ', As[0,1]
        print 'Bs[0,1] = ', Bs[0,1]
        print 'Cs[0,1] = ', Cs[0,1]
        print 
        print 'As[2,1] = ', As[2,1]
        print 'Bs[2,1] = ', Bs[2,1]
        print 'Cs[2,1] = ', Cs[2,1]
        print 
        print 'As[2,2] = ', As[2,2]
        print 'Bs[2,2] = ', Bs[2,2]
        print 'Cs[2,2] = ', Cs[2,2]
        print 
        print 'As[2,0] = ', As[2,0]
        print 'Bs[2,0] = ', Bs[2,0]
        print 'Cs[2,0] = ', Cs[2,0]

    if code == 3:
        
       
        x70 = [arm.x7[i].subs('c0*c1*c2','c210').subs('c0*c3*s1','c30s1').subs('s0*s2','s20').subs('c1*s0*s3','c1s30') for i in range(0,3)]
        x71 = [x70[i].subs('c2*s0','c2s0').subs('c0*c1*c2','c210').subs('c0*c3*s1','c30s1').subs('s0*s2','s20') for i in range(0,3)]
        x72 = [x71[i].subs('c1*s0*s3','c1s30').subs('c1*c2s0','c21s0').subs('c3*s0*s1','c3s10').subs('c1*c2s0','c21s0') for i in range(0,3)]
        x73 = [x72[i].subs('c0*s2','c0s2').subs('s0*s1*s3','s310').subs('c0*c2','c20').subs('c1*s20','c1s20') for i in range(0,3)]
        x74 = [x73[i].subs('c0*s1*s3','c0s31').subs('c2*c3*s1','c32s1').subs('c2*s1*s3','c2s31').subs('c1*c3','c31') for i in range(0,3)]
        x75 = [x74[i].subs('s1*s2*s4','s421').subs('c1*s3','c1s3').subs('c0s2*c1','c10s2').subs('s0*s1','s10') for i in range(0,3)]
        

        print 'X = ', x75[0]
        print
        print 'Y = ', x75[1]
        print
        print 'Z = ', x75[2]
        print
        '''
        Now we simplify them:
        '''
        x76 = [simplify(x75[i]) for i in range(0,3)]

        print 'X = ', x76[0]
        print
        print 'Y = ', x76[1]
        print
        print 'Z = ', x76[2]

        '''
        Formulations are finally simplified to:

        X =  c0*a0 + c0s1*d2 + (c30s1 + c210s3 - s320)*d4

        Y =  s0*a0 + s10*d2 + (c3s10 + c0s32 + c21s30)*d4

        Z =  c1*d2 + (c31 - c2s31)*d4
        

        '''


    if code == 4:
        '''
        We try to find the formulations for R04 and x03, R47 and x47
        '''
        T03 = arm.T[2]
        R03 = T03[0:3,0:3]
        x03 = T03[0:3,3]
        '''
        For X03 The output must be:
        
        X =  a0*c0 + c0*d2*s1

        Y =  a0*s0 + d2*s0*s1

        Z =  c1*d2


        And for R03 we have:
        '''

        print 'R03[0,0] = ', R03[0,0]
        print 'R03[0,1] = ', R03[0,1]
        print 'R03[0,2] = ', R03[0,2]
        print 
        print 'R03[1,0] = ', R03[1,0]
        print 'R03[1,1] = ', R03[1,1]
        print 'R03[1,2] = ', R03[1,2]
        print 
        print 'R03[2,0] = ', R03[2,0]
        print 'R03[2,1] = ', R03[2,1]
        print 'R03[2,2] = ', R03[2,2]

        '''
        For R03 The output can be simplified to:
        
        R03[0,0] =  -s20 + c210
        R03[0,1] =  -c0s1
        R03[0,2] =  -c2s0 - c10s2

        R03[1,0] =  c0s2 + c21s0
        R03[1,1] =  -s10
        R03[1,2] =  c20 - c1s20

        R03[2,0] =  -c2s1
        R03[2,1] =  -c1
        R03[2,2] =  s21
        '''
    if code == 5:
        '''
        We try to find the formulations for R04 and R47 and R04.T*R_d = R47
        '''
        T04 = arm.T[3]
        R04 = T04[0:3,0:3]
            
        print
        print 'R04[0,0] = ', R04[0,0]
        print 'R04[0,1] = ', R04[0,1]
        print 'R04[0,2] = ', R04[0,2]
        print 
        print 'R04[1,0] = ', R04[1,0]
        print 'R04[1,1] = ', R04[1,1]
        print 'R04[1,2] = ', R04[1,2]
        print 
        print 'R04[2,0] = ', R04[2,0]
        print 'R04[2,1] = ', R04[2,1]
        print 'R04[2,2] = ', R04[2,2]
        print 
        '''

        For R04.T*R_d we have:
        '''
        R47_2 = numpy.dot(R04.T, arm.R_d)

        print
        print 'R47[0,0] = ', R47_2[0,0]
        print 'R47[0,1] = ', R47_2[0,1]
        print 'R47[0,2] = ', R47_2[0,2]
        print 
        print 'R47[1,0] = ', R47_2[1,0]
        print 'R47[1,1] = ', R47_2[1,1]
        print 'R47[1,2] = ', R47_2[1,2]
        print 
        print 'R47[2,0] = ', R47_2[2,0]
        print 'R47[2,1] = ', R47_2[2,1]
        print 'R47[2,2] = ', R47_2[2,2]
        print 
        
        '''

        For R47 we have:

        R47[0,0] =  -s4*s6 + c4*c5*c6
        R47[0,1] =  -c6*s4 - c4*c5*s6
        R47[0,2] =  c4*s5

        R47[1,0] =  c4*s6 + c5*c6*s4
        R47[1,1] =  c4*c6 - c5*s4*s6
        R47[1,2] =  s4*s5

        R47[2,0] =  -c6*s5
        R47[2,1] =  s5*s6
        R47[2,2] =  c5

        '''
        R47 = numpy.dot(numpy.dot(arm.R[4], arm.R[5]), arm.R[6])

        print
        print 'R47[0,0] = ', R47[0,0]
        print 'R47[0,1] = ', R47[0,1]
        print 'R47[0,2] = ', R47[0,2]
        print 
        print 'R47[1,0] = ', R47[1,0]
        print 'R47[1,1] = ', R47[1,1]
        print 'R47[1,2] = ', R47[1,2]
        print 
        print 'R47[2,0] = ', R47[2,0]
        print 'R47[2,1] = ', R47[2,1]
        print 'R47[2,2] = ', R47[2,2]
        print 

        """
        EQ1:  a*s0 + b*c3 + c = 0
        EQ2:  alpha*c33 + betta*c3 + gamma - s22*s33 = 0
        EQ3:  c1*d2 + (c31 - c2s31)*d4 - Z = 0

        EQ4: ax*(-s320 + c210s3 + c30s1) + ay*(c0s32 + c21s30 + c3s10) + az*(c31 - c2s31) -  c5 = 0
        EQ5: ax*(-c2s0 - c10s2) + ay*(c20 - c1s20) + az*s21 - s54 = 0
        EQ6: nx*(-s320 + c210s3 + c30s1) + ny*(c0s32 + c21s30 + c3s10) + nz*(c31 - c2s31) + c6s5 = 0

        F10 =   a*c0
        F13 = - b*s3

        F22 = - 2*c2s332
        F23 = - 2*alpha*c3s3 - betta*s3 - 2*c3s322
        
        F31 = - d2*s1 - (c3s1 + c21s3)*d4
        F32 =   s321*d4
        F33 =   - (c1s3 + c32s1)*d4
        
        F40 = ax*(- c0s32 - c21s30 - c3s10) + ay*(- s320  + c210s3 + c30s1)  
        F41 = ax*(- c20s31 + c310) + ay*(- c2s310 + c31s0) + az*(- c3s1 - c21*s3)
        F42 = - ax*(c2s30 + c10s32) + ay*(c20s3 - c1s320) + az*s321


        F43 = ax*(- c3s20 + c3210 - c0s31) + ay*(c30s2 + c321s0 - s310) + az*(- c1s3 - c32s1)
        F45 = s5

        F50 = ax*(-c20 + c1s20) - ay*(c2s0 + c10s2)
        F51 = ax*c0s21 - ay*s210 + az*c1s2
        F52 = ax*(s20 - c210) - ay*(c0s2 + c21s0) + az*c2s1
        F54 = - c4s5
        F55 = - c5s4
        
        F60 = - nx*(c0s32 + c21s30 + c3s10) + ny*(- s320 + c210s3 + c30s1)
        F61 =   nx*(- c20s31 + c310) + ny*(- c2s310 + c31s0) - nz*(c3s1 + c21s3)
        F62 = - nx*(c2s30 + c10s32) + ny*(c20s3 - c1s320) - nz*s321
        F63 =   nx*(- c3s20 + c3210 - c0s31) + ny*(c30s2 + c321s0 - s310) + nz*(- c1s3 - c32s1)
        F65 =   c65
        F66 = - s65

        if F is a (7*7) matrix, we have: F*J.T = [1, 0, 0, 0, 0, 0, 0].T

        J0 = 1

        F10*J0 + F13*J3 = 0  ==>  J3 = - F10*J0/F13 = - F10/F13

        F22*J2 + F23*J3 = 0  ==>  J2 = - F23*J3/F22

        F31*J1 + F32*J2 + F33*J3 = 0 ==>  J1 = - (F32*J2 + F33*J3)/F31

        F40*J0 + F41*J1 + F42*J2 + F43*J3 + F45*J5 = 0 ==> J5 = - (F40 + F41*J1 + F42*J2 + F43*J3)/F45

        F50*J0 + F51*J1 + F52*J2 + F54*J4 + F55*J5 = 0 ==> J4 = - (F50 + F51*J1 + F52*J2 + F55*J5)/F54 

        F60*J0 + F61*J1 + F62*J2 + F63*J3 + F64*J4 + F65*J5 + F66*J6 = 0  ==> J6 = - (F60 + F61*J1 + F62*J2 + F63*J3 + F64*J4 + F65*J5)/ J66



        H100 = - a*s0

        H133 = - b*c3

        H222 = - 2*s33(- s22 + c22) = -2*(- s3322 + c22s33)
        H223 = - 4*c2s2*c3s3 = -4*c32*s32

        H232 = - 4*c3s3*s2c2 = -4*c32*s32
        H233 = - 2*(c33 - s33)*(alpha + s22) - betta*c3 

        H311 = - d2*c1 - (c31 - c2s31)*d4
        H312 = - c1s32*d4
        H313 = (- s31 + c321)*d4

        H321 =   c1s32*d4
        H322 =   c2s31*d4
        H323 =   c3s21*d4

        H331 =   (s1s3 - c32c1)*d4
        H332 =   c3s21*d4
        H333 =   (c2s31 - c31)*d4
        
        H400 = ax*(s320 - c210s3 - c30s1) - ay*(c0s32 + c21s30 + c3s10)  
        H401 = ax*(c2s310 - c31s0) + ay*(c310 - c20s31)  
        H402 = ax*(c1s320 - c20s3) - ay*(c2s30 + c10s32)  
        H403 = ax*(s310 - c30s2 - c321s0) + ay*(c3210 - c3s20 - c0s31)  
        H405 = 0

        H410 =   ax*(c2s310 - c31s0) + ay*(c310 - c20s31)
        H411 = - ax*(c210s3 + c30s1) - ay*(c21s30 + c3s10) + az*(c2s31 - c31)
        H412 =   ax*c0s321 + ay*s3210 - az*c1s32
        H413 = - ax*(c320s1 + c10s3) - ay*(c32s10 + c1s30) + az*(s31 - c321)
        H415 = 0

        H420  = ax*(- c20s3 + c1s320) + ay*(- c2s30 - c10s32)
        H421  = ax*c0s321 + ay*s3210 + az*c1s32
        H422  = ax*(s320 - c210s3) - ay*(c0s32 + c21s30) + az*c2s31
        H423  = - ax*(c32s0 + c310s2) + ay*(c320 - c31s20) + az*c3s21

        H430 =   ax*(s310 - c30s2 - c321s0) + ay*(c3210 - c3s20 - c0s31)
        H431 = - ax*(c320s1 + c10s3) - ay*(c32s10 + c1s30) + az*(s31 - c321)
        H432 = - ax*(c32s0 + c310s2) + ay*(c320 - c31s20) + az*c3s21
        H433 =   ax*(s320 - c210s3 - c30s1) - ay*(c0s32 + c21s30 + c3s10) + az*(c2s31 - c31)

        H455 = c5

        H500 =   ax*(c2s0 + c10s2) - ay*(c20 - c1s20)
        H501 = - ax*s210 + ay*c0s22
        H502 =   ax*(c0s2 + c21s0) + ay*(s20 - c210)

        H510 = - ax*s210 - ay*c0s21
        H511 =   ax*c10s2 - ay*c1s20 - az*s21
        H512 =   ax*c20s1 - ay*c2s10 + az*c21

        H520 = ax*(c0s2 + c21s0) + ay*(s20 - c210)
        H521 = ax*c20s1 + ay*c2s10 + az*c21
        F522 = ax*(c2s0 + c10s2) - ay*(c20 - c1s20) - az*c2s1

        H544 =   s54
        H545 = - c54

        H554 = - c54
        H555 =   s54
        
        H600 = - nx*(c210s3 + c30s1 - s320) - ny*(c0s32 + c21s30 + c3s10)
        H601 = - nx*(- c2s310 + c31s0) + ny*(- c20s31 + c310)
        H602 = - nx*(c20s3 - c1s320) - ny*(c2s30 + c10s32)
        H603 = - nx*(c30s2 + c321s0 - s310) + ny*(- c3s20 + c3210 - c0s31)

        H610 =   nx*(c2s310 - c31s0) + ny*(c310- c20s31)
        H611 = - nx*(c210s3 + c30s1) - ny*(c21s30 + c3s10) - nz*(c31 - c2s31)
        H612 =   nx*c0s321 + ny*s3210 + nz*c1s32
        H613 = - nx*(c320s1 + s3c10) - ny*(c32s10 + c1s30) - nz*(- s31 + c321)

        H620 = - nx*(c20s3 - c1s320) - ny*(c2s30 + c10s32)
        H621 =   nx*c0s321 - ny*s3210 - nz*s32c1 
        H622 =   nx*(s320 - c210s3) - ny*(c0s32 + c21s30) - nz*c2s31
        H623 = - nx*(c32s0 + c310s2) + ny*(c320 - c31s20) - nz*c3s21

        H630 =   nx*(s310 - c30s2 - c321s0) + ny*(- c3s20 + c3210 - c0s31)
        H631 = - nx*(c320s1 + c10s3) - ny*(c32s10 + c1s30) + nz*(s31 - c321)
        H632 = - nx*(c32s0 + c310s2) + ny*(c320 - c31s20) + nz*c3s21
        H633 =   nx*(s320 - c210s3) - ny*(s320 + s30c21) + nz*c2s31

        H655 = - c6s5
        F656 = - c5s6

        F665 = - c5s6
        F666 = - c6s5

        """


def main():

    srs = PR2_ARM(a0=0)

    srs.qr = numpy.array([-0.5, 0.5 + math.pi/2, 0.5, -0.5, 0.5, -0.5, 0.5])
    srs.qr = drc*numpy.array([-45, 7, -51, -22, -90, 80, 15])
    
    srs.forward_update()

    #srs.end_right_arm[1,3] = srs.end_right_arm[1,3] - 0.188 
    
    srs.solve_inverse_kinematics_explained(srs.end_right_arm, srs.qr[0])

    print srs.end_right_arm
    '''
    srs.solve_inverse_kinematics_explained(srs.end_right_arm, phi = 0.1801855)
    print "End of Right Arm:"
    print srs.end_right_arm
    '''

def main_2():

    #srs = PR2_ARM(a0 = 0.1, d2 = 0.4, d4 = 0.321)
    srs = PR2_ARM(a0 = 0.5, d2 = 1.4, d4 = 8.21)
    #srs = SRS_ARM()

    srs.qr = drc*numpy.array([45, -10, 40, -45, 20, 15, -10])
    #srs.qr = drc*numpy.array([-45, 7, -51, -22, -90, 80, 15])
    print 'q1 = ', srs.qr
    
    srs.forward_update()
    era_pos = numpy.copy(srs.end_right_arm_position)
    srs.test_kinematic_equations()

    all_solutions = srs.solve_inverse_kinematics_explained(srs.end_right_arm_position, srs.end_right_arm_orientation, tt0 = 45*drc)

    for i in range(0 ,len(all_solutions)):
        print all_solutions[i]
        srs.qr = all_solutions[i]

        srs.forward_update()
        print 
        '''
        print "End of Right Arm:"
        print 
        print srs.end_right_arm
        print 
        print end_right_arm
        '''
        if vecmat.equal(srs.end_right_arm_position, era_pos):
            print "Successful :-)"
        else:
            print "Failed :-("

def main_3():
    symbolic_help(5)    
     
if __name__ == "__main__" :
    
    main_2()

