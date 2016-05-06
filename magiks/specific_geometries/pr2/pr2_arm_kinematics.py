## @file        	PR2_arm_kinematics.py
#  @brief     		Contains specific functions that define all geometric and kinematic parameters for PR2 robot arm
#  @author      	Nima Ramezani Taghiabadi 
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. :   04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	6.0 
#
#  Last Revision:  	10 July 2015

'''
Changes from ver 5.0:

'''

import copy, time, math
import numpy as np, general_python as genpy

from interval import interval, inf, imath
from sets import Set

from math_tools import general_math as genmath
from math_tools.geometry import trigonometry as trig, trajectory as trajlib, geometry as geo
from math_tools.algebra import vectors_and_matrices as vecmat
from magiks.magiks_core import general_magiks as genkin

drc        = math.pi/180.00

#default_ql = drc*np.array([-130.0, 70.0 , -180.0,   0.0, -180.0,   0.0, -180.0])
#default_qh = drc*np.array([  40.0, 170.0,   44.0, 130.0,  180.0, 130.0,  180.0])

default_ql = drc*np.array([-130.0, 70.0 , -180.0, - 131.0, -180.0, -130.0, -180.0])
default_qh = drc*np.array([  30.0, 170.0,   44.0, -   8.6,  180.0, 0.00,  180.0])

default_W  = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

class PR2_Arm_Symbolic():
    '''
    This class provides a parametric representation of the kinematics of PR2 arm
    The position, orientation and the Jacobian can be expressed in terms of DH parameters and sine and cosine of the joint angles
    It uses sympy as a tool for symbols algebra
    '''
    def __init__(self):
        import sympy
        from sympy import Symbol, simplify

        n = 7

        c = [Symbol('c' + str(i)) for i in range(n)]
        s = [Symbol('s' + str(i)) for i in range(n)]
        a = [Symbol('a0'), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        d = [0.0, 0.0, Symbol('d2'), 0.0, Symbol('d4'), 0.0, 0.0]
        alpha = [- math.pi/2, math.pi/2, - math.pi/2, math.pi/2, - math.pi/2, math.pi/2, 0.0]    
        
        ca    = [math.cos(alpha[i]) for i in range(n)]
        sa    = [math.sin(alpha[i]) for i in range(n)]

        for i in range(len(ca)):
            ca[i] = genmath.round(ca[i])
            sa[i] = genmath.round(sa[i])
        
        # A[i] transfer matrix from link i-1 to i  according to standard DH parameters:
        self.A = [np.array([[ c[i],  -ca[i]*s[i],  sa[i]*s[i], a[i]*c[i] ], 
                               [ s[i],   ca[i]*c[i], -sa[i]*c[i], a[i]*s[i] ],
                               [ 0   ,   sa[i]     ,  ca[i]     , d[i]      ],
                               [ 0   ,   0         ,  0         , 1         ]]) for i in range(n)]
                               
        # B[i] transfer matrix from link i to i-1 or B[i] = inv(A[i])
        self.B = [np.array([[        c[i],         s[i], 0     ,  0          ], 
                               [ -ca[i]*s[i],   ca[i]*c[i], sa[i] , -d[i]*sa[i] ],
                               [  ca[i]*s[i],  -sa[i]*c[i], ca[i] , -d[i]*ca[i] ],
                               [  0         ,   0         , 0     ,  1          ]]) for i in range(n)]

        # R[i] rotation matrix from link i-1 to i  according to standard DH parameters:
        self.R = [np.array([[ c[i],  -ca[i]*s[i],  sa[i]*s[i]], 
                               [ s[i],   ca[i]*c[i], -sa[i]*c[i]],
                               [ 0   ,   sa[i]     ,  ca[i]     ]]) for i in range(n)]

        # T[i] transfer matrix from link -1(ground) to i
        self.T = np.copy(self.A)
        for i in range(n-1):
            self.T[i + 1] = np.dot(self.T[i],self.A[i + 1])

        # Desired Pose:
        T_d = np.array([[ Symbol('r11'), Symbol('r12') , Symbol('r13') , Symbol('px') ], 
                           [ Symbol('r21'), Symbol('r22') , Symbol('r23') , Symbol('py') ],
                           [ Symbol('r31'), Symbol('r32') , Symbol('r33') , Symbol('pz') ],
                           [ 0           , 0            ,  0           , 1            ]])

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

        self.p_EF_W = np.array([0.0, 0.0, Symbol('d7')])
    
        self.H = np.copy(self.T)
        self.H[n - 1] = np.copy(T_d)
        
        # H[i] transfer matrix from link -1(ground) to i calculated from inverse transform
        for i in range(n-1):
            self.H[n - i - 2] = np.dot(self.H[n - i - 1], self.B[n - i - 1])

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


## @brief Contains properties and methods regarding the PR2 arm joint-space.
#  This class has methods and properties for managing the joint-space of a PR2 robot arm.
#  It contains the joint position, velocity and acceleration values and joint limits.
class PR2_ARM_Configuration():
	## The Class Constructor:
	#  @param ql A numpy array of size 7 containing the lower bounds of the arm joints	
	#  @param qh A numpy array of size 7 containing the upper bounds of the arm joints
	#  @param W  A numpy array of size 7 containing the weights of each joint in the objective function. 
	#
	#			 The joint weights will be used in the optimization of redundancy. 
	#            If the weight of a joint is 0, the joint is considered as unlimited. 
	#            This means the value of that joint does not need to be in a specific range. 
    def __init__(self, ql = default_ql, qh = default_qh, W = default_W):
        assert (len(ql) == 7) and (len(qh) == 7), "Error from " + __name__ + ": Given arrays ql and qh must have 7 elements" 
        ## A numpy array of size 7 containing the current values of the joints. 
        #  This property should not be manipulated by the user. Use method Set_Config() to change this property.
        #  The joints are in the following order:
        #  q[0] : Shoulder Pan Joint, 
        #  q[1] : Shoulder Lift Joint,
        #  q[2] : Upper Arm Roll Joint, 
        #  q[3] : Elbow Flex Joint,
        #  q[4] : Forearm Roll Joint, 
        #  q[5] : Wrist Flex Joint, 
        #  q[6] : Wrist Roll Joint
        self.q 		= (qh + ql)/2

		##  A numpy array of size 7 containing the current velocities of the joints.
        self.q_dot  = np.zeros(7)

        ##  A numpy array of size 7 containing the current accelerations of the joints.
        self.q_ddot = np.zeros(7)

        ##  A numpy array of size 7 containing the lower bounds of the arm joints 
        self.ql = ql

        ##  A numpy array of size 7 containing the upper bounds of the arm joints 
        self.qh = qh

        ##  A numpy array of size 7 containing a set of reference values for the joint angles. 
        #
        #  This parameter can be used as reference joint values for redundancy optimization. 
        #  The redundancy will be used to set the joints as close as possible to these reference values.  
        #  This property is by default set as the middle of joint ranges.		
        self.qm = (qh + ql)/2

        ##  A numpy array of size 7 containing the weights of each joint in the objective function. 
        #
        #   The joint weights will be used in the optimization of redundancy. 
        #   If the weight of a joint is 0, the joint is considered as unlimited. 
        #   This means the value of that joint does not need to be in a specific range. 
        self.w 		= np.zeros(7)

		##
		#  \b Note:
		#  --------
		#  Only ql, qh, qm and W can be manipulated by the user.
			
        for i in range(0,7):
            self.w[i] = math.sqrt(W[i])

        # sets all angles to the midrange by default
        self.set_config(self.qm)

	## This function is used to check if a given value for specific joint is in its feasibility range
	#  @param i An integer between 0 to 6 specifying the joint number to be checked
	#  @param qi A float parameter specifying the value for the i-th joint
	#  @return A boolean: True if the given joint angle qi is in feasible range for the i-th joint (within the specified joint limits for that joint)
    def joint_in_range(self,i, qi):
        qi = trig.angle_standard_range(qi) 
        if abs(qi - self.ql[i]) < genmath.epsilon:
            qi = self.ql[i]
        if abs(qi - self.qh[i]) < genmath.epsilon:
            qi = self.qh[i]

        return ((qi <= self.qh[i]) and (qi >= self.ql[i]))
    
	## This function is used to check if all values of the given joint array are in their feasibility range
	#  @param qd A numpy array of size 7 containing the values to be checked
	#  @return A boolean: If The given joints "qd" are out of the range specified by properties: ql and qh, returns False, otherwise returns True
    def all_joints_in_range(self, qd):
        flag = True
        for i in range(0, 7):
            if not genmath.equal(self.w[i], 0.0):
                flag = flag and self.joint_in_range(i, qd[i])
        return flag

	## Use this function to get the midrange error vector
	#  @param None
	#  @return A numpy array of size 7 containing the weighted deviations of the joints from their midrange values
    def midrange_error(self):
        return trig.angles_standard_range(self.q - self.qm)*self.w 

	## This function gives an interval for the magnitude of the arm joint angles correction vector.
	#  @param direction A numpy array of size 7 specifying the direction of change	
	#  @param max_speed A float parameter specifying the maximum feasible speed for a joint change. (Set by infinity by default)	
	#  @param delta_t The step time in which the change(correction) is to be applied.
	#  @return An instance of type <a href="http://pyinterval.googlecode.com/svn/trunk/html/index.html">Interval()</a>
    def joint_stepsize_interval(self, direction, max_speed = genmath.infinity, dt = 0.001):
        '''
        The correction of joints is always restricted by joint limits and maximum feasible joint speed
        If you want to move the joints in a desired direction, how much are you allowed to move?	
        This function returns a feasible interval for the stepsize in the given direction. 
        The joint direction of change must be multiplied by a scalar value in this range so that the applied changes
        are feasible.
        '''
        etta_l = []
        etta_h = []

        for i in range(7):
            if not genmath.equal(direction[i], 0.0):
                if (self.w[i] != 0.0):
                    a = (self.ql[i] - self.q[i])/direction[i]    
                    b = (self.qh[i] - self.q[i])/direction[i]
                    etta_l.append(genmath.sign_choice(a, b, direction[i]))
                    etta_h.append(genmath.sign_choice(b, a, direction[i]))

                a = dt*max_speed/direction[i]
                etta_l.append(genmath.sign_choice(-a, a, direction[i]))
                etta_h.append(genmath.sign_choice( a,-a, direction[i]))

        if (etta_l == []) or (etta_h == []):
            return (0.0, 0.0)
        else:
            return (max(etta_l), min(etta_h))

	## Use this function to set the joint configuration to the given desired values.
	#  @param qd A numpy array of size 7 containing the desired joint values to be set
	#  @return A boolean: True if the given joints are in range and the configuration is set, False if not	
    def set_config(self, qd):
        '''
        This function sets the robot joint configuration to the given "qd"
        This function should not be called by the end user. 
        Always use function "set_config" in class PR2_ARM()  
        '''    
        if not len(qd) == 7:
            print "set_config error: Number of input elements must be 7"
            return False

        if self.all_joints_in_range(qd):
            for i in range(0, 7):
                if genmath.equal(self.w[i], 0.0):
                    self.q[i] = qd[i]
                else:
                    self.q[i] = trig.angle_standard_range(qd[i])

            self.c = [math.cos(self.q[i]) for i in range(0,7)]
            self.s = [math.sin(self.q[i]) for i in range(0,7)]

            [s0, s1, s2, s3, s4, s5, s6] = self.s
            [c0, c1, c2, c3, c4, c5, c6] = self.c
			## protected
            self.s1_mult1 = [s1*s0]
            [s10]         = self.s1_mult1

			## protected
            self.s2_mult1 = s2*np.array([s0, s1, s2, s10])
            [s20,s21, s22, s210]  = self.s2_mult1

            self.s3_mult1 = s3*np.array([s0, s1, s2, s10, s20, s21, s22, s3,s210])
            [s30, s31, s32, s310, s320, s321, s322, s33,s3210] = self.s3_mult1

            self.c0_mult = c0*np.array([s0,s1,s2,s10,s20,s21,s30,s31,s32,s321])
            [c0s0,c0s1,c0s2,c0s10,c0s20,c0s21,c0s30,c0s31,c0s32,c0s321] = self.c0_mult

            self.c1_mult = c1*np.array([c0, s0, s1, s2, s3, s10, s20,s30,s32, s320])
            [c10, c1s0, c1s1, c1s2, c1s3, c1s10, c1s20,c1s30,c1s32,c1s320] = self.c1_mult

            # \cond
            self.c2_mult = c2*np.array([c0,c1,c2,c10,s0,s1,s2,s3,s10,s20,s30,s31,s310, c1s30,c0s31])
            [c20,c21,c22,c210,c2s0,c2s1,c2s2,c2s3,c2s10,c2s20,c2s30,c2s31,c2s310,c21s30,c20s31] = self.c2_mult

            self.c3_mult = c3*np.array([c0,c1,c2,c3, s0,s1,s2,s3,s10,s20,s21,s30,c10,c20,c21,c210,c2s0,c2s10])
            [c30,c31,c32,c33,c3s0,c3s1,c3s2,c3s3,c3s10,c3s20,c3s21,c3s30,c310,c320,c321,c3210,c32s0,c32s10] = self.c3_mult

            self.s0_mult = s0*np.array([c21,c31,c321])
            [c21s0,c31s0,c321s0]  = self.s0_mult

            self.s1_mult2 = s1*np.array([c10,c20,c30, c32,c320])
            [c10s1,c20s1,c30s1, c32s1,c320s1] = self.s1_mult2

            self.s2_mult2 = s2*np.array([c10,c20,c30, c32,c310])
            [c10s2,c20s2,c30s2, c32s2,c310s2]  = self.s2_mult2

            self.s3_mult2 = s3*np.array([c10, c20, c30, c21, c210,s32])
            [c10s3, c20s3, c30s3, c21s3, c210s3,s332]  = self.s3_mult2

            self.cs_mult = [c10*s32, c31*s20, s5*s4]
            [c10s32, c31s20, s54] = self.cs_mult

            self.c5_mult = c5*np.array([c4, s4, s5])
            [c54, c5s4, c5s5] = self.c5_mult

            self.s6_mult = s6*np.array([s4, s5, c4, c5, c54, s54])
            [s64, s65, c4s6, c5s6, c54s6, s654] = self.s6_mult

            self.c6_mult = c6*np.array([c4, c5, s4, s5, c54, s54])
            [c64, c65, c6s4, c6s5, c654, C6s54] = self.c6_mult

            self.mid_dist_sq = None
            '''
            Do not set any other environmental variables here
            '''    
            # \endcond
            return True
        else:
            print "Error from PR2_ARM.set_config(): Given joints are not in their feasible range"
            for i in range(7):
                if qd[i] > self.qh[i]:
                    print "Joint No. ", i , " is ", qd[i], " greater than maximum: ", self.qh[i]
                if qd[i] < self.ql[i]:
                    print "Joint No. ", i , " is ", qd[i], " lower than minimum: ", self.ql[i]

            return False
    
    ## protected   
    def closest_config_metric(self, q):
        dist = 0
        cc   = np.zeros(7)
        for i in range(7):
            (qq, dd) = trig.closest_angle_metric(self.q[i], q[i])
            dist  = dist + dd
            cc[i] = qq
        return (cc, dd)

    ## Use this function to find the current value of the objective function
    #  @param None
    #  @return A float containing the current value of the objective function
    #          (deviation from midrange). This objective function depends on the values of the joints
    def objective_function(self):
        if self.mid_dist_sq == None:
            e = self.midrange_error()
            self.mid_dist_sq = np.dot(e.T,e)
        return self.mid_dist_sq

## @brief Contains properties and methods managing the kinematics of the PR2 robot arm.
#  This class contains all properties and methods required for forward and inverse kinematic computations and kinematic control of the arm
class PR2_ARM():

    ## The Class Constructor:
    #  @param a0 A float specifying the value of /f$ a_0 /f$ in the DH parameters of the arm 	
    #  @param d2 A float specifying the value of /f$ d_2 /f$ in the DH parameters of the arm 	
    #  @param d4 A float specifying the value of /f$ d_4 /f$ in the DH parameters of the arm 	
    #  @param ql A numpy array of size 7 containing the lower bounds of the arm joints	
    #  @param qh A numpy array of size 7 containing the upper bounds of the arm joints
    #  @param W  A numpy array of size 7 containing the weights of each joint in the objective function. 
    #
    #			 The joint weights will be used in the optimization of redundancy. 
    #            If the weight of a joint is 0, the joint is considered as unlimited. 
    #            This means the value of that joint does not need to be in a specific range. 
    def __init__(self, a0 = 0.1, d2 = 0.4, d4 = 0.321, ql = default_ql, qh = default_qh, W = default_W, run_magiks = False):

		## A boolean specifying if orientation of the endeffector should be respected or not in finding the Inverse Kinematic solution
        self.orientation_respected = True

        self.silent = True
        
		## An instance of class PR2_ARM_CONFIGURATION() containing the jointspace properties and methods of the arm
        self.config = PR2_ARM_Configuration(ql = ql, qh = qh, W = W)

		## A \f$ 3 \times 3 \f$ rotation matrix specifying the desired endeffector orientation for the IK problem. 
		# Recommended not to be set directly. Always use function set_target() to change this property. 
        self.Rd = None    

		## A numpy vector of size 3 specifying the desired endeffector position for the IK problem. 
		# Recommended not to be set directly. Always use function set_target() to change this property. 
        self.xd = None    

		## A float specifying the value of \f$ a_0 \f$ in the DH parameters of the arm
        self.a0 = a0
        
		## A float specifying the value of \f$ d_2 \f$ in the DH parameters of the arm
        self.d2 = d2
        
		## A float specifying the value of \f$ d_4 \f$ in the DH parameters of the arm
        self.d4 = d4

        self.dt     = 0.01
        self.max_js = 2.0   #  (Rad/sec)
        self.max_ja = 100.0 # (Rad/sec^2)

        l2 = d2**2 + d4**2 - a0**2
        
        assert l2 > 0, "Error from " + __name__ + "Constructor: Invalid dimensions or DH parameters"

        self.l = math.sqrt(l2)

        d42 = d4*d2
        Q   = -2*d42
        Q2  = Q**2
        d44 = d4**2
        a00 = a0**2

        d22_plus_d44 = d2*d2 + d44
        foura00d44   = 4*a00*d44 
        alpha        = - Q2/foura00d44

		## protected
        self.additional_dims = [Q, Q2, d42, d44, a00, d22_plus_d44, foura00d44, alpha]

		## protected
        self.l_se = [0.0, - d2, 0.0]
        self.l_ew = [0.0, 0.0, d4]

		## An integer between 0 and 1 specifying the objective function code
        self.ofuncode = 0

		## An numpy array of size 3 containing the position of the wrist joint center w.r.t. the shoulder pan joint center in torso coordinate system 
		#  Recommended to be read only. Calling function wrist_position() changes the value of this property. 
        self.wrist_position_vector    = None
        
        self.in_target_flag           = None
        
		## A \f$ 3 \times 3 \f$ numpy matrix representing the orientation of the endeffector (gripper) w.r.t. the torso 
		#  Recommended to be read only. Calling function wrist_orientation() changes the value of this property. 
        self.wrist_orientation_matrix = None
        
        ## A numpy array containing the \b Joint Redundancy Jacobian of the arm.
        #  The Joint Redundancy Jacobian represents the derivatives of the joints w.r.t. the redundant parameter
        #  \f[ 
        #  J_{ij} = \frac {\partial \theta_i }{\partial \phi_j}
        #  \f]  
        self.JRJ                      = None
        
        ## protected
        self.E                        = None
        
		## protected	
        self.F                        = None
        
		##  Phi A variable of type <a href="http://pyinterval.googlecode.com/svn/trunk/html/index.html">Interval()</a>
		##	containing the feasible interval for the redundant parameter \f$ \phi \f$ or the first joint angle (Shoulder Pan Joint) 	
        self.Phi    = None

		##  Delta A variable of type <a href="http://pyinterval.googlecode.com/svn/trunk/html/index.html">Interval()</a> 
		#   containing the feasible interval for the growth of redundant parameter (\f$ \Delta \phi \f$) 		
        self.Delta  = None

        if run_magiks:
            from magiks.magiks_core import inverse_kinematics as iklib, manipulator_library as manlib
            cs       = manlib.manip_config_settings('PR2ARM', joint_mapping = 'TM')
            cs.ql    = np.copy(self.config.ql)
            cs.qh    = np.copy(self.config.qh)
            es       = iklib.eflib.Endeffector_Settings()
            iks      = iklib.Inverse_Kinematics_Settings(algorithm = 'JPI')
            self.ik  = iklib.Inverse_Kinematics(cs, manlib.PR2ARM_geometry_settings, es, iks)
            self.magiks_running = True
        else:
            self.magiks_running = False

        self.use_magiks = False

		# Sets the target initially as the endeffector pose corresponding to the midrange joint configuration:
        self.set_target(self.wrist_position(), self.wrist_orientation())

	## Sets the robot configuration to the desired given joint values
	#  @param qd A numpy array of 7 elements containing the values of the given joints
	#  @return A boolean: True  if the given configuration is successfully set, False if the given configuration is not successfully set. 
    #         (If False is returned, the configuration will not chenge)
    def set_config(self, qd): 
        '''
        Example:
            
        arm = PR2_ARM()
        qd  = np.zeros(7)	
        arm.set_config(qd)	
        print arm.config.q
        print arm.wrist_position()
        print arm.wrist_orientation()	
        '''
        if self.magiks_running:
            permit = self.ik.set_config(qd) and self.config.set_config(qd)
        else:
            permit = self.config.set_config(qd)

        if permit:
            self.wrist_position_vector    = None
            self.wrist_orientation_matrix = None
            self.in_target_flag           = None
            self.JRJ          = None
            self.E           = None
            self.F           = None
            self.Delta       = None
            return True
        else:
            return False

	## Sets the endeffector target to a desired position and orientation.
	#  @param target_position    A numpy array of 3 elements specifying the desired endeffector position
	#  @param target_orientation A \f$ 3 \times 3 \f$ numpy rotation matrix representing the desired endeffector orientation
	#  @return A boolean: True  if the given pose is successfully set, False if the given pose is not successfully set because of an error
    #          (If False is returned, properties self.xd and self.RD will not chenge)
    def set_target(self, target_position, target_orientation):

        assert genmath.equal(np.linalg.det(target_orientation), 1.0)
        self.xd = target_position
        self.Rd = target_orientation
        if self.use_magiks:
            self.ik.set_target([target_position], [geo.Orientation_3D(target_orientation)])

        [Q, Q2, d42, d44, a00, d22_plus_d44, foura00d44, alpha] = self.additional_dims

        sai = math.atan2(self.xd[0], self.xd[1])
        r2  = self.xd[0]**2 + self.xd[1]**2
        ro2 = r2 + self.xd[2]**2
        R2  = ro2 - d22_plus_d44 + a00
        '''
        alpha, betta and gamma are selected so that w^2 * (1 - v^2) = alpha*v^2 + betta*v + gamma
        '''
        
        T2    = R2**2 - 4*a00*r2
        betta = - 2*R2*Q/foura00d44
        gamma = - T2/foura00d44

        self.Phi            = None
        self.Delta          = None

        self.target_parameters = [r2, ro2, R2, T2, alpha, betta, gamma, sai]

	## Use this function to get the current position of the elbow joint center.
	#  @param None
	#  @return A numpy vector of size 3 containing the cartesian coordinates of the elbow joint center  w.r.t. the shoulder pan joint in the torso CS.
    def elbow_position(self):   
        X =  self.a0*self.config.c[0] + self.config.c[0]*self.d2*self.config.s[1]
        Y =  self.a0*self.config.s[0] + self.d2*self.config.s[0]*self.config.s[1]
        Z =  self.config.c[1]*self.d2
        return np.array([X,Y,Z])
    
	## Use this function to get the current position of the wrist joint center.
	#  @param None
	#  @return A numpy vector of size 3 containing the cartesian coordinates of the wrist joint center  w.r.t. the shoulder pan joint in the torso CS.
    def wrist_position(self):        
        '''
        Returns the cartesian coordiantes of the origin of the wrist. The origin of the wrist is the wrist joint center 
        '''    
        if self.wrist_position_vector == None:
            [s0, s1, s2, s3, s4, s5, s6] = self.config.s
            [c0, c1, c2, c3, c4, c5, c6] = self.config.c
            [s10]         = self.config.s1_mult1
            [s30, s31, s32, s310, s320, s321, s322, s33,s3210] = self.config.s3_mult1
            [c0s0,c0s1,c0s2,c0s10,c0s20,c0s21,c0s30,c0s31,c0s32,c0s321] = self.config.c0_mult
            [c20,c21,c22,c210,c2s0,c2s1,c2s2,c2s3,c2s10,c2s20,c2s30,c2s31,c2s310,c21s30,c20s31] = self.config.c2_mult
            [c30,c31,c32,c33,c3s0,c3s1,c3s2,c3s3,c3s10,c3s20,c3s21,c3s30,c310,c320,c321,c3210,c32s0,c32s10] = self.config.c3_mult
            [c10s1,c20s1,c30s1, c32s1,c320s1] = self.config.s1_mult2
            [c10s3, c20s3, c30s3, c21s3,c210s3,s332]  = self.config.s3_mult2

            X =  c0*self.a0 + c0s1*self.d2 + (c30s1 + c210s3 - s320)*self.d4

            Y =  s0*self.a0 + s10*self.d2 + (c3s10 + c0s32 + c21s30)*self.d4

            Z =  c1*self.d2 + (c31 - c2s31)*self.d4
        
            self.wrist_position_vector = np.array([X, Y, Z])

        return copy.copy(self.wrist_position_vector)

	## Use this function to get the current orientation of the gripper(Endeffector).
	#  @param None
	#  @return A \f$ 3 \times 3 \f$ numpy rotation matrix containing the current orientation of the gripper w.r.t. the torso.
    def wrist_orientation(self):        

        if self.wrist_orientation_matrix == None:
            [s0, s1, s2, s3, s4, s5, s6] = self.config.s
            [c0, c1, c2, c3, c4, c5, c6] = self.config.c
            [s10]         = self.config.s1_mult1
            [s20,s21, s22, s210]  = self.config.s2_mult1
            [c0s0,c0s1,c0s2,c0s10,c0s20,c0s21,c0s30,c0s31,c0s32,c0s321] = self.config.c0_mult
            [c10, c1s0, c1s1, c1s2, c1s3, c1s10, c1s20,c1s30,c1s32,c1s320] = self.config.c1_mult
            [c20,c21,c22,c210,c2s0,c2s1,c2s2,c2s3,c2s10,c2s20,c2s30,c2s31,c2s310,c21s30,c20s31] = self.config.c2_mult
            [c21s0,c31s0,c321s0]  = self.config.s0_mult
            [c10s2,c20s2,c30s2, c32s2,c310s2]  = self.config.s2_mult2
            [c10s32, c31s20, s54] = self.config.cs_mult
            [s64, s65, c4s6, c5s6, c54s6, s654] = self.config.s6_mult
            [c64, c65, c6s4, c6s5, c654, C6s54] = self.config.c6_mult

            R03 = np.array([[-s20 + c210, -c0s1, -c2s0 - c10s2],
                               [c0s2 + c21s0, -s10, c20 - c1s20],                
                               [-c2s1, -c1, s21]]) 

            R47 = np.array([[-s64 + c654, -c6s4 - c54s6, c4*s5],
                               [c4s6 + c65*s4, c64 - c5*s64, s54],                
                               [-c6s5, s65, c5]]) 

            R34 =  np.array([[  c3,     0,     s3 ],
                                [  s3,     0,    -c3 ],
                                [  0,      1,     0  ]])

            self.wrist_orientation_matrix = np.dot(np.dot(R03, R34), R47)

        return copy.copy(self.wrist_orientation_matrix)

	## Use this function to check if the endeffector is in the target pose or not.
	#  @param norm_precision A float specifying the required precision for the norm of both position and orientation error
	#  @return A boolean: True if the norm of differences between the actual and desired poses of the endeffector are smaller than \a norm_precision, False if not.
    def in_target(self, norm_precision = 0.01):
        if self.use_magiks:
            return self.ik.in_target()
        else:
            if self.in_target_flag == None:
                self.in_target_flag = vecmat.equal(self.xd, self.wrist_position(), epsilon = norm_precision) and vecmat.equal(self.Rd, self.wrist_orientation(), epsilon = norm_precision)
            return self.in_target_flag
	## protected
    def div_phi_err(self):
        '''
        '''
        if self.F == None:
            self.div_theta_err()

        return copy.copy(self.F)

	## protected
    def div_theta_err(self):
        '''
        '''
        if self.E == None:
            [[r11, r12, r13],
            [r21, r22, r23],
            [r31, r32, r33]] = self.Rd

            [s0, s1, s2, s3, s4, s5, s6] = self.config.s
            [c0, c1, c2, c3, c4, c5, c6] = self.config.c

            [s10]         = self.config.s1_mult1
            [s20,s21, s22, s210]  = self.config.s2_mult1
            [s30, s31, s32, s310, s320, s321, s322, s33,s3210] = self.config.s3_mult1
            [c0s0,c0s1,c0s2,c0s10,c0s20,c0s21,c0s30,c0s31,c0s32,c0s321] = self.config.c0_mult
            [c10, c1s0, c1s1, c1s2, c1s3, c1s10, c1s20,c1s30,c1s32,c1s320] = self.config.c1_mult
            [c20,c21,c22,c210,c2s0,c2s1,c2s2,c2s3,c2s10,c2s20,c2s30,c2s31,c2s310,c21s30,c20s31] = self.config.c2_mult
            [c30,c31,c32,c33,c3s0,c3s1,c3s2,c3s3,c3s10,c3s20,c3s21,c3s30,c310,c320,c321,c3210,c32s0,c32s10] = self.config.c3_mult
            [c21s0,c31s0,c321s0]  = self.config.s0_mult
            [c10s1,c20s1,c30s1, c32s1,c320s1] = self.config.s1_mult2
            [c10s2,c20s2,c30s2, c32s2,c310s2]  = self.config.s2_mult2
            [c10s3, c20s3, c30s3, c21s3, c210s3,s332]  = self.config.s3_mult2
            [c10s32, c31s20, s54] = self.config.cs_mult
            [c54, c5s4, c5s5] = self.config.c5_mult
            [s64, s65, c4s6, c5s6, c54s6, s654] = self.config.s6_mult
            [c64, c65, c6s4, c6s5, c654, C6s54] = self.config.c6_mult

            [Q, Q2, d42, d44, a00, d22_plus_d44, foura00d44, alpha] = self.additional_dims

            [r2, ro2, R2, T2, alpha, betta, gamma, sai] = self.target_parameters

            self.E = np.zeros((7,7))
            self.F = np.zeros(7)

            self.E[0,0] = 1.0
            
            self.E[1,3] = - Q*s3

            self.E[2,2] = - 2*c2*s332
            self.E[2,3] = - 2*alpha*c3s3 - betta*s3 - 2*c3*s322

            self.E[3,1] = - self.d2*s1 - (c3*s1 + c2*c1*s3)*self.d4
            self.E[3,2] =   s321*self.d4
            self.E[3,3] =   - (c1s3 + c32s1)*self.d4
            
            self.E[4,1] = r13*(- c20*s31 + c310) + r23*(- c2s310 + c31s0) - r33*(c3s1 + c21*s3)
            self.E[4,2] = - r13*(c2s30 + c10s32) + r23*(c20s3 - c1s320) + r33*s321

            self.E[4,3] = r13*(- c3s20 + c3210 - c0s31) + r23*(c30s2 + c321s0 - s310) + r33*(- c1s3 - c32s1)
            self.E[4,5] = s5

            self.E[5,1] = r13*c0s21 - r23*s210 + r33*c1s2
            self.E[5,2] = r13*(s20 - c210) - r23*(c0s2 + c21s0) + r33*c2s1
            self.E[5,4] = - c4*s5
            self.E[5,5] = - c5*s4

            self.E[6,1] =   r11*(- c20s31 + c310) + r21*(- c2s310 + c31s0) - r31*(c3s1 + c21s3)
            self.E[6,2] = - r11*(c2s30 + c10s32) + r21*(c20s3 - c1s320) + r31*s321
            self.E[6,3] =   r11*(- c3s20 + c3210 - c0s31) + r21*(c30s2 + c321s0 - s310) - r31*(c1s3 + c32s1)
            self.E[6,5] =   c65
            self.E[6,6] = - s65

            self.F[0] = -1.0
            self.F[1] =   2*self.a0*(s0*self.xd[0] - c0*self.xd[1])
            self.F[4] = r13*(- c0s32 - c21s30 - c3s10) + r23*(- s320  + c210s3 + c30s1)  
            self.F[5] = r13*(-c20 + c1s20) - r23*(c2s0 + c10s2)
            self.F[6] = - r11*(c0s32 + c21s30 + c3s10) + r21*(- s320 + c210s3 + c30s1)

        return copy.copy(self.E)
    
    ##  Use this function to get the Joint Redundancy Jacobian vector of the arm.
    #   Since the robot arm has 7 DOF, there is only one redundant parameter.
    #   In this case, the \em Joint Redundancy Jacobian (JRJ) is a vector of 7 elements: 
    #   \f[
    #   J_i = \frac {\partial \theta_i }{\partial \phi}
    #   \f]   
    #   This function should be called when the endeffector is in target. Otherwise, the output will not be correct.
    #   @param None
    #   @return A numpy vector of size 7 containing the derivative of each joint w.r.t. the redundant parameter $f \phi $f
    def joint_redundancy_jacobian(self):   
        if self.JRJ == None:

            E = self.div_theta_err()
            F = self.div_phi_err()
            if genmath.equal(E[1,3],0.0) or genmath.equal(E[2,2],0.0) or genmath.equal(E[3,1],0.0) or genmath.equal(E[4,5],0.0) or genmath.equal(E[5,4],0.0) or genmath.equal(E[6,6],0.0):
                return None
            else:
                self.JRJ = np.zeros(7)

                self.JRJ[0] =   1.0
                self.JRJ[3] = - F[1]/E[1,3]
                self.JRJ[2] = - E[2,3]*self.JRJ[3]/E[2,2]
                self.JRJ[1] = - (E[3,2]*self.JRJ[2] + E[3,3]*self.JRJ[3])/E[3,1]

                self.JRJ[5] = - (F[4] + np.dot(E[4,1:4],self.JRJ[1:4]))/E[4,5]
                #J[5] = - (F[4,0] + F[4,1]*J[1] + F[4,2]*J[2] + F[4,3]*J[3])/F[4,5]
                self.JRJ[4] = - (F[5] + E[5,1]*self.JRJ[1] + E[5,2]*self.JRJ[2] + E[5,5]*self.JRJ[5])/E[5,4] 
                self.JRJ[6] = - (F[6] + np.dot(E[6,1:6],self.JRJ[1:6]))/E[6,6]
                #F60*J0 + F61*J1 + F62*J2 + F63*J3 + F64*J4 + F65*J5 + F66*J6 = 0  ==> J6 = - (F60 + F61*J1 + F62*J2 + F63*J3 + F64*J4 + F65*J5)/ J66

        return copy.copy(self.JRJ)

	## protected 
    def grown_phi(self, eta, k = 0.99, respect_js = False):
        '''
        grows the redundant parameter by eta (adds eta to the current phi=q[0] and returns the new value for phi
        1 - this function does NOT set the configuration so the joint values do not change
        2 - if the grown phi is not in the feasible interval Delta, it will return the closest point in the range with a safety coefficient k so that
        new phi = old phi + eta        : if    eta in Delta
        new phi = old phi + k*Delta_h  : if    eta > Delta_h
        new phi = old phi + k*Delta_l  : if    eta < Delta_l
        '''  
        Delta = self.delta_phi_interval(respect_js = respect_js)

        # if Delta contains more than one interval, then we need to find which interval contains zero

        assert len(Delta) > 0
        j = 0
        while (j < len(Delta)):
           if 0.0 in interval(Delta[j]):
               (dl,dh) = Delta[j] 
           j = j + 1

        if eta in Delta:
            return self.config.q[0] + eta
        elif eta >= dh:   # if zero is not in any of the intervals, this line will raise an error because dh is not defined
            return self.config.q[0] + k*dh
        else:
            assert eta <= dl  # eta must now be smaller than Delta_l
            return self.config.q[0] + k*dl

	## Use this method to get the permission range of x, y and z of the endeffector.
	#  If the desired position for the endeffector(wrist joint center) is outside the permission range, then there is definitely no solution for the IK problem. 
	#  The inverse of this conditional statement is not necessarily true.
	#  @param fixed An array of booleans of size 4, specifying if the corresponding joint is fixed or free
	#  The array size is 4 because only the first four joints (\f$ q_0, q_1, q_2, q_3 \f$) influence the position of the EE
	#  @return an array of 3 intervals, corresponding to x, y and z
    def position_permission_workspace(self,fixed):
        '''
        Example:
        '''
        int_c = []
        int_s = []

        for i in range(0,4):
            if fixed[i]:
                int_c.append(imath.cos(interval(self.config.q[i])))
                int_s.append(imath.sin(interval(self.config.q[i])))
            else:
                int_c.append(imath.cos(interval([self.config.ql[i], self.config.qh[i]])))
                int_s.append(imath.sin(interval([self.config.ql[i], self.config.qh[i]])))

        int_s10    = int_s[1]*int_s[0]
        int_s32    = int_s[3]*int_s[2]        
        int_s31    = int_s[3]*int_s[1]        
        int_s320   = int_s32*int_s[0]

        int_c21    = int_c[2]*int_c[1]
        int_c31    = int_c[3]*int_c[1]
        int_c0s1   = int_c[0]*int_s[1]
        int_c30s1  = int_c[3]*int_c0s1
        int_c0s32  = int_s32*int_c[0]
        int_c21s3  = int_c21*int_s[3]
        int_c21s30 = int_c21s3*int_s[0]
        int_c210s3 = int_c21s3*int_c[0]
        int_c2s31  = int_s31*int_c[2]
        int_c3s10  = int_s10*int_c[3]

        int_X =  int_c[0]*interval(self.a0) + int_c0s1*interval(self.d2) + (int_c30s1 + int_c210s3 - int_s320)*interval(self.d4)

        int_Y =  int_s[0]*interval(self.a0) + int_s10*interval(self.d2) + (int_c3s10 + int_c0s32 + int_c21s30)*interval(self.d4)

        int_Z =  int_c[1]*interval(self.d2) + (int_c31 - int_c2s31)*interval(self.d4)

        return [int_X, int_Y, int_Z]
        
    ## returns the current configuration and sets the given configuration. 
    #  This function is used to restore the saved old configuration 
    #  @param q_old A numpy vector of size 7 containing the arm joint values to be set
    #  @return A numpy vector of size 7 containing the current joint values
    def restore_config(self, q_old):
        q = np.copy(self.config.q)
        assert self.set_config(q_old)
        return q 

    ## Use this function to get the displacement between the actual and desired wrist poses
    #  The metric returns sum of the norms of position difference Nd Frobenus norm of the orientation difference.
    #   \f[
    #   d(x, x_d) = \Sigma_{i = 1}^{3}(x_i - x_{di})^2 + \Sigma_{i = 1}^{3} \Sigma_{j = 1}^{3}(r_{ij} - r_{dij})^2
    #   \f] 
    #   @param None
    #   @return A scalar value   
    def pose_metric(self):
        d  = np.linalg.norm(self.xd - self.wrist_position())
        d += np.linalg.norm(self.Rd - self.wrist_orientation())
        return d
        
    ##  Finds the optimal value for the redundant parameter \f$ \phi \f$ that minimizes the cost function and returns the 
     #  joints corresponding with optimal redundant parameter value. This function does not change the object configuration.
     #  @param show If True, the values of redundant parameter and the objective function are printed on the console
     #  @return A numpy vector of size 7 containing the optimal arm joint values   
    def optimal_config(self, show = False):
        keep_q     = np.copy(self.config.q)
        counter = 0
        while True:
            J   = self.joint_redundancy_jacobian()
            e   = self.config.midrange_error()
            if show:
                print
                print "Iteration             : ", counter
                print "Redundant Parameter   : ", self.config.q[0]
                print "Objective Function    : ", self.config.objective_function()
            if J == None:
                if show:
                    print "No Jacobian! Optimum phi = ", self.config.q[0]
                return self.restore_config(keep_q)
            P   = self.config.w*J    
            den = np.dot(P.T, P)
            if genmath.equal(den, 0.0):
                if show:
                    print "Division by Zero! Optimum phi = ", self.config.q[0]
                return self.restore_config(keep_q)
            Delta_phi  = - np.dot(P.T, e) / den
            old_err    = self.config.objective_function()
            new_phi    = self.grown_phi(Delta_phi)
            q          = self.IK_config(new_phi)
            if q == None:
                if show:
                    print "No solution for the updated phi! Optimum phi = ", self.config.q[0]
                return self.restore_config(keep_q)

            if not self.set_config(q):
                if show:
                    print "Solution found but not feasible! This should not happen. Optimum phi = ", self.config.q[0]
                return self.restore_config(keep_q)

            if show:
                print "A better phi is found : ", new_phi

            new_err = self.config.objective_function()

            if new_err > old_err - 0.01:
                if show:
                    print "Error not reduced any more. Optimum phi: ", self.config.q[0]
                return self.restore_config(keep_q)

            counter = counter + 1
            
    ## Moves the joints within the solution manifold towards optimum configuration. The optimization is based on the defined cost function.
    #  The joint change respects given joint speed limit
    #  @parameter max_speed A scalar float value specifying the maximum joint speed in Radians per second. The default is infinity
    #  @step_time A scalar float value specifying time to reach or step time. The default value is 0.1 sec.
    #  @return A boolean: True if successfuly reduced the cost function, False if not
    def moveto_optimal(self, max_speed = genmath.infinity, step_time = 0.1):
        if self.in_target():
            q0   = np.copy(self.config.q)
            err  = self.config.objective_function()
            J    = self.joint_redundancy_jacobian()
            e    = self.config.midrange_error()

            P   = self.config.w*J    
            den = np.dot(P.T, P)
            if genmath.equal(den, 0.0):
                genpy.show('Division by Zero! Optimum phi = ' + str(self.config.q[0]), self.silent)
                return False

            delta_phi  = - np.dot(P.T, e) / den
            phi        = self.grown_phi(delta_phi, respect_js = True)
                        
            q = self.IK_config(phi)
            if q != None:
                if self.set_config(q):
                    return True

            return False        

        else:
            genpy.show(self.silent, "Endeffector not in target !")
            return False

    ##  This function should be used when given redundant parameter \f$ \phi \f$ is within the given permission set (PS) 
     #  and yet there is no IK solution for it. The neighborhood of \f$ \phi \f$ is searched for a feasible IK solution.
     #  @parameter phi A scalar float specifying the desired value of the redundant parameter \f$ \phi \f$ (No Default value set)
     #  @parameter PS A list of variables of type interval. It can be the output of function 
     #  If a solution is found, the config is set and a True is returned, 
     #  If all permission set is searched with no solution, False is returned and the object configuration will not change.
    def closest_feasible_phi(self, phi, PS, increment = math.pi/360.0):
        (phi_l, phi_h) = genmath.accommodating_interval(phi, PS)

        assert (phi < phi_h) and (phi > phi_l)

        phi_up   = phi
        phi_down = phi
        while True:
            stay_up   = (phi_up < phi_h)
            stay_down = (phi_down > phi_l)

            if stay_up:
                phi_up = phi_up + increment
                q = self.IK_config(phi_up)
                if q != None:
                    if self.set_config(q):
                        return True

            if stay_down:
                phi_down = phi_down - increment
                q = self.IK_config(phi_down)
                if q != None:
                    if self.set_config(q):
                        return True

            if not (stay_up or stay_down):             
                # Nothing found :-(
                return False
                
    ##  Solves the position based inverse kinematics and returns a direction in the jointspace towards the solution (joint correction)   
     #  @param phi A scalar float specifying the desired value of the redundant parameter. 
     #  If set as None (default), the current value of the shoulder-pan joint is replaced.
     #  @param optimize A boolean If True the IK solution minimize the defined cost function, otherwise the closest solution will be returned. The default is False.
     #  @return A numpy vector of size 7, containing the direction of joint correction.
    def ik_direction(self, phi = None, optimize = False):
        if self.use_magiks:
            return self.ik.ik_direction()
        else:    
            q0 = np.copy(self.config.q)
            if self.goto_target(phi = phi, optimize = optimize):
                q  = self.restore_config(q0)
                return trig.angles_standard_range(q - q0)
            else:
                return np.zeros(7)
    
    ## Given a direction for joint correction and a joint speed limit, this function returns the maximum feasible value by which the joints can move in this direction.
     # @parameter direction A numpy vector of size 7 specifying the desired direction for joints to move
     # @parameter speed_limit A scalar float specifying the maximum feasible joint speed
     # @parameter step_time A scalar float specifying the step time 
     # @return A scalar float containing the maximum feasible norm of joint correction in the specified direction.   
    def feasible_joint_stepsize(self, direction, max_speed, time_step = 0.1):
        speed_limit = abs(speed_limit)
        q0          = np.copy(self.config.q)
        (el, eh)  = self.config.joint_stepsize_interval(direction = direction, max_speed = max_speed, delta_t = time_step) 
        assert el < 1.0
        if eh > 1.0:
            eh = 1.0
        return eh

    def move_joints_towards(self, direction, max_speed, step_time = 0.1):
        q = q0 + direction*self.feasible_joint_stepsize(direction, max_speed, step_time)
        return self.set_config(q)

    def moveto_target(self, optimize = False):
        if self.use_magiks:
            if self.ik.moveto_target():
                self.sync_from_magiks()
                return True
            else:
                return False
        else:
            q0        = np.copy(self.config.q)
            err       = self.pose_metric()
            jdir      = self.ik_direction(phi = self.config.q[0], optimize = optimize)
            err_reduced    = False
            if np.linalg.norm(jdir) > 0.0001:
                (el, eh)  = self.config.joint_stepsize_interval(direction = jdir, max_speed = self.max_js, dt = self.dt) 
                assert el < 0.0
                if eh > 1.0:
                    eh = 1.0
                q = q0 + eh*jdir
                if self.set_config(q):
                    if self.pose_metric() <  err:
                        err_reduced = True
                    else:
                        self.set_config(q0)
            else:
                return True
            return err_reduced

    def goto_target(self, phi = None, optimize = False, show = False):    
        '''
        Finds the inverse kinematic solution for the given redundant parameter phi
        is phi is not feasible, the solution corresponding to the closest phi is returned.
        If argument optimize is True, the solution will be optimized 
        so that the joint values will be as close as possible to self.config.qm
        If phi is not given, current q[0] will be selected as phi
        
        The new joint angles will be set if all the kinematic equations are satisfied. 
        All kinematic parameters will be updated.
        '''
        if self.use_magiks:
            if self.ik.goto_target():
                self.sync_from_magiks()
            else:
                self.ik.set_config(self.config.q)
                print "\n Magiks could not find a solution ! \n"
        else:
            # Finding a feasible phi (theta0)
            if phi == None:
                phi     = self.config.q[0]

            PS = self.permission_set_position()
            if show:
                print "Permission Set for phi = ", PS
                print "Initial phi            = ", phi
            
            if len(PS) == 0:
                if show:
                    print "len(PS) = 0"
                    print "The target is out of workspace! No solution found."
                return False
            else:
                if not (phi in PS):
                    phi = genmath.closest_border(phi, PS, k = 0.01)
                    if show:
                        print "Phi is not in PS"
                        print "Closest phi in PS:", phi

                q = self.IK_config(phi) 
                if q == None:
                    if show:
                        print phi, " is not a feasible phi. No solution found"
                    if not self.closest_feasible_phi(phi, PS):
                        if show:
                            print "The target is out of workspace! No solution found."
                        return False
                    if show:
                        print "Next phi: ", self.config.q[0]
                else:    
                    if not self.set_config(q):
                        if show:
                            print "Not expected to see. Solution exists but not feasible!"
                        return False

                # when you reach here, the feasible q has been set    
                
                if optimize:
                    self.set_config(self.optimal_config())
                 
        return self.in_target()
        
    def all_IK_solutions(self, phi):    
        '''
        Finds all the feasible solutions of the Inverse Kinematic problem for given redundant parameter "phi"
        "phi" is the value of the first joint angle "q[0]"
        This function does NOT set the configuration so the joints do not change
        '''
        if not self.config.joint_in_range(0, phi):
            print "IK_config error: Given theta0 out of feasible range"
            return []

        solution_set = []

        c0 = math.cos(phi)
        s0 = math.sin(phi)

        [Q, Q2, d42, d44, a00, d22_plus_d44, foura00d44, alpha] = self.additional_dims
        [r2, ro2, R2, T2, alpha, betta, gamma, sai] = self.target_parameters

        u  = c0*self.xd[0] + s0*self.xd[1]
        v  = (2*self.a0*u - R2)/Q

        v2 = v**2
        A  = self.d2 + v*self.d4

        if genmath.equal(v, 1.0):
            #"Singular Point"
            return []
            '''
            In this case, a singularity happens
            '''
        elif (v > 1.0) or (v < -1.0): 
            #"Given pose out of workspace"
            return []
        else:
            i  = 0
            tt3 = trig.arccos(v)

            while (i < 2):
                theta3 = (2*i - 1)*tt3  # theta3 is certainly in standard range
                if self.config.joint_in_range(3,theta3):
                    s3     = math.sin(theta3)
                    c3     = v
                    c30    = c3*c0
                    B      = self.d4*s3
                    T34 = genkin.transfer_DH_standard( 0.0 , math.pi/2, 0.0, 0.0, theta3)
                    R34 = T34[0:3,0:3]

                    w2 = (alpha*v2 + betta*v + gamma)/(1 - v2)
                    if genmath.equal(w2, 1.0):
                        w2 = 1.0
                    elif genmath.equal(w2, 0.0):
                        w2 = 0.0
                    elif w2 < 0.0:
                        print "IK_config error: w^2 is negative, This should never happen! Something is wrong!"
                        assert False 

                    w  = math.sqrt(w2)
                    if (w < 1.0):
                        m = 0              
                        while (m < 2) and (not ((w == 0.0) and (m == 1))):  # beghole emam: intor nabashad ke ham w sefr bashad va ham m yek. 
                            s2  = (2*m - 1)*w
                            s20 = s2*s0 
                            c0s2= c0*s2  
                            
                            tt2 = trig.arcsin(s2)
                            j = 0
                            while (j < 2):
                                theta2 = trig.angle_standard_range(math.pi*(1 - j) + (2*j - 1)*tt2)
                                if self.config.joint_in_range(2, theta2):
                                    c2     = math.cos(theta2)
                                    c20    = c2*c0
                                    c2s0   = c2*s0
                                    E      = B*s2
                                    F      = B*c2
                                    R1     = math.sqrt(A**2 + F**2)
                                    sai1   = math.atan2(F,A)
                                    R1_nul = genmath.equal(R1, 0)
                                    if not R1_nul:
                                        z_R1 = self.xd[2]/R1    
                                        flg  = (z_R1 < 1.0) and (z_R1 > - 1.0)
                                    else:
                                        flg = False

                                    if flg:
                                        tt1 = trig.arccos(self.xd[2]/R1)
                                        k = 0                
                                        while (k < 2):
                                            theta1 = (2*k - 1)*tt1 - sai1 
                                            if self.config.joint_in_range(1, theta1):
                                                s1   = math.sin(theta1)
                                                c1   = math.cos(theta1)
                                                
                                                As1Fc1 = self.a0 + A*s1 + F*c1
                                                X      = c0*As1Fc1 - E*s0
                                                Y      = s0*As1Fc1 + E*c0
                                                Z      = A*c1 - F*s1 

                                                u2 = s2*s3*self.d4
                                                u3 = c2*s3*self.d4
                                                u4 = self.d2+ c3*self.d4
                                                u1 = self.a0 + u3*c1 + u4*s1
                                                AA = np.array([[alpha*d44, d44*betta - 2*d42 , - self.d2**2 + d44*(gamma -1)],
                                                                   [0.0 , 2*self.xd[2]*self.d4 , 2*self.xd[2]*self.d2],   
                                                                   [- d44*(1+alpha) , -betta*d44 , - self.xd[2]**2 + d44*(1-gamma) ]])
                                                lnda = np.array([c1*c1, c1, 1.0])
                                                vvct = np.array([v*v, v, 1.0])

                                                if vecmat.equal(self.xd, [X,Y,Z]):
                                                    if self.orientation_respected:
                                                        R03 = np.array([[c20*c1 - s20, -c0*s1, -c2s0 - c1*c0*s2],
                                                                           [c0s2 + c1*c2*s0, -s1*s0, c20 - c1*s20],   
                                                                           [-c2*s1, -c1, s2*s1 ]])

                                                        R04 = np.dot(R03, R34)
                                                        R47 = np.dot(R04.T, self.Rd)
                                                        tt5 = trig.arccos(R47[2,2])
                                                        l = 0
                                                        while (l < 2):
                                                            theta5 = (2*l - 1)*tt5 # theta5 is certainly in standard range
                                                            if self.config.joint_in_range(5, theta5):
                                                                s5     = math.sin(theta5)
                                                                c5     = math.cos(theta5)
                                                                if genmath.equal(s5,0):
                                                                    assert genmath.equal(R47[2,0], 0)
                                                                    # "Singular Point"
                                                                    return []
                                                                    '''
                                                                    In this case, only sum of theta4 + theta6 is known 
                                                                    '''
                                                                else:
                                                                    c6     = - R47[2,0]/s5
                                                                    s6     =   R47[2,1]/s5
                                                                    c4     =   R47[0,2]/s5
                                                                    s4     =   R47[1,2]/s5

                                                                    theta6 = trig.arcsincos(s6, c6)
                                                                    theta4 = trig.arcsincos(s4, c4)

                                                                    assert genmath.equal(R47[1,0] ,  c4*s6 + c5*c6*s4)
                                                                    assert genmath.equal(R47[1,1] ,  c4*c6 - c5*s4*s6)
                                                                    assert genmath.equal(R47[0,0] ,  -s4*s6 + c4*c5*c6)
                                                                    assert genmath.equal(R47[0,1] ,  -c6*s4 - c4*c5*s6)

                                                                    assert self.config.joint_in_range(4, theta4)    
                                                                    assert self.config.joint_in_range(6, theta6)    

                                                                    solution_set.append(np.array([phi, theta1, theta2, theta3, theta4, theta5, theta6]))
                                                            l = l + 1
                                                    else:
                                                        solution_set.append(np.array([phi, theta1, theta2, theta3, self.config.q[4], self.config.q[5], self.config.q[6]]))
                                            k = k + 1
                                j = j + 1
                            m = m + 1
                i = i + 1 
        return solution_set

    def IK_config(self, phi):    
        '''
        Finds the solution of the Inverse Kinematic problem for given redundant parameter "phi"
        In case of redundant solutions, the one corresponding to the lowest objective function is selected.
        property ofuncode specifies the objective function:
            ofuncode = 0 (Default) the solution closest to current joint angles will be selected 
            ofuncode = 1 the solution corresponding to the lowest midrange distance is selected
        This function does NOT set the configuration so the joints do not change
        ''' 
        solution_set = self.all_IK_solutions(phi)

        if len(solution_set) == 0:
            # print "IK_config error: No solution found within the feasible joint ranges for given phi. Change the target or redundant parameter"
            print ".",
            return None

        delta_min = 1000
        for i in range(0, len(solution_set)):
            solution = solution_set[i]
            if self.ofuncode == 0:
                delta    = np.linalg.norm(trig.angles_standard_range(solution - self.config.q))
            elif self.ofuncode == 1:
                P = trig.angles_standard_range(solution - self.config.qm)*self.config.w
                delta = np.dot(P.T,P)
            else:
                print "IK_config error: Value ",self.ofuncode," for argument ofuncode is not supported"
                assert False
 
            if delta < delta_min:
                delta_min = delta
                i_min = i

        return solution_set[i_min]

    def ts_project(self,  js_traj, phi_start = 0.0, phi_end = None):
        '''
        projects the given jointspace trajectory into the taskspace
        The phase starts from phi_start and added by delta_phi in each step.
        if at any time the joint values are not feasible, the process is stopped.
        '''
        
        if phi_end == None:
            phi_end = js_traj.phi_end

        ori_traj = trajlib.Orientation_Trajectory_Polynomial()
        pos_traj = trajlib.Trajectory_Polynomial()
        if phi_end > js_traj.phi_end:
            phi_end = js_traj.phi_end

        phi = phi_start
        stay = True
        while stay:
            if (phi > phi_end) or genmath.equal(phi, phi_end, epsilon = 0.1*self.dt):
                phi = phi_end
                stay = False
            js_traj.set_phi(phi)
            if self.set_config(js_traj.current_position):
                pos_traj.add_point(phi - phi_start, self.wrist_position())
                ori_traj.add_point(phi - phi_start, geo.Orientation_3D(self.wrist_orientation()))
            phi = phi + self.dt

        return (pos_traj, ori_traj)

    def js_project(self,  pos_traj, ori_traj = None, phi_start = 0.0, phi_end = None, relative = True, traj_capacity = 2, traj_type = 'regular'):
        '''
        projects the given taskspace pose trajectory into the jointspace using analytic inverse kinematics.
        The phase starts from phi_start and added by self.dt in each step.
        at any time, if a solution is not found, the process stops
        '''
        self.config.qm = 0.5*(self.config.ql + self.config.qh)
        keep_q = np.copy(self.config.q)

        if ori_traj == None:
            ori_traj = trajlib.Orientation_Path()
            ori_traj.add_point(0.0, geo.Orientation_3D(self.wrist_orientation()))
            ori_traj.add_point(pos_traj.phi_end, geo.Orientation_3D(self.wrist_orientation()))

        if phi_end == None:
            phi_end = min(pos_traj.phi_end, ori_traj.phi_end)

        if (phi_end > pos_traj.phi_end) or (phi_end > ori_traj.phi_end):
            phi_end = min(pos_traj.phi_end, ori_traj.phi_end)

        if traj_type == 'regular':
            jt = trajlib.Trajectory(dimension = 7, capacity = traj_capacity)
        elif traj_type == 'polynomial':
            jt = trajlib.Trajectory_Polynomial(dimension = 7, capacity = traj_capacity)
        else:
            assert False, "\n Unknown Trajectory Type \n"

        jt.vel_max  = self.max_js
        jt.acc_max  = self.max_ja
        jt.pos_max  = self.config.qh
        jt.pos_min  = self.config.ql

        phi   = phi_start
        pos_traj.set_phi(phi)
        ori_traj.set_phi(phi)
        if relative:
            p0    = self.wrist_position() - pos_traj.current_position
        else:
            p0    = np.zeros(3)
            self.set_target(pos_traj.current_position, ori_traj.current_orientation['matrix'])
            self.goto_target(optimize = True)

        jt.add_position(0.0, pos = self.config.q)
        
        stay      = True

        while stay:
            phi = phi + self.dt

            if (phi > phi_end) or genmath.equal(phi, phi_end, epsilon = 0.1*self.dt):
                phi = phi_end
                stay = False

            pos_traj.set_phi(phi)
            ori_traj.set_phi(phi)
            p = p0 + pos_traj.current_position
            self.set_target(p, ori_traj.current_orientation.matrix())

            self.config.qm = np.copy(self.config.q)

            err_reduced = self.moveto_target(optimize = True)
            self.set_target(self.wrist_position(), self.wrist_orientation())
            optim_success = self.moveto_optimal()
            if err_reduced:
                jt.add_position(phi = phi - phi_start, pos = np.copy(self.config.q))
            else:
                print ":",

        if traj_type == 'polynomial':
            jt.interpolate()
            # jt.consistent_velocities()

        self.config.qm = 0.5*(self.config.qh + self.config.ql)
        self.set_config(keep_q)

        return jt

    def sync_from_magiks(self):
        ers = 'Joint values computed from magiks are not feasible'
        assert self.set_config(self.ik.q), genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, ers)

    def permission_set_position(self):
        """
        This function finds and returns the set from which q0 is allowed to be chosen so that 
        joint angles q0 , q2 and q3 in the analytic solution to the inverse kinematic problem are in their range. 
        Consider that being q0 in the permission set, does not guarantee that all q0, q2 and q3 are in their ranges, but
        it means that if q0 is out of permission set, one of the joints q0, q2 or q3 will definitely be out of their 
        feasible range.
        In other words, q0 being in perm. set, is a necessary but not sufficient condition for three joints 
        q0, q2 and q3 to be in their range.
        Permission set is broader than "confidence set" which ensures all q0, q2 and q3 to be in range.
        (has both necessary and sufficient conditions)
        The output depends on the defined joint limits and the desired endeffector pose but does not depend 
        on the current position of the joints.
        """
        if self.Phi != None:
            return self.Phi

        [r2, ro2, R2, T2, alpha, betta, gamma, sai] = self.target_parameters
        [Q, Q2, d42, d44, a00, d22_plus_d44, foura00d44, alpha] = self.additional_dims

        # feasibility set for v imposed by theta1

        int_theta1 = interval([self.config.ql[1],self.config.qh[1]])
        int_lnda   = imath.cos(int_theta1)
        int_lnda2   = int_lnda**2

        AA = np.array([[alpha*d44, d44*betta - 2*d42 , - self.d2**2 + d44*(gamma -1)],
                          [0.0 , 2*self.xd[2]*self.d4 , 2*self.xd[2]*self.d2],   
                          [- d44*(1+alpha) , -betta*d44 , - self.xd[2]**2 + d44*(1-gamma) ]])

        int_alpha = interval(AA[0,0])*int_lnda2 + interval(AA[1,0])*int_lnda + interval(AA[2,0])
        int_betap = interval(AA[0,1])*int_lnda2 + interval(AA[1,1])*int_lnda + interval(AA[2,1])
        int_gamma = interval(AA[0,2])*int_lnda2 + interval(AA[1,2])*int_lnda + interval(AA[2,2])
    
        (alpha_l, alpha_h) = int_alpha[0]
        (betap_l, betap_h) = int_betap[0]
        (gamma_l, gamma_h) = int_gamma[0]

        Vp_1l = genmath.solve_quadratic_inequality(- alpha_l, - betap_l, -gamma_l)
        Vp_1h = genmath.solve_quadratic_inequality(  alpha_h,   betap_h,  gamma_h)
        Vn_1l = genmath.solve_quadratic_inequality(- alpha_l, - betap_h, -gamma_l)
        Vn_1h = genmath.solve_quadratic_inequality(  alpha_h,   betap_l,  gamma_h)

        V1 = (Vp_1l & Vp_1h & interval([0.0,1.0])) | (Vn_1l &  Vn_1h & interval([-1.0,0.0]))

        # Finding wh,wl:

        int_theta2 = interval([self.config.ql[2],self.config.qh[2]])
        int_w   = imath.sin(int_theta2)**2
        (wl, wh) = int_w[0]

        #Finding V2l, V2h

        V2l = genmath.solve_quadratic_inequality(alpha + wl, betta, gamma - wl) & interval([-1.0, 1.0])
        V2h = genmath.solve_quadratic_inequality(- alpha - wh, - betta, wh - gamma) & interval([-1.0, 1.0])

        #Finding V2

        V2 = V2l & V2h

        #Finding V3

        int_theta3 = interval([self.config.ql[3],self.config.qh[3]])
        V3         = imath.cos(int_theta3)

        #Finding V

        V = V3 & V2 & V1

        #Finding Ui

        denum = 2*self.a0*math.sqrt(r2)
        a     = R2/denum
        b     = 2*self.d2*self.d4/denum

        Phi1_3 = interval()
        nv = len(V)
        for i in range(0, nv):
            Ui = interval(a) - interval(b)*interval(V[i])
            (uli, uhi) = Ui[0]            

            zl = trig.arcsin(uli)
            zh = trig.arcsin(uhi)

            B1 = trig.standard_interval(zl - sai, zh - sai)
            B2 = trig.standard_interval(math.pi- zh - sai, math.pi- zl - sai)

            Phi1_3 = Phi1_3 | B1 | B2    

        #Finding P_phi

        Phi0 = interval([self.config.ql[0], self.config.qh[0]])
        
        self.Phi = genmath.connect_interval(Phi0 & Phi1_3)

        return self.Phi

    def delta_phi_interval(self, respect_js = False):
        '''
        Updates the feasible interval for the growth of the redundant parameter according to
        the specified joint limits and the current value of the joints
        '''
        if self.Delta == None:
            J    = self.joint_redundancy_jacobian()
            self.Delta  = self.permission_set_position() - interval(self.config.q[0])  # set initial Delta based on the position permission set for phi

            for i in range(0, 7):
                if (not genmath.equal(J[i], 0.0)) and (not genmath.equal(self.config.w[i], 0.0)):  # if Ji and wi are not zero
                    d1   = (self.config.ql[i] - self.config.q[i])/J[i]    
                    d2   = (self.config.qh[i] - self.config.q[i])/J[i]    
                    dli1 = genmath.binary_choice(d1,d2,J[i])
                    dhi1 = genmath.binary_choice(d2,d1,J[i])

                    d    = self.dt*self.max_js/J[i]

                    if respect_js:    
                        dli2 = genmath.binary_choice(- d,   d, J[i])
                        dhi2 = genmath.binary_choice(  d, - d, J[i])
                        assert dli2 <= 0.0
                        assert dhi2 >= 0.0
                    else:
                        dli2 = - np.inf
                        dhi2 =   np.inf
        

                    if genmath.equal(dli1, 0.0):
                        dli1 = 0.0
                    if genmath.equal(dhi1, 0.0):
                        dhi1 = 0.0

                    assert dli1 <= 0.0
                    assert dhi1 >= 0.0

                    self.Delta   = self.Delta & interval([dli1, dhi1]) & interval([dli2, dhi2])
                    if len(self.Delta) == 0:
                        print "Error: self.Delta is empty:"
                        print "Initial Delta : ", self.permission_set_position() - interval(self.config.q[0])
                        print "interval([dli1, dhi2]): ", interval([dli1, dhi1])
                        print "interval([dli1, dhi2]): ", interval([dli2, dhi2])

        return self.Delta
        
