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
#  @version     	4.0 
#
#  Last Revision:  	30 December 2014

'''
Changes from ver 3.0:
    1- Comments added for doxygen
    2- function joint_jacobian() renamed to joint_redundancy_jacobian() 
    3- property JJ renamed to JRJ
'''

import copy, time, math
import numpy as np
import packages.nima.mathematics.general as gen
from interval import interval, inf, imath
from sets import Set

import packages.nima.robotics.kinematics.kinematicpy.general as genkin
import packages.nima.robotics.kinematics.kinematicpy.inverse_kinematics as iklib
import packages.nima.robotics.kinematics.kinematicpy.manipulator_library as maniplib
import packages.nima.robotics.kinematics.joint_space.configuration as configlib
import packages.nima.robotics.kinematics.task_space.trajectory as trajlib
import packages.nima.robotics.kinematics.joint_space.trajectory as jtrajlib
import packages.nima.mathematics.general as gen
import packages.nima.mathematics.trigonometry as trig
import packages.nima.mathematics.vectors_and_matrices as vecmat

drc        = math.pi/180.00
default_ql = drc*np.array([-130.0, 70.0 , -180.0,   0.0, -180.0,   0.0, -180.0])
default_qh = drc*np.array([  40.0, 170.0,   44.0, 130.0,  180.0, 130.0,  180.0])
default_W  = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

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
        if abs(qi - self.ql[i]) < gen.epsilon:
            qi = self.ql[i]
        if abs(qi - self.qh[i]) < gen.epsilon:
            qi = self.qh[i]

        return ((qi <= self.qh[i]) and (qi >= self.ql[i]))
    
	## This function is used to check if all values of the given joint array are in their feasibility range
	#  @param qd A numpy array of size 7 containing the values to be checked
	#  @return A boolean: If The given joints "qd" are out of the range specified by properties: ql and qh, returns False, otherwise returns True
    def all_joints_in_range(self, qd):
        flag = True
        for i in range(0, 7):
            if not gen.equal(self.w[i], 0.0):
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
    def joint_stepsize_interval(self, direction, max_speed = gen.infinity, delta_t = 0.001):
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
            if not gen.equal(direction[i], 0.0):
                if (self.w[i] != 0.0):
                    a = (self.ql[i] - self.q[i])/direction[i]    
                    b = (self.qh[i] - self.q[i])/direction[i]
                    etta_l.append(gen.sign_choice(a, b, direction[i]))
                    etta_h.append(gen.sign_choice(b, a, direction[i]))

                a = delta_t*max_speed/direction[i]
                etta_l.append(gen.sign_choice(-a, a, direction[i]))
                etta_h.append(gen.sign_choice( a,-a, direction[i]))

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
                if gen.equal(self.w[i], 0.0):
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
            return True
        else:
            print "Error from PR2_ARM.set_config(): Given joints are not in their feasible range"
            print "Lower Limit  :", self.ql
            print "Upper Limit  :", self.qh
            print "Desired Value:", qd
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
    def __init__(self, a0 = 0.1, d2 = 0.4, d4 = 0.321, ql = default_ql, qh = default_qh, W = default_W):

		## A boolean specifying if orientation of the endeffector should be respected or not in finding the Inverse Kinematic solution
        self.orientation_respected = True
        
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
        qd  = numpy.zeros(7)	
        arm.set_config(qd)	
        print arm.config.q
        print arm.wrist_position()
        print arm.wrist_orientation()	
        '''
        if self.config.set_config(qd):
            self.wrist_position_vector    = None
            self.wrist_orientation_matrix = None
            self.JRJ          = None
            self.E           = None
            self.F           = None
            self.Delta       = None
            '''
            if self.velocity_based_ik_required:
                self.ik.configuration.q = np.copy(qd)
                self.ik.configuration.initialize()
                self.ik.forward_update()
            '''
            return True
        else:
            print "Could not set the given joints."
            return False

	## Sets the endeffector target to a desired position and orientation.
	#  @param target_position    A numpy array of 3 elements specifying the desired endeffector position
	#  @param target_orientation A \f$ 3 \times 3 \f$ numpy rotation matrix representing the desired endeffector orientation
	#  @return A boolean: True  if the given pose is successfully set, False if the given pose is not successfully set because of an error
    #          (If False is returned, properties self.xd and self.RD will not chenge)
    def set_target(self, target_position, target_orientation):

        assert gen.equal(np.linalg.det(target_orientation), 1.0)
        self.xd = target_position
        self.Rd = target_orientation

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
        return vecmat.equal(self.xd, self.wrist_position(), epsilon = norm_precision) and vecmat.equal(self.Rd, self.wrist_orientation(), epsilon = norm_precision)

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
            if gen.equal(E[1,3],0.0) or gen.equal(E[2,2],0.0) or gen.equal(E[3,1],0.0) or gen.equal(E[4,5],0.0) or gen.equal(E[5,4],0.0) or gen.equal(E[6,6],0.0):
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
    def grown_phi(self, eta, k = 0.99):
        '''
        grows the redundant parameter by eta (adds eta to the current phi=q[0] and returns the new value for phi
        1 - this function does NOT set the configuration so the joint values do not change
        2 - if the grown phi is not in the feasible interval Delta, it will return the closest point in the range with a safety coefficient k so that
        new phi = old phi + eta        : if    eta in Delta
        new phi = old phi + k*Delta_h  : if    eta > Delta_h
        new phi = old phi + k*Delta_l  : if    eta < Delta_l
        '''  
        Delta = self.delta_phi_interval()

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
        
    def restore_config(self, q_old):
        q = np.copy(self.config.q)
        assert self.set_config(q_old)
        return q 

    def pose_metric(self):
        d  = np.linalg.norm(self.xd - self.wrist_position())
        d += np.linalg.norm(self.Rd - self.wrist_orientation())
        return d
        
    def optimal_config(self, show = False):
        '''
        Returns the optimal value for phi that minimizes the cost function. This function does not change the configuration
        If show = True, the values of phi and objective function are printed on the console
        '''
        
        keep_q     = np.copy(self.config.q)
        counter = 0
        while True:
            J   = self.joint_redundancy_jacobian()
            e   = self.config.midrange_error()
            if show:
                print "Iteration          : ", counter
                print "Value of redundancy: ", self.config.q[0]
                print "Objective Function : ", self.config.objective_function()
            if J == None:
                if show:
                    print "No Jacobian! Optimum phi = ", self.config.q[0]
                return self.restore_config(keep_q)
            P   = self.config.w*J    
            den = np.dot(P.T, P)
            if gen.equal(den, 0.0):
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
                print "A better phi is found: ", new_phi

            new_err = self.config.objective_function()

            if new_err > old_err - 0.01:
                if show:
                    print "Error is not reduced any more. Optimum phi = ", self.config.q[0]
                return self.restore_config(keep_q)

            counter = counter + 1
            
    def reduce_error(self, max_speed = gen.infinity, ttr = 0.1):
        if self.in_target():
            q0        = np.copy(self.config.q)
            err       = self.config.objective_function()
            q_opt     = self.optimal_config()
            (el, eh)  = self.config.joint_stepsize_interval(direction = q_opt - q0, max_speed = max_speed, delta_t = ttr) 
            assert el < 1.0
            if eh > 1.0:
                eh = 1.0
            q = q0 + eh*(q_opt - q0)
            if self.set_config(q):
                if self.config.objective_function() > err:
                    assert self.set_config(q0)
                    return False
            return True
        else:
            print "Error from PR2_ARM.reduce_error(): The endeffector is not in target."
            print self.wrist_position() - self.xd
            print self.wrist_orientation() - self.Rd
            return False

    def closest_feasible_phi(self, phi, PS, increment = math.pi/360.0):
        '''
        This function should be used when given phi is in permission set PS and yet
        there is no IK solution for it. So the neighborhood of phi is searched for a feasible IK solution
        If a solution is found, the config is set and a True is returned, 
        If all the permission set is searched with no solution, False is returned and the configuration does not change
        '''
        (phi_l, phi_h) = gen.accommodating_interval(phi, PS)

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
                
    def ik_direction(self, phi = None, optimize = False):
        '''
        Solves the position based inverse kinematics and returns a direction in the jointspace (joint correction)
        The configuration does not change
        '''
        q0 = np.copy(self.config.q)
        if self.inverse_update(phi = phi, optimize = optimize):
            q  = self.restore_config(q0)
            return trig.angles_standard_range(q - q0)
        else:
            return np.zeros(7)
    
    def feasible_joint_stepsize(self, direction, max_speed, ttr):
        q0        = np.copy(self.config.q)
        err       = self.pose_metric()
        (el, eh)  = self.config.joint_stepsize_interval(direction = direction, max_speed = max_speed, delta_t = ttr) 
        assert el < 1.0
        if eh > 1.0:
            eh = 1.0
        return eh

    def move_joints_towards(self, direction, max_speed, ttr):
        q = q0 + direction*self.feasible_joint_stepsize(direction, max_speed, ttr)
        return self.set_config(q)

    def move_towards_target(self, max_speed, ttr, phi = None, optimize = False, show = False):
        q0        = np.copy(self.config.q)
        err       = self.pose_metric()
        jdir      = self.ik_direction(phi = phi, optimize = optimize)
        # print "direction = ", jdir
        if np.linalg.norm(jdir) > 0.001:
            (el, eh)  = self.config.joint_stepsize_interval(direction = jdir, max_speed = max_speed, delta_t = ttr) 
            # assert el < 0.0
            if eh > 1.0:
                eh = 1.0
            q = q0 + eh*jdir
            if self.set_config(q):
                if self.pose_metric() < err:
                    return True
                else:
                    print "Error is not reduced"
            else:
                print "set config failed"
        else:
            return True
        if show:
            print "Moving towards target Failed:"
            print
            print "direction = ", jdir
            print "--------------------"
            print "Old q: ", q0
            print "New q: ", self.config.q
            print "--------------------"
            print "Old Error: ", err
            print "New Error: ", self.pose_metric()
            print "--------------------"

        assert self.set_config(q0)
        return False

    def inverse_update(self, phi = None, optimize = False, show = False):    
        '''
        Finds the inverse kinematic solution for the given redundant parameter phi
        is phi is not feasible, the solution corresponding to the closest phi is returned.
        If argument optimize is True, the solution will be optimized 
        so that the joint values will be as close as possible to self.config.qm
        If phi is not given, current q[0] will be selected as phi
        
        The new joint angles will be set if all the kinematic equations are satisfied. 
        All kinematic parameters will be updated.
        '''
        # Finding a feasible phi (theta0)
        if phi == None:
            phi     = self.config.q[0]

        PS = self.permission_set_position()
        if show:
            print "Permission Set for phi = ", PS
            print "Initial phi            = ", phi
        
        if len(PS) == 0:
            print "len(PS) = 0"
            print "Error from PR2_ARM.inverse_update(): The target is out of workspace! No solution found."
            return False
        else:
            if not (phi in PS):
                phi = gen.closest_border(phi, PS, k = 0.01)
                if show:
                    print "Phi is not in PS"
                    print "Closest phi in PS:", phi

            q = self.IK_config(phi) 
            if q == None:
                if show:
                    print phi, " is not a feasible phi. No solution found"
                if not self.closest_feasible_phi(phi, PS):
                    print "Error from PR2_ARM.inverse_update(): The target is out of workspace! No solution found."
                    return False
                if show:
                    print "Next phi: ", self.config.q[0]
            else:    
                if not self.set_config(q):
                    if show:
                        print "Not expected to see. Solution exists but not feasible!"
                    assert False

            # when you reach here, the feasible q has been set    
            
            if optimize:
                self.reduce_error()
             
        assert vecmat.equal(self.wrist_position(), self.xd)
        if self.orientation_respected:
            assert vecmat.equal(self.wrist_orientation(), self.Rd)

        return True
        
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

        if gen.equal(v, 1.0):
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
                    if gen.equal(w2, 1.0):
                        w2 = 1.0
                    elif gen.equal(w2, 0.0):
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
                                    R1_nul = gen.equal(R1, 0)
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
                                                                if gen.equal(s5,0):
                                                                    assert gen.equal(R47[2,0], 0)
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

                                                                    assert gen.equal(R47[1,0] ,  c4*s6 + c5*c6*s4)
                                                                    assert gen.equal(R47[1,1] ,  c4*c6 - c5*s4*s6)
                                                                    assert gen.equal(R47[0,0] ,  -s4*s6 + c4*c5*c6)
                                                                    assert gen.equal(R47[0,1] ,  -c6*s4 - c4*c5*s6)

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

    def project_to_ts(self,  js_traj, phi_start = 0.0, phi_end = None, delta_phi = 0.1):
        '''
        projects the given jointspace trajectory into the taskspace
        The phase starts from phi_start and added by delta_phi in each step.
        if at any time the joint values are not feasible, the process is stopped.
        '''
        
        if phi_end == None:
            phi_end = js_traj.phi_end

        ori_traj = trajlib.Orientation_Trajectory_Segment()
        ori_traj.capacity = 200
        pos_traj = trajlib.Polynomial_Trajectory()
        if phi_end > js_traj.phi_end:
            phi_end = js_traj.phi_end

        phi = phi_start
        js_traj.set_phi(phi)
        stay = True
        while stay and (self.set_config(js_traj.current_position)):
            if phi > phi_end:
                phi = phi_end
                stay = False
            pos_traj.add_point(phi - phi_start, self.wrist_position())
            ori_traj.add_point(phi - phi_start, self.wrist_orientation())
            phi = phi + delta_phi
            js_traj.set_phi(phi)
            
        pos_traj.add_point(phi - phi_start, self.wrist_position())
        ori_traj.add_point(phi - phi_start, self.wrist_orientation())
        return (pos_traj, ori_traj)

    def project_to_js(self,  pos_traj, ori_traj = None, phi_start = 0.0, phi_end = None, delta_phi = 0.1, max_speed = 1.0, relative = True):
        '''
        projects the given taskspace pose trajectory into the jointspace using analytic inverse kinematics.
        The phase starts from phi_start and added by delta_phi in each step.
        at any time, if a solution is not found, the process stops
        '''
        keep_q = np.copy(self.config.q)

        if phi_end == None:
            phi_end = pos_traj.phi_end

        if ori_traj == None:
            ori_traj = trajlib.Orientation_Trajectory()
            ori_traj.current_orientation = self.wrist_orientation()

        if phi_end > pos_traj.phi_end:
            phi_end = pos_traj.phi_end

        jt          = trajlib.Polynomial_Trajectory(dimension = 7)
        jt.capacity = 2

        jt.add_point(phi = 0.0, pos = np.copy(self.config.q), vel = np.zeros(7))

        phi   = phi_start
        pos_traj.set_phi(phi)
        ori_traj.set_phi(phi)
        if relative:
            p0    = self.wrist_position() - pos_traj.current_position
            R0    = np.dot(self.wrist_orientation(), ori_traj.current_orientation.T)  
        else:
            p0    = np.zeros(3)
            R0    = np.eye(3)  
        
        phi       = phi + delta_phi
        stay      = True

        while stay:
            if phi > phi_end:
                phi = phi_end
            if phi == phi_end:
                stay = False
            pos_traj.set_phi(phi)
            ori_traj.set_phi(phi)
            p = p0 + pos_traj.current_position
            R = np.dot(R0, ori_traj.current_orientation)
            self.set_target(p, R)
            self.config.qm = np.copy(self.config.q)
            if self.move_towards_target(phi = self.config.q[0], optimize = True, max_speed = max_speed, ttr = delta_phi):
                jt.add_point(phi = phi - phi_start, pos = np.copy(self.config.q))
            else:
                print "Error is not reduced. Point is not Added"

            phi = phi + delta_phi

        jt.interpolate()
        self.config.qm = 0.5*(self.config.qh + self.config.ql)
        self.set_config(keep_q)

        return jt

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

        Vp_1l = gen.solve_quadratic_inequality(- alpha_l, - betap_l, -gamma_l)
        Vp_1h = gen.solve_quadratic_inequality(  alpha_h,   betap_h,  gamma_h)
        Vn_1l = gen.solve_quadratic_inequality(- alpha_l, - betap_h, -gamma_l)
        Vn_1h = gen.solve_quadratic_inequality(  alpha_h,   betap_l,  gamma_h)

        V1 = (Vp_1l & Vp_1h & interval([0.0,1.0])) | (Vn_1l &  Vn_1h & interval([-1.0,0.0]))

        # Finding wh,wl:

        int_theta2 = interval([self.config.ql[2],self.config.qh[2]])
        int_w   = imath.sin(int_theta2)**2
        (wl, wh) = int_w[0]

        #Finding V2l, V2h

        V2l = gen.solve_quadratic_inequality(alpha + wl, betta, gamma - wl) & interval([-1.0, 1.0])
        V2h = gen.solve_quadratic_inequality(- alpha - wh, - betta, wh - gamma) & interval([-1.0, 1.0])

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
        
        self.Phi = gen.connect_interval(Phi0 & Phi1_3)

        return self.Phi

    def delta_phi_interval(self):
        '''
        Updates the feasible interval for the growth of the redundant parameter according to
        the specified joint limits and the current value of the joints
        '''
        if self.Delta == None:
            J    = self.joint_redundancy_jacobian()
            self.Delta  = self.permission_set_position() - interval(self.config.q[0])  # set initial Delta based on the position permission set for phi

            for i in range(0, 7):
                if (not gen.equal(J[i], 0.0)) and (not gen.equal(self.config.w[i], 0.0)):  # if Ji and wi are not zero
                    d1  = (self.config.ql[i] - self.config.q[i])/J[i]    
                    d2  = (self.config.qh[i] - self.config.q[i])/J[i]    
                    dli = gen.binary_choice(d1,d2,J[i])
                    dhi = gen.binary_choice(d2,d1,J[i])

                    if gen.equal(dli, 0.0):
                        dli = 0.0
                    if gen.equal(dhi, 0.0):
                        dhi = 0.0
                    assert dli <= 0.0
                    assert dhi >= 0.0
                    self.Delta   = self.Delta & interval([dli, dhi])
                    if len(self.Delta) == 0:
                        print "Error: self.Delta is empty:"
                        print "Initial Delta : ", self.permission_set_position() - interval(self.config.q[0])
                        print "interval([dli, dhi]): ", interval([dli, dhi])
                            

        return self.Delta
        


