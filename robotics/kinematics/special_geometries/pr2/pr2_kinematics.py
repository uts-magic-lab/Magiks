## @file        PR2_kinematics.py
#  @brief       Contains specific functions that define all geometric and kinematic parameters for PR2 robot
#  @author      Nima Ramezani Taghiabadi 
#
#               PhD Researcher 
#               Faculty of Engineering and Information Technology 
#               University of Technology Sydney (UTS) 
#               Broadway, Ultimo, NSW 2007, Australia 
#               Phone No. :   04 5027 4611 
#               Email(1)  : nima.ramezani@gmail.com 
#               Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     4.0 
#
#  Last Revision    01 January 2015

'''
Changes from version 4.0:
    1- Comments added for doxygen

'''
import copy
import pr2_arm_kinematics as armlib
import packages.nima.mathematics.general as gen
import packages.nima.general as pygen
import packages.nima.mathematics.rotation as rotlib
import packages.nima.mathematics.vectors_and_matrices as vecmat
import packages.nima.mathematics.trigonometry as trig

from interval import interval, inf, imath

import numpy, math
import packages.nima.robotics.kinematics.kinematicpy.general as genkin
import packages.nima.robotics.kinematics.joint_space.configuration as configlib
import packages.nima.robotics.kinematics.task_space.trajectory as trajlib

drc        = math.pi/180.00

default_ql = drc*numpy.array([-130.0, 70.0 , -180.0, - 131.0, -180.0, -130.0, -180.0, 0.8/drc, -1000.0, -1000.0, -180.0,  -40.0,  60.0, - 44.0, -131.0, -180.0, -130.0, -180.0])
default_qh = drc*numpy.array([  30.0, 170.0,   44.0, -   8.6,  180.0, 0.00,  180.0, 1.14/drc,  1000.0,  1000.0,  180.0,  130.0, 170.0,  180.0, -  8.6,  180.0, 0.00,  180.0])

## @brief Contains properties and methods managing the kinematics of the PR2 robot.
#  This class contains all properties and methods required for forward and inverse kinematic 
#  computations and kinematic control of the PR2 robot in two control modes: Fixed-Base and Free-Base
#  In fixed-base mode, the body of the robot is assumed fixed (No navigation) and only the arms can move
#  while in free-base mode, the robot trunk is free to navigate. 
#  Each control mode can be set for one of the arms as the reference arm (Endeffector)
#  Dual arm control (two endeffectors) is not supported in this version
 
class PR2(object):
    '''
    PR2 has 18 degrees of freedom. The Robot will have two arms that each of them contains its associated joint values in its "config" property
    '''

	## The Class Constructor:
	#  @param a0 A float specifying the value of /f$ a_0 /f$ in the DH parameters of the arm 	
	#  @param b0 A float specifying the distance of line connecting the two arms from the center of robot base 	
	#  @param d2 A float specifying the value of /f$ d_2 /f$ in the DH parameters of the arm 	
	#  @param d4 A float specifying the value of /f$ d_4 /f$ in the DH parameters of the arm 	
	#  @param d7 A float specifying the length of the gripper 	
	#  @param l0 A float specifying half of the distance between the arms 	
	#  @param ql A numpy vector of size 18 containing the lower bounds of the arm joints	
	#  @param qh A numpy vector of size 18 containing the upper bounds of the arm joints
    def __init__(self, a0 = 0.1, b0 = 0.05, d2 = 0.4, d4 = 0.321, d7 = 0.168, l0 = 0.188, ql = default_ql, qh = default_qh):

        assert (len(ql) == 18) and (len(qh) == 18), "Error from " __name__ + " Constructor" + ": ql and qh must be numpy vectors of size 18"
		##  A numpy vector of size 7 containing the lower bounds of the arm joints 
        self.ql = ql

		##  A numpy vector of size 7 containing the upper bounds of the arm joints 
        self.qh = qh

		##  A numpy vector of size 18 containing a set of reference values for the joints. 
		#
		#  This parameter can be used as reference joint values for redundancy optimization. 
		#  The redundancy will be used to set the joints as close as possible to these reference values.  
		#  This property is by default set as the middle of joint ranges.		
        self.qm = (qh + ql)/2

		## A numpy vector of size 18 containing the current values of the joints. 
		#  This property should not be manipulated by the user. Use method Set_Config() to change this property.
		#  The joints are in the following order:
		#  q[0] : Right Arm-Shoulder Pan Joint, 
		#  q[1] : Right Arm-Shoulder Lift Joint,
		#  q[2] : Right Arm-Upper Arm Roll Joint, 
		#  q[3] : Right Arm-Elbow Flex Joint,
		#  q[4] : Right Arm-Forearm Roll Joint, 
		#  q[5] : Right Arm-Wrist Flex Joint, 
		#  q[6] : Right Arm-Wrist Roll Joint,
        #  q[7] : "h_ts" the telescoping prismatic joint that lifts up and shifts down the trunk and arms,
        #  q[8] : "X_BO" the X position of the robot base,
        #  q[9] : "Y_BO" the Y position of the robot base,
        #  q[10] : "tau" the rotation angle of the robot base around "z" axis,
        #  q[11] to q[17] are designated for the left arm in the same order as the right arm
        self.q  = (qh + ql)/2

        ## Specifies the objective function for redundancy optimization.
        #  this property should not be set by the user directly. Use method "self.set_ofuncode" to set this property
        #  code legend:
        #  1: midrange distance
        #  0: distance from current configuration
        self.ofuncode = 1
        
        self.W = numpy.zeros(11)
        for i in [0, 1, 2, 3, 5, 7]:
            self.W[i] = 1.0/((qh[i] - ql[i])**2)

        ## An instance of class \ref packages.nima.robotics.kinematics.special_geometries.pr2.pr2_arm_kinematics.PR2_ARM() 
        #  containing the kinematic properties and methods of the right arm
        self.rarm = armlib.PR2_ARM(a0 = a0, d2 = d2, d4 = d4, ql = ql[0:7]  , qh = qh[0:7]  , W = self.W[0:7], vts = vts)

        ## An instance of class \ref packages.nima.robotics.kinematics.special_geometries.pr2.pr2_arm_kinematics.PR2_ARM() 
        #  containing the kinematic properties and methods of the left arm
        self.larm = armlib.PR2_ARM(a0 = a0, d2 = d2, d4 = d4, ql = ql[11:18], qh = qh[11:18], W = self.W[0:7], vts = vts)

        self.d7 = d7
        self.b0 = b0
        self.l0 = l0
        self.p_EFR_WR = numpy.array([0.0, 0.0, self.d7])
        self.p_EFL_WL = numpy.array([0.0, 0.0, self.d7])

        # sets all angles to midrange by default
        self.set_config(self.qm)

        self.control_mode          = "fixed_base"
        
        ## A boolean specifying the reference arm for the endeffector.
        #  If True, the endeeffector is the end of left arm gripper, False if right arm 
        self.larm_reference        = False
        
        ## A numpy vector of size 3 representing the desired position of the Endeffector.
        #  Recommended to be used as a read only property. Use function set_target() to change the value of this property
        self.xd = None
        
		## A \f$ 3 \times 3 \f$ rotation matrix specifying the desired endeffector orientation for the IK problem. 
		# Recommended not to be set directly. Always use function set_target() to change this property. 
        self.Rd = None

	## Use this function to check if a given value for specific joint is in its feasibility range
	#  @param i An integer between 0 to 17 specifying the joint number to be checked
	#  @param qi A float parameter specifying the value for the i-th joint
	#  @return A boolean: True if the given joint angle qi is in feasible range for the i-th joint (within the specified joint limits for that joint)
    def joint_in_range(self,i, qi):
        '''
        Example
        '''
        if i in range(0, 7):
            return self.rarm.config.joint_in_range(i, qi)
        elif i in range(11, 18):
            return self.larm.config.joint_in_range(i - 11, qi)
        else:
            if i == 10: # if base rotation angle is selected, it should be taken into the standard range
                qi = trig.angle_standard_range(qi) 
            
            if abs(qi - self.ql[i]) < gen.epsilon:
                qi = self.ql[i]
            if abs(qi - self.qh[i]) < gen.epsilon:
                qi = self.qh[i]

            return ((qi <= self.qh[i]) and (qi >= self.ql[i]))

	## Use this function to check if all values of the given joint vector are in their feasibility range
	#  @param qd A numpy vector of size 18 containing the values to be checked
	#  @return A boolean: If The given joints "qd" are out of the range specified by properties: ql and qh, returns False, otherwise returns True
    def all_joints_in_range(self, qd):
        '''
        Example
        '''
        #Are all the arm given joints in range ?
        flag = self.rarm.config.all_joints_in_range(qd[0:7])  
        flag = flag and self.larm.config.all_joints_in_range(qd[11:18])  
        
        for i in range(7, 11):
            flag = flag and self.joint_in_range(i, qd[i])
        return flag

    ## protected
    def set_ofuncode(self, new_ofuncode):

        if (new_ofuncode > 2) or (new_ofuncode < 0):
            print "set_ofuncode_error: The given objective function code is not supported"
            return 0
    
        self.ofuncode               = new_ofuncode
        self.ofun_computed          = False
        self.div_theta_ofun_updated = False
        self.div_phi_ofun_updated   = False

	## Use this function to find the current value of the objective function
	#  @param None
	#  @return An instance of type float containing the value of the objective function
    def objective_function(self):
        if not self.ofun_computed:
            self.ofun = self.objective_function_for(self.q[0:11])
            self.ofun_computed = True
        return self.ofun
        
	## Sets the robot configuration to the desired given joint values
	#  @param qd A numpy array of 7 elements containing the values of the given joints
	#  @return A boolean: True  if the given configuration is successfully set, False if the given configuration is not successfully set. 
    #         (If False is returned, the configuration will not chenge)
    def set_config(self, qd):

        if not len(qd) == 18: #If the class is initialized in "dual_arm" mode, this number should be 18
            print "set_config error: Number of input elements must be 18 in dual arm mode"
            return False

        if self.all_joints_in_range(qd) and self.rarm.set_config(qd[0:7]) and self.larm.set_config(qd[11:18]):
            self.q[0:7]   = trig.angles_standard_range(qd[0:7])
            self.q[7:10]  = qd[7:10]
            self.q[10:18] = trig.angles_standard_range(qd[10:18])

            self.C = math.cos(self.q[10])
            self.S = math.sin(self.q[10])

            self.p_BO    = numpy.array([qd[8]   ,  qd[9],0.0])
            self.p_BL_BO = numpy.array([self.b0     ,  self.l0, qd[7]])        
            self.p_BR_BO = numpy.array([self.b0     ,- self.l0, qd[7]])     

            self.R_B     = numpy.array([[  self.C, - self.S, 0.0 ], 
                                        [  self.S,   self.C, 0.0 ],
                                        [  0.0          ,  0.0         , 1.0]]) 

            self.rarm_endeff_position_updated    = False
            self.larm_endeff_position_updated    = False
            self.rarm_endeff_orientation_updated = False
            self.larm_endeff_orientation_updated = False
            self.in_target_updated               = False
            self.redun_jacob_updated             = False
            self.redun_jacob_ext_updated         = False
            self.geometric_jacob_updated         = False
            self.div_theta_err_updated           = False
            self.div_theta_ofun_updated          = False
            self.div_phi_ofun_updated            = False
            self.ofun_computed                   = False
            self.redundant_parameters_updated    = False
            self.in_target_evaluated             = False

            return True
        else:
            print "Error from PR2().set_config(): Given joints are not in their feasible range"
            print "Lower Limit  :", self.ql
            print "Upper Limit  :", self.qh
            print "Desired Value:", qd
            return False

	## Sets the endeffector target to a desired position and orientation.
	# @param target_position    A numpy array of 3 elements specifying the desired endeffector position
	# @param target_orientation A \f$ 3 \times 3 \f$ numpy rotation matrix representing the desired endeffector orientation
	# @return A boolean: True  if the given pose is successfully set, False if the given pose is not successfully set because of an error
    #        (If False is returned, properties self.xd and self.RD will not chenge)
    def set_target(self, target_position, target_orientation):
        '''
        sets the endeffector target to the given position and orientation
        variables self.xd and self.Rd should not be manipulated by the user. Always use this function
        '''    
        self.xd = target_position
        self.Rd = target_orientation

        # Specify the relative target for the right arm
        '''
        print "in ST: p_BR_BO = ", self.p_BR_BO

        print "rel_ef_pos[2]  = ", -self.p_BR_BO[2] + target_position[2] - target_orientation[2,2]*self.d7

        relative_endeffector_position    = - self.p_BR_BO + numpy.dot(self.R_B.T,target_position - self.p_BO - numpy.dot(target_orientation,self.p_EFR_WR))
        print "rel_ef_pos[2]  = ", relative_endeffector_position[2]
        '''
        relative_endeffector_position    = - self.p_BR_BO + numpy.dot(self.R_B.T,target_position - self.p_BO - numpy.dot(target_orientation,self.p_EFR_WR))
        relative_endeffector_orientation = numpy.dot(self.R_B.T, target_orientation)
        #set the relative pose for the arm
        self.rarm.set_target(relative_endeffector_position,relative_endeffector_orientation)

        self.in_target_evaluated           = False

    ##  Finds all the feasible solutions of the Inverse Kinematic problem for given redundant parameter vector \f$ \phi \f$.
    #   @param phi Is a numpy vector of five elements containing the values of five redundant parameters in free-base mode.
    #   The values of phi specify the following parameters: \n
    #   phi[0] : The first joint angle q[0] (Shoulder-Pan joint angle) \n
    #   phi[1] : The \b r cylindrical coordinate of the wrist (wrist joint center) position
    #   w.r.t. the arm base (shoulder-pan joint center) in torso coordinate system:
    #   \f[
    #   r = p_x^2 + p_y^2 
    #   \f] where \f$ p_x \f$ and \f$ p_y \f$ are the \b x and \b y cartesian coordinates of the wrist position 
    #   w.r.t. the arm base in torso coordinate system. \n
    #   phi[2] : \f$ p_z \f$ or the \b z cartesian coordinate of the wrist position 
    #    w.r.t. the arm base in torso coordinate system. \n
    #   phi[3] : The \b \f$ \psi \f$ cylindrical coordinate of the wrist (wrist joint center) position w.r.t. the arm base in torso CS:
    #   \f[
    #   \psi = \arctan( \frac {p_x} {p_y}) 
    #   \f] (\f$ p_x \f$ and \f$ p_y \f$ can be determined from phi[1] and phi[3]) \n
    #   phi[4] : \b \f$ \tau \f$ or the rotation angle of the trunc base around z axis
    #   @return An array of numpy vectors. Each numpy vector is a feasible solution and contains 11 elements. 
    #   The first 7 elements are the arm joints(left arm if property larm_reference is True and right arm, otherwise) 
    #   and the next 4 elements contain the values of non-arm degrees of freedom (q[7] to q[10])
    #   This function only returns the feasible IK solutions but does NOT set the configuration so the joint values do not change
    #   The output will be \b None if given redundant parameters are invalid or out of feasible range.
    def all_IK_solutions(self, phi):  
        if not self.larm_reference:
            jn     = 0
            arm    = self.rarm
            l0     = - self.l0
            p_EF_W = self.p_EFR_WR
        else:
            jn     = 7
            arm    = self.larm
            l0     = self.l0
            p_EF_W = self.p_EFL_WL

        if not self.joint_in_range(jn, phi[0]):
            print "Error from "+__name__+func_name+": The first given redundant parameter (phi[0]) is out of feasible range"
            return None
            
        if not self.joint_in_range(10, phi[4]):
            print "Error from "+__name__+func_name+": The fifth given redundant parameter (phi[4], tau or trunc rotation angle) is out of feasible range"
            return None
            
        # Todo: check later for phi[2] 

        #keep the current target of the arm
        save_arm_target_position    = copy.copy(arm.xd)
        save_arm_target_orientation = copy.copy(arm.Rd)
        
        # Set parameters based on the given redundancy vector phi:
        r   = phi[1]
        pz  = phi[2]
        psi = phi[3]
        tau = phi[4]

        # Calculate the relative endeffector position:
    
        px = r*math.sin(psi)
        py = r*math.cos(psi)

        p_W_B = numpy.array([px, py, pz])


        # Calculate the relative endeffector orientation:
        R_B     = rotlib.rot_z(tau)  # orientation of robot base
        R_W     = self.Rd
        p_EF    = self.xd   
        R_W_B  = numpy.dot(R_B.T, R_W)

        #set the new target for the arm associated with the given phi
        
        arm.set_target(p_W_B, R_W_B)

        #Find all the solutions for the arm IK
        solution_set = []
        arm_solution_set = arm.all_IK_solutions(phi[0])
        
        if len(arm_solution_set) == 0:
            print "IK Error: No solution for the arm for given phi"
        else:
            
            # Calculate the telescopic prismatic joint:
    
            h_ts  = - pz + self.xd[2] - self.Rd[2,2]*self.d7

            #calculate the base position:

            p_B_BO = numpy.array([self.b0 , l0, h_ts])     

            p_BO    = p_EF - numpy.dot(R_B,(p_B_BO + p_W_B)) - numpy.dot(R_W, p_EF_W)

            assert gen.equal(p_BO[2], 0.0) # z coordinate of the base must be zero

            # Now we have all the joints(all 11 degrees of freedom) insert extra DOFs to the arm solution set

            for arm_solution in arm_solution_set:
                solution = numpy.zeros(11)
                solution[0:7] = arm_solution
                solution[7]   = h_ts     # append q[7] = h_ts
                solution[8]   = p_BO[0]  # append q[8] = X_BO
                solution[9]   = p_BO[1]  # append q[9] = Y_BO
                solution[10]  = tau      # append q[10] = tau
                solution_set.append(solution) # append the solution to the solution set

        #return back the previous target of the arm

        arm.set_target(save_arm_target_position, save_arm_target_orientation)

        return solution_set

    ##  Use this function to get the value of the objective function for a given configuration \b \a q
    #   Why has this function been written?
    #   Sometimes you want to find the value of the objective function for a configuration but 
    #   you don't want to set that configuration (you don't want to take the manipulator to that configuration)
    #   In this case, you can not use function "self.objective_function()" which gives the value of the objective 
    #   function for the current configuration of the robot.
    #   @param q A numpy vector of size 11 containing the joint values of the reference arm followed by the four non-arm joints
    #   @return A float indicating the value of the objective function for the given q
    def objective_function_for(self, q):    

        if not self.larm_reference:
            qc = self.q[0:11]
            qm = self.qm[0:11]
        else:
            qc = numpy.append(self.q[11:18] , self.q[7:11])
            qm = numpy.append(self.qm[11:18], self.qm[7:11])

        if self.ofuncode == 0:
            delta = trig.angles_standard_range(q - qc)
            ofun = numpy.dot(delta.T, self.W*delta)
        elif self.ofuncode == 1:
            delta = trig.angles_standard_range(q - qm)
            ofun = numpy.dot(delta.T, self.W*delta)
        else:
            assert False, "Error from "+__name__ + func_name + ": Value "+str(self.ofuncode)+" for argument ofuncode is not supported"
        return ofun

    ## Finds the solution of the Inverse Kinematic problem for given redundant parameter vector \f$ phi \f$ in free-base mode.
    #  In case of redundant solutions, the one corresponding to the lowest objective function is selected.
    #  Class property ofuncode specifies the objective function. legend for ofuncode:
    #        \li ofuncode = 0 (Default) the solution closest to current joint angles will be selected 
    #        \li ofuncode = 1 the solution corresponding to the lowest midrange distance is selected
    #  This function does NOT set the configuration so the joints do not change
    #  @param phi A numpy vector of size 5 containing the values of the five \em redundant parameters
    #  @return A numpy vector of size 5 containing the IK solution corresponding to the given redundant parameters and the lowest value of the objective function. 
    #  The first 7 elements contain the joints of the reference arm and the last four, are the values of the non-arm joints.    
    def IK_config(self, phi):    

        solution_set = self.all_IK_solutions(phi)

        if len(solution_set) == 0:
            print "IK_config error: No solution found within the feasible joint ranges"
            return None

        ofun_min = 1000
        for i in range(0, len(solution_set)):
            solution = solution_set[i]
            ofun = self.objective_function_for(solution)

            if ofun < ofun_min:
                ofun_min = ofun
                i_min = i
            
        return solution_set[i_min]

    def pos_rarm_grip_wrt_tor_shpan(self):
        '''
        Returns the right arm gripper position vector(endeffector position) with respect to the torso shoulder pan joint center
        '''
        R_WR_B   = self.rarm.wrist_orientation()
        p_WR_BR  = self.rarm.wrist_position()
        p_EFR_BR = p_WR_BR + numpy.dot(R_WR_B, self.p_EFR_WR)
        return(p_EFR_BR)

    def pos_larm_grip_wrt_tor_shpan(self):
        '''
        Returns the left arm gripper position vector(endeffector position) with respect to the torso shoulder pan joint center
        '''
        R_WL_B   = self.larm.wrist_orientation()
        p_WL_BL  = self.larm.wrist_position()
        p_EFL_BL = p_WL_BL + numpy.dot(R_WL_B, self.p_EFL_WL)
        return(p_EFL_BL)

    def pos_larm_grip_wrt_tor(self):
        '''
        Returns the left arm gripper position vector(endeffector position) with respect to the torso at the origin.
        The torso origin is at the floor footprint (projection) of the middle point between the two shoulder pan joint centers.
        '''
        p_EFL_BL = self.pos_larm_grip_wrt_tor_shpan()
        p_EFL_BO = self.p_BL_BO + p_EFL_BL
        return(p_EFL_BO)

    def pos_rarm_grip_wrt_tor(self):
        '''
        Returns the right arm gripper position vector(endeffector position) with respect to the torso at the origin.
        The torso origin is at the floor footprint (projection) of the middle point between the two shoulder pan joint centers.
        '''
        p_EFR_BR = self.pos_rarm_grip_wrt_tor_shpan()
        p_EFR_BO = self.p_BR_BO + p_EFR_BR
        return(p_EFR_BO)

    def pos_rarm_grip(self):
        '''
        Returns the global cartesian coordiantes of the end of right arm gripper.
        '''    
        p_EFR_BO = self.pos_rarm_grip_wrt_tor()
        return(self.p_BO + p_EFR_BO)

    def pos_larm_grip(self):
        '''
        Returns the global cartesian coordiantes of the end of left arm gripper.
        '''    
        p_EFL_BO = self.pos_larm_grip_wrt_tor()
        return(self.p_BO + p_EFL_BO)

    def rarm_endeffector_position(self):        
        '''
        Returns the cartesian coordiantes of the end of gripper as the endeffector of the right arm.
        '''    
        if not self.rarm_endeff_position_updated:

            self.R_WR    = self.rarm_endeffector_orientation()
            self.p_WR_BR = self.rarm.wrist_position()

            self.p_EFR   = self.p_BO + numpy.dot(self.R_B,(self.p_BR_BO + self.p_WR_BR)) + numpy.dot(self.R_WR, self.p_EFR_WR)
            self.rarm_endeff_position_updated = True

        return copy.copy(self.p_EFR)
               
    def larm_endeffector_position(self):        
        '''
        Returns the cartesian coordiantes of the end of gripper as the endeffector of the left arm.
        '''    
        if not self.larm_endeff_position_updated:

            self.R_WL    = self.larm_endeffector_orientation()
            self.p_WL_BL = self.larm.wrist_position()

            self.p_EFL   = self.p_BO + numpy.dot(self.R_B,(self.p_BL_BO + self.p_WL_BL)) + numpy.dot(self.R_WL, self.p_EFL_WL)
            self.larm_endeff_position_updated = True

        return copy.copy(self.p_EFL)

    def endeffector_position(self):
        if self.larm_reference:
            return self.larm_endeffector_position()
        else:
            return self.rarm_endeffector_position()

    def rarm_endeffector_orientation(self):        
        '''
        Returns the cartesian coordiantes of the right arm gripper as the endeffector frame.
        '''    
        if not self.rarm_endeff_orientation_updated:

            self.R_WR_B  = self.rarm.wrist_orientation()
            self.R_WR    = numpy.dot(self.R_B, self.R_WR_B)
            self.rarm_endeff_orientation_updated = True

        return copy.copy(self.R_WR)

    def larm_endeffector_orientation(self):        
        '''
        Returns the cartesian coordiantes of the left arm gripper as the endeffector frame.
        '''    
        if not self.larm_endeff_orientation_updated:

            self.R_WL_B  = self.larm.wrist_orientation()
            self.R_WL    = numpy.dot(self.R_B, self.R_WL_B)
            self.larm_endeff_orientation_updated = True

        return copy.copy(self.R_WL)

    def endeffector_orientation(self):
        if self.larm_reference:
            return self.larm_endeffector_orientation()
        else:
            return self.rarm_endeffector_orientation()

    def div_theta_err(self):
        '''
        Returns the divergence of the vector of kinematic constraints "e" in respect with the joints
        It is a 9*9 matrix
        e_ij = rond e_i / rond q_j  (for i,j = 0,..,8)   
        '''
        if not self.larm_reference:
            arm = self.rarm
            l0  = self.l0
        else:
            arm = self.larm
            l0  = - slef.l0

        if not self.div_theta_err_updated:

            self.E = numpy.zeros((9,9))
            self.F = numpy.zeros((9,5))

            phi = numpy.zeros(5)                

            [s0, s1, s2, s3, s4, s5, s6] = arm.config.s
            [c0, c1, c2, c3, c4, c5, c6] = arm.config.c

            tau = self.q[10]
            C5  = math.cos(tau)
            S5  = math.sin(tau)

            [s10]         = arm.config.s1_mult1
            [s20,s21, s22, s210]  = arm.config.s2_mult1
            [s30, s31, s32, s310, s320, s321, s322, s33,s3210] = arm.config.s3_mult1
            [c0s0,c0s1,c0s2,c0s10,c0s20,c0s21,c0s30,c0s31,c0s32,c0s321] = arm.config.c0_mult
            [c10, c1s0, c1s1, c1s2, c1s3, c1s10, c1s20,c1s30,c1s32,c1s320] = arm.config.c1_mult
            [c20,c21,c22,c210,c2s0,c2s1,c2s2,c2s3,c2s10,c2s20,c2s30,c2s31,c2s310,c21s30,c20s31] = arm.config.c2_mult
            [c30,c31,c32,c33,c3s0,c3s1,c3s2,c3s3,c3s10,c3s20,c3s21,c3s30,c310,c320,c321,c3210,c32s0,c32s10] = arm.config.c3_mult
            [c21s0,c31s0,c321s0]  = arm.config.s0_mult
            [c10s1,c20s1,c30s1, c32s1,c320s1] = arm.config.s1_mult2
            [c10s2,c20s2,c30s2, c32s2,c310s2]  = arm.config.s2_mult2
            [c10s3, c20s3, c30s3, c21s3, c210s3,s332]  = arm.config.s3_mult2
            [c10s32, c31s20, s54] = arm.config.cs_mult
            [c54, c5s4, c5s5] = arm.config.c5_mult
            [s64, s65, c4s6, c5s6, c54s6, s654] = arm.config.s6_mult
            [c64, c65, c6s4, c6s5, c654, C6s54] = arm.config.c6_mult

            phi = self.redundant_parameters()
        
            Rd = self.endeffector_orientation()
            xd = self.endeffector_position()

            [Q, Q2, d42, d44, a00, d22_plus_d44, foura00d44, alpha] = arm.additional_dims

            r2   = phi[1]**2
            rho2 = r2 + phi[2]**2
            R2   = rho2 - arm.l**2
            a0   = arm.a0
            d2   = arm.d2
            d4   = arm.d4

            a = Rd[:,2]
            n = Rd[:,0]
            b = xd - self.d7*a

            ax_c5_m_ay_s5 = a[0]*C5 - a[1]*S5
            ax_s5_p_ay_c5 = a[0]*S5 + a[1]*C5

            nx_c5_m_ny_s5 = n[0]*C5 - n[1]*S5
            nx_s5_p_ny_c5 = n[0]*S5 + n[1]*C5

            C14           = math.cos(phi[0] + phi[3])
            S14           = math.sin(phi[0] + phi[3])
            C45           = math.cos(phi[4] + phi[3])
            S45           = math.sin(phi[4] + phi[3])

            self.E[0,2] = - 2*d42*s3
            self.E[2,1] =   arm.d4*s321
            self.E[3,4] =   -s5
            self.E[4,3] =   c4*s5
            self.E[4,4] =   c5*s4

            self.E[5,4] =   c65
            self.E[5,5] = - s65
            self.E[8,8] =   1.0
            self.E[2,6] =   1.0

            self.E[1,1] = 2*a00*d44*s33*c2s2
            self.E[1,2] = 2*a0*d42*s3*phi[1]*S14 + (a0**2)*(d4**2)*(s2**2)*(2*c3*s3)
            # Alternative:  self.E[1,2] = - s3*d42*(2*c3*d42 - R2) + 2*c3s3*(a0**2)*(d4**2)*(s2**2) 

            self.E[6,6] = 1.0
            self.F[6,2] = 1.0

            self.E[2,0] = - arm.d2*s1 - arm.d4*(c3s1 + c21s3) 
            self.E[2,2] =  - arm.d4*(c1s3 + c32s1) 
            self.E[3,0] =  ax_c5_m_ay_s5*(c20s31 - c310) + ax_s5_p_ay_c5*(c2s310 - c31s0) + a[2]*(c3s1 + c21s3) 
            self.E[3,1] =  ax_c5_m_ay_s5*(c2s30 + c10s32) - ax_s5_p_ay_c5*(c20s3 - c1s320) - a[2]*s321 
            self.E[3,2] =  ax_c5_m_ay_s5*(c3s20 - c3210 + c0s31) - ax_s5_p_ay_c5*(c30s2 + c321s0 - s310) + a[2]*(c1s3 + c32s1)
            self.E[4,0] =  - ax_c5_m_ay_s5*c0s21 - ax_s5_p_ay_c5*s210 - a[2]*c1s2
            self.E[4,1] =  ax_c5_m_ay_s5*(c210 -s20)  + ax_s5_p_ay_c5*(c0s2 + c21s0) - a[2]*c2s1
                             
            self.E[5,0] =  nx_c5_m_ny_s5*(- c20s31 + c310) + nx_s5_p_ny_c5*(- c2s310 + c31s0) - n[2]*(c3s1 + c21s3) 
            self.E[5,1] =  - nx_c5_m_ny_s5*(c2s30 + c10s32) + nx_s5_p_ny_c5*(c20s3 - c1s320) + n[2]*s321     
            self.E[5,2] =    nx_c5_m_ny_s5*(- c3s20 + c3210 - c0s31) + nx_s5_p_ny_c5*(c30s2 + c321s0 - s310) - n[2]*(c1s3 + c32s1) 
            self.E[7,7] =  1.0
            self.E[8,8] =  1.0

            self.F[0,2] = -2*phi[2]

            self.F[7,1] =  S45
            self.F[8,1] =  C45

            self.F[0,0] =   2*arm.a0*phi[1]*C14 
            self.F[0,1] =   2*arm.a0*S14 - 2*phi[1]
            self.F[0,3] =   2*arm.a0*phi[1]*C14 

            self.F[1,1] =   2*phi[1]*arm.a0*(phi[1]*S14 - arm.a0)
            self.F[1,2] =   2*phi[1]*arm.a0*S14*phi[2]

            self.F[3,0] =   ax_c5_m_ay_s5 *(c0s32 + c21s30 + c3s10) + ax_s5_p_ay_c5*(s320 - c210s3 - c30s1) 
            self.F[3,4] =   - ax_s5_p_ay_c5*(s320 - c210*s3 - c30*s1) - ax_c5_m_ay_s5*(c0*s32 + c21*s30 + c3s10)
            self.F[4,0] =   ax_c5_m_ay_s5*(c20 - c1s20) + ax_s5_p_ay_c5*(c2s0 + c10s2)
            self.F[4,4] = - ax_c5_m_ay_s5*(c20 - c1s20) - ax_s5_p_ay_c5*(c2s0 + c10s2)  
            self.F[5,0] = - nx_c5_m_ny_s5*(c0s32 + c21s30 + c3s10) + nx_s5_p_ny_c5*(-s320 + c210s3 + c30s1) 
            self.F[5,4] = + nx_c5_m_ny_s5*(c0s32 + c21s30 + c3s10) - nx_s5_p_ny_c5*(-s320 + c210s3 + c30s1)
            self.F[7,4] =   phi[1]*C45 + C5*l0
            self.F[8,4] =   - phi[1]*S45 - S5*l0
            self.F[7,3] =   phi[1]*C45
            self.F[8,3] =   - phi[1]*S45

            self.div_theta_err_updated = True

        return copy.copy(self.E)

    def redundant_parameters(self):
        '''
        returns vector of redundant parameters phi according to the parametrization specified:

        phi[0] = The first arm joint angle (Shoulder Pan joint)
        phi[1] = The "r" coordinate of the relative wrist position(in cylinderical coordinates) phi[1]^2 = px^2 + py^2
        phi[2] = The "z" coordinate of the relative wrist position(in cylinderical coordinates) phi[2]   = pz
        phi[3] = The "psi" coordinate of the relative wrist position(in cylinderical coordinates) phi[3] = arctan(x/y)
        phi[4] = The rotation of the base. Same as "tau" or the 10th joint angle (phi[4] = q[10] = tau)
        '''
        if self.larm_reference:
            arm = self.larm
        else:
            arm = self.rarm

        if not self.redundant_parameters_updated:
            p = arm.wrist_position()
            r2 = p[0]**2 + p[1]**2
            self.phi = numpy.zeros(5)
            self.phi[0] = arm.config.q[0]
            self.phi[1] = math.sqrt(r2)
            self.phi[2] = p[2] 
            self.phi[3] = math.atan2(p[0],p[1]) 
            self.phi[4] = self.q[10]
            self.redundant_parameters_updated = True
        return copy.copy(self.phi)    

    def div_phi_err(self):
        '''
        Returns the divergence of the vector of kinematic constraints "e" in respect with the redundant parameters "phi"
        It is a 9*5 matrix
        f_ij = rond e_i / rond phi_j  (for i = 0,..,8 & j = 0,..,4) 
        the value of "self.F" is computed in function "div_theta_e" to avoid repeated calculations
        '''
        if not self.div_theta_err_updated:    
            self.div_theta_err()
        return copy.copy(self.F)

    def redundancy_jacobian(self):
        '''
        Returns the redundancy jacobian
        Redundancy Jacobian is the partial derivative of the joints in respect with redundant parameters
        RJ is a 9*5 matrix because the first and the last joints (q[0] and q[10]) are themselves selected as the redundant parameters
        '''
        if self.larm_reference:
            arm = self.larm
        else:
            arm = self.rarm

        if not self.redun_jacob_updated:

            d42 = arm.d2*arm.d4
    
            [s0, s1, s2, s3, s4, s5, s6] = arm.config.s
            [c0, c1, c2, c3, c4, c5, c6] = arm.config.c

            s321 = s3*s2*s1
            c65  = c6*c5
            s65  = s6*s5

    
            p = self.endeffector_position()

            r2 = p[0]**2 + p[1]**2
            p1 = math.sqrt(r2)
            p2 = arm.xd[2]
        

            C4 = math.cos(self.q[10])
            S4 = math.sin(self.q[10])

            RJ = numpy.zeros((9,5))        

            E = self.div_theta_err()
            F = self.div_phi_err()


            RJ[2,0] =  F[0,0]/(2*d42*s3)
            RJ[2,1] =  F[0,1]/(2*d42*s3)
            RJ[2,2] =  -p2/(d42*s3)
            RJ[2,3] =  F[0,3]/(2*d42*s3)

            RJ[8,1] =  -F[8,1]
            RJ[8,3] =  -F[8,3]
            RJ[8,4] =  -F[8,4]

            RJ[6,2] =  - 1.0

            RJ[ 7 , 1 ] =  -F[7,1]
            RJ[ 7 , 3 ] =  -F[7,3]
            RJ[ 7 , 4 ] =  -F[7,4]
            '''
            RJ[7,0] =  -RJ[8,0]*E[7,8]/E[7,7]
            RJ[7,1] =  (2*p1 - RJ[8,1]*E[7,8])/E[7,7]
            RJ[7,2] =  -RJ[8,2]*E[7,8]/E[7,7]
            RJ[7,3] =  -RJ[8,3]*E[7,8]/E[7,7]
            RJ[7,4] =  -(F[7,4] + RJ[8,4]*E[7,8])/E[7,7]
            '''
            RJ[1,0] =  -RJ[2,0]*E[1,2]/E[1,1]
            RJ[1,1] =  -(F[1,1] + RJ[2,1]*E[1,2])/E[1,1]
            RJ[1,2] =  -(F[1,2] + RJ[2,2]*E[1,2])/E[1,1]
            RJ[1,3] =  -RJ[2,3]*E[1,2]/E[1,1]
            RJ[1,4] =  -RJ[2,4]*E[1,2]/E[1,1]

            RJ[0,0] =  -(RJ[6,0] + RJ[2,0]*E[2,2] + RJ[1,0]*arm.d4*s321)/E[2,0]
            RJ[0,1] =  -(RJ[6,1] + RJ[2,1]*E[2,2] + RJ[1,1]*arm.d4*s321)/E[2,0]
            RJ[0,2] =  -(RJ[6,2] + RJ[2,2]*E[2,2] + RJ[1,2]*arm.d4*s321)/E[2,0]
            RJ[0,3] =  -(RJ[6,3] + RJ[2,3]*E[2,2] + RJ[1,3]*arm.d4*s321)/E[2,0]
            RJ[0,4] =  -(RJ[6,4] + RJ[2,4]*E[2,2] + RJ[1,4]*arm.d4*s321)/E[2,0]

            RJ[4,0] =  (F[3,0] + RJ[0,0]*E[3,0] + RJ[1,0]*E[3,1] + RJ[2,0]*E[3,2])/s5
            RJ[4,1] =  (RJ[0,1]*E[3,0] + RJ[1,1]*E[3,1] + RJ[2,1]*E[3,2])/s5
            RJ[4,2] =  (RJ[0,2]*E[3,0] + RJ[1,2]*E[3,1] + RJ[2,2]*E[3,2])/s5
            RJ[4,3] =  (RJ[0,3]*E[3,0] + RJ[1,3]*E[3,1] + RJ[2,3]*E[3,2])/s5
            RJ[4,4] =  (- F[3,0] + RJ[0,4]*E[3,0] + RJ[1,4]*E[3,1] + RJ[2,4]*E[3,2])/s5

            RJ[3,0] =  -(F[4,0] + RJ[0,0]*E[4,0] + RJ[1,0]*E[4,1] + RJ[4,0]*c5*s4)/(c4*s5)
            RJ[3,1] =  -(RJ[0,1]*E[4,0] + RJ[1,1]*E[4,1] + RJ[4,1]*c5*s4)/(c4*s5)
            RJ[3,2] =  -(RJ[0,2]*E[4,0] + RJ[1,2]*E[4,1] + RJ[4,2]*c5*s4)/(c4*s5)
            RJ[3,3] =  -(RJ[0,3]*E[4,0] + RJ[1,3]*E[4,1] + RJ[4,3]*c5*s4)/(c4*s5)
            RJ[3,4] =  -(F[4,4] + RJ[0,4]*E[4,0] + RJ[1,4]*E[4,1] + RJ[4,4]*c5*s4)/(c4*s5)

            RJ[5,0] =  (F[5,0] + RJ[0,0]*E[5,0] + RJ[1,0]*E[5,1] + RJ[2,0]*E[5,2] + RJ[4,0]*c65)/s65
            RJ[5,1] =  (RJ[0,1]*E[5,0] + RJ[1,1]*E[5,1] + RJ[2,1]*E[5,2] + RJ[4,1]*c65)/s65
            RJ[5,2] =  (RJ[0,2]*E[5,0] + RJ[1,2]*E[5,1] + RJ[2,2]*E[5,2] + RJ[4,2]*c65)/s65
            RJ[5,3] =  (RJ[0,3]*E[5,0] + RJ[1,3]*E[5,1] + RJ[2,3]*E[5,2] + RJ[4,3]*c65)/s65
            RJ[5,4] =  (- F[5,0] + RJ[0,4]*E[5,0] + RJ[1,4]*E[5,1] + RJ[2,4]*E[5,2] + RJ[4,4]*c65)/s65

            self.RJ = RJ    
            self.redun_jacob_updated = True
        return copy.copy(self.RJ)

    def redundancy_jacobian_extended(self):
        '''
        returns the extended redundancy jacobian which is "11 x 5" matrix.
        It is the redundancy jacobian with two rows inserted At its first and last rows. 
        These rows are corresponding to the first and last joints (shoulder pan joint and base rotation angle).
        '''    
        if not self.redun_jacob_ext_updated:
            self.ERJ = numpy.zeros((11,5))
            RJ  = self.redundancy_jacobian()
            for i in range(9):
                self.ERJ[i+1,:] = RJ[i,:]
            self.ERJ[0,0]  = 1.0
            self.ERJ[10,4] = 1.0
            self.redun_jacob_ext_updated = True
        return copy.copy(self.ERJ)

    def div_theta_ofun(self):
        '''
        Returns the divergence of the objective function in respect with the joint angles
        argument "ofun" specifies the code of the objective function which is set to 0 by default 
        '''        
        if not self.div_theta_ofun_updated:
            self.DTG = numpy.zeros(11)
            for i in range(11):
                self.DTG[i] = 2*self.W[i]*(self.q[i] - self.qm[i])    
            self.div_theta_ofun_updated = True
        return copy.copy(self.DTG)    

    def div_phi_ofun(self):
        '''
        Returns the divergence of the objective function in respect with the redundant parameters "phi"
        argument "ofun" specifies the code of the objective function which is set to 0 by default. 
        The code legend is like function "div_theta_ofun" 
        '''
        if not self.div_phi_ofun_updated:
            J = self.redundancy_jacobian_extended()
            self.DFG = numpy.dot(J.T, self.div_theta_ofun())
            self.div_phi_ofun_updated = True
        return copy.copy(self.DFG)

    def joint_stepsize_interval(self, direction):
        '''
        Returns an interval for the step size by which the joints are allowed to move in the given "direction"
        "direction" is a "n X 1" vector and specifies the direction of motion in the jointspace. 
        '''

        stepsize_interval = interval([-inf, inf])
        for i in range(11):  #  for each joint
            if self.W[i] != 0: # if the joint is limited
                joint_correction_interval = interval([self.ql[i], self.qh[i]]) - interval(self.q[i])  # find the interval for joint correction 
            else:
                joint_correction_interval = interval([-inf, inf])

            stepsize_interval = stepsize_interval & joint_correction_interval/direction[i]

            '''
            print "direction :                         = ", direction[i]
            print "stepsize_interval:                  = ", stepsize_interval
            print
            '''
        
        return stepsize_interval
                
    def optimum_joint_stepsize(self, direction, safety_factor = 0.99):
        '''
        returns the optimum feasible step size that minimizes the objective function 
        if the joints are constrained to move in the given "direction".
        The "safety_factor" specifies how much we can approach to the borders of joint limits
        '''
        if not self.larm_reference:
            qc = self.q[0:11]
            qm = self.qm[0:11]
        else:
            qc = numpy.append(self.q[11:18] , self.q[7:11])
            qm = numpy.append(self.qm[11:18], self.qm[7:11])

        den = numpy.dot(direction.T, self.W * direction)
        num = numpy.dot(direction.T, self.W * (qc - qm))

        if not gen.equal(num, 0.0):
            eta = - num/den
        else:
            print "Error from PR2.optim_joint_stepsize(): Moving in the given direction does not influence the objective function"
            return None

        interval_eta   = self.joint_stepsize_interval(direction)

        (eta_l, eta_h) = interval_eta[0]
        
        if eta < eta_l:
            return safety_factor*eta_l
        elif eta < eta_h:
            return eta
        else:
            return safety_factor*eta_h


    def steepest_descent_redundancy_correction(self):
        '''
        Finds the steepest descent direction of the objective function for redundancy vector "phi"
        '''
        steepest_descent_redundancy_direction = - self.div_phi_ofun()
        steepest_descent_joint_direction      = numpy.dot(self.redundancy_jacobian_extended(), steepest_descent_redundancy_direction)

        eta = self.optimum_joint_stepsize(steepest_descent_joint_direction)

        
        return eta*steepest_descent_redundancy_direction


    def endeffector_in_target(self):
        '''
        Returns true is endeffector is in target
        '''
        if not self.in_target_evaluated:
            self.in_target = vecmat.equal(self.xd, self.endeffector_position()) and vecmat.equal(self.Rd, self.endeffector_orientation())
            self.in_target_evaluated = True
        return self.in_target


    def optimize_config(self, silent = True):
        '''
        Assuming the endeffector is in target,
        this function changes the robot configuration without changing the endeffector pose
        until the minimum possible value of objective function is achieved
        Note:
        This function is a procedure and does not return any thing. Running this function, changes the robot configuration. 
        If the current tip pose is not in target, this function fails with an error
        '''    
        if (self.xd == None) or (self.Rd == None):
            self.set_target(self.endeffector_position(), self.endeffector_orientation())

        assert self.endeffector_in_target(), "optimize_config Error: Endeffector is not in target pose"
        
        ofun_old = self.objective_function()
        k        = 1.0

        while k > 0.001:
            phi = self.redundant_parameters()
            # Find the steepest descent correction for redundancy:    
            delta_phi = self.steepest_descent_redundancy_correction() 
            # Correct the redundancy
            phi = phi + k*delta_phi
            # Run IK to find the joints with the corrected redundancy:
            solution = self.IK_config(phi)
            if solution == None:
                fail = True
            else:
                # Solution exists: Compute the value of the objective function for the new configuration
                ofun_new = self.objective_function_for(solution)
                pygen.show("Objective Function Value: ", ofun_new, silent)
                if ofun_new > ofun_old - gen.epsilon:
                    fail = True
                else:
                    # Objective Function Reduced: Try to set the solution config
                    qd          = numpy.copy(self.q)
                    qd[0:11]    = solution
                    if not self.set_config(qd):
                        fail = True
                    else:
                        # Solution is feasible :-)
                        fail = False
            
            if fail:
                k = k/2
            else:
                k = 1.0   

    
    def extended_config(self, q):
        qq = numpy.copy(self.q)
        if not self.larm_reference:
            qq[0:11]  = q
        else:
            qq[7:11 ] = q[7:11]
            qq[11:18] = q[0:7]
        return qq

    def inverse_update(self):
        '''
        Finds the inverse kinematic solution closest to the current configuration
        The new joint angles will be set if all the kinematic equations are satisfied. 
        All kinematic parameters will be updated.
        THIS FUNCTION IS NOT COMPLETE AND i NEED TO WORK ON IT MORE ...
        '''
        if not self.larm_reference:
            arm    = self.rarm
            p_EF_W = self.p_EFR_WR
            p_B_BO = self.p_BR_BO
        else:
            arm    = self.larm
            p_EF_W = self.p_EFL_WL
            p_B_BO = self.p_BL_BO
            
        if self.control_mode == "fixed_base":
            arm_ori_target = numpy.dot(self.R_B.T, self.Rd)
            arm_pos_target = numpy.dot(self.R_B.T, self.xd - self.p_BO - numpy.dot(self.Rd, p_EF_W)) - p_B_BO
            arm.set_target(arm_pos_target, arm_ori_target)
            if not arm.inverse_update(optimize = True):
                print "Error from inverse_update: No soluton for the arm in fixed mode"
                return False
            else:
                if not self.larm_reference:
                    self.q[0:7] = arm.config.q
                else:
                    self.q[11:18] = arm.config.q

                self.set_config(self.q)
                return True            
        elif self.control_mode == "free_base":
            print "Error from PR2().inverse_update: Free-Base mode is not supported yet."
            return False
        else:            
            print "Error from PR2().inverse_update: Unknown control mode"
            return False


    def project_to_js(self,  pos_traj, ori_traj = None, phi_start = 0.0, phi_end = None, delta_phi = 0.1, relative = True):
        '''
        projects the given taskspace pose trajectory into the jointspace 
        The phase starts from phi_start and added by delta_phi in each step.
        at any time, if a solution is not found, the process stops
        '''
        if phi_end == None:
            phi_end = pos_traj.phi_end
    
        if phi_end > pos_traj.phi_end:
            phi_end = pos_traj.phi_end
    
        if ori_traj == None:
            ori_traj = trajlib.Orientation_Trajectory()
            ori_traj.current_orientation = self.endeffector_orientation()

        jt          = trajlib.Polynomial_Trajectory(dimension = 18)
        jt.capacity = 5
        jt.add_point(phi = 0.0, pos = numpy.copy(self.q), vel = numpy.zeros(18))

        phi   = phi_start
        pos_traj.set_phi(phi)
        ori_traj.set_phi(phi)
        if relative:
            p0    = self.endeffector_position() - pos_traj.current_position
            R0    = numpy.dot(self.endeffector_orientation(), ori_traj.current_orientation.T)  
        else:
            p0    = numpy.zeros(3)
            R0    = numpy.eye(3)  
        
        phi       = phi + delta_phi
        stay      = True

        while (phi <= phi_end) and stay:
            if phi == phi_end:
                stay = False
            pos_traj.set_phi(phi)
            ori_traj.set_phi(phi)
            p = p0 + pos_traj.current_position
            R = numpy.dot(R0, ori_traj.current_orientation)
            self.set_target(p, R)
            if self.inverse_update():
                jt.add_point(phi = phi - phi_start, pos = numpy.copy(self.q))
                phi = phi + delta_phi
                if phi > phi_end:
                    phi = phi_end
            else:
                stay = False

        jt.interpolate()

        return jt

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

        while (phi < phi_end) and (self.set_config(js_traj.current_position)):
            pos_traj.add_point(phi - phi_start, self.endeffector_position())
            print self.endeffector_position()
            ori_traj.add_point(phi - phi_start, self.endeffector_orientation())
            phi = phi + delta_phi
            if phi > phi_end:
                phi = phi_end
            js_traj.set_phi(phi)
            
        pos_traj.add_point(phi - phi_start, self.endeffector_position())
        ori_traj.add_point(phi - phi_start, self.endeffector_orientation())
        return (pos_traj, ori_traj)


