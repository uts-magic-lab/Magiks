## @file        	pyride_synchronizer.py
#  @brief     		Contains simplified functions to control PR2 using pyride engine
#  @author      	Nima Ramezani Taghiabadi 
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. : 04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	7.0
# 
#  Start date:      09 April 2014
#  Last Revision:  	20 July 2015

'''
Changes from ver 5.0:
    1- The new version supports MAGIKS (The general velocity-based IK engine) for online trajectory tracking
       In association with version 4.0 of pr2_arm_kinematics     
'''    

import pr2_kinematics as pr2lib
import pyride_interpreter as pint
import PyPR2, math, time, copy, sys, numpy as np
import general_python as genpy

from math_tools import general_math as gen
from math_tools.geometry import trigonometry as trig, rotation as rot, geometry as geo, trajectory as trajlib
from math_tools.algebra  import vectors_and_matrices as vecmat
from magiks.vision import laser_scan_support as lss

r_arm_joint_names =['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
l_arm_joint_names =['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']

def read_raw_trajectory(duration = 10.0, delay = 0.5):
    t0 = time.time()
    t  = t0
    
    pint.activate_trajectory_input()
    rt = trajlib.Trajectory_Polynomial()
    
    while t - t0 < duration:
        print "I am adding this point: ", pint.rt_position ," at time: ", t-t0
        rt.add_point(phi = t - t0, pos = np.copy(pint.rt_position))
        time.sleep(delay)
        t = time.time()
    
    return rt

def calibrate_pr2_r_arm():
    # Calibration:
    '''
    Trying to compute DH parameters of the right arm according to the FK measured from the robot
    '''
    q = np.zeros(10)    

    # Get the current right arm joint values:
    rajd = PyPR2.getArmJointPositions(False)   # gets the Right Arm Joint Dictionary
    for i in range(7):
        q[i] = rajd[r_arm_joint_names[i]]

    q[1] = q[1] + math.pi/2

    q = trig.angles_standard_range(q)

    # determining a0    
    t = PyPR2.getRelativeTF('r_shoulder_pan_link' , 'r_shoulder_lift_link')
    p = t['position']
    q[7] = p[0]  # This is a0

    # determining d2
    t = PyPR2.getRelativeTF('r_upper_arm_roll_link' , 'r_elbow_flex_link')
    p = t['position']
    q[8] = p[0]  # This is a0

    # determining d4
    t = PyPR2.getRelativeTF('r_forearm_roll_link' , 'r_wrist_flex_link')
    p = t['position']
    q[9] = p[0]  # This is d4

    return(q)
    
def calibrate_pr2_l_arm():
    # Calibration:
    '''
    Trying to compute DH parameters of the left arm according to the FK measured from the robot
    '''
    q = np.zeros(10)    

    # Get the current right arm joint values:
    lajd = PyPR2.getArmJointPositions(True)   # gets the Left Arm Joint Dictionary
    for i in range(7):
        q[i] = lajd[l_arm_joint_names[i]]

    q[1] = q[1] + math.pi/2
    q = trig.angles_standard_range(q)
    # determining a0    
    t = PyPR2.getRelativeTF('l_shoulder_pan_link' , 'l_shoulder_lift_link')
    p = t['position']
    q[7] = p[0]

    # determining d2
    t = PyPR2.getRelativeTF('r_upper_arm_roll_link' , 'r_elbow_flex_link')
    p = t['position']
    q[8] = p[0]

    # determining d4
    t = PyPR2.getRelativeTF('r_forearm_roll_link' , 'r_wrist_flex_link')
    p = t['position']
    q[9] = p[0]

    return(q)

def calibrate_pr2_torso():
    q = np.zeros(7)

    # Determining h_ts
    p = PyPR2.getRelativeTF('base_footprint', 'torso_lift_link')['position']
    q[0] = p[2] # This is h_ts or theta[7]
    '''
    There is a 50 mm offset in x direction between footprint (Torso origin) p_BO in the paper and the base_footprint defined for PR2.
    So I will consider this offset in parameter b0
    '''
    q[6] = p[0] # This is b0 

    # Determining X_BO and Y_BO:
    p = PyPR2.getRobotPose()['position'] # This should give the global coordinates of the base_footprint
    assert gen.equal(p[2], 0.0, epsilon = 0.01) # The robot base_footprint must be on the ground (Accuracy = 1 cm)
    q[1] = p[0]        # This is X_BO (theta[8])
    q[2] = p[1]        # This is Y_BO (theta[9])

    # Determining tau:
    '''    
    o = PyPR2.getRobotPose()['orientation']
    assert gen.equal(np.linalg.norm(o), 1.0)
    # h = (o[3],o[0],o[1], o[2])
    R = rot.rotation_matrix(o)
    uvect_i = vecmat.as_vector([1.0, 0.0, 0.0])
    uvect_k = vecmat.as_vector([0.0, 0.0, 1.0])
    q[3] = vecmat.vectors_angle(R[:, 0], uvect_i, positive_direction = uvect_k) # This is tau (theta[10])
    '''
    q[3] = pint.body_angle(in_degrees = False)    # this is theta[10]

    # Determining l0
    p = PyPR2.getRelativeTF('torso_lift_link' , 'r_shoulder_pan_link')['position']
    assert gen.equal(p[0], 0.0)
    assert gen.equal(p[2], 0.0)
    q[4] = - p[1] # This is l0

    # Determining d7
    pr = PyPR2.getRelativeTF('r_wrist_roll_link' , 'r_gripper_r_finger_tip_link')['position']
    pl = PyPR2.getRelativeTF('r_wrist_roll_link' , 'r_gripper_l_finger_tip_link')['position']
    p = (vecmat.as_vector(pr) + vecmat.as_vector(pl))/2
    assert gen.equal(p[1], 0.0)
    assert gen.equal(p[2], 0.0)
    q[5] = p[0] # This is d7

    return(q)    

## This class introduces a data structure for complete kinematics of a PR2 robot which can sychronize itself with the real robot
#  or the robot in simulation.  
#  An instance of this class is an object supported by all kinematic functions inherited from class 
#  packages.nima.robotics.kinematics.special_geometries.pr2.pr2_kinematics.PR2()
#  containing additional methods in order to actuate, move and control the real robot and/or get the sensory data from it
#  The object can synchronize itself with the robot in both forward and inverse directions via pyride interface engine. 
#  In other words, any changes in the status and configuration of the object can be directly applied
#  to the real robot or robot in simulation and vice versa.
#  Forward synchronization is to actuate the robot to the configuration of the object and 
#  inverse synchronization is to set the object with the configuration of the real robot.
class PyRide_PR2(pr2lib.PR2):
    '''
    '''
    ## Class Constructor    
    def __init__(self, run_magiks = False):

        q_d = np.zeros(18)
        self.r_arm_joint_names = r_arm_joint_names
        self.l_arm_joint_names = l_arm_joint_names

        q_r = calibrate_pr2_r_arm()
        q_l = calibrate_pr2_l_arm()
        q_t = calibrate_pr2_torso()

        ## A dictionary specifying the maximum waiting time when you choose to wait for any motion to finish. \n
        #  This time can be different for the motion of different parts of the robot. \n
        #  max_wait_time['rarm']: Refers to the motion of the right arm (Change in joints q[0] - q[6]) \n
        #  max_wait_time['larm']: Refers to the motion of the left arm (Change in joints q[11] - q[17]) \n
        #  max_wait_time['body']: Refers to the navigation of the body (Change in joints q[8], q[9] , q[10]) \n
        #  max_wait_time['trunk']: Refers to the trunk lifting motion (Change in joint q[7])
        self.max_wait_time = {'rarm':15.0, 'larm':15.0, 'body':60, 'robot':20, 'trunk':20 }

        assert gen.equal(q_r[7], q_l[7])
        assert gen.equal(q_r[8], q_l[8])
        assert gen.equal(q_r[9], q_l[9])

        #pr2lib.PR2.__init__(self)
        super(PyRide_PR2, self).__init__(a0 = q_r[7], d2 = q_r[8], d4 = q_r[9], d7 = q_t[5], l0 = q_t[4], b0 = q_t[6], run_magiks = run_magiks)

        q_d = np.concatenate((q_r[0:7], q_t[0:4], q_l[0:7]))    
        super(PyRide_PR2, self).set_config(q_d)

        self.set_target(self.end_position(), self.end_orientation())
        self.rarm.set_target(self.rarm.wrist_position(), self.rarm.wrist_orientation())
        self.larm.set_target(self.larm.wrist_position(), self.larm.wrist_orientation())
        
        ## A float specifying the speed of arm gripper in the operational space in m/sec
        self.arm_speed     = 0.1

        self.arm_max_speed = 1.0
        self.base_laser_scan_range = range(400, 640)
        self.tilt_laser_scan_range = range(80, 300)

        self.larm_reference = True

    def trunk_synced(self):
        '''
        Returns True if robot and object body heights are identical
        '''    
        p  = PyPR2.getRelativeTF('base_footprint', 'torso_lift_link')['position']
        return gen.equal(p[2] , self.q[7], epsilon = 0.01)

    def body_synced(self):
        '''
        Returns True if robot and object body positions are identical
        '''    
        bp = pint.body_position()
        ba = pint.body_angle(in_degrees = False)

        fx = gen.equal(bp[0], self.q[8], epsilon = 0.1)
        fy = gen.equal(bp[1], self.q[9], epsilon = 0.1)
        ft = gen.equal(ba   , self.q[10], epsilon = 0.02)

        return(fx and fy and ft)
    
    ## Use this function to check if the right arm of the object is synced with the right arm of the real robot
    #  @param None
    #  @return A boolean: True, if the joint angles of the right arm of the object instance 
    #                     are all equal to their equivalents in the real robot, False if not             
    def rarm_synced(self):
        return vecmat.equal(trig.angles_standard_range(pint.rarm_joints(in_degrees = False)), trig.angles_standard_range(self.rarm.config.q), epsilon = 0.01)

    ## Use this function to check if the left arm of the object is synced with the left arm of the real robot
    #  @param None
    #  @return A boolean: True, if the joint angles of the left arm of the object instance 
    #                     are all equal to their equivalents in the real robot, False if not             
    def larm_synced(self):
        return vecmat.equal(trig.angles_standard_range(pint.larm_joints(in_degrees = False)), trig.angles_standard_range(self.larm.config.q), epsilon = 0.01)

    ## Use this function if you want the robot to say something.
    #  @param s A string what you want the robot to say
    #  @return None 
    def say(self, s):
        PyPR2.say(s)

    ## Use this function to check if the robot is synchronized with the object.
    #  @param limb_list An array of strings specifying which parts should be checked.
    #  @return None 
    def synced(self, limb_list = ['body', 'rarm', 'larm', 'trunk']):
        sn = True
        if 'body' in limb_list:
            sn = sn and self.body_synced()
        if 'rarm' in limb_list:
            sn = sn and self.rarm_synced()
        if 'larm' in limb_list:
            sn = sn and self.larm_synced()
        return sn

    ## Synchronizes the object with the robot and verifies the equity of forward kinematics for both the right and left arm endeffectors.
    #  @param None
    #  @return None
    def sync_object(self):

        q_r = calibrate_pr2_r_arm()
        q_l = calibrate_pr2_l_arm()
        q_t = calibrate_pr2_torso()

        q_d = np.concatenate((q_r[0:7], q_t[0:4], q_l[0:7]))    

        assert super(PyRide_PR2, self).set_config(q_d)
        
        laggp1 = self.larm_end_position(relative = False) # Left Arm Global Gripper Position from object fk
        laggp2 = pint.pos_larm_grip() # Left Arm Global Gripper Position measured from robot

        assert vecmat.equal(laggp1, laggp2, epsilon = 0.1) # verify equality with 10 cm accuracy

        raggp1 = self.rarm_end_position(relative = False) # Right Arm Global Gripper Position from object fk
        raggp2 = pint.pos_rarm_grip() # Right Arm Global Gripper Position measured from robot

        assert vecmat.equal(raggp1, raggp2, epsilon = 0.1) # verify equality with 10 cm accuracy

    ## Synchronizes the position and orientation of the real robot body or robot in simulation with the object.
    #  If the body pose is not already synced, it takes the configuration specified by <b> self.q[8:11] </b>
    #  @param ttr A float specifying <em> time to reach </em> in seconds
    #  @param wait A boolean: \li If True, the system waits until the the body pose is synchronized or <b> self.max_wait_time['body'] </b> is elapsed. 
    #                          (You don't exit the function before the motion is finished or the maximum waiting time is elapsed)
    #                         \li If False, you will exit the function immidiately while the defined motion is running
    #  @return A boolean: \li True If the body is successfully synchronized with the object at the time of function exit \n
    #                     \li False If the body fails to synchronize for any reason \n
    #                     \b  Note: If parameter \b wait is False, the function returns True only if the body is already synced
    def sync_body(self, ttr = 5.0, wait = True, max_wait_time = 60.0): 
        if not self.body_synced():
            pint.move_robot_to(x = self.q[8], y = self.q[9], tau = self.q[10], time_to_reach = ttr, in_degrees = False)  # Move the robot body 
            if wait:
                max_wait_time = gen.ensured_in_range(max_wait_time, 30.0, 300.0)
                t0 = time.time()
                t  = 0.0
                while (t < self.max_wait_time_body) and (not self.body_synced()):
                    pint.move_robot_to(x = self.q[8], y = self.q[9], tau = self.q[10], time_to_reach = ttr, in_degrees = False)  # Move the robot body first
                    pint.wait_until_finished(limb_list = ['body'], max_time = 0.2*self.max_wait_time_body)
                    t = time.time() - t0
        return self.body_synced()
    
    def set_config_synced(self, qd, ttr = 5.0, max_time = 120.0, limb_list = ['body', 'rarm', 'larm']):
        '''
        sets the given configuration to the object and
        synchronizes the robot with the object. 
        If the given joints are feasible, the robot takes the given configuration.
        The function will return False if the robot can not reach the desired configuration within max_time
        '''
        if self.set_config(qd): # Make sure the given joints are feasible
            t0 = time.time()
            t  = 0.0
            while (t < max_time) and (not self.synced(limb_list)):
                if ('body' in limb_list) and (not self.body_synced()):
                    pint.move_robot_to(x = self.q[8], y = self.q[9], tau = self.q[10], time_to_reach = ttr, in_degrees = False)  # Move the robot body first
                else:
                    pint.body_reached = True
                if ('rarm' in limb_list) and (not self.rarm_synced()):
                    pint.take_rarm_to(self.rarm.config.q, time_to_reach = ttr) # Move the right arm
                else:
                    pint.rarm_reached = True
                if ('larm' in limb_list) and (not self.larm_synced()):
                    pint.take_larm_to(self.larm.config.q, time_to_reach = ttr) # Move the left arm
                else:
                    pint.larm_reached = True

                assert pint.wait_until_finished(limb_list = limb_list)
                t = time.time() - t0
        else: 
            print "Warning from PyRide_PR2.set_config_synced(): Given joints are not feasible. Configuration was not set"
        return self.synced(limb_list)
    
    ## Synchronizes the right arm of the real robot or robot in simulation with the right arm of the object.
    #  If the right arm is not already synced, it takes the configuration specified by <b> self.q[0:7] </b> or <b> self.rarm.config.q[0:7] </b>
    #  @param ttr A float specifying <em> time to reach </em> in seconds
    #  @param wait A boolean: \li If True, the system waits until the the right arm is synchronized or <b> self.max_wait_time['rarm'] </b> is elapsed. 
    #                          (You don't exit the function before the motion is finished or the maximum waiting time is elapsed)
    #                         \li If False, you will exit the function immidiately while the defined motion is running
    #  @return A boolean: \li True If the right arm is successfully synchronized with the object at the time of function exit \n
    #                     \li False If the right arm fails to synchronize for any reason \n
    #                     \b  Note: If parameter \b wait is False, the function returns True only if the right arm is already synced
    def sync_rarm(self, ttr = 5.0, wait = True):
        if not self.rarm_synced():
            pint.take_rarm_to(self.rarm.config.q, time_to_reach = ttr)
            if wait:
                pint.wait_until_finished(limb_list = ['rarm'], max_time = self.max_wait_time['rarm'])
        return self.rarm_synced()

    ## Synchronizes the left arm of the real robot or robot in simulation with the left arm of the object.
    #  If the left arm is not already synced, it takes the configuration specified by <b> self.q[11:18] </b> or <b> self.larm.config.q[0:7] </b> 
    #  @param ttr A float specifying <em> time to reach </em> in seconds
    #  @param wait A boolean: \li If True, the system waits until the arm reaches the target or <b> self.max_wait_time['larm'] </b> is elapsed. 
    #                          (You don't exit the function before the motion is finished or the maximum waiting time is elapsed) 
    #                         \li If False, you will exit the function immidiately while the defined motion is running.
    #  @return A boolean: \li True If the left arm is successfully synchronized with the object at the time of function exit 
    #                     \li False If the left arm fails to synchronize for any reason \n
    #                     \b Note: If parameter \b wait is False, the function returns True only if the left arm is already synced
    def sync_larm(self, ttr = 5.0, wait = True):
        if not self.larm_synced():
            pint.take_larm_to(self.larm.config.q, time_to_reach = ttr)
            if wait:
                pint.wait_until_finished(limb_list = ['larm'], max_time = self.max_wait_time['larm'])
        return self.larm_synced()

    ## Synchronizes the real robot or robot in simulation with the object.
    #  The body, trunk, right and left arms synchronize simultaneously. If each part is already synced, it will not move. 
    #  If the system is in \em Free-Base mode, the robot navigates to the position specified by \f$ x = q[8] \f$ and \f$ y = q[9] \f$
    #  and twists to rotation angle specified by \f$ \tau = q[10] \f$. 
    #  (If the system is in \em fixed-base mode, the body will not move) \n
    #  The right arm takes the configuration specified by <b> self.q[0:7]   </b> or <b> self.rarm.config.q[0:7] </b> \n
    #  The left arm  takes the configuration specified by <b> self.q[11:18] </b>  or <b> self.larm.config.q[0:7] </b>
    #  @param ttr A float specifying <em> time to reach </em> in seconds
    #  @param wait A boolean: \li If True, the system waits until the robot is synchronized or <b> self.max_wait_time['robot'] </b>  is elapsed. 
    #                          (You don't exit the function before the motion is finished or the maximum waiting time is elapsed)
    #                         \li If False, you will exit the function immidiately while the motion is running
    #  @return A boolean: \li True If robot is successfully synchronized with the object at the time of function exit 
    #                     \li False The robot fails to synchronize for any reason \n
    #                     \b Note: If parameter \b wait is False, the function returns True only if the robot is already synced
    def sync_robot(self, ttr = 5.0, wait = True):
        '''
        '''
        ll = []
        if (self.control_mode == 'free-base') and (not self.body_synced()):
            self.set_config_synced(self.q, ttr = ttr, wait = False)
            ll.append('body')

        self.sync_rarm(ttr = ttr, wait = False)
        ll.append('rarm')
        self.sync_larm(ttr = ttr, wait = False)
        ll.append('larm')

        if wait:
            pint.wait_until_finished(limb_list = ll, max_time = self.max_wait_time['robot'])
        
        return self.synced(limb_list = ll)

    ##  Solves the Inverse Kinematics for the right arm with given redundant parameter \f$ \phi \f$
    #   and takes the right arm endeffector to the target specified by properties \b self.rarm.xd and \b self.rarm.Rd .
    #   @param phi A float by which you specify the value of the desired redundant parameter (The desired value of shoulder-pan joint).
    #              If phi = None, then the optimum phi corresponding to the minimum value of the objective function 
    #              will be selected. (The default value is None)
    #   @param ttr A float specifying <em> time to reach </em> in seconds. 
    #              Use this parameter to specify the amount of time needed to reach the target. 
    #              Obviously this value influences the speed of motion.
    #   @param wait A boolean: \li If True, the system waits until the right arm gipper takes the desired target pose or <b> self.max_wait_time['rarm'] </b>  is elapsed. 
    #                          (You don't exit the function before the target is achieved or the maximum waiting time is elapsed)
    #                         \li If False, you will exit the function immidiately while the motion is running
    #   @return A boolean: \li True If the right arm can reach the target successfully
    #                     \li False If the arm fails to reach the target for any reason \n
    #                     \b Note: If parameter \b wait is False, the function returns True if an IK solution is found for the right arm.
    def rarm_target(self, phi = None, ttr = 5.0, wait = True):
        func_name = ".rarm_target()"
        self.rarm.set_target(self.rarm.xd, self.rarm.Rd)  # make sure the target parameters are set
        if phi == None:
            if self.rarm.goto_target(optimize = True):
                qr = self.rarm.config.q
            elif self.rarm.goto_target(optimize = False):
                qr = self.rarm.config.q
            else:
                assert False, genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, "Could not find an IK solution for given target. Make sure the target pose is in the workspace")
        else:
            C = self.rarm.permission_set_position()
            if phi in C:
                qr = self.rarm.IK_config(phi)
            else:
                assert False, genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, "Given phi is not in the permission set")

        if qr == None:
            assert False, genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, "No IK solution found for the given redundant parameter phi! Change the value of the redundant parameter and try again")
        elif self.rarm.config.set_config(qr):
                self.q[0:7] = qr
        else:
            print "Error from PyRidePR2.rarm_target(): This should not happen! Check your code."
            return False
        
        return self.sync_robot(ttr = ttr, wait = wait) or (not wait)
    
    ##  Solves the Inverse Kinematics for the left arm with given redundant parameter \f$ \phi \f$
    #   and takes the right arm endeffector to the target specified by properties \b self.larm.xd and \b self.larm.Rd .
    #   @param phi A float by which you specify the value of the desired redundant parameter (The desired value of shoulder-pan joint).
    #              If phi = None, then the optimum phi corresponding to the minimum value of the objective function 
    #              will be selected. (The default value is None)
    #   @param ttr A float specifying <em> time to reach </em> in seconds. 
    #              Use this parameter to specify the amount of time needed to reach the target. 
    #              Obviously this value influences the speed of motion.
    #   @param wait A boolean: \li If True, the system waits until the left arm gipper takes the desired target pose or <b> self.max_wait_time['larm'] </b> is elapsed. 
    #                          (You don't exit the function before the target is achieved or the maximum waiting time is elapsed)
    #                         \li If False, you will exit the function immidiately while the motion is running
    #   @return A boolean: \li True If the left arm can reach the target successfully
    #                     \li False If the arm fails to reach the target for any reason \n
    #                     \b Note: If parameter \b wait is False, the function returns True if an IK solution is found for the left arm.
    def larm_target(self, phi = None, ttr = 5.0, wait = True):
        '''
        Solves the Inverse Kinematics for the left  arm with given redundant parameter "phi"
        and takes the right arm endeffector to the target specified by self.larm.xd and self.larm.Rd
        if phi = None, then the optimum phi will be selected.
        '''
        
        self.larm.set_target(self.larm.xd, self.larm.Rd)  # make sure the target parameters are set
        if phi == None:
            if self.larm.goto_target(optimize = True):
                ql = self.larm.config.q
            else:
                assert False, genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, "Could not find an IK solution for given target. Make sure the target pose is in the workspace")

        else:
            C = self.larm.permission_set_position()
            if phi in C:
                ql = self.larm.IK_config(phi)
            else:
                assert False, genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, "Given phi is not in the permission set")
        if ql == None:
            assert False, genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, "No IK solution found for the given redundant parameter phi! Change the value of the redundant parameter and try again")
        else:
            if self.larm.config.set_config(ql):
                self.q[11:18] = ql
            else:
                print "Error from PyRidePR2.larm_target(): This should not happen! Check your code."
                return False
        
        return self.sync_robot(ttr = ttr, wait = wait) or (not wait)

    ## Solves the Inverse Kinematics for the reference arm with given redundant parameter \f$ \phi \f$
    #  The instructions are exactly the same as functions <b> self.rarm_target() </b> and <b> self.larm_target() </b>
    def arm_target(self, phi = None, ttr = 5.0, wait = True):
        if self.larm_reference:
            return self.larm_target(phi = phi, ttr = ttr, wait = wait)
        else:
            return self.rarm_target(phi = phi, ttr = ttr, wait = wait)

    def reach_target(self):
        '''
        # should change to goto_target() calling the goto_target() function of the super class    
        Solves the IK in the current control mode and takes the robot endeffector to the desired pose specified by self.xd and self.Rd
        '''
        if self.larm_reference:
            tl = ['larm']
        else:
            tl = ['rarm']

        if self.control_mode == "Free-Base":
            tl.append('body')

        if self.goto_target():
            self.set_config_synced(qd = self.q, limb_list = tl)
            return True
        else:   
            self.sync_object()    
            return False

    def reference_arm(self):
        if self.larm_reference:
            return(self.larm)
        else:
            return(self.rarm)

    ## Moves the reference arm wrist in backward direction maintaining the gripper orientation
    #  @param dx A float specifying the distance (in meters) by which the wrist should move in backward direction
    #  @param relative A boolean for selecting the orientation with respect to which the backward direction is defined \n 
    #                  If True, the backward direction is defined relative to the gripper orientation \n
    #                  if False the absolute backward direction (with respect to the robot trunk) is considered
    #  @return A boolean: True, if the arm wrist reach the target successfully and
    #                     False, if the IK fails to find a feasible solution or for any reason the wrist can not reach its target
    def arm_back(self, dx = 0.1, relative = False, wait = True):
        ttr = abs(dx/self.arm_speed)
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            n = ori[:,2]
        else:
            n = rot.i_uv

        pos = pos - dx*n

        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr, wait = wait)

    ## Moves the reference arm wrist in forward direction maintaining the gripper orientation
    #  @param dx A float specifying the distance (in meters) by which the wrist should move in forward direction
    #  @param relative A boolean for selecting the orientation with respect to which the forward direction is defined \n
    #                  If True, the forward direction is defined relative to the gripper orientation \n
    #                  If False the absolute forward direction (with respect to the robot trunk) is considered
    #  @return A boolean: True if the arm wrist reach the target successfully
    #                     False if the IK fails to find a feasible solution or for any reason the wrist can not reach its target
    def arm_forward(self, dx = 0.1, relative = False, wait = True):
        ttr = abs(dx/self.arm_speed)
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            n = ori[:,2]
        else:
            n = rot.i_uv
        pos = pos + dx*n
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr, wait = wait)

    ## Moves the reference arm wrist in downward direction maintaining the gripper orientation
    #  @param dx A float specifying the distance (in meters) by which the wrist should move in downward direction
    #  @param relative A boolean for selecting the orientation with respect to which the downward direction is defined \n
    #                  If True, the downward direction is defined relative to the gripper orientation \n
    #                  If False the absolute downward direction (with respect to the robot trunk) is considered
    #  @return A boolean: True if the arm wrist reach the target successfully
    #                     False if the IK fails to find a feasible solution or for any reason the wrist can not reach its target
    def arm_down(self, dx = 0.1, relative = False, wait = True):
        ttr = abs(dx/self.arm_speed)
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            h = ori[:,1]
        else:
            h = rot.k_uv
        pos = pos - dx*h
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr, wait = wait)

    ## Moves the reference arm wrist in upward direction maintaining the gripper orientation
    #  @param dx A float specifying the distance (in meters) by which the wrist should move in upward direction
    #  @param relative A boolean for selecting the orientation with respect to which the upward direction is defined \n
    #                  If True, the upward direction is defined relative to the gripper orientation \n
    #                  If False the absolute upward direction (with respect to the robot trunk) is considered
    #  @return A boolean: True if the arm wrist reach the target successfully
    #                     False if the IK fails to find a feasible solution or for any reason the wrist can not reach its target
    def arm_up(self, dx = 0.1, relative = False, wait = True):
        ttr = abs(dx/self.arm_speed)
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            h = ori[:,1]
        else:
            h = rot.k_uv
        pos = pos + dx*h
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr, wait = wait)

    ## Moves the reference arm wrist to the right maintaining the gripper orientation
    #  @param dx A float specifying the distance (in meters) by which the wrist should move to the right
    #  @param relative A boolean for selecting the orientation with respect to which the direction to the right is defined
    #                  If True, the right direction is defined relative to the gripper orientation,
    #                  if False the absolute direction to the right (with respect to the robot trunk) is considered
    #  @return A boolean: True if the arm wrist reach the target successfully
    #                     False if the IK fails to find a feasible solution or for any reason the wrist can not reach its target
    def arm_right(self, dx = 0.1, relative = False, wait = True):
        '''
        moves the arm to the right as much as dx (m) maintaining the orientation
        '''    
        ttr = abs(dx/self.arm_speed)
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            w = ori[:,0]
        else:
            w = rot.j_uv
        pos = pos - dx*w
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr, wait = wait)

    ## Moves the reference arm wrist to the left maintaining the gripper orientation. 
    #  The speed of motion is set by property self.arm_speed
    #  @param dx A float specifying the distance (in meters) by which the wrist should move to the left
    #  @param relative A boolean for selecting the orientation with respect to which the left direction is defined
    #                  If True, the left direction is defined relative to the gripper orientation,
    #                  if False the absolute left direction (with respect to the robot trunk) is considered
    #  @return A boolean: True if the arm wrist reach the target successfully
    #                     False if the IK fails to find a feasible solution or for any reason the wrist can not reach its target
    def arm_left(self, dx = 0.1, relative = False, wait = True):
        ttr = abs(dx/self.arm_speed)
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            w = ori[:,0]
        else:
            w = rot.j_uv
        pos = pos + dx*w
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr, wait = wait)

    ## Moves the reference arm wrist to the left and downward direction maintaining the gripper orientation. 
    #  The speed of motion is set by property self.arm_speed
    #  @param dx A float specifying the distance (in meters) by which the wrist should move to the left
    #  @param dy A float specifying the distance (in meters) by which the wrist should move downwards
    #  @param relative A boolean for selecting the orientation with respect to which the left and downward directions are defined
    #                  If True, the left and downward directions are defined relative to the gripper orientation,
    #                  if False the absolute left and downward directions (with respect to the robot trunk) are considered
    #  @return A boolean: True if the arm wrist reach the target successfully
    #                     False if the IK fails to find a feasible solution or for any reason the wrist can not reach its target
    def arm_left_down(self, dx = 0.1, dy = 0.1, relative = False, wait = True):
        ttr = math.sqrt(dx*dx + dy*dy)/self.arm_speed
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            w = ori[:,0]
            h = ori[:,1]
        else:
            w = rot.j_uv
            h = rot.k_uv
        pos   = pos + dx*w - dy*h
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr, wait = wait)

    ## Moves the reference arm wrist to both left and upward directions maintaining the gripper orientation. 
    #  The speed of motion is set by property self.arm_speed
    #  @param dx A float specifying the distance (in meters) by which the wrist should move to the left
    #  @param dy A float specifying the distance (in meters) by which the wrist should move upwards
    #  @param relative A boolean for selecting the orientation with respect to which the left and upward directions are defined
    #                  If True, the left and upward directions are defined relative to the gripper orientation,
    #                  if False the absolute left and upward directions (with respect to the robot trunk) are considered
    #  @return A boolean: True if the arm wrist reach the target successfully
    #                     False if the IK fails to find a feasible solution or for any reason the wrist can not reach its target
    def arm_left_up(self, dx = 0.1, dy = 0.1, relative = False, wait = True):
        ttr = math.sqrt(dx*dx + dy*dy)/self.arm_speed
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            w = ori[:,0]
            h = ori[:,1]
        else:
            w = rot.j_uv
            h = rot.k_uv
        pos   = pos + dx*w + dy*h
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr, wait = wait)

    def arm_right_up(self, dx = 0.1, dy = 0.1, relative = False, wait = True):
        '''
        moves the arm to the left as much as dx (m) maintaining the orientation
        '''    
        ttr = math.sqrt(dx*dx + dy*dy)/self.arm_speed
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            w = ori[:,0]
            h = ori[:,1]
        else:
            w = rot.j_uv
            h = rot.k_uv
        pos = pos - dx*w + dy*h
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr, wait = wait)
    
    def arm_right_down(self, dx = 0.1, dy = 0.1, relative = False, wait = True):
        '''
        moves the arm to the left as much as dx (m) maintaining the orientation
        '''    
        ttr = math.sqrt(dx*dx + dy*dy)/self.arm_speed
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            w = ori[:,0]
            h = ori[:,1]
        else:
            w = rot.j_uv
            h = rot.k_uv
        pos = pos - dx*w - dy*h
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr, wait = wait)

    ##  Starting from the current wrist position , the arm wrist draws an arc around the given \b center 
    #   with the given \b angle in the plane specified by the given \b normal vector.
    #   @param center A numpy vector of 3 elements specifying the position of the arc center w.r.t. the starting point and
    #                 should be in the coordinate system of the gripper
    #   @param angle A float specifying the arc angle in radians (The default value is \f$ \pi \f$)  
    #   @param N An integer specifying the number of key points in the arc trajectory (The default value is 100)
    #   @param normal A numpy vector of 3 elements specifying a vector perpendicular to the arc plane. 
    #                 The given vector should be in the coordinate system of the gripper and 
    #                 must be perpendicular to the vector given by parameter \b center 
    #                 (The default value is unit vector \f$ i = [1.0, 0.0, 0.0] \f$ specifying the forward direction w.r.t. the gripper)
    #   @param wait A boolean: \li If True, the system waits until the arc reaches the end of arc trajectory. 
    #                          (You don't exit the function before the trajectory is finished)
    #                          \li If False, you will exit the function immidiately while the defined motion is running
    #   @return A boolean: \li True if the arc trajectory is successfully projected to the jointspace and 
    #                      in case argument \b wait is True, the wrist finishes the trajectory successfully
    #                      \li False if for any reason the IK trajectory projection fails or in case parameter \b wait is True,
    #                      the robot wrist fails to finish the trajectory           
    def arm_arc(self, center = np.array([0.0, -0.05, 0.0]), angle = math.pi, normal = np.array([1.0, 0.0, 0.0]), N = 100, wait = True):
        '''
        '''
        # assert genmath.equal(vecmat.angle(center, normal), 0.0), "Error from PyRide_PR2(): center and normal must be perpendicular"

        #tt = trajlib.Trajectory_Polynomial()
        #jt = trajlib.Trajectory_Polynomial(dimension = 7)
        d_theta = angle/N
        r       = np.linalg.norm(center)
        ttr     = r*d_theta/self.arm_speed

        arm  = self.reference_arm()
        p0   = arm.wrist_position()    
        ori  = arm.wrist_orientation()    
        '''
        J    = [- vecmat.normalize(center)]
        J    = np.append(J, [vecmat.normalize(normal)])
        Jdag = vecmat.right_pseudo_inverse(J)        
        '''    
        #tt.add_point(phi = 0.0, pos = p0)
        #jt.add_point(phi = 0.0, pos = arm.config.q)
        if self.larm_reference:
            g   = pint.gen_larm_joint_posvel_dict(arm.config.q, np.zeros(7), 0.0)
        else:
            g   = pint.gen_rarm_joint_posvel_dict(arm.config.q, np.zeros(7), 0.0)

        config_list = [g]
        
        for i in range(N):
            theta = (i+1)*d_theta
            '''
            b     = np.array([math.cos(theta), 0.0])
            x     = np.dot(Jdag, b)
            pos   = p0 + center + r*normalize(x)
            '''
            q0   = np.copy(arm.config.q)
            ax   = np.append(theta, vecmat.normalize(normal))
            R    = rot.rotation_matrix(ax, parametrization = 'angle_axis')
            pos  = p0 + center - np.dot(R, center)
            arm.set_target(pos, ori)
            arm.config.qm = np.copy(arm.config.q)
            if arm.move_towards_target(max_speed = self.arm_max_speed, ttr = d_theta):
                if i == N - 1:
                    vel = np.zeros(7)
                else:
                    vel = (arm.config.q - q0)/ttr

                if self.larm_reference:
                    g   = pint.gen_larm_joint_posvel_dict(arm.config.q, vel, ttr)
                else:
                    g   = pint.gen_rarm_joint_posvel_dict(arm.config.q, vel, ttr)
            else:
                print "The arc is not in the workspace!"
                return False

            config_list.append(g)

        if self.larm_reference:
            pint.larm_failed  = False
            pint.larm_reached = False
            moving_limbs = ['larm']
        else:
            pint.rarm_failed  = False
            pint.rarm_reached = False
            moving_limbs = ['rarm']

        PyPR2.moveArmWithJointTrajectoryAndSpeed(config_list)
        arm.config.qm = 0.5*(arm.config.ql+arm.config.qh)
        if wait:
            pint.wait_until_finished(limb_list = moving_limbs, max_time = 10*N*ttr)
        self.sync_object()

    ## Changes the orientation of the reference arm gripper to a desired direction with out changing the wrist position.  
    #  @direction A string specifying the direction. Must be selected among: \n
    #             'forward', 'backward', 'upward', 'downward', 'right', 'left'  
    #  @param ttr A float specifying the time to reach the target orientation
    #  @param wait A boolean: \li If True, the system waits until the arm gripper 
    #                         is reached to the desired orientation or <b> self.max_wait_time </b> of the reference arm is elapsed. 
    #                         (You don't exit the function before the motion is finished or the maximum waiting time is elapsed)
    #                         \li If False, you will exit the function immidiately while the defined motion is running
    #  @return A boolean: \li True if the arm gripper reach the target orientation successfully
    #                     \li False if the IK fails to find a feasible solution or for any reason the gripper can not reach the target orientation.
    def arm_orient(self, direction = 'forward', ttr = 2.0, wait = True):
        if direction == 'upward':
            ori = rot.point_upward_orientation
        elif direction == 'downward':
            ori = rot.point_downward_orientation
        elif direction == 'forward':
            ori = rot.point_forward_orientation
        elif direction == 'backward':
            ori = rot.point_backward_orientation
        elif direction == 'right':
            ori = rot.point_right_orientation
        elif direction == 'left':
            ori = rot.point_left_orientation
        else:
            assert False, genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, direction + " is not a valid value for direction")

        self.sync_object()
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr, wait = wait)

    def front_line_tilt(self):
        '''
        Returns the front line of tilt scan
        '''
        if not pint.tl_active:
            self.activate_tilt_laser()

        (a, b, r) = lss.front_line(dist = pint.tl_dist, position_range = self.tilt_laser_scan_range, angle_min = -0.829031407833, angle_step = 0.00436332309619)

        return (a, b, np.linalg.norm(r) )

    def front_line_base(self):
        '''
        Returns the front line of tilt scan
        '''
        if not pint.bl_active:
            self.activate_base_laser()

        (a, b, r) = lss.front_line(dist = pint.bl_dist, position_range = self.base_laser_scan_range, angle_min = -2.26892805099, angle_step = 0.00436332309619)

        return (a, b, np.linalg.norm(r) )

    '''
    def front_plane(self):

        # Start:
        dist = copy.copy(pint.dist)

        (beta, intercept, residuals) = front_line(dist)
        stay = True
        while stay:
            outliers = rlib.which(residuals , '<' , 0.1 )
            dist     = rlib.pick(dist, - outliers)
            (beta, intercept, residuals) = front_line(dist)
    '''

    ## Runs a given arm task-space pose trajectory on the robot. The given trajectory will be tracked by the wrist.
    #  @param pos_traj An instance of class packages.nima.robotics.kinematics.task_space.trajectory.Trajectory_Polynomial()
    #                  specifying the trajectory in the 3D taskspace to be tracked by the arm.
    #  @param ori_traj An instance of class packages.nima.robotics.kinematics.task_space.trajectory.Trajectory_Polynomial()
    #                  specifying the trajectory in the 3D taskspace to be tracked by the arm.
    def arm_trajectory(self, pos_traj, ori_traj = None, resolution = 50, relative = True, wait = True):
        '''
        First, projects the given taskspace pose trajectory into the jointspace 
        using both velocity-based and position-based inverse kinematics of the arm and
        creates a joint trajectory.
        Then the joint trajectory is given to function run_config_trajectory of pyride_interpreter to run on the robot.
        '''
        L = sum(pos_traj.points_dist())  # You will need to accommodate for orientation as well

        self.sync_object()
        arm     = self.reference_arm()

        assert resolution > 4, genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, str(resolution) + " is an invalid value for resolution. Must be greater than 4")

        keep_dt = arm.dt
        arm.dt  = pos_traj.phi_end/resolution
        #jt  = arm.js_project(pos_traj, ori_traj, relative = relative, traj_type = 'polynomial')
        jt  = arm.js_project(pos_traj, ori_traj, relative = relative)
        arm.dt  = keep_dt

        #jt.segment[0].interpolate() # Check why you have to do an extra interpolate .... !!!!
        #jt.consistent_velocities()
        '''
        jt.plot()
        jt.plot(1)
        jt.plot(4)
        
        (pt, ot) = arm.project_to_ts(jt)
        pt.plot3d()
        '''    

        pint.run_config_trajectory(jt, duration = L/self.arm_speed, dt = None, phi_dot = None, is_left_arm = self.larm_reference)

        if self.larm_reference:
            moving_limbs = ['larm']
        else:
            moving_limbs = ['rarm']

        if wait:
            pint.wait_until_finished(limb_list = moving_limbs, max_time = 10*L/self.arm_speed)
        
        self.sync_object()

    ## Activates the Kinematic Control Service
    def activate_kc(self): 
        self.kc_service_cnt = 0
        self.prev_qdot      = np.zeros(7)
        self.kc_gain        = 1.0
        PyPR2.registerRawTrajectoryInput( self.kc_service ) 
        assert PyPR2.useJointVelocityControl(True)

    ## Deactivates the Kinematic Control Service
    def deactivate_kc(self):
        self.kc_service_cnt = 0
        PyPR2.registerRawTrajectoryInput( None )
        assert PyPR2.useJointVelocityControl(False)

    def kc_service(self, data):
        if self.larm_reference:
            pe = self.p_EFL_WL
            actual_pos = pint.larm_wrist_position()
            actual_ori = geo.Orientation_3D(pint.larm_wrist_orientation(), representation = 'quaternion')
            actual_jnt = pint.larm_joints(in_degrees = False)
        else:
            pe = self.p_EFR_WR
            actual_pos = pint.rarm_wrist_position()
            actual_ori = geo.Orientation_3D(pint.rarm_wrist_orientation(), representation = 'quaternion')
            actual_jnt = pint.rarm_joints(in_degrees = False)

        self.kc_service_cnt += 1
        self.sync_object()
        arm   = self.reference_arm()
        desired_pos   = np.array(data['position'])
        desired_ori   = geo.Orientation_3D(data['orientation'], representation = 'quaternion')
        desired_pos  -= np.dot(desired_ori['matrix'], pe)
        arm.set_target(desired_pos, desired_ori['matrix'])
        if self.kc_service_cnt == 1:
            '''
            self.kc_time   = data['timestamp']
            self.t0        = data['timestamp']
            '''
            self.kc_time   = 0.0
            self.t0        = time.time()
            self.prev_err  = arm.pose_metric()
            self.kc_ptd   = trajlib.Trajectory_Polynomial() 
            self.kc_otd   = trajlib.Orientation_Trajectory_Polynomial() 
            self.kc_pta   = trajlib.Trajectory_Polynomial() 
            self.kc_ota   = trajlib.Orientation_Trajectory_Polynomial() 
            self.kc_jtd   = trajlib.Trajectory(dimension = 7, capacity = 10) 
            self.kc_jta   = trajlib.Trajectory(dimension = 7, capacity = 10) 
            '''
            arm.goto_target(optimize = True)
            '''
            self.kc_jtd.add_point(phi = self.kc_time, pos = np.copy(arm.config.q), vel = np.zeros(7), acc = np.zeros(7))
        else:        
            '''
            dt           = data['timestamp'] - self.kc_time
            self.kc_time = data['timestamp']
            '''
            dt            = time.time() - self.t0 - self.kc_time
            self.kc_time += dt
            if data['in_progress']:
                q0 = np.copy(arm.config.q)
                arm.config.qm = np.copy(arm.config.q)
                # arm.goto_target(optimize = True)
                arm.dt = dt    
                if arm.moveto_target(optimize = True):
                    jdir  = arm.config.q - q0
                    q_dot = jdir/dt

                    (jpos, vel, acc) = trajlib.feasible_position(Xd = np.copy(arm.config.q), X0 = q0, V0 = self.prev_qdot, X_min = arm.config.ql, X_max = arm.config.qh, v_max = arm.max_js, a_max = arm.max_ja, dt = dt, smooth = True)
                    pint.send_arm_joint_speed(vel, is_left_arm = self.larm_reference)
                    self.prev_qdot = np.copy(vel)

                    self.kc_jtd.add_point(phi = self.kc_time, pos = jpos, vel = vel, acc = acc)
            else:
                self.kc_jtd.add_point(phi = self.kc_time, pos = np.copy(arm.config.q), vel = np.zeros(7), acc = np.zeros(7))
                pint.send_arm_joint_speed(np.zeros(7), is_left_arm = self.larm_reference)
                arm.config.qm = 0.5*(arm.config.ql + arm.config.qh)
                self.deactivate_kc()

        self.kc_ptd.add_point(phi = self.kc_time, pos = np.copy(desired_pos))
        self.kc_otd.add_point(phi = self.kc_time, ori = copy.deepcopy(desired_ori))
        self.kc_pta.add_point(phi = self.kc_time, pos = np.copy(actual_pos))
        self.kc_ota.add_point(phi = self.kc_time, ori = copy.deepcopy(actual_ori))
        self.kc_jta.add_point(phi = self.kc_time, pos = np.copy(actual_jnt))

    def arm_track(self, k = 1.0, delay = 0.1, max_speed = 1.0, relative = True):
    
        ts        = time.time()
        t0        = 0.0

        arm = self.reference_arm()
        assert arm.magiks_running

        self.sync_object()

        # activate raw trajectory input:
        pint.activate_trajectory_input()
        if pint.rt_orientation == None:
            H  = arm.ik.transfer_matrices()
            ra = arm.ik.task_frame[0].orientation(H)
            pint.rt_orientation = np.copy(ra['matrix'])

        if relative:
            H     = arm.ik.transfer_matrices()
            p0    = arm.ik.task_point[0].position(H) - pint.rt_position
            ra    = arm.ik.task_frame[0].orientation(H)
            R0    = np.dot(ra['matrix'], pint.rt_orientation.T)
        else:
            p0    = np.zeros(3)
            R0    = np.eye(3)  
        
        t     = time.time() - ts
        dt    = t - t0
        t0    = t

        stay  = True
        cnt   = 0
        q_dot = np.zeros(7)

        while stay:
            cnt  += 1
            p = p0 + pint.rt_position
            # p = p0 + pint.rt_position
            R = np.dot(R0, pint.rt_orientation)
            # arm.set_target(p, R)
            arm.ik.set_target([p], [geo.Orientation_3D(R, np.zeros((3,3)))])
            
            t     = time.time() - ts
            dt    = t - t0

            # print
            # print "q_dot          : ", q_dot
            # print "ik.q           : ", ik.q
            # print "ik.q + dt*q_dot: ", ik.q + dt*q_dot
            
            # ik.set_config(ik.q + dt*q_dot)

            # print "q_dt after sync: ", (ik.q - q_p)/dt

            err = arm.ik.pose_error()
            ern = arm.ik.pose_error_norm()
            Je  = arm.ik.error_jacobian()

            if arm.ik.ik_settings.algorithm == "JPI":
                Je_dag = np.linalg.pinv(Je)
            else:
                assert False

            # q_dot = - np.dot(Je_dag, np.append(pint.rt_velocity, np.zeros(3)) + k*err)
            # q_dot = - np.dot(Je_dag, k*np.append(pint.rt_velocity, np.zeros(3)))
            q_dot   = k*arm.ik.ik_direction()
            # q_dot = arm.ik_direction()
            
            qf = arm.config.q + q_dot*dt # estimated joint values in the future (next time step)
            arm.ik.set_config(qf)  
            '''
            if arm.ik.pose_error_norm() < ern:
                # error is expected to reduce, send the joint speed
                landa = 1.0
                pint.send_arm_joint_speed(q_dot, is_left_arm = self.larm_reference)
            else:
                # error is going to raise, send the speed with a gain
                landa = 0.01/(0.01 + ern)
            '''
            pint.send_arm_joint_speed(q_dot, is_left_arm = self.larm_reference)
            '''
            # pint.send_arm_joint_speed(q_dot*k, is_left_arm = self.larm_reference)                            

            if cnt == 100*(cnt/100):
                print
                print "cnt:     ", cnt
                print "traj pos:", pint.rt_position
                print "Actual:  ", ik.task_point[0].position(ik.transfer_matrices())
                print "Desired: ", ik.task_point[0].rd
                print "Pose err:", ik.pose_error_norm()   
                print "Qd Norm: ", np.linalg.norm(q_dot)
            '''
            t     = time.time() - ts
            t0    = t
            print "times: ", t, dt
            # time.sleep(0.01)
            '''
            print
            print "CJV: ", ik.q
            print "EJV: ", ik.q + dt*q_dot
            '''
            stay  = (t < 100.0)
        
        print "Time Out!"
        pint.send_arm_joint_speed(np.zeros(7), is_left_arm = self.larm_reference)
        pint.deactivate_trajectory_input()
        # self.sync_object()
