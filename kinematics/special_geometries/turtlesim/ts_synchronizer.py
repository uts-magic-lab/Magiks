'''   Header
@file:          pyride_synchronizer.py
@brief:    	    Contains a class inherited from PR2 in pr2_kinematics.py which is connected to a real-time robot (A real PR2 or PR2 in simulation)
@author:        Nima Ramezani Taghiabadi
                PhD Researcher
                Faculty of Engineering and Information Technology
                University of Technology Sydney (UTS)
                Broadway, Ultimo, NSW 2007, Australia
                Room No.: CB10.03.512
                Phone:    02 9514 4621
                Mobile:   04 5027 4611
                Email(1): Nima.RamezaniTaghiabadi@uts.edu.au 
                Email(2): Nima.RamezaniTaghiabadi@student.uts.edu.au 
                Email(3): N.RamezaniTaghiabadi@uws.edu.au 
                Email(4): nima.ramezani@gmail.com
                Email(5): nima_ramezani@yahoo.com
                Email(6): ramezanitn@alum.sharif.edu

@version:	    2.0
Start date:     9 April 2014
Last Revision:  9 April 2014

Changes from ver 1.0:
'''

import pr2_kinematics as pr2lib
import pyride_interpreter as pint
import PyPR2, numpy, math, time

import packages.nima.mathematics.general as gen
import packages.nima.mathematics.trigonometry as trig
import packages.nima.mathematics.rotation as rot
import packages.nima.mathematics.vectors_and_matrices as vecmat


r_arm_joint_names =['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
l_arm_joint_names =['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']

def calibrate_pr2_r_arm():
    # Calibration:
    '''
    Trying to compute DH parameters of the right arm according to the FK measured from the robot
    '''
    q = numpy.zeros(10)    

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
    q = numpy.zeros(10)    

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
    q = numpy.zeros(7)

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
    
    o = PyPR2.getRobotPose()['orientation']
    assert gen.equal(numpy.linalg.norm(o), 1.0)
    # h = (o[3],o[0],o[1], o[2])
    R = rot.rotation_matrix(o)
    uvect_i = vecmat.as_vector([1.0, 0.0, 0.0])
    uvect_k = vecmat.as_vector([0.0, 0.0, 1.0])
    q[3] = vecmat.vectors_angle(R[:, 0], uvect_i, positive_direction = uvect_k) # This is tau (theta[10])
    
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


class PyRide_PR2(pr2lib.PR2):
    '''
    Any changes in the status and configuration of the robot is directly applied to the real robot or robot in simulation    
    '''
    
    def __init__(self):

        q_d = numpy.zeros(18)
        self.r_arm_joint_names = r_arm_joint_names
        self.l_arm_joint_names = l_arm_joint_names

        q_r = calibrate_pr2_r_arm()
        q_l = calibrate_pr2_l_arm()
        q_t = calibrate_pr2_torso()

        assert gen.equal(q_r[7], q_l[7])
        assert gen.equal(q_r[8], q_l[8])
        assert gen.equal(q_r[9], q_l[9])

        #pr2lib.PR2.__init__(self)
        super(PyRide_PR2, self).__init__(a0 = q_r[7], d2 = q_r[8], d4 = q_r[9], d7 = q_t[5], l0 = q_t[4], b0 = q_t[6])

        q_d = numpy.concatenate((q_r[0:7], q_t[0:4], q_l[0:7]))    
        super(PyRide_PR2, self).set_config(q_d)
    
    def body_synced(self):
        '''
        Returns True if robot and object body positions are identical
        '''    
        bp = pint.body_position()
        ba = pint.body_angle(in_degrees = False)
        fx = gen.equal(bp[0], self.q[8], epsilon = 0.1)
        fy = gen.equal(bp[1], self.q[9], epsilon = 0.1)
        ft = gen.equal(ba   , self.q[10], epsilon = 0.1)

        return(fx and fy and ft)

    def rarm_synced(self):
        return vecmat.equal(trig.angles_standard_range(pint.rarm_joints(in_degrees = False)), trig.angles_standard_range(self.rarm.config.q), epsilon = 0.01)

    def larm_synced(self):
        return vecmat.equal(trig.angles_standard_range(pint.larm_joints(in_degrees = False)), trig.angles_standard_range(self.larm.config.q), epsilon = 0.01)

    def sync(self):

        '''
        Synchronizes the object with the robot and verifies the forwark kinematics of both the endeffectors
        '''        
        q_r = calibrate_pr2_r_arm()
        q_l = calibrate_pr2_l_arm()
        q_t = calibrate_pr2_torso()

        q_d = numpy.concatenate((q_r[0:7], q_t[0:4], q_l[0:7]))    

        assert super(PyRide_PR2, self).set_config(q_d)
        
        laggp1 = self.larm_endeffector_position() # Left Arm Global Gripper Position from object fk
        laggp2 = pint.pos_larm_grip() # Left Arm Global Gripper Position measured from robot

        assert vecmat.equal(laggp1, laggp2, epsilon = 0.01) # verify equality with 1 cm accuracy

        raggp1 = self.rarm_endeffector_position() # Right Arm Global Gripper Position from object fk
        raggp2 = pint.pos_rarm_grip() # Right Arm Global Gripper Position measured from robot

        assert vecmat.equal(raggp1, raggp2, epsilon = 0.01) # verify equality with 1 cm accuracy

    def sync_robot(self, q_d, ttr = 1.0):

        assert gen.equal(q_d[7], self.q[7]) # At the moment, we do not have any function in PyRide for moving the prismatic(telescopic) lifting joint. This equality test will be removed when we have it.

        assert self.set_config(q_d) # Make sure the given joints are feasible

        pint.take_rarm_to(self.rarm.config.q, time_to_reach = ttr) # Move the right arm
        pint.take_larm_to(self.larm.config.q, time_to_reach = ttr) # Move the left  arm
        pint.move_robot_to(x = self.q[8], y = self.q[9], tau = self.q[10], time_to_reach = ttr)  # Move the robot body first

        time.sleep(ttr)
        counter = 1
        while ((not self.body_synced()) or (not self.rarm_synced()) or (not self.larm_synced())) and (counter<20):
            counter = counter + 1
            print "Counter = ", counter
            if pint.body_reached:
                assert self.body_synced()
            else:
                if not pint.body_failed:
                    PyPR2.cancelMoveBodyAction()
                pint.move_robot_to(x = self.q[8], y = self.q[9], tau = self.q[10], time_to_reach = ttr)  # Move the robot body first

            if pint.rarm_reached:
                assert self.rarm_synced()
            else:
                if not pint.rarm_failed:
                    PyPR2.cancelMoveArmAction(False)
                pint.take_rarm_to(self.q[0:7], time_to_reach = ttr)  # Move the robot body first

            if pint.larm_reached:
                assert self.larm_synced()
            else:
                if not pint.larm_failed:
                    PyPR2.cancelMoveArmAction(True)
                pint.take_larm_to(self.q[11:18], time_to_reach = ttr)  # Move the robot body first

            time.sleep(ttr)
            
        if (counter<20):
            assert vecmat.equal(self.q, q_d)
            print("fekr nakonam halahalaha ino bebini !")
        else:
            print("Sharmande dada! Hopefully next time :-)")

