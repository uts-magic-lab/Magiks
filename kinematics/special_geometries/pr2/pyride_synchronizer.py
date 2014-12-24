'''   Header
@file:          object_synchronizer.py
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

@version:	    1.0
Last Revision:  4 April 2014

Changes from ver 1.0:
'''

import pr2_kinematics as pr2lib
import PyPR2, numpy, math
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
    q = numpy.zeros(6)

    # Determining h_ts
    '''
    There is a 50 mm offset in x direction between my base footprint r_bo in the paper and the base_footprint defined for PR2.
    So I will consider this offset in calculating r_bo from navigation data
    '''
    p = PyPR2.getRelativeTF('base_footprint', 'torso_lift_link')['position']
    q[0] = p[2] # This is h_ts or theta[7]

    # Determining X_BO and Y_BO:
    p = PyPR2.getRobotPose()['position'] # This should give the global coordinates of the base_footprint
    assert gen.equal(p[2], 0.0, epsilon = 0.001) # The robot base_footprint must be on the ground
    q[1] = p[0] # This is X_BO (theta[8])
    q[2] = p[1] # This is Y_BO (theta[9])

    # Determining tau:
    
    o = PyPR2.getRobotPose()['orientation']
    o_unit = vecmat.normalize(o)
    R = rot.rotation_matrix(o_unit)
    uvect_i = vecmat.as_vector([1.0, 0.0, 0.0])
    uvect_k = vecmat.as_vector([0.0, 0.0, 1.0])
    q[3] = vecmat.vectors_angle(uvect_i, R[:, 0], positive_direction = uvect_k)  # This is tau (theta[10])
    
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



def larm_shoulder_pan_joint(in_degrees = True):
    '''
    returns the current angle of the shoulder pan joint of the left arm
    '''
    if in_degrees:
        gain = 180.0/math.pi
    else:
        gain = 1.0     
    lajd = PyPR2.getArmJointPositions(True)
    return(gain*lajd['l_shoulder_pan_joint'])

def rarm_shoulder_pan_joint(in_degrees = True):
    '''
    returns the current angle of the shoulder pan joint of the right arm
    '''
    if in_degrees:
        gain = 180.0/math.pi
    else:
        gain = 1.0     
    rajd = PyPR2.getArmJointPositions(False)
    return(gain*rajd['r_shoulder_pan_joint'])

def rarm_shoulder_lift_joint(in_degrees = True):
    '''
    returns the current angle of the shoulder lift joint of the right arm
    '''
    if in_degrees:
        gain = 180.0/math.pi
    else:
        gain = 1.0     
    rajd = PyPR2.getArmJointPositions(False)
    return(gain*rajd['r_shoulder_lift_joint'])

def larm_shoulder_lift_joint(in_degrees = True):
    '''
    returns the current angle of the shoulder lift joint of the left arm
    '''
    if in_degrees:
        gain = 180.0/math.pi
    else:
        gain = 1.0     
    rajd = PyPR2.getArmJointPositions(True)
    return(gain*rajd['l_shoulder_lift_joint'])

def larm_elbow_flex_joint(in_degrees = True):
    '''
    returns the current angle of the shoulder pan joint of the left arm
    '''
    if in_degrees:
        gain = 180.0/math.pi
    else:
        gain = 1.0     
    lajd = PyPR2.getArmJointPositions(True)
    return(gain*lajd['l_elbow_flex_joint'])

def rarm_elbow_flex_joint(in_degrees = True):
    '''
    returns the current angle of the shoulder pan joint of the right arm
    '''
    if in_degrees:
        gain = 180.0/math.pi
    else:
        gain = 1.0     
    rajd = PyPR2.getArmJointPositions(False)
    return(gain*rajd['r_elbow_flex_joint'])

def larm_wrist_position():
    '''
    returns the wrist position of the right arm comparable to function wrist_position() in the arm object
    '''    
    p0 = PyPR2.getRelativeTF('torso_lift_link' , 'l_shoulder_pan_link')['position']
    p  = PyPR2.getRelativeTF('torso_lift_link' , 'l_wrist_flex_link')['position']
    x = p[0] - p0[0]
    y = p[1] - p0[1]
    z = p[2] - p0[2]
    pos = [x,y,z]
    return(pos)    
"""
def rarm_gripper_position():
    '''
    returns the gripper position of the right arm comparable to function wrist_position() in the arm object
    '''    
    p0 = PyPR2.getRelativeTF('torso_lift_link' , 'r_shoulder_pan_link')['position']
    pr = vecmat.as_vector(PyPR2.getRelativeTF('torso_lift_link' , 'r_gripper_r_finger_tip_link')['position'])
    pl = vecmat.as_vector(PyPR2.getRelativeTF('torso_lift_link' , 'r_gripper_l_finger_tip_link')['position'])
    p  = (pl + pr)/2
    x = p[0] - p0[0]
    y = p[1] - p0[1]
    z = p[2] - p0[2]
    pos = [x,y,z]
    return(pos)    
"""
def rarm_wrist_position():
    '''
    returns the wrist position of the right arm comparable to function wrist_position() in the arm object
    '''    
    p0 = PyPR2.getRelativeTF('torso_lift_link' , 'r_shoulder_pan_link')['position']
    p  = PyPR2.getRelativeTF('torso_lift_link' , 'r_wrist_flex_link')['position']
    x = p[0] - p0[0]
    y = p[1] - p0[1]
    z = p[2] - p0[2]
    pos = [x,y,z]
    return(pos)    

def rarm_elbow_position():
    '''
    returns the elbow position of the right arm comparable to function wrist_position() in the arm object
    '''    
    p0 = PyPR2.getRelativeTF('torso_lift_link' , 'r_shoulder_pan_link')['position']
    p  = PyPR2.getRelativeTF('torso_lift_link' , 'r_elbow_flex_link')['position']
    x = p[0] - p0[0]
    y = p[1] - p0[1]
    z = p[2] - p0[2]
    pos = [x,y,z]
    return(pos)    

def larm_elbow_position():
    '''
    returns the elbow position of the right arm comparable to function wrist_position() in the arm object
    '''    
    p0 = PyPR2.getRelativeTF('torso_lift_link' , 'l_shoulder_pan_link')['position']
    p  = PyPR2.getRelativeTF('torso_lift_link' , 'l_elbow_flex_link')['position']
    x = p[0] - p0[0]
    y = p[1] - p0[1]
    z = p[2] - p0[2]
    pos = [x,y,z]
    return(pos)    

def rarm_gripper_position():
    '''
    Returns the global position of the right arm gripper finger tip
    '''
    pr = vecmat.as_vector(PyPR2.getRelativeTF('base_footprint' , 'r_gripper_r_finger_tip_link')['position'])
    pl = vecmat.as_vector(PyPR2.getRelativeTF('base_footprint' , 'r_gripper_l_finger_tip_link')['position'])
    p  = (pl + pr)/2

    p0 = p = PyPR2.getRobotPose()['position']

    x = p[0] + p0[0]
    y = p[1] + p0[1]
    z = p[2] + p0[2]

    pos = [x,y,z]
    return(pos)    

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
        super(PyRide_PR2, self).__init__(a0 = q_r[7], d2 = q_r[8], d4 = q_r[9], d7 = q_t[5], l0 = q_t[4])

        # Takes the object right arm to the current robot right arm configuration: (This must be done for left arm, head and other joints as well)
        
        q_d = numpy.concatenate((q_r[0:7], q_t[0:4], q_l[0:7]))    
        #self.rarm.config.set_config(q_r[0:7])
        #self.larm.config.set_config(q_l[0:7])
        '''
        print "self.ql[7] = ", self.ql[7]*180.0/math.pi
        print "self.qd[7] = ", q_d[7]*180.0/math.pi
        print "self.qh[7] = ", self.qh[7]*180.0/math.pi
        '''
        super(PyRide_PR2, self).set_config(q_d)

        # PyPR2.getRelativeTF('torso_lift_link' , 'r_wrist_flex_link')['position'] - (0.0, -0.188, 0.0)
        # t

        # t = PyPR2.getRelativeTF('r_shoulder_pan_link' , 'r_wrist_roll_link')
        # t = PyPR2.getRelativeTF('r_shoulder_lift_link', 'r_shoulder_pan_link')
 
        # PyPR2.moveArmWithJointPos(**rajd)

