## @file        	pyride_interpreter.py
#  @brief     		Contains simplified functions to control PR2 using pyride engine
#  @author      	Nima Ramezani Taghiabadi 
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. :   04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	5.0 
#
#  Last Revision:  	13 May 2015

'''
Changes from ver 4.0:
	1- Added functions for receiving online dmp trajectory from pyride
	
'''

import PyPR2, numpy, math, time
from math_tools.algebra  import vectors_and_matrices as vecmat
from math_tools.geometry import geometry as geo
from math_tools import general_math as gen

# from cgkit.cgtypes import quat, mat3

# Global Variables

## A boolean global variable indicating if the left arm joints have reached their target.
#  Set as True by default, this variable switches to False immidiately after any move arm task function is called for the left arm
#  and is set back to True when the left arm reaches its target
larm_reached = True

## A boolean global variable indicating if the right arm joints have reached their target.
#  Set as True by default, this variable switches to False immidiately after any move arm task function is called for the right arm
#  and is set back to True when the right arm reaches its target
rarm_reached = True

## A boolean global variable indicating if the body joints (non-arm joints) have reached their target.
#  Set as True by default, this variable switches to False immidiately after any move function is called 
#  for the body navigation joints or the prismatic lifter joint   
#  and is set back to True when the body joints reach their target, so it always shows the status of the lastest task.
body_reached = False

## A boolean global variable indicating if the left arm joints failed to reach their target.
#  Set as False by default, this variable switches to True only if any move arm task function is called 
#  and the left arm can not reach its target within a certain time. 
#  It is set back to False immidiately after a move arm task function is called for left arm, so it always shows the status of the lastest task.
larm_failed  = False


rarm_failed  = False
body_failed  = False
counter      = 0

## Counter for the base laser scan. This counter starts increasing as soon as 
#  the base laser scan is activated by function activate_base_laser()
bl_cnt       = 0
## Counter for the raw trajectory function calls. This counter starts increasing as soon as 
#  the raw trajectory input is activated by function activate_raw_trajectory_input()
rt_cnt       = 0

bl_dist      = None
bl_active    = False
tl_cnt       = 0
tl_dist      = None
tl_active    = False

rt_orientation   = None
rt_position      = numpy.zeros(3)
rt_velocity      = numpy.zeros(3)
rt_acceleration  = numpy.zeros(3)
rt_active        = False

# Service event functions

## Callback function for arm reach: This function is called when the arm joints reach their target values
#  Global variable larm_reached or rarm_reached will be switched to True depending on which arm is specified by argument is_left_arm
#  @param is_left_arm A boolean parameter specifying which arm is considered:
#  True for the left arm and False for the right arm.
#  @return None
def on_move_arm_finished( is_left_arm ):
    global larm_reached, rarm_reached
    if hasattr( PyPR2, 'onMoveArmPoseComplete' ) and hasattr( PyPR2.onMoveArmPoseComplete, '__call__' ):
      PyPR2.onMoveArmPoseComplete( is_left_arm )
    if is_left_arm:
        print "larm reached"
        larm_reached = True
    else:
        print "rarm reached"
        rarm_reached = True

## Callback function for arm reach failure: This function is called when the arm joints fail to reach their target values
#  Global variable larm_failed or rarm_failed will be switched to True depending on which arm is specified by argument is_left_arm
#  @param is_left_arm A boolean parameter specifying which arm is considered:
#  True for the left arm and False for the right arm.
#  @return None 
def on_move_arm_failed( is_left_arm ):
    global larm_failed, rarm_failed
    if hasattr( PyPR2, 'onMoveArmPoseFailed' ) and hasattr( PyPR2.onMoveArmPoseFailed, '__call__' ):
      PyPR2.onMoveArmPoseFailed( is_left_arm )
    if is_left_arm:
        print "larm failed"
        larm_failed = True
    else:
        rarm_failed = True
        print "rarm failed"

def on_move_body_finished():
    global body_reached
    body_reached = True
    print "body reached"

def on_move_body_failed():
    global body_failed
    body_failed = True
    print "body failed"

def on_base_laser(range, intensity):
    global bl_cnt, bl_dist, bl_active
    bl_active = True
    bl_cnt += 1
    bl_dist = range

def on_tilt_laser(range, intensity):
    global tl_cnt, tl_dist, tl_active
    tl_active = True
    tl_cnt += 1
    tl_dist = range

def set_callback_functions():
    PyPR2.onMoveArmActionSuccess = on_move_arm_finished
    PyPR2.onMoveArmActionFailed  = on_move_arm_failed
    PyPR2.onMoveBodySuccess      = on_move_body_finished
    PyPR2.onMoveBodyFailed       = on_move_body_failed

def on_trajectory_received( data ):
    global rt_cnt, rt_active
    rt_active = True
    rt_cnt += 1
    rt_position[0] = data['position'][0]
    rt_position[1] = data['position'][1]
    rt_position[2] = data['position'][2]

    rt_velocity[0] = data['velocity'][0]
    rt_velocity[1] = data['velocity'][1]
    rt_velocity[2] = data['velocity'][2]

    rt_acceleration[0] = data['acceleration'][0]
    rt_acceleration[1] = data['acceleration'][1]
    rt_acceleration[2] = data['acceleration'][2]

    rt_orientation[0] = data['acceleration'][2]

set_callback_functions()

# Conversion Functions

## Use this function to convert a vector of left arm joint values, into a joint dictionary known by pyride
#  @param q A numpy vector of 7 elements containing the desired left arm joint values
#  @time_to_reach A float specifying the amount of time (in seconds) needed to reach the target
#  @return A dictionary known by pyride which can be passed to function <provide the link>  
def gen_larm_joint_dict(q, time_to_reach = 5.0):
    g={'l_wrist_roll_joint': q[6], 'l_forearm_roll_joint': q[4], 'l_elbow_flex_joint': q[3], 'l_shoulder_lift_joint': q[1] - math.pi/2, 'l_upper_arm_roll_joint': q[2], 'l_wrist_flex_joint': q[5], 'l_shoulder_pan_joint': q[0], 'time_to_reach': time_to_reach }
    return(g)

def gen_rarm_joint_dict(q, time_to_reach = 5.0):
    g={'r_wrist_roll_joint': q[6], 'r_forearm_roll_joint': q[4], 'r_elbow_flex_joint': q[3], 'r_shoulder_lift_joint': q[1] - math.pi/2, 'r_upper_arm_roll_joint': q[2], 'r_wrist_flex_joint': q[5], 'r_shoulder_pan_joint': q[0], 'time_to_reach': time_to_reach }
    return(g)

def gen_rarm_joint_posvel_dict(q, q_dot, time_to_reach):
    g={'r_wrist_roll_joint': {'position':q[6], 'velocity': q_dot[6]}, 'r_forearm_roll_joint': {'position':q[4], 'velocity': q_dot[4]}, 'r_elbow_flex_joint': {'position':q[3], 'velocity': q_dot[3]}, 'r_shoulder_lift_joint': {'position':q[1] - math.pi/2, 'velocity': q_dot[1]}, 'r_upper_arm_roll_joint': {'position':q[2], 'velocity': q_dot[2]}, 'r_wrist_flex_joint': {'position':q[5], 'velocity': q_dot[5]}, 'r_shoulder_pan_joint': {'position':q[0], 'velocity': q_dot[0]}, 'time_to_reach':time_to_reach}
    return(g)

def gen_larm_joint_posvel_dict(q, q_dot, time_to_reach):
    g={'l_wrist_roll_joint': {'position':q[6], 'velocity': q_dot[6]}, 'l_forearm_roll_joint': {'position':q[4], 'velocity': q_dot[4]}, 'l_elbow_flex_joint': {'position':q[3], 'velocity': q_dot[3]}, 'l_shoulder_lift_joint': {'position':q[1] - math.pi/2, 'velocity': q_dot[1]}, 'l_upper_arm_roll_joint': {'position':q[2], 'velocity': q_dot[2]}, 'l_wrist_flex_joint': {'position':q[5], 'velocity': q_dot[5]}, 'l_shoulder_pan_joint': {'position':q[0], 'velocity': q_dot[0]}, 'time_to_reach':time_to_reach}
    return(g)

def gen_larm_joint_vel_dict(q):
    g={'l_wrist_roll_joint': q[6], 'l_forearm_roll_joint': q[4], 'l_elbow_flex_joint': q[3], 'l_shoulder_lift_joint': q[1], 'l_upper_arm_roll_joint': q[2], 'l_wrist_flex_joint': q[5], 'l_shoulder_pan_joint': q[0]}
    return(g)

def gen_rarm_joint_vel_dict(q):
    g={'r_wrist_roll_joint': q[6], 'r_forearm_roll_joint': q[4], 'r_elbow_flex_joint': q[3], 'r_shoulder_lift_joint': q[1], 'r_upper_arm_roll_joint': q[2], 'r_wrist_flex_joint': q[5], 'r_shoulder_pan_joint': q[0]}
    return(g)

def gen_rarm_joint_vector_from_dic(g):
    q = numpy.zeros(7)
    q[0] = g['r_shoulder_pan_joint']
    q[1] = g['r_shoulder_lift_joint'] + math.pi/2
    q[2] = g['r_upper_arm_roll_joint']
    q[3] = g['r_elbow_flex_joint']
    q[4] = g['r_forearm_roll_joint']
    q[5] = g['r_wrist_flex_joint']
    q[6] = g['r_wrist_roll_joint']
    return q

def gen_larm_joint_vector_from_dic(g):
    q = numpy.zeros(7)
    q[0] = g['l_shoulder_pan_joint']
    q[1] = g['l_shoulder_lift_joint'] + math.pi/2
    q[2] = g['l_upper_arm_roll_joint']
    q[3] = g['l_elbow_flex_joint']
    q[4] = g['l_forearm_roll_joint']
    q[5] = g['l_wrist_flex_joint']
    q[6] = g['l_wrist_roll_joint']
    return q

# Sensory Functions

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

def rarm_joints(in_degrees = True):
    if in_degrees:
        gain = 180.0/math.pi
    else:
        gain = 1.0     
    rajd = PyPR2.getArmJointPositions(False)
    return(gain*gen_rarm_joint_vector_from_dic(rajd))

def larm_joints(in_degrees = True):
    if in_degrees:
        gain = 180.0/math.pi
    else:
        gain = 1.0     
    lajd = PyPR2.getArmJointPositions(True)
    return(gain*gen_larm_joint_vector_from_dic(lajd))

def larm_wrist_position():
    '''
    returns the wrist position of the left arm comparable to function wrist_position() in the arm object
    '''    
    p0 = PyPR2.getRelativeTF('torso_lift_link' , 'l_shoulder_pan_link')['position']
    p  = PyPR2.getRelativeTF('torso_lift_link' , 'l_wrist_flex_link')['position']
    x = p[0] - p0[0]
    y = p[1] - p0[1]
    z = p[2] - p0[2]
    pos = numpy.array([x,y,z])
    return(pos)    

def larm_wrist_orientation():
    '''
    returns the wrist orientation of the left arm (the same orientation returned by function wrist_orientation() in the arm object but in quaternions)
    '''    
    q  = PyPR2.getRelativeTF('torso_lift_link' , 'l_wrist_flex_link')['orientation']
    return(q)    

def rarm_wrist_orientation():
    '''
    returns the wrist orientation of the right arm (the same orientation returned by function wrist_orientation() in the arm object but in quaternions)
    '''    
    q  = PyPR2.getRelativeTF('torso_lift_link' , 'r_wrist_flex_link')['orientation']
    return(q)    

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
    pos = numpy.array([x,y,z])
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
    pos = numpy.array([x,y,z])
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
    pos = numpy.array([x,y,z])
    return(pos)    

def pos_rarm_grip_wrt_tor_shpan():
    '''
    Returns the right arm gripper position vector(endeffector position) with respect to the torso shoulder pan joint center
    '''
    p0 = PyPR2.getRelativeTF('torso_lift_link' , 'r_shoulder_pan_link')['position']
    pr = vecmat.as_vector(PyPR2.getRelativeTF('torso_lift_link' , 'r_gripper_r_finger_tip_link')['position'])
    pl = vecmat.as_vector(PyPR2.getRelativeTF('torso_lift_link' , 'r_gripper_l_finger_tip_link')['position'])
    p  = (pl + pr)/2
    x = p[0] - p0[0]
    y = p[1] - p0[1]
    z = p[2] - p0[2]
    pos = numpy.array([x,y,z])
    return(pos)    

def pos_larm_grip_wrt_tor():
    '''
    Returns the left arm gripper position vector(endeffector position) with respect to the (torso) at the origin.
    The torso origin is at the floor footprint (projection) of the middle point between the two shoulder pan joint centers.
    The torso origin is about 51 mm below the base_link origin and 50 mm shifted towards the back. The orientations of torso and base are identical.
    '''
    
    p0 = PyPR2.getRelativeTF('base_link' , 'l_shoulder_pan_link')['position']  
    p  = p0 + pos_larm_grip_wrt_tor_shpan() + numpy.array([0.05, 0.0, 0.051])
    
    return p

def pos_rarm_grip_wrt_tor():
    '''
    Returns the right arm gripper position vector(endeffector position) with respect to the (torso) at the origin. (Refer to pos_larm_grip_wrt_tor())
    '''
    
    p0 = PyPR2.getRelativeTF('base_link' , 'r_shoulder_pan_link')['position']  
    p  = p0 + pos_rarm_grip_wrt_tor_shpan() + numpy.array([0.0, 0.0, 0.051])
    
    return p

def pos_larm_grip_wrt_tor_shpan():
    '''
    Returns the left arm gripper position vector(endeffector position) with respect to the torso shoulder pan joint center
    '''
    p0 = PyPR2.getRelativeTF('torso_lift_link' , 'l_shoulder_pan_link')['position']
    pr = vecmat.as_vector(PyPR2.getRelativeTF('torso_lift_link' , 'l_gripper_r_finger_tip_link')['position'])
    pl = vecmat.as_vector(PyPR2.getRelativeTF('torso_lift_link' , 'l_gripper_l_finger_tip_link')['position'])
    p  = (pl + pr)/2
    x = p[0] - p0[0]
    y = p[1] - p0[1]
    z = p[2] - p0[2]
    pos = numpy.array([x,y,z])
    return(pos)    

def pos_rarm_grip():
    '''
    Returns the global position of the right arm gripper finger tip
    '''
    pr = vecmat.as_vector(PyPR2.getRelativeTF('base_footprint' , 'r_gripper_r_finger_tip_link')['position'])
    pl = vecmat.as_vector(PyPR2.getRelativeTF('base_footprint' , 'r_gripper_l_finger_tip_link')['position'])
    p  = (pl + pr)/2

    orient = geo.Orientation_3D(PyPR2.getRobotPose()['orientation'], representation = 'quaternion')
    pg     = numpy.dot(orient.matrix(), p) # p global is Rb Multiply by p

    p0 = PyPR2.getRobotPose()['position']

    x = pg[0] + p0[0]
    y = pg[1] + p0[1]
    z = pg[2] + p0[2]

    pos = numpy.array([x,y,z])
    return(pos)    

def pos_larm_grip():
    '''
    Returns the global position of the left arm gripper finger tip
    '''
    pr = vecmat.as_vector(PyPR2.getRelativeTF('base_footprint' , 'l_gripper_r_finger_tip_link')['position'])
    pl = vecmat.as_vector(PyPR2.getRelativeTF('base_footprint' , 'l_gripper_l_finger_tip_link')['position'])
    p  = (pl + pr)/2

    orient = geo.Orientation_3D(PyPR2.getRobotPose()['orientation'], representation = 'quaternion')
    '''
    qt = quat(h)   # convert to cgkit quaternion
    Rb = qt.toMat3()  # convert to rotation matrix 
    '''
    pg = numpy.dot(orient.matrix(), p) # p global is Rb Multiply by p

    p0 = PyPR2.getRobotPose()['position']

    x = pg[0] + p0[0]
    y = pg[1] + p0[1]
    z = pg[2] + p0[2]

    pos = numpy.array([x,y,z])
    return(pos)    

def body_position():
    return(PyPR2.getRobotPose()['position'])

def body_angle(in_degrees=True ):
    if in_degrees:
        gain = 180.0/math.pi
    else:
        gain = 1.0     

    orient = geo.Orientation_3D(PyPR2.getRobotPose()['orientation'], representation = 'quaternion')
    tau = orient.angle()
    u   = orient.axis()
    return (gen.sign(u[2])*gain*tau)

def bl_midpoint(): 
    '''
    Returns the univrsal coordinates of the base laser middle point
    '''
    p0 = PyPR2.getRelativeTF('base_footprint' , 'base_laserscan')['position']    

# Actuating Functions

'''
def activate_trajectory_input(name = 'fig8', amplitude = 1.0, system_freq = 0.1, sample_freq = 2, cycle = 1):
    global rt_cnt
    rt_cnt = 0
    PyPR2.registerRawTrajectoryInput( on_trajectory_received )
    PyPR2.recallRhythDMPTrajectory(name=name, amplitude=amplitude, system_freq= system_freq, sample_freq= sample_freq, cycle= cycle)
'''

def activate_trajectory_input():
    '''
    To start receiving raw trajectory input, you should send a ROS service request call:
    rosservice call /rhyth_dmp/recall_dmp_traj <figure name> <amplitude> <frequency> <sampling frequency> <number of cycles>
    For example:
    rosservice call /rhyth_dmp/recall_dmp_traj fig8 0.05 0.2 20 3

    generates 3 cycles of figure eight with frequency 1.0
    '''
    global rt_cnt
    rt_cnt = 0
    PyPR2.registerRawTrajectoryInput( on_trajectory_received )
    assert PyPR2.useJointVelocityControl(True)

def deactivate_trajectory_input():
    global rt_cnt, rt_position, rt_velocity, rt_acceleration, rt_orientation
    rt_cnt = 0
    PyPR2.registerRawTrajectoryInput( None )
    assert PyPR2.useJointVelocityControl(False)
    rt_orientation   = None
    rt_position      = numpy.zeros(3)
    rt_velocity      = numpy.zeros(3)
    rt_acceleration  = numpy.zeros(3)

def activate_base_laser():
	global bl_cnt
	bl_cnt = 0
	PyPR2.registerBaseScanCallback( on_base_laser )

def activate_tilt_laser():
	global tl_cnt
	tl_cnt = 0
	PyPR2.registerTiltScanCallback( on_tilt_laser )

def deactivate_base_laser():
    global bl_active
    PyPR2.registerBaseScanCallback( None )
    bl_active = False
    
def deactivate_tilt_laser():
    global tl_active
    PyPR2.registerBaseScanCallback( None )
    tl_active = False

def set_lg(x = 1):
    '''
    sets the position of left gripper from 0 to 8
    '''
    if x > 8:
        x = 8
    if x < 0:
        x = 0

    PyPR2.setGripperPosition(1, 0.01*x)

def set_rg(x = 1):
    '''
    sets the position of right gripper from 0 to 8
    '''
    if x > 8:
        x = 8
    if x < 0:
        x = 0

    PyPR2.setGripperPosition(2, 0.01*x)

def twist_forearm(angle = 180.0, is_left_arm = True):
        if is_left_arm:
            joint_name = 'l_forearm_roll_joint'
        else:
            joint_name = 'r_forearm_roll_joint'
            
        jd   = PyPR2.getArmJointPositions(is_left_arm)
        jd[joint_name] = jd[joint_name] + angle*math.pi/180.0
        PyPR2.moveArmWithJointPos(**jd)    

def twist_lg(angle = 180.0):
        jd   = PyPR2.getArmJointPositions(True)
        jd['l_wrist_roll_joint'] = jd['l_wrist_roll_joint'] + angle*math.pi/180.0
        PyPR2.moveArmWithJointPos(**jd)    

def twist_rg(angle = 180.0):
        jd   = PyPR2.getArmJointPositions(False)
        jd['r_wrist_roll_joint'] = jd['r_wrist_roll_joint'] + angle*math.pi/180.0
        PyPR2.moveArmWithJointPos(**jd)    

def look_lg():
    #pos = pos_larm_grip_wrt_tor_shpan()
    pos = PyPR2.getRelativeTF( '/base_link', '/l_gripper_tool_frame' )['position']
    PyPR2.pointHeadTo('base_link', pos[0],pos[1],pos[2]) 

def look_rg():
    pos = PyPR2.getRelativeTF( '/base_link', '/r_gripper_tool_frame' )['position']
    PyPR2.pointHeadTo('base_link', pos[0],pos[1],pos[2]) 

def take_rarm_to(qd, time_to_reach = 5.0):
    global rarm_reached, rarm_failed
    rarm_reached = False
    rarm_failed  = False
    g = gen_rarm_joint_dict(qd, time_to_reach)
    PyPR2.moveArmWithJointPos(**g)

def take_larm_to(qd, time_to_reach = 5.0):
    global larm_reached, larm_failed
    larm_reached = False
    larm_failed  = False
    g = gen_larm_joint_dict(qd, time_to_reach)
    PyPR2.moveArmWithJointPos(**g)

def take_robot_to(X, Y, time_to_reach = 5.0):
    global body_reached, body_failed
    body_reached = False
    body_failed  = False
    rp = PyPR2.getRobotPose()
    P0 = rp['position']
    dX = X - P0[0]
    dY = Y - P0[1]
    tau = body_angle(in_degrees = False)
    S   = math.sin(tau)
    C   = math.cos(tau)
    dx =  C*dX + S*dY
    dy = -S*dX + C*dY

    PyPR2.moveBodyTo(dx, dy, 0.0, time_to_reach)

def rotate_robot_to(desired_angle, time_to_reach = None, in_degrees = True):
    global body_reached, body_failed
    max_speed = math.pi/18.0 # (The maximum speed is 10 degrees per second)
    if in_degrees:
        gain = math.pi/180.0
    else:
        gain = 1.0     
    body_reached = False
    body_failed  = False
    tau0         = body_angle(in_degrees = in_degrees)
    d_theta      = gain*(desired_angle - tau0)
    if time_to_reach == None:
        time_to_reach = abs(d_theta/max_speed)    

    PyPR2.moveBodyTo(0.0, 0.0, d_theta, time_to_reach + 2.0) 	

def move_robot_to(x, y , tau, time_to_reach = 5.0, in_degrees = True):
    global body_reached, body_failed
    if in_degrees:
        gain = math.pi/180.0
    else:
        gain = 1.0     
    body_reached = False
    body_failed  = False
    rp = PyPR2.getRobotPose()
    p0 = rp['position']
    dx = x - p0[0]
    dy = y - p0[1]

    '''
    o  = quat(rp['orientation'])   # convert to cgkit quaternion
    R  = numpy.linalg.inv(o.toMat3())
    '''
    o  = geo.Orientation_3D(rp['orientation'], representation = 'quaternion')
    R  = numpy.linalg.inv(o.matrix())

    dp = numpy.dot(R.T, vecmat.as_vector([dx, dy, 0.0]))
    tau0 = body_angle(in_degrees = in_degrees)
    PyPR2.moveBodyTo( dp[0], dp[1], gain*(tau - tau0), time_to_reach)

def finished(limb_list = ['body', 'rarm', 'larm']):
    '''
    returns True if the motion of all the items in the limb_list is reached or failed.
    '''
    global body_reached, body_failed
    fin = True
    if 'body' in limb_list:
        fin = fin and (body_reached or body_failed)
    if 'rarm' in limb_list:
        fin = fin and (rarm_reached or rarm_failed)
    if 'larm' in limb_list:
        fin = fin and (larm_reached or larm_failed)
    return fin
    
def wait_until_finished(limb_list = ['body', 'rarm', 'larm'], max_time = 20.0 ):
    '''
    waits until the motion of the items in the limb_list is reached or failed.
    the function will return False if the motion of items is not finished by the max_time
    '''    
    t0 = time.time()
    t  = 0.0
    while (t < max_time) and (not finished(limb_list)):
        t = time.time() - t0
        time.sleep(0.01)    
    if finished(limb_list):
        return True
    else:
        print "Time Out !"
        return False

def run_navigation_trajectory(duration, pos_traj, ori_traj, phi_dot = 1.0, k = 1.0):
    t_s   = time.time()
    t     = 0.0
    pos_traj.set_phi(0.0)
    while t < duration:
        t0 = t
        # Get the desired pose:
        pos_traj.set_phi(phi_dot*t)
        ori_traj.set_phi(phi_dot*t)
        xd   = pos_traj.current_position[0]
        yd   = pos_traj.current_position[1]
        thd  = ori_traj.current_position[0]
        # Get the desired pose speed:
        vxd  = pos_traj.current_velocity[0]*phi_dot
        vyd   = pos_traj.current_velocity[1]*phi_dot
        vthd = ori_traj.current_velocity[0]*phi_dot
        # Get the actual pose:
        PyPR2.getRobotPose()
        rp  = PyPR2.getRobotPose()
        p0  = rp['position']
        xa  = p0[0]
        ya  = p0[1]
        tha = body_angle(in_degrees = False)
        # calculate error:
        ex  = xa - xd
        ey  = ya - yd
        eth = tha - thd
        # find the control speed to be sent:
        vx  = vxd  - k*(xa - xd)        
        vy  = vyd  - k*(ya - yd)        
        vth = vthd - k*(tha - thd)        
        
        PyPR2.moveBodyWithSpeed(vx, vy, vth)

        t      = time.time() - t_s
        dt     = t - t0

'''        
def run_config_trajectory(j_traj, n = 10, is_left_arm = False):
    phi      = 0.0
    ttr      = 5.0
    dic_list = []

    for i in range(n):
        j_traj.set_phi(i*j_traj.phi_end/(n-1))
        if is_left_arm:
            g   = gen_larm_joint_dict(q = j_traj.current_position, time_to_reach = ttr)
        else:
            g   = gen_rarm_joint_dict(q = j_traj.current_position, time_to_reach = ttr)

        ttr = j_traj.current_phi - phi
        phi = j_traj.current_phi
        dic_list.append(g) 

    PyPR2.moveArmWithJointTrajectory(dic_list)
''' 
       
## Runs a given arm joint trajectory on the robot.
#  @param j_traj An instance of class packages.nima.robotics.kinematics.task_space.trajectory.Trajectory()  
def run_config_trajectory(j_traj, duration = 10.0, dt = None, phi_dot = None, is_left_arm = False):
    global rarm_reached, rarm_failed
    global larm_reached, larm_failed
    if is_left_arm:
        larm_failed  = False
        larm_reached = False
    else:
        rarm_failed  = False
        rarm_reached = False

    phi      = 0.0
    dic_list = []
    t = 0.0

    if phi_dot == None:
        phi_dot = j_traj.phi_end/duration
    if dt == None:
        dt = 0.1*duration

    stay = True
    while stay:
        if (t > duration) or gen.equal(t, duration, epsilon = 0.1*dt):
            t    = duration
            stay = False

        j_traj.set_phi(t*phi_dot)
        if is_left_arm:
            g   = gen_larm_joint_posvel_dict(j_traj.current_position, j_traj.current_velocity*phi_dot, dt)
        else:
            g   = gen_rarm_joint_posvel_dict(j_traj.current_position, j_traj.current_velocity*phi_dot, dt)

        dic_list.append(g) 

        t = t + dt

    PyPR2.moveArmWithJointTrajectoryAndSpeed(dic_list)

def send_arm_joint_speed(q_dot, is_left_arm = False):
    if is_left_arm:
        g = gen_larm_joint_vel_dict(q_dot)
    else:
        g = gen_rarm_joint_vel_dict(q_dot)

    PyPR2.moveArmWithJointVelocity(**g)

'''

The problem is here:
>>> ps.pint.larm_joints(in_degrees = False)
array([ 0.51655997,  1.7272608 ,  2.4242567 , -1.93160875, -1.00904676,
       -1.99999936, -6.15865849])
>>> b.larm.config.q
array([ 0.51620768,  1.72741597,  2.42425919, -1.93156323, -1.00904459,
       -2.26706906,  0.1245308 ])
'''
