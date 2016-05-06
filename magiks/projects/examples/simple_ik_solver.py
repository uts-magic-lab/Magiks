# HEADER

'''   Header
@file:          simple_ik_solver.py
@brief:    	This example shows how to use MAGIKS to solve the inverse kinematic problem for a PA10 manipulator
@author:        Nima Ramezani; DFKI(Bremen,Germnay), UTS(Sydney, Australia)
@start date:    23 April 2016
@version:	1.0
Last Revision:  01 May 2016
'''

# BODY

# To run a quick and simple example, we will use pre-defined and default settings provided in MAGIKS modules.
# MAGIKS has a module named as 'manipulator_library.py' that contains pre-defined geometry and configuration settings for some manipulators. These manipulators include:
# PUMA, PA10, PR2ARM, EXO and AILA.
# So, to use the predefined geometry and configuration settings for PA10, you should put the the following script in your code:
import __init__
__init__.set_file_path( False )

import magiks.magiks_core.manipulator_library as manlib

gs  = manlib.PA10_geometry_settings
cs  = manlib.manip_config_settings('PA10')

# For end-effector and IK settings, you can select the default settings defined in modules 
# 'endeffector.py' and 'inverse_kinematics.py'. 
# You can then change some of these setting parameters: 

import magiks.magiks_core.inverse_kinematics as iklib
import magiks.taskspace.endeffector as eflib
es  = eflib.Endeffector_Settings()
iks = iklib.Inverse_Kinematics_Settings()

'''
Now, you are ready to construct your IK solver model. It is an instance of class ```Inverse_Kinematics```.
'''
 
pa10 = iklib.Inverse_Kinematics(cs, gs, es, iks) 

'''
By default, initialy, all the joint values of each manipulator are in the middle of their mechanical feasible ranges. Current joint values are stored in property ```q```. Check the initial joint values for PA10: 
'''
print pa10.q


'''
Now, let's set a random configuration for the manipulator within the feasible joint ranges:
'''
assert pa10.set_config(pa10.random_config())

'''
Let's see the joint values in the generated configuration  
'''
print pa10.config_str()

# q = [-170.92   11.62 -142.86 -100.87 -157.08  -59.38   24.1 ]    (Joints are all in range)
# These values may be different in your system

'''
To get the end-effector position corresponding with this configuration,
first, get the transfer matrices and then 
call method 'position()' of the first 'task_point' object:
'''
H = pa10.transfer_matrices()
target_p = pa10.task_point[0].position(H)
print target_p
# [-0.46763799, -0.38504628,  0.62643187]

'''
Similarly, you can get the orientation of the end-effector by 
calling method 'orientation()' of the first object of 'task_frame':
'''
target_o = pa10.task_frame[0].orientation(H)
print target_o.matrix()
#[[-0.00958766,  0.58264559, -0.81266979],
# [-0.89192486, -0.37241008, -0.25647763],
# [-0.45208198,  0.72238137,  0.52324663]]

'''
Method 'orientation()' returns an object of type 'Orientation_3D'
You can convert the rotation into any desired convention.
For example to get the orientation as unit quaternions:
'''
print target_o.quaternion()
# [ 0.53414625,  0.45814184, -0.1687683 , -0.69015295]

'''
Since MAGIKS can handle multiple customized end-effectors,
it uses a list of reference positions named as 'task_point' and 
a list of reference orientations named as 'task_frame'. 
Each element of this list is a reference point for position and a reference frame for the orientation
which specify the system end-effector.
By default, 'task_point' and 'task_frame' have only one element 
referring to the position and orientation of the last link of the chain.

We want to set this pose as the target for the end-effector.
Then, we will change the configuration into another random value and run MAGIKS IK engine to find a 
feasible joint configuration for the given target.
To set the current pose as target:
'''
pa10.set_target([target_p], [target_o])

'''
Now, let's change the configuration into another random set of joint values:
'''
assert pa10.set_config(pa10.random_config())

'''
The new configuration is the starting point for the IK problem. To see the joint values:  
'''
pa10.config_str()

'''
You can see the end-effector pose associated with this configuration:
'''
pa10.task_point[0].position(pa10.transfer_matrices())
pa10.task_frame[0].orientation(pa10.transfer_matrices()).quaternion()

#### Running IK in Pose Projection Scenario (PPS):
'''
To run the IK solver, we need to call function 'goto_taget()':
'''

assert pa10.goto_target()

'''
If the result is True, you have ssuccessfully solved the IK for this target pose.
You can see the solution:
'''
pa10.config_str()
# q = [-148.75  -11.98  127.48  135.59  -59.61 -160.29  104.2 ]    (Joints are all in range)

