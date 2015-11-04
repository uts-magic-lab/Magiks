# A practical example

This example shows how to use S-PR2 to control the PR2 robot in simulation or real envirionment.

### Launch PyRide

Before proceeding, make sure that PyRide is installed on your computer.
PyRide is required to communicate with ROS in order to send and receive data to/from S-PR2 object into the real robot.
If you have installed PyRide, please find the instructions 
[here](https://github.com/uts-magic-lab/pyride_pr2/blob/master/README.md).

**Note**
If you are running your project in Gazebo simulation environment, make sure you have installed Gazebo on your computer.
Then before launching Pyride, you need to have the Gazebo simulation environment launched and PR2 spawned:

```
roslaunch gazebo_ros empty_world.launch
roslaunch pr2_gazebo pr2.launch
```

After the installation, launch PyRide in the ROS environment in which PR2 is working:

```
roslaunch pyride_pr2 pyride.launch
```

You can now connect to the python interface by using this command:

```
telnet host 27005
```

where ```host``` is the robot/machine on which PyRIDE is running. If pyride is running on the same computer as the MAGIKS is installed
replace it by ```localhost```. 

If everything is running well, you should be in the python console environment.

### Introduce python packages to PyRide

For the time being, PyRide requires all the packages paths to be added to the ```sys.path``` list.
Before start, you need to add the path of the installed packages to the list of system packages in pyride python environment.
You should do this for all the packages required by magiks:  numpy, sympy, pyinterval, matplotlib

Example:

```
import sys

numpy_path      = '/usr/lib/python2.7/dist-packages/'
sympy_path      = '/usr/local/lib/python2.7/dist-packages/'
pyinterval_path = '/usr/local/lib/python2.7/dist-packages/pyinterval-1.0b21-py2.7-linux-x86_64.egg/'
mtpltlib_path   = '/usr/lib/pymodules/python2.7'

sys.path.append(sympy_path)
sys.path.append(numpy_path)
sys.path.append(pyinterval_path)
sys.path.append(mtpltlib_path)
```

These paths might be different in your computer.

Finally add the path to the MAGIKS directory:

```
sys.path.append(<your_magiks_path>)
```

### Create a S-PR2 object:

To start working with S-PR2, you need to create a 
[```Skilled_PR2()```](http://uts-magic-lab.github.io/Magiks/classmagiks_1_1specific__geometries_1_1pr2_1_1skilled__pr2_1_1_skilled___p_r2.html)
object:

```
from magiks.specific_geometries.pr2 import skilled_pr2 as spr
obj = spr.Skilled_PR2()
```

Now you can start to work with the robot.

### Arm Basic Motion Premitives:

Arm Basic Movement Premitive (ABMP) methods can be used to move the arms towards specific directions.
The object has a reference arm on which every arm movement method, is applied.
It can be either left or right arm. 
To check which one is the current reference arm, check boolean property ```obj.larm_reference```.
If it is ```True```, left arm, otherwise right arm is the reference arm.
You can change the reference arm by setting this property:

To move the right arm *10 (cm)* backwards:

```
obj.larm_reference = False #set the reference arm to right arm
obj.arm_back()
```

The right arm pulls back *10 (cm)* towards torso maintaining the orientation.
The default movement length is *10 (cm)*. You can set your desired length by setting argument **dx**:
```
obj.arm_back(dx = 0.15)
```

You can move the arm maintaining its orientation to one of the six basic directions w.r.t. the torso:
```
obj.arm_forward()
obj.arm_back()
obj.arm_up()
obj.arm_down()
obj.arm_left()
obj.arm_right()
```
In all these movements, the reference for the direction is torso.
You can change the reference to the arm gripper (end-effector) by setting
boolean argument **relative** to ```True```.
For example, to move the left arm forward *12 (cm)* towards the gripper pointing direction:
```
obj.larm_reference = True #sets the reference arm to left arm
obj.arm_forward(dx = 0.12, relative = True)
```

You can change the arm orientation to one of the six basic orientations w.r.t. the torso:
```
obj.larm_reference = False
obj.arm_orient('forward') # default
obj.arm_orient('backward')
obj.arm_orient('upward')
obj.arm_orient('downward')
obj.arm_orient('left')
obj.arm_orient('right')
```

You can change the motion speed by setting property ```obj.arm_speed``` (set to *0.05* by default)

### Move to a target pose in the task-space

To move the arm to a specific position and orientation in the task-space, you need to first set a target and then move the arm:
```
pos_l = obj.larm.wrist_position()    # Read the left arm position w.r.t. the left arm base
ori_l = obj.larm.wrist_orientation() # Read the left arm orientation w.r.t. the torso
obj.rarm.set_target(pos_l, ori_l)    # Set the left arm pose as a target for the right arm
obj.larm_reference = False           # Make sure right arm is the reference arm
obj.arm_target()                     # Move the arm to the desired pose
```

### Draw a trajectory

To draw a trajectory, you should define it.
For this example, we will use one of the predefined trajectories in the library.
You can create your own trajectory easily by specifying the path points with your desired position, speed and acceleration for each point.
Here we pick one pre-defined trajectory from library:

```
from math_tools.geometry import shape_trajectories as sht
S = sht.S(height = 0.2)
```
To have a three-dimensional plot of the selected trajectory, type:
```
S.plot3d()
```

Now run the trajectory on the arm:
```
obj.arm_orient()        # Change the gripper orientation to forward for a better view 
obj.arm_back(dx = 0.25) # Move back for a better dexterity for the arm
obj.arm_trajectory(S)   # Run the motion
```
