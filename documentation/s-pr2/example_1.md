This example shows how to use S-PR2 to control the PR2 robot in simulation or real envirionment.

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

## Introduce python packages to PyRide
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

## Create a S-PR2 object:

To start working with S-PR2, you need to create a 
[```Skilled_PR2()```](http://uts-magic-lab.github.io/Magiks/classmagiks_1_1specific__geometries_1_1pr2_1_1skilled__pr2_1_1_skilled___p_r2.html)
object:

```
from magiks.specific_geometries.pr2 import skilled_pr2 as spr
obj = spr.Skilled_PR2()
```

