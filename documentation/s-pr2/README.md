#Skilled-PR2 (S-PR2)


##Introduction

S-PR2 is a new and powerful Python based software package that can be used as a kinematic control toolbox for PR2 service robot. 
The software exploits an analytic Inverse Kinematic solver for PR2 redundant arm (7 DOF)
with capability of optimum redundancy resolution. 
S-PR2 can be used by robot researchers and programmers to generate their desired trajectories in the operational space and execute
on a physical robot or in simulation. 
The package gives PR2 robot, the skills of tracking a user-defined trajectory like writing letters or drawing shapes on the board.

S-PR2 provides an efficient and reliable development environment providing optimal and feasible kinematic control 
functions for PR2 users. S-PR2 has adopted the lastest techniques of the Inverse Kinematic (IK) solution in recent years.

#Why use S-PR2?

The existing IK solver for the PR2, which is a part of Robot Operating System (ROS),
is fast but lacks some important advanced features required for generating smooth and reliable trajectories. 
The existing package:

* Only projects a single pose to the joint-space and not a trajectory
* Does not provide all the available solutions (corresponding to a certain value of a redundant parameter) 
* Can not exploit the available redundancy in Degrees of Freedom (DOF) to minimise a desired cost function or to fulfil additional constraints
* Cannot give self motion trajectory connecting two points in the solution manifold
* Does not return a feasibility set for a redundant parameter to be chosen
* Is fully embedded within the ROS system and cannot be easily replaced or migrated to other systems
* Its algorithms are only provided in binary and so not available to inspect or modify

<img src="https://cloud.githubusercontent.com/assets/6646691/9621510/8ebab36a-516a-11e5-81f4-b04c9531ca7d.png" width="500">

**Figure 2. S-PR2 Remote Shell Access.**

##Prerequisites

Currently, Magiks is dependant on the following Python library packages

* crlibm (required by pyinterval).
* pyinterval.
* matplotlib.
* numpy.
* sympy.
* cgkit.

The following sections give detailed instructions on how these required libraries can be installed on your system. **NOTE** that we assume your target machine use a recent Ubuntu Linux system.

### Install numpy,sympy and other required dependancies
First, we install required packages from the standard Ubuntu distribution repository:

1. ```sudo apt-get update``` (make sure we have latest package listing).
2. ```sudo apt-get install python-pip python-pygame python-matplotlib python-numpy python-sympy scons```.

### Compile and install crlibm library
There is no standard package for crlibm library on Ubuntu/Debian system. The
source code can be downloaded from [here](http://lipforge.ens-lyon.fr/frs/download.php/162/crlibm-1.0beta4.tar.gz). Use the following commands to compile and install the library from a terminal:

1. ```tar zxf crlibm-1.0beta4.tar.gz;cd crlibm-1.0beta4```.
2. ```./configure CPPFLAGS=-fPIC```.
3. ```make```.
4. ```sudo make install```.

The crlibm library should be installed under ```/usr/local/lib``` directory.

### Install pyinterval package
```sudo pip install pyinterval```

### Compile and install cgkit
cgkit also requires custom installation. The source code can be downloaded from [here](http://liquidtelecom.dl.sourceforge.net/project/cgkit/cgkit/cgkit-2.0.0/cgkit-2.0.0-py3k.tar.gz). Similar to crlibm, use the following commands to install cgkit library from a terminal:

1. ```tar zxf cgkit-2.0.0-py3k.tar.gz;cd cgkit-2.0.0/supportlib```.
2. ```scons```.
3. ```cd ..```.
4. ```sudo python setup.py install```.

**Note**: For the time being, before start, you need to add the path of the installed packages to the list of system packages in pyride python environment.

For example if your numpy package is installed in: /usr/lib/python2.7/dist-packages/
you will need to do:

>>> import sys
>>> sys.path.append('/usr/lib/python2.7/dist-packages/')

in your pyride script console so that you can import numpy:

>>> import numpy

You should do this for all the above packages that you installed.

## A quick usage of S-PR2 under Magiks

To start working with S-PR2, you need to create a Skilled_PR2 object.

First add S-PR2 path to the system paths. In the pyride python console write:

```
>>> import sys
>>> sys.path.append(<your_magiks_path> + 'magiks/projects/s_pr2')
>>> import initialize
>>> from magiks.specific_geometries.pr2 import skilled_pr2 as spr
>>> obj = spr.Skilled\_PR2()
```

