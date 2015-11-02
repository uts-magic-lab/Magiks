#Manipulator General Inverse Kinematic Solver (MAGIKS)

##Introduction

MAGIKS is A numerical iterative Inverse Kinematic solver which has been developed in Python. This software is presented as a developer's package containing various functions, classes and methods for the calculation of different kinematic properties of a general chained-linke
manipulaor with arbitrary geometry and degrees of freedom. Some of the main parameters include:

* Forward Kinematics including the current position and orientation of any reference point on the manipulator (e.g. arm endeffector, elbow, ...) or a linear combination of some reference points (e.g. center of mass, middle of two arm endeffectors, ...). 
* Values and gradients of any customized desired pose error function or displacement metrics
* The geometric, analytic Jacobian and Jacobians customized for any desired pose displecement metric
* Various Inverse Kinematic solutions based on different alogorithms and redundancy resolution techniqes 

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

```
tar zxf crlibm-1.0beta4.tar.gz
cd crlibm-1.0beta4
./configure CPPFLAGS=-fPIC
make
sudo make install
```

The crlibm library should be installed under this directory:
```
/usr/local/lib
``` 

### Install pyinterval package
```
sudo pip install pyinterval
```

### Compile and install cgkit
cgkit also requires custom installation. The source code can be downloaded from [here](http://liquidtelecom.dl.sourceforge.net/project/cgkit/cgkit/cgkit-2.0.0/cgkit-2.0.0-py3k.tar.gz). Similar to crlibm, use the following commands to install cgkit library from a terminal:

```
tar zxf cgkit-2.0.0-py3k.tar.gz
cd cgkit-2.0.0/supportlib
scons
cd ..
sudo python setup.py install
```

**Note**: For the time being, before start, you need to add the path of the installed packages to the list of system packages in pyride python environment.

For example if your numpy package is installed in: /usr/lib/python2.7/dist-packages/
you will need to do:

```
import sys
sys.path.append('/usr/lib/python2.7/dist-packages/')
```
in your pyride script console so that you can import numpy:
```
import numpy
```
You should do this for all the above packages that you installed.

## A quick usage of S-PR2 under Magiks

S-PR2 is a part of MAGIKS that is specifically designed for PR2 humanoid robot.
You can find more information about S-PR2 [here](http://uts-magic-lab.github.io/Magiks/namespacemagiks_1_1specific__geometries_1_1pr2_1_1skilled__pr2.html).

To start working with S-PR2, you need to create a Skilled_PR2 object.

First add S-PR2 path to the system paths. In the pyride python console write:

```
import sys
sys.path.append(<your_magiks_path> + 'magiks/projects/s_pr2')
import initialize
from magiks.specific_geometries.pr2 import skilled_pr2 as spr
obj = spr.Skilled\_PR2()
```

