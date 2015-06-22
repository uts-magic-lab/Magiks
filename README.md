#Manipulator General Inverse Kinematic Solver (MAGIKS)

##Introduction

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

1. ```tar zxf crlibm-1.0beta3.tar.gz;cd crlibm-1.0beta4```.
2. ```./configure CPPFLAGS=-fPIC```.
3. ```make```.
4. ```sudo make install```.

The crlibm library should be installed under ```/usr/local/lib``` directory.

### Install pyinterval package
```sudo pip install pyinterval```

### Compile and install cgkit
cgkit also requires custom installation. The source code can be downloaded from [here](http://liquidtelecom.dl.sourceforge.net/project/cgkit/cgkit/cgkit-2.0.0/cgkit-2.0.0-py3k.tar.gz). Similar to crlibm, use the following commands to install cgkit library from a terminal:

1. ```tar zxf cgkit-2.0.0-py3k.tar.gz;cd cgkit-2.0.0-py3k\supportlib```.
2. ```scons```.
3. ```cd ..```.
4. ```sudo python setup.py install```.

**Note**: For the time being, before start, you need to add the path of the installed packages to the list of system packages in pyride python environment.

For example if your numpy package is installed in: /usr/lib/python2.7/dist-packages/
you will need to do:

```>>> import sys
'''>>> sys.path.append('/usr/lib/python2.7/dist-packages/')
in your pyride script console so that you can import numpy:
```>>> import numpy

You should do this for all the above packages that you installed.

## A quick usage of S-PR2 under Magiks

To start working with S-PR2, you need to create a Skilled_PR2 object.

First add S-PR2 path to the system paths. In the pyride python console write:

```>>> import sys```
```>>> sys.path.append(<your_magiks_path> + 'magiks/projects/s_pr2')```
```>>> import initialize```
```>>> from magiks.specific_geometries.pr2 import skilled_pr2 as spr```

```>>> obj = spr.Skilled\_PR2()```

