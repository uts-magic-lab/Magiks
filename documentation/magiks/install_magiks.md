# Installing MAGIKS

MAGIKS is a set of scripts for python developers. It can be easily installed by checking out the spurce code from the github repository. Make a directory anywhere in which you want to copy the scripts. This will be the root directory of MAGIKS. Then go to that directory and type:
 
```
git clone https://github.com/uts-magic-lab/Magiks.git
```

Before using MAGIKS, you need to install some dependant packages:

##Prerequisites

Currently, Magiks is dependant on the following Python library packages

* crlibm (required by pyinterval).
* pyinterval.
* matplotlib.
* numpy.
* sympy.
* cgkit.
* PyRide (If you want to use pyride ROS interface for S-PR2)

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

### Install PyRide package

PyRide is required if you want to control the physical PR2 robot or PR2 in simulation via S-PR2 package under MAGIKS. 
It is a ROS interface by which you can communicate with ROS and can synchronize the configuration of the PR2 object with the real robot. 
You can also run a trajectory or implement a real-time trajectory control on PR2.
You will need PyRide if you work with any of these packages:

* [```PyRide_Synchronizer```](http://uts-magic-lab.github.io/Magiks/namespacemagiks_1_1specific__geometries_1_1pr2_1_1pyride__synchronizer.html)
* [```pyride_interpreter```](http://uts-magic-lab.github.io/Magiks/namespacemagiks_1_1specific__geometries_1_1pr2_1_1pyride__interpreter.html)
* [```Skilled_PR2```](http://uts-magic-lab.github.io/Magiks/namespacemagiks_1_1specific__geometries_1_1pr2_1_1skilled__pr2.html)
* [```Writer-PR2()```](http://uts-magic-lab.github.io/Magiks/namespacemagiks_1_1specific__geometries_1_1pr2_1_1writer__pr2.html)

To install pyride, follow the instructions from [here](https://github.com/uts-magic-lab/pyride_pr2/blob/master/README.md). 

