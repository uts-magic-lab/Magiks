# Manipulator General Inverse Kinematic Solver (MAGIKS)

## Introduction

MAGIKS is a numerical iterative Inverse Kinematic solver which has been developed in Python. This software is presented as a developer's package containing various functions, classes and methods for the calculation of different kinematic properties of a general chained-linke
manipulaor with arbitrary geometry and degrees of freedom. Some of the main parameters include:

* Forward Kinematics including the current position and orientation of any reference point on the manipulator (e.g. arm endeffector, elbow, ...) or a linear combination of some reference points (e.g. center of mass, middle of two arm endeffectors, ...). 
* Values and gradients of any customized desired pose error function or displacement metrics
* The geometric, analytic Jacobian and Jacobians customized for any desired pose displecement metric
* Various Inverse Kinematic solutions based on different alogorithms and redundancy resolution techniqes 

This package is almost completed and ready to use, but the API documention page is still under construction.
We are continuously working on it to provide a complete API documentation with clarifying examples.

To install and use MAGIKS follow the installation instructions [here](http://uts-magic-lab/Magiks/blob/master/documentation/magiks/install_magiks.md). 

The [API documentation](http://uts-magic-lab.github.io/Magiks/index.html)
for MAGIKS is available for developers but is not completed yet and is under construction. 

[This example](http://uts-magic-lab/Magiks/blob/master/documentation/magiks/example_1.md) shows how to use MAGIKS for a simple pose projection task for 
(7 DOF) PA10 robot.


## A quick usage of S-PR2 under MAGIKS

[S-PR2](http://uts-magic-lab/Magiks/blob/master/documentation/s-pr2/README.md)
is a part of MAGIKS that is specifically designed for PR2 humanoid robot.
It supports an analytic-closed form kinematic control in addition to the numeric IK solver embedded in MAGIKS. 

As it is part of MAGIKS package, by installing MAGIKS, you will have S-PR2 installed. 

The [API documentation](http://uts-magic-lab.github.io/Magiks/namespacemagiks_1_1specific__geometries_1_1pr2.html)
for S-PR2 is available for developers and more than %80 complete.

To start quickly with S-PR2, see the following example:

* [How to write a simple script to control PR2 robot via S-PR2](http://uts-magic-lab/Magiks/blob/master/documentation/s-pr2/example_1.md)


