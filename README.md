# Manipulator General Inverse Kinematic Solver (MAGIKS)

## Introduction
The **Manipulator General Inverse Kinematic Solver (_MAGIKS_)**
is a numerical iterative Inverse Kinematic solver developed in Python 
which is presented as a developer's package containing various functions, classes and methods 
for the calculation of various kinematic properties of a general chained-link manipulaor 
with arbitrary geometry and degrees of freedom. 

*MAGIKS* is developed by *Nima Ramezani* in the Magic-Lab in the Faculty of Engineering and Information Technology (FEIT), University of Technology Sydney (UTS).
Arguably, *MAGIKS* is the most comprehensive Jacobian-based general IK solver package proposed up to now, 
in terms of covering various algorithms and techniques. 
The special and unique features of *MAGIKS* made it applicable to a great number of general chain-link models.
This generality expands MAGIK's domain of application to areas other than robotics including Biomechanics and Computer Animation.
These features has made MAGIKS suitable for both researchers and end-users seeking a relaible tool-box for manipulation control.

Some of the main features of *MAGIKS* include:

* Computes Forward and Inverse kinematics of a robotic manipulator with a general geometry and arbitrary degree of freedom
* Computes joint correction vector, supporting various numeric algorithms and redundancy resolution techniques
* Supports Damped Least Squares (DLS) technique for singularity avoidance with Adaptive Damping Factor (DLS-ADF)
  to handle singularity and local minimum problems.  
* Handles Joint limits using the novel and efficient method named as **Virtual Joint-space Mapping (VJM)**
* Computes Geometric and Analytic Jacobians for given endeffector(s) in a general robot manipulator
* Supports multiple end-effectors in terms of multiple reference points for 
  position and multiple frames for orientation
* Supports linear combination of reference points as end-effector (Using this feature, one can define the Center of Mass as end-effector)
* Computes various pose metrics with associated Error Jacobians and 
  supports utilization of those metrics in the IK solution algorithm.
* Gives smooth, optimal and feasible projected joint-space trajectory for a given task-space target pose or pose trajectory
* Customized and faster analytic inverse kinematics for specific robot manipulator models (PR2)

This package is almost completed and ready to use, but the API documention page is still under construction.
We are continuously working on it to provide a complete API documentation with clarified examples.

## Installation and Use

To install and use *MAGIKS*, follow the installation instructions [here](https://github.com/uts-magic-lab/Magiks/blob/master/documentation/magiks/install_magiks.md). 

The [API documentation](http://uts-magic-lab.github.io/Magiks/index.html)
for *MAGIKS* is available for developers but is not completed yet and is under construction. 

[This example](https://github.com/uts-magic-lab/Magiks/blob/master/documentation/magiks/example_1.md) shows how to use MAGIKS for a simple pose projection task for 
(7 DOF) PA10 robot.

## A quick usage of S-PR2 under MAGIKS

[S-PR2](https://github.com/uts-magic-lab/Magiks/blob/master/documentation/s-pr2/README.md)
is a part of MAGIKS that is specifically designed for PR2 humanoid robot.
It supports an analytic-closed form kinematic control in addition to the numeric IK solver embedded in *MAGIKS*. 

As it is part of *MAGIKS* package, by installing MAGIKS, you will have *S-PR2* installed. 

The [API documentation](http://uts-magic-lab.github.io/Magiks/namespacemagiks_1_1specific__geometries_1_1pr2.html)
for S-PR2 is available for developers.

To start quickly with *S-PR2*, see the following example:

* [How to write a simple script to control PR2 robot via S-PR2](https://github.com/uts-magic-lab/Magiks/blob/master/documentation/s-pr2/example_1.md)


