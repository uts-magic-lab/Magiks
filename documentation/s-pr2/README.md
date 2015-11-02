#Skilled-PR2 (S-PR2)


## Introduction

S-PR2 is a new and powerful Python based software package that can be used as a kinematic control toolbox for PR2 service robot. 
The software exploits an analytic Inverse Kinematic solver for PR2 redundant arm (7 DOF)
with capability of optimum redundancy resolution. 
S-PR2 can be used by robot researchers and programmers to generate their desired trajectories in the operational space and execute
on a physical robot or in simulation. 
The package gives PR2 robot, the skills of tracking a user-defined trajectory like writing letters or drawing shapes on the board.

S-PR2 provides an efficient and reliable development environment providing optimal and feasible kinematic control 
functions for PR2 users. S-PR2 has adopted the lastest techniques of the Inverse Kinematic (IK) solution in recent years.

# What is PR2?

The PR2 is a wheel-based service robot produced by 
[Willow-Garage](https://www.willowgarage.com/)
with two 7-DOF arms, a tilting head and a sliding joint, 
adjusting the working height of the arms adding one degree of freedom
to the system. 

<img src="https://github.com/uts-magic-lab/Magiks/blob/master/documentation/s-pr2/figures/PR2_Robot_Willow_Garage_3.jpg" width="500">

The navigating platform can move on the floor and rotate around the $z$ axis providing three extra degrees of freedom. 
In free-base mode, eleven degrees of freedom kinematically influence the position or orientation of one gripper.

# Existing Package

The existing [IK solver](http://wiki.ros.org/pr2_kinematics/Tutorials/Tutorial%204) for the PR2, 
which is a part of Robot Operating System (ROS), is fast but 
lacks some important advanced features required for generating smooth and reliable trajectories. 
The existing package:

* Only projects a single pose to the joint-space and not a trajectory
* Does not provide all the available solutions (corresponding to a certain value of a redundant parameter) 
* Can not exploit the available redundancy in Degrees of Freedom (DOF) to minimise a desired cost function or to fulfil additional constraints
* Cannot give self motion trajectory connecting two points in the solution manifold
* Does not return a feasibility set for a redundant parameter to be chosen
* Is fully embedded within the ROS system and cannot be easily replaced or migrated to other systems
* Its algorithms are only provided in binary and so is not available to inspect or modify

# Why use S-PR2?

We introduced a comprehensive kinematic control software package for the PR2 robot, 
which is based on an analytic inverse kinematic solution and addresses all the aforementioned issues.
S-PR2 generates more effective robot motion plans, 
hence it can be used to produce morereliable robot behaviours. 
In S-PR2, the method used for redundancy resolution exploits the advantage of the existence of an analytic IK solution for 
improving an iterative redundancy optimization in terms of reliability, accuracy, and computational cost where 
closed-form IK equations are used to reduce dimensionality and complexity of the objective function. 

S-PR2 allows an end user to easily generate desired trajectories in task-space, project them to the joint-space while 
exploiting DOF redundancy, visualize the trajectories, and finally, 
execute them on a PR2 robot using simple Python function calls. 
The typical user does not need any specific knowledge of the underlying robot control system in order to use S-PR2. 
Expert robot programmers can, however, use the lower level function modules in S-PR2 to write customized IK control systems. 
Since, S-PR2 is written in pure Python scripting language, 
the kinematic software is agnostic to the underlying operation platform.

In the following sections, we will first describe the system structure and core functional modules in S-PR2, then 
each subcomponent will be briefly explained in terms of their methods and capabilities. 
Finally, an example of a practical application of the package is demonstrated.

## System Architechture

S-PR2 is implemented in a hierarchical structure in which
higher level classes inherit and extend functionalities of the lower level classes. 
The overall structure of S-PR2 is shown in Figure 1. 

<img src="https://github.com/uts-magic-lab/Magiks/blob/master/documentation/s-pr2/figures/spr2_structure_diagram2.png" width="500">

**Figure 1. Architecture diagram of S-PR2**

The core module is the PR2-Arm-Kinematics that contains all the required kinematic methods,
including an analytic arm IK solver and an optimal trajectory planner. 
This component inherits joint-space methods and properties from a generalized class called Manipulator_Configuration. 
Each module instance is constructed with an ensemble of setting parameters specific to the object. 
For example, Manipulator_Configuration_Settings contains general joint-space settings of a manipulator such as 
mechanical limits, types of joints and a set of weightings for each joint
that is used in defining a user-defined objective function.

## PR2 Arm Kinematic Module

The arm kinematic module is the main kinematic engine of S-PR2 with the lowest level of functionality. 
This module provides methods required for kinematic computations of the PR2 arm.

# Analytic Inverse Kinematic Solver 

The most important part of the module is an analytic IK solver that can give all the feasible configurations 
corresponding to a desired EE pose for an arbitrary value of the redundant parameter (up to maximum 8 solutions). 
The redundant parameter is by default selected as the first arm joint named Shoulder-Pan Joint. 
If the given task pose is outside the arm’s reachable region or 
an ill chosen value is used for the redundant parameter, 
no solution can be found by the solver. 
Since identifying the reason for failing to find a solution can be difficult, 
S-PR2 provides a feasibility Permission Set for the redundant parameter and 
searches within this set to find a value that 
leads to a solution with respect to the user’s desired criterion, 
i.e. the distance of the current configuration from the middle of the joint ranges. 
If the computed permission set is empty, then 
it is certain that the requested task pose lies outside of the robot workspace in which case no valid solution exists.

# Permission Set

In IK problems, the key challenge is to use redundancy to find the optimal joint values within their feasible ranges.
Each joint limitation makes part of the solution manifold invalid
and imposes a restriction on the feasible domain from which the redundant parameters can be chosen. 
For the PR2 that has only one redundant parameter in its manipulator, 
a Permission Set consisting of a union of intervals can be obtained by the intersection of feasibility permission sets 
imposed on the redundant parameter by the limitations of each joint. 
In other words, a permission set is a set from which the redundant parameter must be selected otherwise 
no solution in the feasible range can be found for the desired EE pose. 
S-PR2 can compute the permission set for a given desired EE pose using 
[Arithmetic of Intervals](https://en.wikipedia.org/wiki/Interval_arithmetic).

# Optimal Redundancy Resolution

A task in the cartesian space requires six kinematic constraints for the endeffector. 
If the number of free joints (DOF) is greater than the number of constraints required to fulfil a task, 
the robot is said to be in a Redundant state. 
In this case, one or some of the joints can be arbitrarily chosen as redundant parameters. 
It is also possible to consider a user-defined function of joints as a redundant parameter. 
Selecting an appropriate function for redundant parameters is called parametrization and it depends upon the geometry of the manipulator. 
Each redundant parameter adds an additional constraint to the system, so the total number of redundant
parameters denoted as Degree of Redundancy is computed
as $r = n − m$ where $n$ is the degree of freedom and m is the number of main kinematics constraints
($m = 6$ for a desired end effector position and orientation).

Some redundancy parametrizations have been proposed for specific geometries, like 
[Lee and Bejczy](http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=131621&url=http%3A%2F%2Fieeexplore.ieee.org%2Fxpls%2Fabs_all.jsp%3Farnumber%3D131621) 
who used some of the joints as redundant parameters in a 8 DOF arm or 
[Shimizu et.al.] (http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=4631505&url=http%3A%2F%2Fieeexplore.ieee.org%2Fxpls%2Fabs_all.jsp%3Farnumber%3D4631505), 
who used a user defined angle as the redundant parameter in a 7 DOF arm. 
The analytic solution of IK in redundant systems is to express joint positions as 
closed-form functions of redundant parameters, and 
the problem of Optimal Redundancy Resolution in analytic IK 
is to find a feasible value for the redundant parameter(s) to 
minimize a desired cost function subject to some additional constraints.
The method used for redundancy resolution in fixed-base mode is similar to Shimizu’s work but with a different parametrization. 
The analytic IK method used is also different as their formulations could not be used for the PR2 arm due to a slight difference in the [Denavit-Hartenberg (DH) parameters](http://wiki.ros.org/pr2_calibration_estimation) of the PR2 arms.

# Smooth Trajectory Projection

The arm kinematic module can project a trajectory from task-space into the joint-space. 
Trajectories are instances of module Trajectory consisting of segments 
established from a number of key points containing positions, velocities and accelerations. 
Each key point can apply constraint(s) to the trajectory. 
Trajectory points are interpolated from a fitted spline in form of a polynomial or Fourier series. 
The user can visualize both joint-space and task-space trajectories before executing them on a physical robot. 
Orientation-Trajectory is a module inherited from Trajectory supported by various methods for smooth orientation trajectory interpolation. Given the key orientations and corresponding angular velocities and accelerations, 
the interpolated orientation can be computed at any phase value using methods like [SLERP](https://en.wikipedia.org/wiki/Slerp)
with various conventions like quaternions or rotation matrix.
The module also supports polynomial interpolation using one of the 
[vectorial](http://link.springer.com/article/10.1023%2FA%3A1024265401576)
 non-redundant representations of three-dimensional rotation.


# Self Motion Generation

In a redundant manipulator, if the EE is fixed in some pose, 
the joints can still change in the solution manifold
to generate a motion that does not influence the end effector pose.
This is known as the *self Motion*. 
Different values of the joints in the solution manifold can be achieved by changing the redundant parameters. 
The arm kinematic module in S-PR2 is able to generate a smooth internal motion 
within the solution manifold, connecting two different IK solution points in the joint-space. 
This enables the robot to change itself into a better configuration while the end effector pose is fixed.

## PR2 Kinematics Module

The PR2 kinematic module is a comprehensive toolbox for all kinematic computations of the PR2. 
It exploits the arm local IK solver to compute the configuration in terms of 
eleven joints (degrees of freedom) given the desired end effector pose in the global coordinate system. 
This module also generates optimal trajectories for each of the eleven degrees
of freedom corresponding to a desired task.

# Redundancy Resolution in Free-Base Mode
 
In free-base mode, the redundancy resolution technique
used for the arm is extended to five redundant parameters for
the entire robot. In free-base mode, the position, orientation
and height of the robot trunk are computed so that a desired
cost function is minimized. This is another useful feature that
can not be found in any of the existing packages for PR2.
With the closed form arm IK equations, the redundancy opti-
mization problem is transferred from the eleven dimensional
joint-space into a five dimensional solution manifold spanned
by the redundant parameters. In this method, gradient of the
objective function with respect to redundant parameters are
computed to find an optimum direction for the change of
redundant parameters and hence the joint values.

## Application Example

Writing and drawing are complicated tasks that require
high accuracy and an understanding of the environment. We
used S-PR2 to create a system that enables PR2 to write
letters or draw any desired shape on the board: Writer-PR2.
Raw trajectories are given by an external device such as an
iPad tablet or loaded from a file containing some predefined
trajectories (for example English alphabets), then mapped
into the desired size, location, and orientation. They are
then projected into a feasible joint-space trajectory according
to the current robot state and configuration. The shapes or
letters are drawn/written by any of the arms with desired size
and speed.


