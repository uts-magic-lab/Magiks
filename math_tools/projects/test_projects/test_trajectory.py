'''   Header
@file:          test_trajectory.py
@brief:    	    This module tests the performance of trajectory.py
@author:        Nima Ramezani; UTS
@start date:    June 2014
@version:	    5.0
Last Revision:  11 June 2015

Major changes from previous version:
    1- Made compatible with trajectory version 6
    2- Some comments added
'''

import __init__
__init__.set_file_path( False )

import matplotlib.pyplot as plt
import scipy.io   # Required for reading Matlab workspace file
import numpy as np
import math
    
from math_tools.geometry import trajectory as tj, rotation as rot, geometry as geo

## This function shows how you can create a polynomial path, add points with constraints on velocity and acceleration, 
#  interpolate it, plot it and see the key points and note how the constraints are fulfilled.
def test_path_polynomial():
    # First create an blank instance of class Path_Polynomial:
    p = tj.Path_Polynomial(dimension = 3)

    # Now let's add some points: 

    # We add the first point at phase \f$ \phi = 0.0 \f$, at zero with zero velocity.
    # There is no constraint for acceleration:
    p.add_point(0.0, pos = np.zeros(3), vel = np.zeros(3))
    
    # The second point is added at phase \f$ \phi = 1.0 \f$, with a velocity constraint.
    # Again, there is no constraint for acceleration:
    p.add_point(1.0, pos = np.array([1.0, 2.0, -1.0]), vel = np.array([-1.0, -7.0, 2.0]))

    # The third point is added at phase \f$ \phi = 1.5 \f$, with an acceleration constraint.
    # There is no constraint for velocity:
    p.add_point(1.5, pos = np.array([2.0, 1.0, -2.0]), acc = np.array([1.0, -1.0, 2.0]))

    # A path has a capacity of 3 points by default. To add more points you need to increase the capacity.
    # We change the capacity so that the path can accommodate 5 points:
    p.capacity = 5
    # The fourth point is added at phase \f$ \phi = 2.0 \f$, with a velocity and acceleration constraint.
    # There is no constraint for position:
    p.add_point(2.0, pos = None, vel = np.zeros(3), acc = np.array([1.0, -1.0, 2.0]))

    # Finally, the fifth point is added at phase \f$ \phi = 3.0 \f$, with position, velocity and acceleration constraints:
    p.add_point(3.0, pos = np.zeros(3), vel = np.zeros(3), acc = np.zeros(3))
    p.interpolate()
    
    # To see the position, vlocity and acceleration values at a phase you need to set the phase and then read the values:
    p.set_phi(3.0)

    print "POSITION     : ", p.current_position
    print "VELOCITY     : ", p.current_velocity
    print "ACCELERATION : ", p.current_acceleration

    # to plot the path position, use function plot. For example to see the position in x axis:
    p.plot(axis = 0, show_points = True)

    # To plot the velocity, use function plot with argument wtp set to 'velocity'.
    # For example to see the velocity in z axis:
    p.plot(axis = 2, wtp = 'velocity', show_points = True)

    # Similarly, to plot the acceleration, use function plot with argument wtp set to 'acceleration'.
    # For example to see the acceleration in y axis:
    p.plot(axis = 1, wtp = 'acceleration', show_points = True)

def test_trajectory_polynomial():
    p = tj.Trajectory_Polynomial(dimension = 3, capacity = 2)
    # Add some points and constraints:
    p.add_point(0.0, pos = np.array([1.0, None, 3.0]))
    p.add_point(1.0, pos = None, vel = np.array([1.0, -1.0, 2.0]))
    p.add_point(3.0, pos = np.array([3.0,  0.0, 4.0]))
    p.add_point(4.0, pos = np.array([2.0, -1.0, 3.0]),  vel = np.array([1.0, 1.0, 1.0]))
    p.add_point(5.0, pos = np.array([0.0, -2.0, 1.0]),  vel = np.array([0.0, -1.0, -2.0]))
    p.add_point(6.0, pos = np.array([-1.0, -1.0, 2.0]), vel = np.array([-1.0, 0.0, 3.0]))
    p.add_point(7.0, pos = np.array([-2.0, 1.0, 5.0]),  vel = np.array([-2.0, 2.0, 1.0]))
    p.add_point(8.0, pos = np.array([0.0, 0.0, 0.0]),   vel = np.array([0.0, 0.0, 0.0]), acc = np.array([0.0, 0.0, 0.0]))
    # interpolate:
    p.interpolate()

    # This is the plot for the position of x axis:
    p.plot(axis = 0, show_points = True)

    # You can see that the velocities are not consistent. The velocity function is not continuous that the border of segments:
    p.plot(axis = 0, wtp = 'velocity', show_points = True)

    # To make the velocities consistent at the border of segments, use:
    p.consistent_velocities()

    # Now look at the plot for the position of x axis:
    p.plot(axis = 0, show_points = True)
    # And the velocity of x axis:
    p.plot(axis = 0, wtp = 'velocity', show_points = True)
    # You can see that the function is continuous, but it is broken at the border of segments. 
    # The reason is that accelerations are not consistent.
    # Similarly, to make accelerations consistent, use:
    p.consistent_accelerations()
    # and now see the velocity plot for x axis: 
    p.plot(axis = 0, wtp = 'velocity', show_points = True)
    # and see the influence on the position plot: 
    p.plot(axis = 0, show_points = True)

def test_orientation_path_polynomial():
    # First create an blank instance of class Orientation_Path_Polynomial:
    ot = tj.Orientation_Path_Polynomial()
    # Let's add some orientations as key points:
    ot.add_point(phi = 0.0, ori = geo.Orientation_3D(rot.point_forward_orientation, representation = 'matrix'))
    ot.add_point(phi = 1.0, ori = geo.Orientation_3D(rot.point_upward_orientation,  representation = 'matrix'))
    # Interpolate:
    ot.interpolate()
    # And see the plot
    ot.plot()

def test_add_vector():
    p0 = np.zeros(3)
    p = tj.Polynomial_Trajectory(dimension = 3)
    p.add_point(0.0, p0)
    p.add_vector(1.0, np.array([ 1.0 ,  0.0, 0.0]))
    p.new_segment()
    p.add_vector(1.0, np.array([ 0.0 ,  1.0, 0.0]))
    p.new_segment()
    p.add_vector(1.0, np.array([-1.0 ,  0.0, 0.0]))
    p.new_segment()
    p.add_vector(1.0, np.array([ 0.0 , -1.0, 0.0]))
    
    p.scatter_plot()
    p.consistent_velocities()
    p.scatter_plot()
        
def test_circle():
    s = 1.0/math.sqrt(3.0)
    c = stj.Circle_3D(normal_vector = np.array([s, s, s]))
    print c.R
    c.scatter_plot()    

def test_read_trajectory():
    '''
    This test uses function add_position to add points read from a given trajectory in a matlab workspace .mat file
    '''
    data_file = "pr2_trajectory_data.mat"
    workspace = scipy.io.loadmat(data_file)
    p       = workspace['p']         
    q       = workspace['q']
    N       = len(p)
    assert N == len(q)

    ## Create a position and orientation trajectory from given data:

    pt = tj.Trajectory(capacity = 5)

    pt.vel_max = 0.1
    pt.acc_max = 0.1
    pt.accuracy_level = 1

    t  = 0.0
    dt = 0.005

    for i in range(N):
        pos = p[i, :]
        pt.add_position(t, pos = pos)
        t   = t + dt

    #pt.clear_accelerations()
    #pt.clear_velocities()
    #pt.interpolate()
    pt.plot(axis = 0, n = 10000, show_points = True)
    #pt.plot(axis = 1, n = 10000, show_points = True)
    #pt.plot(axis = 2, n = 10000, show_points = True)
    pt.plot(axis = 0, n = 10000, wtp = 'velocity', show_points = True)
    #pt.plot(axis = 1, n = 10000, wtp = 'velocity', show_points = True)
    #pt.plot(axis = 2, n = 10000, wtp = 'velocity', show_points = True)
    pt.plot(axis = 0, n = 10000, wtp = 'acceleration', show_points = True)
    #pt.plot(axis = 1, n = 10000, wtp = 'acceleration', show_points = True)
    #pt.plot(axis = 2, n = 10000, wtp = 'acceleration', show_points = True)

if __name__ == "__main__" :
    # test_1()
    # test_2()
    # test_3()
    # test_colin_graf()
    # test_path()

    # test_trajectory_polynomial()
    # test_orientation_path_polynomial()
    test_read_trajectory()
