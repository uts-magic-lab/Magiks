'''   Header
File:           special_paths.py
Brief:    	    This module provides classes inherited from class Path() in module path.py
                The library contains a number of special paths in the jointspace or taskspace, like walking paths for 
                special humanoid robots like nao
                
Author:         Nima Ramezani Taghiabadi
                PhD Researcher
                Faculty of Engineering and Information Technology
                University of Technology Sydney (UTS)
                Broadway, Ultimo, NSW 2007, Australia
                Room No.: CB11.W7.101.07
                Mobile:   04 5027 4611
                Email(1): Nima.RamezaniTaghiabadi@student.uts.edu.au 
                Email(2): nima.ramezani@gmail.com
                Email(3): nima_ramezani@yahoo.com
                Email(4): ramezanitn@alum.sharif.edu
Version:	    1
Started:        17 July 2014
Last Revision:  17 July 2014

'''

import trajectory as trajlib
import special_trajectories as strajlib
import path as pathlib
import numpy as np
import copy, math

from packages.nima.mathematics import polynomials as pl
from packages.nima.mathematics import general as gen

zv = np.zeros(3)

class Colin_Graf_Path(pathlib.Path):
    '''
    A three dimensional walking taskspace path for any humanoid robot like NAO based on paper written by: Colin Graf et.al. 2009
    "A Robust Closed-Loop Gait for the Standard Platform League Humanoid" 
    Proceedings of the 4th Workshop on Humanoid Soccer Robots

    It is a path with dim 6. The first three are x,y,z coordinates of the position of right foot w.r.t. the COM in torso CS.
    the next three are x,y,z coordinates of the position of left foot w.r.t. the COM in torso CS.
    '''

    def __init__(self, xl = 0.22, yl=0.55, xm=0.22, ym=0.55, h=30.0, dx=0.0, dy=120.0, Ol = zv, Or = zv, Cs = zv, Ss = np.array([20.0,0.0,0.0])):
        super(Colin_Graf_Path, self).__init__()
        
        tj_1 = strajlib.Colin_Graf_Trajectory(first_half = True, xl=xl, yl=yl, xm=xm, ym=ym, h=h, dx=dx, dy=dy, Ol=Ol, Or=Or, Cs=Cs, Ss=Ss)
        self.trajectory.append(tj_1)
        
        '''
        I did not complete this path! because I faced a problem. At the end of a cycle (phi = 0.5), the y coordinate of COM (x in the paper) has a gap.
        this means there must be a double support phase in which the COM moves forward. But the x coordinate of COM, has maximum velicity 
        at phi = 0.5 and has to change rapidly to 0 at phi = 0.5 because COM is not moving in x direction in the double support phase.
        This gives a significant acceleration to the COM
        '''


def nima_walk_ts_path_for_rft_wrt_lft(foot_dist, step_length, step_height):
    '''
    Returns a taskspace path for walking for a humanoid robot or any biomechanical link-segment model
    foot_dist is the distance between the foot centers in x direction.
    step_length is the length of walking step

    Axis y: forward walking direction
    Axis z: upwards (from floor towards the sky)
    Axis x: Determined according to the right hand rule

    phase 0: moving from initial state to the start of walking when left foot is ahead of right foot as much as step_length/2
    phase 1: right foot moves forward and reaches step_length/2 ahead of the left foot
    phase 2: both feet are on the floor and their positions do not change. The COM comes forward.
    phase 3: left leg moves ahead and reaches step_length/2 ahead of the right leg
    phase 4: both legs are on the floor and their positions do not change. The COM comes forward. posture 4 = posture 0
    '''

    # Path for moving the right leg when left foot is the support foot
    L     = 0.5*step_length
    h     = step_height

    p1    = np.array([foot_dist, - L, 0.0])  # start of walk
    p2    = np.array([foot_dist, 0.0, h  ])  # middle of step 1
    p3    = np.array([foot_dist,   L, 0.0])  # end of step 1

    v0    = np.zeros(3)
    v2    = np.array([0.0, None  , 0.0])

    # t0 = trajlib.Polynomial_Trajectory()  # moving from initial state to the start of walking when left foot is ahead of right foot as much as step_length/2
    t1 = trajlib.Polynomial_Trajectory()  # both feet are on the floor and their positions do not change. The COM comes forward.
    t2 = trajlib.Polynomial_Trajectory()  # right foot moves forward and reaches step_length/2 ahead of the left foot
    t3 = trajlib.Polynomial_Trajectory()  # both feet are on the floor and their positions do not change. The COM comes forward.

    # t0.interpolate([0.0, 1.0]     , positions = [p0, p1]    , velocities = [v0, v0])
    t1.interpolate([0.0], positions = [p1])
    t2.interpolate([0.0, 0.5, 1.0], positions = [p1, p2, p3], velocities = [v0, v2, v0])
    t3.interpolate([0.0], positions = [p3])

    t1.phi_end = 0.15
    t3.phi_end = 0.15

    pth = pathlib.Path()
    pth.connect_trajectories([t1, t2, t3])


    return(pth)

def nima_walk_ts_path_for_lft_wrt_rft(foot_dist, step_length, step_height):
    '''
    Returns a taskspace path for walking for a humanoid robot or any biomechanical link-segment model
    foot_dist is the distance between the foot centers in x direction.
    step_length is the length of walking step

    Axis y: forward walking direction
    Axis z: upwards (from floor towards the sky)
    Axis x: Determined according to the right hand rule

    phase 0: moving from initial state to the start of walking when left foot is ahead of right foot as much as step_length/2
    phase 1: right foot moves forward and reaches step_length/2 ahead of the left foot
    phase 2: both feet are on the floor and their positions do not change. The COM comes forward.
    phase 3: left leg moves ahead and reaches step_length/2 ahead of the right leg
    phase 4: both legs are on the floor and their positions do not change. The COM comes forward. posture 4 = posture 0
    '''

    # Path for moving the right leg when left foot is the support foot
    L     = 0.5*step_length
    h     = step_height

    p1    = np.array([- foot_dist, - L, 0.0])  # start of walk
    p2    = np.array([- foot_dist, 0.0, h  ])  # middle of step 1
    p3    = np.array([- foot_dist,   L, 0.0])  # end of step 1

    v0    = np.zeros(3)
    v2    = np.array([0.0, None  , 0.0])

    # t0 = trajlib.Polynomial_Trajectory()  # moving from initial state to the start of walking when left foot is ahead of right foot as much as step_length/2
    t1 = trajlib.Polynomial_Trajectory()  # both feet are on the floor and their positions do not change. The COM comes forward.
    t2 = trajlib.Polynomial_Trajectory()  # right foot moves forward and reaches step_length/2 ahead of the left foot
    t3 = trajlib.Polynomial_Trajectory()  # both feet are on the floor and their positions do not change. The COM comes forward.

    # t0.interpolate([0.0, 1.0]     , positions = [p0, p1]    , velocities = [v0, v0])
    t1.interpolate([0.0], positions = [p1])
    t2.interpolate([0.0, 0.5, 1.0], positions = [p1, p2, p3], velocities = [v0, v2, v0])
    t3.interpolate([0.0], positions = [p3])

    t1.phi_end = 0.15
    t3.phi_end = 0.15

    pth = pathlib.Path()
    pth.connect_trajectories([t1, t2, t3])


    return(pth)

