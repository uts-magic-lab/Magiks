'''   Header
@file:          trajectory_shapes.py
@brief:    	    This module contains a number of trajectories with various geometric shapes.
                These shape trajectories can be projected to the jointspace of any robot
                and be used to control the robot endeffector drawing any desired shape
@author:        Nima Ramezani Taghiabadi
                PhD Researcher
                Faculty of Engineering and Information Technology
                University of Technology Sydney (UTS)
                Broadway, Ultimo, NSW 2007, Australia
                Room No.: CB10.03.512
                Phone:    02 9514 4621
                Mobile:   04 5027 4611
                Email(1): Nima.RamezaniTaghiabadi@student.uts.edu.au 
                Email(2): nima.ramezani@gmail.com
                Email(3): nima_ramezani@yahoo.com
                Email(4): ramezanitn@alum.sharif.edu
@version:	    2

Last Revision:  01 October 2014

Changes from Version 1:
    1- direction changed to a Rotation Matrix where width, height and normal directions can be specified
    2- shapes can be added to the trajectory. by this you can connect shapes and make one trajectory containing multiple shapes
    3- functions get_sign() and get_dir() are removed

'''

import numpy as np
import copy, math, rotation as rot, trajectory as trajlib

from math_tools import general_math as gen



def line(initial_pos, final_pos, initial_vel = np.zeros(3), final_vel = np.zeros(3)):
    dim = len(initial_pos)
    assert dim == len(final_pos)
    assert dim == len(initial_vel)
    assert dim == len(final_vel)

    tr = trajlib.Trajectory_Polynomial(dimension = dim)
    tr.add_point(0.0, initial_pos, initial_vel)
    tr.add_point(1.0, final_pos, final_vel)
    tr.interpolate()
    return tr

def square_3D(length = 0.1, initial_pos = np.zeros(3), direction = "+z+x", smooth = False):
    
    traj = trajlib.Trajectory_Polynomial(dimension = 3)
    traj.add_point(0.0, initial_pos)

    traj.add_vector(1.0,   length*dir_vect(direction[0:2]))
    traj.new_segment()
    traj.add_vector(1.0,   length*dir_vect(direction[2:4]))
    traj.new_segment()
    traj.add_vector(1.0, - length*dir_vect(direction[0:2]))
    traj.new_segment()
    traj.add_vector(1.0, - length*dir_vect(direction[2:4]))

    if smooth:
        p.consistent_velocities()
    
    return traj

def decode_arguments(traj, direction, static_ends):
    if static_ends:
        vel = np.zeros(3)
    else:
        vel = np.array([None, None, None])
    
    if traj == None:
        p = trajlib.Trajectory_Polynomial(dimension = 3)
        p.add_point(0.0, np.zeros(3), vel)
    else:
        lsi = len(traj.segment) - 1

        if (lsi < 0) or (len(traj.segment[lsi].point) > 1):
            traj.new_segment()

        p = copy.copy(traj)

    if direction == None:
        direction = rot.point_forward_orientation

    w = direction[:,0]
    h = direction[:,1]
    n = direction[:,2]

    return (p, w, h, n, vel)

def arc(traj = None, center_dw = 0.1, center_dh = 0.0, direction = None, angle = 360.0, d_theta = 10.0, clockwise = True, static_ends = True, in_degrees = True):
    '''
    adds a set of points to the given trajectory establishing an arc in a plane parallel to the yz plane starting from initial_pos, 
    The points follow an arc with given angle around the center
    input trj must be a Trajectory_Polynomial
    if trj is not given, a new trajectory will be created
    '''
    if in_degrees:
        angle = angle*math.pi/180.0
        d_theta = d_theta*math.pi/180.0

    (trj, w, h, n, v) = decode_arguments(traj, direction, static_ends)

    if clockwise:
        sgn = 1
    else:
        sgn = - 1
    
    center = center_dw*w + center_dh*h

    nseg = len(trj.segment)
    npnt = len(trj.segment[nseg-1].point)

    initial_pos = np.copy(trj.segment[nseg-1].point[npnt - 1].pos)
    initial_phi = trj.phi_end

    r    = math.sqrt(center_dw**2 + center_dh**2)
    t0   = math.acos(- center_dw/r)
    t    = 0.0
    stay = True
    pos  = np.zeros(3)
    while stay:
        t = t + d_theta  
        if t > angle:  
            t = angle

        pos    = initial_pos + center + r*math.cos(sgn*t + t0)*w + r*math.sin(sgn*t + t0)*h
        vel    = - r*sgn*math.sin(sgn*t + t0)*w + r*sgn*math.cos(sgn*t + t0)*h
        # trj.new_segment()
        trj.add_point(phi = initial_phi + t, pos = np.copy(pos), vel = vel)

        if gen.equal(t, angle):
            stay = False

    lsi = len(trj.segment) - 1
    lpi = len(trj.segment[lsi].point) - 1
    trj.segment[lsi].point[lpi].vel = v
    
    return trj


def M(traj = None, width = 0.08, height = 0.1, direction = None, static_ends = True, adjust = True, smooth = True, complete = False):

    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)
    if complete:
        traj.add_vector(0.2, 0.2*height*n, vel)
        traj.new_segment()

    traj.add_vector(1.0, height*h, vel)
    traj.new_segment()
    traj.add_vector(1.0, 0.5*width*w - 0.5*height*h, vel)
    traj.new_segment()
    traj.add_vector(1.0, 0.5*width*w + 0.5*height*h, vel)
    traj.new_segment()
    traj.add_vector(1.0, - height*h, vel)
    if complete:
        traj.new_segment()
        traj.add_vector(1.2, 0.2*width*w - height*h - 0.2*height*n, vel)

    if adjust:
        traj.adjust_phase_by_distance(gain = 100.0)

    if smooth:
        traj.consistent_velocities()

    return traj

def N(traj = None, width = 0.08, height = 0.1, direction = None, static_ends = True, adjust = True, smooth = True, complete = False):

    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)
    w = -w
    if complete:
        traj.add_vector(0.2, 0.2*height*n, vel)
        traj.new_segment()
    traj.add_vector(1.0, height*h, vel)
    traj.new_segment()
    traj.add_vector(1.0, width*w - height*h, vel)
    traj.new_segment()
    traj.add_vector(1.0, height*h, vel)
    if complete:
        traj.new_segment()
        traj.add_vector(0.2, - 0.2*height*n, vel)
        traj.new_segment()
        traj.add_vector(1.0, 0.2*width*w - height*h, vel)
    if adjust:
        traj.adjust_phase_by_distance(gain = 100.0)
    if smooth:
        traj.consistent_velocities()
    return traj

def V(traj = None, width = 0.08, height = 0.1, direction = None, static_ends = True, adjust = True, smooth = True, complete = False):

    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)
    w = - w
    if complete:
        traj.add_vector(1.2, height*h + 0.2*height*n, vel)
        traj.new_segment()
    traj.add_vector(1.0, 0.5*width*w - height*h, vel)
    traj.new_segment()
    traj.add_vector(1.0, 0.5*width*w + height*h, vel)
    if complete:
        traj.new_segment()
        traj.add_vector(1.2, - height*h + 0.2*width*w - 0.2*height*n, vel)
    if adjust:
        traj.adjust_phase_by_distance(gain = 100.0)
    if smooth:
        traj.consistent_velocities()
    return traj

def W(traj = None, width = 0.08, height = 0.1, direction = None, static_ends = True, adjust = True, smooth = True, complete = False):

    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)
    w = - w
    if complete:
        traj.add_vector(1.2, height*h + 0.2*height*n, vel)
        traj.new_segment()
    traj.add_vector(1.0, 0.15*width*w - height*h, vel)
    traj.new_segment()
    traj.add_vector(1.0, 0.35*width*w + 0.6*height*h, vel)
    traj.new_segment()
    traj.add_vector(1.0, 0.35*width*w - 0.6*height*h, vel)
    traj.new_segment()
    traj.add_vector(1.0, 0.15*width*w + height*h, vel)
    if complete:
        traj.new_segment()
        traj.add_vector(1.2, - height*h - 0.2*height*n + 0.2*width*w, vel)
    if adjust:
        traj.adjust_phase_by_distance(gain = 100.0)
    if smooth:
        traj.consistent_velocities()
    return traj

def U(traj = None, width = 0.05, height = 0.1, direction = None, static_ends = True, adjust = True, smooth = True, complete = False):
    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)
    if complete:
        traj.add_vector(1.2, height*h + 0.2*height*n, vel)
        traj.new_segment()
    traj.add_vector(1.0,  - h*(height- width/2))
    traj = arc(traj, center_dw = - width/2, center_dh = 0.0, angle = 180.0, clockwise = False, static_ends = False, direction=direction)
    traj.new_segment()
    traj.add_vector(1.0, h*(height- width/2), vel)
    if complete:
        traj.new_segment()
        traj.add_vector(1.2, 0.2*width*w - height*h - 0.2*height*n, vel)
    if smooth:
        traj.consistent_velocities()
    return traj

def O(traj = None, width = 0.05, height = 0.1, depth = 0.02, direction = None, static_ends = True, adjust = True, smooth = True, complete = False):
    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)
    if complete:
        traj.add_vector(1.0, (height-width/2)*h, vel)
        traj.new_segment()
        traj.add_vector(0.2, depth*n, vel)
        traj.new_segment()
    traj.add_vector(1.0,  - h*(height- width))
    traj = arc(traj, center_dw = - width/2, center_dh = 0.0, angle = 180.0, clockwise = False, static_ends = False, direction=direction)
    traj.new_segment()
    traj.add_vector(1.0, h*(height- width), vel)
    traj = arc(traj, center_dw = width/2, center_dh = 0.0, angle = 180.0, clockwise = False, static_ends = False, direction=direction)
    if complete:
        traj.new_segment()
        traj.add_vector(0.2, - depth*n, vel)
        traj.new_segment()
        traj.add_vector(1.0, - 1.2*width*w - (height-width/2)*h, vel)
    if smooth:
        traj.consistent_velocities()
    return traj

def G(traj = None, width = 0.05, height = 0.1, depth = 0.02, direction = None, static_ends = True, adjust = True, smooth = True, complete = False):
    assert height > width, "Error from trajectory_shapes.G(): Height must be greater than width"
    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)
    if complete:
        traj.add_vector(1.0, h*(height- width/2) - w*width, vel)
        traj.new_segment()
        traj.add_vector(0.2, depth*n, vel)
        traj.new_segment()
    traj = arc(traj, center_dw = width/2, center_dh = 0.0, angle = 180.0, clockwise = False, static_ends = False, direction=direction)
    traj.new_segment()
    traj.add_vector(1.0,  - h*(height- width))
    traj = arc(traj, center_dw = - width/2, center_dh = 0.0, angle = 180.0, clockwise = False, static_ends = False, direction=direction)
    traj.add_vector(1.0,  0.5*h*(height- width))
    traj.new_segment()
    traj.add_vector(1.0,  0.25*w*width)
    if complete:
        traj.new_segment()
        traj.add_vector(0.2, - depth*n, vel)
        traj.new_segment()
        traj.add_vector(1.0, - 0.5*h*height - 0.45*w*width, vel)
    if smooth:
        traj.consistent_velocities()
    return traj

def C(traj = None, width = 0.05, height = 0.1, depth = 0.02, direction = None, static_ends = True, adjust = True, smooth = True, complete = False):
    assert height > width, "Error from trajectory_shapes.C(): Height must be greater than width"
    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)
    
    if complete:
        traj.add_vector(1.0, h*(height- width/2) - w*width, vel)
        traj.new_segment()
        traj.add_vector(0.2, depth*n, vel)
        traj.new_segment()
    traj = arc(traj, center_dw = width/2, center_dh = 0.0, angle = 180.0, clockwise = False, static_ends = False, direction=direction)
    traj.new_segment()
    traj.add_vector(1.0,  - h*(height- width))
    traj = arc(traj, center_dw = - width/2, center_dh = 0.0, angle = 180.0, clockwise = False, static_ends = False, direction=direction)
    if complete:
        traj.new_segment()
        traj.add_vector(0.2, - depth*n, vel)
        traj.new_segment()
        traj.add_vector(1.0, - h*width/2 - 0.2*w*width, vel)
    if smooth:
        traj.consistent_velocities()
    return traj

def J(traj = None, width = 0.05, height = 0.1, depth = 0.02, direction = None, static_ends = True, adjust = True, smooth = True, complete = False):
    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)
    if complete:
        traj.add_vector(1.0, h*(height- width/2) - w*width, vel)
        traj.add_vector(0.2, depth*n, vel)
        traj.new_segment()
    traj = arc(traj, center_dw = - width/2, center_dh = 0.0, angle = 180.0, clockwise = False, static_ends = False, direction=direction)
    traj.new_segment()
    traj.add_vector(1.0, h*(height- width/2), vel)
    if smooth:
        traj.consistent_velocities()
    return traj

def P(traj = None, width = 0.08, height = 0.1, direction = None, static_ends = True, adjust = True, smooth = True, complete = False):
    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)
    w = -w
    if complete:
        traj.add_vector(0.2, 0.2*height*n, vel)
        traj.new_segment()

    traj.add_vector(1.0, height*h, vel)
    traj = arc(traj, center_dh = -height/4, center_dw = 0.0, angle = 180.0, clockwise = True, static_ends = static_ends, direction=direction)

    if complete:
        traj.new_segment()
        traj.add_vector(1.2, - 0.2*height*n + 1.2*width*w - 0.5*height*h, vel)
    if adjust:
        traj.adjust_phase_by_distance(gain = 100.0)
    if smooth:
        traj.consistent_velocities()
    return traj

def B(traj = None, width = 0.08, height = 0.1, direction = None, static_ends = True, adjust = True, smooth = True, complete = False):
    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)

    if complete:
        traj.add_vector(0.2, 0.2*height*n, vel)
        traj.new_segment()

    traj.add_vector(1.0, height*h, vel)
    traj = arc(traj, center_dh = - height/4, center_dw = 0.0, angle = 180.0, clockwise = True, static_ends = static_ends, direction=direction)
    traj = arc(traj, center_dh = - height/4, center_dw = 0.0, angle = 180.0, clockwise = True, static_ends = static_ends, direction=direction)

    if complete:
        traj.new_segment()
        traj.add_vector(1.2, -1.2*width*w - 0.2*height*n, vel)

    if adjust:
        traj.adjust_phase_by_distance(gain = 100.0)

    if smooth:
        traj.consistent_velocities()
    return traj

def D_old(traj = None, width = 0.08, height = 0.1, direction = None, static_ends = True, adjust = True, smooth = True, complete = False):
    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)

    if complete:
        traj.add_vector(0.2, 0.2*height*n, vel)
        traj.new_segment()

    traj.add_vector(1.0, height*h, vel)
    traj = arc(traj, center_dh = - height/2, center_dw = 0.0, angle = 180.0, clockwise = True, static_ends = static_ends, direction=direction)

    if complete:
        traj.new_segment()
        traj.add_vector(1.2, -1.2*width*w - 0.2*height*n, vel)

    if adjust:
        traj.adjust_phase_by_distance(gain = 100.0)

    if smooth:
        traj.consistent_velocities()
    return traj

def D(traj = None, width = 0.05, height = 0.1, depth = 0.02, direction = None, static_ends = True, adjust = False, smooth = True, complete = False):
    assert height > width, "Error from D(): height must be greater than width"
    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)

    assert height >= 2*width, "Error from D(): height must be greater than or equal twice width"
    if complete:
        traj.add_vector(0.2, depth*n, vel)
        traj.new_segment()

    traj.add_vector(1.0, height*h, vel)
    traj = arc(traj, center_dh = - width, center_dw = 0.0, angle = 90.0, clockwise = True, static_ends = static_ends, direction=direction)
    traj.new_segment()
    traj.add_vector(1.0, - (height- 2*width)*h, vel)
    traj = arc(traj, center_dh = 0, center_dw = width, angle = 90.0, clockwise = True, static_ends = static_ends, direction=direction)

    if complete:
        traj.new_segment()
        traj.add_vector(1.2, -1.2*width*w - depth*n, vel)

    if adjust:
        traj.adjust_phase_by_distance(gain = 100.0)

    if smooth:
        traj.consistent_velocities()
    return traj

def C_old(traj = None, height = 0.1, direction = None, static_ends = True, adjust = True, smooth = True, complete = False):
    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)

    if complete:
        print "Not Tested!"
        traj.add_vector(1.0, (2+math.sqrt(3))*height*h/4.0 - 0.75*height*w, vel)
        traj.new_segment()
        traj.add_vector(0.2, 0.2*height*n, vel)
        traj.new_segment()

    traj = arc(traj, center_dh = - height*math.sqrt(3)/4.0, center_dw = height/4.0, angle = 250.0, clockwise = False, static_ends = static_ends, direction=direction)

    if complete:
        traj.new_segment()
        traj.add_vector(0.2, - 0.2*height*n, vel)
        traj.new_segment()
        traj.add_vector(1.0, -0.2*height*h -0.2*width*w, vel)

    if adjust:
        traj.adjust_phase_by_distance(gain = 100.0)

    if smooth:
        traj.consistent_velocities()
    return traj

def S(traj = None, height = 0.1, depth = 0.02, direction = None, static_ends = True, adjust = False, smooth = True, complete = False):
    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)
    w     = - w
    cth   = math.cos(math.pi/3)
    sth   = math.sin(math.pi/3)
    width = height/2
    
    if complete:
        traj.add_vector(1.0, 0.5*h*width*(3+cth) + 0.5*w*width*(1+sth), vel)
        traj.new_segment()
        traj.add_vector(0.2, depth*n, vel)
        traj.new_segment()

    traj = arc(traj, center_dh = - 0.5*width*cth, center_dw = 0.5*width*sth, angle = 240.0, clockwise = False, static_ends = False, direction=direction)
    traj = arc(traj, center_dh = - 0.5*width, center_dw = 0, angle = 240.0, clockwise = True, static_ends = False, direction=direction)
    lsi  = len(traj.segment) - 1
    lpi  = len(traj.segment[lsi].point) - 1
    traj.segment[lsi].point[lpi].vel = vel
    if complete:
        traj.add_vector(0.2, - depth*n, vel)
        traj.new_segment()
        traj.add_vector(1.0, 0.5*h*width*(cth-1) + 0.6*w*width*(1+sth), vel)
    if adjust:
        traj.adjust_phase_by_distance(gain = 100.0)
    if smooth:
        traj.consistent_velocities()
    return traj

def G_old(traj = None, height = 0.1, direction = None, static_ends = True, adjust = False, smooth = True, complete = False):
    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)

    if complete:
        print "Not Supported!"

    traj = arc(traj, center_dh = - height*math.sqrt(3)/4.0, center_dw = height/4.0, angle = 300.0, clockwise = False, static_ends = static_ends, direction=direction)

    if adjust:
        traj.adjust_phase_by_distance(gain = 100.0)

    if smooth:
        traj.consistent_velocities()
    return traj

def O_old(traj = None, height = 0.1, direction = None, static_ends = True, adjust = False, smooth = True, complete = False):
    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)

    if complete:
        print "Not Supported!"

    traj = arc(traj, center_dh = - height/2.0, center_dw = 0.0, angle = 360.0, clockwise = False, static_ends = static_ends, direction=direction)

    if adjust:
        traj.adjust_phase_by_distance(gain = 100.0)

    if smooth:
        traj.consistent_velocities()
    return traj

def A(traj = None, width = 0.08, height = 0.1, direction = None, static_ends = True, adjust = True, smooth = True, complete = False):
    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)
    if complete:
        traj.add_vector(0.2, 0.2*height*n, vel)
        traj.new_segment()

    traj.add_vector(1.0, - 0.5*width*w + height*h, vel)
    traj.new_segment()
    traj.add_vector(1.0, - 0.5*width*w - height*h, vel)

    if complete:
        traj.new_segment()
        traj.add_vector(0.2, 0.75*width*w + 0.5*height*h - 0.2*height*n, vel)
        traj.new_segment()
        traj.add_vector(0.2, 0.2*height*n, vel)
        traj.new_segment()
        traj.add_vector(0.5, - 0.5*width*w, vel)
        traj.new_segment()
        traj.add_vector(1.2, - 0.2*height*n - 0.45*width*w - 0.5*height*h, vel)
    
    if adjust:
        traj.adjust_phase_by_distance(gain = 100.0)
    if smooth:
        traj.consistent_velocities()
    return traj

def canvas_shape_to_trajectory(shape, traj = None, width = 0.2, height = 0.1, direction = None, static_ends = True, adjust = True, smooth = True, width_pixels  = 640, height_pixels = 480):
    (traj, w, h, n, vel) = decode_arguments(traj, direction, static_ends)
    
    (X0, Y0) = shape[0] 
    new_pos  = np.zeros(3)    

    print "height ", h
    print "width  ", w

    for i in range(1, len(shape)):
        (X, Y) = shape[i]
        old_pos = np.copy(new_pos)
        new_pos = width*(X - X0)*w/width_pixels + height*(-Y+Y0)*h/height_pixels
        print "Num points: ", len(shape)
        if i < 5:
            print
            print i
            print "X0,Y0= ",X0,Y0
            print "X ,Y = ",X,Y
            print "Old Pos= ",old_pos
            print "New Pos= ",new_pos
            print "Delta= ",new_pos-old_pos

        if i == len(shape) - 1:
            v = np.copy(vel)
        else:
            v = np.array([None, None, None])

        traj.add_vector(0.1, new_pos - old_pos, v)
        traj.new_segment()
        
    if smooth:
        traj.consistent_velocities()     

    return traj
