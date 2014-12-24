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
@version:	    1

Last Revision:  02 September 2014

Hello


'''

import numpy as np
import copy, math

from packages.nima.mathematics import general as gen
from packages.nima.robotics.kinematics.task_space import trajectory as trajlib


def get_sign(d):
    if d == '+':
        return 1.0
    elif d == '-':
        return -1.0
    else:
        assert False, "Invalid Direction"

def get_dir(d):
    if d == '':
        p = np.zeros(3)
    elif d == 'x':
        p = np.array([1.0, 0.0, 0.0])
    elif d == 'y':
        p = np.array([0.0, 1.0, 0.0])
    elif d == 'z':
        p = np.array([0.0, 0.0, 1.0])
    else:
        assert False, "Invalid Direction"
    return p

def dir_vect(direction):

    n  = len(direction)
    s1 = 0.0
    a1 = ''
    s2 = 0.0
    a2 = ''

    if n in [2, 4, 6]:
        s0 = get_sign(direction[0])
        a0 = direction[1]
        if n in [4, 6]:
            s1 = get_sign(direction[2])
            a1 = direction[3]
        if n == 6:
            s2 = get_sign(direction[4])
            a2 = direction[5]
    else:
        assert False, "Invalid Direction"

    p = s0*get_dir(a0) + s1*get_dir(a1) + s2*get_dir(a2)
    return p

    
def line_3D(initial_pos, final_pos, initial_vel = np.zeros(3), final_vel = np.zeros(3)):
    dim = len(initial_pos)
    assert dim == len(final_pos)
    assert dim == len(initial_vel)
    assert dim == len(final_vel)

    tr = trajlib.Polynomial_Trajectory(dimension = dim)
    tr.add_point(0.0, initial_pos, initial_vel)
    tr.add_point(1.0, final_pos, final_vel)
    tr.interpolate()
    return tr

def square_3D(length = 0.1, initial_pos = np.zeros(3), direction = "+z+x", smooth = False):
    
    p = trajlib.Polynomial_Trajectory(dimension = 3)
    p.add_point(0.0, initial_pos)

    p.add_vector(1.0,   length*dir_vect(direction[0:2]))
    p.new_segment()
    p.add_vector(1.0,   length*dir_vect(direction[2:4]))
    p.new_segment()
    p.add_vector(1.0, - length*dir_vect(direction[0:2]))
    p.new_segment()
    p.add_vector(1.0, - length*dir_vect(direction[2:4]))

    if smooth:
        p.consistent_velocities()
    
    return p

'''
def arc(initial_pos = np.zeros(3), center= np.array([0.0, 0.1, 0.0]), normal_vector = np.array([1.0, 0.0, 0.0]), angle = 360.0, d_theta = 10.0, in_degrees = True):
    assert mathgen.equal(numpy.dot(center-initial_pos, normal_vector), 0.0), "Error from trajectory_shapes.arc(): The given normal vector is not perpendicular to the circle"
    
    r       = np.linalg.norm(initial_pos - center)
    i_vect  = (start_point - center)/r
    j_vect  = np.cross(normal_vector, i_vect)
    R       = np.append(np.append([i_vect],[j_vect],axis = 0), [normal_vector], axis = 0).T
    later I will complete this function
'''    

def arc(trj = None, center_dy = 0.1, center_dz = 0.0, angle = 2*math.pi, d_theta = 0.5, clockwise = True):
    '''
    adds a set of points to the given trajectory establishing an arc in a plane parallel to the yz plane starting from initial_pos, 
    The points follow an arc with given angle around the center
    input trj must be a Polynomial_Trajectory
    if trj is not given, a new trajectory will be created
    '''
    if clockwise:
        sgn = 1
    else:
        sgn = - 1
    
    if trj == None:
        trj = trajlib.Polynomial_Trajectory()
        trj.add_point(0.0, pos = np.zeros(3), vel = np.zeros(3))
    else:
        trj.new_segment()    

    nseg = len(trj.segment)
    npnt = len(trj.segment[nseg-1].point)

    initial_pos = np.copy(trj.segment[nseg-1].point[npnt - 1].pos)
    initial_phi = trj.phi_end

    r    = math.sqrt(center_dy**2 + center_dz**2)
    t0   = math.acos(- center_dy/r)
    t    = 0.0
    stay = True
    pos  = np.zeros(3)
    vel  = np.zeros(3)
    while stay:
        t = t + d_theta  
        if t > angle:  
            t = angle
            stay = False
        pos[0] = initial_pos[0]
        pos[1] = initial_pos[1] + center_dy + r*math.cos(sgn*t + t0)
        pos[2] = initial_pos[2] + center_dz + r*math.sin(sgn*t + t0)
        vel[1] = - sgn*r*math.sin(sgn*t + t0)
        vel[2] =   sgn*r*math.cos(sgn*t + t0)

        # trj.new_segment()
        trj.add_point(phi = initial_phi + t, pos = np.copy(pos), vel = np.copy(vel), acc = None)

    nseg = len(trj.segment)
    trj.segment[nseg - 1].point[1].vel = np.array([None, None, None])
    
    return trj


def M(p = None, width = 0.04, height = 0.05, initial_pos = np.zeros(3), direction = "-y+z", smooth = False):
    if p == None:
        p = trajlib.Polynomial_Trajectory(dimension = 3)
        p.add_point(0.0, initial_pos)

    p.add_vector(1.0, np.array([0.0, 0.0, height]))
    p.new_segment()
    p.add_vector(1.0, np.array([0.0, width/2, - height/2]))
    p.new_segment()
    p.add_vector(1.0, np.array([0.0, width/2, + height/2]))
    p.new_segment()
    p.add_vector(1.0, np.array([0.0, 0.0, - height]))
    if smooth:
        p.consistent_velocities()
    return p
    
def N(p = None, width = 0.04, height = 0.05, initial_pos = np.zeros(3), direction = "-y+z", smooth = False):
    if p == None:
        p = trajlib.Polynomial_Trajectory(dimension = 3)
        p.add_point(0.0, initial_pos)

    p.add_vector(1.0, np.array([0.0, 0.0, height]))
    p.new_segment()
    p.add_vector(1.0, np.array([0.0, width, - height]))
    p.new_segment()
    p.add_vector(1.0, np.array([0.0, 0.0, height]))
    if smooth:
        p.consistent_velocities()
    return p

def V(width = 0.04, height = 0.05, initial_pos = np.zeros(3), direction = "-y+z", smooth = False):
    sy = get_sign(direction[0])
    sz = get_sign(direction[2])
    if p == None:
        p = trajlib.Polynomial_Trajectory(dimension = 3)
        p.add_point(0.0, initial_pos)

    p.add_vector(1.0, np.array([0.0, sy*width/2, -sz*height]))
    p.new_segment()
    p.add_vector(1.0, np.array([0.0, sy*width/2,  sz*height]))
    if smooth:
        p.consistent_velocities()
    return p
   
def W(width = 0.04, height = 0.05, initial_pos = np.zeros(3), direction = "-y+z", smooth = False):
    p = trajlib.Polynomial_Trajectory(dimension = 3)

    sy = get_sign(direction[0])
    sz = get_sign(direction[2])

    p.add_point(0.0, initial_pos)
    p.add_vector(1.0, np.array([0.0, sy*0.15*width, - sz*height]))
    p.new_segment()
    p.add_vector(1.0, np.array([0.0, sy*0.35*width,   sz*0.6*height]))
    p.new_segment()
    p.add_vector(1.0, np.array([0.0, sy*0.35*width, - sz*0.6*height]))
    p.new_segment()
    p.add_vector(1.0, np.array([0.0, sy*0.15*width,   sz*height]))
    if smooth:
        p.consistent_velocities()
    return p

def O(diameter = 0.05, initial_pos = np.zeros(3), direction = "-y+z", smooth = True, N = 50): 
    p   = trajlib.Polynomial_Trajectory(dimension = 3)
    dt  = 2*math.pi/N
    t   = math.pi
    r   = diameter/2
    cen = initial_pos + np.array([0.0, r, 0.0])
    for i in range(N+1):
        p.add_point(phi = i, pos = cen + np.array([0.0, r*math.cos(t), r*math.sin(t)]))
        t = t - dt        
        if i < N :
            p.new_segment()

    if smooth:
        p.consistent_velocities()
    return p

def U(width = 0.04, height = 0.05, initial_pos = np.zeros(3), direction = "-y+z", smooth = False):
    p = trajlib.Polynomial_Trajectory(dimension = 3)

    sy = get_sign(direction[0])
    sz = get_sign(direction[2])

    p.add_point(0.0, initial_pos)
    p.add_vector(1.0, np.array([0.0, 0.0, - sz*(height- width/2)]))
    p.new_segment()
    p = arc(p, center_dy = sy*width/2, center_dz = 0.0, angle = math.pi, clockwise = False)
    p.new_segment()
    p.add_vector(1.0, np.array([0.0, 0.0, sz*(height- width/2)]))
    if smooth:
        p.consistent_velocities()
    return p

def P(height = 0.05, initial_pos = np.zeros(3), direction = "-y+z", smooth = False):
    p = trajlib.Polynomial_Trajectory(dimension = 3)

    sy = get_sign(direction[0])
    sz = get_sign(direction[2])

    p.add_point(0.0, initial_pos)
    p.add_vector(1.0, np.array([0.0, 0.0, sz*height]))
    p.new_segment()
    p = arc(p, center_dz = -sz*height/4, center_dy = 0.0, angle = math.pi, clockwise = False)
    if smooth:
        p.consistent_velocities()
    return p

def B(height = 0.05, initial_pos = np.zeros(3), direction = "-y+z", smooth = False):
    p = trajlib.Polynomial_Trajectory(dimension = 3)

    sy = get_sign(direction[0])
    sz = get_sign(direction[2])

    p.add_point(0.0, initial_pos)
    p.add_vector(1.0, np.array([0.0, 0.0, sz*height]))
    p.new_segment()
    p = arc(p, center_dz = -sz*height/4, center_dy = 0.0, angle = math.pi, clockwise = False)
    p = arc(p, center_dz = -sz*height/4, center_dy = 0.0, angle = math.pi, clockwise = False)
    if smooth:
        p.consistent_velocities()
    return p

    
'''
def A(width = 0.05, height = 0.1, initial_pos = np.zeros(3), direction = "+y+z", smooth = False):
    a_axis = dir_vect("+x+y+z") - abs(dir_vect(direction))
    p = trajlib.Polynomial_Trajectory(dimension = 3)
    p.add_point(0.0, initial_pos)
    
    p.add_vector(1.0, width*get_sign(direction[0])*get_dir(direction[1]) + height*get_sign(direction[2])*get_dir(direction[3]))
    p.new_segment()
    p.add_vector(1.0, width*get_sign(direction[0])*get_dir(direction[1]) - height*get_sign(direction[2])*get_dir(direction[3]))

    p.add_vector(1.0, width*get_sign(direction[0])*get_dir(direction[1]) - height*get_sign(direction[2])*get_dir(direction[3]))
    
'''
