'''   Header
@file:          laser_scan_support.py
@brief:    	    This module contains some functions which can convert raw laser data into some information
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
@version:	    1.0

First Revision: 12 December 2014

'''

import numpy as np
import copy, math

from math_tools.algebra import vectors_and_matrices as vecmat

def ls_line(dist, angle):
    '''
    Returns the least squares line passing through the points specified by given distances and angles
    '''
    N = len(dist)
    assert N == len(angle), "Error from ls_line(): dist and angle must have the same length"
    x = np.zeros(N)
    y = np.zeros(N)

    for i in range(N):
        x[i] = dist[i]*math.sin(angle[i])
        y[i] = dist[i]*math.cos(angle[i])

    (a, b) = np.polyfit(x, y, 1)
    r      = a*x + b - y
    return (a, b, r)    

def front_line(dist, position_range, angle_min, angle_step):
    '''
    Returns the equation of front line in the form of a tuple (a, b, r)
    the equation of the line is: y = a*x + b 
    r is the vector of residuals    
    '''
    d     = []
    theta = []
    for i in position_range:
        theta.append(angle_min + angle_step*i)
        d.append(dist[i])
    
    (a, b, e ) = ls_line(d, theta)
    while max(e) > 0.01:
        outliers = vecmat.which(e, '>' , 0.01)
        d        = vecmat.remove(d, outliers)
        theta    = vecmat.remove(theta, outliers)
        (a, b, e ) = ls_line(d, theta)
        print
        print "len(e)  = ", len(e)
        print "max(e)  = ", max(e)
        print "Angle   = ", 180.0*math.atan(a) / math.pi

    return (a, b , e)
