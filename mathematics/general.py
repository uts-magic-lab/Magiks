'''   Header
@file:          general.py
@brief:    	    This module provides general mathematical functions
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
@version:	0.2
Last Revision:  20 November 2011
'''


import math, numpy
from interval import interval, inf

# global variables:
two_pi   = 2*math.pi
pi       = 1*math.pi
epsilon  = 0.00001
deg_to_rad_coeff = (math.pi/180.00)
f0       = float(0)
f1       = float(1)
err_code = 0

def round(x):
    
    if abs(x) < epsilon:
        y = 0
    elif abs(x - 1) < epsilon:
        y = 1
    elif abs(x + 1) < epsilon:
        y = -1
    else:
        y = x    
    return y    


def equal(v1,v2):
    '''
    Returns 1 if two values v1 and v2 are equal otherwise returns 0
    ''' 
    return (abs(v1-v2) < epsilon)

def sign(x):
    '''
    Returns 1 if x is positive, -1 if negative and 0 if abs(x) is smaller than epsilon
    '''
        
    if (abs(x) < epsilon):
        return 0
    elif x > 0:
        return 1
    else:
        return -1

def connect_interval(C):
    '''
    Sometimes the interval includes two descrite intervals so that
    the upper bound of interval i equals the lower bound of interval i + 1
    In this case it is better to connect the two intervals.
    This functions does this and returns an interval in which all sequential continuous intervals are replaced by one interval.
    '''    
    nC = len(C)
    (a, b) = C[0]
    R = interval()
    for i in range(1, nC):
        (aa, bb) = C[i]
        if equal(aa,b):
            b = bb
        else:
            R = R | interval([a,b])
            a = aa
            b = bb

    R = R | interval([a,b])

    return R        

def binary_choice(a,b,z):
    '''
    This function returns "a" or "b" depending on the sign of "z",
    if "z" is positive or zero, "a" is returned otherwise "b" is returned
    '''
    if z>=0:
        return a
    else:
        return b
    
def solve_quadratic_inequality(a, b, c):
    '''
    solves the quadratic inequality "a*x^2 + b*x + c > 0" for "x"
    and returns a feasibility set for "x" so that the inequality holds
    '''
    Delta = b**2 - 4*a*c
    if Delta > 0:
        sqrt_delta = math.sqrt(Delta)
        x_l = (- b - sqrt_delta)/(2*a)
        x_h = (- b + sqrt_delta)/(2*a)
        if a > 0:
            return interval([-inf, x_l] , [x_h, inf])
        else:
            return interval([x_l, x_h])
    else:
        if a > 0: 
            return interval([-inf, inf])
        else:
            return interval()
    
    
