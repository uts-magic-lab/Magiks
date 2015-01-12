## @file        	general.py
#  @brief     		This module provides general mathematical functions
#  @author      	Nima Ramezani Taghiabadi 
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. :   04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	1.0
# 
#  Last Revision:  	03 January 2015 

import math, numpy
from interval import interval, inf

# global variables:
two_pi   = 2*math.pi
pi       = 1*math.pi
deg_to_rad    = (math.pi/180.00) #convert an angle from degree to radian by multiplying it by this variable
rad_to_deg    = (180.00/math.pi) #convert an angle from radian to degree by multiplying it by this variable

epsilon  = 0.00001 # a very small number 
f0       = float(0) # constant number zero as a float
f1       = float(1) # constant number one as a float
err_code = 0
infinity = float("inf")

def sign_choice(x, y, z):
    if z > 0:
        return x
    elif z < 0:
        return y        
    else:
        print "Error from sign_choice(): value z can not be zero"

def inv(x):
    if abs(x) < epsilon:
        if x > 0:
            return infinity
        else:
            return - infinity    
    else:
        return 1.0/x

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

def round_mat(A):
    n = A.shape[0]
    m = A.shape[1]
    B = numpy.zeros((n,m))

    for i in range(n):
        for j in range(m):
            B[i,j] = round(A[i,j])
    return(B)    

def equal(v1,v2, epsilon = epsilon):
    '''
    Returns 1 if two values v1 and v2 are equal otherwise returns 0
    ''' 
    return (abs(v1-v2) < epsilon)

def ensured_in_range(x, xl, xh):
    if x < xl: 
        x = xl
    if x > xh: 
        x = xh
    return x

def sign(x):
    '''
    Returns 1 if x is positive, -1 if negative and 0 if abs(x) is smaller than epsilon
    '''
        
    if (abs(x) < epsilon):
        return 0.0
    elif x > 0:
        return 1.0
    else:
        return -1.0

def connect_interval(C):
    '''
    Sometimes the interval includes two descrite intervals so that
    the upper bound of interval i equals the lower bound of interval i + 1
    In this case it is better to connect the two intervals.
    This functions does this and returns an interval in which all sequential continuous intervals are replaced by one interval.
    '''    
    nC = len(C)
    if nC == 0:
        return(C)
    else:
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
    
def closest_border(x, S, k = 0.01):
    '''
    Returns the closest border of set S to value x. S is an interval variable from package interval
    '''
    d_min = infinity
    for intrvl in S:
        (xl, xh) = intrvl
        d = abs(x - xl)
        if d < d_min:
            d_min   = d
            clx     = xl + k*(xh-xl)

        d = abs(x - xh)
        if d < d_min:
            d_min   = d
            clx     = xh - k*(xh-xl)
    return clx    

def accommodating_interval(x, S):
    '''
    This function returns the interval in which x is located(accommodated)
    if x is not in the interval, then None is returned    
    '''
    for intrvl in S:
        (xl,xh) = intrvl
        if (x >= xl)  and (x <= xh):
            return intrvl

    return None

def gauss_rec(phi, W_arr, r, c, h, N_w):
    '''
    This function is translated from simulink model created by Gabriel
    The .mdl file can be found in:
    /home/nimasoft/Dropbox/software/matlab/packages/gabriell/Periodic_learning_v5_fafo_simulink/output_dyn_system_v4_simple.m
    '''

    '''
    phi, r, h must be scalar values
    c, W_arr  must be arrays of minimum length N_w
    N_w = length(W_arr)
    '''

    # put input to the output dynamical system
    num_in = 0
    den_in = 0
    for i in range(N_w):
      (psi,arg) = gauss_kernel_cosine(phi, c[i], h)
      '''  
      print "W_arr[i] = ", W_arr[i]  
      print "r        = ", r
      print "psi      = ", psi
      print "num_in   = ", num_in
      '''  
      num_in = num_in + psi*W_arr[i]*r
      den_in = den_in + psi

    return(num_in/den_in)

def gauss_kernel_cosine(phi, c, h):
    '''
    This function is translated from Matlab code written by Gabriel.
    The Matlab code can be found in:
    /home/nimasoft/Dropbox/software/matlab/packages/gabriell/Periodic_learning_v5_fafo_simulink/gauss_kernel_cosine.m
    '''
    # psi: Gaussian kernel function
    # phi: phase 
    # c: value between 0 and 2*pi
    # h: 'width' of the kernel function

    arg = math.cos(phi - c) - 1.0
    psi = math.exp(h*arg)
    output = (psi, arg)
    return output


