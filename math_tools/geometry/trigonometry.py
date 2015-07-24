
## @file        	trigonometry.py
#  @brief     		This module provides some useful functions dealing with vectorial forms of trigonometric functions
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

import math,numpy

from math_tools import general_math as gen
from interval   import interval

def in_domain_sin(x):
    '''
    Returns 1 if x is between -1 and 1
    '''
    return not ((x > gen.f1 + gen.epsilon) or ( x < - gen.f1 - gen.epsilon))

def check_in_domain_sin(x):
    '''
    Checks x to be between -1 and 1 and removes the machine error if close enough to the boundaries
    If x is outside the domain and too far to -1.0 or 1.0, then an error is raised  
    If x is slightly lower than -1.0 or greater than 1.0 values -1.0 and 1.0 are returned respectively otherwise x is returned)
    '''
    y = x
    if x > gen.f1:
        assert x < gen.f1 + gen.epsilon
        y = gen.f1
    elif x < - gen.f1:
        assert x > - gen.f1 - gen.epsilon
        y = - gen.f1
    return y

def closest_angle_metric(theta0, theta):
    q = closest(theta0, [theta - 2*math.pi, theta, theta + 2*math.pi])
    d = abs(theta0 - q)
    return (q, d)    

def quarter_number(angle_rad):
    '''
    Returns the number of the quarter in which the angle is located
    '''
    a = angle_standard_range(angle_rad) #bring the angle to the standard range (- pi , pi)
    if a < - math.pi/2:
        qn = 3
    elif a < 0:
        qn = 4
    elif a < math.pi/2:
        qn = 1
    else:
        qn = 2

    return qn


def reminder_two_pi(angle_rad):
    '''
    return reminder of the given angle (in radian) divided by 2*pi 
    '''
    rest = angle_rad % gen.two_pi
    return rest

def angle_standard_range(angle_rad):
    '''
    return the corresponding angle between -pi and +pi radiants.
    '''
    if abs(angle_rad - math.pi) < gen.epsilon:
        return math.pi
            
    if abs(angle_rad + math.pi) < gen.epsilon:
        return - math.pi 

    if (angle_rad > -gen.pi) and (angle_rad < gen.pi):
        return angle_rad
    else:
        mod_two_pi = reminder_two_pi(angle_rad)
        if mod_two_pi > gen.pi :
            mod_mpi_ppi = mod_two_pi - gen.two_pi
        else :
            mod_mpi_ppi = mod_two_pi

        if abs(mod_mpi_ppi - math.pi) < gen.epsilon:
            mod_mpi_ppi = math.pi
            
        if abs(mod_mpi_ppi + math.pi) < gen.epsilon:
            mod_mpi_ppi = - math.pi

        
        # assert mod_mpi_ppi >= -gen.pi 
        # assert mod_mpi_ppi <= gen.pi 

        return mod_mpi_ppi

def angles_standard_range(q_in):
    '''
    Changes an array of angle values "q_in" if they are out of standard range (-pi  +pi) to their equivalent value in the standard range
    Returns the angles in standard range
    '''
    q_out = q_in
    for i in range(0, len(q_in)):
        q_out[i] = angle_standard_range(q_in[i]) 

    return q_out
    
def standard_interval(a,b):
    '''
    returns and interval from a to be in standard range [-pi,pi]
    '''
    ast = angle_standard_range(a)
    bst = angle_standard_range(b)

    if ast > bst:
        return interval([-math.pi,bst],[ast, math.pi])
    else:
        return interval([ast, bst])


def arccos(x):
    '''
    return arccos(x) using acos function of math package after checking the domain
    '''
    y = check_in_domain_sin(x)
    z = math.acos(y)
    return z

def arcsin(x):
    '''
    return arcsin(x) using asin function of math package after checking the domain
    '''
    y = check_in_domain_sin(x)
    z = math.asin(y)
    return z

def arcsincos(s, c):
    '''
    return the angle that its sine is "s" and its cosine is "c"
    '''
    assert gen.equal(s**2 + c**2, 1.0) 
    if s < 0:                                                        
        theta = - arccos(c)
    else:
        theta = arccos(c)

    return theta

"""
def arctan_frac(x,y):
    '''
    return arctang(x/y) using atan function of math package
    If y is very close to 0.0 (closer than gen.epsilon), one of the values - pi/2 or pi/2 is returned depending on the sign of x
    '''
    if (y < gen.epsilon) and (y >= 0.0):
        if x > 0:
            return 0.5*math.pi
        else:
            return  - 0.5*math.pi
    elif (y > - gen.epsilon) and (y <= 0.0):
        if x > 0:
            return  - 0.5*math.pi
        else:
            return 0.5*math.pi
    else:
        return math.atan(x/y)
"""

def vect_tg(v):
    n = len(v)
    tgv = numpy.zeros((n))
    for i in range(n):
        tgv[i] = math.tan(v[i])
    return tgv


def vect_cos(v):
    n = len(v)
    cosv = numpy.zeros((n))
    for i in range(0,n):
        cosv[i] = math.cos(v[i])
    return cosv

def vect_sin(v):
    n = len(v)
    sinv = numpy.zeros((n))
    for i in range(0,n):
        sinv[i] = math.sin(v[i])
    return sinv

def vect_arccos(v):
    n = len(v)
    arccosv = numpy.zeros((n))
    for i in range(0,n):
        arccosv[i] = arccos(v[i])
    return arccosv

def vect_arcsin(v):
    n = len(v)
    arcsinv = numpy.zeros((n))
    for i in range(0,n):
        arcsinv[i] = arcsin(v[i])
    return arcsinv

def solve_equation(code, parameters):
    '''
    Solves the trigonometric equation for t denoted by:
    code = 1: parameters should be: [p1, p2, p3]
    p1*cos(t) + p2*sin(t) + p3 = 0
    code = 2:
    p1*cos^2(t) + p2*sin^2(t) + p3*sin(t)*cos(t) + p4*sin(t) + p5*cos(t) + p6 = 0
    
    It returns the solution set as an array
    If no solution exists, it returns an empty array
    '''
    solution_set = []
    if code == 1:
        R = math.sqrt(parameters[0]**2 + parameters[1]**2)
        assert (not gen.equal(R, 0))
        s = - parameters[2] / R
        if in_domain_sin(s):
            sai0 = math.atan2(parameters[0], parameters[1])
            if gen.equal(math.sin(sai0), parameters[0]/R):
                sai = sai0
            elif equal(math.sin(sai0), - parameters[0]/R):
                sai = sai0 + math.pi
            else:
                # If you came here, something is wrong !
                assert(False)
     
            t1 = arcsin(s) - sai
            t2 = math.pi - t1 - 2*sai
            
            if gen.equal(parameters[0]*math.cos(t1) + parameters[1]*math.sin(t1) + parameters[2] , 0):
                solution_set.append(t1)
            
            if gen.equal(parameters[0]*math.cos(t2) + parameters[1]*math.sin(t2) + parameters[2] , 0):
                solution_set.append(t2)

        return solution_set
        
def solve_system(code,A,B,x,y,z):
    '''
    solves a trigonometric system of equations 

        x = cos(t0)*(A*sin(t1) + B*cos(t1))   (1)
        y = sin(t0)*(A*sin(t1) + B*cos(t1))   (2)
        z = -B*sin(t1) + A*cos(t1)            (3)

        for two angles t0 and t1 and returns a set of paired solutions as tulips (t0,t1)

        The system is over constrained so a solution may not exist. In thic case, an empty set is returned.
    '''
    solution_set = []
    t0 = []
    t0.append(math.atan2(y, x))
    t0.append(t0[0] + math.pi)
    
    t1 = solve_equation(1, [A, - B, - z])
    
    for i in range(0,len(t0)): 
        for j in range(0,len(t1)):   
            q = A*math.sin(t1[j]) + B*math.cos(t1[j])
            p = A*math.cos(t1[j]) - B*math.sin(t1[j])
            c0 = math.cos(t0[i])
            s0 = math.sin(t0[i])
            if (gen.equal(x , c0*q) and
               gen.equal(y , s0*q) and
               gen.equal(z , p)):
                solution_set.append((t0[i],t1[j]))
    
    return solution_set

class Fourier_Series(object):
    '''
    A_0 + Sigma_{n = 1}^{N} A_n*sin(2*pi*n*x/T) + B_n*cos(2*pi*n*x/T)
    '''
    def __init__(self, N = 1):
        self.N  = N
        self.A  = [0.0 for i in range(N)]
        self.B  = [0.0 for i in range(N)]
        self.T  = 2*math.pi

    def position(self, t):
        s = 0.0
        w = 2*math.pi/self.T
        for n in range(self.N):
            s = s + self.A[n]*math.sin(w*n*t) + self.B[n]*math.cos(w*n*t)
        return(s)   

    def velocity(self, t):
        s = 0.0
        w = 2*math.pi/self.T
        for n in range(self.N):
            s = s + w*n*(self.A[n]*math.cos(w*n*t) - self.B[n]*math.sin(w*n*t))
        return(s)   
 
    def acceleration(self, t):
        s  = 0.0
        w  = 2*math.pi/self.T
        w2 = w*w
        for n in range(self.N):
            n2 = n*n 
            s  = s - w2*n2*(self.A[n]*math.sin(w*n*t) + self.B[n]*math.cos(w*n*t))
        return(s)   

