'''   Header
@file:          quaternions.py
@brief:    	    This module provides some useful functions dealing with quaternions.
@author:        Nima Ramezani; DFKI Bremen
@start date:    February 2010
@version:	    0.1
Last Revision:  31 March 2011
'''
import math,numpy

def permutation_uvw(R):

    u = 0
    v = 1
    w = 2

    for j in range(1,3):
        if R[j,j] > R[u,u]:
            u = j
            v = j + 1
            w = j + 2
            if v > 2:
                v = v - 3 
            if w > 2:
                w = w - 3 

    uvw = [u,v,w]   
    return uvw

def unit_quaternion(TRM):
    '''
    Returns a vector (4 X 1) containing elements of the unit quaternion corresponding to transformation or rotation matrix TRM. 
    (TRM can be the 4*4 transformation matrix or 3*3 rotation matrix)

    The first three elements of the output vector, represent the vectorial part and the forth (last) element, contains the real part of the unit quaternion
    '''

    uqn = numpy.zeros((4))
    uqn[3] = 1.00
    [u,v,w] = permutation_uvw(TRM)
        
    p = math.sqrt(1 + TRM[u,u] - TRM[v,v] - TRM[w,w])

    if (p > 0.000001):
        uqn[u] = p/2
        uqn[v] = (TRM[v,u] + TRM[u,v])/(2*p)
        uqn[w] = (TRM[w,u] + TRM[u,w])/(2*p)
        uqn[3] = (TRM[w,v] - TRM[v,w])/(2*p)
    
    assert abs(numpy.linalg.norm(uqn) - 1) < 0.000001
    return uqn

def unit_quaternion_speed(TRM,TRMD):
    '''
    Returns a vector (4 X 1) containing elements of the time derivation of the unit quaternion corresponding to transformation or rotation matrix TRM.
    Matrix TRMD is the derivation of the given transfer or rotation matrix TRM in respect with time
  
    (TRM and TRMD can be 4*4 transformation or 3*3 rotation matrices)

    The first three elements of the output vector, represent the vectorial part 
    The forth (last) element, contains the real part of the unit quaternion speed.
    '''

    uqns = numpy.zeros((4))
    [u,v,w] = permutation_uvw(TRM)
        
    p = math.sqrt(1+TRM[u,u]-TRM[v,v]-TRM[w,w])
    pp = (TRMD[u,u] - TRMD[v,v] - TRMD[w,w])/(2*p)

    uqns[u] = pp/2
    uqns[v] = (TRMD[v,u] + TRMD[u,v])/(2*p)
    uqns[v] = uqns[v] - pp*(TRM[v,u] + TRM[u,v])/(2*p*p)
    uqns[w] = (TRMD[w,u] + TRMD[u,w])/(2*p)
    uqns[w] = uqns[w] - pp*(TRM[w,u] + TRM[u,w])/(2*p*p)
    uqns[3] = (TRMD[w,v] - TRMD[v,w])/(2*p)
    uqns[3] = uqns[3] - pp*(TRM[w,v] - TRM[v,w])/(2*p*p)
    
    return uqns


def normalized_quaternion(TRM):
    '''
    Returns a vector (3 X 1) containing elements of the normalized quaternion corresponding to transformation matrix TRM. 
    (TRM is the 4*4 transformation matrix) 
    '''
    nqn = numpy.zeros((3))
    [u,v,w] = permutation_uvw(TRM)
 
    nqn[u] = (1+TRM[u,u]-TRM[v,v]-TRM[w,w])/(TRM[w,v]-TRM[v,w])
    nqn[v] = (TRM[u,v]+TRM[v,u])/(TRM[w,v]-TRM[v,w])
    nqn[w] = (TRM[u,w]+TRM[w,u])/(TRM[w,v]-TRM[v,w])

    return nqn


