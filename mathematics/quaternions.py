## @file        	quaternions.py
#  @brief     		This module provides some useful functions dealing with quaternions
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
#
#  Last Revision:  	03 January 2015 

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
    the first element of the output vector contains the scalar part of the unit quaternion and the last three elements represent the vectorial part
    
    '''

    uqn = numpy.zeros((4))
    uqn[0] = 1.00
    [u,v,w] = permutation_uvw(TRM)
        
    p = math.sqrt(1 + TRM[u,u] - TRM[v,v] - TRM[w,w])

    if (p > 0.000001):
        uqn[u+1] = p/2.0
        uqn[v+1] = (TRM[v,u] + TRM[u,v])/(2*p)
        uqn[w+1] = (TRM[w,u] + TRM[u,w])/(2*p)
        uqn[0] = (TRM[w,v] - TRM[v,w])/(2*p)
    
    assert abs(numpy.linalg.norm(uqn) - 1) < 0.000001
    return uqn

def unit_quaternion_speed(TRM,TRMD):
    '''
    Returns a vector (4 X 1) containing elements of the time derivation of the unit quaternion corresponding to transformation or rotation matrix TRM.
    Matrix TRMD is the derivation of the given transfer or rotation matrix TRM in respect with time
  
    (TRM and TRMD can be 4*4 transformation or 3*3 rotation matrices)

    The last three elements of the output vector, represent the vectorial part 
    The first element, contains the real part of the unit quaternion speed.
    '''

    uqns = numpy.zeros((4))
    [u,v,w] = permutation_uvw(TRM)
        
    p = math.sqrt(1+TRM[u,u]-TRM[v,v]-TRM[w,w])
    pp = (TRMD[u,u] - TRMD[v,v] - TRMD[w,w])/(2*p)

    uqns[u+1] = pp/2
    uqns[v+1] = (TRMD[v,u] + TRMD[u,v])/(2*p)
    uqns[v+1] = uqns[v] - pp*(TRM[v,u] + TRM[u,v])/(2*p*p)
    uqns[w+1] = (TRMD[w,u] + TRMD[u,w])/(2*p)
    uqns[w+1] = uqns[w] - pp*(TRM[w,u] + TRM[u,w])/(2*p*p)
    uqns[0] = (TRMD[w,v] - TRMD[v,w])/(2*p)
    uqns[0] = uqns[0] - pp*(TRM[w,v] - TRM[v,w])/(2*p*p)
    
    return uqns


def normalized_quaternion(TRM):
    '''
    Returns a vector (3 X 1) containing elements of the normalized quaternion corresponding to transformation matrix TRM. 
    (TRM is the 4*4 transformation matrix) 
    A normalized quaternion here means a quaternion that its real part is 1.0
    '''
    nqn = numpy.zeros((3))
    [u,v,w] = permutation_uvw(TRM)
 
    nqn[u] = (1+TRM[u,u]-TRM[v,v]-TRM[w,w])/(TRM[w,v]-TRM[v,w])
    nqn[v] = (TRM[u,v]+TRM[v,u])/(TRM[w,v]-TRM[v,w])
    nqn[w] = (TRM[u,w]+TRM[w,u])/(TRM[w,v]-TRM[v,w])

    return nqn


