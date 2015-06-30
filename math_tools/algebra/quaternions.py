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
import general_python as genpy
from math_tools import general_math as gen

def dbl2str(n):
    d = abs(int(n));
    f = int((n - d) * 1000000);
    s = '%d.%d' % (d, f) if f > 0 else '%d' % d
    ss = '+' if n >= 0 else '-'
    return ss, s

class Quaternion(object):
    def __init__(q, w=0, x=0, y=0, z=0):
        q.w = w
        q.x = x
        q.y = y
        q.z = z

    ## Operator function for addition of two quaternions
    def __add__(q1, q2):
        return Quaternion(
            q1.w + q2.w,
            q1.x + q2.x,
            q1.y + q2.y,
            q1.z + q2.z,
        )

    ## Operator function for subtraction of two quaternions
    def __sub__(q1, q2):
        return Quaternion(
            q1.w - q2.w,
            q1.x - q2.x,
            q1.y - q2.y,
            q1.z - q2.z,
        )

    ## Operator function for multiplication of two quaternions
    def __mul__(q1, q2):
        return Quaternion(
            q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
            q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
            q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
            q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w,
        )

    ## Operator function for division of two quaternions
    def __div__(q1, q2):
        s = float(q2.w*q2.w + q2.x*q2.x + q2.y*q2.y + q2.z*q2.z)
        return Quaternion(
            (  q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z) / s,
            (- q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y) / s,
            (- q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x) / s,
            (- q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w) / s
        )

    ## Operator function for the magnitude or norm of the quaternion
    #  @return A scalar float containing the norm of the given quaternion
    def __abs__(q):
        return math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);

    ## Operator function for the negative of a quaternion
    def __neg__(q):
        return Quaternion(-q.w, -q.x, -q.y, -q.z)

    ## Operator function for the conjugate of a quaternion.
    #  @return The conjugate of the given quaternion (\f$ \bar q \f$) 
    def __invert__(q):
        """Conjugate of Quaternion.

        >>> q = Quaternion((2, 2, 2, 2))
        >>> print(q)
        (2 + 2i + 2j + 2k)
        >>> print(~q)
        (2 - 2i - 2j - 2k)
        >>> print(~~q)
        (2 + 2i + 2j + 2k)

        """
        return Quaternion((q.w, -q.x, -q.y, -q.z))

    def as_tuple(self):
        return (self.w, self.x, self.y, self.z)

    ## @return The quaternion instance as a numpy vector
    def as_vector(self):
        return numpy.array([self.w, self.x, self.y, self.z])

    def __str__(q):
        args = list(
            dbl2str(q.w) +
            dbl2str(q.x) +
            dbl2str(q.y) +
            dbl2str(q.z)
        )
        if args[0] == '+':
            args[0] = ''
        return '(%s%s %s %si %s %sj %s %sk)' % tuple(args)

    def normalize(q):
        """Convert Quaternion to Unit Quaternion.

        Unit Quaternion is Quaternion who's length is equal to 1.

        >>> q = Quaternion((1, 3, 3, 3))
        >>> q.normalize()
        >>> print(q) # doctest: +ELLIPSIS
        (0.1889822... + 0.5669467...i + 0.5669467...j + 0.5669467...k)

        """
        norm = abs(q)
        q.w = q.w / norm,
        q.x = q.x / norm,
        q.y = q.y / norm,
        q.z = q.z / norm,


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
    assert gen.equal(numpy.linalg.det(TRM[0:3,0:3]), 1.0), genpy.err_str(__name__, self.__class__.__name__,sys._getframe().f_code.co_name, "Given TRM is not a rotation matrix.")

    uqn = numpy.zeros((4))
    uqn[0] = 1.00
    [u,v,w] = permutation_uvw(TRM)
        
    p = 1.0 + TRM[u,u] - TRM[v,v] - TRM[w,w]

    if (p > gen.epsilon):
        p        = math.sqrt(p)
        uqn[u+1] = p/2.0
        uqn[v+1] = (TRM[v,u] + TRM[u,v])/(2*p)
        uqn[w+1] = (TRM[w,u] + TRM[u,w])/(2*p)
        uqn[0] = (TRM[w,v] - TRM[v,w])/(2*p)
    
    if uqn[0] < 0:
        uqn = - uqn

    assert gen.equal(numpy.linalg.norm(uqn), 1.0), "Impossible !"
    return uqn

def unit_quaternion_velocity(TRM, TRMD):
    '''
    Returns a vector (4 X 1) containing elements of the time derivation of the unit quaternion corresponding to transformation or rotation matrix TRM.
    Matrix TRMD is the time derivative of the given transfer or rotation matrix TRM
  
    (TRM and TRMD can be 4*4 transformation or 3*3 rotation matrices)

    The last three elements of the output vector, represent the vectorial part 
    The first element, contains the real part of the unit quaternion speed.
    '''
    uqns = numpy.zeros((4))
    [u,v,w] = permutation_uvw(TRM)
        
    p = math.sqrt(1+TRM[u,u]-TRM[v,v]-TRM[w,w])
    pp = (TRMD[u,u] - TRMD[v,v] - TRMD[w,w])/(2*p)

    uqns[u+1] = pp/2
    uqns[v+1] = (TRMD[v,u] + TRMD[u,v])/(2*p) - pp*(TRM[v,u] + TRM[u,v])/(2*p*p)
    uqns[w+1] = (TRMD[w,u] + TRMD[u,w])/(2*p) - pp*(TRM[w,u] + TRM[u,w])/(2*p*p)
    uqns[0] = (TRMD[w,v] - TRMD[v,w])/(2*p) - pp*(TRM[w,v] - TRM[v,w])/(2*p*p)
    # to check if the quaternion was negative:

    if (p > 0.000001):
        w = (TRM[w,v] - TRM[v,w])/(2*p)
    else:
        w = 1.0    
    
    if w < 0:
        uqns = - uqns
    
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


