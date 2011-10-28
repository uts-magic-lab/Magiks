'''   Header
@file:          vectors_and_matrices.py
@brief:    	    This module provides some useful functions dealing with vectors and matrices.
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
@version:	    0.3
Last Revision:  29 October 2011
'''

import math,numpy, general, trigonometry

# global variables:
f0       = float(0)
f1       = float(1)
err_code = 0

def as_matrix(v):
    '''
    gets vector v with n^2 elements and returns n*n matrix
    '''
    n_2 = len(v)
    n = int(math.sqrt(n_2))
    R = numpy.zeros((n,n))
    for i in range(n):
        for j in range(n):
            print "n*i + j = ", n*i + j
            print "R = ", v, n_2, n, R
            
            R[i,j] = v[n*i + j]
    return R            

def format_array( list_of_vals, format="%.3f" ) : 
    '''
    parametrized usage of sprintf
    '''
    formatted = []
    for v in list_of_vals : 
        if int(v) == v : 
            formatted.append( v ) 
        else : 
            formatted.append( format%v  )          
    return formatted

def format_vector( vector, format="%.3f" ) : 
    '''
    parametrized usage of sprintf
    '''
    n = len(vector)
    formatted = numpy.zeros((n))
    
    for i in range(n):
        formatted[i] = format%vector[i]          
    return formatted

def format_matrix( matrix, format="%.3f" ) : 
    '''
    parametrized usage of sprintf
    '''
    m = matrix.shape[0]
    n = matrix.shape[1]
    formatted = numpy.zeros((m,n))
    
    for i in range(m):
        for j in range(n):
            formatted[i,j] = format%matrix[i,j]          
    return formatted


def matrix_column_multiply(A,v):
    '''
    equivalent to numpy.dot( A, numpy.diag(v) ) 
    
    multiplies each column of matrix A by the corresponding element in vector v. 
    This is equivalent to A*diag(v) but requires less calculations

    return numpy.dot( A, numpy.diag(v) ) 
    '''
    m = A.shape[0]
    n = A.shape[1]
    assert n == len(v)
    Adiagv = numpy.zeros((m,n))
    for i in range(0,m):
        for j in range(0,n):
            Adiagv[i,j] = A[i,j]*v[j]
    return Adiagv

def normalize(v):
    '''
    Returns the unit vector parallel to the given vector.
    '''
    l = numpy.linalg.norm(v)
    if general.equal(l, 0.0):
        return(v)
    else:
        return(v/l)

def linear_map(q, f, g):
    '''
    return the linear mapping of vector q by coefficients f and g
        
        u = (q - g) / f
    
    This code should be implemented before:

        for i in range(0,len(f[i])):
            if abs(f[i]) < mgvlib.epsilon:
                print 'Error 01: Division by zero occured'
                print 'Something is wrong with the joint limits or the joint limit multipliers'
                print 'Make sure that the joint limits are well defined, and method "initialize" of the manipulator configuration has been implemented'
                assert False
    '''
        
    u = (q - g) / f
    return u    
 
def inner_product(v1, v2):
    '''
    returns the inner product of the two vectors v1 and v2
    '''
    return numpy.sum(v1*v2)
   
def as_vector(v):
    '''
    Returns a numpy array equal to v. Input argument v must be a normal array of numbers
    '''
    return(numpy.array(v))

def vectors_angle(v1, v2, in_degrees = False, positive_direction = numpy.zeros(3)):
    '''
    Returns the angle between two vectors. The positive direction, specifies which halfspace is used as positive vectors for the cross product of u and v
    if not specified, the returned angle will be always positive. 
    If specified, then if the sign of the returned angle is the same as the inner product of positive_direction and crossproduct(v1,v2)
    '''
        
    l_v1 = numpy.linalg.norm(v1)
    l_v2 = numpy.linalg.norm(v2)
    assert not general.equal(l_v1, 0.0)
    assert not general.equal(l_v2, 0.0)

    cos_theta = inner_product(v1, v2)/(l_v1*l_v2)
    
    theta = trigonometry.arccos(cos_theta)
    if general.equal(numpy.linalg.norm(positive_direction), 0.0):
        return(theta)
    else:
        return math.copysign(theta, inner_product(numpy.cross(v1, v2), positive_direction))

def linear_map_inv(u,f,g):
    '''
    return the inverse linear mapping of vector u by coefficients f and g
    
        q = f * u + g 
    '''
    q = f * u + g 
    return q 
    
def diag_old(v):
    '''
    return an square diagonal matrix whose diagonal elements are elements of vector v
    '''
    # numpy.diag : Use k>0 for diagonals above the main diagonal, and k<0 for diagonals below the main diagonal.
    return numpy.diag(v, k = 0 )
    

def vector_element_multiply_old(v1,v2):
    '''
    elementwise multiplication of two vectors. 
    
    !!! http://www.scipy.org/NumPy_for_Matlab_Users !!!     
    '''
    return v1 * v2
    

def cut_vector_old(v4):
    '''
    returns a three element vector containing the first three elements of v4
    '''
    return v4[0:3]

def extend_vector(v3):
    '''
    get a three element vector and add one element to its end. Return a four element vector
    '''
    v4 = numpy.zeros((4))
    for i in range(0,3):
        v4[i] = v3[i]
    v4[3] = 1
    return v4
    
def equal(v1,v2, epsilon = general.epsilon):
    '''
    Returns 1 if two vectors or matrices are equal otherwise returns 0
    '''
    return (numpy.linalg.norm(v1-v2) < len(v1)*epsilon)

def uvect(TRM,m):
    '''
    uvect is a shorted phrase of "unit vector"
    
    Returns a vector (3 X 1) containing the first three elements of the m-th column of transformation matrix TRM. 
    (TRM is the 4*4 transformation matrix or a 3X3 rotation matrix) 
    If m is in [0,1,2] the m-th unit vector is returned. If m is 3, then the function returns the position vector
    '''
    return TRM[ 0:3,m ] 

def extract_rotation_matrix(TRM):
    '''
    Returns the rotation matrix (3 X 3) extracted from transformation matrix TRM. TRM is a 4*4 matrix
    '''
    return TRM[ 0:3, 0:3 ] 
    
def cross_old(u,v):
    '''
    Return the cross product of two vectors u and v. u and v are (3 element) vectors
    '''
    return numpy.cross(u,v)

def skew(v):
    '''
    Return the skew matrix (3X3) of vector v. v has 3 elements
    '''
    sk = numpy.zeros((3,3))
    #sk = numpy.array([v,v,v])

    sk[0,0] = 0
    sk[0,1] = - v[2]
    sk[0,2] = v[1]

    sk[1,0] = v[2]
    sk[1,1] = 0
    sk[1,2] = - v[0]

    sk[2,0] = - v[1]
    sk[2,1] = v[0]
    sk[2,2] = 0

    return sk;


def extended_matrix( R, p ):
    '''
    Convert a (3 X 3) rotation matrix into a (4 X 4) transformation matrix. The added elements will be zero
    '''
    em = numpy.zeros((4,4))
    for i in range(0,3):
        em[i,3] = p[i]
        em[3,i] = 0
        for j in range(0,3):
            em[i,j] = R[i,j]
    return em;


def right_pseudo_inverse(M):
    '''
    Return the right pseudo-inverse of matrix M
    '''
    A = numpy.dot(M,M.T);
    vinv = numpy.dot(v.T,numpy.linalg.inv(A))
    return vinv


def right_dls_inverse(M, k):
    '''
    returns the right side damped least square inverse of matrix M. k is the damping factor.
    
    --> take a look at ! numpy.linalg.pinv(a, rcond=1.0000000000000001e-15) !
    '''
    m = M.shape[0]
    A = numpy.dot(M, M.T);
    
    vinv = numpy.dot(v.T,numpy.linalg.inv(A + k*k*numpy.eye(m)))
    return vinv


def relative_trace(RMa,RMd):
    '''
    Return the trace of the product of the two given matrices Rma and RMd. RMa will be multiplied by transpose of RMd from left side
    '''
    R = numpy.dot(RMa, RMd.T)
    
    trace_R = numpy.trace(R) 
    # numpy.trace(a, offset=0, axis1=0, axis2=1, dtype=None, out=None)
    # trace_R = R[0,0] + R[1,1] + R[2,2]

    return trace_R

