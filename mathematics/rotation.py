## @file        	rotation.py
#  @brief     		This module provides some useful functions for rotation in 3D space 
#  @author      	Nima Ramezani Taghiabadi 
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. :   04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	2.0
#
#  start date:      February 2011 
#  Last Revision:  	03 January 2015 

import math, numpy 
import general, quaternions, vectors_and_matrices, trigonometry
import packages.nima.general as gen 


i_uv = numpy.array([1.0, 0.0, 0.0])
j_uv = numpy.array([0.0, 1.0, 0.0])
k_uv = numpy.array([0.0, 0.0, 1.0])

point_forward_orientation   = numpy.append(numpy.append([j_uv],[k_uv],axis = 0), [i_uv], axis = 0).T
point_backward_orientation  = numpy.append(numpy.append([j_uv],[-k_uv],axis = 0), [-i_uv], axis = 0).T
point_left_orientation      = numpy.append(numpy.append([k_uv],[i_uv],axis = 0), [j_uv], axis = 0).T
point_right_orientation     = numpy.append(numpy.append([-k_uv],[i_uv],axis = 0), [-j_uv], axis = 0).T
point_upward_orientation    = numpy.eye(3)
point_downward_orientation  = numpy.append(numpy.append([-i_uv],[j_uv],axis = 0), [-k_uv], axis = 0).T

param_set = [
        'pm_angle_axis'                                ,
        'pm_unit_quaternion'                           ,
        'pm_angular_spherical'                         ,
        'pm_vectorial_identity'                        ,
        'pm_vectorial_Cayley_Gibbs_Rodrigues'          ,
        'pm_vectorial_Wiener_Milenkovic'               ,
        'pm_vectorial_linear'                          ,
        'pm_vectorial_reduced_Euler_Rodrigues'         ,
        'pm_vectorial_normailized_quaternion'          ,
        'pm_angular_Euler_Kardan'                      
        ]

def DH_transfer_matrix(theta,alpha,a,d):
    '''
    Returns the transformation matrix (4 X 4) based on Denavit-Hartenberg standard. (Reference 1, pages 36 to 40)
    '''
    sa = math.sin(alpha)
    ca = math.cos(alpha)
    st = math.sin(theta)
    ct = math.cos(theta)

    TM = numpy.array( [  [ct  , -ca*st,  sa*st, a*ct ],
                         [st  ,  ca*ct, -sa*ct, a*st ],
                         [0.0 ,  sa   ,  ca   , d    ],
                         [0.0 ,  0.0  ,  0.0  , 1.0  ]  ] )
    return TM

def orthogonal(R):
    '''
    returns an orientation orthogonal to the given orientation    
    '''    
    w =   R[:,0]
    h =   R[:,1]
    n =   R[:,2]
        
    N = numpy.cross(- n, w)
    if N[0] < 0:
        N = - N

    H = - n

    if H[2] < 0:
        H = - H
    
    W = numpy.cross(H,N)

    return numpy.append(numpy.append([W],[H],axis = 0), [N], axis = 0).T

def reverse(R):
    '''
    returns an orientation reverse to the given orientation    
    '''    
    R2 = numpy.copy(R)

    R2[:0] = - R[:0]
    R2[:1] = - R[:1]
    return R2

def trans_hemogeneous(v):
    '''
    Returns a  4X4 hemogeneous translation matrix corresponding to translation by given vector v
    if elements of v are numeric  values, then the elements of the translation matrix are numeric
    if elements of v are symbolic values, then the trans matrix is symbolic in terms of elements of v
    '''
    TH = numpy.array([[ 1, 0, 0, v[0] ], 
                      [ 0, 1, 0, v[1] ],
                      [ 0, 0, 1, v[2] ],
                      [ 0, 0, 0, 1    ]])
    return TH


def rot_y(theta, hemogeneous = False, symbolic = False, S = None, C = None):
    '''
    Returns the rotation matrix corresponding to rotation of "theta" around "z" axis  
    if hemogeneous is True, then a 4X4 hemogeneous transfer matrix is returned
    if symbolic is set to True
    Returns the rotation matrix in terms of given c = 'cos(t)' and s = 'sin(t)' 
    if c and s are numeric values, then the elements of the rotation matrix are numeric
    if c and s are symbolic values, then the rot matrix is symbolic in terms of s and c
    if symbolic is set to True the value of theta is ignored

    '''
    if symbolic:    
        from sympy import Symbol
        S = replace_if_none(S, Symbol('s'))
        C = replace_if_none(C, Symbol('c'))
    else:
        C = math.cos(theta)
        S = math.sin(theta)

    if hemogeneous:
        R_y     = numpy.array([[  C   , 0 , S , 0 ], 
                               [  0   , 1 , 0 , 0 ],
                               [- S   , 0 , C , 0 ],
                               [  0   , 0 , 0 , 1 ]]) 
    else:
        R_y     = numpy.array([[  C   , 0  , S   ], 
                               [  0   , 1  , 0   ],
                               [ -S   , 0  , C   ]]) 
    return R_y

def rot_x(theta, hemogeneous = False, symbolic = False, S = None, C = None):
    '''
    Similar to rot_y but rotation is around x axis
    '''
    if symbolic:    
        from sympy import Symbol
        S = replace_if_none(S, Symbol('s'))
        C = replace_if_none(C, Symbol('c'))
    else:
        C = math.cos(theta)
        S = math.sin(theta)
    if hemogeneous:
        R_x     = numpy.array([[  1 , 0  , 0, 0 ], 
                               [  0 , C  , -S, 0 ],
                               [  0 , S  ,  C, 0 ],
                               [  0 , 0  , 0, 1 ]]) 
    else:
        R_x     = numpy.array([[  1 ,  0  , 0 ], 
                               [  0 ,  C  , -S ],
                               [  0 ,  S  ,  C ]]) 
    return R_x

def rot_z(theta, hemogeneous = False, symbolic = False, S = None, C = None):
    '''
    Similar to rot_y but rotation is around z axis
    '''
    if symbolic:
        from sympy import Symbol
        S = gen.replace_if_none(S, Symbol('s'))    
        C = gen.replace_if_none(S, Symbol('c'))    
    else:
        C = math.cos(theta)
        S = math.sin(theta)

    if hemogeneous:
        R_z     = numpy.array([[  C   , -S   , 0, 0 ], 
                               [  S   ,  C   , 0, 0 ],
                               [  0   , 0   , 1, 0 ],
                               [  0   , 0   , 0, 1 ]]) 
    else:
        R_z     = numpy.array([[  C   , -S , 0 ], 
                               [  S   ,  C , 0 ],
                               [  0 , 0   , 1 ]]) 
    return R_z


def rotation_matrix(rv, parametrization = 'unit_quaternion', symbolic = False, derivative = False, C = None, S = None, U = None, W = None):
    '''
    This function returns the rotation matrix corresponding to rotation vector as p(phi)*U where:
    phi is the rotation angle and U is the unit vector directing towards the axis of rotation
    If type is "Linear Parametrization", then: p(phi) = sin(phi)
    if "parametrization = 'unit_quaternion', a unit quaternion must be given. The first element rv[0] is the scalar part and 
    the next three elements (rv[1], rv[2], rv[3]) represent the vectorial part
        
    if argument derivative is True the derivative of the rotation matrix wrt angle phi (rv[0]) is return. 
    Arguments "symbolic" and "derivative" are now supported only for angle-axis parametrization. C = cos(phi) , S = sin(phi) 
    when phi is the rotation angle and [Ux, Uy, Uz] is the axis
    '''

    if symbolic:
        from sympy import Symbol
        C = gen.replace_if_none(C, Symbol('c'))
        S = gen.replace_if_none(S, Symbol('s'))
        U = gen.replace_if_none(U, [Symbol('ux'), Symbol('uy'), Symbol('uz')])
        W = gen.replace_if_none(W, Symbol('w'))

    u = numpy.zeros((3))
    
    if parametrization == 'angular_spherical':
        sin_phi = math.sin(rv[0])
        cos_phi = math.cos(rv[0])
        u[0] = math.sin(rv[1])*math.cos(rv[2])
        u[1] = math.sin(rv[1])*math.sin(rv[2])
        u[2] = math.cos(rv[1])
        
    elif parametrization == 'angle_axis':
        if symbolic:
            sin_phi = S
            cos_phi = C
            u[0] = U[0]
            u[1] = U[1]
            u[2] = U[2]

        else:
            sin_phi = math.sin(rv[0])
            cos_phi = math.cos(rv[0])
            u[0] = rv[1]
            u[1] = rv[2]
            u[2] = rv[3]
        
    elif parametrization == 'unit_quaternion':
        assert len(rv) == 4

        assert general.equal(numpy.linalg.norm(rv), 1.0)

        RM = numpy.eye(3)
        a2 = rv[0]**2
        b2 = rv[1]**2
        c2 = rv[2]**2
        d2 = rv[3]**2
        ab = 2*rv[0]*rv[1]
        ac = 2*rv[0]*rv[2]
        ad = 2*rv[0]*rv[3]
        
        bc = 2*rv[1]*rv[2]
        bd = 2*rv[1]*rv[3]
        
        cd = 2*rv[2]*rv[3]
        
        RM[0,0] = a2 + b2 - c2 - d2
        RM[0,1] = bc - ad
        RM[0,2] = bd + ac
        
        RM[1,0] = bc + ad
        RM[1,1] = a2 - b2 + c2 - d2
        RM[1,2] = cd - ab
        
        RM[2,0] = bd - ac
        RM[2,1] = cd + ab
        RM[2,2] = a2 - b2 - c2 + d2
        return(RM)
        
    else:
        p = numpy.linalg.norm(rv)
        u = rv/p
        
    if parametrization == 'vectorial_identity':
        #p = math.pi + angle_in_range(p)
        sin_phi = math.sin(p)
        cos_phi = math.cos(p)
    elif parametrization == 'vectorial_linear':
        assert (p <= 1) 
        sin_phi = p
        cos_phi = math.sqrt(1 - p*p)
    elif parametrization == 'vectorial_reduced_Euler_Rodrigues':
        assert (p <= 2) 
        sin_phi_2 = p/2
        cos_phi_2 = math.sqrt(1 - p*p/4)
        sin_phi = 2*sin_phi_2*cos_phi_2
        cos_phi = 2*cos_phi_2*cos_phi_2 - 1
    elif parametrization == 'vectorial_Cayley_Gibbs_Rodrigues':
        t2 = p*p/4
        sin_phi = p/(1 + t2)
        cos_phi = (1 - t2)/(1 + t2)
    elif parametrization == 'vectorial_Wiener_Milenkovic':
        phi_4 = math.atan(p/4)
        sin_phi = math.sin(4*phi_4)
        cos_phi = math.cos(4*phi_4)

    u = vectors_and_matrices.normalize(u)
    skew_u = vectors_and_matrices.skew(u)

    if derivative:
        RM = cos_phi*skew_u + sin_phi*numpy.dot(skew_u,skew_u)
    else: 
        RM = numpy.eye(3) + sin_phi*skew_u + (1 - cos_phi)*numpy.dot(skew_u,skew_u)
    return(RM)    


def angle_axis(TRM):

    '''
    Returns a vector (4 X 1) containing elements of the angle and axis corresponding to transformation or rotation matrix TRM. 
    (TRM can be the 4*4 transformation matrix or 3*3 rotation matrix)

    The first three elements of the output vector, represent the axis unit vector and the forth (last) element, contains the angle in radians
    '''

    an_ax = numpy.zeros((4))
    an_ax[0] = 1.00

    q = quaternions.unit_quaternion(TRM)

    trace_R = TRM[0,0] + TRM[1,1] + TRM[2,2]

    cos_phi_2 = q[3]
    cos_phi = 0.5*(trace_R - 1)

    phi = trigonometry.arccos(cos_phi)

    if cos_phi_2 < 0:
        '''
        phi is in zone 3 or 4
        '''
        phi = 2*math.pi - phi

    if (phi != 0) and (phi != 2*math.pi):
        q = q / math.sin(phi/2)
        an_ax = q 

    an_ax[0] = phi


    return an_ax
        
def spherical_angles(TRM):
    '''
    This function returns three spherical angles phi,thete and sai cprresponding to orientation corresponding to the given rotation or transfer matrix: TRM
    The output is a vector of three elements containing phi,theta and sai in order.
    '''
    sa = numpy.zeros((3))
    ax = angle_axis(TRM)

    sa[0] = ax[0]
        
    sa[2] = math.atan2(ax[2], ax[1])
    sa[1] = trigonometry.arccos(ax[3])

    return sa

def orientation_vector(TRM, parametrization):
    '''
    This function returns the orientation as an array of three or four elements depending on the given parametrization
    if the parametrization is angular the output is an array of three angles
    if the parametrization is vectorial, the output will be a vector as u = p(phi)*u
    phi and u are defined based on the angle-axis representation of rotaion
    phi is the rotation angle and U is the unit vector directing towards the axis of rotation corresponding to the given rotation or transfer matrix: TRM
    and p(phi) is a function of phi depending on the parametrization
    '''

    if parametrization == 'angular_spherical':
        rv = spherical_angles(TRM)
        return rv
    elif parametrization == 'unit_quaternion':
        rv = quaternions.unit_quaternion(TRM)
        return rv
    elif parametrization == 'angle_axis':
        rv = angle_axis(TRM)
        return rv
    else:
        ax = angle_axis(TRM)
        phi = ax[0]
        u = numpy.zeros((3))
        for j in range(0,3):
            u[j] = ax[j+1]

    if parametrization == 'vectorial_identity':
        p = phi
    elif parametrization == 'vectorial_linear':
        p = math.sin(phi)
    elif parametrization == 'vectorial_reduced_Euler_Rodrigues':
        p = 2*math.sin(0.5*phi)
    elif parametrization == 'vectorial_Cayley_Gibbs_Rodrigues':
        p = 2*math.tan(0.5*phi)
    elif parametrization == 'vectorial_Wiener_Milenkovic':
        p = 4*math.tan(0.25*phi)
    else: 
        assert False

    rv = p*u

    return rv
    
def relative_rotation_vector(RMa,RMd,parametrization):
    '''
    this function calculates the relative rotation vector
    '''
    na = RMa[:,0]
    sa = RMa[:,1]
    aa = RMa[:,2]

    nd = RMd[:,0]
    sd = RMd[:,1]
    ad = RMd[:,2]

    '''
    na = uvect(RMa,0)
    sa = uvect(RMa,1)
    aa = uvect(RMa,2)

    nd = uvect(RMd,0)
    sd = uvect(RMd,1)
    ad = uvect(RMd,2)
    '''

    rrvl = 0.5*(numpy.cross(na,nd) + numpy.cross(sa,sd) + numpy.cross(aa,ad))

    if parametrization == 'vectorial_linear':
        rrv = rrvl
    else:
        p = numpy.linalg.norm(rrvl)
        u = rrvl/p
        phi = trigonometry.arcsin(p)

        if parametrization == 'vectorial_identity':
            rrv = phi*u
        
        elif parametrization == 'reduced_Euler_Rodrigues':
            rrv = 2*math.sin(0.5*phi)*u

        elif parametrization == 'Cayley_Gibbs_Rodrigues':
            rrv = 2*math.tan(0.5*phi)*u

        elif parametrization == 'Wiener_Milenkovic':
            p = 4*math.tan(0.25*phi)*u

        else: 
            assert False

    return rrv


def relative_rotation_angle(RMa,RMd):
    '''
    return the "Angle" when relative rotation matrix defined as: R_actual * Transpose(R_desired) is represented in Angle-Axis form
    
    '''
    R = numpy.dot(RMa,RMd.T)
    trace_R = R[0,0] + R[1,1] + R[2,2]

    cos_phi = 0.5*(trace_R - 1)

    phi = trigonometry.arccos(cos_phi)

    return phi

