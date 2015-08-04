## @file        	geometry.py
#  @brief     		This module contains a comprehensive data structure with relevant method and properties
#                   for identities in three-dimensional geometry like a point, a rotation or orinetation in 3D space, an ellipsoid and etc.
#  @author      	Nima Ramezani Taghiabadi 
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. :   04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	3.0
#
#  start date:      02 May 2015 
#  Last Revision:  	02 May 2015 

import math, sys, numpy as np, trigonometry as trig, rotation as rot, general_python as genpy

from math_tools.algebra import vectors_and_matrices as vm, quaternions as quat, optimization as opt
from math_tools import general_math as gen


'''
Changes from version 2:
All cgkit functions removed
'''
"""
References:

  [1] Olivier A. Bauchau et.al, "The Vectorial Parameterization of Rotation", Nonlinear Dynamics, 32, No 1, pp 71 - 92, 2003 
"""

'''
i_uv = np.array([1.0, 0.0, 0.0])
j_uv = np.array([0.0, 1.0, 0.0])
k_uv = np.array([0.0, 0.0, 1.0])

backward_rotmat  = np.append(np.append([j_uv],[-k_uv],axis = 0), [-i_uv], axis = 0).T
left_rotmat      = np.append(np.append([k_uv],[i_uv],axis = 0), [j_uv], axis = 0).T
right_rotmat     = np.append(np.append([-k_uv],[i_uv],axis = 0), [-j_uv], axis = 0).T
upward_rotmat    = np.eye(3)
downward_rotmat  = np.append(np.append([-i_uv],[j_uv],axis = 0), [-k_uv], axis = 0).T

ori_backward      = Orientation(backward_rotmat)
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
'''

## This class, introduces a structure for a point in the three-dimensional space. 
class Point_3D(object):
    
    ## Class Constructor
    #  @param pos The desired position vector at the key point
    #  @param vel The desired velocity vector at the key point
    #  @param acc The desired acceleration vector at the key point   
    def __init__(self, pos, vel = None, acc = None, representation = 'cartesian'):
       
       ## An integer indicating the dimension of space in which the kepy point is defined.
       #  This number specifies the number of elements of position, velocity and acceleration vectors  
       self.dim     = len(pos) 
       
       self.__setitem__(representation, pos)
       self.set_velocity(vel, representation)
       self.set_acceleration(acc, representation)

    def clear(self):
       # A numpy vector indicating the position of the point in cartesian coordinates
       self.p = None

       # A numpy vector indicating the position of the point in spherical coordinates
       self.s = None

       # A numpy vector indicating the position of the point in cylindrical coordinates
       self.c = None

    def clear_velocity(self):
       # A numpy vector indicating the velocity of the point in cartesian coordinates
       self.pd     = None

       # A numpy vector indicating the velocity of the point in spherical coordinates
       self.sd     = None

       # A numpy vector indicating the velocity of the point in cylindrical coordinates
       self.cd     = None

    def clear_acceleration(self):
       # A numpy vector indicating the acceleration of the point in cartesian coordinates
       self.pdd    = None
       # A numpy vector indicating the acceleration of the point in spherical coordinates
       self.sdd    = None
       # A numpy vector indicating the acceleration of the point in cylindrical coordinates
       self.cdd    = None

    def __getitem__(self, representation):
       if representation == 'cartesian':
           return self.cartesian()
       elif representation == 'cartesian_velocity':
           return self.cartesian_velocity()
       elif representation == 'cartesian_acceleration':     
           return self.cartesian_acceleration()
       if representation == 'spherical':
           return self.spherical()
       elif representation == 'spherical_velocity':
           return self.spherical_velocity()
       elif representation == 'spherical_acceleration':     
           return self.spherical_acceleration()
       if representation == 'cylindrical':
           return self.cylindrical()
       elif representation == 'cylindrical_velocity':
           return self.cylindrical_velocity()
       elif representation == 'cylindrical_acceleration':     
           return self.cylindrical_acceleration()
       else:
           assert False, genpy.err_str(__name__ , self.__class__.__name__ , '__getitem__', representation + ' is not a valid value for representation')

    def is_none(self) :
       return self.p == None     

    def velocity_is_none(self) :
       return self.pd == None     

    def acceleration_is_none(self) :
       return self.pdd == None     

    def set_velocity(self, value, representation): 
       self.clear_velocity() 
       if representation == 'cartesian':
           self.pd   = value
       elif representation == 'spherical':
           self.sd   = value
       elif representation == 'cylindrical':
           self.cd   = value
       else:
           assert False, genpy.err_str(__name__ , self.__class__.__name__ , 'set_velocity', representation + ' is not a valid value for representation')

    def set_acceleration(self, value, representation): 
       self.clear_acceleration() 
       if representation == 'cartesian':
           self.pdd   = value
       elif representation == 'spherical':
           self.sdd   = value
       elif representation == 'cylindrical':
           self.cdd   = value
       else:
           assert False, genpy.err_str(__name__ , self.__class__.__name__ , 'set_acceleration', representation + ' is not a valid value for representation')

    def __setitem__(self, representation, value):
        self.clear() 
        if representation == 'cartesian':
            self.p   = value
        elif representation == 'spherical':
            '''
            s[0] = ro
            s[1] = theta
            s[2] = phi
            '''
            self.s   = value
        elif representation == 'cylindrical':
            '''
            c[0] = r
            c[1] = theta
            c[2] = z
            '''
            self.c   = value
        else:
            assert False, genpy.err_str(__name__ , self.__class__.__name__ , '__setitem__', representation + ' is not a valid value for representation')

    def cartesian(self):
        if self.p != None:
            return self.p
        elif self.s != None:
            self.p    = np.zeros(3)
            self.p[0] = self.s[0]*math.cos(self.s[1])*math.cos(self.s[2])
            self.p[1] = self.s[0]*math.sin(self.s[1])*math.cos(self.s[2])
            self.p[2] = self.s[0]*math.sin(self.s[2])
            return self.p
        elif self.c != None:
            self.p    = np.zeros(3)
            self.p[0] = self.c[0]*math.cos(self.c[1])
            self.p[1] = self.c[0]*math.sin(self.c[1])
            self.p[2] = self.c[2]
            return self.p
        else:
            return None            

    def spherical(self):
        if self.s == None:
            self.s    = np.zeros(3)
            [r, t, z] = self.cylindrical()
            self.s[0] = math.sqrt(r*r + z*z)  # ro
            self.s[1] = t                     # theta  
            self.s[2] = math.atan2(z, r)      # phi
        return self.s

    def cylindrical(self):
        if self.c == None:
            self.c    = np.zeros(3)
            [x, y, z] = self.cartesian()
            self.c[0] = math.sqrt(x*x + y*y)  # r
            self.c[1] = math.atan2(y, x)      # theta
            self.c[2] = z
        return self.c

    def dist(self, point):
        p1 = self.cartesian()
        p2 = point.cartesian()
        return np.linalg.norm(p1 - p2)

    ## This function is the string representation of the key point
    #  @param None
    #  @return A string representing all the information about the key point  
    def __str__( self ):
       s  = "Point Dimension: " + str(self.dim) + '\n' 

       if self.p != None: 
           s += "Position       : " + str(self.p) + '\n'
       if self.pd != None: 
           s += "Velocity       : " + str(self.pd) + '\n'
       if self.pdd != None: 
           s += "Acceleration   : " + str(self.pdd) + '\n'
       return s

    ## Use this function to get the current value of position, velocity or acceleration in a desired dimension 
    #  @param field_name A string, must be selected from 
    #                    set: ['position', 'velocity', 'acceleration'  
    #                    specifying which vector is desired.  
    #  @param axis A non-negative integer specifying which element of the vector should be returned. 
    #             (Must not be greater than the space dimension) 
    #  @return A float containing the value of the element specified by argument \b axis from the vector specified by argument \b field_name   
    def value(self, field_name = 'position', axis = 0):
        assert (axis <= self.dim), "Error from " + __name__ + func_name + ": Argument axis must not esxeed the space dimension"
        assert (axis >= 0), "Error from " + __name__ + func_name + ": Argument axis can not have a negative value"  
        
        if field_name == 'position':
            return self.pos[axis]
        elif field_name == 'velocity':
            return self.vel[axis]
        elif field_name == 'acceleration':
            return self.acc[axis]
        else:
            print "Error from " + __name__ + func_name + ": " + field_name + " is not not a valid value for argument field_name"

    def __add__(p1, p2):
        return Point_3D(pos = p1.cartesian() + p2.cartesian())

    def __sub__(p1, p2):
        return Point_3D(pos = p1.cartesian() - p2.cartesian())

    def __neg__(p):
        return Point_3D(pos = - p.cartesian())

## This class, introduces a rotation in the three-dimensional space. 
#  It supports various representations and convensions of orientation. 
#  This class also supports computation orientation speed represented by various convensions.
class Orientation_3D(object):

    ## private
    def clear_acceleration(self):    
        ## A numpy vector of size 3 containing \f$ \ddot{\p} \f$ (second derivative of orientation vector \f$ p \f$ w.r.t. time) 
        #  where \f$ p \f$ is the vectorial representation of orientation which is defined according to property \b self.parametrization
        self.pdd = None

        ## A numpy 3x3 rotation matrix: Representation of orientation acceleration as 3x3 numpy rotation matrix second derivative w.r.t. time
        self.Rdd = None

    ## Clears all velocity properties. All velocity properties (self.omega, self.Rd, self.Qd, self.ud, self.pd, self.phid) will be set to None.
     # @param None
     # @return None
    def clear_velocity(self):    
        ## A numpy vector of size 3 containing the <em> angular velocity </em> representation of orientation velocity
        self.omega = None

        ## A numpy 3x3 rotation matrix: Representation of orientation velocity as 3x3 numpy rotation matrix derivative w.r.t. time
        self.Rd = None

        ## A numpy vector of size 4: Quaternion representation of orientation velocity as a numpy vector (\f$ \frac{ dq }{ dt } \f$)
        self.Qd = None

        ## A float containing \f$ \dot{\phi} \f$ (derivative of \f$ \phi \f$ w.r.t. time), 
        #  where \f$ \phi \f$ is the \b angle in \em Angle-Axis representation of orientation
        self.phid = None

        ## A numpy vector of size 3 containing \f$ \dot{\u} \f$ (derivative of unit vector \f$ u \f$ w.r.t. time) 
        #  where \f$ u \f$ is the \b axis in \em Angle-Axis representation of orientation
        self.ud = None

        ## A numpy vector of size 3 containing \f$ \dot{\p} \f$ (derivative of orientation vector \f$ p \f$ w.r.t. time) 
        #  where \f$ p \f$ is the vectorial representation of orientation which is defined according to property \b self.parametrization
        self.pd = None

    ## Clears all rotation properties. All rotation properties (self.R, self.Q, self.u, self.p, self.phi, self.H, self.sa) will be set to None.
     # @param None
     # @return None
    def clear(self):
        ## A numpy 3x3 rotation matrix: Representation of orientation as 3x3 numpy rotation matrix 
        self.R = None

        ## A numpy vector of size 4: Quaternion representation of orientation as a numpy vector
        self.Q = None

        ## A numpy vector of size 3: Representation of orientation as a non-redundant numpy vector (depends on the generator function)
        self.p = None

        ## A float: Angle phi in Angle-Axis representation of orientation
        self.phi = None

        ## A numpy vector of size 3: Unit vector u in Angle-Axis representation of orientation
        self.u = None

        self.H = None

        ## A numpy vector of 3 elements: Non-redundant representation of orientation by three <em> Spherical Orientation Angles </em>
        self.sa = None

    ## Use this function to find out if a rotation by any convention is defined for the instance
     # @param None
     # @return A boolean: True if all rotation properties (self.p, self.R, self.u, self.phi, self.Q) are set to None, False, if otherwise. 
    def is_none(self) :
        return (self.p == None) and (self.R == None) and (self.u == None) and (self.phi == None) and (self.Q == None)

    ## Use this function to find out if a rotation velocity is defined for the instance
     # @param None
     # @return A boolean: True if all rotation speed properties (self.Rd, self.ud, self.phid, self.Qd, self.pd) are set to None, False, if otherwise. 
    def velocity_is_none(self) :
        return (self.pd == None) and (self.Rd == None) and (self.ud == None) and (self.phid == None) and (self.Qd == None)

    ## private    
    def acceleration_is_none(self) :
        return self.pdd == None     
        
    ## Class Constructor:
    #  @param ori: The orientation or rotation defined
    #         \li A tuple or numpy vector of 4 elements if representation = 'quaternion',    
    #         \li A 3 X 3 numpy matrix if representation = 'matrix',    
    #         \li A tuple or numpy vector of 4 elements if representation = 'angle_axis',
    #         \li A tuple or numpy vector of 3 elements if representation = 'vector',
    #  @param  representation: A string: Must be one of the supported convensions:
    #         \li matrix
    #         \li quaternion
    #         \li angle_axis
    #         \li vector
    def __init__(self, ori, representation = 'matrix', ori_velocity = None, ori_acceleration = None, parametrization = 'identity'):
        self.representation      = representation
        self.parametrization = parametrization
        self.__setitem__(representation, ori)
        self.m = 1.0
        if ori_velocity == None:
            self.clear_velocity()
        else:
            self.set_velocity(ori_velocity, representation)

        if ori_acceleration == None:
            self.clear_acceleration()
        else:
            self.set_acceleration(ori_acceleration, representation)

    def __str__(self):
        
        s =  "3D Orientation represented by " + self.representation + ": \n \n" 
        s += str(self[self.representation])
        s += "\n"
        return s

    def tensor(self):
        if self.H == None:
            if gen.equal(self.angle(), 0.0) :
                return np.eye(3)
            
            p        = self.vector()
            px       = rot.skew(p)
            px2      = np.dot(px, px)
            p_nrm    = np.linalg.norm(p)
            p2       = p_nrm*p_nrm
            p2_inv   = gen.inv(p2)
            v    = self.noo()
            z    = self.zeta()
            m    = self.moo()
            v2   = v*v
            v2_z = v2*gen.inv(z)
            self.H = m*np.eye(3) + 0.5*v2*px + p2_inv*(m - v2_z)*px2 
        return self.H

    def matrix_velocity(self):
        if self.velocity_is_none():
            return None    
        elif self.Rd != None:
            return self.Rd
        elif self.omega != None:
            r  = self.matrix()
            wx = rot.skew(self.omega)
            self.Rd = np.dot(wx, r)
        elif self.Qd != None:
            assert False, "Not Supported !"
        elif (self.phid != None) and (self.ud != None):
            phi = self.angle()
            u   = self.axis()
            ux  = rot.skew(u)
            self.omega = self.phid*u + np.dot(math.sin(phi)*np.eye(3) + (1.0 - math.cos(phi))*ux, ud)
            '''
            Reference: Olivier A. Bauchau et.al, "The Vectorial Parameterization of Rotation", Nonlinear Dynamics, 32, No 1, pp 71 - 92, 2003 (EQ.10)
            '''    
        elif self.pd != None:
            self.omega = np.dot(self.tensor(), self.pd)
        else:
            assert False, "This should not happen !"

        return self.matrix_velocity()

    def angular_velocity(self):
        if self.velocity_is_none():
            return None    
        elif self.omega == None:
            rd = self.matrix_velocity()
            r  = self.matrix()
            # Ref [1]: Equation 9
            self.omega  = rot.axial(np.dot(rd, r.T))
        return self.omega

    def quaternion_velocity(self):
        if self.velocity_is_none():
            return None    
        elif self.Qd == None:
            '''
            r  = self.matrix()
            rd = self.matrix_velocity()
            self.Qd = quaternions.unit_quaternion_velocity(r, rd)
            '''
            phi  = self.angle()
            phid = self.angle_velocity()
            u    = self.axis()
            ud   = self.axis_velocity()
            s    = math.sin(phi/2)
            c    = math.cos(phi/2)
            wd   = - 0.5*s*phid
            ed   =   0.5*c*phid*u + s*ud
            self.Qd = np.array([wd, ed[0], ed[1], ed[2]])
        return self.Qd

    def angle_velocity(self):
        if self.velocity_is_none():
            return None    
        elif self.phid == None:
            phi     = self.angle()
            rd      = self.matrix_velocity()
            sin_phi = math.sin(phi)
            self.phid = np.trace(rd)/(-2*sin_phi)
        return self.phid

    def axis_velocity(self):
        if self.velocity_is_none():
            return None    
        elif self.ud == None:
            u       = self.axis()
            phi     = self.angle()
            sin_phi = math.sin(phi)
            cos_phi = math.cos(phi)
            phid    = self.angle_velocity()             
            rd      = self.matrix_velocity()
            axrd    = rot.axial(rd)
            
            self.ud = (axrd - cos_phi*phid*u)/sin_phi
        return self.ud
    
    def matrix(self):
        if self.R != None:
            return self.R

        elif self.Q != None:
            self.R = rot.rotation_matrix(self.Q) 
            return self.matrix()

        elif (self.phi != None) and (self.u != None):
            ux = rot.skew(self.u)
            self.R = np.eye(3) + math.sin(self.phi)*ux + (1.0 - math.cos(self.phi))*np.dot(ux,ux)
            return self.R

        elif self.p != None:
            px    = rot.skew(self.p)
            px2   = np.dot(px, px)
            p     = np.linalg.norm(self.p)
            self.phi = self.gen_fun_inv(p)
            v    = self.noo()
            z    = self.zeta()
            v2   = v*v
            v2_z = v2*gen.inv(z)
            self.R = np.eye(3) + v2_z*px + 0.5*v2*px2
            return self.R

        else:
            return None

    def vector(self):
        if self.p == None:
            if (self.parametrization == 'linear') and (self.m == 1.0): # Special Case: Linear parametrization p(phi) = sin(phi)
                self.p = rot.axial(self.matrix())
                return self.p
            else:
                phi = self.angle()
                f = self.gen_fun(phi)

            self.p = f*self.axis()
        return self.p    
            
    def vector_velocity(self):
        if self.velocity_is_none():
            return None    
        elif self.pd == None:
            if (self.parametrization == 'linear') and (self.m == 1.0): # Special Case: Linear parametrization p(phi) = sin(phi)
                self.pd = rot.axial(self.matrix_velocity())
                return self.pd
            else:
                phi = self.angle()
                f  = self.gen_fun(phi)
                fp = self.gen_fun_derivative(phi)

            self.pd = fp*self.angle_velocity()*self.axis()+f*self.axis_velocity()
        return self.pd

    def vector_acceleration(self):
        return self.pdd

    ## Sets the parametrization for the generating function in vectorial representation of orientation.
    #  @param parametrization A string specifying the parametrization. Valid values are:
    #                         'identity', 'linear', 'cayley-gibbs-rodrigues', 'bauchau-trainelli', 'exponential'
    #  @return None
    def set_parametrization(self, parametrization = 'identity'): # p(phi) = (phi/m)
        valid_parametrizations = ['identity', 'linear', 'cayley-gibbs-rodrigues', 'bauchau-trainelli', 'exponential']
        genpy.check_valid(parametrization, valid_parametrizations, __name__, self.__class__.__name__,sys._getframe().f_code.co_name, 'parametrization')
        if parametrization != self.parametrization:
            self.parametrization = parametrization
            self.p  = None
            self.pd = None
            if self.parametrization in ['cayley-gibbs-rodrigues']: 
                self.m = 2.0
            else:
                self.m = 1.0

    ## Returns the value of the generating function for the given angle phi
    #  @param phi A float variable specifying the angle
    #  @return A float containing the value of the generating function for the given phi.
    #          The generating function depends on property self.parametrization 
    def gen_fun(self, phi):
        genpy.check_type(phi, [float, np.float64], __name__, self.__class__.__name__,sys._getframe().f_code.co_name, 'phi')
        if self.parametrization == 'linear': # generating function: p(phi) = m*sin(phi/m)
            return self.m*math.sin(phi/self.m)
        elif self.parametrization == 'identity': # p(phi) = phi/m (m = 1.0)
            return phi/self.m
        elif self.parametrization == 'cayley-gibbs-rodrigues': # p(phi) = m*tan(phi/m)  (m = 2)
            return self.m*math.tan(phi/self.m)
        elif self.parametrization == 'bauchau-trainelli':  #  p(phi) = (6*(phi - sin(phi)))**(1.0/3.0)
            s   = math.sin(phi)
            a   = 6*(phi - s)
            return a**0.33333333
        elif self.parametrization == 'exponential': # p(phi) = exp(phi)-1
            return  math.exp(phi) - 1.0
        else:
            assert False,"Not Supported!"

    def gen_fun_inv(self, p):
        if self.parametrization == 'linear':  # generating function: p(phi) = m*sin(phi/m)
            return self.m*trig.arcsin(p/self.m)
        elif self.parametrization == 'identity':  # p(phi) = phi/m (m = 1)
            return self.m*p
        elif self.parametrization == 'cayley-gibbs-rodrigues': # p(phi) = m*tan(phi/m)  (m = 2)
            return self.m*trig.arctan(p/self.m)
        elif self.parametrization == 'bauchau-trainelli':  # p(phi) = (6*(phi - sin(phi)))**(1.0/3.0)
            assert False, "Not Supported!"
        elif self.parametrization == 'exponential': # p(phi) = exp(phi)-1
            return  math.log(p+1.0)
        else:
            assert False,"Not Supported!"

    def gen_fun_derivative(self, phi):
        if self.parametrization == 'linear':
            return math.cos(phi/self.m)
        elif self.parametrization == 'identity':
            return 1.0/self.m
        elif self.parametrization == 'cayley-gibbs-rodrigues':
            t  = math.tan(phi/self.m)
            return 1.0 + t*t
        elif self.parametrization == 'bauchau-trainelli':  # Bauchau-Trainelli parametrization
            s   = math.sin(phi)
            c   = math.cos(phi)
            a   = 6*(phi - s)
            f   = a**0.33333333
            b   = gen.inv(f)
            return  2*(1.0-c)*b*b
        elif self.parametrization == 'exponential': # Exponential
            return  math.exp(phi)
        else:
            assert False,"Not Supported!"

    def noo(self, phi = None):
        if phi == None:
            phi = self.angle()

        if gen.equal(phi, 0.0) and self.parametrization == 'identity':
            return self.m

        p = self.gen_fun(phi)    
        return 2*math.sin(phi/2)*gen.inv(p)
        
    def zeta(self, phi = None):
        if phi == None:
            phi = self.angle()

        if gen.equal(phi, 0.0) and self.parametrization == 'identity':
            return self.m

        p = self.gen_fun(phi)    
        return 2*math.tan(phi/2)*gen.inv(p)

    def moo(self, phi = None):
        if phi == None:
            phi = self.angle()
        pp = self.gen_fun_derivative(phi)    
        return gen.inv(pp)

    def quaternion(self):
        if self.Q == None:
            r       = self.matrix()
            self.Q  = quat.unit_quaternion(r)

            '''
            phi = self.angle()
            u   = self.axis()
            e   = math.sin(phi/2)*u
            w   = math.cos(phi/2)    
            self.Q  = np.array([w, e[0], e[1], e[2]])
            '''
            # Q = self.quaternion_cgt()
            # self.Q = np.array([Q.w, Q.x, Q.y, Q.z])
        return self.Q
            
    def angle(self):
        if self.phi == None:
            if self.p != None:
                self.phi = self.gen_fun_inv(np.linalg.norm(self.p))
            else:
                # Alternative 1: (self.phi, self.u_cgt) = self.quaternion_cgt().toAngleAxis() 
                # Alternative 2: self.phi  = trig.arccos(0.5*(np.trace(self.matrix()) - 1.0))
                # Alternative 3
                '''
                r  = self.matrix()
                ax = rot.angle_axis(r)
                self.phi = ax[0]
                self.u   = ax[1:4]
                '''
                self.phi  = trig.arccos(0.5*(np.trace(self.matrix()) - 1.0))

        return self.phi

    def spherical(self):
        if self.sa == None:
            self.sa = np.zeros((3))
            ax      = self.angle_axis()
            self.sa[0] = ax[0]
            self.sa[2] = math.atan2(ax[2], ax[1])
            self.sa[1] = trig.arccos(ax[3])

        return self.sa
            
    def angle_axis(self):
        u = self.axis()
        return np.array([self.angle(), u[0], u[1], u[2]])

    def axis(self):
        if self.u == None:
            r  = self.matrix()
            ax = rot.angle_axis(r)
            self.phi = ax[0]
            self.u   = ax[1:4]

        return self.u

    def frame_axis(self, k):
        r = self.matrix()
        return r[0:3, k]

    def __setitem__(self, representation, value):

        self.clear()
        if representation == 'matrix':
            self.R = value
        elif representation == 'quaternion':
            self.Q = value
        elif representation == 'angle_axis':
            self.phi = value[0]
            self.u   = value[1:4]
        elif representation == 'vector':
            self.p = value
        else:
            assert False, genpy.err_str(__name__ , self.__class__.__name__ , '__setitem__', representation + ' is not a valid value for representation')

    def __getitem__(self, representation):
        if representation == 'matrix':
            return self.matrix()
        elif representation == 'quaternion':
            return self.quaternion()
        elif representation == 'angle_axis':
            return self.angle_axis()
        elif representation == 'angle':
            return self.angle()
        elif representation == 'axis':
            return self.axis()
        elif representation == 'vector':
            return self.vector()
        elif representation == 'trace':
            return np.trace(self.matrix())
        elif representation == 'diag':
            return np.diag(self.matrix())
        elif representation == 'angular_spherical':
            return self.spherical()
        elif representation == 'matrix_velocity':
            return self.matrix_velocity()
        elif representation == 'quaternion_velocity':
            return self.quaternion_velocity()
        elif representation == 'angle_axis_velocity':
            u = self.axis_velocity()
            return np.array([self.angle_velocity(), u[0], u[1], u[2]])
        elif representation == 'angle_velocity':
            return self.angle_velocity()
        elif representation == 'axis_velocity':
            return self.axis_velocity()
        elif representation == 'vector_velocity':
            return self.vector_velocity()
        elif representation == 'vector_acceleration':
            return self.vector_acceleration()
        elif representation == 'trace_velocity':
            return np.trace(self.matrix_velocity())
        elif representation == 'diag_velocity':
            return np.diag(self.matrix_velocity())
        else:
            assert False, genpy.err_str(__name__ , self.__class__.__name__ , '__getitem__', representation + ' is not a valid value for representation')

    def set_acceleration(self, ori_dd, representation = None):
        self.clear_acceleration()
        if representation == 'vector':
            self.pdd = oridd
        else:
            assert False, genpy.err_str(__name__ , self.__class__.__name__ , '__getitem__', representation + ' is not a valid value for representation')

    def set_velocity(self, value, representation = None):
        self.clear_velocity()
        if representation == None:
            representation = self.representation
        if representation == 'quaternion':
            self.Qd = value
        elif representation == 'matrix':
            self.Rd = value
        elif representation == 'angle_axis':
            self.phid = value[0]
            self.ud   = value[1:4]
        elif representation == 'vector':
            self.pd = value
        else:
            assert False, "Not supported Yet !"
   
    def set_acceleration(self, value, representation = None): 
        self.clear_acceleration() 
        if representation == None:
            representation = self.representation
        if representation == 'vector':
            self.pdd   = value
        elif representation == 'matrix':
            self.Rdd   = value
        else:
            assert False, genpy.err_str(__name__ , self.__class__.__name__ , 'set_acceleration', representation + ' is not a valid value for representation')

    def get_velocity(self, representation = None):
        if representation == None:
            representation = self.representation

        if representation == 'quaternion':
            return self.quaternion_velocity()
        elif representation == 'matrix':
            return self.matrix_velocity()
        elif representation == 'angle_axis':
            return self.angle_axis_velocity()
        elif representation == 'angular_spherical':
            assert False
            return self.spherical_velocity()
        else:
            assert False, "Not supported Yet !"
    
    def __div__(o1, o2):
        r1 = o1.matrix()
        r2 = o2.matrix()
        r1_dot = o1.matrix_velocity()
        r2_dot = o2.matrix_velocity()
        r      = np.dot(r1, r2.T)
        if (r1_dot == None) or (r2_dot == None):
            o = Orientation_3D(r)
        else:
            r_dot = np.dot(r1_dot, r2.T) + np.dot(r1, r2_dot.T)
            o = Orientation_3D(r, ori_velocity = r_dot)

        if o1.parametrization == o2.parametrization:
            o.set_parametrization(o1.parametrization)
        else:
            o.parametrization = None
        o.representation = o1.representation    
        return o
            
    def __mul__(o1, o2):
        r1 = o1.matrix()
        r2 = o2.matrix()
        r1_dot = o1.matrix_velocity()
        r2_dot = o2.matrix_velocity()
        r      = np.dot(r1, r2)
        if (r1_dot == None) or (r2_dot == None):
            o = Orientation_3D(r)
        else:
            r_dot = np.dot(r1_dot, r2) + np.dot(r1, r2_dot)
            o = Orientation_3D(r, ori_velocity = r_dot)
            
        if o1.parametrization == o2.parametrization:
            o.set_parametrization(o1.parametrization)
        else:
            o.parametrization = None
        o.representation = o1.representation    
        return o
            
class Ellipsoid(object):
    ## Class Constructor
    #  Creates an ellipsoid with given charachteristic matrix A and center. The equation of the ellipsoid is given as:
    #  $$
    #  **(x - c)**^T \cdot **M** \cdot **(x - c)** 
    #  $$
    #  @param M      A numpy square matrix representing the charachteristic matrix of the ellipsoid.
    #  @param center An instance of class Point_3D() specifying the ellipsoid center
    def __init__(self, M = np.eye(3), center = Point_3D(pos = np.zeros(3))):
        '''
        For example to have a non-rotated ellipsoid centered at the origin with equation:
        x^2/a^2 + y^2/b^2 + z^2/c^2 = 1
        Arguments M and center must be selected as:
             [ a^(-2)   0      0     ]
        M =  [   0    b^(-2)   0     ]
             [   0      0    c^(-2)  ]

        center = (0, 0, 0)    
        '''
        # Checking arguments:
        func_name = sys._getframe().f_code.co_name
        genpy.check_type(M, [np.ndarray], __name__, self.__class__.__name__, func_name, 'M', shape = (3,3))
        (landa, V) =  np.linalg.eig(M)
        # V is a rotation that transforms from principal coordinate system to the actual coordinate system
        # if x is a vector in principal CS and x' is its representation in actual CS: x' = V*x and x = V^T*x'
        assert vm.positive(landa, non_zero = True) and gen.equal(np.linalg.norm(V.imag) , 0.0), genpy.err_str(__name__, self.__class__.__name__, func_name, 'Matrix M must have real and positive eigen values')
        #
        self.M      = M
        self.center = center
        self.a      = math.sqrt(1.0/landa[0])
        self.b      = math.sqrt(1.0/landa[1])
        self.c      = math.sqrt(1.0/landa[2])
        self.R      = V.real
        
    def volume(self):
        return (4.0/3.0)*math.pi*self.a*self.b*self.c

    def min_dist(self, point):
        p0 = np.dot(self.R, point.cartesian() - self.center['cartesian'])
        (theta, phi) = opt.distance_from_ellipsoid(a  = self.a, b = self.b, c = self.c, x0 = p0[0], y0 = p0[1], z0 = p0[2], silent = False)
        px  = self.a*math.cos(theta)*math.cos(phi)
        py  = self.b*math.sin(theta)*math.cos(phi)
        pz  = self.c*math.sin(phi)
        pp  = np.array([px, py, pz])
        return Point_3D(pos = np.dot(self.R, pp) + self.center['cartesian'])
        
    def max_dist(self, point):
        p0 = np.dot(self.R, point.cartesian() - self.center['cartesian'])
        (theta, phi) = opt.distance_from_ellipsoid(a  = self.a, b = self.b, c = self.c, x0 = p0[0], y0 = p0[1], z0 = p0[2], silent = False, maximum = True)
        px  = self.a*math.cos(theta)*math.cos(phi)
        py  = self.b*math.sin(theta)*math.cos(phi)
        pz  = self.c*math.sin(phi)
        pp  = np.array([px, py, pz])
        return Point_3D(pos = np.dot(self.R, pp) + self.center['cartesian'])

    def possess(self, point):
        x_c = point.cartesian() - self.center.cartesian()
        d   = np.dot(np.dot((x_c).T, self.M), x_c)
        return gen.equal(d, 1.0)
