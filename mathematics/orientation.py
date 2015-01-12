## @file        	orientation.py
#  @brief     		This module contains a comprehensive data structure with relevant method and properties
#                   for a rotation or orinetation in 3D space.
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
#  start date:      07 January 2015 
#  Last Revision:  	07 January 2015 

import math, numpy as np, general as gen, quaternions, vectors_and_matrices as vm, trigonometry as trig
import rotation as rot

import packages.nima.general
# from cgkit.cgtypes import quat, mat3, vec3

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

class Orientation(object):

    def clear_speed(self):    
        ## A numpy vector of size 3 containing the <em> angular velocity </em> representation of orientation speed
        self.omega = None

        ## A numpy 3x3 rotation matrix: Representation of orientation speed as 3x3 numpy rotation matrix derivative w.r.t. time
        self.Rd = None

        ## A numpy vector of size 4: Quaternion representation of orientation speed as a numpy vector (\f$ \frac{ dq }{ dt } \f$)
        self.Qd = None

        ## A float containing \f$ \dot{\phi} \f$ (derivative of \f$ \phi \f$ w.r.t. time), 
        #  where \f$ \phi \f$ is the \b angle in \em Angle-Axis representation of orientation
        self.phid = None

        ## A numpy vector of size 3 containing \f$ \dot{\u} \f$ (derivative of unit vector \f$ u \f$ w.r.t. time) 
        #  where \f$ u \f$ is the \b axis in \em Angle-Axis representation of orientation
        self.ud = None

        ## A numpy vector of size 3 containing \f$ \dot{\p} \f$ (derivative of orientation vector \f$ p \f$ w.r.t. time) 
        #  where \f$ p \f$ is the vectorial representation of orientation which is defined according to property \b self.generating_function
        self.pd = None

    def clear(self):
        ## A numpy 3x3 rotation matrix: Representation of orientation as 3x3 numpy rotation matrix 
        self.R = None

        ## An instance of type cgkit.cgtypes.Mat3(): Representation of orientation as 3x3 rotation matrix 
        self.R_cgt = None

        ## A numpy vector of size 4: Quaternion representation of orientation as a numpy vector
        self.Q = None

        ## An instance of type cgkit.cgtypes.quat(): Quaternion representation of orientation
        self.Q_cgt = None

        ## A numpy vector of size 3: Representation of orientation as a non-redundant numpy vector (depends on the generator function)
        self.p = None

        ## An instance of type cgkit.cgtypes.vec3(): Representation of orientation as a non-redundant vector
        self.p_cgt = None

        ## A float: Angle phi in Angle-Axis representation of orientation
        self.phi = None

        ## A numpy vector of size 3: Unit vector u in Angle-Axis representation of orientation
        self.u = None

        ## An instance of type cgkit.cgtypes.vec3(): Unit vector u in Angle-Axis representation of orientation
        self.u_cgt = None

        self.H = None

        ## An instance of type cgkit.cgtypes.vec3(): Representation of orientation by three Euler-Kardan angles 
        self.exyz_cgt = None

        ## A numpy vector of 3 elements: Non-redundant representation of orientation by three <em> Spherical Orientation Angles </em>
        self.sa = None

        
    ## Class Constructor:
    #  @param The orientation:
    #         \li A tuple or numpy vector of 4 elements if representation = 'quaternion',    
    def __init__(self, ori, ori_speed = None, representation = 'matrix'):
        self.clear_speed()
        self.representation = representation
        self.__setitem__(representation, ori)
        self.generating_function = 'm*sin(09phi/m)'
        self.m = 1.0
        if ori_speed == None:
            self.clear_speed()
        else:
            self.set_speed(ori_speed, representation)

    def __str__(self):
        
        s =  "3D Orientation represented by " + self.representation + ": \n \n" 
        s += str(self[self.representation])
        s += "\n"
        return s

    def matrix_speed(self):
        if self.Rd != None:
            return self.Rd
        elif self.omega != None:
            r  = self.matrix()
            wx = rotation.skew(w)
            self.Rd = np.dot(wx, r)
            return self.matrix_speed()
        elif (self.phid != None) and (self.ud != None):
            phi = self.angle()
            u   = self.axis()
            ux  = rotation.skew(u)
            self.omega = self.phid*u + np.dot(math.sin(phi)*np.eye(3) + (1.0 - math.cos(phi))*ux, ud)
            '''
            Reference: Olivier A. Bauchau et.al, "The Vectorial Parameterization of Rotation", Nonlinear Dynamics, 32, No 1, pp 71 - 92, 2003 (EQ.10)
            '''    
            return self.matrix_speed()
        else:
            return None

    def angular_velocity(self):
        if self.omega == None:
            rd = self.matrix_speed()
            r  = self.matrix()
            # Ref [1]: Equation 9
            self.omega  = rotation.axial(np.dot(rd, r.T))
        return self.omega

    def quaternion_speed(self):
        if self.Qd == None:
            '''
            r  = self.matrix()
            rd = self.matrix_speed()
            self.Qd = quaternions.unit_quaternion_speed(r, rd)
            '''
            phi  = self.angle()
            phid = self.angle_speed()
            u    = self.axis()
            ud   = self.axis_speed()
            s    = math.sin(phi/2)
            c    = math.cos(phi/2)
            wd   = - 0.5*s*phid
            ed   =   0.5*c*phid*u + s*ud
            self.Qd = np.array([wd, ed[0], ed[1], ed[2]])
        return self.Qd

    def angle_speed(self):
        if self.phid == None:
            phi     = self.angle()
            rd      = self.matrix_speed()
            sin_phi = math.sin(phi)
            self.phid = np.trace(rd)/(-2*sin_phi)
        return self.phid

    def axis_speed(self):
        if self.ud == None:
            u       = self.axis()
            phi     = self.angle()
            sin_phi = math.sin(phi)
            cos_phi = math.cos(phi)
            phid    = self.angle_speed()             
            rd      = self.matrix_speed()
            axrd    = rot.axial(rd)
            
            self.ud = (axrd - cos_phi*phid*u)/sin_phi
        return self.ud
    
    def matrix(self):
        if self.R != None:
            return self.R

        elif self.Q != None:
            self.Q_cgt = quat(self.Q)
            return self.matrix()

        elif (self.phi != None) and (self.u != None):
            ux = rotation.skew(self.u)
            self.R = np.eye(3) + math.sin(self.phi)*ux + (1.0 - math.cos(self.phi))*np.dot(ux,ux)
            return self.R

        elif self.p != None:
            px  = rot.skew(p)
            px2 = np.dot(px, px)
            if self.generating_function == 'sin(phi/m)':
                if self.m == 1.0: # Special Case: Linear parametrization
                    c    = math.cos(phi/2)
                    v    = gen.inv(c)
                    v2   = v*v
                    v2_2 = 0.5*v2
                    v2_e = 1.0
                else:
                    assert False, "Not supported!"
            else:
                assert False,"Not Supported!"
            self.R = np.eye(3) + v2_e*px + v2_2*px2
            return self.R

        elif (self.phi != None) and (self.u_cgt != None):
            self.Q = quat()
            self.Q.fromAngleAxis(self.phi, self.u_cgt)
            return self.matrix()

        elif self.R_cgt != None:
            self.R = np.array(self.R_cgt)
            return self.R

        elif self.Q_cgt != None:
            self.R_cgt = self.Q_cgt.toMat3()
            return self.matrix()

        else:
            return None

    def vector(self):
        if self.p == None:
            if (self.generating_function == 'm*sin(phi/m)') and (self.m == 1.0): # Special Case: Linear parametrization
                self.p = rot.axial(self.matrix())
                return self.p
            elif self.generating_function == 'phi/m': # Identity parametrization if m = 1
                f = self.angle()/self.m
            elif self.generating_function == 'm*tan(phi/m)': # Cayley-Gibbs-Rodrigues parameterization if m = 2
                f = self.m*math.tan(self.angle()/self.m)
            elif self.generating_function == '(6*(phi - sin(phi)))**(1.0/3.0)':  # Bauchau-Trainelli parametrization
                phi = self.angle()
                s   = math.sin(phi)
                f   = (6*(phi - s))**(1.0/3.0)
            elif self.generating_function == 'm*exp(phi/m)': # Exponential
                phi = self.angle()
                e   = math.exp(phi)
                f   = e - 1.0
            else:
                assert False,"Not Supported!"
            self.p = f*self.axis()
        return self.p    
            
    def vector_speed(self):
        if self.pd == None:
            if (self.generating_function == 'm*sin(phi/m)') and (self.m == 1.0): # Special Case: Linear parametrization
                self.pd = rot.axial(self.matrix_speed())
                return self.pd
            elif self.generating_function == 'phi/m': # Identity parametrization if m = 1
                f  = self.angle()/self.m
                fp = 1.0/self.m
            elif self.generating_function == 'm*tan(phi/m)': # Cayley-Gibbs-Rodrigues parameterization if m = 2
                t  = math.tan(self.angle()/self.m)
                f  = self.m*t
                fp = 1.0 + t*t
            elif self.generating_function == '(6*(phi - sin(phi)))**(1.0/3.0)':  # Bauchau-Trainelli parametrization
                phi = self.angle()
                s   = math.sin(phi)
                c   = math.cos(phi)
                f   = (6*(phi - s))**(1.0/3.0)
                a   = gen.inv(6*(phi - s))
                fp  = 2*(1.0-c)*(a**(2.0/3.0))
            elif self.generating_function == 'm*exp(phi/m)': # Exponential
                phi = self.angle()
                e   = math.exp(phi)
                f   = e - 1.0
                fp  = e
            else:
                assert False,"Not Supported!"
            self.pd = fp*self.angle_speed()*self.axis()+f*self.axis_speed()
        return self.pd

    def set_generating_function(self, gen_fun_str = 'phi/m'):
        if gen_fun_str != self.generating_function:
            self.generating_function = gen_fun_str
            self.p  = None
            self.pd = None

    def matrix_cgt(self):
        if self.R_cgt == None:
            self.R_cgt =  mat3(tuple(self.matrix()))
        return self.R_cgt

    def quaternion_cgt(self):
        if self.Q_cgt == None:
            Q = quat()
            Q.fromMat(self.matrix_cgt())
            self.Q_cgt = Q
        return self.Q_cgt

    def quaternion(self):
        if self.Q == None:
            """
            r       = self.matrix()
            self.Q  = quaternions.unit_quaternion(r)
            print "1.", self.Q
            """
            phi = self.angle()
            u   = self.axis()
            e   = math.sin(phi/2)*u
            w   = math.cos(phi/2)    
            self.Q  = np.array([w, e[0], e[1], e[2]])

            # Q = self.quaternion_cgt()
            # self.Q = np.array([Q.w, Q.x, Q.y, Q.z])
        return self.Q
            
    def angle(self):
        if self.phi == None:
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

    def axis_cgt(self):
        if self.u == None:
            (self.phi, self.u_cgt) = self.quaternion_cgt().toAngleAxis() 
        return self.u_cgt

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
            '''
            # Alternative 1:
            r  = self.matrix()
            ax = rot.angle_axis(r)
            self.phi = ax[0]
            self.u   = ax[1:4]
            print "1.",self.u 
            '''
            # Alternative 2:
            sin_phi = math.sin(self.angle())
            self.u   = rot.axial(self.matrix())/sin_phi
            # Alternative 3:
            # self.u = np.array(self.axis_cgt())
        return self.u

    def frame_axis(self, k):
        r = self.matrix()
        return r[0:3, k]

    def __setitem__(self, representation, ori):

        self.clear()
        if representation == 'matrix':
            self.R = ori
        elif representation == 'quaternion':
            self.Q = ori
        elif representation == 'angle_axis':
            self.phi = ori[0]
            self.u   = ori[1:4]
        elif representation == 'vector':
            self.p = ori
        elif representation == 'matrix_cgt':
            self.R_cgt = ori
        elif representation == 'quaternion_cgt':
            self.Q_cgt = ori
        elif representation == 'angle_axis_cgt':
            self.phi   = ori[0]
            self.u_cgt = vec3(ori[1:4])
        else:
            assert False, "Not supported Yet !"

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
        elif representation == 'matrix_speed':
            return self.matrix_speed()
        elif representation == 'quaternion_speed':
            return self.quaternion_speed()
        elif representation == 'angle_axis_speed':
            return self.angle_axis_speed()
        elif representation == 'angle_speed':
            return self.angle_speed()
        elif representation == 'axis_speed':
            return self.axis_speed()
        elif representation == 'vector_speed':
            return self.vector_speed()
        elif representation == 'trace_speed':
            return np.trace(self.matrix_speed())
        elif representation == 'diag_speed':
            return np.diag(self.matrix_speed())
        elif representation == 'matrix_cgt':
            return self.matrix_cgt()
        elif representation == 'quaternion_cgt':
            return self.quaternion_cgt()
        elif representation == 'angle_axis_cgt':
            return self.angle_axis_cgt()
        else:
            assert False, "Not supported Yet !"

    def set_speed(self, ori, representation = None):
        self.clear_speed()
        if representation == None:
            representation = self.representation
        if representation == 'quaternion':
            self.Qd = ori
        elif representation == 'matrix':
            self.Rd = ori
        elif representation == 'angle_axis':
            self.phid = ori[0]
            self.ud   = ori[1:4]
        else:
            assert False, "Not supported Yet !"
        
    def get_speed(self, representation = None):
        if representation == None:
            representation = self.representation

        if representation == 'quaternion':
            return self.quaternion_speed()
        elif representation == 'matrix':
            return self.matrix_speed()
        elif representation == 'angle_axis':
            return self.angle_axis_speed()
        elif representation == 'angular_spherical':
            assert False
            return self.spherical_speed()
        else:
            assert False, "Not supported Yet !"
    
    def __div__(o1, o2):
        r1 = o1.matrix()
        r2 = o2.matrix()
        r1_dot = o1.matrix_speed()
        r2_dot = o2.matrix_speed()
        r      = np.dot(r1, r2.T)
        if (r1_dot == None) or (r2_dot == None):
            o = Orientation(r)
        else:
            r_dot = np.dot(r1_dot, r2.T) + np.dot(r1, r2_dot.T)
            o = Orientation(r, r_dot)

        if o1.generating_function == o2.generating_function:
            o.set_generating_function(o1.generating_function)
        else:
            o.generating_function = None
        o.representation = o1.representation    
        return o
            
    def __mul__(o1, o2):
        r1 = o1.matrix()
        r2 = o2.matrix()
        r1_dot = o1.matrix_speed()
        r2_dot = o2.matrix_speed()
        r      = np.dot(r1, r2)
        if (r1_dot == None) or (r2_dot == None):
            o = Orientation(r)
        else:
            r_dot = np.dot(r1_dot, r2) + np.dot(r1, r2_dot)
            o = Orientation(r, r_dot)
            
        if o1.generating_function == o2.generating_function:
            o.set_generating_function(o1.generating_function)
        else:
            o.generating_function = None
        o.representation = o1.representation    
        return o
            
            
