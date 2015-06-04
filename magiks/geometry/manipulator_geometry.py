# HEADER
'''   
@file:          manipulator_geometry.py
@brief:    	    This module provides a class representing the positions and orientations of each of the links of a manipulator. 
                Includes list of relative and absolute transformation matrices and a method for calculating them from given configuration.
@author:        Nima Ramezani Taghiabadi
                PhD Researcher
                Faculty of Engineering and Information Technology
                University of Technology Sydney (UTS)
                Broadway, Ultimo, NSW 2007, Australia
                Mobile:   04 5027 4611
                Email(1): Nima.RamezaniTaghiabadi@uts.edu.au 
                Email(2): nima.ramezani@gmail.com
                Email(3): nima_ramezani@yahoo.com
                Email(4): ramezanitn@alum.sharif.edu
@start date:    February 2010
@version:	    2.0
Last Revision:  11 December 2014
'''

# BODY

import numpy, math


from magiks.jacobian import jacobian as jaclib

from magiks.jointspace import manipulator_configuration as configlib
from math_tools.geometry import rotation

class Manipulator_Geometry_Settings(object):

    '''
    Link-Segment self is a chain of joints and links which can be used as a mathematical self of a robot, mechanical manipulator,
    the whole or parts of human or animal moving mechanism

    TODO : erase njoints and then (compare again the concept of an ABSTRACT super class) 
        self.theta      = numpy.zeros( 0 )
    '''

    def __init__(self, nlink, manip_name):
        '''
        Predefined Class Properties :

        theta, alpha, a and d are vectors of real numbers by which the geometry of the manipulator is defined.
        These values are based on Denavit Hartenberg standrad representation (Please refer to reference 1, pages 36 to 40)
        The transformation matrix is calculated based on this standard.

        If the joint is revoulte, the value of theta[i] is added by q[i] in the transformation matrix
        If the joint is prismatic, the value of d[i] is added by q[i]  in the transformation matrix

        These parameters are only used to define the geometry of the manipulator. The values of these parameters will never change in the module
        '''
        self.name = manip_name
        self.nlink = nlink

        self.theta      = numpy.zeros((nlink))
        self.alpha      = numpy.zeros((nlink))
        
        self.a          = numpy.zeros((nlink))
        self.d          = numpy.zeros((nlink))

class Manipulator_Geometry(configlib.Manipulator_Configuration):
    '''
    This class contains a list of relative and absolute transfer matrices which represent the forward kinematics of a manipulator. 
    Method "update" calculates each transfer matrix according to the given joint configuration and replaces old "T" and "H" values with new.
    '''

    def __init__(self, config_settings, geo_settings):
        '''
        '''
        super(Manipulator_Geometry, self).__init__(config_settings)
        self.geo_settings      = geo_settings

        # "self.analytic_jacobian" is an instance of class "Analytic_Jacobian" in package "jacobian"
        # This class contains a two-dimensional list of matrices which are the derivatives of absolute transfer matrices in respect with each of the joint parameters
        self.ajac  = jaclib.Analytic_Jacobian(self.config_settings)

        '''
        T  : A list of transformation matrices. T[i] represents i-1 to i transformation matrix. Elements of the list, are 4 X 4 matrices.
        '''
        self.T = None

        '''
        H  : A list of transformation matrices. H[i] represents ground (-1) to i transformation matrix. Elements of the list, are 4 X 4 matrices.
        '''
        self.H = None
        
    def transfer_matrices(self):
        if self.H == None:
            geo         = self.geo_settings
            self.T      = [numpy.eye(4) for i in range(0, geo.nlink) ]
            self.H      = [numpy.eye(4) for i in range(0, geo.nlink) ]
            
            '''
            This function calculates the transformation matrices (H[i] and T[i]) which represent the
            position and orientation of i-th link of the model.
            '''
            if self.config_settings.prismatic[0]:
                self.T[0] = rotation.DH_transfer_matrix(geo.theta[0], geo.alpha[0], geo.a[0], geo.d[0] + self.q[0])
            else:
                self.T[0] = rotation.DH_transfer_matrix(geo.theta[0] + self.q[0], geo.alpha[0], geo.a[0], geo.d[0])

            self.H[0] = self.T[0]

            for i in range(1, self.config_settings.njoint):

                if self.config_settings.prismatic[i]:
                    self.T[i] = rotation.DH_transfer_matrix(geo.theta[i], geo.alpha[i], geo.a[i], geo.d[i] + self.q[i])
                else:
                    self.T[i] = rotation.DH_transfer_matrix(geo.theta[i] + self.q[i], geo.alpha[i], geo.a[i], geo.d[i])

                self.H[i] = numpy.dot(self.H[i-1], self.T[i])

            self.ajac.H = self.H
        return self.H

    def set_config(self, qd):
        if super(Manipulator_Geometry, self).set_config(qd):
            self.T = None
            self.H = None
            self.ajac.clear()
            return True
        else:
            return False        
