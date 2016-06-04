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
@version:	    3.0
Last Revision:  11 August 2015
'''
'''
Changes from previous version:
    1- function set_config_virtual() added

'''

# BODY

import numpy, math


from magiks.jacobian import jacobian as jaclib

from magiks.jointspace import manipulator_configuration
from math_tools.geometry import rotation

## @brief This class contains geometrical properties and dimensional settings of a chained-link manipulator.
#  These settings mainly include the standard DH parameters by which the geometry of the manipulator is specified.          
#  These values are based on <em> Denavit Hartenberg </em>  standrad representation.
#  The transformation matrices are calculated based on this standard.
#  If the joint is revoulte, the value of <tt> theta[i] </tt>  is added to <tt> q[i] </tt> in the transformation matrix
#  If the joint is prismatic, the value of <tt> d[i] </tt> is added by <tt> q[i] </tt> in the transformation matrix
class Manipulator_Geometry_Settings(object):
	## The Class Constructor:
	#  @param nlink A positive integer specifying the number of links of the manipulator 	
	#  @param manip_name A character string specifying the name of a manipulator. Examples: \b PR2ARM or \b PA10
    def __init__(self, nlink, manip_name):
        self.name = manip_name
		##  A positive integer containing the number of links of the manipulator
        self.nlink = nlink

		##  A numpy vector containing the \f$ \theta \f$ values of the DH parameters
        #   If the proximal joint of the link is revoulte, the value of <tt> theta[i] </tt> is added to <tt> q[i] </tt> in the transformation matrix
        self.theta      = numpy.zeros((nlink))
        
		##  A numpy vector containing the \f$ \alpha \f$ values of the DH parameters
        self.alpha      = numpy.zeros((nlink))
        
		##  A numpy vector containing the \f$ a \f$ values of the DH parameters
        self.a          = numpy.zeros((nlink))
        
		##  A numpy vector containing the \f$ d \f$ values of the DH parameters
        #   If the joint is prismatic, the value of <tt> d[i] </tt> is added to <tt> q[i] </tt> in the transformation matrix
        self.d          = numpy.zeros((nlink))

## @brief This class contains a list of relative and absolute transfer matrices representing the forward kinematics of a manipulator.
#
class Manipulator_Geometry(manipulator_configuration.Manipulator_Configuration):

	## The Class Constructor:
	#  @param config_settings An instance of class 
    #         \link magiks.jointspace.manipulator_configuration.Manipulator_Configuration_Settings Manipulator_Configuration_Settings \endlink 
    #         containing the configuration settings of the manipulator
	#  @param geo_settings An instance of class Manipulator_Geometry_Settings containing the geometric settings of the manipulator
    def __init__(self, config_settings, geo_settings):
        super(Manipulator_Geometry, self).__init__(config_settings)
        
        ## An instance of class Manipulator_Geometry_Settings
        #  containing geometry and dimensions of the manipulator in terms of DH parameters
        self.geo_settings      = geo_settings

        ## An instance of class 
        #  \link magiks.jacobian.Analytic_Jacobian Analytic_Jacobian \endlink.
        #  This class contains a two-dimensional list of derivatives of absolute homogeneous transfer matrices with respect to each of the joints
        self.ajac  = jaclib.Analytic_Jacobian(self.config_settings)

        ## A list of homogeneous transformation matrices. 
        #  <tt> T[i] </tt> represents <em> i-1 </em> to \a i transformation matrix. Elements of the list, are <tt> 4 X 4 </tt> matrices.
        self.T = None

        ## A list of transformation matrices. <tt> H[i] </tt> represents ground (-1) to i transformation matrix. Elements of the list, are <tt> 4 X 4 </tt> matrices.
        self.H = None
        
    ## This function is used to generate a string representing a set of selected object properties and their values.
    #  If the set of displayed parameters is not specified, property <tt> str_parameter_set </tt> will be used.
    #  @param parameter_set A set of parameters to be displayed in the generated string in abbreviation form
    #  @return A character string containing the object properties specified by argument <tt> parameter_set </tt>  with their values   
    def __str__( self, parameter_set = None ) : 
        if parameter_set == None:
            parameter_set = self.str_parameter_set
        if parameter_set == []:
            return ''    
        s =   '\n'
        s  += 'Manipulator Configuration:' + '\n' + '\n'
        for p in parameter_set:
            value = self.parameter_value(p)
            param = key_dic[p]
            s += param + " "*(45-len(param)) + ': ' + value + '\n'
        return s
    
    ## Use this function to get the current homogeneous transfer matrices of the manipulator.
    #  Values of the transfer matrices depend on the manipulator configuration.
    #  @return A list of homogeneous <tt> 4 X 4 </tt> matrices. The i-th element of the list represents the position and orientation of the i-th link.
    def transfer_matrices(self):
        if self.H == None:
            geo         = self.geo_settings
            self.T      = [numpy.eye(4) for i in range(0, geo.nlink) ]
            self.H      = [numpy.eye(4) for i in range(0, geo.nlink) ]
            
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

# \cond

    def set_config_virtual(self, qvrd):
        if super(Manipulator_Geometry, self).set_config_virtual(qvrd):
            self.T = None
            self.H = None
            self.ajac.clear()
            return True
        else:
            return False        

    def set_config(self, qd):
        if super(Manipulator_Geometry, self).set_config(qd):
            self.T = None
            self.H = None
            self.ajac.clear()
            return True
        else:
            return False  
            
# \endcond
                  
