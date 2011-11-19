# HEADER
'''   
@file:          transfer_matrices.py
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

from packages.nima.mathematics import rotation

class Transfer_Matrices:
    '''
    This class contains a list of relative and absolute transfer matrices which represent the forward kinematics of a manipulator. 
    Method "update" calculates each transfer matrix according to the given joint configuration and replaces old "T" and "H" values with new.
    '''

    def __init__(self, njoint):
        '''
        '''
        '''
        T  : A list of transformation matrices. T[i] represents i-1 to i transformation matrix. Elements of the list, are 4 X 4 matrices.
        '''
        self.T      = [numpy.eye(4) for i in range(0,njoint) ]
        
        '''
        H  : A list of transformation matrices. H[i] represents ground (-1) to i transformation matrix. Elements of the list, are 4 X 4 matrices.
        '''
        self.H      = [numpy.eye(4) for i in range(0,njoint) ]


    def update(self, geo, config):	
        '''
        This function calculates the transformation matrices (H[i] and T[i]) which represent the
        position and orientation of i-th link of the model.
	    '''

        if config.settings.prismatic[0]:
            self.T[0] = rotation.DH_transfer_matrix(geo.theta[0], geo.alpha[0], geo.a[0], geo.d[0] + config.q[0])
        else:
            self.T[0] = rotation.DH_transfer_matrix(geo.theta[0] + config.q[0], geo.alpha[0], geo.a[0], geo.d[0])

        self.H[0] = self.T[0]

        for i in range(1, config.settings.njoint):

            if config.settings.prismatic[i]:
                self.T[i] = rotation.DH_transfer_matrix(geo.theta[i], geo.alpha[i], geo.a[i], geo.d[i] + config.q[i])
            else:
                self.T[i] = rotation.DH_transfer_matrix(geo.theta[i] + config.q[i], geo.alpha[i], geo.a[i], geo.d[i])

            self.H[i] = numpy.dot(self.H[i-1], self.T[i])
