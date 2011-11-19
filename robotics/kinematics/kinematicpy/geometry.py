# HEADER
'''   
@file:          geometry.py
@brief:    	    This module provides a class representing the geometry of a manipulator. The geometry is represented via DH parameters.
@author:        Nima Ramezani Taghiabadi
                PhD Researcher
                Faculty of Engineering and Information Technology
                University of Technology Sydney
                Broadway, Ultimo, NSW 2007
                Room No.: CB10.03.512
                Phone:    02 9514 4621
                Mobile:   04 5027 4611
                Email(1): Nima.RamezaniTaghiabadi@student.uts.edu.au 
                Email(2): nima.ramezani@gmail.com
                Email(3): nima_ramezani@yahoo.com
                Email(4): ramezanitn@alum.sharif.edu

@version:	    0.2
Last Revision:  23 October 2012
'''

import numpy 

## introduce the init mechanism like we did for the example in the test folder 


class Manipulator_Geometry :

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
