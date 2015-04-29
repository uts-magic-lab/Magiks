## @file:           task_point.py
#  @brief:    	    This module provides a class representing the reference poisition of an endeffector and the desired value for it.
#  @author      	Nima Ramezani Taghiabadi 
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. :   04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	4.0
# 
#  Last Revision:  	11 January 2015

import numpy, math

from packages.nima.robotics.kinematics.jacobian import jacobian as jaclib
from packages.nima.mathematics.algebra import vectors_and_matrices as vecmat
from packages.nima.mathematics.geometry import metric, trajectory as trajlib 

class Task_Point() : 
    '''
    refernce point only for introducing an endeffector position 
    
    reference_position == Reference Position + Target Point for this reference position + actual position for this point
    
    "reference_position" is a class or data structure containing properties by which a point in the task space of a chained link manipulator is defined.
   
    TASK POSITION is RELATIVE to the "FRAME" of the link, 
        the "FRAME" of the link is determined via DH parameters and the zero configuration  
    '''
    
    def __init__(self, config_settings, link_point_list):
        '''
        link_point_list == WEIGHTING 
        
        Create and define the default values for class properties
        '''
        self.config_settings   = config_settings

        # "lp" is a list of link points. The position of reference position is defined by a linear combination of the listed link points.
        self.lp    = link_point_list

        self.clear_error()
        # "rd" represents the desired position for the reference position with respect to the ground coordinate system
        self.rd    = None

        self.H     = None

        self.geometric_jacobian = jaclib.Geometric_Jacobian(self.config_settings)

        self.error_jacobian = jaclib.Error_Jacobian(self.config_settings)

        # "error" is an instance of class "Position_Metric" which represents the error between the desired and actual positions of the reference point
        self.error = metric.Position_Metric()

    def clear_error():
        self.error.clear()
        self.error_jacobian.clear()

    def clear_position():
        # "ra" represents the position vector of the reference position with respect to the ground coordinate system.
        self.ra    = None
        self.geometric_jacobian.clear()
        self.clear_error()        

    def __str__(self):
        s  =  "        Actual Position (mm):                   " +  vecmat.vector_to_str(1000*(self.position())) + "\n"
        s +=  "        Desired Position (mm):                  " +  vecmat.vector_to_str(1000*(self.rd)) + "\n"
        s +=  "        Position Error   (mm):                  " +  vecmat.vector_to_str(1000*(self.error.value)) + "\n"
        return(s)

    def set_target(self, rd):
        assert len(rd) == 3
        self.rd = rd
        self.clear_error()

    def set_transfer_matrices(self, H):
        len_H = len(H)
        assert len_H == self.config_settings.DOF, genpy.err_str(__name__, self.__class__.__name__,'set_transfer_matrices', 'The number of given transfer_matrices is: '+str(len_H)+' which does not match the settings DOF ('+str(self.config_settings.DOF)+')')
        self.H                        = H
        self.clear_position()

    def set_analytic_jacobian(self, analytic_jacobian):
        self.analytic_jacobian                = analytic_jacobian
        self.geometric_jacobian.clear()
        self.error_jacobian.clear()
        
    def position(self):
        if self.ra == None:
            self.ra = numpy.zeros((3))
            for j in range(0, len(self.lp)):
                x = numpy.dot(self.H[self.lp[j].ln], vecmat.extend_vector(self.lp[j].pv))
                self.ra = self.ra + self.lp[j].w * x[0:3]
        return self.ra
        
