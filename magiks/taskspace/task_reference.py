## @file:           task_reference.py
#  @brief:    	    This module provides a class representing the reference poisitions and orientations of an endeffector and the desired value for it.
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
#  Last Revision:  	28 January 2015

import numpy, math, general_python as genpy

from magiks.jacobian import jacobian as jaclib
from math_tools.algebra import vectors_and_matrices as vecmat
from math_tools.geometry import pose_metric, trajectory as trajlib, geometry as geo

class Task_Reference(object):

    def __init__(self, config_settings):
        self.config_settings   = config_settings

        # "rd" represents the desired position for the reference position with respect to the ground coordinate system
        self.rd    = None

        self.geo_jac = jaclib.Geometric_Jacobian(self.config_settings)

        self.err_jac = jaclib.Error_Jacobian(self.config_settings)

        # "error" is an instance of class "Position_Metric" which represents the error between the desired and actual positions of the reference point
        self.error = pose_metric.Metric()

        self.clear()

        self.priority_level = 'Main Task'

    def clear_error(self):
        self.error.clear()
        self.err_jac.clear()

    def clear(self):
        # "ra" represents the position vector of the reference position with respect to the ground coordinate system.
        self.ra    = None
        self.geo_jac.clear()
        self.clear_error()        

    def set_target(self, rd):
        self.rd = rd
        self.clear_error()


class Task_Point(Task_Reference) : 
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
        super(Task_Point, self).__init__(config_settings)

        # "lp" is a list of link points. The position of reference position is defined by a linear combination of the listed link points.
        self.lp    = link_point_list
        self.error = pose_metric.Position_Metric()

    def __str__(self):
        s  =  "        Actual Position (mm):                   " +  vecmat.vector_to_str(1000*(self.ra)) + "\n"
        s +=  "        Desired Position (mm):                  " +  vecmat.vector_to_str(1000*(self.rd)) + "\n"
        s +=  "        Position Error   (mm):                  " +  vecmat.vector_to_str(1000*(self.error.value(self.ra, self.rd))) + "\n"
        return(s)

    def position(self, H):
        if self.ra == None:
            self.ra = numpy.zeros((3))
            for j in range(0, len(self.lp)):
                x = numpy.dot(H[self.lp[j].ln], vecmat.extend_vector(self.lp[j].pv))
                self.ra = self.ra + self.lp[j].w * x[0:3]
        return self.ra
        
    def error_jacobian(self, H, ajac):
        if self.err_jac.value == None:
            self.err_jac.update_for_position(self, ajac)
        return self.err_jac.value

    def geometric_jacobian(self, ajac):
        if self.geo_jac.value == None:
            self.geo_jac.update_for_position(self, ajac)
        return self.geo_jac.value

class Task_Frame(Task_Reference) : 
    '''
    old name: Task_Frame : 
    
    ## link_number is NOT part of this class / element but of the SET that holds / manages these classes/elements!
    ##   -> link_number DOES NOT determine the NATURE of a Reference_Orientation
    
    This class is used to introduce a reference orientation for the endeffector. The reference orientation is the orientation of link which is specified by: "link_number"
    '''

    def __init__(self, config_settings, link_number):

        super(Task_Frame, self).__init__(config_settings)

        self.ln     = link_number

        self.error = pose_metric.Orientation_Metric()

    def __str__(self):
        s  =  "        Actual Orientation :"  + "\n" + "\n"  + str(self.ra) + "\n"  
        s +=  "        Desired Orientation:"  + "\n" + "\n"  +  str(self.rd) + "\n" 
        s +=  "        Orientation Error :                     "  +  vecmat.vector_to_str(self.error.value(self.ra,self.rd)) + "\n"
        return(s)

    def orientation(self, H):
        '''
        update the actual (current) orientation of the reference orientation. Change property "ra"
        '''
        if self.ra == None:
            # Extracting the orientation (Rotation Matrix) of each reference orientation from its corresponding transformation matrix
            self.ra = geo.Orientation_3D(H[self.ln][ 0:3, 0:3 ])
        return self.ra

    def error_jacobian(self, ajac):
        if self.err_jac.value == None:
            self.err_jac.update_for_orientation(self, ajac)
        return self.err_jac.value

    def geometric_jacobian(self, H):
        if self.geo_jac.value == None:
            self.geo_jac.update_for_orientation(self, H)
        return self.geo_jac.value
            
