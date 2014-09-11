# HEADER
'''   
@file:          reference_position.py
@brief:    	    This module provides a class representing the reference poisition of an endeffector and the desired value for it.
@author:        Nima Ramezani Taghiabadi
                PhD Researcher
                Faculty of Engineering and Information Technology
                University of Technology Sydney (UTS)
                Broadway, Ultimo, NSW 2007, Australia
                Room No.: CB11.07.
                Phone:    
                Mobile:   04 5027 4611
                Email(1): Nima.RamezaniTaghiabadi@student.uts.edu.au 
                Email(2): nima.ramezani@gmail.com
                Email(3): nima_ramezani@yahoo.com
                Email(4): ramezanitn@alum.sharif.edu
@version:	    1.0
Last Revision:  12 September 2014

Changes from version 0.3 (previous version):
    Added VTS feature for moving targets
    The reference points now possess a trajectory for the position
'''

# BODY

import numpy
import math
import metric

from packages.nima.mathematics import vectors_and_matrices as vecmat
import packages.nima.robotics.kinematics.jacobian.jacobian as jaclib 
import trajectory as trajlib 


class Reference_Position() : 
    ## Old Name: Task_Point:
    '''
    refernce point only for introducing an endeffector position 
    
    reference_position == Reference Position + Target Point for this reference position + actual position for this point
    
    "reference_position" is a class or data structure containing properties by which a point in the task space of a chained link manipulator is defined.
   
    TASK POSITION is RELATIVE to the "FRAME" of the link, 
        the "FRAME" of the link is determined via DH parameters and the zero configuration  
    '''
    
    def __init__(self, link_point_list):
        '''
        link_point_list == WEIGHTING 
        
        Create and define the default values for class properties
        '''
        # "lp" is a list of link points. The position of reference position is defined by a linear combination of the listed link points.
        self.lp = link_point_list

        # "ra" represents the position vector of the reference position in respect to the ground coordinate system.
        self.ra  = numpy.zeros((3))
        
        # "rd" represents the desired position for the reference position in respect to the ground coordinate system
        self.rd = numpy.zeros((3))

        # "va" represents the current velocity vector of the reference position in respect to the ground coordinate system.
        self.va  = numpy.zeros((3))
        
        # "vd" represents the desired velocity for the reference position in respect to the ground coordinate system
        self.vd = numpy.zeros((3))

        # "error" is an instance of class "Position_Metric" which represents the error between the desired and actual positions of the reference point
        self.error = metric.Position_Metric()

        # This offset is added to the target when target is set from trajectory
        # It is used for relative trajectory setting
        self.target_offset = numpy.zeros(3)

        self.desired_trajectory = trajlib.Polynomial_Trajectory(dimension = 3)

    def __str__(self):
        s  =  "        Actual Position (mm):                   " +  str(vecmat.format_vector(1000*(self.ra))) + "\n"
        s +=  "        Desired Position (mm):                  " +  str(vecmat.format_vector(1000*(self.rd))) + "\n"
        s +=  "        Position Error   (mm):                  " +  str(vecmat.format_vector(1000*(self.error.value))) + "\n"
        return(s)

    def set_target_from_trajectory(self, phi = 0.0):
        self.desired_trajectory.set_phi(phi)
        self.rd = self.target_offset + self.desired_trajectory.current_position
        self.vd = self.desired_trajectory.current_velocity

    def update_position(self, transfer_matrices):
        '''
        Calclates and updates the value of "ra" property which represents the actual position of the reference_position according to the given transfer matrices "transfer_matrices"
        '''
        self.ra = numpy.zeros((3))
        for j in range(0, len(self.lp)):
            x = numpy.dot(transfer_matrices.H[self.lp[j].ln], vecmat.extend_vector(self.lp[j].pv))
            self.ra = self.ra + self.lp[j].w * x[0:3]

    def update_geometric_jacobian(self, config, anal_jacob):
        '''
        Calclates and updates the value of "geometric_jacobian" which represents the actual geometric jacobian 
        of the reference_position according to the given configuration and analytic jacobian.
        '''
        self.geometric_jacobian = jaclib.Geometric_Jacobian(config.DOF)
        self.geometric_jacobian.update_for_position(self, config, anal_jacob)

    def update_error_jacobian(self,config):
        '''
        Calclates and updates the value of "error_jacobian" which represents the actual error jacobian 
        of the reference_position according to the given configuration.
        '''
        self.error_jacobian = jaclib.Error_Jacobian(config.DOF)
        self.error_jacobian.update_for_position(self, config)

    def update_task_jacobian(self):
        '''
        Calclates and updates the value of "task_jacobian" which represents the actual task jacobian matrix
        of the reference_position.
        '''
        self.task_jacobian = jaclib.Task_Jacobian()
        self.task_jacobian.update_for_position(self)

