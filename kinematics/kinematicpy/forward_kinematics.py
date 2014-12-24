# HEADER
'''   
@file:          forward_kinematics.py
@brief:    	    This module provides a functor class regaring the forward kinematic calculations of a general manipulator.
                It contains the positions and orientations of chained links of a manipulator and their partial derivatives in respect with joints. 
                Includes list of relative and absolute transformation matrices and their partial derivatives in respect with joint parameters 
                (which is known as the standrad analytic Jacobian) and a method for calculating and updating them according to the given geometry 
                and joint configuration.
@author:        Nima Ramezani Taghiabadi
                PhD Researcher
                Faculty of Engineering and Information Technology
                University of Technology Sydney (UTS)
                Broadway, Ultimo, NSW 2007, Australia
                Room No.: CB10.03.512
                Phone:    02 9514 4621
                Mobile:   04 5027 4611
                Email(1): Nima.RamezaniTaghiabadi@student.uts.edu.au 
                Email(2): nima.ramezani@gmail.com
                Email(3): nima_ramezani@yahoo.com
                Email(4): ramezanitn@alum.sharif.edu

@version:	    1.4
Last Revision:  23 October 2012
'''
# BODY

import numpy, math, copy

import packages.nima.robotics.kinematics.task_space.transfer_matrices as tmlib 
import packages.nima.robotics.kinematics.jacobian.jacobian as jaclib 

class Forward_Kinematics(object):
    '''
    This class contains all kinematic properties of a manipulator which can be directly calculated from the joint configuration. 
    These properties are:
    
        1- Absolute and relative transfer matrices of the links
        2- The standard analytic jacobian (derivatives of absolute transfer matrices in respect with joints
        3- The geometric Jacobian
        4- The error Jacobian (Jacobian of the pose residual functions) 
        5- The pose residual vector 
        
    Method "update" calculates and update all these properties according to the given joint configuration.
    '''
    def __init__(self, manip_geometry, manip_configuration):
        '''
        '''
        assert manip_geometry.nlink == manip_configuration.settings.njoint
        
        self.geometry           = copy.deepcopy(manip_geometry)
        
        # "self.configuration" is an instance of class "Joint_Configuration" in package "joint_space" which contains the joint configuration of the manipulator and the corresponding methods
        self.configuration      = copy.deepcopy(manip_configuration)
        
        # "self.transfer_matrices" is an instance of class "Transfer_Matrices" in package "forward_kinematics" a sub-package of "task_space" 
        # This class contains the relative and absolute transfer matrices of each link of a manipulator
        self.transfer_matrices  = tmlib.Transfer_Matrices(self.geometry.nlink)    
        
        # "self.analytic_jacobian" is an instance of class "Analytic_Jacobian" in package "jacobian"
        # This class contains a two-dimensional list of matrices which are the derivatives of absolute transfer matrices in respect with each of the joint parameters
        self.analytic_jacobian  = jaclib.Analytic_Jacobian(self.geometry.nlink)

        # self.tasklist, determines the properties required to be calculated and updated when "update" method is implemented.
        self.tasklist       = ['update_transfer_matrices',
                               'update_analytic_jacobian']
    
    def forward_update(self):
        '''
        This function calculates the transformation matrices (H[i] and T[i]) which represent the
        absolute and relative poses of i-th link of the model.
        Absolute pose: Position and orientation in respect with ground (link -1)
        Relative pose: Position and orientation in respect with the previous link (link i-1)

        It uses the joint values in property(attribute) "q" and updates the model posture(configuration)
        by calculating and changing the values of all other properties.         
        '''

        #self.configuration.initialize()
        #self.configuration.q_to_qstar()
        
        self.configuration.update_joint_limit_jacobian_multipliers()
        
        if 'update_transfer_matrices' in self.tasklist:        
            self.transfer_matrices.update(self.geometry, self.configuration)

        if 'update_analytic_jacobian' in self.tasklist:
            self.analytic_jacobian.update(self.transfer_matrices, self.configuration)

    def take_to_grid_configuration(self, config_number, number_of_intervals):
        '''
        Replaces joint configuration with the configuration in a jointspace lattice with "number_of_intervals" divisions for each joint 
        The order of the configuration in the jointspace lattice is identified by "config_number",
        Other kinematic properties of the manipulator will be synchronized with this change
        '''
        self.configuration.change_to_grid_configuration(config_number, number_of_intervals)
        self.forward_update()

    def take_to_random_configuration(self):
        '''
        Replaces joint configuration with random values in their feasible range.
        Other kinematic properties of the manipulator will be updated with this change
        '''
        self.configuration.generate_random()
        self.forward_update()
