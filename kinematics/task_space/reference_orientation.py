'''   
@file:          reference_orientation.py
@brief:    	    This module provides a class representing the reference orientation of an endeffector and the desired value for it.
@author:        Nima Ramezani; DFKI Bremen
@start date:    February 2010
@version:	    0.2
Last Revision:  27 June 2011
'''

# HEADER
import numpy, math, metric

from packages.nima.mathematics import vectors_and_matrices as vecmat
import packages.nima.robotics.kinematics.jacobian.jacobian as jaclib 
import trajectory as trajlib 

class Reference_Orientation : 
    '''
    old name: Task_Frame : 
    
    ## link_number is NOT part of this class / element but of the SET that holds / manages these classes/elements!
    ##   -> link_number DOES NOT determine the NATURE of a Reference_Orientation
    
    This class is used to introduce a reference orientation for the endeffector. The reference orientation is the orientation of link which is specified by: "link_number"
    '''

    def __init__(self, link_number, default_error_function = 'Axis Inner Product'):
        '''
        '''
        self.ln     = link_number

        # ra is the 3 X 3 rotation matrix representing the actual orientation of the frame respect to the ground coordinate system (Absolute Orientation).
        self.ra     = numpy.eye(3)
        
        # rd represents the desired orientation vector of the reference orientation
        self.rd    = numpy.eye(3)

        self.error = metric.Orientation_Metric(default_error_function)
        self.desired_trajectory = trajlib.Orientation_Trajectory()


    def __str__(self):
        s  =  "        Actual Orientation :"  + "\n" + "\n"  + str(vecmat.format_matrix(self.ra)) + "\n" + "\n"  
        s +=  "        Desired Orientation:"  + "\n" + "\n"  +  str(vecmat.format_matrix(self.rd)) + "\n" + "\n" 
        s +=  "        Orientation Error :                     "  +  str(vecmat.format_vector(self.error.value)) + "\n"
        return(s)

    def set_target_from_trajectory(self, phi = 0.0):
        self.desired_trajectory.set_phi(phi)
        self.rd = numpy.dot(self.target_offset, self.desired_trajectory.current_orientation)
        self.vd = self.desired_trajectory.current_orientation_rate

    def update_orientation(self, tm):
        '''
        update the actual (current) orientation of the reference orientation. Change property "ra"
        '''
        # Extracting the orientation (Rotation Matrix) of each reference orientation from its corresponding transformation matrix
        self.ra = vecmat.extract_rotation_matrix(tm.H[self.ln])

    def update_geometric_jacobian(self, fwd_kin):
        '''
        update the actual (current) geometric jacobian of the reference orientation. Change property "geometric_jacobian" 
        require "fwd_kin" which contains kinematic data of the manipulator (including "configuration", "transfer_matrices" and "analytic_jacobian")
        '''
        self.geometric_jacobian = jaclib.Geometric_Jacobian(fwd_kin.configuration.DOF)
        self.geometric_jacobian.update_for_orientation(self, fwd_kin)

    def update_error_jacobian(self, fwd_kin):
        '''
        update the actual (current) error jacobian of the reference orientation. Change property "geometric_jacobian" 
        require "fwd_kin" which contains kinematic data of the manipulator (including "configuration", "transfer_matrices" and "analytic_jacobian")
        '''
        self.error_jacobian = jaclib.Error_Jacobian(fwd_kin.configuration.DOF)
        self.error_jacobian.update_for_orientation(self, fwd_kin.configuration, fwd_kin.analytic_jacobian)
                    
    def update_task_jacobian(self):
        '''
        update the task jacobian of the reference orientation. 
        '''
        self.task_jacobian = jaclib.Task_Jacobian()
        self.task_jacobian.update_for_orientation(self)


