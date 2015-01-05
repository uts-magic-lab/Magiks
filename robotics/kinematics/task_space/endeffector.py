# HEADER
# HEADER
'''   
@file:          endeffector.py
@brief:    	    This module provides a class containing a set of reference positions and orientations in the taskspace together 
                with their associated poses. 
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
@version:	    2.0
Last Revision:  11 December 2014
'''
# BODY

import numpy, math

import packages.nima.robotics.kinematics.task_space.link_point as lplib 
import packages.nima.robotics.kinematics.task_space.reference_position as tplib 
import packages.nima.robotics.kinematics.task_space.reference_orientation as tflib 

from packages.nima.mathematics import rotation
from packages.nima.mathematics import vectors_and_matrices as vecmat


class Endeffector_Settings:
    '''
    '''
    def __init__(self, default_orientation_error_function = 'Axis Inner Product'):
        self.default_orientation_error_function   = default_orientation_error_function
        self.last_link_number         = 0

        '''
        If there are more than one endeffector, 
        precision settings can be set individually for each "position reference" (Task Point) and "orientation reference" (Task Frame)
        You can change each one after the creation of class instance
        '''    

        self.precision_base_for_position   = "Coordinate Difference"
        self.precision_base_for_rotation   = "Axis Angle"

        self.precision_for_position   = 0.02 # coordinate distance in meters
        self.precision_for_rotation   = 2.0  # axis angle in degrees

class Endeffector:
    '''
    This class contains all kinematic properties of the taskspace of a manipulator.
    These properties are:
    
        1- reference_position: A list of operational points (Position References)
        2- reference_orientation: A list of operational link frames (Orientation References)
        3- The geometric Jacobian
        4- The error Jacobian
        5- The pose error vector and its norm
        
    Method "update" calculates and update all these properties according to the given joint configuration.
    '''
    def __init__(self, settings):

        self.settings = settings
        # Property "reference_positions" : is a list of instances of class Reference_Position
        # Each instance is a "task point" and determines a position reference for the endeffector
        '''
        The following code defines one task point by default for the link_segment model
        This defined task point is the origin of the last link of the model (The defined endeffector by default)
        '''
        
        last_link_origin    = lplib.Link_Point(settings.last_link_number,1,[0,0,0])
        endeffector_position_reference_by_default = tplib.Reference_Position([last_link_origin])

        self.reference_positions        = [endeffector_position_reference_by_default]

        self.reference_positions[0].error.precision      = settings.precision_for_position
        self.reference_positions[0].error.precision_base = settings.precision_base_for_position

        # Property "reference_orientations" : is a list of instances of class Reference_Orientation
        # Each instance is a "task frame" and determines an orientation reference for the endeffector

        '''
        The following code defines one task frame by default for the link_segment model
        This defined task frame is the orientation of the last link of the model (The defined endeffector by default)
        '''
        endeffector_orientation_reference_by_default = tflib.Reference_Orientation(self.settings.last_link_number, default_error_function = self.settings.default_orientation_error_function)

        self.reference_orientations = [endeffector_orientation_reference_by_default]

        self.reference_orientations[0].error.precision      = settings.precision_for_rotation
        self.reference_orientations[0].error.precision_base = settings.precision_base_for_rotation

        # mp -- number of position coordinates (3 by default)
        # mo -- number of orientation coordinates (3 by default) 

        self.mp = 3
        self.mo = 3
                       
        # Property "self.all_reference_positions_in_target" is True if all of the reference_positions are in their desired positions
        self.all_reference_positions_in_target   = False
        # Property "self.all_reference_orientations_in_target" is True if all of the reference_orientations are identical to their desired orientations
        self.all_reference_orientations_in_target   = False
        # Property "self.in_target" is True if all of the reference_positions and reference_orientations are in their desired positions and orientations
        self.in_target  = False
        # self.pose_error is the overall vector containing values of error functions (both position and orientation) 
        # ("self.mp" elements representing position error come first )
        # ("self.mo" elements representing orientation error come next) 
        
        # self.pose is the pose vector. Three elements for each reference_position followed by nine elements for each reference_orientation
        # represent actual positions and orientations of the endeffector respectively in one pose vector
        self.pose       = None 
        # self.pose_error is the error vector
        self.pose_error = None 
        # self.error_norm contains the norm of the error vector
        self.error_norm = 0
        # Matrix of Geometric Jacobian (for both positions and orientations)
        # The first rows represent geometric jacobian for position. Number of rows for position = 3*len(reference_positions)
        # The next  rows represent geometric jacobian for orientation. Number of rows for orientation = 3*len(reference_orientations)
        self.GJ = None 
        # Matrix of Error Jacobian (for both position and orientation errors)
        # (The first "self.mp" rows represent error jacobian for position)
        # (The next  "self.mo" rows represent error jacobian for orientation)
        self.EJ = None  
        # Matrix of Task Jacobian(for both position and orientation errors)
        # each row is for one kinematic constraint(similar to error J.) 
        # and each column is for one element of pose parametrization (3 for position and 9 for orientation)
        self.TJ = None  
        # self.tasklist, determines the properties required to be calculated and updated when "update" method is implemented.
        self.tasklist       = ['update_taskspace_pose',
                               'update_error',
                               'update_geometric_jacobian',
                               'update_error_jacobian']

    def __str__(self):
        s = ''
        for i in range(len(self.reference_positions)):
            s +=  "    Reference Position " + str(i) + ":" + "\n" + "\n"
            #s +=  "-----------------------------" + "\n"
            s +=  str(self.reference_positions[i])
        s +=  "\n"
        for i in range(len(self.reference_orientations)):
            s +=  "    Reference Orientation " + str(i) + ":" + "\n" + "\n"
            #s +=  "-----------------------------" + "\n"
            s +=  str(self.reference_orientations[i])
            
        s +=  "\n"
        s +=  "    Pose Error:                                 "
        s +=  vecmat.vector_to_str(self.pose_error) + "\n"
        s +=  "    Norm of Pose Error:                         "
        s +=  str(self.error_norm) + "\n"
        return s

    def update_pose(self, trans_mat):
        '''
        Calculates and updates the actual positions and orientations of all reference_positions and reference_orientations according to the given configuration
        '''
        #Create pose vector:
        self.pose = numpy.zeros(( 3*len(self.reference_positions) + 9*len(self.reference_orientations) ))
        
        cnt = 0 #counter for pose error vector
        k   = 0 #counter for pose vector
        
        # For reference_positions:
        for tp in self.reference_positions:
            # Calculating the position of each task point
            tp.update_position(trans_mat)
            # Counting total number of constraints
            cnt = cnt + len(tp.error.value)
            #Arranging three position coordinates of each reference_position in the endeffector pose vector
            for j in range(0,3):
                self.pose[k] = tp.ra[j]
                k +=1
                
        self.mp = cnt
        # For reference_orientations:
        for tf in self.reference_orientations:
            tf.update_orientation(trans_mat)    
            #Counting total number of constraints
            cnt = cnt + tf.error.W.shape[0]
            #Arranging nine orientation coordinates of each reference_orientation (Elements of the Rotation Matrix) in the endeffector pose vector
            for i in range(0,3):
                for j in range(0,3):
                    self.pose[k] = tf.ra[i,j]
                    k +=1

        self.mo = cnt - self.mp
        
    def update_pose_error(self):
        '''
        Calculates and updates the current values of position and orientation errors between the actual and target poses of reference_positions and reference_orientations
        Also updates the values of "pose_error" and "error_norm"
        '''
        #Creation of the vector of errors
        cnt = 0
        self.pose_error = numpy.zeros((self.mp + self.mo))
        #For reference_positions
        self.all_reference_positions_in_target = True
        for tp in self.reference_positions:
            # Calculating position error for each reference_position
            tp.error.update(tp.ra, tp.rd)
            self.all_reference_positions_in_target = self.all_reference_positions_in_target and tp.error.in_target
            #Arranging the vector of errors for position
            for k in range(0,len(tp.error.value)):
                self.pose_error[cnt] = tp.error.value[k]
                cnt = cnt + 1
        #For reference_orientations
        self.all_reference_orientations_in_target = True
        for tf in self.reference_orientations:
            # Calculating orientation error for each reference_orientation
            tf.error.update(tf.ra, tf.rd)
            self.all_reference_orientations_in_target = self.all_reference_orientations_in_target and tf.error.in_target
            #Arranging the vector of errors for orientation"
            for k in range(0,len(tf.error.value)):
                self.pose_error[cnt] = tf.error.value[k]
                cnt = cnt + 1

        self.error_norm = numpy.linalg.norm(self.pose_error)
        
    def update_geometric_jacobian(self, fwd_kin):
        '''
        Calculates and updates the geometric jacobian of the endeffector according to the reference_positions and reference_orientations
        '''
        #Create the geometric jacobian matrix
        cnt = 0
        np = 3*len(self.reference_positions)
        no = 3*len(self.reference_orientations)
        self.GJ = numpy.zeros((np + no, fwd_kin.configuration.settings.DOF))
        #For reference_positions
        for i in range(0,len(self.reference_positions)):
            tp = self.reference_positions[i]
            # Calculating geometric jacobian matrice corresponding to each reference_position
            tp.update_geometric_jacobian(fwd_kin.configuration, fwd_kin.analytic_jacobian)
            #Arranging geometric jacobian for position
            for k in range(0,3):
                for j in range(0, fwd_kin.configuration.settings.DOF):
                    self.GJ[3*i + k, j] = tp.geometric_jacobian.value[k,j]
        #For reference_orientations
        for i in range(0,len(self.reference_orientations)):
            tf = self.reference_orientations[i]
            #Calculate geometric jacobian matrice corresponding to each reference_orientation"
            tf.update_geometric_jacobian(fwd_kin)
            #Arranging geometric jacobian for orientation
            for k in range(0,3):
                for j in range(0, fwd_kin.configuration.settings.DOF):
                    self.GJ[np + 3*i + k,j] = tf.geometric_jacobian.value[k,j]

    def update_error_jacobian(self, fwd_kin):
        '''
        Calculates and updates the error jacobian of the endeffector according to the actual and target poses of the reference_positions and reference_orientations and their corresponding metrics
        '''
        cnt = 0
        #Creation of the error jacobian matrix"
        self.EJ = numpy.zeros((self.mp + self.mo, fwd_kin.configuration.settings.DOF))
        #For reference_positions
        for tp in self.reference_positions:
            # Calculating error jacobian for each reference_position
            tp.update_error_jacobian(fwd_kin.configuration)
            #Arranging error jacobian for position
            for k in range(0,len(tp.error.value)):
                for j in range(0, fwd_kin.configuration.settings.DOF):
                    self.EJ[cnt,j] = fwd_kin.configuration.jmc_c[j]*tp.error_jacobian.value[k,j]
                cnt = cnt + 1
        #For reference_orientations
        for tf in self.reference_orientations:
            # Calculating error jacobian for each reference_orientation
            tf.update_error_jacobian(fwd_kin)
            #Arranging error jacobian for orientation
            for k in range(0,len(tf.error.value)):
                for j in range(0, fwd_kin.configuration.settings.DOF):
                    self.EJ[cnt,j] = fwd_kin.configuration.jmc_c[j]*tf.error_jacobian.value[k,j]
                cnt = cnt + 1
    
    def update_task_jacobian(self):
        '''
        Calculates and updates the task jacobian of the endeffector according to the actual and target poses of the reference_positions and reference_orientations and their corresponding metrics
        '''
        cnt = 0
        col = 0
        ncol = 3*len(self.reference_positions) + 9*len(self.reference_orientations)
        #Creation of the task jacobian matrix"
        self.TJ = numpy.zeros((self.mp + self.mo, ncol))
        #For reference_positions
        for tp in self.reference_positions:
            # Calculating task jacobian for each reference_position
            tp.update_task_jacobian()
            #Arranging task jacobian for position
            for k in range(0,len(tp.error.value)):
                for j in range(0, 3):
                    self.TJ[cnt,j+col] = tp.task_jacobian.value[k,j]
                cnt = cnt + 1
            col = col + 3
        #For reference_orientations
        for tf in self.reference_orientations:
            # Calculating task jacobian for each reference_orientation
            tf.update_task_jacobian()
            #Arranging task jacobian for orientation
            for k in range(0,len(tf.error.value)):
                for j in range(0, 9):
                    self.TJ[cnt,j + col] = tf.task_jacobian.value[k,j]
                cnt = cnt + 1
            col = col + 9
    
    def update_target(self, phi = 0.0):
        '''
        updates the target pose and target pose rate vectors (xd, xd_dot)
        target pose vector is also known as the "task"
        '''
        #Creation of the target pose and target pose rate vector:"
        self.xd = numpy.array([])
        self.vd = numpy.array([])
        #For reference_positions
        for tp in self.reference_positions:
            tp.set_target_from_trajectory(phi = phi)
            self.xd = numpy.append(self.xd, tp.rd)
            self.vd = numpy.append(self.vd, tp.vd)
        #For reference_orientations
        for tf in self.reference_orientations:
            tf.set_target_from_trajectory(phi = phi)
            self.xd = numpy.append(self.xd, tf.rd)
            self.vd = numpy.append(self.vd, tf.vd)

    def update(self, fwd_kin):
        '''
        This function calculates the transformation matrices (H[i] and T[i]) which represent the
        position and orientation of i-th link of the model.
        It uses the joint values in property / attribute q and updates the model posture
        by calculating and changing the values of all the other properties.         
        '''
        
        if 'update_taskspace_pose' in self.tasklist:
            self.update_pose(fwd_kin.transfer_matrices)
        
        if 'update_error' in self.tasklist:
            self.update_pose_error()

        self.in_target = self.all_reference_positions_in_target and self.all_reference_orientations_in_target
            
        if 'update_geometric_jacobian' in self.tasklist:
            self.update_geometric_jacobian(fwd_kin)
        
        if 'update_error_jacobian' in self.tasklist:
            self.update_error_jacobian(fwd_kin)

    def pose_to_tuple(self, pose_selection , parametrization):
        '''
        return the endeffector target pose as a concatenated tuple
        '''
        assert pose_selection in ['actual','desired']
        pose_tpl = ()
        
        for tp in self.reference_positions:
            if pose_selection == 'actual':
                pv = tp.ra
            elif pose_selection == 'desired':    
                pv = tp.rd
            else:
                assert False    
            
            for j in range(3):
                pose_tpl += pv[j],
            
        for tf in self.reference_orientations:
            if pose_selection == 'actual':
                rm = tf.ra
            elif pose_selection == 'desired':    
                rm = tf.rd
            else:
                assert False    

            if parametrization == 'Rotation Matrix':
                for j in range(3):
                    for k in range(3):
                        pose_tpl += rm[j][k],
            else:    
                ov = rotation.orientation_vector(rm, parametrization)
                for j in range(3):
                    pose_tpl += ov[j],

        return pose_tpl

    def tuple_to_pose(self, pose_selection, pose_tuple, parametrization):
        '''
        get the pose as a concatenated tuple and change endeffector target 
        (inverse of "self.target_pose_to_tuple")
        '''
        assert pose_selection in ['actual','desired']
        actual = (pose_selection == 'actual')
        
        cnt = 0
        for tp in self.reference_positions:
            for j in range(3):
                if actual:
                    tp.ra[j] = pose_tuple[cnt]
                else:
                    tp.rd[j] = pose_tuple[cnt]
                    
                cnt += 1
            
        for tf in self.reference_orientations:
            if parametrization == 'Rotation Matrix':
                for j in range(3):
                    for k in range(3):
                        if actual:
                            tf.ra[j][k] = pose_tuple[cnt]
                        else:    
                            tf.rd[j][k] = pose_tuple[cnt]
                        
                        cnt += 1

            else:
            
                ov = numpy.zeros((3))
                for j in range(0,3):
                    ov[j] = pose_tuple[cnt] 
                    cnt += 1

                if actual:
                    tf.ra = rotation.rotation_matrix(ov, parametrization)
                else:
                    tf.rd = rotation.rotation_matrix(ov, parametrization)
        
