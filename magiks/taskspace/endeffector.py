## @file:           endeffector.py
#  @brief:    	    This module provides a class containing a set of reference positions and orientations in the taskspace together 
#                   with their associated poses. 
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
#  Last Revision:  	23 August 2015

'''
Changes from last version:
    1- a new task cost function added:  Liegeois_Midrange_Deviance() used for Jacobian Nullspace Gradient Projection to avoid joint limits as a second-priority cost function
'''

import numpy, math, copy, sys
import link_point as lplib, task_reference as trlib, cost_function as cflib
import general_python as genpy

from magiks.geometry import manipulator_geometry as mangeolib 
from magiks.magiks_core import function_library as flib
from math_tools.geometry import rotation as rot, geometry as geolib
from math_tools.algebra import vectors_and_matrices as vecmat

class Endeffector_Settings(object):
    '''
    '''
    def __init__(self):

        '''
        If there are more than one endeffector, 
        precision settings can be set individually for each "position reference" (Task Point) and "orientation reference" (Task Frame)
        You can change each one after the creation of class instance
        '''    

        self.precision_base_for_position   = "Coordinate Difference"
        self.precision_base_for_rotation   = "Axis Angle"

        self.precision_for_position   = 0.02 # coordinate distance in meters
        self.precision_for_rotation   = 2.0  # axis angle in degrees

class Endeffector(mangeolib.Manipulator_Geometry):
    '''
    This class contains all kinematic properties of the taskspace of a manipulator.
    These properties are:
    
        1- task_point: A list of operational points (Position References)
        2- reference_orientation: A list of operational link frames (Orientation References)
        3- The geometric Jacobian
        4- The error Jacobian
        5- The pose error vector and its norm
        
    Method "update" calculates and update all these properties according to the given joint configuration.
    '''
    def __init__(self, config_settings, geo_settings, end_settings):
        super(Endeffector, self).__init__(copy.deepcopy(config_settings), copy.copy(geo_settings))
        self.end_settings = copy.copy(end_settings)
        # Property "task_points" : is a list of instances of class task_point
        # Each instance is a "task point" and determines a position reference for the endeffector
        '''
        The following code defines one task point by default for the link_segment model
        This defined task point is the origin of the last link of the model (The defined endeffector by default)
        '''
        self.task_point = []
        self.task_frame = []
        self.task_cost  = []

        last_link_number    = geo_settings.nlink - 1
        last_link_origin    = lplib.Link_Point(last_link_number,1,[0,0,0])
        endeffector_position_reference_by_default = trlib.Task_Point(self.config_settings, [last_link_origin])
        self.add_taskpoint(endeffector_position_reference_by_default)

        # Property "task_frame" : is a list of instances of class Task_Frame
        # Each instance is a "task frame" and determines an orientation reference for the endeffector

        '''
        The following code defines one task frame by default for the link_segment model
        This defined task frame is the orientation of the last link of the model (The defined endeffector by default)
        '''
        endeffector_orientation_reference_by_default = trlib.Task_Frame(self.config_settings, last_link_number)
        self.add_taskframe(endeffector_orientation_reference_by_default)

        self.task_cost.append(cflib.Cost_Function(input_ref = 'Joint Values'))
        self.task_cost[0].function = flib.Zghal_Function(xl = numpy.copy(self.ql), xh = numpy.copy(self.qh))
        self.task_cost[0].abs_grad = numpy.zeros(self.config_settings.DOF)
        # self.task_cost[0].purpose = 'Joint Damping'
        '''
        self.task_cost.append(cflib.Cost_Function(input_ref = 'Joint Values'))
        self.task_cost[1].function = flib.Liegeois_Midrange_Deviance(xl = numpy.copy(self.ql), xh = numpy.copy(self.qh))
        # self.task_cost[1].purpose = 'Gradient Projection'
        '''    

        # mp -- number of position coordinates (3 by default)
        # mo -- number of orientation coordinates (3 by default) 
        # ("self.mp" elements representing position error come first )
        # ("self.mo" elements representing orientation error come next) 
        
        (mp, mo) = self.num_task_constraints()

        self.clear_pose()        

    def clear_pose(self):
        for tp in self.task_point:
            tp.clear()
                
        for tf in self.task_frame:
            tf.clear()
                
        # self.current_pose is the pose vector. Three elements for each task_point followed by nine elements for each reference_orientation
        # represent actual positions and orientations of the endeffector respectively in one pose vector
        self.current_pose             = None
        # Matrix of Geometric Jacobian (for both positions and orientations)
        # The first rows represent geometric jacobian for position. Number of rows for position = 3*len(task_points)
        # The next  rows represent geometric jacobian for orientation. Number of rows for orientation = 3*len(task_frame)
        self.geo_jac                  = None
        self.clear_error()

    def clear_error(self):
        for tp in self.task_point:
            tp.clear_error()
            
        for tf in self.task_frame:
            tf.clear_error()
                
        # self.current_pose_error is the overall vector containing values of error functions (both position and orientation) 
        self.current_pose_error       = None
        # self.current_error_norm contains the norm of the error vector
        self.current_error_norm       = None

        # Matrix of Error Jacobian (for both position and orientation errors)
        # (The first "self.mp" rows represent error jacobian for position)
        # (The next  "self.mo" rows represent error jacobian for orientation)
        self.err_jac                  = None
        # Property "self.all_task_points_in_target" is True if all of the task_points are in their desired positions
        self.all_taskpoints_in_target = None
        # Property "self.all_reference_orientations_in_target" is True if all of the reference_orientations are identical to their desired orientations
        self.all_taskframes_in_target = None
        # Property "self.in_target" is True if all of the task_points and task_frames are in their desired positions and orientations
        self.is_in_target = None
        # Property self.mu contains the current value of the manipulability index 
        self.mu           = None

    def add_taskpoint(self, task_point):
        self.task_point.append(task_point)
        n_tp = len(self.task_point) - 1
        
        self.task_point[n_tp].error.settings.precision      = self.end_settings.precision_for_position
        self.task_point[n_tp].error.settings.precision_base = self.end_settings.precision_base_for_position

        self.clear_pose()

    def add_taskframe(self, task_frame):
        self.task_frame.append(task_frame)
        n_tf = len(self.task_frame) - 1
        self.task_frame[n_tf].error.settings.precision                  = self.end_settings.precision_for_rotation
        self.task_frame[n_tf].error.settings.precision_base = self.end_settings.precision_base_for_rotation

    def __str__(self):
        s = ''
        for i in range(len(self.task_point)):
            s +=  "    Reference Position " + str(i) + ":" + "\n" + "\n"
            #s +=  "-----------------------------" + "\n"
            s +=  str(self.task_point[i])
        s +=  "\n"
        for i in range(len(self.task_frame)):
            s +=  "    Reference Orientation " + str(i) + ":" + "\n" + "\n"
            #s +=  "-----------------------------" + "\n"
            s +=  str(self.task_frame[i])
            
        s +=  "\n"
        s +=  "    Pose Error:                                 "
        s +=  vecmat.vector_to_str(self.pose_error()) + "\n"
        s +=  "    Norm of Pose Error:                         "
        s +=  str(self.pose_error_norm()) + "\n"
        return s

    def num_task_constraints(self):
        cnt = 0 # constraint counter
        
        # For task_points:
        for tp in self.task_point:
            # Counting total number of constraints
            cnt = cnt + tp.error.settings.weight.shape[0]
                
        self.mp = cnt
        # For task_frame:
        for tf in self.task_frame:
            #Counting total number of constraints
            cnt = cnt + tf.error.settings.weight.shape[0]

        self.mo = cnt - self.mp
        return (self.mp, self.mo)

    def joint_damping_weights(self):

        W = numpy.ones(self.config_settings.DOF)
        for cf in self.task_cost:
            if cf.purpose == 'Joint Damping':
                abs_grad  = abs(cf.gradient(self))
                delta     = abs_grad - cf.abs_grad
                for i in range(self.config_settings.DOF):
                    if delta[i] >= 0.0:
                        W[i]      += cf.weight*abs_grad[i]
                    cf.abs_grad[i] = abs_grad[i]
                
        return numpy.diag(1.0/W)
    
    def pose(self):
        if self.current_pose == None:
            '''
            Calculates and updates the actual positions and orientations of all task_points and task_frames according to the given configuration
            '''
            #Create pose vector:
            self.current_pose = numpy.zeros(( 3*len(self.task_point) + 9*len(self.task_frame) ))
            
            k   = 0 #counter for pose vector
            
            H = self.transfer_matrices()
            # For task_points:
            for tp in self.task_point:
                pos = tp.position(H)
                #Arranging three position coordinates of each task_point in the endeffector pose vector
                for j in range(0,3):
                    self.current_pose[k] = pos[j]
                    k +=1
                    
            # For task_frames:
            for tf in self.task_frame:
                #Arranging nine orientation coordinates of each reference_orientation (Elements of the Rotation Matrix) in the endeffector pose vector
                ori = tf.orientation(H).matrix()
                for i in range(0,3):
                    for j in range(0,3):
                        self.current_pose[k] = ori[i,j]
                        k +=1

        return self.current_pose

    def pose_error(self):        
        if self.current_pose_error == None:
            '''
            Calculates and updates the current values of position and orientation errors between the actual and target poses of task_points and task_frames
            Also updates the values of "pose_error" and "error_norm"
            '''
            #Creation of the vector of errors
            cnt = 0
            p   = self.pose() # This should change
            self.num_task_constraints()
            self.current_pose_error = numpy.zeros((self.mp + self.mo))
            #For task_points
            self.all_task_points_in_target = True
            H = self.transfer_matrices()
            for tp in self.task_point:
                ers = "Target has not been set for some of the taskpoints"
                assert tp.rd != None, genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, ers)
                # Calculating position error for each task_point
                self.all_task_points_in_target = self.all_task_points_in_target and tp.error.in_target(tp.position(H), tp.rd)
                #Arranging the vector of errors for position
                tp_err = tp.error.value(tp.ra, tp.rd)
                for k in range(0,len(tp_err)):
                    self.current_pose_error[cnt] = tp_err[k]
                    cnt = cnt + 1
            #For task_frames
            self.all_task_frames_in_target = True
            for tf in self.task_frame:
                ers = "Target has not been set for some of the taskframes"
                assert tf.rd != None, genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, ers)
                # Calculating orientation error for each task_frame
                self.all_task_frames_in_target = self.all_task_frames_in_target and tf.error.in_target(tf.orientation(H), tf.rd)
                #Arranging the vector of errors for orientation"
                tf_err = tf.error.value(tf.ra, tf.rd)
                for k in range(0,len(tf_err)):
                    self.current_pose_error[cnt] = tf_err[k]
                    cnt = cnt + 1

            self.is_in_target = self.all_task_points_in_target and self.all_task_frames_in_target
            
        return self.current_pose_error
        
    def pose_error_norm(self):        
        if self.current_error_norm == None:
            p = self.pose_error()
            self.current_error_norm = numpy.linalg.norm(self.current_pose_error)
        return self.current_error_norm

    def geometric_jacobian(self):
        if self.geo_jac == None:
            '''
            Calculates and updates the geometric jacobian of the endeffector according to the task_points and task_frames
            '''
            #Create the geometric jacobian matrix
            cnt = 0
            npos = 3*len(self.task_point)
            nori = 3*len(self.task_frame)
            self.geo_jac = numpy.zeros((npos + nori, self.config_settings.DOF))
            #For task_points
            for i in range(0,len(self.task_point)):
                tp = self.task_point[i]
                # Calculating geometric jacobian matrice corresponding to each task_point
                tp.update_geometric_jacobian(self.analytic_jacobian)
                #Arranging geometric jacobian for position
                for k in range(0,3):
                    for j in range(0, self.config_settings.DOF):
                        self.geo_jac[3*i + k, j] = tp.geometric_jacobian.value[k,j]
            #For task_frames
            for i in range(0,len(self.task_frame)):
                tf = self.task_frame[i]
                #Calculate geometric jacobian matrice corresponding to each reference_orientation"
                tf.update_geometric_jacobian(self.H)
                #Arranging geometric jacobian for orientation
                for k in range(0,3):
                    for j in range(0, self.config_settings.DOF):
                        self.geo_jac[npos + 3*i + k,j] = tf.geometric_jacobian.value[k,j]

        return self.geo_jac

    def error_jacobian(self):
        if self.err_jac == None:
            '''
            Calculates and updates the error jacobian of the endeffector according to the actual and target poses of the task_points and task_frames and their corresponding metrics
            '''
            cnt = 0
            #Creation of the error jacobian matrix"
            self.err_jac = numpy.zeros((self.mp + self.mo, self.config_settings.DOF))
            H = self.transfer_matrices()
            #For task_points
            for tp in self.task_point:
                ers = "Target has not been set for some of the taskpoints"
                assert tp.rd != None, genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, ers)
                # Calculating error jacobian for each task_point
                tp_ej = tp.error_jacobian(H, self.ajac)

                tp_er = tp.error.value(tp.position(H), tp.rd)
                #Arranging error jacobian for position
                for k in range(0,len(tp_er)):
                    for j in range(0, self.config_settings.DOF):
                        self.err_jac[cnt,j] = self.jmc_c[j]*tp_ej[k,j]
                    cnt = cnt + 1
            #For task_frames
            for tf in self.task_frame:
                ers = "Target has not been set for some of the taskframes"
                assert tf.rd != None, genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, ers)
                # Calculating error jacobian for each reference_orientation
                tf_ej = tf.error_jacobian(self.ajac)
                tf_er = tf.error.value(tf.orientation(H), tf.rd)
                #Arranging error jacobian for orientation
                for k in range(0,len(tf_er)):
                    for j in range(0, self.config_settings.DOF):
                        self.err_jac[cnt,j] = self.jmc_c[j]*tf_ej[k,j]
                    cnt = cnt + 1
        return self.err_jac

    def manipulability(self):
        if self.mu == None:    
            J       = self.error_jacobian()
            self.mu = math.sqrt(abs(numpy.linalg.det(numpy.dot(J, J.T))))
        return self.mu    
            
    def in_target(self):
        if self.is_in_target == None:
            pe = self.pose_error()
        return self.is_in_target
        
    def pose_to_tuple(self, pose_selection , parametrization):
        '''
        return the endeffector target pose as a concatenated tuple
        '''
        assert pose_selection in ['actual','desired']
        pose_tpl = ()

        H = self.transfer_matrices()
        for tp in self.task_point:
            if pose_selection == 'actual':
                pv = tp.position(H)
            elif pose_selection == 'desired':    
                pv = tp.rd
            else:
                assert False    
            
            for j in range(3):
                pose_tpl += pv[j],
            
        for tf in self.task_frame:
            if pose_selection == 'actual':
                rm = tf.orientation(H)
            elif pose_selection == 'desired':    
                rm = tf.rd
            else:
                assert False    

            if parametrization == 'Rotation Matrix':
                for j in range(3):
                    for k in range(3):
                        pose_tpl += rm['matrix'][j][k],
            else:    
                ov = rm[parametrization]
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
        r   = numpy.zeros(3)
        for tp in self.task_point:
            for j in range(3):
                r[j] = pose_tuple[cnt]
                cnt += 1
            if actual:
                tp.ra = copy.copy(r)
            else:
                tp.set_target(copy.copy(r))

        for tf in self.task_frame:
            if parametrization == 'Rotation Matrix':
                r = numpy.eye(3)
                for j in range(3):
                    for k in range(3):
                        r[j][k] = pose_tuple[cnt]
                        cnt += 1
                if actual:
                    tf.ra['matrix'] = r
                else:
                    tf.set_target(geolib.Orientation_3D(r, ori_velocity = numpy.zeros((3,3))))

            else:
            
                ov = numpy.zeros((3))
                for j in range(0,3):
                    ov[j] = pose_tuple[cnt] 
                    cnt += 1

                if actual:
                    tf.ra = rotation.rotation_matrix(ov, parametrization)
                else:
                    tf.rd = rotation.rotation_matrix(ov, parametrization)

        self.clear_error()    

    def set_config_virtual(self, qvrd):
        if super(Endeffector, self).set_config_virtual(qvrd):
            self.clear_pose()
            return True
        else:
            return False

    def set_config(self, qd):
        if super(Endeffector, self).set_config(qd):
            self.clear_pose()
            return True
        else:
            return False        

    def set_target(self, position, orientation):
        assert len(position)    <= len(self.task_point)
        assert len(orientation) <= len(self.task_frame)
        cnt = 0
        for p in position:
            self.task_point[cnt].set_target(p)
            cnt += 1
        cnt = 0
        for o in orientation:
            o.set_velocity(numpy.zeros((3,3)), representation = 'matrix')
            self.task_frame[cnt].set_target(o)
            cnt += 1

        self.clear_error()    
