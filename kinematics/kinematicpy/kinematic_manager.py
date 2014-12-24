'''   Header
@file:          kinematic_manager.py
@brief:    	    This module contains the main class for handling the kinematics of a manipulator
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
@version:	    0.5
Last Revision:  23 October 2012
'''


# BODY
import numpy, math, time, copy, pickle

from packages.nima.mathematics import rotation

import packages.nima.robotics.kinematics.kinematicpy.geometry as geolib 
import packages.nima.robotics.kinematics.kinematicpy.forward_kinematics as fklib 
import packages.nima.robotics.kinematics.kinematicpy.inverse_kinematics as iklib 
import packages.nima.robotics.kinematics.task_space.workspace as wslib 
import packages.nima.robotics.kinematics.joint_space.configuration as conflib 
import packages.nima.robotics.kinematics.kinematicpy.log_manager as loglib 

import packages.nima.robotics.kinematics.kinematicpy.manipulator_library as maniplib

"""
def generate_my_default_kin_manager() : 
    '''
    the name of method says that it creates somthing , but that it sets values, "massively"
    '''
    default_grids = ... 
"""

def generate_orientation_error_function_package(orientation_constraint):

    if orientation_constraint == 'Aligned Axis i,j,k':
        basis_err_func = 'Axis Inner Product'
        # define the weighting matrix. It considers All three axis i, j and k to be aligned
        weighting_matrix   = numpy.eye(3)
        # define the corresponding power array. (Please refer to the documentation)
        power_array        = numpy.array([1, 1, 1])
        constant_offset    = numpy.array([-1.0, -1.0, -1.0])

    elif orientation_constraint == 'Aligned Axis i,j':
        basis_err_func = 'Axis Inner Product'
        # define the weighting matrix. It considers only axis "i" and "j" to be identical. If Axis "i" and "j" are identical, then Axis "k" will be identical as well.
        weighting_matrix   = numpy.array([[1, 0, 0], 
                                          [0, 1, 0]] )
        # define the corresponding power array. (Please refer to the documentation)
        power_array        = numpy.array([1, 1, 0])
        constant_offset    = numpy.array([-1.0, -1.0])

    elif orientation_constraint == 'Aligned Axis j,k':

        basis_err_func = 'Axis Inner Product'
        # define the weighting matrix. It considers only axis "j" and "k" to be identical. If Axis "j" and "k" are identical, then Axis "i" will be identical as well.
        weighting_matrix   = numpy.array([[0, 1, 0], 
                                          [0, 0, 1]] )
        power_array        = numpy.array([0, 1, 1])
        constant_offset    = numpy.array([-1.0, -1.0])
        
    elif orientation_constraint == 'Aligned Axis i,k':

        basis_err_func = 'Axis Inner Product'
        # define the weighting matrix. It considers only axis "i" and "k" to be identical. If Axis "i" and "k" are identical, then Axis "j" will be identical as well.
        weighting_matrix   = numpy.array([[1, 0, 0], 
                                          [0, 0, 1]] )
        # define the corresponding power array. (Please refer to the documentation)
        power_array        = numpy.array([1, 0, 1])
        constant_offset    = numpy.array([-1.0, -1.0])
        
    elif orientation_constraint == 'Trace of Relative Rotation Matrix equals three':
        #Defibe basis error function. It ensures that trace of relative rotation matrix is three or : trace(R_a * R_d^T) = 3.0
        basis_err_func = 'relative_rotation_matrix_trace_minus_three'
        # define the appropriate weighting matrix. 
        weighting_matrix   = numpy.array([[1]])
        # define the corresponding power array. (Please refer to the documentation)
        power_array        = numpy.array([1])
        constant_offset    = numpy.array([0])
        
    elif orientation_constraint == 'Relative Rotation Angle equals zero':
        basis_err_func = 'relative_rotation_angle'
        weighting_matrix   = numpy.array([[1]])
        power_array        = numpy.array([1])
        constant_offset    = numpy.array([0])
    elif orientation_constraint == 'Difference of Normalized Quaternions equals zero':
        basis_err_func = 'differential_quaternions'
        weighting_matrix   = numpy.eye(3)
        power_array        = numpy.array([1, 1, 1])
        constant_offset    = numpy.array([0, 0, 0])
    elif orientation_constraint == 'Relative Orientation Vector equals zero (Generator Function: Identity)':
        basis_err_func = 'relative_rotation_vector_identity'
        weighting_matrix   = numpy.eye(3)
        power_array        = numpy.array([1, 1, 1])
        constant_offset    = numpy.array([0, 0, 0])
    elif orientation_constraint == 'Difference of Rotation Matrices equals zero':
        basis_err_func = 'differential_rotation_matrix'
        weighting_matrix   = numpy.eye(9)
        power_array        = numpy.array([1, 1, 1, 1, 1, 1, 1, 1, 1])
        constant_offset    = numpy.array([0, 0, 0, 0, 0, 0, 0, 0, 0])

    else:
        assert False
    
    return (basis_err_func, weighting_matrix, constant_offset, power_array)
        
def generate_position_error_function_package(position_constraint):
    if position_constraint == 'Identical Coordinate x,y,z':
        basis_err_func = 'differential_cartesian_coordinates'
        # define the weighting matrix as Identity. It ensures all three position coordinates are identical when the error function is zero.
        weighting_matrix   = numpy.eye(3)
        # define the corresponding power array. (Please refer to the documentation)
        power_array        = numpy.array([1, 1, 1])
        constant_offset    = numpy.array([0, 0, 0])
    else:
        assert False
    
    return (basis_err_func, weighting_matrix, constant_offset, power_array)
    
class Kinematic_Manager_Settings():
    '''
    '''
    
#    def __init__(self, manip_name, ik_settings, grid_settings, loging_settings, .. ):
    

    def __init__(self, manip_name):
        '''
        '''
        
        #self.ik_settings     = ik_settings
        
        # Name of the manipulator
        self.manip_name = manip_name

        #Number of endeffectors (position references)
        self.num_position_ref       = 1
        #Number of endeffectors (orientation references)
        self.num_orientation_ref    = 1

        self.orientation_constraint = 'Aligned Axis j,k'
        self.position_constraint    = 'Identical Coordinate x,y,z'
        
        # Representation of Orientation Error
        #self.err_func_orient        = 'Axis Inner Product'
               
        self.config_settings                = conflib.Joint_Configuration_Settings(default_joint_handling = 'Trigonometric Mapping')
        self.ik_settings                    = iklib.Inverse_Kinematics_Settings(run_mode = 'normal_run', num_steps = 100) 
        # When searching for nearest starting config in a precomputed set of poses, "self.workspace_settings_ic.grid_resolution" determine the jointspace grid size by which precomputed poses are generated
        # "number_of_search_items" Determine how many startiong configs are to be found in search (Number of trials) (For each starting config, an IK trial is implemented)
        self.workspace_settings_ic          = wslib.Workspace_Settings(number_of_search_items = 10)
        self.workspace_settings_tp          = wslib.Workspace_Settings(representation_of_orientation = 'Rotation Matrix')

        # In the test evaluation, "self.grid_res_tar_pose" determine the jointspace grid size by which target poses are generated.
        
        import os
        
        ## MANUAL : path = os.getcwd() + "/" + manip_name
        ## via LIB 
        path = os.path.join( os.getcwd(), manip_name + '_results' ) 
        
        if os.path.exists(path) == False:
            print "path not found"
            os.makedirs(path) 
    
        # similar simplification ... 
        self.path_str =  path + '/'
        
    def __str__( self ) : 
        '''
        returns the algorithmic setting for the inverse kinematic evaluator
        '''
        s =   '\n'
        s  += 'Kinematic Manager Settings:' + '\n' + '\n'
        s  += 'Manipulator                               : ' + self.manip_name + '\n'
        s  += 'Number of position references             : ' + str(self.num_position_ref) + '\n'
        s  += 'Number of orientation references          : ' + str(self.num_orientation_ref) + '\n'
        s  += 'Error function for position               : ' + self.position_constraint + '\n'
        s  += 'Error function for orientation            : ' + self.orientation_constraint + '\n'
        s  += 'Maximum number of iterations              : ' + str(self.ik_settings.number_of_steps) + '\n' + '\n'

        s  += 'Initial Configuration Workspace: ' + '\n'
        s  += str(self.workspace_settings_ic)  + '\n'
            
        s  += 'Target Pose Workspace: ' + '\n' 
        s  += str(self.workspace_settings_tp)  + '\n'
        
        return s
            
    def generate_key( self ):
        '''
        returns the settings key string for the inverse kinematic evaluator
        '''
        
        key_str  = self.manip_name + '_'
        for i in range(0, self.num_position_ref):
            key_str += 'P'
        for i in range(0, self.num_orientation_ref):
            key_str += 'O'
        key_str += '_MNI' + str(self.ik_settings.number_of_steps) + '_'
        if self.orientation_constraint == 'Aligned Axis i,j,k':
            key_str += 'AxInPr_ijk'
        elif self.orientation_constraint == 'Aligned Axis i,j':
            key_str += 'AxInPr_ij'
        elif self.orientation_constraint == 'Aligned Axis i,k':
            key_str += 'AxInPr_ik'
        elif self.orientation_constraint == 'Aligned Axis j,k':
            key_str += 'AxInPr_jk'
        elif self.orientation_constraint == 'Trace of Relative Rotation Matrix equals three':
            key_str += 'ReRoMxTr3'
        elif self.orientation_constraint == 'Difference of Normalized Quaternions equals zero':
            key_str += 'DiNoQu'
        elif self.orientation_constraint == 'Relative Rotation Angle equals zero':
            key_str += 'ReRoAn'
        elif self.orientation_constraint == 'Relative Orientation Vector equals zero (Generator Function: Identity)':
            key_str += 'ReOrVe_Idt'
        elif self.orientation_constraint == 'Difference of Rotation Matrices equals zero':
            key_str += 'DiRoMx'
        else:
            assert False    


        if self.config_settings.default_joint_handling == 'No Mapping':
            key_str += '_JSNM'
        elif self.config_settings.default_joint_handling == 'Linear Mapping':
            key_str += '_JSLM'
        elif self.config_settings.default_joint_handling == 'Trigonometric Mapping':
            key_str += '_JSTM'
        elif self.config_settings.default_joint_handling == 'Tangent Mapping':
            key_str += '_JSTMG'
        else:
            assert False
            
        key_str += '_TP_' + self.workspace_settings_tp.gen_key()
        key_str += '_IC_' + self.workspace_settings_ic.gen_key()
            
        key_str += '_' + self.workspace_settings_ic.search_criteria_key()    
            
        return key_str    

class Kinematic_Manager :
    '''
    
    Kinematic_Manager is the kinematic manager of a manipulator. 
    contains a mathematical kinematic model of a chain of joints and links which can be referred to a robot or any mechanical manipulator,
    the whole or parts of human or animal moving mechanism.
    This class contains all kinematic properties of a manipulator like geometry, joint configuration, transfer matrices, endeffector(s) and the jacobians
    plus other miscellaneous geometric and algorithmic parameters for forward and inverse kinematic computations.
    It also has a method which evaluate the performance of inverse kinematic solver by evaluating it for a set of target poses in the 
    workspace generated from a jointspace grid.
    '''

    def __init__(self, geometry, config, settings):
        '''
        Predefined Class Properties :
        '''
        self.settings           = settings
        self.km_log             = loglib.Eval_Log_Data_Multiple(settings)
        
        # "self.forward_kinematics" is an instance of class "Forward_Kinematics" in package "kinematicpy" which contains all kinematic properties of a manipulator which depend on the joint configuration
        # These properties are: transfer matrices, analytic, geometric and error jacobians, taskpoints and taskframes and the pose error vector.

        #self.forward_kinematics = fklib.Forward_Kinematics(geometry, config)        
        self.inverse_kinematics = iklib.Inverse_Kinematics(geometry, config, self.settings.ik_settings )
        
        #set desired position and orientation constraints:

        for tp in self.inverse_kinematics.endeffector.reference_positions:
            (tp.error.basis_error_function, tp.error.W, tp.error.C, tp.error.power) = generate_position_error_function_package(self.settings.position_constraint)
        
        for tf in self.inverse_kinematics.endeffector.reference_orientations:
            (tf.error.basis_error_function, tf.error.W, tf.error.C, tf.error.power) = generate_orientation_error_function_package(self.settings.orientation_constraint)
        
        #self.inverse_kinematics.configuration.set_joint_limits_for(settings.manip_name)
        self.inverse_kinematics.configuration.initialize()

        if self.settings.num_position_ref       == 0:
            self.inverse_kinematics.endeffector.reference_positions = []
        if self.settings.num_orientation_ref    == 0:
            self.inverse_kinematics.endeffector.reference_orientations = []

        if settings.workspace_settings_ic.read_workspace:    
            print 'Reading Init Config Workspace ... '
            self.workspace_ic = wslib.read_from_file(settings.path_str + settings.workspace_settings_ic.gen_file_name(geometry.name))
            self.workspace_ic.settings = self.settings.workspace_settings_ic
            print 'Reading Init Config Workspace Finished '
        else:    
            self.workspace_ic = wslib.Workspace(settings.workspace_settings_ic)
            self.workspace_ic.create(self.inverse_kinematics.geometry, self.inverse_kinematics.configuration, self.inverse_kinematics.endeffector)
            
        if settings.workspace_settings_ic.write_workspace:    
            print 'Saving init config workspace to : ', settings.path_str + settings.workspace_settings_ic.gen_file_name(geometry.name) + ' ... '
            self.workspace_ic.write_to_file(settings.path_str + settings.workspace_settings_ic.gen_file_name(geometry.name))
            print 'Finished saving workspace.'

        if settings.workspace_settings_tp.read_workspace:    
            print 'Reading Target Pose Workspace ... '
            st = copy.copy(self.settings.workspace_settings_tp)
            self.workspace_tp = wslib.read_from_file(settings.path_str + settings.workspace_settings_tp.gen_file_name(geometry.name))
            self.workspace_tp.settings = self.settings.workspace_settings_tp
            print 'Reading Target Pose Workspace Finished '
        else:    
            self.workspace_tp = wslib.Workspace(settings.workspace_settings_tp)
            self.workspace_tp.create(self.inverse_kinematics.geometry, self.inverse_kinematics.configuration, self.inverse_kinematics.endeffector)
            
        if settings.workspace_settings_tp.write_workspace:    
            print 'Saving target pose workspace to : ', settings.path_str + settings.workspace_settings_tp.gen_file_name(geometry.name) + ' ... '
            self.workspace_tp.write_to_file(settings.path_str + settings.workspace_settings_tp.gen_file_name(geometry.name))
            print 'Finished saving workspace.'

        #self.inverse_kinematics.settings.number_of_steps = settings.max_num_iters
        
        #self.inverse_kinematics.configuration.joint_handling = settings.joint_map
        
    def transfer_kinematics(self):
        '''
        transfer_kinematics --> the word transfer is not nice : why do not use COPY then ? 
        
        QUESTION(S) : 1. is it REALLY needed? 2. is it ADVANTAGEOUS to have the "configuration" saved at more than one place ? 
                    -> YES.                      
        
        Transfer manipulator kinematics (including "configuration", "transfer_matrices" and "analytic_jacobian") 
        from "forward_kinematics" property to "inverse_kinematics"
        The kinematic properties of "inverse_kinematics" are considered as the initial state for inverse kinematic iterative solver
        '''            
        print "deprecated."
        assert False 
        
        self.inverse_kinematics.configuration     = copy.deepcopy(self.forward_kinematics.configuration)
        self.inverse_kinematics.transfer_matrices = copy.deepcopy(self.forward_kinematics.transfer_matrices)
        self.inverse_kinematics.analytic_jacobian = copy.deepcopy(self.forward_kinematics.analytic_jacobian)
        
    def take_target_to_grid_configuration(self, config_number, number_of_intervals):
        '''
        Replaces the endeffector target pose of the inverse_kinematics with the pose corresponding to joint configuration with the configuration
         in a gridded jointspace with "number_of_intervals" divisions for each joint 
        The order of the configuration in the gridded jointspace is identified by "config_number",
        '''
        # create a temporary instance of Inverse_Kinematic class as a functor
        ik_temp = iklib.Inverse_Kinematics(self.inverse_kinematics.geometry, self.inverse_kinematics.configuration, self.settings.ik_settings)
        # copy kinematic properties of the manipulator to the temporary functor

        ik_temp.endeffector   = copy.deepcopy(self.inverse_kinematics.endeffector)

        # only a forward pose update is required for the temporary functor to do (no errors, no jacobians)
        ik_temp.tasklist             = ['update_transfer_matrices']
        ik_temp.endeffector.tasklist = ['update_taskspace_pose']

        # change temporary configuration to the specified configuration
        ik_temp.take_to_grid_configuration(config_number, number_of_intervals)
        # set pose targets of the manipulator as current poses of the temporary functor
        
        no_of_ref_pos = len(self.inverse_kinematics.endeffector.reference_positions)
        for i in range( no_of_ref_pos ):        
            tp      = self.inverse_kinematics.endeffector.reference_positions[i]
            tp_temp = ik_temp.endeffector.reference_positions[i]
            tp.rd   = numpy.copy(tp_temp.ra)        
            
        #ref_pos_const_ee    = self.inverse_kinematics.endeffector.reference_positions
        #ref_pos_tmp_ee      = ik_temp.endeffector.reference_positions        
        #for tmp_pos, const_pos in zip( ref_pos_tmp_ee, ref_pos_const_ee ) : 
        #    const_pos.rd   = numpy.copy(tmp_pos.ra)
                                
        for i in range(0, len(self.inverse_kinematics.endeffector.reference_orientations)):
            tf      = self.inverse_kinematics.endeffector.reference_orientations[i]
            tf_temp = ik_temp.endeffector.reference_orientations[i]
            tf.rd = numpy.copy(tf_temp.ra)
           
    def test_result(self, target_config_number, num_suc_til_now, config_run_time, average_step_time, config_num_steps, steps_til_now, time_til_now): 
            '''
            generate a string that describes print out 
            '''
            
            #Calculte initial pose:
            ik_temp = copy.deepcopy(self.inverse_kinematics)
            
            ik_temp.configuration = self.inverse_kinematics.initial_config
            ik_temp.forward_update()
            
            s = "\n"

            if self.inverse_kinematics.endeffector.in_target:
                s += "Target Configuration Number: " + str(target_config_number) + " Inversed Successfully with " + str(ik_temp.start_node) + " starting point trials:" + "\n" 
            else:
                s += "Target Configuration Number: " + str(target_config_number) + " was NOT Inversed Successfully after " + str(ik_temp.start_node) + " starting point trials:" + "\n"

            s += "\n" 
            s += "Number of successful attempts until now:          " + str(num_suc_til_now) + "\n"  
            s += "\n"
            s += "Number of steps for this configuration:           " + str(config_num_steps) + "\n" 
            s += "Time elapsed for this configuration:              " + str(1000*config_run_time) + " (ms)" + "\n" 
            s += "\n"
            s += "Total time elapsed until now:                     " + str(1000*time_til_now) + " (ms)"+ "\n" 
            s += "Total number of steps until now:                  " + str(steps_til_now) + "\n" 
            s += "\n"
            s += "Average time elapsed for one iteration:           " + str(1000*average_step_time) + " (ms)" + "\n"
            s += "Average time elapsed for one configuration:       " + str(1000*time_til_now/(target_config_number+1)) + " (ms)" + "\n"
            s += "Average number of steps for one configuration:    " + str(steps_til_now/(target_config_number+1)) + "\n"
            s += "\n" + "\n"
            s += "Percentage of success until now:                  %" + str(100.00*num_suc_til_now/(target_config_number+1)) + "\n"

            s +=  "******************************************************************************************************************************" + "\n"
            s +=  "Initial Configuration:                       "
            #s +=  "\n"
            s +=  str(ik_temp.configuration) + "\n"

            s +=  "Final Configuration:                         "
            #s +=  "\n"
            s +=  str(self.inverse_kinematics.configuration)  + "\n"

            s +=  "******************************************************************************************************************************" + "\n"
            s +=  "Initial Pose:                                         " + "\n"
            s +=  "-------------" + "\n"
            s += str(ik_temp.endeffector)
            s +=  "\n"
            s +=  "Final Pose:                                           " + "\n"
            s +=  "-------------" + "\n"
            s += str(self.inverse_kinematics.endeffector)
            s +=  "\n"
            s += "______________________________________________________________________________________________________________________________"

            return s

    def evaluate_ik_solver(self, verbose = False):
        '''
        run for the whole grid 
        
        Evaluate the ik solver by implementing it for a number of target poses which are evenly distributed in the workspace of the manipulator
        A grid(lattice) is created in the jointspace and the target poses are generated by computing the forward kinematics for those configurations
        Therefore, each configuration makes a corresponding target pose
        The ik solver is implemented for each target pose, from a constant initial configuration and a list of results is returned.
        '''
        # "p" specifies the number of tested target poses = number of configurations in the gridded jointspace
        #p = self.settings.grid_res_tar_pose ** self.inverse_kinematics.configuration.DOF
        n_suc = 0               #specifies number of successful attempts
        n_iters_total = 0       #specifies total number of iterations
        time_total = 0.00       #specifies total implementation time
        cnt = 0
        self.km_log.body = []
        
        for pose_tuple in self.workspace_tp.pose_list:
            
        #for qq in self.workspace_tp.config_list:
            
            #Set target for IK solver
            
            #self.take_target_to_grid_configuration(config_number = i, number_of_intervals = self.settings.grid_res_tar_pose)        
            '''
            self.inverse_kinematics.configuration.q = numpy.copy(qq)
            self.inverse_kinematics.forward_update()
            
            for tp in self.inverse_kinematics.endeffector.reference_positions:
                tp.rd = numpy.copy(tp.ra)
            for tf in self.inverse_kinematics.endeffector.reference_orientations:
                tf.rd = numpy.copy(tf.ra)
            '''
            self.inverse_kinematics.endeffector.tuple_to_pose('desired', pose_tuple, self.settings.workspace_settings_tp.representation_of_orientation)
            self.inverse_kinematics.endeffector.update(self.inverse_kinematics)
            
            #take the manipulator to stored initial state
            #self.inverse_kinematics.take_to_grid_configuration(0, self.settings.workspace_settings_ic.grid_resolution)

            start_kinematic_inversion = time.time()

            #find a list of the best initial configurations:
            
            self.inverse_kinematics.initial_config_list = self.workspace_ic.nearest_configs(self.inverse_kinematics)
            
            #implement the inverse kinematic solver

            self.inverse_kinematics.inverse_update()

            elapsed_kinematic_inversion = time.time() - start_kinematic_inversion

            self.workspace_tp.config_list[cnt] = numpy.copy(self.inverse_kinematics.configuration.q)

            (running_time, number_of_steps) = self.inverse_kinematics.log_info

            if number_of_steps == 0:
                number_of_steps = 1

            average_step_time = running_time / number_of_steps
            n_iters_total = n_iters_total + number_of_steps

            if self.inverse_kinematics.endeffector.in_target:
                n_suc = n_suc + 1

            time_total = time_total + elapsed_kinematic_inversion

            # Retrieve initial status from which the solution is achieved:
            ik_initial = copy.deepcopy(self.inverse_kinematics)

            ik_initial.configuration = self.inverse_kinematics.initial_config
            ik_initial.configuration.initialize()

            ik_initial.configuration.update_joint_limit_jacobian_multipliers()
            ik_initial.forward_update()
            start_config_str = str(ik_initial.configuration)
            start_pose_str = str(ik_initial.endeffector)
            # Final status:
            final_config_str = str(self.inverse_kinematics.configuration)
            final_pose_str = str(self.inverse_kinematics.endeffector)
            # Run Time Log:
            time_log = loglib.Log_Run_Time(elapsed_kinematic_inversion, time_total, average_step_time)
            # Num Iter Log:
            iter_log = loglib.Log_Num_Iter(number_of_steps, n_iters_total)
            # Success Log:
            suc_log = loglib.Log_Success(n_suc, self.inverse_kinematics.endeffector.in_target)
            self.km_log.body.append(loglib.Eval_Log_Data_Single(cnt, self.inverse_kinematics.start_node, suc_log, time_log, iter_log, start_config_str, final_config_str, start_pose_str, final_pose_str))

            cnt += 1

            #test_result_list.append(result_info_detail)
            if verbose:
                print str(self.km_log.body[cnt - 1])
                print 'Log: (Running Time, Number of Iterations)', self.inverse_kinematics.log_info;
                print "______________________________________________________________________________________________________________________________"
            
            #evaluation_log = (self.inverse_kinematics.endeffector.in_target, elapsed_kinematic_inversion, self.inverse_kinematics.log_info)
            #self.log_list.append(evaluation_log)
        self.km_log.header = self.settings
        self.km_log.footer.calculate_statistics(self.km_log.body)
        
        
    def execute(self, save_evaluation_log = True, save_evaluation_results = True, verbose = False):
        '''
        execute the evaluation according to the settings
        '''

        # set error function for orientation
        #self.inverse_kinematics.number_of_steps              = self.settings.max_num_iters
        #self.inverse_kinematics.configuration.joint_handling = self.settings.joint_map
    
        result_info_detail = str()

        test_key_str = self.settings.generate_key()
    
        if save_evaluation_results:
            FILE_RESULT = open(self.settings.path_str + test_key_str + '.res', "w")
        if save_evaluation_log:
            FILE_DETAIL = open(self.settings.path_str + test_key_str + '.log', "w")

        if self.settings.workspace_settings_ic.search_criteria == 'Nearest to Current Configuration':
            self.settings.ik_settings.include_current_config = True
        else:    
            self.settings.ik_settings.include_current_config = False
            
        self.evaluate_ik_solver(verbose = verbose)
    
        if save_evaluation_log:
            print 'Writing log file started ...'
            FILE_DETAIL.write(str(self.km_log.header))
            FILE_DETAIL.write("\n" + "--------------------------------------------------------------------------------" + "\n")
            for body_log in self.km_log.body:
                FILE_DETAIL.write(str(body_log))
            FILE_DETAIL.write("\n" + "--------------------------------------------------------------------------------" + "\n")
            FILE_DETAIL.write(str(self.km_log.footer))
            print 'Writing log file ended.'

        if save_evaluation_results:
            print 'Writing result file started ...'
            pickle.dump(self.km_log, FILE_RESULT)
            print 'Writing result file finished.'
