# HEADER

## @file        	kinematic_manager.py
#  @brief           This module contains the main class for handling the kinematics of a manipulator
#  @author      	Nima Ramezani Taghiabadi
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. :   04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	3.0
#
#  Last Revision:  	03 January 2015

# BODY

import numpy, math, time, copy, pickle

from packages.nima.mathematics import rotation

import packages.nima.robotics.kinematics.kinematicpy.geometry as geolib 
import packages.nima.robotics.kinematics.kinematicpy.forward_kinematics as fklib 
import packages.nima.robotics.kinematics.kinematicpy.inverse_kinematics as iklib 
import packages.nima.robotics.kinematics.task_space.workspace as wslib 
import packages.nima.robotics.kinematics.joint_space.configuration as conflib 
import packages.nima.robotics.kinematics.kinematicpy.log_manager as loglib 
import packages.nima.robotics.kinematics.task_space.metric as metriclib 

import packages.nima.robotics.kinematics.kinematicpy.manipulator_library as maniplib

"""
def generate_my_default_kin_manager() : 
    '''
    the name of method says that it creates somthing , but that it sets values, "massively"
    '''
    default_grids = ... 
"""
'''
'PP': 'Pose Projection',       # Inverse Kinematics for a single pose
'TG': 'Trajectory Generation', # Constant Target Kinematic Control 
'TP': 'Trajectory Projection', # Variable Target Kinematic Control
'''
all_application_scenarios = ['PPS', 'TGS', 'TPS'] 

key_dic = {
    # For Application Scenario:
    'NA'            : 'Not Applicable',
    'AS'            : 'Application Scenario',
    'PPS'           : 'Pose Projection Scenario',            # Single Pose Inverse Kinematic (Project a single pose to the workspace)
    'TGS'           : 'Trajectory Generation Scenario',      # Kinematic Control-Constant Target Scenario
    'TPS'           : 'Trajectory Projection Scenario',      # Kinematic Control-Variable Target Scenario (Project a ts trajectory to the jointspace)(Tracking a taskspace trajectory)
    # For Algorithm (Joint Update Rule): 
    'AL'            : 'Algorithm',
    'JI'            : 'Jacobian Inverse',
    'JT'            : 'Jacobian Transpose',
    'JPI'           : 'Jacobian Pseudo Inverse',
    'DLS(CDF)'      : 'Damped Least Squares Method (Constant Damping Factor)',
    'DLS(VDF)'      : 'Damped Least Squares Method (Variable Damping Factor)',

    # For Manipulators: 
    'MAN'           : 'Manipulator',
    'PUMA'          : 'PUMA (6 DOF)',
    'PA10'          : 'Mitsubishi PA10 (7 DOF)',
    'EXO'           : 'DFKI Capio Exoskeleton (9 DOF)',
    'PR2ARM'        : 'Willowgarage PR2 Arm (7 DOF)',
    # For Position Metrics:
    'ICC(xyz)'      : 'Identical Cartesian Coordinates x-y-z', # default
    # For Orientation Metrics:
    'AxInPr(ijk)'   : 'Axis Inner Product (Aligned Axis i-j-k)',
    'AxInPr(ij)'    : 'Axis Inner Product (Aligned Axis i-j)',
    'AxInPr(ik)'    : 'Axis Inner Product (Aligned Axis i-k)',
    'AxInPr(jk)'    : 'Axis Inner Product (Aligned Axis j-k)',
    'ReRoMaTr'      : 'Trace of Relative Rotation Matrix equals three',
    'ReRoMa'        : 'Relative Rotation Matrix equals identity',
    'DiQu'          : 'Difference of Quaternions equals zero',
    'ReQu'          : 'Relative Quaternions equals identity',
    'DiNoQu'        : 'Normalized Difference of Quaternions equals zero',
    'ReRoAn'        : 'Relative Rotation Angle equals zero',
    'ReOrVe(LIN)'   : 'Relative Orientation Vector equals zero (Linear parametrization)',
    'ReOrVe(IDTY)'  : 'Relative Orientation Vector equals zero (Identity parametrization)',
    'ReOrVe(CaGiRo)': 'Relative Orientation Vector equals zero (Cayley-Gibbs-Rodrigues parametrization)' ,
    'ReOrVe(EXP)'   : 'Relative Orientation Vector equals zero (Exponential parametrization)' ,
    'ReOrVe(BaTr)'  : 'Relative Orientation Vector equals zero (Bauchau-Trainelli parametrization)' ,
    'DiRoMa'        : 'Difference of Rotation Matrices equals zero',
    'DiOrVe(IDTY)'  : 'Difference of Orientation Vectors equals zero (Identity parametrization)',
    'DiOrVe(LIN)'   : 'Difference of Orientation Vectors equals zero (Linear parametrization)',
    'DiOrVe(CaGiRo)': 'Difference Orientation Vector equals zero (Cayley-Gibbs-Rodrigues parametrization)' ,
    'DiOrVe(EXP)'   : 'Difference Orientation Vector equals zero (Exponential parametrization)' ,
    'DiOrVe(BaTr)'  : 'Difference Orientation Vector equals zero (Bauchau-Trainelli parametrization)' ,
    # For Joinspace Mapping:
    'NM'            : 'No Mapping', # default
    'LM'            : 'Linear Mapping',
    'CM'            : 'Cosine Mapping',
    'TM'            : 'Tangent Mapping',
    'MM'            : 'Mechanism Mapping',
    # For Workspace Coverage: 
    'WC-TP'         : 'Workspace Coverage for Target Pose',
    'WC-IC'         : 'Workspace Coverage for Initial Configuration',
    # Forward:
    'KC'    : 'Kinematic Constraints',
    'PP'    : 'Precision for Position (mm)',
    'PO'    : 'Precision for Orientation (deg)', 
    'PM'    : 'Position Metric', 
    'OM'    : 'Orientation Metric', 
    'DF'    : 'Damping Factor',
    'MNI'   : 'Maximum Number of Iterations',

    # Inverse:
    'Application Scenario'              : 'AS',
    'Algorithm'                         : 'AL',
    'Manipulator'                       : 'MAN',
    'Kinematic Constraints'             : 'KC',
    'Precision for Position (mm)'       : 'PP',
    'Precision for Orientation (deg)'   : 'PO', 
    'Position Metric'                   : 'PM', 
    'Orientation Metric'                : 'OM', 
    'Damping Factor'                    : 'DF'
}

param_dic = {'ReOrVe(IDTY)':'phi/m', 'ReOrVe(LIN)':'m*sin(phi/m)', 'ReOrVe(CaGiRo)' : 'm*tan(phi/m)', 'ReOrVe(EXP)' : 'm*exp(phi/m)', 'ReOrVe(BaTr)' : '(6*(phi - sin(phi)))**(1.0/3.0)',
             'DiOrVe(IDTY)':'phi/m', 'DiOrVe(LIN)':'m*sin(phi/m)', 'DiOrVe(CaGiRo)' : 'm*tan(phi/m)', 'DiOrVe(EXP)' : 'm*exp(phi/m)', 'DiOrVe(BaTr)' : '(6*(phi - sin(phi)))**(1.0/3.0)'}
def generate_orientation_metric_settings(orientation_constraint):
    func_name = "kinematic.manager.generate_orientation_error_function_package()"

    if orientation_constraint == 'AxInPr(ijk)':
        ms        = metriclib.Metric_Settings(metric_type = 'relative', representation  = 'diag')
        # ms        = metriclib.Metric_Settings(metric_type = 'special', representation  = 'Axis Inner Product')    
        ms.offset = numpy.array([-1.0, -1.0, -1.0])

    elif orientation_constraint == 'AxInPr(ij)':
        # define the weighting matrix. It considers only axis "i" and "j" to be identical. If Axis "i" and "j" are identical, then Axis "k" will be identical as well.
        ms = metriclib.Metric_Settings(metric_type = 'relative', representation  = 'diag')    
        # ms        = metriclib.Metric_Settings(metric_type = 'special', representation  = 'Axis Inner Product')    
        ms.weight   = numpy.array([[1, 0, 0], [0, 1, 0]] )
        # define the corresponding power array. (Please refer to the documentation)
        ms.power    = numpy.array([1, 1, 0])
        ms.offset   = numpy.array([-1.0, -1.0])

    elif orientation_constraint == 'AxInPr(jk)':
        # define the weighting matrix. It considers only axis "j" and "k" to be identical. If Axis "j" and "k" are identical, then Axis "i" will be identical as well.
        self.metric_settings = metriclib.Metric_Settings(category = 'Relative', representation  = 'diag')    
        self.metric_settings.weighting_matrix   = numpy.array([[0, 1, 0], [0, 0, 1]])
        self.metric_settings.power_array        = numpy.array([0, 1, 1])
        self.metric_settings.constant_offset    = numpy.array([-1.0, -1.0])

    elif orientation_constraint == 'AxInPr(ik)':
        # define the weighting matrix. It considers only axis "i" and "k" to be identical. If Axis "i" and "k" are identical, then Axis "j" will be identical as well.
        ms = metriclib.Metric_Settings(metric_type = 'relative', representation  = 'diag')    
        ms.weight   = numpy.array([[1, 0, 0], [0, 0, 1]] )
        # define the corresponding power array. (Please refer to the documentation)
        ms.power    = numpy.array([1, 0, 1])
        ms.offset   = numpy.array([-1.0, -1.0])
        
    elif orientation_constraint == 'ReRoMaTr':
        '''
        represents the orientation error by calculating the trace of relative rotation matrix minus three: Trace(R_a * R_d^T) - 3
        '''
        #Defibe basis error function. It ensures that trace of relative rotation matrix is three or : trace(R_a * R_d^T) = 3.0
        ms = metriclib.Metric_Settings(metric_type = 'relative', representation  = 'trace')
        # define the appropriate weighting matrix. 
        ms.weight             = numpy.array([[1]])
        # define the corresponding power array. (Please refer to the documentation)
        ms.power              = numpy.array([1])
        ms.offset             = numpy.array([- 3.0])
        
    elif orientation_constraint == 'ReRoAn':
        ms = metriclib.Metric_Settings(metric_type = 'relative', representation  = 'angle')
        ms.weight   = numpy.array([[1]])
        ms.power    = numpy.array([1])
        ms.offset   = numpy.array([0])

    elif orientation_constraint in ['ReOrVe(IDTY)', 'ReOrVe(LIN)', 'ReOrVe(CaGiRo)', 'ReOrVe(EXP)', 'ReOrVe(BaTr)']:
        ms = metriclib.Metric_Settings(metric_type = 'relative', representation  = 'vector')
        ms.generating_function = param_dic[orientation_constraint]

    elif orientation_constraint in ['DiOrVe(IDTY)', 'DiOrVe(LIN)', 'DiOrVe(CaGiRo)', 'DiOrVe(EXP)', 'DiOrVe(BaTr)']:
        ms = metriclib.Metric_Settings(metric_type = 'differential', representation  = 'vector')
        ms.generating_function = param_dic[orientation_constraint]

    elif orientation_constraint == 'DiNoQu':
        basis_err_func = 'normalized_differential_quaternions'
        weighting_matrix   = numpy.eye(3)
        power_array        = numpy.array([1, 1, 1])
        constant_offset    = numpy.array([0, 0, 0])
    elif orientation_constraint == 'DiQu':
        ms = metriclib.Metric_Settings(metric_type = 'differential', representation  = 'quaternion')
        ms.weight   = numpy.eye(4)
        ms.power    = numpy.array([1, 1, 1, 1])
        ms.offset   = numpy.array([0, 0, 0, 0])
    elif orientation_constraint == 'DiOrVe(IDTY)':
        basis_err_func = 'differential_vectorial_identity'
        weighting_matrix   = numpy.eye(3)
        power_array        = numpy.array([1, 1, 1])
        constant_offset    = numpy.array([0, 0, 0])
    elif orientation_constraint == 'DiOrVe(LIN)':
        basis_err_func = 'differential_vectorial_linear'
        weighting_matrix   = numpy.eye(3)
        power_array        = numpy.array([1, 1, 1])
        constant_offset    = numpy.array([0, 0, 0])
    elif orientation_constraint == 'ReOrVe(LIN)':
        basis_err_func = 'relative_rotation_vector_linear'
        weighting_matrix   = numpy.eye(3)
        power_array        = numpy.array([1, 1, 1])
        constant_offset    = numpy.array([0, 0, 0])
    elif orientation_constraint == 'ReOrVe(CaGiRo)':
        basis_err_func = 'relative_rotation_vector_Cayley_Gibbs_Rodrigues'
        weighting_matrix   = numpy.eye(3)
        power_array        = numpy.array([1, 1, 1])
        constant_offset    = numpy.array([0, 0, 0])
    elif orientation_constraint == 'DiRoMa':
        ms = metriclib.Metric_Settings(metric_type = 'differential', representation  = 'matrix')
        ms.weight   = numpy.eye(9)
        ms.power    = numpy.ones(9)
        ms.offset   = numpy.zeros(9)
    elif orientation_constraint == 'ReRoMa':
        ms = metriclib.Metric_Settings(metric_type = 'relative', representation  = 'matrix')
        ms.weight   = numpy.eye(9)
        ms.power    = numpy.ones(9)
        ms.offset   = - numpy.eye(3).flatten()
    elif orientation_constraint == 'ReQu':
        ms = metriclib.Metric_Settings(metric_type = 'relative', representation  = 'quaternion')
        ms.weight   = numpy.eye(4)
        ms.power    = numpy.ones(4)
        ms.offset   = numpy.array([- 1.0, 0.0, 0.0, 0.0])
    # Special Metrics
    elif orientation_constraint == 'AxInPr(ijk)+DiNoQu':
        ms        = metriclib.Metric_Settings(metric_type = 'special', representation  = 'AxInPr + DiNoQu')
        ms.weight = numpy.eye(6)
        ms.power  = numpy.ones(6)
        ms.offset = numpy.array([-1.0, -1.0, -1.0, 0.0, 0.0, 0.0])

    else:
        assert False, func_name + ": " + orientation_constraint + " is an unknown value for orientation_constraint"
    
    return ms
        
def generate_position_metric_settings(position_constraint):
    func_name = ".generate_position_metric_settings()"
    if position_constraint == 'ICC(xyz)':
        ms = metriclib.Metric_Settings()    
    else:
        assert False, __name__ + func_name + ": " + position_constraint + " is an unknown value for position_constraint"
    return ms
    
class Kinematic_Manager_Settings():
    '''
    '''
    
#    def __init__(self, manip_name, ik_settings, grid_settings, loging_settings, .. ):
    

    def __init__(self, manip_name):
        '''
        '''
        err_head       = "Error from kinematic_manager.Kinematic_Manager_Settings." 


        self.str_parameter_set = ['AS', 'AL','MAN', 'KC', 'MNI', 'PP', 'PO', 'PM', 'OM', 'WC-TP', 'WC-IC', 'DF']
        self.csv_parameter_set = ['AS', 'AL','MAN', 'KC', 'MNI', 'PP', 'PO', 'PM', 'OM', 'WC-TP', 'WC-IC', 'DF']
        self.key_parameter_set = ['AS', 'AL','MAN', 'KC', 'MNI', 'PM', 'OM', 'WC-TP', 'WC-IC']

        # Name of the manipulator
        self.manip_name = manip_name
        #Number of Kinematic Constraints 
        '''
        This value can reflect the number of endeffectors as well.
        The value of this property has no influence in the model and is designed to be shown in the test key and the table of test specifications
        '''
        self.NKC       = 6 # 6 kinematic constraints: 3 for position and 3 for orientations
        #Number of endeffectors (orientation references)
        self.num_orientation_ref    = 1

        self.application_scenario   = 'PPS' # Pose Projection Scenario
        self.solution_class         = 'Numeric'

        self.orientation_constraint = 'AxInPr(jk)' 
        self.position_constraint    = 'ICC(xyz)'  #Identical Cartesian Coordinates (x,y,z)
        
        # Representation of Orientation Error
        #self.err_func_orient        = 'Axis Inner Product'
               
        self.config_settings                = conflib.Configuration_Settings(default_joint_handling = 'NM')
        self.ik_settings                    = iklib.Inverse_Kinematics_Settings(algorithm = "JPI", run_mode = 'normal_run', num_steps = 100) 
        # Setting Desired Precision in Inverse Kinematics

        self.ik_settings.ef_settings.precision_base_for_position   = "Coordinate Distance"  
        self.ik_settings.ef_settings.precision_base_for_rotation   = "Axis Angle"  

        self.ik_settings.ef_settings.precision_for_position   = 0.01 # coordinate distance in meters or error_function value depending on the precision_base
        self.ik_settings.ef_settings.precision_for_rotation   = 1.0  # axis angle in degrees or error_function value depending on the precision_base


        # When searching for nearest starting config in a precomputed set of poses, "self.workspace_settings_ic.grid_resolution" determine the jointspace grid size by which precomputed poses are generated
        # "number_of_search_items" Determine how many startiong configs are to be found in search (Number of trials) (For each starting config, an IK trial is implemented)
        self.workspace_settings_ic          = wslib.Workspace_Settings(number_of_search_items = 10)
        self.workspace_settings_tp          = wslib.Workspace_Settings(representation_of_orientation = 'Rotation Matrix')

        # In the test evaluation, "self.grid_res_tar_pose" determine the jointspace grid size by which target poses are generated.
        
        import os
        
        ## MANUAL : path = os.getcwd() + "/" + manip_name
        ## via LIB 
        path = os.path.join( os.getcwd(), manip_name+ '_results' ) 
        
        if os.path.exists(path) == False:
            print "path not found"
            os.makedirs(path) 
    
        # similar simplification ... 
        self.path_str =  path + '/'

    def valid(self):
        func_name = 'valid()'
        vld = True
        vld = vld and (self.application_scenario in all_application_scenarios)
        assert vld, self.__class__.err_head + func_name + ": The given application_scenario is not known!"
        return vld        
        
    def __str__( self, parameter_set = None ) : 
        '''
        returns the algorithmic setting for the inverse kinematic evaluator
        '''
        if parameter_set == None:
            parameter_set = self.str_parameter_set
        s =   '\n'
        s  += 'Kinematic Manager Settings:' + '\n' + '\n'
        for p in parameter_set:
            value = self.parameter_value(p)
            param = key_dic[p]
            if p in ['KC','PP','PO', 'DF', 'MNI', 'WC-TP', 'WC-IC']:
                s += param + " "*(30-len(param)) +': ' + value + '\n'
            else:
                s += param + " "*(30-len(param)) + ': ' + key_dic[value] + '\n'
        '''
        s  += 'Application Scenario                : ' + key_dic[self.application_scenario] + '\n'
        s  += 'Algorithm                           : ' + key_dic[self.ik_settings.algorithm] + '\n'
        s  += 'Manipulator                         : ' + self.manip_name + '\n'
        s  += 'Number of position references       : ' + str(self.num_position_ref) + '\n'
        s  += 'Number of orientation references    : ' + str(self.num_orientation_ref) + '\n'
        s  += 'Metric for position                 : ' + key_dic[self.position_constraint] + '\n'
        s  += 'Metric for orientation              : ' + key_dic[self.orientation_constraint] + '\n'
        s  += 'Maximum number of iterations        : ' + str(self.ik_settings.number_of_steps) + '\n' + '\n'

        s  += 'Initial Configuration Workspace: ' + '\n'
        s  += str(self.workspace_settings_ic)  + '\n'
            
        s  += 'Target Pose Workspace: ' + '\n' 
        s  += str(self.workspace_settings_tp)  + '\n'
        '''
        return s
    
    def parameter_value(self, parameter):
        if parameter == 'AS':
            return self.application_scenario
        elif parameter == 'AL':
            return self.ik_settings.algorithm
        elif parameter == 'MAN':
            return self.manip_name
        elif parameter == 'KC':
            return str(self.NKC)
        elif parameter == 'MNI':
            return str(self.ik_settings.number_of_steps)
        elif parameter == 'PP':
            return str(1000*self.ik_settings.ef_settings.precision_for_position)
        elif parameter == 'PO':
            return str(self.ik_settings.ef_settings.precision_for_rotation)
        elif parameter == 'PM':
            return self.position_constraint
        elif parameter == 'OM':
            return self.orientation_constraint
        elif parameter == 'DF':
            return str(self.ik_settings.damping_factor)
        elif parameter == 'WC-TP':
            return self.workspace_settings_tp.gen_key()
        elif parameter == 'WC-IC':
            return self.workspace_settings_ic.gen_key()

        else:
            assert False, parameter + " is an Unknown Setting Parameter"

    def csv(self, header = True, parameter_set = None):
        '''
        returns the algorithmic setting for the inverse kinematic evaluator
        '''
        if parameter_set == None:
            parameter_set = self.csv_parameter_set

        if header:
            s  = 'Setting Parameter' + ',' + 'Parameter Key' + ',' + 'Value' + ',' + 'Interpretation' + '\n'
        else:
            s  = ''
        
        for p in parameter_set:
            value = self.parameter_value(p)
            s    += key_dic[p] + ',' + p + ',' + value + ','
            if not p in ['KC','PP','PO', 'DF', 'MNI', 'WC-TP', 'WC-IC']:
                s += key_dic[value]
            s += '\n'
        return s
        
    def csv_horizontal_header(self, parameter_set = None, use_key = False):
        if parameter_set == None:
            parameter_set = self.csv_parameter_set
        s = ''
        for p in parameter_set:
            if use_key:
                s += p + ","
            else:
                s += key_dic[p] + ","
        s  = s[0:len(s) - 1]
        return s

    def csv_horizontal(self, parameter_set = None):
        if parameter_set == None:
            parameter_set = self.csv_parameter_set
        s = ''
        for p in parameter_set:
            s += self.parameter_value(p) + ','
        s  = s[0:len(s) - 1]

        return s

    def generate_key( self, parameter_set = None):
        '''
        returns the settings key string for the test evaluator
        '''
        s = ''
        if parameter_set == None:
            parameter_set = self.key_parameter_set
        for p in parameter_set:
            if p in ['KC','PP','PO', 'DF', 'MNI', 'WC-TP', 'WC-IC']:
                s     += p + self.parameter_value(p) + '_'
            else:
                s     += self.parameter_value(p) + '_'
        return s[0:len(s) - 1]

        '''
        if self.valid():

            key_str  = "AS:" + self.application_scenario
            rg_str   = self.manip_name
            key_str += "_RG:" + rg_str
            
            key_str += "_AL:" + self.ik_settings.algorithm
            key_str += "_EE:"         
            for i in range(0, self.num_position_ref):
                key_str += 'P'
            for i in range(0, self.num_orientation_ref):
                key_str += 'O'
            
            key_str += "_PRF:" + self.position_constraint + '-' + self.orientation_constraint

            key_str += '_TP:' + self.workspace_settings_tp.gen_key()
            key_str += '_IC:' + self.workspace_settings_ic.gen_key()+'-'+self.workspace_settings_ic.search_criteria_key()
            
            return key_str    
        else:
            print("Error from Kinematic_Manager_Settings.generate_key():Invalid Settings")
            return None
        '''

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
            tp.error.settings = generate_position_metric_settings(self.settings.position_constraint)
        
        for tf in self.inverse_kinematics.endeffector.reference_orientations:
            tf.error.settings = generate_orientation_metric_settings(self.settings.orientation_constraint)
        
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
        
        
    def execute(self, save_evaluation_log = True, save_evaluation_results = False, save_evaluation_csv = True, verbose = False):
        '''
        execute the evaluation according to the settings
        '''

        # set error function for orientation
        #self.inverse_kinematics.number_of_steps              = self.settings.max_num_iters
        #self.inverse_kinematics.configuration.joint_handling = self.settings.joint_map
    
        result_info_detail = str()

        test_key_str = self.settings.generate_key()
    
        if self.settings.workspace_settings_ic.search_criteria == 'Nearest to Current Configuration':
            self.settings.ik_settings.include_current_config = True
        else:    
            self.settings.ik_settings.include_current_config = False
            
        self.evaluate_ik_solver(verbose = verbose)
    
        if save_evaluation_csv:
            self.km_log.write_csv(filename = self.settings.path_str + test_key_str + '.csv')

        if save_evaluation_log:
            self.km_log.write_log(filename = self.settings.path_str + test_key_str + '.log')

        if save_evaluation_results:
            self.km_log.write_self(filename = self.settings.path_str + test_key_str + '.res')
