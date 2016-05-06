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
import general_python as genpy
import magiks.jointspace.manipulator_configuration as conflib 
import magiks.geometry.manipulator_geometry as geolib 

from math_tools.geometry import rotation, pose_metric as metriclib
from magiks.magiks_core import inverse_kinematics as iklib, log_manager as loglib, manipulator_library as manlib
from magiks.taskspace import endeffector as endlib, workspace as wslib

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

# \cond
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
    'DLS(ADF)'      : 'Damped Least Squares Method (Adaptive Damping Factor)',
    'DLS(MADF)'     : 'Damped Least Squares Method (Manipulability Adjusted Damping Factor)',

    # For Manipulators: 
    'MAN'           : 'Manipulator',
    'PUMA'          : 'PUMA (6 DOF)',
    'PA10'          : 'Mitsubishi PA10 (7 DOF)',
    'EXO'           : 'DFKI Capio Exoskeleton (9 DOF)',
    'PR2ARM'        : 'Willowgarage PR2 Arm (7 DOF)',
    # For Position Metrics:
    'DiCaCo(xyz)'   : 'Difference of Cartesian Coordinates x-y-z', # default
    'DiCaCo(xy)'    : 'Difference of Cartesian Coordinates x-y',
    'DiCaCo(xz)'    : 'Identical Cartesian Coordinates x-z',
    'DiCaCo(yz)'    : 'Identical Cartesian Coordinates y-z',
    'DiCaCo(x)'     : 'Identical Cartesian Coordinates x', 
    'DiCaCo(y)'     : 'Identical Cartesian Coordinates y',
    'DiCaCo(z)'     : 'Identical Cartesian Coordinates z',
    # For Orientation Metrics:
    'AxInPr(ijk)'   : 'Axis Inner Product (Aligned Axis i-j-k)',
    'AxInPr(ij)'    : 'Axis Inner Product (Aligned Axis i-j)',
    'AxInPr(ik)'    : 'Axis Inner Product (Aligned Axis i-k)',
    'AxInPr(jk)'    : 'Axis Inner Product (Aligned Axis j-k)',
    'AxInPr(i)'     : 'Axis Inner Product (Aligned Axis i)',
    'AxInPr(j)'     : 'Axis Inner Product (Aligned Axis j)',
    'AxInPr(k)'     : 'Axis Inner Product (Aligned Axis k)',
    'ReRoMaTr'      : 'Trace of Relative Rotation Matrix',
    'ReRoMa'        : 'Relative Rotation Matrix',
    'DiAnAx'        : 'Difference of Angle-Axis',
    'DiUnQu'        : 'Difference of Unit Quaternions',
    'ReAnAx'        : 'Relative Angle-Axis',
    'ReUnQu'        : 'Relative Unit Quaternion',
    'DiNoQu'        : 'Difference of Non-Redundant(Normalized) Quaternions',
    'ReRoAn'        : 'Relative Rotation Angle',
    'ReOrVe(LIN)'   : 'Relative Orientation Vector (Linear parametrization)',
    'ReOrVe(IDTY)'  : 'Relative Orientation Vector (Identity parametrization)',
    'ReOrVe(CaGiRo)': 'Relative Orientation Vector (Cayley-Gibbs-Rodrigues parametrization)' ,
    'ReOrVe(EXP)'   : 'Relative Orientation Vector (Exponential parametrization)' ,
    'ReOrVe(BaTr)'  : 'Relative Orientation Vector (Bauchau-Trainelli parametrization)' ,
    'DiRoMa'        : 'Difference of Rotation Matrices',
    'DiOrVe(IDTY)'  : 'Difference of Orientation Vectors (Identity parametrization)',
    'DiOrVe(LIN)'   : 'Difference of Orientation Vectors (Linear parametrization)',
    'DiOrVe(CaGiRo)': 'Difference of Orientation Vectors (Cayley-Gibbs-Rodrigues parametrization)' ,
    'DiOrVe(EXP)'   : 'Difference of Orientation Vectors (Exponential parametrization)' ,
    'DiOrVe(BaTr)'  : 'Difference of Orientation Vectors (Bauchau-Trainelli parametrization)' ,
     # Special Orientation Metrics:
    'AxInPr + DiNoQu': 'Combined AxInPr and DiNoQu',
    'ReRoAn + ReOrVe(LIN)': 'Combined Relative Angle and Vector (Linear Parametrization)',

     # For Joinspace Mapping:
    'NM'            : 'No Mapping', # default
    'LM'            : 'Linear Mapping',
    'TM'            : 'Trogonometric Mapping',
    'MM'            : 'Mechanism Mapping',
    # For Workspace Coverage: 
    'WCTP'          : 'Workspace Coverage for Target Poses',
    'WCIC'          : 'Workspace Coverage for Initial Configurations',

    # Forward:
    'KC'    : 'Kinematic Constraints',
    'PP'    : 'Precision for Position (mm)',
    'PO'    : 'Precision for Orientation (deg)', 
    'PM'    : 'Position Metric', 
    'OM'    : 'Orientation Metric', 
    'JM'    : 'Joint Mapping', 
    'DF'    : 'Damping Factor',
    'DFG'   : 'Damping Factor Gain',
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
    'Join Mapping'                      : 'JM', 
    'Damping Factor'                    : 'DF',
    'Damping Factor Gain'               : 'DFG'
}


nkc_dic = {'DiCaCo(xyz)':3,'DiCaCo(xy)':2,'DiCaCo(xz)':2,'DiCaCo(yz)':2, 'DiCaCo(x)':1, 'DiCaCo(y)':1,'DiCaCo(z)':1,
           'AxInPr(ijk)':3,'AxInPr(ij)':3,'AxInPr(ik)':3,'AxInPr(jk)':3, 'AxInPr(i)':2, 'AxInPr(j)':2,'AxInPr(k)':2}

param_dic = {'ReOrVe(IDTY)':'identity', 'ReOrVe(LIN)':'linear', 'ReOrVe(CaGiRo)' : 'cayley-gibbs-rodrigues', 'ReOrVe(EXP)' : 'exponential', 'ReOrVe(BaTr)' : 'bauchau-trainelli',
             'DiOrVe(IDTY)':'identity', 'DiOrVe(LIN)':'linear', 'DiOrVe(CaGiRo)' : 'cayley-gibbs-rodrigues', 'DiOrVe(EXP)' : 'exponential', 'DiOrVe(BaTr)' : 'bauchau-trainelli'}

def generate_orientation_metric_settings(orientation_constraint):
    
    if orientation_constraint[0:6] == 'AxInPr':
        # ms        = metriclib.Metric_Settings(metric_type = 'relative', representation  = 'diag')
        ms          = metriclib.Metric_Settings(metric_type = 'special', representation  = 'AxInPr')    
    
        # Set Weighting Matrix and Power Array for AxInPr Group:

        if orientation_constraint == 'AxInPr(ij)':
            ms.weight   = numpy.array([[1, 0, 0], [0, 1, 0]] )
            ms.power    = numpy.array([1, 1, 0])
            ms.required_identical_coordinate = [True, True, False]
        elif orientation_constraint == 'AxInPr(jk)':
            ms.weight   = numpy.array([[0, 1, 0], [0, 0, 1]])
            ms.power    = numpy.array([0, 1, 1])
            ms.required_identical_coordinate = [False, True, True]
        elif orientation_constraint == 'AxInPr(ik)':
            ms.weight   = numpy.array([[1, 0, 0], [0, 0, 1]] )
            ms.power    = numpy.array([1, 0, 1])
            ms.offset   = numpy.array([-1.0, -1.0])
            ms.required_identical_coordinate = [True, False, True]
        elif orientation_constraint == 'AxInPr(i)':
            ms.weight   = numpy.array([[1, 0, 0]] )
            ms.power    = numpy.array([1, 0, 0])
            ms.required_identical_coordinate = [True, False, False]
        elif orientation_constraint == 'AxInPr(j)':
            ms.weight   = numpy.array([[0, 1, 0]] )
            ms.power    = numpy.array([0, 1, 0])
            ms.required_identical_coordinate = [False, True, False]
        elif orientation_constraint == 'AxInPr(k)':
            ms.weight   = numpy.array([[0, 0, 1]] )
            ms.power    = numpy.array([0, 0, 1])
            ms.required_identical_coordinate = [False, False, True]

        if orientation_constraint == 'AxInPr(ijk)':
            ms.offset = - numpy.ones(3)
        elif orientation_constraint in ['AxInPr(ij)', 'AxInPr(ik)', 'AxInPr(jk)']:
            ms.offset = - numpy.ones(2)
        elif orientation_constraint in ['AxInPr(i)', 'AxInPr(j)', 'AxInPr(k)']:
            ms.offset = - numpy.ones(1)
        else:
            assert False

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
        ms.parametrization = param_dic[orientation_constraint]

    elif orientation_constraint in ['DiOrVe(IDTY)', 'DiOrVe(LIN)', 'DiOrVe(CaGiRo)', 'DiOrVe(EXP)', 'DiOrVe(BaTr)']:
        ms = metriclib.Metric_Settings(metric_type = 'differential', representation  = 'vector')
        ms.parametrization = param_dic[orientation_constraint]

    elif orientation_constraint == 'DiNoQu':
        ms        = metriclib.Metric_Settings(metric_type = 'special', representation  = 'DiNoQu')
        ms.weight = numpy.eye(3)
        ms.power  = numpy.ones(3)
        ms.offset = numpy.zeros(3)

    elif orientation_constraint == 'DiUnQu':
        ms = metriclib.Metric_Settings(metric_type = 'differential', representation  = 'quaternion')
        ms.weight   = numpy.eye(4)
        ms.power    = numpy.array([1, 1, 1, 1])
        ms.offset   = numpy.array([0, 0, 0, 0])

    elif orientation_constraint == 'DiAnAx':
        ms = metriclib.Metric_Settings(metric_type = 'differential', representation  = 'angle_axis')
        ms.weight   = numpy.eye(4)
        ms.power    = numpy.array([1, 1, 1, 1])
        ms.offset   = numpy.array([0, 0, 0, 0])

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

    elif orientation_constraint == 'ReUnQu':
        ms = metriclib.Metric_Settings(metric_type = 'relative', representation  = 'quaternion')
        ms.weight   = numpy.eye(4)
        ms.power    = numpy.ones(4)
        ms.offset   = numpy.array([- 1.0, 0.0, 0.0, 0.0])

    elif orientation_constraint == 'ReAnAx':
        ms = metriclib.Metric_Settings(metric_type = 'relative', representation  = 'angle_axis')
        ms.weight   = numpy.eye(4)
        ms.power    = numpy.array([1, 1, 1, 1])
        ms.offset   = numpy.array([0, 0, 0, 0])

    # Special Metrics
    elif orientation_constraint == 'AxInPr(ijk) + DiNoQu':
        ms        = metriclib.Metric_Settings(metric_type = 'special', representation  = 'AxInPr + DiNoQu')
        ms.weight = numpy.eye(6)
        ms.power  = numpy.ones(6)
        ms.offset = numpy.array([-1.0, -1.0, -1.0, 0.0, 0.0, 0.0])

    elif orientation_constraint == 'ReRoAn + ReOrVe(LIN)':
        ms        = metriclib.Metric_Settings(metric_type = 'special', representation  = 'ReRoAn + ReOrVe')
        ms.weight = numpy.eye(4)
        ms.power  = numpy.ones(4)
        ms.offset = numpy.zeros(4)
        ms.parametrization = 'linear'
    else:
        assert False, func_name + ": " + orientation_constraint + " is an unknown value for orientation_constraint"
    
    return ms
        
def generate_position_metric_settings(position_constraint):
    func_name = ".generate_position_metric_settings()"
    if position_constraint == 'DiCaCo(xyz)':
        ms = metriclib.Metric_Settings()    
    elif position_constraint == 'DiCaCo(xy)':
        ms          = metriclib.Metric_Settings()
        ms.weight   = numpy.array([[1, 0, 0], [0, 1, 0]] )
        ms.power    = numpy.array([1, 1, 0])
        ms.offset   = numpy.zeros(2)
        ms.required_identical_coordinate = [True, True, False]
    elif position_constraint == 'DiCaCo(xz)':
        ms          = metriclib.Metric_Settings()
        ms.weight   = numpy.array([[1, 0, 0], [0, 0, 1]] )
        ms.power    = numpy.array([1, 0, 1])
        ms.constant_offset    = numpy.zeros(2)
        ms.required_identical_coordinate = [True, False, True]
    elif position_constraint == 'DiCaCo(yz)':
        ms          = metriclib.Metric_Settings()
        ms.constant_offset    = numpy.zeros(2)
        ms.weighting_matrix   = numpy.array([[0, 1, 0], [0, 0, 1]])
        ms.power_array        = numpy.array([0, 1, 1])
        ms.required_identical_coordinate = [False, True, True]
    elif position_constraint == 'DiCaCo(x)':
        ms          = metriclib.Metric_Settings()
        ms.weight   = numpy.array([[1, 0, 0]] )
        ms.power    = numpy.array([1, 0, 0])
        ms.offset   = numpy.zeros(1)
        ms.required_identical_coordinate = [True, False, False]
    elif position_constraint == 'DiCaCo(y)':
        ms          = metriclib.Metric_Settings()
        ms.weight   = numpy.array([[0, 1, 0]] )
        ms.power    = numpy.array([0, 1, 0])
        ms.offset   = numpy.zeros(1)
        ms.required_identical_coordinate = [False, True, False]
    elif position_constraint == 'DiCaCo(z)':
        ms          = metriclib.Metric_Settings()
        ms.weight   = numpy.array([[0, 0, 1]] )
        ms.power    = numpy.array([0, 0, 1])
        ms.offset   = numpy.numpy.zeros(1)
        ms.required_identical_coordinate = [False, False, True]
    else:
        assert False, __name__ + func_name + ": " + position_constraint + " is an unknown value for position_constraint"
    return ms
    
class Kinematic_Manager_Settings(object):
    '''
    '''
    
#    def __init__(self, manip_name, ik_settings, grid_settings, loging_settings, .. ):
    

    def __init__(self, manip_name):
        '''
        '''
        err_head       = "Error from kinematic_manager.Kinematic_Manager_Settings." 


        self.str_parameter_set = ['AS', 'AL','MAN', 'KC', 'MNI', 'PP', 'PO', 'PM', 'OM', 'JM', 'WCTP', 'WCIC', 'DF', 'DFG']
        self.csv_parameter_set = ['AS', 'AL','MAN', 'KC', 'MNI', 'PP', 'PO', 'PM', 'OM', 'JM', 'WCTP', 'WCIC', 'DF', 'DFG']
        self.key_parameter_set = ['AS', 'AL','MAN', 'KC', 'MNI', 'PM', 'OM', 'JM', 'WCTP', 'WCIC', 'PP','PO', 'DF', 'DFG']

        # Geometry Settings set from the Name of the manipulator
        self.geo_settings           = manlib.manip_geo_settings(manip_name)
        #Number of Kinematic Constraints 
        '''
        This value can reflect the number of endeffectors as well.
        The value of this property has no influence in the model and is designed to be shown in the test key and the table of test specifications
        '''
        #Number of endeffectors (orientation references)
        self.num_position_ref       = 1
        self.num_orientation_ref    = 1

        self.application_scenario   = 'PPS' # Pose Projection Scenario
        self.solution_class         = 'Numeric'

        self.orientation_constraint = 'AxInPr(jk)' 
        self.position_constraint    = 'DiCaCo(xyz)'  #Identical Cartesian Coordinates (x,y,z)
        
        # Representation of Orientation Error
        #self.err_func_orient        = 'Axis Inner Product'

        self.config_settings        = manlib.manip_config_settings(manip_name, joint_mapping = 'NM')
        self.end_settings           = endlib.Endeffector_Settings() 
        self.ik_settings            = iklib.Inverse_Kinematics_Settings(algorithm = "JPI", run_mode = 'normal_run', num_steps = 100) 
        # Setting Desired Precision in Inverse Kinematics
        self.ik_settings.ngp_active = False

        self.end_settings.precision_base_for_position   = "Coordinate Distance"  
        self.end_settings.precision_base_for_rotation   = "Axis Angle"  

        self.end_settings.precision_for_position   = 0.01 # coordinate distance in meters or error_function value depending on the precision_base
        self.end_settings.precision_for_rotation   = 1.0  # axis angle in degrees or error_function value depending on the precision_base


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

    def NKC(self):
        if self.orientation_constraint[0:6] == 'AxInPr':
            noc = nkc_dic[self.orientation_constraint]
        else:
            noc = 3
        return self.num_position_ref*nkc_dic[self.position_constraint] + self.num_orientation_ref*noc

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
            if p in ['KC','PP','PO', 'DF', 'DFG', 'MNI', 'WCTP', 'WCIC']:
                s += param + " "*(45-len(param)) +': ' + value + '\n'
            else:
                s += param + " "*(45-len(param)) + ': ' + key_dic[value] + '\n'
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
            return self.geo_settings.name
        elif parameter == 'KC':
            return str(self.NKC())
        elif parameter == 'MNI':
            return str(self.ik_settings.number_of_steps)
        elif parameter == 'PP':
            return str(1000*self.end_settings.precision_for_position)
        elif parameter == 'PO':
            return str(self.end_settings.precision_for_rotation)
        elif parameter == 'PM':
            if self.num_position_ref == 0:
                return 'NA'
            else:
                return self.position_constraint
        elif parameter == 'OM':
            if self.num_orientation_ref == 0:
                return 'NA'
            else:
                return self.orientation_constraint
        elif parameter == 'JM':
            if 'TM' in self.config_settings.joint_handling:
                return 'TM'
            elif 'LM' in self.config_settings.joint_handling:
                return 'LM'
            else:
                return 'NM'       
            # return genpy.most_common(self.config_settings.joint_handling)
        elif parameter == 'DF':
            if self.ik_settings.algorithm[0:3] == 'DLS':
                return str(self.ik_settings.initial_damping_factor)
            else:
                return 'NA'
        elif parameter == 'DFG':
            if self.ik_settings.algorithm == 'DLS(ADF)':
                return str(self.ik_settings.df_gain)
            elif self.ik_settings.algorithm == 'DLS(MADF)':
                return str(self.ik_settings.manipulability_threshold)
            else:
                return 'NA'
        
        elif parameter == 'WCTP':
            return self.workspace_settings_tp.gen_key()
        elif parameter == 'WCIC':
            return self.workspace_settings_ic.gen_key()
        else:
            assert False, genpy.err_str(__name__, self.__class__.__name__, 'parameter_value', parameter + ' is an Unknown Setting Parameter')

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
            if not p in ['KC','PP','PO', 'DF', 'DFG','MNI', 'WCTP', 'WCIC']:
                s += key_dic[value]
            s += '\n'
        return s
        
    def latex(self, header = True, parameter_set = None):
        '''
        returns the algorithmic setting for the inverse kinematic evaluator in latex format
        '''
        lf = '& \multicolumn{1}{l}{} & \multicolumn{1}{l}{} & \\ \n'
        if parameter_set == None:
            parameter_set = self.csv_parameter_set

        s = '\begin{table}[h]' + '\n' + '\begin{tabular}{lccl}' + '\n' + '\hline' + '\n'

        if header:
            s  += '\multicolumn{1}{c}{\textbf{Setting Parameter}}' + '&' + '\textbf{Parameter Key}' + '&' + '\textbf{Value}' + '&' + '\multicolumn{1}{c}{\textbf{Interpretation}}' + '\\ \hline \n'
            s  += lf 
        for p in parameter_set:
            value = self.parameter_value(p)
            s    += key_dic[p] + '&' + p + '&' + value + '&'
            if not p in ['KC','PP','PO', 'DF', 'DFG','MNI', 'WCTP', 'WCIC']:
                s += key_dic[value]
            s += '\\ \n'
            s += lf
        s += '\hline \n \end{tabular} \n \end{table}'
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
            if p in ['KC','PP','PO', 'DF', 'DFG','MNI', 'WCTP', 'WCIC']:
                s     += p + ':' + self.parameter_value(p) + '_'
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

class Kinematic_Manager(object):
    '''
    
    Kinematic_Manager is the kinematic manager of a manipulator. 
    contains a mathematical kinematic model of a chain of joints and links which can be referred to a robot or any mechanical manipulator,
    the whole or parts of human or animal moving mechanism.
    This class contains all kinematic properties of a manipulator like geometry, joint configuration, transfer matrices, endeffector(s) and the jacobians
    plus other miscellaneous geometric and algorithmic parameters for forward and inverse kinematic computations.
    It also has a method which evaluate the performance of inverse kinematic solver by evaluating it for a set of target poses in the 
    workspace generated from a jointspace grid.
    '''

    def __init__(self, settings):
        '''
        Predefined Class Properties :
        '''
        self.settings           = settings
        self.km_log             = loglib.Test_Log(settings)
        
        # "self.forward_kinematics" is an instance of class "Forward_Kinematics" in package "kinemagic" which contains all kinematic properties of a manipulator which depend on the joint configuration
        # These properties are: transfer matrices, analytic, geometric and error jacobians, taskpoints and taskframes and the pose error vector.

        #self.forward_kinematics = fklib.Forward_Kinematics(geometry, config)      
        if self.settings.workspace_settings_ic.search_criteria == 'NCC':
            self.settings.ik_settings.include_current_config = True
        else:    
            self.settings.ik_settings.include_current_config = False


        self.inverse_kinematics = iklib.Inverse_Kinematics(self.settings.config_settings, self.settings.geo_settings, self.settings.end_settings, self.settings.ik_settings )
        
        #set desired position and orientation constraints:

        for tp in self.inverse_kinematics.task_point:
            tp.error.settings = generate_position_metric_settings(self.settings.position_constraint)
            tp.error.settings.precision = self.inverse_kinematics.end_settings.precision_for_position
        
        for tf in self.inverse_kinematics.task_frame:
            tf.error.settings = generate_orientation_metric_settings(self.settings.orientation_constraint)
            tf.error.settings.precision = self.inverse_kinematics.end_settings.precision_for_rotation

        self.inverse_kinematics.num_task_constraints()
        #self.inverse_kinematics.configuration.set_joint_limits_for(settings.manip_name)
        ###### self.inverse_kinematics.initialize()
        '''
        if self.settings.num_position_ref       == 0:
            self.inverse_kinematics.endeffector.reference_positions = []
        if self.settings.num_orientation_ref    == 0:
            self.inverse_kinematics.endeffector.reference_orientations = []
        '''

        if settings.workspace_settings_ic.read_workspace:    
            print 'Reading Init Config Workspace ... '
            self.workspace_ic = wslib.read_from_file(settings.path_str + settings.workspace_settings_ic.gen_file_name(geometry.name))
            self.workspace_ic.settings = self.settings.workspace_settings_ic
            print 'Reading Init Config Workspace Finished '
        else:    
            self.workspace_ic = wslib.Workspace(settings.config_settings, settings.geo_settings, settings.end_settings, settings.workspace_settings_ic)
            self.workspace_ic.create()
            
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
            self.workspace_tp = wslib.Workspace(settings.config_settings, settings.geo_settings, settings.end_settings, settings.workspace_settings_tp)
            self.workspace_tp.create()
        if settings.workspace_settings_tp.write_workspace:    
            print 'Saving target pose workspace to : ', settings.path_str + settings.workspace_settings_tp.gen_file_name(geometry.name) + ' ... '
            self.workspace_tp.write_to_file(settings.path_str + settings.workspace_settings_tp.gen_file_name(geometry.name))
            print 'Finished saving workspace.'

        #self.inverse_kinematics.settings.number_of_steps = settings.max_num_iters
        
        #self.inverse_kinematics.configuration.joint_handling = settings.joint_map
        
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
            self.inverse_kinematics.tuple_to_pose('desired', pose_tuple, self.settings.workspace_settings_tp.representation_of_orientation)

            #take the manipulator to stored initial state
            #self.inverse_kinematics.take_to_grid_configuration(0, self.settings.workspace_settings_ic.grid_resolution)

            start_kinematic_inversion = time.time()

            #find a list of the best initial configurations:
            
            self.inverse_kinematics.initial_config_list = self.workspace_ic.nearest_configs(self.inverse_kinematics)
            
            #implement the inverse kinematic solver
            self.inverse_kinematics.goto_target()

            elapsed_kinematic_inversion = time.time() - start_kinematic_inversion

            self.workspace_tp.config_list[cnt] = numpy.copy(self.inverse_kinematics.q)

            (running_time, number_of_steps) = (self.inverse_kinematics.run_log.time_elapsed, len(self.inverse_kinematics.run_log.step))

            if number_of_steps == 0:
                number_of_steps = 1
            average_step_time = running_time / number_of_steps
            n_iters_total = n_iters_total + number_of_steps

            if self.inverse_kinematics.in_target():
                n_suc = n_suc + 1

            time_total = time_total + elapsed_kinematic_inversion

            # Retrieve initial status from which the solution is achieved:
            ik_initial = copy.deepcopy(self.inverse_kinematics)
            assert ik_initial.set_config(self.inverse_kinematics.initial_config)
            if ik_initial.in_target():
                print "Already in Target!"

            start_config_str = ik_initial.config_str()
            start_pose_str   = str(ik_initial)
            # Final status:
            final_config_str = self.inverse_kinematics.config_str()
            final_pose_str   = str(self.inverse_kinematics)

            run_log = loglib.Single_Run_Log(cnt, success = self.inverse_kinematics.in_target(), config_in_range =  self.inverse_kinematics.joints_in_range(self.inverse_kinematics.free_config(self.inverse_kinematics.q)))
            run_log.num_trial           = 1 + int(number_of_steps/self.inverse_kinematics.ik_settings.number_of_steps)
            run_log.num_suc_til_now     = n_suc
            run_log.num_iter            = number_of_steps
            run_log.num_iter_til_now    = n_iters_total
            run_log.run_time            = elapsed_kinematic_inversion
            run_log.run_time_til_now    = time_total
            run_log.mean_stp_time       = average_step_time
            run_log.start_config_str    = start_config_str
            run_log.final_config_str    = final_config_str
            run_log.start_pose_str      = start_pose_str
            run_log.final_pose_str      = final_pose_str

            self.km_log.body.append(run_log)

            cnt += 1

            #test_result_list.append(result_info_detail)
            if verbose:
                print str(self.km_log.body[cnt - 1])
                print 'Log: (Running Time, Number of Iterations)', (running_time, number_of_steps)
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

        if self.settings.workspace_settings_ic.search_criteria == 'NCC': # Nearest to Current Configuration
            assert self.inverse_kinematics.ik_settings.include_current_config, "Error todo"
        else:    
            assert not self.inverse_kinematics.ik_settings.include_current_config, "Error todo"

        self.evaluate_ik_solver(verbose = verbose)
    
        if save_evaluation_csv:
            self.km_log.write_csv(filename = self.settings.path_str + test_key_str + '.csv')

        if save_evaluation_log:
            self.km_log.write_log(filename = self.settings.path_str + test_key_str + '.log')

        if save_evaluation_results:
            self.km_log.write_self(filename = self.settings.path_str + test_key_str + '.res')

# \endcond
