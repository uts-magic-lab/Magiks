# HEADER

## @file        	manipulator_library.py
#  @brief           Contains a function that defines geometric parameters and joint limits for a number of known manipulators 
#                   Some examples: (PUMA, PA10, EXO,  AILA Arm and PR2 Arm)
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

import numpy, math

import magiks.geometry.manipulator_geometry as geolib 
import math_tools.general_math as genmath # A library of mathemathical global variables 
import magiks.jointspace.manipulator_configuration as conflib 

# Geometry of PUMA:
PUMA_geometry_settings         = geolib.Manipulator_Geometry_Settings(6, 'PUMA')
PUMA_geometry_settings.theta   = genmath.deg_to_rad*numpy.array([   90.00, -45.00,   45.00,   0.00,   0.00,  0.00  ])
PUMA_geometry_settings.alpha   = genmath.deg_to_rad*numpy.array([  -90.00,   0.00,   90.00, -90.00,  90.00,  0.00  ])
PUMA_geometry_settings.a       =                       numpy.array([  0.0000, 0.4318, -0.0203, 0.0000, 0.0000,  0.0000])
PUMA_geometry_settings.d       =                       numpy.array([  0.0000, 0.1491,  0.0000, 0.4331, 0.0000,  0.0562])

# Geometry of PA10:
PA10_geometry_settings         = geolib.Manipulator_Geometry_Settings(7, 'PA10')
PA10_geometry_settings.theta   = genmath.deg_to_rad*numpy.array([     0.00,    0.00,   0.00,    0.00,   0.00,   0.00,   0.00])
PA10_geometry_settings.alpha   = genmath.deg_to_rad*numpy.array([    -90.00,   90.00,  -90.00,   90.00,  -90.00,  90.00,   0.00])
PA10_geometry_settings.a       =                       numpy.array([    0.0000,  0.0000,  0.0000,  0.0000,  0.0000, 0.0000, 0.0000])
PA10_geometry_settings.d       =                       numpy.array([  0.3150,  0.0000, 0.4500,  0.0000, 0.5000, 0.0000, 0.0800])

# Geometry of PA10 Reduced:
PA10R_geometry_settings        = geolib.Manipulator_Geometry_Settings(5, 'PA10R')
PA10R_geometry_settings.theta  = genmath.deg_to_rad*numpy.array([     0.00,    0.00,   0.00,    0.00,   0.00])
PA10R_geometry_settings.alpha  = genmath.deg_to_rad*numpy.array([    -90.00,   90.00,  -90.00,   90.00,  -90.00])
PA10R_geometry_settings.a      =                       numpy.array([    0.0000,  0.0000,  0.0000,  0.0000,  0.0000])
PA10R_geometry_settings.d      =                       numpy.array([  0.3150,  0.0000, 0.4500,  0.0000, 0.5000 ])
PA10R_geometry_settings.d6     = 0.08

# Geometry of PR2ARM:
PR2ARM_geometry_settings       = geolib.Manipulator_Geometry_Settings(7, 'PR2ARM')
PR2ARM_geometry_settings.theta = genmath.deg_to_rad*numpy.array([     0.00,     0.00,   0.00,    0.00,   0.00,   0.00,   0.00])
PR2ARM_geometry_settings.alpha = genmath.deg_to_rad*numpy.array([    -90.00,   90.00,  -90.00,   90.00,  -90.00,  90.00,   0.00])
PR2ARM_geometry_settings.a     =                       numpy.array([    0.1000,  0.0000,  0.0000,  0.0000,  0.0000, 0.0000, 0.0000])
PR2ARM_geometry_settings.d     =                       numpy.array([    0.0000,  0.0000, 0.4000,  0.0000, 0.3210, 0.0000, 0.0000])

# Geometry of DFKI Exoskeleton:
EXO_geometry_settings          = geolib.Manipulator_Geometry_Settings(15, 'EXO')
EXO_geometry_settings.theta    = genmath.deg_to_rad*numpy.array([ 0.0 , 0.0 , 0.0 , 77.0, 36.0, 0.0 , 62.0, 43.0, 0.0 , 0.0, 0.0, 0.0 , 0.0, 0.0, 0.0])
EXO_geometry_settings.alpha    = genmath.deg_to_rad*numpy.array([ 90.0, 90.0, 90.0,-90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 0.0, 0.0, 90.0, 0.0, 0.0 ,90.0])
EXO_geometry_settings.a        =                numpy.array([0.000, 0.000, 0.000, 0.000, 0.046, 0.000, 0.000, 0.000, 0.046, 0.000, 0.000, -0.120, 0.000, 0.000 ,0.000])
EXO_geometry_settings.d        =                numpy.array([0.048, 0.205, 0.000, 0.000, 0.046, 0.050, 0.078, 0.050, 0.278, 0.129, 0.106,  0.000, 0.095, 0.125 ,0.100])

# Geometry of DFKI AILA:
AILA_geometry_settings         = geolib.Manipulator_Geometry_Settings(7, 'AILA')
AILA_geometry_settings.theta   = genmath.deg_to_rad*numpy.array([  0.00,    0.00,   0.00,    0.00,   0.00,   0.00,   0.00])
AILA_geometry_settings.alpha   = [math.pi/2, -math.pi/2, math.pi/2, -math.pi/2, math.pi/2, -math.pi/2, math.pi /2]
AILA_geometry_settings.a       = numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
AILA_geometry_settings.d       = numpy.array([214.0174059160209765, 0.0, 284.9916407054798242, 0.0, 288.6165426053090641, 0.0, 0.0])

# Geometry of DFKI ROT:

ROT_geometry_settings          = geolib.Manipulator_Geometry_Settings(5, 'ROT')
ROT_geometry_settings.theta    = genmath.deg_to_rad*numpy.array([     0.00,    0.00,   0.00,    -90.00,   90.00 ])
ROT_geometry_settings.alpha    = genmath.deg_to_rad*numpy.array([    -90.00,   90.00,  -90.00,   90.00,  -90.00 ])
ROT_geometry_settings.a        =                       numpy.array([    0.0000,  0.0000,  0.0000,  0.1000,  0.0000 ])
ROT_geometry_settings.d        =                       numpy.array([    0.4000,  0.0000,  0.5000,  0.0000,  0.0000 ])


# Configuration Settings:

# For PA10:
def PA10_config_settings(joint_mapping):
    cs                    = conflib.Manipulator_Configuration_Settings(njoint = 7, DOF = 7, joint_mapping = joint_mapping)
    cs.limited[4] = False
    cs.limited[6] = False
    cs.joint_handling[4] = 'NM'
    cs.joint_handling[6] = 'NM'

    cs.ql = genmath.deg_to_rad*numpy.array([ -177.00, -91.00, -174.00,  -137.00,  -180.00,   -165.00, -180.00])
    cs.qh = genmath.deg_to_rad*numpy.array([  177.00,  91.00,  174.00,   137.00,   180.00,    165.00,  180.00])

    return cs

# For PA10 Reduced:
def PA10R_config_settings(joint_mapping):
    cs                   = conflib.Manipulator_Configuration_Settings(njoint = 5, DOF = 5, joint_mapping = joint_mapping)
    cs.limited[4]        = False
    cs.joint_handling[4] = 'NM'
    cs.ql = genmath.deg_to_rad*numpy.array([ -177.00, -91.00, -174.00,  -137.00,  -180.00])
    cs.qh = genmath.deg_to_rad*numpy.array([  177.00,  91.00,  174.00,   137.00,   180.00])
    return cs

# For PR2ARM:
def PR2ARM_config_settings(joint_mapping):
    cs                   = conflib.Manipulator_Configuration_Settings(njoint = 7, DOF = 7, joint_mapping = joint_mapping)
    cs.limited[4] = False
    cs.limited[6] = False
    cs.joint_handling[4] = 'NM'
    cs.joint_handling[6] = 'NM'
    
    cs.joint_label = ['Shoulder Pan Joint', 'Shoulder Lift Joint', 'Upper Arm Roll Joint', 
                      'Shoulder Pan Joint', 'Elbow Flex Joint', 'Forearm Roll Joint', 
                      'Wrist Flex Joint', 'Wrist Roll Joint']

    cs.ql = genmath.deg_to_rad*numpy.array([- 130.00,  60.00, -180.00,   0.00, -180.00,   0.00, -180.00])
    cs.qh = genmath.deg_to_rad*numpy.array([   40.00, 170.00,   44.00, 130.00,  180.00, 130.00,  180.00])
    return cs

# For PUMA:
def PUMA_config_settings(joint_mapping):
    cs                   = conflib.Manipulator_Configuration_Settings(njoint = 6, DOF = 6, joint_mapping = joint_mapping)
    cs.joint_handling[5] = 'NM'
    cs.joint_handling[5] = 'NM'
    cs.ql = genmath.deg_to_rad*numpy.array([   -160.00,-180.00,  -90.00,-110.00,-100.00, -180.00])
    cs.qh = genmath.deg_to_rad*numpy.array([    160.00,  90.00,  180.00, 170.00, 100.00,  180.00])
    return cs

# For DFKI Exoskeleton:
def EXO_config_settings(joint_mapping):
    cs            = conflib.Manipulator_Configuration_Settings(njoint = 15, DOF = 9, joint_mapping = joint_mapping)
    cs.limited[3] = False
    cs.limited[4] = False
    cs.limited[5] = False
    cs.limited[7] = False
    cs.limited[8] = False
    cs.joint_handling[3] = 'NM'
    cs.joint_handling[4] = 'NM'
    cs.joint_handling[5] = 'NM'
    cs.joint_handling[7] = 'NM'
    cs.joint_handling[8] = 'NM'

    cs.ql = genmath.deg_to_rad*numpy.array([-12.0,-14.0, 0.0,-180.0,-180.0,-180.0,-180.0,-180.0,-180.0,-180.0,-180.0,-180.0,45.0 ,-180.0,-180.0])
    cs.qh = genmath.deg_to_rad*numpy.array([ 17.0,  8.0, 0.2, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0,120.0, 180.0, 180.0])
    cs.qh[2] = 0.2

    '''
         Joint Number : 0      1     2    3      4      5      6      7      8      9      10     11     12    13     14
    Free Joint Number : 0      1     2                  3                    4              5             6     7      8
    '''
    cs.qh[2]           = 0.2
    cs.prismatic[2]    = True
    cs.free[3]         = False
    cs.free[4]         = False
    cs.free[6]         = False
    cs.free[7]         = False
    cs.free[9]         = False
    cs.free[11]        = False
    return cs

# For DFKI AILA:
def AILA_config_settings(joint_mapping):
    cs                = conflib.Configuration_Settings(njoint = 7, DOF = 7, joint_mapping = joint_mapping)
    cs.ql = genmath.deg_to_rad*numpy.array([ -90.0, -90.0, -90.0, -90.0, -90.0, -15.0, -45.0])
    cs.qh = genmath.deg_to_rad*numpy.array([  90.0,  90.0,  90.0,  90.0,  90.0,  15.0,  45.0])
    return cs

# For DFKI ROT:
def ROT_config_settings(joint_mapping):
    cs                = conflib.Configuration_Settings(njoint = 5, DOF = 5,joint_mapping = joint_mapping)
    cs.ql = genmath.deg_to_rad*numpy.array([ -180.0, -90.0, -180.0, -90.0, -90.0])
    cs.qh = genmath.deg_to_rad*numpy.array([  180.0,  90.0,  180.0,  90.0,  90.0])
    return cs
    
def manip_geo_settings(manip_name): 
    if manip_name == 'PUMA':
        return PUMA_geometry_settings
    elif manip_name == 'PA10':
        return PA10_geometry_settings
    elif manip_name == 'PA10R':
        return PA10R_geometry_settings
    elif manip_name == 'EXO':
        return EXO_geometry_settings
    elif manip_name == 'AILA':
        return AILA_geometry_settings
    elif manip_name == 'ROT':
        return ROT_geometry_settings
    elif manip_name == 'PR2ARM':
        return PR2ARM_geometry_settings
    else: 
        assert False, "Error from manipulator_geometry(): " + manip_name + " is an Unknown Manipulator"
        
def manip_config_settings(manip_name, joint_mapping = 'TM'): 
    if manip_name == 'PUMA':
        settings = PUMA_config_settings(joint_mapping)
    elif manip_name == 'PA10':
        settings = PA10_config_settings(joint_mapping)
    elif manip_name == 'PA10R':
        settings = PA10R_config_settings(joint_mapping)
    elif manip_name == 'EXO':
        settings = EXO_config_settings(joint_mapping)
    elif manip_name == 'AILA':
        settings = AILA_config_settings(joint_mapping)
    elif manip_name == 'ROT':
        settings = ROT_config_settings(joint_mapping)
    elif manip_name == 'PR2ARM':
        settings = PR2ARM_config_settings(joint_mapping)
    else : 
        assert False, "Error from manipulator_configuration(): " + manip_name + " is an Unknown Manipulator"
        
    return settings
