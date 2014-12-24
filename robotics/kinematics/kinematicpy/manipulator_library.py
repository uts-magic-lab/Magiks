'''   Header
@file:          manipulator_library.py
@brief:    	    Contains a function that defines geometric parameters and joint limits for four known manipulators (PUMA, PA10, EXO,  AILA Arm and PR2 Arm)
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
Last Revision:  10 December 2014
'''
'''
Main Changes from previous version:
    The library contains instances of config and geometry settings for various manipulators
'''
import numpy, math

import packages.nima.robotics.kinematics.kinematicpy.geometry as geolib 
import packages.nima.mathematics.global_variables as mgv # A library of mathemathical global variables 
import packages.nima.robotics.kinematics.kinematicpy.forward_kinematics as fklib 
import packages.nima.robotics.kinematics.kinematicpy.inverse_kinematics as iklib 
import packages.nima.robotics.kinematics.joint_space.configuration as conflib 

# Geometry of PUMA:
PUMA_geometry_settings         = geolib.Manipulator_Geometry(6, 'PUMA')
PUMA_geometry_settings.theta   = mgv.deg_to_rad*numpy.array([   90.00, -45.00,   45.00,   0.00,   0.00,  0.00  ])
PUMA_geometry_settings.alpha   = mgv.deg_to_rad*numpy.array([  -90.00,   0.00,   90.00, -90.00,  90.00,  0.00  ])
PUMA_geometry_settings.a       =                       numpy.array([  0.0000, 0.4318, -0.0203, 0.0000, 0.0000,  0.0000])
PUMA_geometry_settings.d       =                       numpy.array([  0.0000, 0.1491,  0.0000, 0.4331, 0.0000,  0.0562])

# Geometry of PA10:
PA10_geometry_settings         = geolib.Manipulator_Geometry(7, 'PA10')
PA10_geometry_settings.theta   = mgv.deg_to_rad*numpy.array([     0.00,    0.00,   0.00,    0.00,   0.00,   0.00,   0.00])
PA10_geometry_settings.alpha   = mgv.deg_to_rad*numpy.array([    -90.00,   90.00,  -90.00,   90.00,  -90.00,  90.00,   0.00])
PA10_geometry_settings.a       =                       numpy.array([    0.0000,  0.0000,  0.0000,  0.0000,  0.0000, 0.0000, 0.0000])
PA10_geometry_settings.d       =                       numpy.array([  0.3150,  0.0000, 0.4500,  0.0000, 0.5000, 0.0000, 0.0800])

# Geometry of PA10 Reduced:
PA10R_geometry_settings        = geolib.Manipulator_Geometry(5, 'PA10R')
PA10R_geometry_settings.theta  = mgv.deg_to_rad*numpy.array([     0.00,    0.00,   0.00,    0.00,   0.00])
PA10R_geometry_settings.alpha  = mgv.deg_to_rad*numpy.array([    -90.00,   90.00,  -90.00,   90.00,  -90.00])
PA10R_geometry_settings.a      =                       numpy.array([    0.0000,  0.0000,  0.0000,  0.0000,  0.0000])
PA10R_geometry_settings.d      =                       numpy.array([  0.3150,  0.0000, 0.4500,  0.0000, 0.5000 ])
PA10R_geometry_settings.d6     = 0.08

# Geometry of PR2ARM:
PR2ARM_geometry_settings       = geolib.Manipulator_Geometry(7, 'PR2ARM')
PR2ARM_geometry_settings.theta = mgv.deg_to_rad*numpy.array([     0.00,     0.00,   0.00,    0.00,   0.00,   0.00,   0.00])
PR2ARM_geometry_settings.alpha = mgv.deg_to_rad*numpy.array([    -90.00,   90.00,  -90.00,   90.00,  -90.00,  90.00,   0.00])
PR2ARM_geometry_settings.a     =                       numpy.array([    0.1000,  0.0000,  0.0000,  0.0000,  0.0000, 0.0000, 0.0000])
PR2ARM_geometry_settings.d     =                       numpy.array([    0.0000,  0.0000, 0.4000,  0.0000, 0.3210, 0.0000, 0.0000])

# Geometry of DFKI Exoskeleton:
EXO_geometry_settings          = geolib.Manipulator_Geometry(15, 'EXO')
EXO_geometry_settings.theta    = mgv.deg_to_rad*numpy.array([ 0.0 , 0.0 , 0.0 , 77.0, 36.0, 0.0 , 62.0, 43.0, 0.0 , 0.0, 0.0, 0.0 , 0.0, 0.0, 0.0])
EXO_geometry_settings.alpha    = mgv.deg_to_rad*numpy.array([ 90.0, 90.0, 90.0,-90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 0.0, 0.0, 90.0, 0.0, 0.0 ,90.0])
EXO_geometry_settings.a        =                       numpy.array([0.000, 0.000, 0.000, 0.000, 0.046, 0.000, 0.000, 0.000, 0.046, 0.000, 0.000, -0.120, 0.000, 0.000 ,0.000])
EXO_geometry_settings.d        =                       numpy.array([0.048, 0.205, 0.000, 0.000, 0.046, 0.050, 0.078, 0.050, 0.278, 0.129, 0.106,  0.000, 0.095, 0.125 ,0.100])

# Geometry of DFKI AILA:
AILA_geometry_settings         = geolib.Manipulator_Geometry(7, 'AILA')
AILA_geometry_settings.theta   = mgv.deg_to_rad*numpy.array([  0.00,    0.00,   0.00,    0.00,   0.00,   0.00,   0.00])
AILA_geometry_settings.alpha   = [math.pi/2, -math.pi/2, math.pi/2, -math.pi/2, math.pi/2, -math.pi/2, math.pi /2]
AILA_geometry_settings.a       = numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
AILA_geometry_settings.d       = numpy.array([214.0174059160209765, 0.0, 284.9916407054798242, 0.0, 288.6165426053090641, 0.0, 0.0])

# Geometry of DFKI ROT:

ROT_geometry_settings          = geolib.Manipulator_Geometry(5, 'ROT')
ROT_geometry_settings.theta    = mgv.deg_to_rad*numpy.array([     0.00,    0.00,   0.00,    -90.00,   90.00 ])
ROT_geometry_settings.alpha    = mgv.deg_to_rad*numpy.array([    -90.00,   90.00,  -90.00,   90.00,  -90.00 ])
ROT_geometry_settings.a        =                       numpy.array([    0.0000,  0.0000,  0.0000,  0.1000,  0.0000 ])
ROT_geometry_settings.d        =                       numpy.array([    0.4000,  0.0000,  0.5000,  0.0000,  0.0000 ])


# Configuration Settings:

# For PA10:
PA10_config_settings                    = conflib.Configuration_Settings(njoint = 7, DOF = 7)
PA10_config_settings.joint_handling     = [PA10_config_settings.default_joint_handling for i in range(PA10_config_settings.DOF)]
PA10_config_settings.joint_handling[4]  = 'NM'
PA10_config_settings.joint_handling[6]  = 'NM'
PA10_config_settings.ql = mgv.deg_to_rad*numpy.array([ -177.00, -91.00, -174.00,  -137.00,  -180.00,   -165.00, -180.00])
PA10_config_settings.qh = mgv.deg_to_rad*numpy.array([  177.00,  91.00,  174.00,   137.00,   180.00,    165.00,  180.00])

# For PA10 Reduced:
PA10R_config_settings                    = conflib.Configuration_Settings(njoint = 5, DOF = 5)
PA10R_config_settings.joint_handling     = [PA10R_config_settings.default_joint_handling for i in range(PA10R_config_settings.DOF)]
PA10R_config_settings.joint_handling[4]  = 'NM'
PA10R_config_settings.ql = mgv.deg_to_rad*numpy.array([ -177.00, -91.00, -174.00,  -137.00,  -180.00])
PA10R_config_settings.qh = mgv.deg_to_rad*numpy.array([  177.00,  91.00,  174.00,   137.00,   180.00])

# For PR2ARM:
PR2ARM_config_settings                   = conflib.Configuration_Settings(njoint = 7, DOF = 7)
PR2ARM_config_settings.joint_handling    = [PR2ARM_config_settings.default_joint_handling for i in range(PR2ARM_config_settings.DOF)]
PR2ARM_config_settings.joint_handling[4] = 'NM'
PR2ARM_config_settings.joint_handling[6] = 'NM'
PR2ARM_config_settings.ql = mgv.deg_to_rad*numpy.array([- 130.00,  60.00, -180.00,   0.00, -180.00,   0.00, -180.00])
PR2ARM_config_settings.qh = mgv.deg_to_rad*numpy.array([   40.00, 170.00,   44.00, 130.00,  180.00, 130.00,  180.00])

# For PUMA:
PUMA_config_settings                   = conflib.Configuration_Settings(njoint = 6, DOF = 6)
PUMA_config_settings.joint_handling    = [ PUMA_config_settings.default_joint_handling for i in range(PUMA_config_settings.DOF)]
PUMA_config_settings.joint_handling[5] = 'NM'
PUMA_config_settings.ql = mgv.deg_to_rad*numpy.array([   -160.00,-180.00,  -90.00,-110.00,-100.00, -180.00])
PUMA_config_settings.qh = mgv.deg_to_rad*numpy.array([    160.00,  90.00,  180.00, 170.00, 100.00,  180.00])

# For DFKI Exoskeleton:
EXO_config_settings                   = conflib.Configuration_Settings(njoint = 15, DOF = 9)
EXO_config_settings.joint_handling    = [EXO_config_settings.default_joint_handling for i in range(EXO_config_settings.DOF)]
EXO_config_settings.joint_handling[3] = 'NM'
EXO_config_settings.joint_handling[4] = 'NM'
EXO_config_settings.joint_handling[5] = 'NM'
EXO_config_settings.joint_handling[7] = 'NM'
EXO_config_settings.joint_handling[8] = 'NM'
EXO_config_settings.ql = mgv.deg_to_rad*numpy.array([-12.0,-14.0, 0.0,-180.0,-180.0,-180.0,-180.0,-180.0,-180.0,-180.0,-180.0,-180.0,45.0 ,-180.0,-180.0])
EXO_config_settings.qh = mgv.deg_to_rad*numpy.array([ 17.0,  8.0, 0.2, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0,120.0, 180.0, 180.0])
'''
     Joint Number : 0      1     2    3      4      5      6      7      8      9      10     11     12    13     14
Free Joint Number : 0      1     2                  3                    4              5             6     7      8
'''
EXO_config_settings.qh[2]           = 0.2
EXO_config_settings.prismatic[2]    = True
EXO_config_settings.free[3]         = False
EXO_config_settings.free[4]         = False
EXO_config_settings.free[6]         = False
EXO_config_settings.free[7]         = False
EXO_config_settings.free[9]         = False
EXO_config_settings.free[11]        = False

# For DFKI AILA:
AILA_config_settings                = conflib.Configuration_Settings(njoint = 7, DOF = 7)
AILA_config_settings.joint_handling = [AILA_config_settings.default_joint_handling for i in range(AILA_config_settings.DOF)]
AILA_config_settings.ql = mgv.deg_to_rad*numpy.array([ -90.0, -90.0, -90.0, -90.0, -90.0, -15.0, -45.0])
AILA_config_settings.qh = mgv.deg_to_rad*numpy.array([  90.0,  90.0,  90.0,  90.0,  90.0,  15.0,  45.0])

# For DFKI ROT:
ROT_config_settings                = conflib.Configuration_Settings(njoint = 5, DOF = 5)
ROT_config_settings.joint_handling = [ ROT_config_settings.default_joint_handling for i in range(ROT_config_settings.DOF)]
ROT_config_settings.ql = mgv.deg_to_rad*numpy.array([ -180.0, -90.0, -180.0, -90.0, -90.0])
ROT_config_settings.qh = mgv.deg_to_rad*numpy.array([  180.0,  90.0,  180.0,  90.0,  90.0])

def manipulator_geometry(manip_name): 
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
        
def manipulator_configuration(manip_name): 
    if manip_name == 'PUMA':
        settings = PUMA_config_settings
    elif manip_name == 'PA10':
        settings = PA10_config_settings
    elif manip_name == 'PA10R':
        settings = PA10R_config_settings
    elif manip_name == 'EXO':
        settings = EXO_config_settings
    elif manip_name == 'AILA':
        settings = AILA_config_settings
    elif manip_name == 'ROT':
        settings = ROT_config_settings
    elif manip_name == 'PR2ARM':
        settings = PR2ARM_config_settings
    else : 
        assert False, "Error from manipulator_configuration(): " + manip_name + " is an Unknown Manipulator"
        
    model_config   = conflib.Configuration(settings)
    model_config.q = 0.5*(settings.qh + settings.ql)
    model_config.initialize()
    model_config.update_joint_limit_jacobian_multipliers()

    return model_config
