'''   Header
@file:          manipulator_library.py
@brief:    	    Contains a function that defines geometric parameters and joint limits for four known manipulators (PUMA, PA10, EXO and AILA)
@author:        Nima Ramezani; DFKI Bremen
@start date:    February 2011
@version:	    0.2
Last Revision:  31 May 2011
'''

import numpy, math

from packages.dfki.nima.mathematics import global_variables as mgvlib

import packages.dfki.nima.robotics.kinematics.kinematicpy.geometry as geolib 
import packages.dfki.nima.mathematics.global_variables as mgv # A library of mathemathical global variables 
import packages.dfki.nima.robotics.kinematics.kinematicpy.forward_kinematics as fklib 
import packages.dfki.nima.robotics.kinematics.kinematicpy.inverse_kinematics as iklib 
import packages.dfki.nima.robotics.kinematics.joint_space.configuration as conflib 

class Manipulator_Geometry_PUMA( geolib.Manipulator_Geometry ) : 
    '''
    '''
    def __init__( self ) :
        '''
        Define the DH parameters for manipulator PUMA560 with 6 DOF
        '''
        number_of_joints = 6 
        geolib.Manipulator_Geometry.__init__( self, number_of_joints, 'PUMA' ) 

        self.theta = mgv.deg_to_rad_coeff*numpy.array([   90.00, -45.00,   45.00,   0.00,   0.00,  0.00  ])
        self.alpha = mgv.deg_to_rad_coeff*numpy.array([  -90.00,   0.00,   90.00, -90.00,  90.00,  0.00  ])
        self.a    =                       numpy.array([  0.0000, 0.4318, -0.0203, 0.0000, 0.0000,  0.0000])
        self.d    =                       numpy.array([  0.0000, 0.1491,  0.0000, 0.4331, 0.0000,  0.0562])


class Manipulator_Geometry_PA10( geolib.Manipulator_Geometry ) : 
    '''
    '''
    def __init__( self ) :
        '''
        Define the DH parameters for manipulator PA10 with 7 DOF
        '''
        number_of_joints = 7
        geolib.Manipulator_Geometry.__init__( self, number_of_joints, 'PA10' ) 

        self.theta = mgv.deg_to_rad_coeff*numpy.array([     0.00,    0.00,   0.00,    0.00,   0.00,   0.00,   0.00])
        #self.alpha = mgv.deg_to_rad_coeff*numpy.array([    90.00,  -90.00,  90.00,  -90.00,  90.00, -90.00,  90.00])
        self.alpha = mgv.deg_to_rad_coeff*numpy.array([    -90.00,   90.00,  -90.00,   90.00,  -90.00,  90.00,   0.00])
        self.a    =                       numpy.array([    0.0000,  0.0000,  0.0000,  0.0000,  0.0000, 0.0000, 0.0000])
        self.d    =                       numpy.array([  0.3150,  0.0000, 0.4500,  0.0000, 0.5000, 0.0000, 0.0800])
        #self.d    =                       numpy.array([  0.3150,  0.0000, 0.4500,  0.0000, 0.5000, 0.0000, 0.0000])

class Manipulator_Geometry_PA10_Reduced( geolib.Manipulator_Geometry ) : 
    '''
    '''
    def __init__( self ) :
        '''
        Define the DH parameters for manipulator PA10 reduced with 5 DOF
        '''
        number_of_joints = 5
        geolib.Manipulator_Geometry.__init__( self, number_of_joints, 'PA10_R' ) 

        self.theta = mgv.deg_to_rad_coeff*numpy.array([     0.00,    0.00,   0.00,    0.00,   0.00])
        self.alpha = mgv.deg_to_rad_coeff*numpy.array([    -90.00,   90.00,  -90.00,   90.00,  -90.00])
        self.a    =                       numpy.array([    0.0000,  0.0000,  0.0000,  0.0000,  0.0000])
        self.d    =                       numpy.array([  0.3150,  0.0000, 0.4500,  0.0000, 0.5000 ])
        self.d6   = 0.08

class Manipulator_Geometry_EXO( geolib.Manipulator_Geometry ) : 
    '''
    '''
    def __init__( self ) :
        '''
        '''
        number_of_joints = 15
        geolib.Manipulator_Geometry.__init__( self, number_of_joints, 'EXO' ) 

        self.theta = mgv.deg_to_rad_coeff*numpy.array([ 0.0 , 0.0 , 0.0 , 77.0, 36.0, 0.0 , 62.0, 43.0, 0.0 , 0.0, 0.0, 0.0 , 0.0, 0.0, 0.0])
        self.alpha = mgv.deg_to_rad_coeff*numpy.array([ 90.0, 90.0, 90.0,-90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 0.0, 0.0, 90.0, 0.0, 0.0 ,90.0])
        self.a    =                       numpy.array([0.000, 0.000, 0.000, 0.000, 0.046, 0.000, 0.000, 0.000, 0.046, 0.000, 0.000, -0.120, 0.000, 0.000 ,0.000])
        self.d    =                       numpy.array([0.048, 0.205, 0.000, 0.000, 0.046, 0.050, 0.078, 0.050, 0.278, 0.129, 0.106,  0.000, 0.095, 0.125 ,0.100])


class Manipulator_Geometry_AILA( geolib.Manipulator_Geometry ) : 
    '''
    '''
    def __init__( self ) :
        '''
        a standard in (not only) python programming: the constructor of a subclass HAS to call the constructor of the superclass 
        '''
        number_of_joints = 7
        geolib.Manipulator_Geometry.__init__( self, number_of_joints, 'AILA' ) 

        #self.theta = [-0.1501133228169865,0.0831943006923638,-0.0688532969938148,-0.0151025367054874,0.2157068997642504,0.0000000000000000,1.5707963705062866]
        self.theta = mgv.deg_to_rad_coeff*numpy.array([  0.00,    0.00,   0.00,    0.00,   0.00,   0.00,   0.00])
        self.alpha = [math.pi/2, -math.pi/2, math.pi/2, -math.pi/2, math.pi/2, -math.pi/2, math.pi /2]
        #self.alpha = [0.0, math.pi/2, -math.pi/2, math.pi/2, -math.pi/2, math.pi/2, -math.pi/2]
        self.a    = numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        #self.d    = numpy.array([0.2140174059160209765, 0.0, 0.2849916407054798242, 0.0, 0.2886165426053090641, 0.0, 0.0])
        self.d    = numpy.array([214.0174059160209765, 0.0, 284.9916407054798242, 0.0, 288.6165426053090641, 0.0, 0.0])

class Manipulator_Geometry_ROT( geolib.Manipulator_Geometry ) : 
    '''
    '''
    def __init__( self ) :
        '''
        Define the DH parameters for manipulator with 5 DOF
        '''
        number_of_joints = 5
        geolib.Manipulator_Geometry.__init__( self, number_of_joints, 'ROT' ) 

        self.theta = mgv.deg_to_rad_coeff*numpy.array([     0.00,    0.00,   0.00,    -90.00,   90.00 ])
        self.alpha = mgv.deg_to_rad_coeff*numpy.array([    -90.00,   90.00,  -90.00,   90.00,  -90.00 ])
        self.a    =                       numpy.array([    0.0000,  0.0000,  0.0000,  0.1000,  0.0000 ])
        self.d    =                       numpy.array([    0.4000,  0.0000,  0.5000,  0.0000,  0.0000 ])

class Manipulator_Configuration_PA10( conflib.Joint_Configuration ) : 
    '''
    '''
    def __init__( self, settings ) :
        '''
        Define joints for PA10
        '''
        number_of_joints = 7
        settings.joint_handling = [ settings.default_joint_handling for i in range(7)]
        settings.joint_handling[4] = 'No Mapping'
        settings.joint_handling[6] = 'No Mapping'
        
        conflib.Joint_Configuration.__init__( self, number_of_joints, settings) 
        self.DOF = 7
        
        self.ql = mgvlib.deg_to_rad_coeff*numpy.array([ -177.00, -91.00, -174.00,  -137.00,  -180.00,   -165.00, -180.00])
        self.qh = mgvlib.deg_to_rad_coeff*numpy.array([  177.00,  91.00,  174.00,   137.00,   180.00,    165.00,  180.00])
        
class Manipulator_Configuration_PA10_Reduced( conflib.Joint_Configuration ) : 
    '''
    '''
    def __init__( self, settings ) :
        '''
        Define joints for PA10 Reduced
        '''
        number_of_joints = 5
        settings.joint_handling = [ settings.default_joint_handling for i in range(number_of_joints)]
        settings.joint_handling[4] = 'No Mapping'
        
        conflib.Joint_Configuration.__init__( self, number_of_joints, settings) 
        self.DOF = 5
        
        self.ql = mgvlib.deg_to_rad_coeff*numpy.array([ -177.00, -91.00, -174.00,  -137.00,  -180.00])
        self.qh = mgvlib.deg_to_rad_coeff*numpy.array([  177.00,  91.00,  174.00,   137.00,   180.00])

class Manipulator_Configuration_PUMA( conflib.Joint_Configuration ) : 
    '''
    '''
    def __init__( self, settings ) :
        '''
        Define joints for PUMA560
        '''
        number_of_joints = 6
        settings.joint_handling = [ settings.default_joint_handling for i in range(6)]
        settings.joint_handling[5] = 'No Mapping'
        
        conflib.Joint_Configuration.__init__( self, number_of_joints, settings)
        self.DOF = 6
        
        self.ql = mgvlib.deg_to_rad_coeff*numpy.array([   -160.00,-180.00,  -90.00,-110.00,-100.00, -180.00])
        self.qh = mgvlib.deg_to_rad_coeff*numpy.array([    160.00,  90.00,  180.00, 170.00, 100.00,  180.00])

class Manipulator_Configuration_EXO( conflib.Joint_Configuration ) : 
    '''
    '''
    def __init__( self, settings ) :
        '''
        Define joints for VI-BOT EXOSKELETON (DFKI)
        '''
        number_of_joints = 15
        settings.joint_handling = [ settings.default_joint_handling for i in range(9)]
        settings.joint_handling[3] = 'No Mapping'
        settings.joint_handling[4] = 'No Mapping'
        settings.joint_handling[5] = 'No Mapping'
        settings.joint_handling[7] = 'No Mapping'
        settings.joint_handling[8] = 'No Mapping'
        
        conflib.Joint_Configuration.__init__( self, number_of_joints, settings) 
        self.DOF = 9
        
        self.ql = mgvlib.deg_to_rad_coeff*numpy.array([-12.0,-14.0, 0.0,-180.0,-180.0,-180.0,-180.0,-180.0,-180.0,-180.0,-180.0,-180.0,45.0 ,-180.0,-180.0])
        self.qh = mgvlib.deg_to_rad_coeff*numpy.array([ 17.0,  8.0, 0.2, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0,120.0, 180.0, 180.0])
                                          #Joint Number : 0      1     2    3      4      5      6      7      8      9      10     11     12    13     14
                                     #Free Joint Number : 0      1     2                  3                    4              5             6     7      8
        self.qh[2] = 0.2
        self.prismatic[2] = True
        self.free[3] = False
        self.free[4] = False
        self.free[6] = False
        self.free[7] = False
        self.free[9] = False
        self.free[11] = False

class Manipulator_Configuration_AILA( conflib.Joint_Configuration ) : 
    '''
    '''
    def __init__( self, settings = conflib.Joint_Configuration_Settings() ) :
        '''
        Define joints for AILA arm (DFKI)
        '''
        number_of_joints = 7
        settings.joint_handling = [ settings.default_joint_handling for i in range(7)]

        conflib.Joint_Configuration.__init__( self, number_of_joints, settings) 
        self.DOF = 7
        
        self.ql = mgvlib.deg_to_rad_coeff*numpy.array([ -90.0, -90.0, -90.0, -90.0, -90.0, -15.0, -45.0])
        self.qh = mgvlib.deg_to_rad_coeff*numpy.array([  90.0,  90.0,  90.0,  90.0,  90.0,  15.0,  45.0])

class Manipulator_Configuration_ROT( conflib.Joint_Configuration ) : 
    '''
    '''
    def __init__( self, settings ) :
        '''
        Define joints for ROT
        '''
        number_of_joints = 5
        settings.joint_handling = [ settings.default_joint_handling for i in range(5)]
        conflib.Joint_Configuration.__init__( self, number_of_joints, settings) 
        self.DOF = 5
        
        self.ql = mgvlib.deg_to_rad_coeff*numpy.array([ -180.0, -90.0, -180.0, -90.0, -90.0])
        self.qh = mgvlib.deg_to_rad_coeff*numpy.array([  180.0,  90.0,  180.0,  90.0,  90.0])


def manipulator_geometry(manip_name): 
    '''    
    
    '''

    # loses its function since this information is saved where it belongs to : inside the class
    # 
    
    ## ! this numbers are not needed outdside the classes ! 

    if manip_name == 'PUMA':
        model_geometry = Manipulator_Geometry_PUMA()

    elif manip_name == 'PA10':
        model_geometry = Manipulator_Geometry_PA10()

    elif manip_name == 'PA10_R':
        model_geometry = Manipulator_Geometry_PA10_Reduced()

    elif manip_name == 'EXO':
        model_geometry = Manipulator_Geometry_EXO()

    elif manip_name == 'AILA':
        model_geometry = Manipulator_Geometry_AILA()
    
    elif manip_name == 'ROT':
        model_geometry = Manipulator_Geometry_ROT()
        
    else : 
        assert False 
        
    #model.forward_kinematics.configuration.set_joint_limits_for(manip)
    #model.forward_kinematics.configuration.initialize()

    return model_geometry

def manipulator_configuration(manip_name, settings): 
    '''    
    
    '''

    if manip_name == 'PUMA':
        model_config = Manipulator_Configuration_PUMA(settings)

    elif manip_name == 'PA10':
        model_config = Manipulator_Configuration_PA10(settings)

    elif manip_name == 'PA10_R':
        model_config = Manipulator_Configuration_PA10_Reduced(settings)

    elif manip_name == 'EXO':
        model_config = Manipulator_Configuration_EXO(settings)

    elif manip_name == 'AILA':
        model_config = Manipulator_Configuration_AILA(settings)

    elif manip_name == 'ROT':
        model_config = Manipulator_Configuration_ROT(settings)
    
    else : 
        assert False 
        
    #model.forward_kinematics.configuration.set_joint_limits_for(manip)
    #model.forward_kinematics.configuration.initialize()
    model_config.q = 0.5*(model_config.qh + model_config.ql)
    model_config.initialize()
    model_config.update_joint_limit_jacobian_multipliers()

    return model_config

    
    
