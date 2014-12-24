#~ 
# HEADER
'''   
@file:          configuration.py
@brief:    	    This module provides a class representing the configuration of a manipulator in the jointspace including some methods.
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
@version:	    1.5
Last Revision:  23 October 2012
'''
# BODY
import numpy, math, random

from packages.nima.mathematics import vectors_and_matrices as vecmat
from packages.nima.mathematics import global_variables as mgvlib
from packages.nima.mathematics import trigonometry
from packages.nima.mathematics import discrete

def grid_point(config_number, number_of_intervals, qh, ql):
    '''
    returns joint configuration with the configuration in a gridded jointspace with "number_of_intervals" divisions for each joint 
    The order of the configuration in the gridded jointspace is identified by "config_number",
    '''
    dof = len(qh)
    assert dof == len(ql)
    q = numpy.zeros(dof)
    A = discrete.number_in_base(N = config_number, base = number_of_intervals, n_digits = dof)
    j = 0
    for j in range(0, dof):
        q[j] = ql[j] + (2*A[j] + 1)*(qh[j] - ql[j])/(2*number_of_intervals)
        j = j + 1

    return q

class Joint_Configuration_Settings:
    
    def __init__(self, njoint = 0, DOF= 0, joint_handling = [], default_joint_handling = 'Trigonometric Mapping'):
        # joint_handling - method for handling joints, type of mapping for limited joints
        self.joint_handling = joint_handling
        self.default_joint_handling = default_joint_handling
        self.njoint = njoint
        # DOF - is a short form of Degrees of Freedom and represents the number of free joints
        self.DOF = DOF

        if self.joint_handling == []:
           self.joint_handling = [default_joint_handling for i in range(0, DOF)]
        else:
            assert len(settings.joint_handling) == DOF

## introduce the init mechanism like we did for the example in the test folder 

class Joint_Configuration:
    '''
    Contains joint parameters of a manipulator, joint types, joint limits and etc ...
    plus methods for joint mapping, in_range checking and everything regarding the joints.
    The settings for configuration consists of only "joint_handling"
    '''
    def __init__(self, settings):


        self.settings = settings
#        self.settings.njoint = njoint

        # q    - A list of real numbers representing the joint configuration of the manipulator in the limited space
        self.q   = numpy.zeros((settings.njoint))

        # qstar - A list of real numbers representing the joint configuration of the manipulator in the unlimited space
        self.qstar   = numpy.zeros((settings.njoint))

        # ql  - A vector of real numbers representing lower bounds for the joint configuration
        # qh  - A vector of real numbers representing higher bounds for the joint configuration
        self.ql   = [-math.pi for i in range(0, settings.njoint)]
        self.qh   = [math.pi for i in range(0, settings.njoint)]

        # prismatic  - A list of booleans representing the type of the corresponding joint number. If True the joint is prismatic, if False it is revolute
        self.prismatic = [False for i in range(0, settings.njoint)]

        # free  - A list of booleans representing whether the corresponding joint is free or fixed at a certain value. The default is True for all joints
        self.free = [True for i in range(0, settings.njoint)]

    def __str__(self):
        s = "q = "
        s += str(vecmat.format_vector((180.0/math.pi)*self.q, format="%.2f"))
        s += "    (Joints are"
        if not self.joints_in_range():
            s += " NOT"
        s += " all in range)"
        
        return s

    def free_configuration(self):
        '''
        return a vector (DOF elements) containing the values of free joints. 
        '''
        fc = numpy.zeros((self.DOF))
        j = 0
        for jj in range(0,self.settings.njoint):
            if self.free[jj]:                    
                fc[j] = self.q[jj]
                j = j + 1
        return fc

    def free_configuration_inv(self, qs):
        '''
        puts the values of free joints in their proper place in the main config vector. Return the main config vector
        '''
        c = numpy.copy(self.q)
        j = 0
        for jj in range(0,self.settings.njoint):
            if self.free[jj]:                    
                c[jj] = qs[j]
                j = j + 1
        return c

    def bring_revolute_joints_to_standard_range(self):
        '''
        Change revolute joint values (joint angles) if they are out of standard range (-pi  +pi) to their equivalent value in the standard range
        '''
        for i in range(0, self.settings.njoint):
            if not self.prismatic[i]:
                self.q[i] = trigonometry.angle_standard_range( self.q[i] ) 

    def joints_in_range(self):
        '''
        First, bring revolute joint values (joint angles) to standard range (-pi   +pi)        
        Then, if The joints are out of the given range specified by properties: ql and qh, return False, otherwise return True  
        '''
        flag = True
        self.bring_revolute_joints_to_standard_range()
        for i in range(0, self.settings.njoint):
            if abs(self.q[i] - self.ql[i]) < mgvlib.epsilon:
                self.q[i] = self.ql[i]
            if abs(self.q[i] - self.qh[i]) < mgvlib.epsilon:
                self.q[i] = self.qh[i]
            flag = flag and (self.q[i] <= self.qh[i]) and (self.q[i] >= self.ql[i])
        return flag

    def qstar_to_q(self):
        '''
        map from unlimited to limited jointspace. Get mapped values in the unlimited jointspace (property: qstar) and return the main joint configuration vector
        '''
        qq = numpy.zeros((self.DOF))
        for i in range(self.DOF):
            if self.settings.joint_handling[i] == 'No Mapping':
                #qq[i] = self.qstar[i]
                qq[i] = trigonometry.angle_standard_range( self.qstar[i] ) 
            elif self.settings.joint_handling[i] == 'Linear Mapping':
                self.qstar[i]  = trigonometry.angle_standard_range( self.qstar[i] ) 
                qq[i]          = (self.qstar[i] - self.jmc_b[i]) / self.jmc_a[i]
            elif self.settings.joint_handling[i] == 'Trigonometric Mapping':
                qq[i] = self.jmc_f[i] * math.cos(self.qstar[i]) + self.jmc_g[i] 
            elif self.settings.joint_handling[i] == 'Tangent Mapping':
                qq[i] = 2*math.atan(self.jmc_f[i] * math.cos(self.qstar[i]) + self.jmc_g[i]) 
                
            else:
                print 'Wrong Joint Handling (Free Joint ' + str(i) + '): ' + self.settings.joint_handling[i]
                assert False
                
        return self.free_configuration_inv(qq)

    def q_to_qstar(self):
        '''
        map from limited to unlimited jointspace. Get main joint values in the limited jointspace (property: q) and updates the joint values in the unlimited space (property: qstar)
        '''
        qf = self.free_configuration()
        self.qstar = numpy.zeros((self.DOF))
         
        for i in range(self.DOF):
            if self.settings.joint_handling[i] == 'No Mapping':
                self.qstar[i] = qf[i]
            elif self.settings.joint_handling[i] == 'Linear Mapping':
                self.qstar[i] = self.jmc_a[i] * qf[i] + self.jmc_b[i] 
            elif self.settings.joint_handling[i] == 'Trigonometric Mapping':
                self.qstar[i] = trigonometry.arccos((qf[i] - self.jmc_g[i]) / self.jmc_f[i])
            elif self.settings.joint_handling[i] == 'Tangent Mapping':
                self.qstar[i] = trigonometry.arccos((math.tan(0.5*qf[i]) - self.jmc_g[i]) / self.jmc_f[i])
            else:
                print 'Wrong Joint Handling (Free Joint ' + str(i) + '): ' + self.settings.joint_handling[i]
                assert False
                
    def update_joint_limit_jacobian_multipliers(self):
        '''
        joint_limit_multipliers are coefficients of forward and inverse mapping to/from limited to/from unlimited jointspace.
        i.e: jmc_a is the derivative of q to q_star
        (Refer also to the comments of method: initialize in this class)
        This method calculates and updates the values of one of the joint limit multipliers: "jmc_c" which is called "joint_limit_jacobian_multipliers"
        Each column of the error jacobian is multiplied by the corresponding element of vector "jmc_c": Jstar = Je * diag(jmc_c) 
        Since the coefficient "jmc_c" for a joint, depends on the current value of that joint, it should be updated everytime "q" changes
        The other multipliers: "jmc_a", "jmc_b", "jmc_f" and "jmc_g" do not depend on the joints, therefore do not need to be updated
        '''
        for i in range(0,self.DOF):
            if self.settings.joint_handling[i] == 'No Mapping':
                self.jmc_c[i] = 1.0

            elif self.settings.joint_handling[i] == 'Linear Mapping':
                self.jmc_c[i] = 1.0 / self.jmc_a[i]

            elif self.settings.joint_handling[i] == 'Trigonometric Mapping':
                self.jmc_c[i] = - math.sin(self.qstar[i])/self.jmc_a[i]

            elif self.settings.joint_handling[i] == 'Tangent Mapping':
                t = self.jmc_f[i] * math.cos(self.qstar[i]) + self.jmc_g[i]
                self.jmc_c[i] = - 2.0 * math.sin(self.qstar[i])/(self.jmc_a[i]*(1 + t**2))
            else:
                assert False
                

    def initialize(self):
        '''
        Everything regarding joint parameters needed to be done before running kinematic calculations
        This function must be called whenever you change joint limits or change type of a joint.
        '''
        # Counting free joints in order to detremine the degree of freedom: "DOF"
        self.DOF = 0
        # Important: Here you do not consider prismatic joints. Do it in the future
        for i in range(0,self.settings.njoint):
            # First bring revolute joint limits in the range (- pi , pi)
            if (not self.prismatic[i]):
                self.ql[i]       = trigonometry.angle_standard_range( self.ql[i] ) 
                self.qh[i]       = trigonometry.angle_standard_range( self.qh[i] ) 
                
            if self.free[i]:
                self.DOF += 1
                
        # check all the joints are already in their desired range
        '''
        for i in range(self.DOF):
            if not self.settings.joint_handling[i] == 'No Mapping':
                assert self.joints_in_range() #check only for joint i
        '''

        '''
        The following code defines the joint limit multipliers
        jmc_a and jmc_b are auxiliary coefficients which are used for conversion from "q" to "qstar" space
        (Please refer to the joint limits section of the documentation associated with this code)

        To make sure that all joints are in their desired range identified as: (self.ql, self.qh), the real jointspace is mapped into an unlimited jointspace.
        
        "q    " represent the real joint values which are in a limited jointpace
        "qstar" represent the mapped joint values in unlimited jointspace
        
        The following code creates and defines "Joint Limit Multipliers". These are coefficients for this jointspace mapping - (jmc: Jointspace Mapping Coefficients)
        in linear mapping:
        
            q  = a * qstar + b
            qs = (q - b) / a
        
        in trigonometric mapping:

            q      = f * cos(qstar) + g
            qstar  = arccos [  (q - g) / f ]
            
        a, b, f and g are calculated in such a way that the real joint values which are in their feasible range are mapped into an equivalent qstar in an unlimited space (-pi, +pi)
        
        "jmc_c" is multiplied by the columns of the error jacobian matrix
        '''
        self.jmc_a = numpy.zeros((self.DOF))
        self.jmc_b = numpy.zeros((self.DOF))
        self.jmc_c = numpy.zeros((self.DOF))
        self.jmc_f = numpy.zeros((self.DOF))
        self.jmc_g = numpy.zeros((self.DOF))
        
        ii = 0
        for i in range(0,self.settings.njoint):
            if self.free[i]:
                self.jmc_f[ii] = 0.5*(self.qh[i] - self.ql[i])
                self.jmc_g[ii] = 0.5*(self.qh[i] + self.ql[i])

                if self.settings.joint_handling[ii] == 'No Mapping':
                    self.jmc_a[ii] = 1.00000
                    self.jmc_b[ii] = 0.00000
                elif self.settings.joint_handling[ii] == 'Linear Mapping':
                    self.jmc_a[ii] = 2*math.pi/(self.qh[i] - self.ql[i])
                    self.jmc_b[ii] = math.pi * (self.qh[i] + self.ql[i])/(self.ql[i] - self.qh[i])
                elif self.settings.joint_handling[ii] == 'Trigonometric Mapping':
                    self.jmc_a[ii] = 2/(self.qh[i] - self.ql[i])
                    self.jmc_b[ii] = (self.qh[i] + self.ql[i])/(self.ql[i] - self.qh[i])
                elif self.settings.joint_handling[ii] == 'Tangent Mapping':
                    th = math.tan(0.5*self.qh[i])
                    tl = math.tan(0.5*self.ql[i]) 
                    self.jmc_a[ii] = 2/(th - tl)
                    self.jmc_b[ii] = (tl + th)/(tl - th)
                    self.jmc_f[ii] = 0.5*(th - tl)
                    self.jmc_g[ii] = 0.5*(th + tl)
                else:
                    assert False
                    
                ii += 1
        # map q to qstar for the first time:
        self.q_to_qstar()

        # assign the value of jmc_c the jacobian multiplier:
        self.update_joint_limit_jacobian_multipliers()

    def joint_correction_in_range(self, dq):
        '''
        Return True if the joint configuration will be in the desired range after being added by the given "dq"
        '''
        i = 0
        in_range = True
        for ii in range(0,self.settings.njoint):
            if self.free[ii]:
                in_range = in_range and (self.q[ii] + dq[i] <= self.qh[ii]) and (self.q[ii] + dq[i] >= self.ql[ii]) 
                i = i + 1
        return in_range

    def grow_qstar(self,delta_qs):
        '''
        Changes and updates the values of the joint configuration in the unlimited space ("qstar") after correction by "delta_qs".
        The real joint values ("q") is then calculated and updated as well.
        "delta_qs" should contain only the correction values for the free joints in the unlimited space.
        This method changes the joint configuration. Therefore a forward kinematics update should be implemented after it.
        '''
        self.qstar = self.qstar + delta_qs
        self.q = self.qstar_to_q()
        
    def grow_q(self,delta_q):
        '''
        Return the value of the joint configuration after correction by "delta_q". "delta_q" should contain only the values of the free joints.
        '''
        qq = numpy.copy(self.q)
        i = 0
        for ii in range(0,self.configuration.number_of_joints):
            if self.free[ii]:
                qq[ii] = qq[ii] + delta_q[i]
                i = i + 1
        return qq

    """
    def set_joint_limits_for(self, manip):
        '''
        Determines joint limits for a number of known manipulators: PUMA, PA10, EXO and AILA
        '''
        if manip == 'PA10':
            self.ql = mgvlib.deg_to_rad_coeff*numpy.array([ -177.00, -91.00, -174.00,  -137.00,  -180.00,   -165.00, -180.00])
            self.qh = mgvlib.deg_to_rad_coeff*numpy.array([  177.00,  91.00,  174.00,   137.00,   180.00,    165.00,  180.00])
        elif manip == 'PUMA':
            self.ql = mgvlib.deg_to_rad_coeff*numpy.array([   -160.00,-180.00,  -90.00,-110.00,-100.00, -180.00])
            self.qh = mgvlib.deg_to_rad_coeff*numpy.array([    160.00,  90.00,  180.00, 170.00, 100.00,  180.00])
        elif manip == 'EXO':
            self.ql = mgvlib.deg_to_rad_coeff*numpy.array([-12.0,-14.0, 0.0,-180.0,-180.0,-180.0,-180.0,-180.0,-180.0,-180.0,-180.0,-180.0,45.0 ,-180.0,-180.0])
            self.qh = mgvlib.deg_to_rad_coeff*numpy.array([ 17.0,  8.0, 0.2, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0,120.0, 180.0, 180.0])
            self.qh[2] = 0.2
            self.prismatic[2] = True
            self.free[3] = False
            self.free[4] = False
            self.free[6] = False
            self.free[7] = False
            self.free[9] = False
            self.free[11] = False
        elif manip == 'AILA':
            self.ql = mgvlib.deg_to_rad_coeff*numpy.array([ -90.0, -90.0, -90.0, -90.0, -90.0, -15.0, -45.0])
            self.qh = mgvlib.deg_to_rad_coeff*numpy.array([  90.0,  90.0,  90.0,  90.0,  90.0,  15.0,  45.0])
        
        # set all the joints in the middle of their ranges
        self.q = 0.5*(self.qh + self.ql)
    """

    def change_to_grid_configuration(self, config_number, number_of_intervals):
        '''
        Replaces joint configuration with the configuration in a gridded jointspace with "number_of_intervals" divisions for each joint 
        The order of the configuration in the gridded jointspace is identified by "config_number",
        '''
        A = discrete.number_in_base(N = config_number, base = number_of_intervals, n_digits = self.settings.DOF)
        j = 0
        for jj in range(0,self.settings.njoint):

            if self.free[jj]:
                self.q[jj] = self.ql[jj] + (2*A[j] + 1)*(self.qh[jj] - self.ql[jj])/(2*number_of_intervals)
                j = j + 1

        self.initialize()
        #self.update_joint_limit_jacobian_multipliers()

    def generate_random(self):
        j = 0
        for jj in range(0,self.settings.njoint):
            if self.free[jj]:
                self.q[jj] = (self.qh[jj] - self.ql[jj]) * random.random() + self.ql[jj]
                j = j + 1
        self.initialize()
        self.update_joint_limit_jacobian_multipliers()
        
