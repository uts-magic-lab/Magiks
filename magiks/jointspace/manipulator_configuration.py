#~ 
# HEADER
'''   
@file:          manipulator_configuration.py
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
@version:	    2.0
Last Revision:  24 December 2014
'''
'''
Major change from previous version:
   All properties that can be changed by the user are in settings now. 
   So, one should not change any property of class Configuration. property q must be changed using method set_config()
   Name: :Joint_Configuration_Settings" changed to "Configuration Settings"
   Name: :Joint_Configuration" changed to "Configuration"
'''

# BODY
import numpy as np, math, random, general_python as genpy

from math_tools import general_math as gen
from math_tools.algebra import vectors_and_matrices as vecmat
from math_tools.geometry import trigonometry as trig
from math_tools.discrete import discrete


class Manipulator_Configuration_Settings:
    '''
    Contains joint parameters of a manipulator, joint types, joint_handling, joint limits and etc ...
    '''

    def __init__(self, njoint = 0, DOF= 0, joint_mapping = 'NM'):
        self.njoint = njoint
        # DOF - is a short form of Degrees of Freedom and represents the number of free joints
        self.DOF = DOF

        self.joint_handling = [joint_mapping for i in range(DOF)]
        self.limited        = [True for i in range(0, DOF)]

        ## Specify if the joint limits must be respected when new joint values are being set
        #  If this property is True, the system checks for the given joint values to be in the defined feasible range
        #  and fails with an error if given joint values are beyond the limits.
        #  If False, any value for the joints are accepted.
        self.joint_limits_respected = True

        # ql  - A vector of real numbers representing lower bounds for the joint configuration
        # qh  - A vector of real numbers representing higher bounds for the joint configuration
        self.ql   = [-math.pi for i in range(0, njoint)]
        self.qh   = [math.pi for i in range(0, njoint)]

        # prismatic  - A list of booleans representing the type of the corresponding joint number. If True the joint is prismatic, if False it is revolute
        self.prismatic = [False for i in range(0, njoint)]

        # free  - A list of booleans representing whether the corresponding joint is free or fixed at a certain value. The default is True for all joints
        self.free = [True for i in range(0, njoint)]
        
class Manipulator_Configuration(object):
    '''
    Contains methods for joint mapping, in_range checking and everything regarding the joints.
    '''

    def __init__(self, settings):
        err_head       = "Error from configuration.Joint_Configuration." 

        self.config_settings = settings

        # q    - A list of real numbers representing the joint configuration of the manipulator in the jointspace
        self.q     = 0.5*(settings.qh + settings.ql)        

        # qstar - A list of real numbers representing the joint configuration of the manipulator in the mapped jointspace
        self.qstar   = None

        self.initialize()
        '''
        If you have a method in class A and you intended to have a method with the same name in a class B inherited from A,
        you should not call this method in class A, so I created a function set_configuration() which is called by set_config()
        '''

        self.q_to_qstar()
        self.update_joint_limit_jacobian_multipliers()

    def config_str(self):
        s = "q = "
        s += vecmat.vector_to_str((180.0/math.pi)*self.q, format="%.2f")
        s += "    (Joints are"
        if not self.joints_in_range(self.free_config(self.q)):
            s += " NOT"
        s += " all in range)"
        
        return s

    ## protected
    def bring_revolute_joints_to_standard_range(self):
        '''
        Change revolute joint values (joint angles) if they are out of standard range (-pi  +pi) to their equivalent value in the standard range
        '''
        for i in range(0, self.config_settings.njoint):
            if not self.config_settings.prismatic[i]:
                self.q[i] = trig.angle_standard_range( self.q[i] ) 

	## This function is used to check if a given value for specific joint is in its feasibility range
	#  @param i An integer between 0 to 6 specifying the joint number to be checked
	#  @param qi A float parameter specifying the value for the i-th joint
	#  @return A boolean: True if the given joint angle qi is in feasible range for the i-th joint (within the specified joint limits for that joint)
    def joint_in_range(self,i, qi):
        if abs(qi - self.ql[i]) < gen.epsilon:
            qi = self.ql[i]
        if abs(qi - self.qh[i]) < gen.epsilon:
            qi = self.qh[i]

        return ((qi <= self.qh[i]) and (qi >= self.ql[i]))
    
	## This function is used to check if all values of the given joint array are in their feasibility range
	#  @param qd A numpy array of size DOF containing the values to be checked
	#  @return A boolean: If The given joints "qd" are out of the range specified by properties: ql and qh, returns False, otherwise returns True
    def joints_in_range(self, qd): 
        flag = True
        for i in range(0, self.config_settings.DOF):
            if self.config_settings.limited[i]:
                flag = flag and self.joint_in_range(i, qd[i])
        return flag

    def qstar_to_q(self, qs):
        '''
        map from unlimited to limited jointspace. Get mapped values in the unlimited jointspace (property: qstar) and return the main joint configuration vector
        '''
        qq = np.zeros((self.config_settings.DOF))
        for i in range(self.config_settings.DOF):
            if self.config_settings.limited[i]:
                if self.config_settings.joint_handling[i] == 'NM':
                    qq[i] = trig.angle_standard_range( qs[i] ) 
                elif self.config_settings.joint_handling[i] == 'LM':
                    qs[i]  = trig.angle_standard_range( qs[i] ) 
                    qq[i]  = (qs[i] - self.jmc_b[i]) / self.jmc_a[i]
                elif self.config_settings.joint_handling[i] == 'TM':
                    qq[i] = self.jmc_f[i] * math.cos(qs[i]) + self.jmc_g[i] 
                elif self.config_settings.joint_handling[i] == 'TGM':
                    qq[i] = 2*math.atan(self.jmc_f[i] * math.cos(qs[i]) + self.jmc_g[i]) 
                else:
                    assert False, genpy.err_str(__name__, self.__class__.__name__, 'qstar_to_q', self.config_settings.joint_handling[i] + " is not a valid value for joint_handling")
            else:
                qq[i] = qs[i]
                
        # return self.free_config_inv(qq)
        return qq

    def q_to_qstar(self):
        '''
        or sync_qstar  or sync_vjs vjs: Virtual Joint Space
        map from limited to unlimited jointspace. Get main joint values in the limited jointspace (property: q) and updates the joint values in the unlimited space (property: qstar)
        '''
        qf = self.free_config(self.q)

        self.qstar = np.zeros((self.config_settings.DOF))
         
        for i in range(self.config_settings.DOF):
            if (not self.config_settings.limited) or (self.config_settings.joint_handling[i] == 'NM'):
                self.qstar[i] = qf[i]
            elif self.config_settings.joint_handling[i] == 'LM':
                assert self.joint_in_range(i, qf[i]), "Joint " + str(i) + " = " + str(qf[i]) + " is not in feasible range: (" + str(self.config_settings.ql[i]) + ',' + str(self.config_settings.qh[i]) + ')'
                self.qstar[i] = self.jmc_a[i] * qf[i] + self.jmc_b[i] 
            elif self.config_settings.joint_handling[i] == 'TM':
                '''
                ql = self.config_settings.ql[i]
                qh = self.config_settings.qh[i]
                print "i: ", i, "ql: ", ql," q: ", qf[i],"qh: ", qh, " g: ", self.jmc_g[i], " f: ", self.jmc_f[i], " c: ", (qf[i] - self.jmc_g[i]) / self.jmc_f[i]
                '''
                assert self.joint_in_range(i, qf[i]), "Joint " + str(i) + " = " + str(qf[i]) + " is not in feasible range: (" + str(self.config_settings.ql[i]) + ',' + str(self.config_settings.qh[i]) + ')'
                self.qstar[i] = trig.arccos((qf[i] - self.jmc_g[i]) / self.jmc_f[i])
            elif self.config_settings.joint_handling[i] == 'TGM':
                assert self.joint_in_range(i, qf[i]), "Joint " + str(i) + " = " + str(qf[i]) + " is not in feasible range: (" + str(self.config_settings.ql[i]) + ',' + str(self.config_settings.qh[i]) + ')'
                self.qstar[i] = trig.arccos((math.tan(0.5*qf[i]) - self.jmc_g[i]) / self.jmc_f[i])
            else:
                print 'Wrong Joint Handling (Free Joint ' + str(i) + '): ' + self.config_settings.joint_handling[i]
                assert False, genpy.err_str(__name__, self.__class__.__name__, 'q_to_q', self.config_settings.joint_handling[i] + " is not a valid value for joint_handling")

    ## protected            
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
        func_name = "update_joint_limit_jacobian_multipliers"
        for i in range(0,self.config_settings.DOF):
            if self.config_settings.joint_handling[i] == 'NM':
                self.jmc_c[i] = 1.0

            elif self.config_settings.joint_handling[i] == 'LM':
                self.jmc_c[i] = 1.0 / self.jmc_a[i]

            elif self.config_settings.joint_handling[i] == 'TM':
                self.jmc_c[i] = - math.sin(self.qstar[i])/self.jmc_a[i]

            elif self.config_settings.joint_handling[i] == 'TGM':
                t = self.jmc_f[i] * math.cos(self.qstar[i]) + self.jmc_g[i]
                self.jmc_c[i] = - 2.0 * math.sin(self.qstar[i])/(self.jmc_a[i]*(1 + t**2))
            else:
                assert False, genpy.err_str(__name__, self.__class__.__name__, 'update_joint_limit_jacobian_multipliers', self.config_settings.joint_handling[i] + " is not a valid value for joint_handling")

    def initialize(self):
        '''
        Everything regarding joint parameters needed to be done before running kinematic calculations
        This function must be called whenever you change joint limits or change type of a joint.
        '''
        func_name = "initialize()"
        # Counting free joints in order to detremine the degree of freedom: "DOF"
        self.config_settings.DOF = 0
        # Important: Here you do not consider prismatic joints. Do it in the future
        for i in range(0,self.config_settings.njoint):
            # First bring revolute joint limits in the range (- pi , pi)
            if (not self.config_settings.prismatic[i]):
                self.config_settings.ql[i]       = trig.angle_standard_range( self.config_settings.ql[i] ) 
                self.config_settings.qh[i]       = trig.angle_standard_range( self.config_settings.qh[i] ) 
                
            if self.config_settings.free[i]:
                self.config_settings.DOF += 1

        self.ql = self.free_config(self.config_settings.ql)
        self.qh = self.free_config(self.config_settings.qh)
                
        # check all the joints are already in their desired range
        '''
        for i in range(self.DOF):
            if not self.config_settings.joint_handling[i] == 'No Mapping':
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
        self.jmc_a = np.zeros((self.config_settings.DOF))
        self.jmc_b = np.zeros((self.config_settings.DOF))
        self.jmc_c = np.zeros((self.config_settings.DOF))
        self.jmc_f = np.zeros((self.config_settings.DOF))
        self.jmc_g = np.zeros((self.config_settings.DOF))
        
        self.jmc_f = 0.5*(self.qh - self.ql)
        self.jmc_g = 0.5*(self.qh + self.ql)

        for i in range(0, self.config_settings.DOF):
            if self.config_settings.joint_handling[i] == 'NM':
                self.jmc_a[i] = 1.00000
                self.jmc_b[i] = 0.00000
            elif self.config_settings.joint_handling[i] == 'LM':
                self.jmc_a[i] = 2*math.pi/(self.qh[i] - self.ql[i])
                self.jmc_b[i] = math.pi * (self.qh[i] + self.ql[i])/(self.ql[i] - self.qh[i])
            elif self.config_settings.joint_handling[i] == 'TM':
                self.jmc_a[i] = 2.0/(self.qh[i] - self.ql[i])
                self.jmc_b[i] = (self.qh[i] + self.ql[i])/(self.ql[i] - self.qh[i])
            elif self.config_settings.joint_handling[i] == 'TGM':
                th = math.tan(0.5*self.qh[i])
                tl = math.tan(0.5*self.ql[i]) 
                self.jmc_a[i] = 2/(th - tl)
                self.jmc_b[i] = (tl + th)/(tl - th)
                self.jmc_f[i] = 0.5*(th - tl)
                self.jmc_g[i] = 0.5*(th + tl)
            else:
                assert False, genpy.err_str(__name__, self.__class__.__name__, 'initialize', self.config_settings.joint_handling[i] + " is not a valid value for joint_handling")
                    
        # map q to qstar for the first time:
        self.q_to_qstar()

        # assign the value of jmc_c the jacobian multiplier:
        self.update_joint_limit_jacobian_multipliers()

    def joint_correction_in_range(self, dq):
        '''
        Return True if the joint configuration will be in the desired range after being added by the given "dq"
        '''
        in_range = True
        for i in range(0,self.DOF):
            in_range = in_range and (self.q[i] + dq[i] <= self.qh[i]) and (self.q[i] + dq[i] >= self.ql[i]) 
            i = i + 1
        return in_range

    def set_qstar(self, qs):
        '''
        Changes and updates the values of the joint configuration in the unlimited space ("qstar") after correction by "delta_qs".
        The real joint values ("q") is then calculated and updated as well.
        "delta_qs" should contain only the correction values for the free joints in the unlimited space.
        This method changes the joint configuration. Therefore a forward kinematics update should be implemented after it.
        '''
        self.qstar = qs
        qd         =  self.qstar_to_q(qs)
        permission = not self.config_settings.joint_limits_respected
        if not permission:
            permission = self.joints_in_range(qd)
        if permission:
            self.q = self.free_config_inv(qd)
            self.update_joint_limit_jacobian_multipliers()
            return True
        else:
            assert False, genpy.err_str(__name__, self.__class__.__name__, 'set_qstar', 'The joint values computed from qstar are out of range while joint limits are respected.')
            return False
        
	## This function gives an interval for the magnitude of the arm joint angles correction vector. (Copied from pr2_arm_kinematics.py)
	#  @param direction A numpy array of size 7 specifying the direction of change	
	#  @param max_speed A float parameter specifying the maximum feasible speed for a joint change. (Set by infinity by default)	
	#  @param delta_t The step time in which the change(correction) is to be applied.
	#  @return An instance of type <a href="http://pyinterval.googlecode.com/svn/trunk/html/index.html">Interval()</a>
    def joint_stepsize_interval(self, direction, max_speed = gen.infinity, delta_t = 0.001):
        
        '''
        The correction of joints is always restricted by joint limits and maximum feasible joint speed
        If you want to move the joints in a desired direction, how much are you allowed to move?	
        This function returns a feasible interval for the stepsize in the given direction. 
        The joint direction of change must be multiplied by a scalar value in this range so that the applied changes
        are feasible.
        '''
        etta_l = []
        etta_h = []

        # for ii in range(self.config_settings.njoint):
        for i in range(self.config_settings.DOF):
            if not gen.equal(direction[i], 0.0):
                if self.config_settings.limited[i]:
                    a = (self.ql[i] - self.q[i])/direction[i]    
                    b = (self.qh[i] - self.q[i])/direction[i]
                    etta_l.append(gen.sign_choice(a, b, direction[i]))
                    etta_h.append(gen.sign_choice(b, a, direction[i]))

                a = delta_t*max_speed/direction[i]
                etta_l.append(gen.sign_choice(-a, a, direction[i]))
                etta_h.append(gen.sign_choice( a,-a, direction[i]))

        if (etta_l == []) or (etta_h == []):
            return (0.0, 0.0)
        else:
            return (max(etta_l), min(etta_h))

    def free_config(self, q):
        '''
        return a vector (DOF elements) containing the values of free joints. 
        '''
        fc = np.zeros((self.config_settings.DOF))
        j = 0
        for jj in range(0,self.config_settings.njoint):
            if self.config_settings.free[jj]:                    
                fc[j] = q[jj]
                j = j + 1
        return fc

    def free_config_inv(self, qs):
        '''
        puts the values of free joints in their proper place in the main config vector. Return the main config vector
        '''
        c = np.copy(self.q)
        j = 0
        for jj in range(0,self.config_settings.njoint):
            if self.config_settings.free[jj]:                    
                c[jj] = qs[j]
                j = j + 1
        return c

	## Use this function to set the joint configuration to the given desired values.
    #  If property joint_limits_respected is True, the system checks for given qd to be in the defined feasible range
    #  and fails with an error if given joint values are beyond the limits
    #  If property joint_limits_respected is False, any value for the joints are accepted.
	#  @param qd A numpy array of size 7 containing the desired joint values to be set
	#  @return A boolean: True if the given joints are in range and the configuration is set, False if not	
    def set_config(self, qd, set_qstar = True):
        '''
        This function sets the robot joint configuration to the given "qd"
        This function should not be called by the end user. 
        Always use function "set_config" to change the values of the joints 
        '''    
        len_qd = len(qd)
        assert len_qd == self.config_settings.DOF, genpy.err_str(__name__, self.__class__.__name__,'set_config','Length of given config vector is '+str(len_qd)+' which does not match the settings DOF('+str(self.config_settings.DOF)+')')
        permission = not self.config_settings.joint_limits_respected
        if not permission:
            permission = self.joints_in_range(qd)
    
        if permission:
            j = 0
            for jj in range(0,self.config_settings.njoint):
                if self.config_settings.free[jj]:
                    if self.config_settings.limited[j] and (not self.config_settings.prismatic[jj]):
                        self.q[jj] = trig.angle_standard_range(qd[j])
                    else:
                        self.q[jj] = qd[j]
                    j = j + 1

            self.q_to_qstar()
            self.update_joint_limit_jacobian_multipliers()

            return True
        else:
            print "Error from configuration.set_config(): Given joints are not in their feasible range"
            for i in range(self.config_settings.DOF):
                if not self.joint_in_range(i, qd[i]):        
                    print    
                    print "Joint Number: ", i
                    print "Lower Limit  :", self.ql[i]
                    print "Desired Value:", qd[i]
                    print "Upper Limit  :", self.qh[i]
            return False
    
    def random_config(self):
        q_rnd = np.zeros(self.config_settings.DOF)
        
        for j in range(0, self.config_settings.DOF):
            q_rnd[j] = self.ql[j] + random.random()*(self.qh[j] - self.ql[j])
        return q_rnd

    def grid_config(self, config_number, number_of_intervals):
        '''
        Replaces joint configuration with the configuration in a gridded jointspace with "number_of_intervals" divisions for each joint 
        The order of the configuration in the gridded jointspace is identified by "config_number",
        '''
        q_grd = np.zeros(self.config_settings.DOF)
        A     = discrete.number_in_base(N = config_number, base = number_of_intervals, n_digits = self.config_settings.DOF)

        for j in range(0, self.config_settings.DOF):
            q_grd[j] = self.ql[j] + (2*A[j] + 1)*(self.qh[j] - self.ql[j])/(2*number_of_intervals)

        return q_grd

    def objective_function_gradient(self, k = 1.0):
        dof = self.config_settings.DOF
        qh  = self.qh
        ql  = self.ql
        u   = np.zeros(dof)
        qs  = trig.angles_standard_range(self.qstar)
        for i in range(0, self.config_settings.DOF):
            if self.config_settings.joint_handling[i] == 'TM':
                u[i] = - k*qs[i]/(dof*((2*math.pi)**2))
            elif self.config_settings.joint_handling[i] == 'NM':
                if self.config_settings.limited[i]:
                    mean = 0.5*(self.qh[i] + self.ql[i])
                    u[i] = - k*(qs[i] - mean)/(dof*((self.qh[i] - self.ql[i])**2))
                    # print "qs[i] = ", qs[i], " mean = ", mean, " qh-ql= ", self.qh[i] - self.ql[i], " u = ", u[i]
            else:
                assert False, genpy.err_str(__name__, self.__class__.__name__, 'objective_function_gradient', self.config_settings.joint_handling[i] + " is not a valid value for joint_handling")
        return u

    def objective_function_value(self, k = 1.0):
        dof = self.config_settings.DOF
        qs  = trig.angles_standard_range(self.qstar)
        qh  = self.qh
        ql  = self.ql
        f   = 0.0
        for i in range(0, self.config_settings.DOF):
            df = 0.0
            if self.config_settings.joint_handling[i] == 'TM':
                f += - k*qs[i]/(dof*((2*math.pi)**2))
            elif self.config_settings.joint_handling[i] == 'NM':
                if self.config_settings.limited[i]:
                    mean = 0.5*(self.qh[i] + self.ql[i])
                    df   = k*((qs[i] - mean)**2)/(dof*((self.qh[i] - self.ql[i])**2))
                    # print "qs[i] = ", qs[i], " mean = ", mean, " qh-ql= ", self.qh[i] - self.ql[i], " df = ", df

            else:
                assert False, genpy.err_str(__name__, self.__class__.__name__, 'objective_function_value', self.config_settings.joint_handling[i] + " is not a valid value for joint_handling")
            f += df
        return f
