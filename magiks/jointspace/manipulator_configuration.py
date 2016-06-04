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
@version:	    3.0
Last Revision:  11 August 2015
'''
'''
Major change from previous version:
    1- methods set_joint_bounds() added to class Manipulator_Configuration()
    2- functions q_to_qstar() and qstar_to_q() renamed to mapto_virtual() and mapfrom_virtual() respectively
    3- property qstar renamed to qvr
    
'''

# BODY
import numpy as np, math, random, general_python as genpy, copy

from math_tools import general_math as gen
from math_tools.algebra import vectors_and_matrices as vecmat
from math_tools.geometry import trigonometry as trig
from math_tools.discrete import discrete

## @brief This class contains joint properties of a chained-link manipulator.
#   These properties include degree of freedom, joint types, joint limits and mapping functions.
class Manipulator_Configuration_Settings:
	## The Class Constructor:
	#  @param njoint A positive integer specifying the number of joints of the manipulator 	
	#  @param DOF A positive integer specifying the number of degrees of freedom
    #  @param joint_mapping A string specifying the default mapping function.
    #         This parameter must be chosen among these options:
    #         * \c NM: No Mapping (Identity mapping function)
    #         * \c LM: Linear Mapping
    #         * \c TM: Trigonometric Mapping   
    def __init__(self, njoint = 0, DOF= 0, joint_mapping = 'NM'):
        self.njoint = njoint
        ## An integer: Is a short form of Degrees of Freedom and represents the number of free joints
        self.DOF = DOF

        ## An array of string containing the specific mapping function for each joint
        #  The array is of length \c DOF and is defined only for free joints.
        self.joint_handling = [joint_mapping for i in range(DOF)]

        ## An array of booleans specifying which joints are limited. The array is of length \c DOF and is defined only for free joints.
        #  The array is of length \c DOF and is defined only for free joints.
        self.limited        = [True for i in range(0, DOF)]

        ## An array of string containing indivifual labels or names the for each joint. 
        #  The array is of length \c DOF and is defined only for free joints.
        self.joint_label    = ['Joint ' + str(i) for i in range(DOF)]

        ## A boolean specify if the joint limits must be respected when new joint values are being set.
        #  If this property is True, the system checks for the given joint values to be in the defined feasible range
        #  and fails with an error if given joint values are beyond the limits.
        #  If False, any value for the joints are accepted.
        self.joint_limits_respected = True

        ## A vector of real numbers representing lower bounds for the joint configuration. 
        #  The array is of length /c njoint and is defined for both fixed and free joints.
        self.ql   = [-math.pi for i in range(0, njoint)]
        ## A vector of real numbers representing higher bounds for the joint configuration
        #  The array is of length /c njoint and is defined for both fixed and free joints.
        self.qh   = [math.pi for i in range(0, njoint)]

        ## A list of booleans representing the type of the corresponding joint number. 
        #  If True, the joint is prismatic, if False it is revolute.
        #  The array is of length /c njoint and is defined for both fixed and free joints.
        self.prismatic = [False for i in range(0, njoint)]

        # A list of booleans representing whether the corresponding joint is free or fixed at a certain value. The default is True for all joints
        self.free = [True for i in range(0, njoint)]
   
## @brief This class represents a model of the joint configuration of a chained-link manipulator.
#   It contains current actual and virtual(mapped) joint values and the required joint settings. 
#   The class also, has methods to check the feasibility of given joint values, set a given configuration,
#   forward and inverse conversion of joint values into the mapped virtual space and a number of 
#   auxiliary methods. This class contains everything you need to handle joint vlaues of the manipulator.  
class Manipulator_Configuration(object):
	## The Class Constructor:
	#  @param settings An instance of class Manipulator_Configuration_Settings containing the joint settings of the manipulator
    #         including joint types, joint limits and mapping functions	
    def __init__(self, settings):
        # An instance of class Manipulator_Configuration_Settings containing the joint settings of the manipulator
        # including joint types, joint limits and mapping functions
        self.config_settings = settings

        # A list of real numbers representing the current joint configuration of the manipulator in the jointspace
        self.q     = 0.5*(settings.qh + settings.ql)        

        # A list of real numbers representing the current joint configuration of the manipulator in the mapped jointspace
        self.qvr   = None

        self.initialize()
        '''
        If you have a method in class A and you intended to have a method with the same name in a class B inherited from A,
        you should not call this method in class A, so I created a function set_configuration() which is called by set_config()
        '''

        self.mapto_virtual()
        self.update_virtual_jacobian_multipliers()
        
    ## Only use this function to set the joint limits. Do not change the properties directly.
    #  @param ql A numpy array of length \c njoint containing the lower bounds of the joints 
    #  @param qh A numpy array of length \c njoint containing the upper bounds of the joints 
    def set_joint_bounds(self, ql, qh):
        self.config_settings.ql = copy.copy(ql)
        self.config_settings.qh = copy.copy(qh)
        self.initialize()

    ## Use this function to see the current configuration of the manipulator
    #  @return A string containing the current values of joints in degrees
    def config_str(self):
        s = "q = "
        s += vecmat.vector_to_str((180.0/math.pi)*self.q, format="%.2f")
        s += "    (Joints are"
        if not self.joints_in_range(self.free_config(self.q)):
            s += " NOT"
        s += " all in range)"
        
        return s

    # \cond  
    ## protected
    def bring_revolute_joints_to_standard_range(self):
        '''
        Change revolute joint values (joint angles) if they are out of standard range (-pi  +pi) to their equivalent value in the standard range
        '''
        for i in range(0, self.config_settings.njoint):
            if not self.config_settings.prismatic[i]:
                self.q[i] = trig.angle_standard_range( self.q[i] ) 
    # \endcond
	## This function is used to check if a given value for specific joint is in its feasibility range
	#  @param i An integer between 0 to 6 specifying the joint number to be checked
	#  @param qi A float parameter specifying the value for the i-th joint
	#  @return A boolean: True if the given joint angle qi is in feasible range for the i-th joint (within the specified joint limits for that joint)
    def joint_in_range(self, i, qi):
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

	## This function converts the free joint values from unlimited virtual joint-space into their actual values.
    #  @param qs A numpy array of length \c DOF containing the values of free joints in the virtual joint-space.
    #  @return A numpy array of length \c DOF containing the actual values of free joints equivalent to the given argument \c qs.
    def mapfrom_virtual(self, qs):
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
                    assert False, genpy.err_str(__name__, self.__class__.__name__, 'mapfrom_virtual', self.config_settings.joint_handling[i] + " is not a valid value for joint_handling")
            else:
                qq[i] = qs[i]
                
        # return self.free_config_inv(qq)
        return qq

	## This function converts the current actual free joint values (picked from property \c q) 
    #  into their equivalent values in the unlimited virtual joint-space.
    #  @return A numpy array of length \c DOF containing the current values of free joints in the unlimited virtual joint-space.
    def mapto_virtual(self):
        qf = self.free_config(self.q)

        self.qvr = np.zeros((self.config_settings.DOF))
         
        for i in range(self.config_settings.DOF):
            if (not self.config_settings.limited) or (self.config_settings.joint_handling[i] == 'NM'):
                self.qvr[i] = qf[i]
            elif self.config_settings.joint_handling[i] == 'LM':
                assert self.joint_in_range(i, qf[i]), "Joint " + str(i) + " = " + str(qf[i]) + " is not in feasible range: (" + str(self.config_settings.ql[i]) + ',' + str(self.config_settings.qh[i]) + ')'
                self.qvr[i] = self.jmc_a[i] * qf[i] + self.jmc_b[i] 
            elif self.config_settings.joint_handling[i] == 'TM':
                '''
                ql = self.config_settings.ql[i]
                qh = self.config_settings.qh[i]
                print "i: ", i, "ql: ", ql," q: ", qf[i],"qh: ", qh, " g: ", self.jmc_g[i], " f: ", self.jmc_f[i], " c: ", (qf[i] - self.jmc_g[i]) / self.jmc_f[i]
                '''
                assert self.joint_in_range(i, qf[i]), "Joint " + str(i) + " = " + str(qf[i]) + " is not in feasible range: (" + str(self.config_settings.ql[i]) + ',' + str(self.config_settings.qh[i]) + ')'
                self.qvr[i] = trig.arccos((qf[i] - self.jmc_g[i]) / self.jmc_f[i])
            elif self.config_settings.joint_handling[i] == 'TGM':
                assert self.joint_in_range(i, qf[i]), "Joint " + str(i) + " = " + str(qf[i]) + " is not in feasible range: (" + str(self.config_settings.ql[i]) + ',' + str(self.config_settings.qh[i]) + ')'
                self.qvr[i] = trig.arccos((math.tan(0.5*qf[i]) - self.jmc_g[i]) / self.jmc_f[i])
            else:
                print 'Wrong Joint Handling (Free Joint ' + str(i) + '): ' + self.config_settings.joint_handling[i]
                assert False, genpy.err_str(__name__, self.__class__.__name__, 'q_to_q', self.config_settings.joint_handling[i] + " is not a valid value for joint_handling")

    # \cond
    ## protected            
    def update_virtual_jacobian_multipliers(self):
        '''
        joint_limit_multipliers are coefficients of forward and inverse mapping to/from limited to/from unlimited jointspace.
        i.e: jmc_a is the derivative of q to q_star
        (Refer also to the comments of method: initialize in this class)
        This method calculates and updates the values of one of the joint limit multipliers: "jmc_c" which is called "joint_limit_jacobian_multipliers"
        Each column of the error jacobian is multiplied by the corresponding element of vector "jmc_c": Jstar = Je * diag(jmc_c) 
        Since the coefficient "jmc_c" for a joint, depends on the current value of that joint, it should be updated everytime "q" changes
        The other multipliers: "jmc_a", "jmc_b", "jmc_f" and "jmc_g" do not depend on the joints, therefore do not need to be updated
        '''
        func_name = "update_virtual_jacobian_multipliers"
        for i in range(0,self.config_settings.DOF):
            if self.config_settings.joint_handling[i] == 'NM':
                self.jmc_c[i] = 1.0

            elif self.config_settings.joint_handling[i] == 'LM':
                self.jmc_c[i] = 1.0 / self.jmc_a[i]

            elif self.config_settings.joint_handling[i] == 'TM':
                self.jmc_c[i] = - math.sin(self.qvr[i])/self.jmc_a[i]

            elif self.config_settings.joint_handling[i] == 'TGM':
                t = self.jmc_f[i] * math.cos(self.qvr[i]) + self.jmc_g[i]
                self.jmc_c[i] = - 2.0 * math.sin(self.qvr[i])/(self.jmc_a[i]*(1 + t**2))
            else:
                assert False, genpy.err_str(__name__, self.__class__.__name__, 'update_virtual_jacobian_multipliers', self.config_settings.joint_handling[i] + " is not a valid value for joint_handling")

    # \endcond
    
    # This function is implemented once when a new instance of the object is constructed.
    # If you chang any joints from fixed to free or vice-versa, after the construction of the object, you will need to call this function.
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
        jmc_a and jmc_b are auxiliary coefficients which are used for conversion from "q" to "qvr" space
        (Please refer to the joint limits section of the documentation associated with this code)

        To make sure that all joints are in their desired range identified as: (self.ql, self.qh), the real jointspace is mapped into an unlimited jointspace.
        
        "q    " represent the real joint values which are in a limited jointpace
        "qvr" represent the mapped joint values in unlimited jointspace
        
        The following code creates and defines "Joint Limit Multipliers". These are coefficients for this jointspace mapping - (jmc: Jointspace Mapping Coefficients)
        in linear mapping:
        
            q  = a * qvr + b
            qs = (q - b) / a
        
        in trigonometric mapping:

            q      = f * cos(qvr) + g
            qvr  = arccos [  (q - g) / f ]
            
        a, b, f and g are calculated in such a way that the real joint values which are in their feasible range are mapped into an equivalent qvr in an unlimited space (-pi, +pi)
        
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
                    
        # map q to qvr for the first time:
        self.mapto_virtual()

        # assign the value of jmc_c the jacobian multiplier:
        self.update_virtual_jacobian_multipliers()

    # \cond
    def joint_correction_in_range(self, dq):
        '''
        Return True if the joint configuration will be in the desired range after being added by the given "dq"
        '''
        in_range = True
        for i in range(0,self.DOF):
            in_range = in_range and (self.q[i] + dq[i] <= self.qh[i]) and (self.q[i] + dq[i] >= self.ql[i]) 
            i = i + 1
        return in_range
    # \endcond

    ## Sets the configuration given values of the free joints in the virtual joint-space.
    #  This function changes and updates the values of the joint configuration in the unlimited virtual space (property \c qvr).
    #  The actual joint values (property \c q) is then calculated and updated as well.
    #  This method changes the joint configuration. Therefore a forward kinematics update will be implemented.
    #  @param qvrd A numpy array of length \c DOF containing the desired values of the free joints in the unlimited space.
    def set_config_virtual(self, qvrd):
        self.qvr   = qvrd
        qd         =  self.mapfrom_virtual(qvrd)
        permission = not self.config_settings.joint_limits_respected
        if not permission:
            permission = self.joints_in_range(qd)
        if permission:
            self.q = self.free_config_inv(qd)
            self.update_virtual_jacobian_multipliers()
            return True
        else:
            assert False, genpy.err_str(__name__, self.__class__.__name__, 'set_config_virtual', 'The joint values computed from qvr are out of range while joint limits are respected.')
            return False
        
	## The correction of joints is always restricted by joint limits and maximum feasible joint speed
    #  If you want to move the joints in a desired direction, how much are you allowed to move in order to respect joint position and speed limits?	
    #  This function returns a feasible interval for the stepsize in the given direction. 
    #  In other words, it gives an interval for the magnitude of the joint angles correction vector 
    #  so that both joint position and speed limits are fulfilled. 
    #  The joint direction of change must be multiplied by a scalar value in this range so that the applied changes are feasible.
	#  @param direction A numpy array of size \c DOF specifying the direction of change	
	#  @param max_speed A float parameter specifying the maximum feasible speed for a joint change. (Set as infinity by default)	
	#  @param delta_t The step time in which the change(correction) is to be applied.
	#  @return An instance of type <a href="http://pyinterval.googlecode.com/svn/trunk/html/index.html">Interval()</a>
    def joint_stepsize_interval(self, direction, max_speed = gen.infinity, delta_t = 0.001):
        etta_l = []
        etta_h = []

        # for ii in range(self.config_settings.njoint):
        for i in range(self.config_settings.DOF):
            if not gen.equal(direction[i], 0.0):
                if self.config_settings.limited[i] and self.config_settings.joint_handling[i] == 'NM':
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

    
    ## This function picks and returns the values of free joints among all the joints in the given argument \c q.
    #  @param A numpy vector of size \c njoint containing the actual values of all the joints
    #  @return A numpy vector of size \c DOF containing the values of free joints
    def free_config(self, q):
        fc = np.zeros((self.config_settings.DOF))
        j = 0
        for jj in range(0,self.config_settings.njoint):
            if self.config_settings.free[jj]:                    
                fc[j] = q[jj]
                j = j + 1
        return fc

    # \cond
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
    # \endcond

	## Use this function to set the joint configuration to the given desired values.
    #  If property joint_limits_respected is True, the system checks for given qd to be in the defined feasible range
    #  and fails with an error if given joint values are beyond the limits
    #  If property joint_limits_respected is False, any value for the joints are accepted.
	#  @param qd A numpy array of size 7 containing the desired joint values to be set
	#  @return A boolean: True if the given joints are in range and the configuration is set, False if not	
    def set_config(self, qd, set_virtual = True):
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

            self.mapto_virtual()
            self.update_virtual_jacobian_multipliers()

            return True
        else:
            print "Warning from configuration.set_config(): Given joints are not in their feasible range"
            for i in range(self.config_settings.DOF):
                if not self.joint_in_range(i, qd[i]):        
                    print    
                    print "Joint Number: ", i
                    print "Lower Limit  :", self.ql[i]
                    print "Desired Value:", qd[i]
                    print "Upper Limit  :", self.qh[i]
            return False
    
    ## Generates a random joint configuration in the defined feasible range in the settings.
    # This function does not change manipulator configuration.
    # @return An array of real numbers containing the joint values of the generated random configuration. 
    def random_config(self):
        q_rnd = np.zeros(self.config_settings.DOF)
        
        for j in range(0, self.config_settings.DOF):
            q_rnd[j] = self.ql[j] + random.random()*(self.qh[j] - self.ql[j])
        return q_rnd
    
    ## If The feasible range of each joint is divided into a number of equal intervals,
    #  a joint-space lattice or grid is established.
    #  If a manipulator has \f$ n \f$ degrees of freedom, and each joint is divided to \f$ N + 1 \f$ intervals,   
    #  a typical lattice \f$ \boldsymbol{\Lambda}_N \f$ in the joint space 
    #  can be generated in the following form:
    #  \f[
    #  \boldsymbol{\Lambda}_N = \bigg \lbrace \boldsymbol{q}_o + \sum _{i = 1} ^{n} k_i \cdot a_i \cdot  \boldsymbol{b}_i \quad \bigg \arrowvert \quad  k_i \in \mathbb{Z}_N \bigg \rbrace 
    #  \f]
    #  This function creates a lattice in which \a N is specified by argument <tt> number_of_intervals </tt> 
    #  and returns the joint configuration corresponding to a specific node of the grid.
    #  This node is specified by argument <tt> config_number </tt> which is the order of the configuration in the gridded joint-space.
    #  The function does not change manipulator configuration.
    #  @return A numpy array of length \c DOF containing the joint values of the specified node of the generated grid. 
    def grid_config(self, config_number, number_of_intervals):
        q_grd = np.zeros(self.config_settings.DOF)
        A     = discrete.number_in_base(N = config_number, base = number_of_intervals, n_digits = self.config_settings.DOF)

        for j in range(0, self.config_settings.DOF):
            q_grd[j] = self.ql[j] + (2*A[j] + 1)*(self.qh[j] - self.ql[j])/(2*number_of_intervals)

        return q_grd

# \cond    
    def objective_function_gradient(self, k = 1.0):
        dof = self.config_settings.DOF
        qh  = self.qh
        ql  = self.ql
        u   = np.zeros(dof)
        qs  = trig.angles_standard_range(self.qvr)
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
        qs  = trig.angles_standard_range(self.qvr)
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
        
# \endcond
