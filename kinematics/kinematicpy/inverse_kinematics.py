# HEADER
'''   
@file:          inverse_kinematics.py
@brief:    	    This module provides a functor class regaring the inverse kinematic calculations of a manipulator.
@author:        Nima Ramezani Taghiabadi
                PhD Researcher
                Faculty of Engineering and Information Technology
                University of Technology Sydney
                Broadway, Ultimo, NSW 2007
                Room No.: CB10.03.512
                Phone:    02 9514 4621
                Mobile:   04 5027 4611
                Email(1): Nima.RamezaniTaghiabadi@student.uts.edu.au 
                Email(2): nima.ramezani@gmail.com
                Email(3): nima_ramezani@yahoo.com
                Email(4): ramezanitn@alum.sharif.edu

@version:	    2.0
Last Revision:  12 September 2014
'''
# BODY

import numpy, math, time, copy

import packages.nima.robotics.kinematics.kinematicpy.forward_kinematics as fklib 
import packages.nima.robotics.kinematics.task_space.endeffector as eflib 
import packages.nima.mathematics.vectors_and_matrices as vecmat

from packages.nima.mathematics import rotation
from packages.nima.mathematics import polynomials
from packages.nima.mathematics import discrete

class Inverse_Kinematics_Settings():
    '''
    '''    
    # Class Sets:
    all_run_modes  = [ 'normal_run', 'binary_run' ]
    all_algorithms = [ 'Jacobian Inverse', 'Jacobian Transpose', 'Jacobian Pseudo Inverse', 'DLS Method' ]

    # Class Dictionaries:
    alg_dic = {'Jacobian Inverse':'JI','Jacobian Transpose':'JT', 'Jacobian Pseudo Inverse':'JPI', 'DLS Method':'DLS'}


    def __init__(self, algorithm = 'Jacobian Pseudo Inverse',run_mode = 'normal_run', num_steps = 100, step_size = 1.0, representation_of_orientation_for_binary_run = 'vectorial_identity'): 

        assert run_mode in self.__class__.all_run_modes
        assert algorithm in self.__class__.all_algorithms
        
        self.algorithm            = algorithm
        self.ef_settings          = eflib.Endeffector_Settings(default_orientation_error_function = 'Axis Inner Product')  
        self.run_mode             = run_mode # 'normal_run' by default
        
        self.number_of_steps      = num_steps
        self.step_size            = step_size
        self.method               = "Jacobian Pseudo-Inverse"
        # self.method               = "Jacobian Transpose"

        self.representation_of_orientation_for_binary_run = representation_of_orientation_for_binary_run
        self.include_current_config = True
        
    def __str__( self ) : 
        '''
        returns the algorithmic setting for the inverse kinematic solver
        '''
        s  = 'Running Mode    : ' + self.run_mode 
        if self.run_mode == 'binary_run':
            s += '(Representation of Orientation for Interpolation : ' + self.representation_of_orientation_for_binary_run + '\n'
        else:    
            s += '\n'
            
        s += 'Number of Steps : ' + str(self.number_of_steps) + '\n'
        
    def algorithm_key(self):
        assert self.algorithm in self.__class__.all_algorithms
        return self.__class__.alg_dic[self.algorithm]

class Inverse_Kinematics( fklib.Forward_Kinematics ):
    '''
    Inherits all properties and methods of a "Forward_Kinematics" class 
    and has some additional properties and methods for inverse kinematic calculations
    Method "update" changes the "configuration" property inherited from "Forward_Kinematics"
    
    
    # inverse_update (call from outside)    -- runs the suff on the grid already, calls stepwise_run or run
    # stepwise_run()                   -- 
    # run()                            -- run() # single run() 
    # step_forward (SINGLE STEP)            -- one update_step() 
    
    '''
    
    def __init__(self, manip_geometry, manip_config, settings ):
        ### def __init__( self, geometry ) : # njoint):
        '''
        '''
        fklib.Forward_Kinematics.__init__(self, manip_geometry, manip_config)
        
        self.settings               = settings 
        
        ## fklib.Forward_Kinematics.__init__(self, DEEPCOPY(??geometry??) ) # njoint)
        
        settings.ef_settings.last_link_number = manip_geometry.nlink - 1
        
        self.endeffector            = eflib.Endeffector(settings.ef_settings)

        
        self.log_info               = (0.000000, 0)
        
               
        self.initial_config         = copy.copy(manip_config)
        
        self.initial_config_list    = []
        
        self.analytic_jacobian.evaluate_requirements(self.endeffector.reference_positions, self.endeffector.reference_orientations)

    def settings_key():
        '''
        generate and return the text key for the settings
        '''
        
    def forward_update(self):
        '''
        runs the "forward_update" method of the super class
        in addition, it updates the endeffector
        '''

        super( Inverse_Kinematics, self ).forward_update() 
        self.endeffector.update(self)

    def take_to_grid_configuration(self, config_number, number_of_intervals):
        '''
        runs the "take_to_grid_configuration" method of the super class
        in addition, it updates the endeffector
        '''
        super( Inverse_Kinematics, self ).take_to_grid_configuration(config_number, number_of_intervals)
        self.endeffector.update(self)

    def take_to_random_configuration(self):
        '''
        runs the "take_to_random_configuration" method of the super class
        in addition, it updates the endeffector
        '''
        super( Inverse_Kinematics, self ).take_to_random_configuration()
        self.endeffector.update(self)
    
    def joint_speed(self):
        # Pose Error is multiplied by step_size
        # err = - self.endeffector.pose_error + numpy.dot(self.endeffector.TJ, self.endeffector.vd)
        err = - self.endeffector.pose_error
        # Error jacobian is placed in Je
        Je  = self.endeffector.EJ
        # right pseudo inverse of error jacobian is calculated and placed in Je_dagger
        #(Je_dagger,not_singular) = mathpy.right_pseudo_inverse(Je)
        Je_dagger = numpy.linalg.pinv(Je)
        # Joint Correction is calculated
        q_dot = numpy.dot(Je_dagger, err)
        
        return vecmat.collapse(q_dot, 0.1)


    def update_rule_nul_space(self, step_size):
        '''
        In this update rule the null space of jacobian is used to keep the joints in their feasible range. Should be used with no Mapping
        '''
            
        A    = numpy.zeros((self.configuration.DOF))
        
        for i in range(0,self.configuration.DOF):
            if self.configuration.settings.joint_handling[i] == 'Trigonometric Mapping':
                A[i] = - 0.1*self.configuration.qstar[i]/(self.configuration.DOF*((2*math.pi)**2))
            elif self.configuration.settings.joint_handling[i] == 'No Mapping':
                mean = 0.5*(self.configuration.qh[i] + self.configuration.ql[i])
                A[i] = - 0.1*(self.configuration.qstar[i] - mean)/(self.configuration.DOF*((self.configuration.qh[i] - self.configuration.ql[i])**2))
            else:
                assert False    
            
        start_kinematic_inversion = time.time()
        err              = step_size * self.endeffector.pose_error
        J                = self.endeffector.EJ
        J_dag            = numpy.linalg.pinv(J)
        J_dag_J          = numpy.dot(J_dag, J)
        In_minus_J_dag_J = numpy.eye(self.configuration.DOF) - J_dag_J
        
        delta_qs        = - numpy.dot(J_dag, err) + numpy.dot(In_minus_J_dag_J, A)
        #delta_qs        = - numpy.dot(J_dag, err)
        
        self.configuration.grow_qstar(delta_qs)
        self.forward_update()
        elapsed_kinematic_inversion = time.time() - start_kinematic_inversion
        (time_til_now, num_iter_til_now ) = self.log_info
        time_til_now     += elapsed_kinematic_inversion
        num_iter_til_now += 1
        self.log_info = (time_til_now, num_iter_til_now )
        
    def ik_direction(self):
        '''
        Returns the joint direction (joint update correction) expected to lead the endeffector closer to the target.
        This direction should be multiplied by a proper stepsize to ensure that the pose error norm is reduced.
        '''        
        # Pose Error is multiplied by step_size
        err = self.endeffector.pose_error
        # Error jacobian is placed in Je
        Je  = self.endeffector.EJ
        # right pseudo inverse of error jacobian is calculated and placed in Je_dagger
        #(Je_dagger,not_singular) = mathpy.right_pseudo_inverse(Je)
        if self.settings.method == "Jacobian Pseudo-Inverse":
            Je_dagger = numpy.linalg.pinv(Je)
        elif self.settings.method == "Jacobian Transpose":
            Je_dagger = Je.T
        # Joint Correction is calculated
        delta_qs = - numpy.dot(Je_dagger, err)
        return delta_qs       

    def update_step(self, step_size):
        '''
        Implements an update rule for the joint configuration. This function performs one iteration of the algorithm.
        '''
        start_kinematic_inversion = time.time()


        delta_qs = step_size*self.ik_direction()

        self.configuration.grow_qstar(delta_qs)
        
        self.forward_update()
       
        elapsed_kinematic_inversion = time.time() - start_kinematic_inversion
        
        (time_til_now, num_iter_til_now ) = self.log_info
        time_til_now     += elapsed_kinematic_inversion
        num_iter_til_now += 1
        self.log_info = (time_til_now, num_iter_til_now )

    def copy_from(self, ik):
        '''
        Transfers all properties from ik to self
        '''
        self.configuration     = copy.deepcopy(ik.configuration)    
        self.transfer_matrices = copy.deepcopy(ik.transfer_matrices)    
        self.analytic_jacobian = copy.deepcopy(ik.analytic_jacobian)    
        self.endeffector       = copy.deepcopy(ik.endeffector)    
    
    def run(self):
        '''
        SINGLE STEP 
        
        runs the inverse kinematic algorithm from the given starting point. 
        
        The stop criteria is the occurance of one of the following:
            1- The counter exceeds property "number_of_steps" of the class and the norm of error is still more than the previous iteration
            2- The solution is found according to the defined precision
        the running time and number of iteration is returned in the "log_info" property
        '''

        run_time = 0.0
        min_config = copy.deepcopy(self)
        counter = 0

        not_yet = not self.endeffector.in_target
        have_time = (counter < self.settings.number_of_steps)
        err_reduced = False

        while not_yet and (have_time or err_reduced): 

            try:
                self.update_step(step_size = self.settings.step_size)
                #self.update_rule_new(step_size = 1.0)
                counter     += 1
    
            except numpy.linalg.linalg.LinAlgError:
                    
                print "********************************************************Singular Matrix ***********************************************************";
                break
    
            not_yet = not self.endeffector.in_target
            have_time = (counter < self.settings.number_of_steps)
            err_reduced = (self.endeffector.error_norm < min_config.endeffector.error_norm)
            if not err_reduced:
                print "Current Error     : ", self.endeffector.error_norm
                print "Min Error Achieved: ", min_config.endeffector.error_norm
    
            if err_reduced:
                min_config  = copy.deepcopy(self)
    
        if not_yet:
            self.copy_from(min_config)

    def run_new(self):
        '''
        SINGLE STEP 
        
        runs the inverse kinematic algorithm from the given starting point. 
        
        The stop criteria is the occurance of one of the following:
            1- The counter exceeds property "number_of_steps" of the class and the norm of error is still more than the previous iteration
            2- The solution is found according to the defined precision
        the running time and number of iteration is returned in the "log_info" property
        '''

        run_time = 0.0
        min_config = copy.deepcopy(self)
        counter = 0

        not_yet = not self.endeffector.in_target
        have_time = (counter < self.settings.number_of_steps)
        err_reduced = False
        not_in_range = True

        while (not_yet or not_in_range) and (have_time or err_reduced): 

            try:
                #self.update_step(step_size = 1.0)
                self.update_rule_nul_space(step_size = 1.0)
                counter     += 1
    
            except numpy.linalg.linalg.LinAlgError:
                    
                print "********************************************************Singular Matrix ***********************************************************";
                break
    
            not_yet      = not self.endeffector.in_target
            have_time    = (counter < self.settings.number_of_steps)
            err_reduced  = (self.endeffector.error_norm < min_config.endeffector.error_norm)
            not_in_range =  not self.configuration.joints_in_range()
    
            if err_reduced:
                min_config  = copy.deepcopy(self)
 
        print 'I came out and in range: ', not not_in_range
        
        #assert False
        if not_yet:
            self.copy_from(min_config)
    
    def run_binary(self, verbose):
        '''
        SINGLE STEP 
        
        Implement "Binary Stepwise run" that 
        recursively introduces intermediate target poses reducing the distance from current to final target pose. 
        to select a target pose somewhere between the starting and final target poses. 
        Procedure:
        If the IK algorithm (method: "run") is unable to converge to the final target pose in first attempt, 
        an auxiliary pose in the middle point between the starting and final poses is selected as the target.
        If the algorithm is unable to converge again, another pose in the middle of the current and the auxiliary poses will be selected. 
        This procedure continues until the target is close enough to the starting point so that the algorithm can find a solution. 
        When a solution is found, the target is reset to the final target and the procedure continues.
        For position, the middle point is simply identified as the midpoint of the line connecting start and target poses. 
        For orientation, first we need a non-redundant convection which represents both start and target orientations with three individual parameters. 
        Then, the linear midpoint is calculated and converted inversely to its corresponding rotation matrix. There are several interpolation methods 
        which can be used for finding an orientation midpoint between the starting and target poses. In this method, 
        the taskspace parametrization (here a non-redundant representation of orientation) is specified via string variable: 
        "self.settings.representation_of_orientation"  
        i.e: 'vectorial_identity' which means : "Vectorial Representation of Rotation" with "Identity Parametrization" 
        '''

        v0 = numpy.zeros(3)    
        poly = polynomials.Polynomial1_Interpolator()

        in_target =         True
        err_changed =       True
        end_of_path =       False
        s =                 float(1)
        run_time    = 0.0000000
        counter = 0
        num_iters = 0  
              
        end_position             = []    
        end_orientation_vector   = []    
        end_orientation_matrix   = []    
        for tp in self.endeffector.reference_positions:
            end_position.append(tp.rd)
        for tf in self.endeffector.reference_orientations:
            end_orientation_matrix.append(tf.rd)
            end_orientation_vector.append(rotation.orientation_vector(tf.rd, self.settings.representation_of_orientation_for_binary_run))
    
        while err_changed and (not (end_of_path and in_target)) and (counter < self.settings.number_of_steps) and (s > 0.01):
            start_position    = []    
            coeff_position    = []    
            start_orientation_vector = []    
            coeff_orientation_vector = []    
            for i in range(0, len(self.endeffector.reference_positions)):
                tp = self.endeffector.reference_positions[i]
                start_position.append(tp.ra)
                poly.find_coefficients(1.0, start_position[i], end_position[i], v0, v0)
                coeff_position.append(copy.copy(poly))
            for i in range(0, len(self.endeffector.reference_orientations)):
                tf = self.endeffector.reference_orientations[i]
                start_orientation_vector.append(rotation.orientation_vector(tf.ra , self.settings.representation_of_orientation_for_binary_run))
                poly.find_coefficients(1.0, start_orientation_vector[i], end_orientation_vector[i], v0, v0)
                coeff_orientation_vector.append(copy.copy(poly))

            '''
            print "EF initial target position = ", end_position[0]
            print "EF interpo target position = ", self.endeffector.reference_positions[0].rd
            print
            
            assert False
            '''
    
            s = 1.0
            in_target = False
            err_reduced = False
    
            while not (in_target or err_reduced or (s < 0.01)) :

                for i in range(0,len(self.endeffector.reference_positions)):
                    self.endeffector.reference_positions[i].rd  = coeff_position[i].interpolated_position(s)
    
                for i in range(0,len(self.endeffector.reference_orientations)):
                    self.endeffector.reference_orientations[i].rd = rotation.rotation_matrix(coeff_orientation_vector[i].interpolated_position(s), self.settings.representation_of_orientation_for_binary_run)
    
                self.configuration.q_to_qstar()
                
                self.forward_update()
                
                counter += 1 
                num_iters += 1 
        
                prev_norm = self.endeffector.error_norm
    
                self.run()
                
                if not self.endeffector.in_target:
                    assert not (prev_norm < self.endeffector.error_norm)
    
                #run_time  += time_run  
                #num_iters += num_iters_run
                counter   += 1
    
                in_target = self.endeffector.in_target
                err_reduced = (self.endeffector.error_norm < prev_norm)
                err_changed = (self.endeffector.error_norm != prev_norm)
                end_of_path = (s == 1.0)
                if verbose:
                    print 'counter        = ', counter
                    print 'previous error = ', prev_norm
                    print 'current  error = ', self.endeffector.error_norm
                    print 'in_target = ',in_target,' err_reduced = ',err_reduced, ' s = ',s
                
                s = 0.5*s
    
        # return the initial target pose(s)
        for i in range(0,len(self.endeffector.reference_positions)):
            self.endeffector.reference_positions[i].rd = numpy.copy(end_position[i])
    
        for i in range(0,len(self.endeffector.reference_orientations)):
            self.endeffector.reference_orientations[i].rd = numpy.copy(end_orientation_matrix[i])
    
        self.forward_update()


    def inverse_update(self):
        '''
        '''
        self.log_info               = (0.000000, 0)
                
        self.analytic_jacobian.evaluate_requirements(self.endeffector.reference_positions, self.endeffector.reference_orientations)

        if self.settings.include_current_config:

            self.initial_config         = copy.copy(self.configuration)
            
            if self.endeffector.in_target:
                return True
                
            if self.settings.run_mode == 'binary_run':
                self.run_binary(True)
                
            elif self.settings.run_mode == 'normal_run':
                self.run()
                #self.run_new()

        self.start_node = 0
        p = len(self.initial_config_list) 
        while (not self.endeffector.in_target) and (self.start_node < p):

            self.configuration.q = numpy.copy(self.initial_config_list[self.start_node])
            self.initial_config  = copy.copy(self.configuration) # Just for keeping the information of the initial status (Initial status will be shown later in the results log file)
            self.configuration.initialize()

            self.forward_update()
                
            if self.settings.run_mode == 'binary_run':
                self.run_binary(False)
            elif self.settings.run_mode == 'normal_run':
                self.run()
                #self.run_new()
 
            self.start_node += 1

 
        
