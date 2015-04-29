# HEADER

## @file        	inverse_kinematics.py
#  @brief           This module provides a functor class regaring the inverse kinematic calculations of a manipulator
#  @author      	Nima Ramezani Taghiabadi
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. :   04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	4.0
#
#  Last Revision:  	03 January 2015

# BODY

import math, time, copy, numpy as np

from packages.nima.mathematics.geometry import geometry as geo, trajectory as trajlib
from packages.nima.mathematics.algebra import polynomials, vectors_and_matrices as vecmat
from packages.nima.mathematics.discrete import discrete

import packages.nima.robotics.kinematics.jointspace.manipulator_configuration as conflib
import packages.nima.robotics.kinematics.taskspace.endeffector as eflib

# Module Dictionaries:
key_dic = {
    'JI'            : 'Jacobian Inverse',
    'JT'            : 'Jacobian Transpose',
    'JPI'           : 'Jacobian Pseudo Inverse',
    'DLS(CDF)'      : 'Damped Least Squares Method(Constant Damping Factor)',
    'DLS(ADF)'      : 'Damped Least Squares Method(Adaptive Damping Factor)'
}


class Inverse_Kinematics_Settings():
    '''
    '''    
    # Class Sets:
    all_run_modes  = [ 'normal_run', 'binary_run' ]
    all_algorithms = [ 'JI', 'JT', 'JPI', 'DLS(CDF)', 'DLS(ADF)' ]


    def __init__(self, algorithm = 'JPI', run_mode = 'normal_run', num_steps = 100): 

        func_name = "__init__()"

        assert run_mode in self.__class__.all_run_modes, self.__class__.err_head + func_name + ": The given run_mode is not known!"
        assert algorithm in self.__class__.all_algorithms, self.__class__.err_head + func_name + ": The given algorithm is not known!"
        
        self.ngp_active             = True
        self.joint_limits_respected = True
        self.return_min_config      = False

        self.algorithm              = algorithm
        self.run_mode               = run_mode
        self.number_of_steps        = num_steps
        self.initial_damping_factor = 1.0
        self.real_time              = False
        self.time_step              = 0.020

        self.representation_of_orientation_for_binary_run = 'vectorial_identity'
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
            
        s += 'Algorithm       : ' + key_dic[self.algorithm] + '\n'
        s += 'Number of Steps : ' + str(self.number_of_steps) + '\n'
        return s
        
    def algorithm_key(self):
        assert self.algorithm in self.__class__.all_algorithms
        return self.__class__.alg_dic[self.algorithm]

class Inverse_Kinematics( eflib.Endeffector ):
    '''
    Inherits all properties and methods of a "Forward_Kinematics" class 
    and has some additional properties and methods for inverse kinematic calculations
    Method "update" changes the "configuration" property inherited from "Forward_Kinematics"
    
    
    # inverse_update (call from outside)    -- runs the suff on the grid already, calls stepwise_run or run
    # stepwise_run()                   -- 
    # run()                            -- run() # single run() 
    # step_forward (SINGLE STEP)            -- one update_step() 
    
    '''
    err_head       = "Error from inverse_kinematics.Inverse_Kinematics." 
    
    def __init__(self, config_settings, geo_settings, end_settings, ik_settings):
        ### def __init__( self, geometry ) : # njoint):
        '''
        '''
        super(Inverse_Kinematics, self).__init__(config_settings, geo_settings, end_settings)    

        self.ik_settings               = ik_settings 
        
        ## fklib.Forward_Kinematics.__init__(self, DEEPCOPY(??geometry??) ) # njoint)
        
        self.end_settings.last_link_number = geo_settings.nlink - 1
        
        self.log_info               = (0.000000, 0)
        
        # self.initial_config       = self.free_config(self.q)
        self.initial_config         = None
        self.damping_factor         = ik_settings.initial_damping_factor
        
        self.initial_config_list    = []


        
    def settings_key():
        '''
        generate and return the text key for the settings
        '''
        
    def joint_speed(self):
        # Pose Error is multiplied by step_size
        # err = - self.pose_error + np.dot(self.TJ, self.vd)
        err = - self.pose_error()
        # Error jacobian is placed in Je
        Je  = self.error_jacobian()
        # right pseudo inverse of error jacobian is calculated and placed in Je_dagger
        #(Je_dagger,not_singular) = mathpy.right_pseudo_inverse(Je)
        Je_dagger = np.linalg.pinv(Je)
        # Joint Correction is calculated
        q_dot = np.dot(Je_dagger, err)
        
        return vecmat.collapse(q_dot, 0.1)

    def ik_direction(self, u = None):
        '''
        Returns the joint direction (joint update correction) expected to lead the endeffector closer to the target.
        This direction should be multiplied by a proper stepsize to ensure that the pose error norm is reduced.
        '''        
        # Pose Error is multiplied by step_size
        err = self.pose_error()
        # Error jacobian is placed in Je
        Je  = self.error_jacobian()

        # right pseudo inverse of error jacobian is calculated and placed in Je_dagger
        #(Je_dagger,not_singular) = mathpy.right_pseudo_inverse(Je)
        if self.ik_settings.algorithm == "JI":
            Je_dagger = np.linalg.inv(Je)
        elif self.ik_settings.algorithm == "JPI":
            Je_dag = np.linalg.pinv(Je)
        elif self.ik_settings.algorithm == "JT":
            Je_dag = - Je.T
        elif self.ik_settings.algorithm in ["DLS(CDF)", "DLS(ADF)"]:
            Je_dag = vecmat.right_dls_inverse(Je, self.damping_factor)
        else:
            assert False, genpy.err_str(__name, self.__class__.__name__, 'ik_direction', self.ik_settings.algorithm + " is an unknown value for algorithm")

        # Joint Correction is calculated here:
        delta_qs = - np.dot(Je_dag, err)
    
        if u != None:
            assert len(u) == self.config_settings.DOF
            J_dag_J           = np.dot(np.linalg.pinv(Je), Je)
            In_minus_J_dag_J  = np.eye(self.config_settings.DOF) - J_dag_J
            dq                = np.dot(In_minus_J_dag_J, u)
            # print "Internal motion: ", dq
            delta_qs         += dq
        
        return delta_qs       

    def optimum_stepsize(self, direction):
        if self.ik_settings.algorithm in ['JPI','JI']:
            k = 1.0
        elif self.ik_settings.algorithm in ['JT', 'DLS(CDF)', 'DLS(ADF)']:
            J_delta_q = np.dot(self.error_jacobian(), direction)        
            k = - np.sum(J_delta_q*J_delta_q)/np.sum(J_delta_q*self.pose_error())
        else:
            assert False, genpy.err_str(__name__, self.__class__.__name__, 'optimum_stepsize', self.ik_settings.algorithm + " is an unknown value for algorithm")

        # assert k >= 0.0
        '''
        if self.ik_settings.joint_limits_respected:
            (el, eh)  = self.joint_stepsize_interval(direction = direction) 
            # assert el < 0.0
            if eh < k: # k = 1.0 when algorithm is JPI
                k = eh  
        '''
        return k        

    def update_step(self):
        '''
        Implements an update rule for the joint configuration. This function performs one iteration of the algorithm.
        '''
        '''
        print 'US Start: **************************'
        print 'US: qqs', np.sum(self.q), np.sum(self.qstar)
        print 'US: per', np.sum(self.pose_error())
        print 'US: ejt', np.sum(self.task_point[0].error_jacobian()), np.sum(self.task_frame[0].error_jacobian())
        print 'US: EJ ', np.sum(self.error_jacobian()[0,:])
        '''
           
        start_kinematic_inversion = time.time()
        if self.ik_settings.ngp_active:
            u = self.objective_function_gradient(k = 1.0)
            # print "u = ", u
        else:
            u = None    
        jdir     = self.ik_direction(u)
        k        = self.optimum_stepsize(jdir)
        delta_qs = k*jdir
       
        assert self.set_qstar(self.qstar + delta_qs)
        self.clear_pose()
        self.T = None
        self.H = None
        self.ajac.clear()

        elapsed_kinematic_inversion = time.time() - start_kinematic_inversion
        
        (time_til_now, num_iter_til_now ) = self.log_info
        time_til_now     += elapsed_kinematic_inversion
        num_iter_til_now += 1
        self.log_info = (time_til_now, num_iter_til_now )

    def copy_from(self, ik):
        '''
        Transfers all properties from ik to self
        '''
        self.config_settings = copy.copy(ik.config_settings)
        self.geo_settings    = copy.copy(ik.geo_settings)
        self.end_settings    = copy.copy(ik.end_settings)
        self.ik_settings     = copy.copy(ik.ik_settings)

        self.damping_factor  = ik.damping_factor

        self.q               = copy.copy(ik.q)    
        self.qstar           = copy.copy(ik.qstar)
        self.jmc_a           = copy.copy(ik.jmc_a)
        self.jmc_b           = copy.copy(ik.jmc_b)
        self.jmc_c           = copy.copy(ik.jmc_c)
        self.jmc_f           = copy.copy(ik.jmc_f)
        self.jmc_g           = copy.copy(ik.jmc_g)

        self.T               = copy.copy(ik.T)
        self.H               = copy.copy(ik.H)

        self.ajac = copy.deepcopy(ik.ajac)    

        self.mp = ik.mp
        self.mo = ik.mo
        
        for i in range(len(ik.task_point)):
            self.task_point[i] = copy.deepcopy(ik.task_point[i])
            
        for i in range(len(ik.task_frame)):
            self.task_frame[i] = copy.deepcopy(ik.task_frame[i])

        self.current_pose              = copy.copy(ik.current_pose)
        self.current_pose_error        = copy.copy(ik.current_pose_error)
        self.current_error_norm        = copy.copy(ik.current_error_norm)

        self.all_task_points_in_target = ik.all_task_points_in_target
        self.all_task_frames_in_target = ik.all_task_frames_in_target
        self.is_in_target      = ik.is_in_target
        
        self.geo_jac           = copy.copy(ik.geo_jac)
        self.err_jac           = copy.copy(ik.err_jac)    
        self.log_info          = copy.copy(ik.log_info)
        
        

    def run_dls_vdf(self):
        '''
        runs the IK iterations following Damped Least Squares method with Variable Damping Factor
        The damping factor starts from the initial value. 
        At each time step, if the error norm is reduced, the joint correction is applied and the damping factor is reduced to half and 
        if it is increased, the algorithm returns to previous configuration and the DF is multiplied by 2.
        This will continue until the error norm is reduced or maximum number of iterations achieved.
        '''
        counter = 0
        self.damping_factor = self.ik_settings.initial_damping_factor
        not_arrived = not self.in_target()
        have_time   = (counter < self.ik_settings.number_of_steps)
        err_reduced = False
        frontier    = copy.deepcopy(self)

        while not_arrived and have_time: 

            try:
                frontier.update_step()
                counter     += 1
    
            except np.linalg.linalg.LinAlgError:
                    
                print "******** Singular Matrix ********";
                break
    
            err_reduced = (frontier.pose_error_norm() < self.pose_error_norm())
            '''
            print
            print "Counter        : ", counter
            print "Damping Factor : ", self.damping_factor
            print "Frontier    DF : ", frontier.damping_factor
            print "Current  Error : ", self.pose_error_norm()
            print "Frontier Error : ", frontier.pose_error_norm()
            '''

            if err_reduced:
                # print "Error Reduced by DF = ", frontier.settings.damping_factor
                self.copy_from(frontier)
                frontier.damping_factor = frontier.damping_factor/2
            else:
                # print "Error NOT Reduced by DF = ", frontier.settings.damping_factor
                df       = frontier.damping_factor*2
                (niter, telap) = frontier.log_info
                frontier = copy.deepcopy(self)
                frontier.damping_factor = df
                self.log_info = (niter, telap)
                frontier.log_info = (niter, telap)

            rs = self.ik_settings.joint_limits_respected
            ir = self.joints_in_range(self.free_config(self.q))

            not_arrived = (not self.in_target() ) or (rs and (not ir))
            have_time   = (counter < self.ik_settings.number_of_steps)
        #
            
    def run(self):
        '''
        Runs the inverse kinematic algorithm from the given starting point. 
        
        The stop criteria is the occurance of one of the following:
            1- The counter exceeds property "number_of_steps" of the class and the norm of error is more than its value in the previous iteration
            2- The solution is found according to the defined precision
        when the stop criteria is met, the configuration corresponding to the minimum pose error norm is returned as the solution

        The running time and number of iteration is returned in the "log_info" property.
        This method should only be used for "Pose Projection" or "PP" application scenario.
        
        '''

        run_time = 0.0
        if self.ik_settings.return_min_config:
            min_config = copy.deepcopy(self)
        counter = 0

        not_yet = not self.in_target()
        have_time = (counter < self.ik_settings.number_of_steps)
        err_reduced = False

        # while not_yet and (have_time or err_reduced): 
        while not_yet and have_time: 

            try:
                self.update_step()
                #self.update_rule_new(step_size = 1.0)
                counter     += 1
    
            except np.linalg.linalg.LinAlgError:
                    
                print "******** Singular Matrix ********";
                break
    
            rs = self.ik_settings.joint_limits_respected
            ir = self.joints_in_range(self.free_config(self.q))
            it = self.in_target()
    
            not_yet     = (not it) or ((not ir) and rs)
            have_time   = (counter < self.ik_settings.number_of_steps)
            '''
            print
            print "Current config    : ", self.q
            print "Current config v  : ", self.qstar
            
            print "Current Error     : ", self.pose_error_norm()
            print "Current Ofun      : ", self.objective_function_value()

            ra = self.task_point[0].position(self.H)
            rd = self.task_point[0].rd

            print "Position Error    : ", self.task_point[0].error.value(ra, rd)
            print "Position in target: ", self.task_point[0].error.in_target(ra, rd)

            ra = self.task_frame[0].orientation(self.H)
            rd = self.task_frame[0].rd

            print "Rotation Error    : ", self.task_frame[0].error.value(ra, rd)
            print "Rotation in target: ", self.task_frame[0].error.in_target(ra, rd)
            '''
            
            if self.ik_settings.return_min_config:
                '''    
                print "Min Error Achieved: ", min_config.pose_error_norm()
                print "Min Error Ofun    : ", min_config.objective_function_value()
                '''
                er          = self.pose_error_norm() < min_config.pose_error_norm()
                err_reduced = er and (ir or (not rs))
                if err_reduced:
                    min_config  = copy.deepcopy(self)
        
        if not_yet and self.ik_settings.return_min_config:
            log = copy.copy(self.log_info)
            self.copy_from(min_config)
            self.log_info = copy.copy(log)
    
    def inverse_update(self):
        '''
        '''

        func_name = '.inverse_update()'

        self.log_info               = (0.000000, 0)
                
        if self.ik_settings.include_current_config:
            self.initial_config         = copy.copy(self.q)
            if self.in_target():
                return True
                
            if self.ik_settings.run_mode == 'binary_run':
                self.run_binary(True)
            elif self.ik_settings.run_mode == 'normal_run':
                if self.ik_settings.algorithm in ["JT","JPI","DLS(CDF)", "JI"]:
                    self.run()
                elif self.ik_settings.algorithm == "DLS(ADF)":
                    self.run_dls_vdf()
                else:
                    assert False, "Error from: " + __name__ + func_name + ": " + self.ik_settings.algorithm + " is not a valid value for algorithm" 

        self.start_node = 0
        p = len(self.initial_config_list) 

        while (not self.in_target()) and (self.start_node < p):

            assert self.set_config(self.initial_config_list[self.start_node])
            
            self.initial_config  = self.free_config(self.q) # Just for keeping the information of the initial status (Initial status will be shown later in the results log file)

            # self.forward_update()
                
            if self.ik_settings.run_mode == 'binary_run':
                self.run_binary(True)
            elif self.ik_settings.run_mode == 'normal_run':
                if self.ik_settings.algorithm in ["JT","JPI","DLS(CDF)", "JI"]:
                    self.run()
                elif self.ik_settings.algorithm == "DLS(ADF)":
                    self.run_dls_vdf()
                else:
                    assert False, "Error from: " + __name__ + func_name + ": " + self.ik_settings.algorithm + " is not a valid value for algorithm" 
 
            self.start_node += 1

    def move_towards_target(self, max_speed, ttr, show = False):
        q0        = self.free_config(self.q)
        err       = self.pose_error_norm()
        
        jdir      = self.ik_direction()

        if np.linalg.norm(jdir) > 0.00001:
            (el, eh)  = self.joint_stepsize_interval(direction = jdir, max_speed = max_speed, delta_t = ttr) 
            # assert el < 0.0
            max_eta = self.optimum_stepsize(jdir)
            if eh > max_eta: 
                eh = max_eta  # max_eta = 1.0 when algorithm is JPI

            # self.grow_qstar(eh*jdir)
            assert self.set_qstar(self.qstar + eh*jdir)
            self.clear_pose()
            self.T = None
            self.H = None
            self.ajac.clear()
        
            if self.pose_error_norm() < err:
                return True    
            else:
                print "Error is not reduced"
                
            '''    
            q = q0 + eh*jdir
            if self.set_config(q):
                if self.error_norm < err:
                    print "********************Hurrrraaaaaa"
                    return True
                else:
                    print "Error is not reduced"
            else:
                print "set config failed"
            '''    
        else:
            return True
            
        if show:
            print "Moving towards target Failed:"
            print
            print "direction = ", jdir
            print "--------------------"
            print "Old q: ", q0
            print "New q: ", self.q
            print "--------------------"
            print "Old Error: ", err
            print "New Error: ", self.pose_metric()
            print "--------------------"

        assert self.set_config(q0)
        return False

    def project_to_js(self,  pos_traj, ori_traj = None, phi_start = 0.0, phi_end = None, phi_dot = 0.5, max_speed = 1.0, relative = True):
        '''
        projects the given taskspace pose trajectory into the jointspace using numeric inverse kinematics.
        The phase starts from phi_start and added by delta_phi in each step.
        at any time, if a solution is not found, the process stops
        '''
        ts        = time.time()
        t0        = 0.0

        keep_q = self.free_config(self.q)

        if phi_end == None:
            phi_end = pos_traj.phi_end

        if ori_traj == None:
            H  = self.transfer_matrices()
            ra = self.task_frame[0].orientation(H)
            ori_traj = trajlib.Orientation_Path()
            ori_traj.add_point(0.0, ra)
            ori_traj.add_point(pos_traj.phi_end, ra)

        if phi_end > pos_traj.phi_end:
            phi_end = pos_traj.phi_end

        jt          = trajlib.Trajectory_Polynomial(dimension = self.config_settings.DOF)
        jt.capacity = 2

        jt.add_point(phi = 0.0, pos = self.free_config(self.q), vel = np.zeros(self.config_settings.DOF))

        phi   = phi_start
        pos_traj.set_phi(phi)
        ori_traj.set_phi(phi)
        if relative:
            H     = self.transfer_matrices()
            p0    = self.task_point[0].position(H) - pos_traj.current_position
            ra    = self.task_frame[0].orientation(H)
            R0    = np.dot(ra['matrix'], ori_traj.current_orientation['matrix'].T)  
        else:
            p0    = np.zeros(3)
            R0    = np.eye(3)  
        
        t     = time.time() - ts
        if self.ik_settings.real_time:
            dt    = t - t0
            t0    = t
        else:
            dt    = self.ik_settings.time_step
        phi      += phi_dot*dt

        stay      = True
        cnt       = 0
        while stay:
            
            cnt  += 1
            if phi > phi_end:
                phi = phi_end
            if phi == phi_end:
                stay = False
            pos_traj.set_phi(phi)
            ori_traj.set_phi(phi)
            p = p0 + pos_traj.current_position
            R = np.dot(R0, ori_traj.current_orientation['matrix'])
            self.set_target([p], [geo.Orientation_3D(R, np.zeros((3,3)))])
            if self.move_towards_target(max_speed = max_speed, ttr = dt):
                jt.add_point(phi = phi - phi_start, pos = self.free_config(self.q))
                # print cnt, "- phi = ", phi, ": Error is reduced. Point is Added. :-)", self.pose_error_norm()
                if self.ik_settings.algorithm == "DLS(ADF)":
                    self.ik_settings.damping_factor = self.ik_settings.damping_factor / 2.0
            else:
                # print cnt, "- phi = ", phi, ": Error is not reduced. Point is not Added. :-("
                print "."
                if self.ik_settings.algorithm == "DLS(ADF)":
                    self.ik_settings.damping_factor = 2*self.ik_settings.damping_factor

            t     = time.time() - ts
            if self.ik_settings.real_time:
                dt    = t - t0
                t0    = t
            else:
                dt    = self.ik_settings.time_step
            phi      += phi_dot*dt

        jt.interpolate()
        self.set_config(keep_q)

        return jt
 
    def project_to_ts(self,  js_traj, phi_start = 0.0, phi_end = None, delta_phi = 0.1):
        '''
        projects the given jointspace trajectory into the taskspace
        The phase starts from phi_start and added by delta_phi in each step.
        if at any time the joint values are not feasible, the process is stopped.
        '''
        
        if phi_end == None:
            phi_end = js_traj.phi_end

        ori_traj = trajlib.Orientation_Trajectory()
        pos_traj = trajlib.Trajectory_Polynomial()
        if phi_end > js_traj.phi_end:
            phi_end = js_traj.phi_end

        phi = phi_start
        js_traj.set_phi(phi)
        stay = True
        while stay and (self.set_config(js_traj.current_position)):
            if phi > phi_end:
                phi = phi_end
                stay = False

            H = self.transfer_matrices()

            pos_traj.add_point(phi - phi_start, self.task_point[0].position(H))
            ori_traj.add_point(phi - phi_start, self.task_frame[0].orientation(H))
            phi = phi + delta_phi
            js_traj.set_phi(phi)
            
        pos_traj.add_point(phi - phi_start, self.task_point[0].position(self.transfer_matrices()))
        ori_traj.add_point(phi - phi_start, self.task_frame[0].orientation(self.transfer_matrices()))
        return (pos_traj, ori_traj)
