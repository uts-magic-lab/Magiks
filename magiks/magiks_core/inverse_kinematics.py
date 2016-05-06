# HEADER

## @file        	inverse_kinematics.py
#  @brief           This module provides a functor class regaring the inverse kinematic calculations of a manipulator
#  @author      	Nima Ramezani Taghiabadi
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. : 04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	5.1
#
#  Last Revision:  	21 August 2015

'''
Changes from previous version:
    1- added function js_create()
    2- added settings in ik_settings:
    respect_error_reduction, respect_limits_in_trajectories
    3- property ik_settings.joint_limits_respected renamed to ik_settings.continue_until_inrange
    4- method DLS(MADF) added, property manipulability_threshold added to the ik_settings
    5- property run_log added to the object. Functions run(), run_dls_vdf(), js_project() and js_create() generate log
       The log is plottable and can be saved as a .csv file 
'''

# BODY

import math, time, sys, copy, numpy as np

import general_python as genpy

from math_tools import general_math as genmath
from math_tools.geometry import geometry as geo, trajectory as trajlib
from math_tools.algebra  import polynomials, vectors_and_matrices as vecmat
from math_tools.discrete import discrete

from magiks.jointspace import manipulator_configuration as conflib
from magiks.taskspace  import endeffector as eflib

all_algorithms = ['JI', 'JT', 'JPI', 'DLS(CDF)', 'DLS(ADF)', 'DLS(MADF)']

# \cond
# Module Dictionaries:
key_dic = {
    'JI'            : 'Jacobian Inverse',
    'JT'            : 'Jacobian Transpose',
    'JPI'           : 'Jacobian Pseudo Inverse',
    'DLS(CDF)'      : 'Damped Least Squares Method(Constant Damping Factor)',
    'DLS(ADF)'      : 'Damped Least Squares Method(Adaptive Damping Factor)',
    'DLS(MADF)'     : 'Damped Least Squares Method(Manipulability Adjusted Damping Factor)'
}

all_figures = ['Time', 'Duration', 'Phase', 'Manipulability', 'Damping Factor', 'In Range', 'Target Reached', 'Error Norm']

class Step_Log():
    def __init__(self):
        self.current_time     = 0.0
        self.time_elapsed     = 0.0
        self.phase            = 0.0
        self.configuration    = None
        self.end_pose         = None
        self.pose_error       = None
        self.error_norm       = None
        self.manipulability   = None
        self.damping_factor   = None
        self.is_in_target     = None
        self.joints_in_range  = None

    def value(self, figure):
        if figure == 'Time':
            return self.current_time
        elif figure == 'Duration':
            return self.time_elapsed
        elif figure == 'Phase':
            return self.phase
        elif figure == 'Manipulability':
            return self.manipulability
        elif figure == 'Damping Factor':
            return self.damping_factor
        elif figure == 'In Range':
            return self.joints_in_range
        elif figure == 'Target Reached':
            return self.is_in_target
        elif figure == 'Error Norm':
            return self.error_norm
        else:
            ers = figure + ' is not a valid value for argument figure'
            assert False, genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, ers)

    def csv_str(self, figures = all_figures):
        i = 0
        s = str(i)
        for fig in figures:
            s += ',' + str(self.value(fig))
        return s + '\n'
       
class Run_Log():
    def __init__(self):
        self.clear()

    def initial_config(self):
        return step[0].configuration

    def final_config(self):
        lsi = len(self.step) - 1  # last step index
        return step[lsi].configuration

    def success(self):
        lsi = len(self.step) - 1  # last step index
        return step[lsi].is_in_target
        
    def clear(self):
        self.step         = []
        self.time_elapsed = 0.0
        self.time_start   = time.time()

    def csv_str(self, figures = all_figures, header = True):
        if header:
            s = 'Row'
            for fig in figures:
                s += ',' + fig
            s += '\n'
        else:
            s = ''
        for stp in self.step:
            s += stp.csv_str(figures = figures)
        return s

    def write_csv(self, filename = 'iklog.csv', figures = all_figures, header = True):
        FILE_HANDLE = open(filename, "w")
        FILE_HANDLE.write(self.csv_str(figures = figures, header = header))

    def plot(self, axis_x = 'Time', axis_y = 'Error Norm', ltype = '-', plt = None):
        if plt == None:
            show = True
            import matplotlib.pyplot as plt
        else:
            show = False
        x = []
        y = []
        for stp in self.step:
            x.append(stp.value(axis_x))
            y.append(stp.value(axis_y))
                
        plt.xlabel(axis_x)
        plt.ylabel(axis_y)
        plt.plot(x,y, ltype)

        if show:
            plt.show()        
        

class Inverse_Kinematics_Settings():
    '''
    '''    
    # Class Sets:
    all_run_modes  = [ 'normal_run', 'binary_run' ]
    all_algorithms = [ 'JI', 'JT', 'JPI', 'DLS(CDF)', 'DLS(ADF)' , 'DLS(MADF)']


    def __init__(self, algorithm = 'JPI', run_mode = 'normal_run', num_steps = 100): 

        assert run_mode in self.__class__.all_run_modes, self.__class__.err_head + func_name + ": The given run_mode is not known!"
        assert algorithm in self.__class__.all_algorithms, self.__class__.err_head + func_name + ": The given algorithm is not known!"
        
        self.ngp_active              = True  # Nullspace Gradient Projection Active?
        self.continue_until_inrange  = True  # Continue iterations until all joints are in range?
        self.return_min_config       = True   # If True returns the configuration corresponding to minimum error achieved if False returnts the config of the last iteration
        self.respect_error_reduction = True   # If True the configuration is added to the trajectory only if the norm of pose error is reduced
        self.respect_limits          = True   # If True, position, velocity and acceleration limits will be respected in function moveto_target()
        self.respect_limits_in_trajectories = True # If True, position, velocity and acceleration limits will be respected in output trajectories
        self.generate_log            = True   # If True, the system will generate log after each run or trajectory creation/projection. The log is stored in property self.run_log
        self.df_gain                 = 2.0   # Determines the damping factor gain in DLS-ADF algorithm

        self.algorithm               = algorithm
        self.run_mode                = run_mode
        self.number_of_steps         = num_steps
        self.initial_damping_factor  = 1.0
        self.manipulability_threshold= 0.0001
        self.real_time               = False
        self.time_step               = 0.010 # (sec)
        self.max_js                  = 2.0   # (Rad/sec)
        self.max_ja                  = 100.0 # (Rad/sec^2)
        
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
# \endcond
class Inverse_Kinematics( eflib.Endeffector ):
    '''
    Inherits all properties and methods of a "Forward_Kinematics" class 
    and has some additional properties and methods for inverse kinematic calculations
    Method "update" changes the "configuration" property inherited from "Forward_Kinematics"
    
    
    # goto_target (call from outside)    -- runs the suff on the grid already, calls stepwise_run or run
    # stepwise_run()                   -- 
    # run()                            -- run() # single run() 
    # step_forward (SINGLE STEP)            -- one update_step() 
    
    '''
    
    def __init__(self, config_settings, geo_settings, end_settings, ik_settings):
        ### def __init__( self, geometry ) : # njoint):
        '''
        '''
        super(Inverse_Kinematics, self).__init__(copy.copy(config_settings), copy.copy(geo_settings), copy.copy(end_settings))

        self.ik_settings               = copy.copy(ik_settings)
        
        ## fklib.Forward_Kinematics.__init__(self, DEEPCOPY(??geometry??) ) # njoint)
        
        self.end_settings.last_link_number = geo_settings.nlink - 1
        
        self.run_log                = Run_Log()
        
        # self.initial_config       = self.free_config(self.q)
        self.initial_config         = None
        self.damping_factor         = ik_settings.initial_damping_factor
        
        self.initial_config_list    = []

    # \cond

    def settings_key():
        '''
        generate and return the text key for the settings
        '''
        
    def ik_direction(self, u = None):
        '''
        Returns the joint direction (joint update correction) expected to lead the endeffector closer to the target.
        This direction should be multiplied by a proper stepsize to ensure that the pose error norm is reduced.
        '''      
        genpy.check_valid(self.ik_settings.algorithm, all_algorithms, __name__, self.__class__.__name__, sys._getframe().f_code.co_name, 'ik_settings.algorithm')

        err = self.pose_error()
        Je  = self.error_jacobian()
        W   = self.joint_damping_weights()   

        # right pseudo inverse of error jacobian is calculated and placed in Je_dagger
        #(Je_dagger,not_singular) = mathpy.right_pseudo_inverse(Je)
        if self.ik_settings.algorithm == "JI":
            Je_dag = np.linalg.inv(Je)
        elif self.ik_settings.algorithm == "JPI":
            Je_dag = vecmat.weighted_pseudo_inverse(Je, W)
        elif self.ik_settings.algorithm == "JT":
            Je_dag = - Je.T
        elif self.ik_settings.algorithm in ["DLS(CDF)", "DLS(ADF)"]:
            # Je_dag = vecmat.right_dls_inverse(Je, self.damping_factor)
            Je_dag = vecmat.weighted_dls_inverse(Je, W, k = self.damping_factor)
        elif self.ik_settings.algorithm in ['DLS(MADF)']:
            mu = self.manipulability()
            if  mu < self.ik_settings.manipulability_threshold:
                self.damping_factor = self.ik_settings.initial_damping_factor*(1.0 - mu/self.ik_settings.manipulability_threshold)
            Je_dag = vecmat.weighted_dls_inverse(Je, W, k = self.damping_factor)
        else:
            assert False

        # Joint Correction is calculated here:
        delta_qs = - np.dot(Je_dag, err)
        # print delta_qs
    
        if u != None:
            assert len(u) == self.config_settings.DOF
            J_dag_J           = np.dot(Je_dag, Je)
            In_minus_J_dag_J  = np.eye(self.config_settings.DOF) - J_dag_J
            dq                = np.dot(In_minus_J_dag_J, u)
            # print "Self motion: ", dq, self.qvr
            delta_qs         += dq
        
        return delta_qs       

    def optimum_stepsize(self, direction):
        if self.ik_settings.algorithm in ['JPI','JI']:
            k = 1.0
        elif self.ik_settings.algorithm in ['JT', 'DLS(CDF)', 'DLS(ADF)', 'DLS(MADF)']:
            J_delta_q = np.dot(self.error_jacobian(), direction)        
            k = - np.sum(J_delta_q*J_delta_q)/np.sum(J_delta_q*self.pose_error())
        else:
            assert False, genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, self.ik_settings.algorithm + " is an unknown value for algorithm")

        # assert k >= 0.0
        return k        

    def update_step(self):
        '''
        Implements an update rule for the joint configuration. This function performs one iteration of the algorithm.
        '''
        '''
        print 'US Start: **************************'
        print 'US: qqs', np.sum(self.q), np.sum(self.qvr)
        print 'US: per', np.sum(self.pose_error())
        print 'US: ejt', np.sum(self.task_point[0].error_jacobian()), np.sum(self.task_frame[0].error_jacobian())
        print 'US: EJ ', np.sum(self.error_jacobian()[0,:])
        '''
           
        ts = time.time()
        if self.ik_settings.ngp_active:
            u = self.objective_function_gradient(k = 1.0)
            # print "u = ", u
        else:
            u = None    

        jdir     = self.ik_direction(u)
        k        = self.optimum_stepsize(jdir)
        delta_qs = k*jdir
       
        assert self.set_config_virtual(self.qvr + delta_qs)

        if self.ik_settings.generate_log:
            telap    = time.time() - ts
            step_log = self.report_status()
            step_log.time_elapsed = telap
            self.run_log.step.append(step_log)

    def report_status(self):
        sl = Step_Log()
        sl.current_time     = time.time() - self.run_log.time_start
        sl.configuration    = np.copy(self.free_config(self.q))
        sl.end_pose         = copy.copy(self.pose())
        sl.pose_error       = copy.copy(self.pose_error())
        sl.error_norm       = copy.copy(self.pose_error_norm())
        sl.manipulability   = self.manipulability()
        sl.damping_factor   = self.damping_factor
        sl.is_in_target     = self.in_target()
        sl.joints_in_range  = self.joints_in_range(sl.configuration)
        return sl

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
        self.qvr             = copy.copy(ik.qvr)
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
        self.run_log           = copy.deepcopy(ik.run_log)
    # \endcond    
    def js_create(self, phi_end = None, traj_capacity = 2, traj_type = 'regular'):
        '''
        '''
        self.damping_factor = self.ik_settings.initial_damping_factor
        if self.ik_settings.generate_log:
            self.run_log.clear()
        keep_q = np.copy(self.q)
        H      = self.transfer_matrices()

        if phi_end == None:
            phi_end = self.ik_settings.number_of_steps*self.ik_settings.time_step

        if traj_type == 'regular':
            jt = trajlib.Trajectory(dimension = self.config_settings.DOF, capacity = traj_capacity)
        elif traj_type == 'polynomial':
            jt = trajlib.Trajectory_Polynomial(dimension = self.config_settings.DOF, capacity = traj_capacity)
        else:
            assert False, "\n Unknown Trajectory Type \n"

        jt.plot_settings.axis_label = self.config_settings.joint_label
        if self.ik_settings.respect_limits_in_trajectories:
            jt.vel_max  = self.ik_settings.max_js
            jt.acc_max  = self.ik_settings.max_ja
            jt.pos_max  = self.config_settings.qh
            jt.pos_min  = self.config_settings.ql

        phi   = 0.0

        jt.add_position(0.0, pos = np.copy(self.q))
        
        stay      = not self.in_target()

        while stay:
            stay = not self.in_target()
            phi = phi + self.ik_settings.time_step

            if (phi > phi_end) or genmath.equal(phi, phi_end, epsilon = 0.1*self.ik_settings.time_step):
                phi = phi_end
                stay = False

            err_reduced = self.moveto_target()

            if err_reduced or (not self.ik_settings.respect_error_reduction):
                jt.add_position(phi = phi, pos = np.copy(self.q))
                    

        if traj_type == 'polynomial':
            jt.interpolate()

        self.set_config(keep_q)
        if self.ik_settings.generate_log:
            self.run_log.time_elapsed = time.time() - self.run_log.time_start

        return jt

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

        while not_arrived and have_time: 
            previous_pose_error_norm = self.pose_error_norm()
            previous_configuration   = self.free_config(self.q)

            try:
                self.update_step()
                counter     += 1
    
            except np.linalg.linalg.LinAlgError:
                    
                print "******** Singular Matrix ********";
                break
    
            err_reduced = (self.pose_error_norm() < previous_pose_error_norm)

            if err_reduced or self.in_target():
                self.damping_factor /= self.ik_settings.df_gain
                # self.damping_factor /= self.ik_settings.df_gain
            else:
                self.damping_factor *= self.ik_settings.df_gain

                if self.damping_factor < 0.001:
                    self.set_config(previous_configuration)
                else:
                    self.damping_factor /= self.ik_settings.df_gain

            rs = self.ik_settings.continue_until_inrange
            ir = self.joints_in_range(self.free_config(self.q))

            not_arrived = (not self.in_target() ) or (rs and (not ir))
            have_time   = (counter < self.ik_settings.number_of_steps)
        
        if self.ik_settings.generate_log:
            self.run_log.time_elapsed = time.time() - self.run_log.time_start
    # \cond


    def run(self):
        '''
        Runs the inverse kinematic algorithm from the given starting point. 
        
        The stop criteria is the occurance of one of the following:
            1- The counter exceeds property "number_of_steps" of the class and the norm of error is more than its value in the previous iteration
            2- The solution is found according to the defined precision
        when the stop criteria is met, the configuration corresponding to the minimum pose error norm is returned as the solution

        The running time and number of iteration is returned in the "pps_log" property.
        This method should only be used for "Pose Projection" or "PP" application scenario.
        
        '''
        self.damping_factor = self.ik_settings.initial_damping_factor

        if self.ik_settings.return_min_config:
            min_config = self.free_config(self.q)
            min_error  = self.pose_error_norm()

        counter = 0

        not_yet = not self.in_target()
        have_time = (counter < self.ik_settings.number_of_steps)
        err_reduced = False

        # while not_yet and (have_time or err_reduced): 
        while not_yet and have_time: 

            try:
                self.update_step()
                counter     += 1
    
            except np.linalg.linalg.LinAlgError:
                    
                print "******** Singular Matrix ********";
                break
    
            rs = self.ik_settings.continue_until_inrange
            ir = self.joints_in_range(self.free_config(self.q))
            it = self.in_target()
    
            not_yet     = (not it) or ((not ir) and rs)
            have_time   = (counter < self.ik_settings.number_of_steps)
            '''
            print
            print "Current config    : ", self.q
            print "Current config v  : ", self.qvr
            
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
                er          = self.pose_error_norm() < min_error
                err_reduced = er and (ir or (not rs))
                if err_reduced:
                    min_config = self.free_config(self.q)
                    min_error  = self.pose_error_norm()
        
        if not_yet and self.ik_settings.return_min_config:
            self.set_config(min_config)
        
        if self.ik_settings.generate_log:
            self.run_log.time_elapsed = time.time() - self.run_log.time_start

    def goto_target(self):
        '''
        '''
        if self.ik_settings.generate_log:
            self.run_log.clear()
                
        if self.ik_settings.include_current_config:
            self.initial_config         = self.free_config(self.q)
            if self.in_target():
                return True
                
            if self.ik_settings.run_mode == 'binary_run':
                self.run_binary(True)
            elif self.ik_settings.run_mode == 'normal_run':
                if self.ik_settings.algorithm in ["JT","JPI","DLS(CDF)", "JI", "DLS(MADF)"]:
                    self.run()
                elif self.ik_settings.algorithm == "DLS(ADF)":
                    self.run_dls_vdf()
                else:
                    assert False, genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, self.ik_settings.algorithm + " is not a valid value for algorithm")

        self.start_node = 0
        p = len(self.initial_config_list) 

        while (not self.in_target()) and (self.start_node < p):

            assert self.set_config(self.initial_config_list[self.start_node])
            
            self.initial_config  = self.free_config(self.q) # Just for keeping the information of the initial status (Initial status will be shown later in the results log file)

            # self.forward_update()
                
            if self.ik_settings.run_mode == 'binary_run':
                self.run_binary(True)
            elif self.ik_settings.run_mode == 'normal_run':
                if self.ik_settings.algorithm in ["JT","JPI","DLS(CDF)", "JI", "DLS(MADF)"]:
                    self.run()
                elif self.ik_settings.algorithm == "DLS(ADF)":
                    self.run_dls_vdf()
                else:
                    assert False, genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, self.ik_settings.algorithm + ' is not a valid value for algorithm')
 
            self.start_node += 1

        return self.in_target()
    # \endcond

    def moveto_target(self, show = False):
        ts        = time.time()
        q0        = self.free_config(self.q)
        err       = self.pose_error_norm()

        if self.ik_settings.ngp_active:
            u = self.objective_function_gradient(k = 1.0)
            # print "u = ", u
        else:
            u = None    

        jdir      = self.ik_direction(u)

        elt = time.time() - ts  # elt: elapsed time
        if np.linalg.norm(jdir) > 0.00001:
            max_eta = self.optimum_stepsize(jdir)
            if self.ik_settings.respect_limits:
                (el, eh)  = self.joint_stepsize_interval(direction = jdir, max_speed = self.ik_settings.max_js, delta_t = self.ik_settings.time_step) 
                # assert el < 0.0
                if eh > max_eta: 
                    eh = max_eta  # max_eta = 1.0 when algorithm is JPI
            else:
                eh = max_eta  # max_eta = 1.0 when algorithm is JPI

            self.set_config_virtual(self.qvr + eh*jdir)

            if self.pose_error_norm() < err:
                self.damping_factor = self.damping_factor/self.ik_settings.df_gain
                if self.ik_settings.generate_log:
                    step_log = self.report_status()
                    step_log.time_elapsed = elt
                    self.run_log.step.append(step_log)
                return True    
            else:
                self.damping_factor = self.damping_factor*self.ik_settings.df_gain
                if self.ik_settings.generate_log:
                    step_log = self.report_status()
                    step_log.time_elapsed = elt
                    self.run_log.step.append(step_log)
                if self.ik_settings.respect_error_reduction:
                    assert self.set_config(q0)
                return False
        else:
            if self.ik_settings.generate_log:
                step_log = self.report_status()
                step_log.time_elapsed = elt
                self.run_log.step.append(step_log)
            return True
    # \cond
    def js_project(self,  pos_traj, ori_traj = None, phi_start = 0.0, phi_end = None, relative = True, traj_capacity = 2, traj_type = 'regular'):
        '''
        projects the given taskspace pose trajectory into the jointspace using the numeric inverse kinematics.
        The phase starts from phi_start and added by self.ik_settings.time_step in each step.
        at any time, if a solution is not found, the target is ignored and no configuration is added to the trajectory
        '''
        self.damping_factor = self.ik_settings.initial_damping_factor
        keep_q = np.copy(self.q)
        H      = self.transfer_matrices()

        if ori_traj == None:
            ori_traj = trajlib.Orientation_Path()
            ori_traj.add_point(0.0, self.task_frame[0].orientation(H))
            ori_traj.add_point(pos_traj.phi_end, self.task_frame[0].orientation(H))

        if phi_end == None:
            phi_end = min(pos_traj.phi_end, ori_traj.phi_end)

        if (phi_end > pos_traj.phi_end) or (phi_end > ori_traj.phi_end):
            phi_end = min(pos_traj.phi_end, ori_traj.phi_end)

        if traj_type == 'regular':
            jt = trajlib.Trajectory(dimension = self.config_settings.DOF, capacity = traj_capacity)
        elif traj_type == 'polynomial':
            jt = trajlib.Trajectory_Polynomial(dimension = self.config_settings.DOF, capacity = traj_capacity)
        else:
            assert False, "\n Unknown Trajectory Type \n"

        if self.ik_settings.respect_limits_in_trajectories:
            jt.vel_max  = self.ik_settings.max_js
            jt.acc_max  = self.ik_settings.max_ja
            jt.pos_max  = self.config_settings.qh
            jt.pos_min  = self.config_settings.ql

        phi   = phi_start
        pos_traj.set_phi(phi)
        ori_traj.set_phi(phi)
        if relative:
            p0    = self.task_point[0].position(H) - pos_traj.current_position
            self.set_target([self.task_point[0].position(H)], [self.task_frame[0].orientation(H)])
        else:
            p0    = np.zeros(3)
            self.set_target([pos_traj.current_position], [ori_traj.current_orientation])
            self.goto_target()

        jt.add_position(0.0, pos = np.copy(self.q))

        if self.ik_settings.generate_log:
            self.run_log.clear()
            self.run_log.step.append(self.report_status())
        
        stay      = True

        while stay:
            phi = phi + self.ik_settings.time_step

            if (phi > phi_end) or genmath.equal(phi, phi_end, epsilon = 0.1*self.ik_settings.time_step):
                phi = phi_end
                stay = False

            pos_traj.set_phi(phi)
            ori_traj.set_phi(phi)
            p = p0 + pos_traj.current_position
            self.set_target([p], [ori_traj.current_orientation])

            err_reduced = self.moveto_target()
            if err_reduced or (not self.ik_settings.respect_error_reduction):
                jt.add_position(phi = phi - phi_start, pos = np.copy(self.q))

        if traj_type == 'polynomial':
            jt.interpolate()

        self.set_config(keep_q)

        if self.ik_settings.generate_log:
            self.run_log.time_elapsed = time.time() - self.run_log.time_start

        return jt

    def ts_project(self,  js_traj, phi_start = 0.0, phi_end = None):
        '''
        projects the given jointspace trajectory into the taskspace
        The phase starts from phi_start and added by delta_phi in each step.
        if at any time the joint values are not feasible, the process is stopped.
        '''
        
        if phi_end == None:
            phi_end = js_traj.phi_end

        pt = trajlib.Trajectory_Polynomial(dimension = 3*len(self.task_point) + 9*len(self.task_frame))
        if phi_end > js_traj.phi_end:
            phi_end = js_traj.phi_end

        phi = phi_start
        stay = True
        while stay:
            if (phi > phi_end) or genmath.equal(phi, phi_end, epsilon = 0.1*self.ik_settings.time_step):
                phi = phi_end
                stay = False
            js_traj.set_phi(phi)
            if self.set_config(js_traj.current_position):
                pt.add_point(phi - phi_start, self.pose())
            phi = phi + self.ik_settings.time_step
        return pt

    """
    def ts_project(self,  js_traj, phi_start = 0.0, phi_end = None):
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
            phi = phi + self.ik_settings.time_step
            js_traj.set_phi(phi)
            
        pos_traj.add_point(phi - phi_start, self.task_point[0].position(self.transfer_matrices()))
        ori_traj.add_point(phi - phi_start, self.task_frame[0].orientation(self.transfer_matrices()))
        return (pos_traj, ori_traj)
    """
    # \endcond
