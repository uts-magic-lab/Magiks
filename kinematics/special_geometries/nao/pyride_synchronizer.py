'''   Header
@file:          pyride_synchronizer.py
@brief:    	    Contains a class inherited from NAO in nao_dynamics.py which is connected to a real-time robot (A real NAO or NAO in simulation)
@author:        Nima Ramezani Taghiabadi
                PhD Researcher
                Faculty of Engineering and Information Technology
                University of Technology Sydney (UTS)
                Broadway, Ultimo, NSW 2007, Australia
                Room No.: CB10.03.512
                Phone:    02 9514 4621
                Mobile:   04 5027 4611
                Email(1): Nima.RamezaniTaghiabadi@uts.edu.au 
                Email(2): Nima.RamezaniTaghiabadi@student.uts.edu.au 
                Email(3): N.RamezaniTaghiabadi@uws.edu.au 
                Email(4): nima.ramezani@gmail.com
                Email(5): nima_ramezani@yahoo.com
                Email(6): ramezanitn@alum.sharif.edu

@version:	    1.0
Start date:     20 June 2014
Last Revision:  26 June 2014

Changes from ver 1.0:
'''

import nao_dynamics as nd
import pyride_interpreter as pint
import numpy as np
import time

import packages.nima.robotics.kinematics.task_space.trajectory as tj

class PyRide_NAO(nd.NAO):
    '''
    Any changes in the status and configuration of the robot is directly applied to the real robot or robot in simulation
    '''
    
    def __init__(self, mass = nd.dflt_mass, dims = nd.dflt_dims):
        time.sleep(0.2)
        super(PyRide_NAO, self).__init__(mass = mass, dims = dims, q0 = pint.q)

    def set_config(self, qd):
        super(PyRide_NAO, self).set_config(qd)
        pint.take_robot_to(self.q)
        
    def pos_zmp_wrt_lfoot(self):
        return super(PyRide_NAO, self).pos_zmp_wrt_lfoot(a = - pint.a)

    def pos_zmp_wrt_lfoot_jacobian(self):
        return super(PyRide_NAO, self).pos_zmp_wrt_lfoot_jacobian(a = - pint.a)

    def joint_speed(self, k = 1.0):
        return super(PyRide_NAO, self).joint_speed(k = k)
        self.set_config(pint.q)

    def set_config_smooth(self, qd, time_to_reach = 1.0):
        j_traj = tj.Polynomial_Trajectory(dimension = 21)
        j_traj.interpolate(phi = [0.0, time_to_reach], positions = [pint.q, qd], velocities = [np.zeros(21), np.zeros(21)])
        t0 = time.time()
        t  = 0.0
        while t < time_to_reach:
            j_traj.set_phi(t)
            self.set_config(j_traj.current_position)
            t = time.time() - t0
        
    def reach_target_smooth(self, time_to_reach = 1.0):
        nao = self.self_copy()
        nao.reach_target()
        self.set_config_smooth(qd = nao.q, time_to_reach = time_to_reach)
        
