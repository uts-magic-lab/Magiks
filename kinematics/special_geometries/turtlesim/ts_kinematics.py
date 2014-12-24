'''   Header
@file:          ts_kinematics.py
@brief:    	    Contains specific functions that define some kinematic parameters for turtlesim
                This is a module for testing a very simple robot on ROS and can be used as a starting point for development of more complecated robots

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
                Email(3): nimaramezani@yahoo.com
                Email(4): ramezanitn@alum.sharif.edu
@version:	    1.0
Last Revision:  15 May 2014

Changes from version 2.0:
                - dual arm mode added

'''
import packages.nima.mathematics.general as gen
import packages.nima.mathematics.rotation as rotlib
import packages.nima.mathematics.vectors_and_matrices as vecmat

import numpy, math

drc        = math.pi/180.00
default_ql = numpy.array([-10.0, -10.0 , -180.0*drc])
default_qh = numpy.array([ 10.0,  10.0,   180.0*drc])

class TURTLE(object):
    '''
    turtle has 3 degrees of freedom. x,y and theta. This simple Robot navigates on the floor
    '''

    def __init__(self, ql = default_ql, qh = default_qh):
        '''
        ql and qh define the lower and higher bounds of the joints
        '''    
        assert (len(ql) == 3) and (len(qh) == 3) 
        self.ql = ql
        self.qh = qh
        self.qm = (qh + ql)/2
        self.q  = (qh + ql)/2


        # sets all angles to midrange by default
        self.set_config(self.qm)

    def joint_in_range(self,i, qi):
        '''
        returns True if the given joint parameter qi is in feasible range for the i-th joint (within the specified joint limits for that joint)
        '''

        if abs(qi - self.ql[i]) < gen.epsilon:
            qi = self.ql[i]
        if abs(qi - self.qh[i]) < gen.epsilon:
            qi = self.qh[i]

        return ((qi <= self.qh[i]) and (qi >= self.ql[i]))

    def all_joints_in_range(self, qd):
        '''
        If The given joints "qd" are out of the range specified by properties: ql and qh, returns False, otherwise returns True  
        '''
        flag = True
        for i in range(0, 3):
            flag = flag and self.joint_in_range(i, qd[i])
        return flag

    def set_config(self, qd):
        '''
        sets the configuration to "qd"
        '''    
        if not len(qd) == 3: 
            print "set_config error: Number of input elements must be 3"
            return False

        if self.all_joints_in_range(qd):
            self.q[0:2]   = qd[0:2]
            self.q[2]     = trig.angles_standard_range(qd[2])

            self.C = math.cos(self.q[2])
            self.S = math.sin(self.q[2])

            self.R = numpy.array([[  self.C, self.S], 
                                  [- self.S, self.C]]) 

            self.position_updated    = False
            self.orientation_updated = False
            self.in_target_updated   = False

            return True
        else:
            print "set_config error: Given joints are not in their feasible range"
            return False

    def set_target(self, target_position, target_orientation):
        '''
        sets the endeffector target to the given position and orientation
        variables self.xd and self.Rd should not be manipulated by the user. Always use this function
        '''    
        self.xd = target_position
        self.Rd = target_orientation

    def all_IK_solutions(self, dy = 0.0):    
        '''
        Returns the value of theta and change in x and y to reach the target position
        the problem is redundant so you can select dy, the default value is 0.
        '''
        solution_set = []
        # Calculate the relative position:
        DP = self.pd - self.current_position()

        ss = trig.solve_equation(code = 1, parameters = (DP[1], DP[0], - dy))
        
        for theta in ss:
            s = math.sin(theta)
            c = math.cos(theta)    
            if gen.equal(c, 0.0, epsilon = 0.01):
                dx = (-DP[1] + c*dy)/s
            else:
                dx = ( DP[0] - s*dy)/c
            
            solution = (dx, dy, theta)
            solution_set.append(solution) # append the solution to the solution set

        return solution_set

    def current_position(self):        
        '''
        Returns the cartesian coordiantes of the turtle.
        '''    
        if not self.position_updated:

        return numpy.array([self.q[0], self.q[1]])
               

