'''   Header
@file:          ts_synchronizer.py
@brief:    	    Contains a class inherited from TURTLESIM in ts_kinematics.py which is connected to a real-time turtlesim in simulation
                It can be used as a starting point for any navigating robot: PR2, turtlebot, ...
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
Start date:     16 May 2014
Last Revision:  16 May 2014

Changes from ver 1.0:
'''

import ts_kinematics as tsk
import ts_listener as tsl
import ts_actuator as tsa
import numpy, math, time

import packages.nima.mathematics.general as gen
import packages.nima.mathematics.trigonometry as trig
import packages.nima.mathematics.rotation as rot
import packages.nima.mathematics.vectors_and_matrices as vecmat


def calibrate_turtlesim():
    # Calibration:
    '''
    Trying to extract upper and lower bound limits for turtlesim motion   ql, qh  (Incomplete)
    '''
    ql = numpy.zeros(3)
    qh = numpy.zeros(3)

    return((ql, qh))
    

class REAL_TURTLESIM(tsk.TURTLESIM):
    '''
    Any changes in the status and configuration of the robot is directly applied to the real robot or robot in simulation    
    '''
    def __init__(self):
        q_d = numpy.zeros(3)
        tsl.listen()  
        q_d[0] =       
        super(PyRide_PR2, self).__init__()
        super(PyRide_PR2, self).set_config(q_d)
    
    def body_synced(self):
        '''
        Returns True if robot and object body positions are identical
        '''    
        bp = pint.body_position()
        ba = pint.body_angle(in_degrees = False)
        fx = gen.equal(bp[0], self.q[8], epsilon = 0.1)
        fy = gen.equal(bp[1], self.q[9], epsilon = 0.1)
        ft = gen.equal(ba   , self.q[10], epsilon = 0.1)

        return(fx and fy and ft)

    def rarm_synced(self):
        return vecmat.equal(trig.angles_standard_range(pint.rarm_joints(in_degrees = False)), trig.angles_standard_range(self.rarm.config.q), epsilon = 0.01)

    def larm_synced(self):
        return vecmat.equal(trig.angles_standard_range(pint.larm_joints(in_degrees = False)), trig.angles_standard_range(self.larm.config.q), epsilon = 0.01)

    def sync(self):

        '''
        Synchronizes the object with the robot and verifies the forwark kinematics of both the endeffectors
        '''        
        q_r = calibrate_pr2_r_arm()
        q_l = calibrate_pr2_l_arm()
        q_t = calibrate_pr2_torso()

        q_d = numpy.concatenate((q_r[0:7], q_t[0:4], q_l[0:7]))    

        assert super(PyRide_PR2, self).set_config(q_d)
        
        laggp1 = self.larm_endeffector_position() # Left Arm Global Gripper Position from object fk
        laggp2 = pint.pos_larm_grip() # Left Arm Global Gripper Position measured from robot

        assert vecmat.equal(laggp1, laggp2, epsilon = 0.01) # verify equality with 1 cm accuracy

        raggp1 = self.rarm_endeffector_position() # Right Arm Global Gripper Position from object fk
        raggp2 = pint.pos_rarm_grip() # Right Arm Global Gripper Position measured from robot

        assert vecmat.equal(raggp1, raggp2, epsilon = 0.01) # verify equality with 1 cm accuracy

    def sync_robot(self, q_d, ttr = 1.0):

        assert gen.equal(q_d[7], self.q[7]) # At the moment, we do not have any function in PyRide for moving the prismatic(telescopic) lifting joint. This equality test will be removed when we have it.

        assert self.set_config(q_d) # Make sure the given joints are feasible

        pint.take_rarm_to(self.rarm.config.q, time_to_reach = ttr) # Move the right arm
        pint.take_larm_to(self.larm.config.q, time_to_reach = ttr) # Move the left  arm
        pint.move_robot_to(x = self.q[8], y = self.q[9], tau = self.q[10], time_to_reach = ttr)  # Move the robot body first

        time.sleep(ttr)
        counter = 1
        while ((not self.body_synced()) or (not self.rarm_synced()) or (not self.larm_synced())) and (counter<20):
            counter = counter + 1
            print "Counter = ", counter
            if pint.body_reached:
                assert self.body_synced()
            else:
                if not pint.body_failed:
                    PyPR2.cancelMoveBodyAction()
                pint.move_robot_to(x = self.q[8], y = self.q[9], tau = self.q[10], time_to_reach = ttr)  # Move the robot body first

            if pint.rarm_reached:
                assert self.rarm_synced()
            else:
                if not pint.rarm_failed:
                    PyPR2.cancelMoveArmAction(False)
                pint.take_rarm_to(self.q[0:7], time_to_reach = ttr)  # Move the robot body first

            if pint.larm_reached:
                assert self.larm_synced()
            else:
                if not pint.larm_failed:
                    PyPR2.cancelMoveArmAction(True)
                pint.take_larm_to(self.q[11:18], time_to_reach = ttr)  # Move the robot body first

            time.sleep(ttr)
            
        if (counter<20):
            assert vecmat.equal(self.q, q_d)
            print("fekr nakonam halahalaha ino bebini !")
        else:
            print("Sharmande dada! Hopefully next time :-)")

