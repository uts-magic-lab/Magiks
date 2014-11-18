'''   Header
@file:          pyride_synchronizer.py
@brief:    	    Contains a class inherited from PR2 in pr2_kinematics.py which is connected to a real-time robot (A real PR2 or PR2 in simulation)
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

@version:	    4.0
Start date:     9 April 2014
Last Revision:  19 November 2014

Changes from ver 3.0:
    function arm_trajectory() modified
        In ver 3, the joint trajectory was a list dictionaries containing joint positions and velocities
        in the new version, the joint trajectory is generated via function project_to_js() of each arm
        then function run_config_trajectory() in module pyride_interpreter is used to run the action
        also argument delta_phi is replaced by resolution where delta_phi = pos_traj.phi_end/resolution

        all speed arguments are replaced by property self.arm_speed that determines the speed of arm motion
        all max_speed arguments are replaced by property self.arm_max_speed that determines the maximum speed of arm joints
'''

import pr2_kinematics as pr2lib
import pyride_interpreter as pint
import PyPR2, numpy, math, time

import packages.nima.mathematics.general as gen
import packages.nima.mathematics.trigonometry as trig
import packages.nima.mathematics.rotation as rot
import packages.nima.mathematics.vectors_and_matrices as vecmat

import packages.nima.robotics.kinematics.task_space.special_trajectories as strajlib
import packages.nima.robotics.kinematics.task_space.trajectory as trajlib

import packages.nima.robotics.computer_vision.laser_scan_support as lss

r_arm_joint_names =['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
l_arm_joint_names =['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']

def calibrate_pr2_r_arm():
    # Calibration:
    '''
    Trying to compute DH parameters of the right arm according to the FK measured from the robot
    '''
    q = numpy.zeros(10)    

    # Get the current right arm joint values:
    rajd = PyPR2.getArmJointPositions(False)   # gets the Right Arm Joint Dictionary
    for i in range(7):
        q[i] = rajd[r_arm_joint_names[i]]

    q[1] = q[1] + math.pi/2

    q = trig.angles_standard_range(q)

    # determining a0    
    t = PyPR2.getRelativeTF('r_shoulder_pan_link' , 'r_shoulder_lift_link')
    p = t['position']
    q[7] = p[0]  # This is a0

    # determining d2
    t = PyPR2.getRelativeTF('r_upper_arm_roll_link' , 'r_elbow_flex_link')
    p = t['position']
    q[8] = p[0]  # This is a0

    # determining d4
    t = PyPR2.getRelativeTF('r_forearm_roll_link' , 'r_wrist_flex_link')
    p = t['position']
    q[9] = p[0]  # This is d4

    return(q)
    
def calibrate_pr2_l_arm():
    # Calibration:
    '''
    Trying to compute DH parameters of the left arm according to the FK measured from the robot
    '''
    q = numpy.zeros(10)    

    # Get the current right arm joint values:
    lajd = PyPR2.getArmJointPositions(True)   # gets the Left Arm Joint Dictionary
    for i in range(7):
        q[i] = lajd[l_arm_joint_names[i]]

    q[1] = q[1] + math.pi/2
    q = trig.angles_standard_range(q)
    # determining a0    
    t = PyPR2.getRelativeTF('l_shoulder_pan_link' , 'l_shoulder_lift_link')
    p = t['position']
    q[7] = p[0]

    # determining d2
    t = PyPR2.getRelativeTF('r_upper_arm_roll_link' , 'r_elbow_flex_link')
    p = t['position']
    q[8] = p[0]

    # determining d4
    t = PyPR2.getRelativeTF('r_forearm_roll_link' , 'r_wrist_flex_link')
    p = t['position']
    q[9] = p[0]

    return(q)

def calibrate_pr2_torso():
    q = numpy.zeros(7)

    # Determining h_ts
    p = PyPR2.getRelativeTF('base_footprint', 'torso_lift_link')['position']
    q[0] = p[2] # This is h_ts or theta[7]
    '''
    There is a 50 mm offset in x direction between footprint (Torso origin) p_BO in the paper and the base_footprint defined for PR2.
    So I will consider this offset in parameter b0
    '''
    q[6] = p[0] # This is b0 

    # Determining X_BO and Y_BO:
    p = PyPR2.getRobotPose()['position'] # This should give the global coordinates of the base_footprint
    assert gen.equal(p[2], 0.0, epsilon = 0.01) # The robot base_footprint must be on the ground (Accuracy = 1 cm)
    q[1] = p[0]        # This is X_BO (theta[8])
    q[2] = p[1]        # This is Y_BO (theta[9])

    # Determining tau:
    '''    
    o = PyPR2.getRobotPose()['orientation']
    assert gen.equal(numpy.linalg.norm(o), 1.0)
    # h = (o[3],o[0],o[1], o[2])
    R = rot.rotation_matrix(o)
    uvect_i = vecmat.as_vector([1.0, 0.0, 0.0])
    uvect_k = vecmat.as_vector([0.0, 0.0, 1.0])
    q[3] = vecmat.vectors_angle(R[:, 0], uvect_i, positive_direction = uvect_k) # This is tau (theta[10])
    '''
    q[3] = pint.body_angle(in_degrees = False)    # this is theta[10]

    # Determining l0
    p = PyPR2.getRelativeTF('torso_lift_link' , 'r_shoulder_pan_link')['position']
    assert gen.equal(p[0], 0.0)
    assert gen.equal(p[2], 0.0)
    q[4] = - p[1] # This is l0

    # Determining d7
    pr = PyPR2.getRelativeTF('r_wrist_roll_link' , 'r_gripper_r_finger_tip_link')['position']
    pl = PyPR2.getRelativeTF('r_wrist_roll_link' , 'r_gripper_l_finger_tip_link')['position']
    p = (vecmat.as_vector(pr) + vecmat.as_vector(pl))/2
    assert gen.equal(p[1], 0.0)
    assert gen.equal(p[2], 0.0)
    q[5] = p[0] # This is d7

    return(q)    


class PyRide_PR2(pr2lib.PR2):
    '''
    Any changes in the status and configuration of the robot is directly applied to the real robot or robot in simulation
    '''
    
    def __init__(self, vts = False):

        q_d = numpy.zeros(18)
        self.r_arm_joint_names = r_arm_joint_names
        self.l_arm_joint_names = l_arm_joint_names

        q_r = calibrate_pr2_r_arm()
        q_l = calibrate_pr2_l_arm()
        q_t = calibrate_pr2_torso()

        assert gen.equal(q_r[7], q_l[7])
        assert gen.equal(q_r[8], q_l[8])
        assert gen.equal(q_r[9], q_l[9])

        #pr2lib.PR2.__init__(self)
        super(PyRide_PR2, self).__init__(a0 = q_r[7], d2 = q_r[8], d4 = q_r[9], d7 = q_t[5], l0 = q_t[4], b0 = q_t[6], vts = vts)

        q_d = numpy.concatenate((q_r[0:7], q_t[0:4], q_l[0:7]))    
        super(PyRide_PR2, self).set_config(q_d)

        self.set_target(self.endeffector_position(), self.endeffector_orientation())
        self.rarm.set_target(self.rarm.wrist_position(), self.rarm.wrist_orientation())
        self.larm.set_target(self.larm.wrist_position(), self.larm.wrist_orientation())
        
        self.arm_speed     = 0.02
        self.arm_max_speed = 1.0
        self.base_laser_scan_range = range(400, 640)
        self.tilt_laser_scan_range = range(80, 300)

    def height_synced(self):
        '''
        Returns True if robot and object body heights are identical
        '''    
        p  = PyPR2.getRelativeTF('base_footprint', 'torso_lift_link')['position']
        return gen.equal(p[2] , self.q[7], epsilon = 0.01)

    def body_synced(self):
        '''
        Returns True if robot and object body positions are identical
        '''    
        bp = pint.body_position()
        ba = pint.body_angle(in_degrees = False)

        fx = gen.equal(bp[0], self.q[8], epsilon = 0.1)
        fy = gen.equal(bp[1], self.q[9], epsilon = 0.1)
        ft = gen.equal(ba   , self.q[10], epsilon = 0.02)

        return(fx and fy and ft)

    def rarm_synced(self):
        return vecmat.equal(trig.angles_standard_range(pint.rarm_joints(in_degrees = False)), trig.angles_standard_range(self.rarm.config.q), epsilon = 0.01)

    def larm_synced(self):
        return vecmat.equal(trig.angles_standard_range(pint.larm_joints(in_degrees = False)), trig.angles_standard_range(self.larm.config.q), epsilon = 0.01)

    def say(self, s):
        PyPR2.say(s)

    def synced(self, target_list = ['body', 'rarm', 'larm']):
        sn = True
        if 'body' in target_list:
            sn = sn and self.body_synced()
        if 'rarm' in target_list:
            sn = sn and self.rarm_synced()
        if 'larm' in target_list:
            sn = sn and self.larm_synced()
        return sn

    def sync_object(self):

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

        assert vecmat.equal(laggp1, laggp2, epsilon = 0.1) # verify equality with 10 cm accuracy

        raggp1 = self.rarm_endeffector_position() # Right Arm Global Gripper Position from object fk
        raggp2 = pint.pos_rarm_grip() # Right Arm Global Gripper Position measured from robot

        assert vecmat.equal(raggp1, raggp2, epsilon = 0.1) # verify equality with 10 cm accuracy

    def set_config_smooth(self, qd, ttr = 5.0, max_time = 120.0, target_list = ['body', 'rarm', 'larm']):
        '''
        sets the given configuration to the object and
        synchronizes the robot with the object. 
        If the given joints are feasible, the robot takes the given configuration.
        The function will return False if the robot can not reach the desired configuration within max_time
        '''
        if self.set_config(qd): # Make sure the given joints are feasible
            t0 = time.time()
            t  = 0.0
            while (t < max_time) and (not self.synced(target_list)):
                if ('body' in target_list) and (not self.body_synced()):
                    pint.move_robot_to(x = self.q[8], y = self.q[9], tau = self.q[10], time_to_reach = ttr, in_degrees = False)  # Move the robot body first
                else:
                    pint.body_reached = True
                if ('rarm' in target_list) and (not self.rarm_synced()):
                    pint.take_rarm_to(self.rarm.config.q, time_to_reach = ttr) # Move the right arm
                else:
                    pint.rarm_reached = True
                if ('larm' in target_list) and (not self.larm_synced()):
                    pint.take_larm_to(self.larm.config.q, time_to_reach = ttr) # Move the left arm
                else:
                    pint.larm_reached = True

                assert pint.wait_until_finished(target_list = target_list)
                t = time.time() - t0
        else: 
            print "Error from PyRide_PR2.set_config_smooth(): Given joints are not feasible"
        return self.synced(target_list)

    def sync_robot(self, ttr = 5.0, wait = True):
        '''
        '''
        if not self.body_synced():
            self.set_config_smooth(self.q, ttr = ttr)

        if not self.rarm_synced():
            pint.take_rarm_to(self.rarm.config.q, time_to_reach = ttr)
            if wait:
                pint.wait_until_finished(target_list = ['rarm'])

        if not self.larm_synced():
            pint.take_larm_to(self.larm.config.q, time_to_reach = ttr)
            if wait:
                pint.wait_until_finished(target_list = ['larm'])
        

        self.sync_object()

    def reach_target_rarm(self, phi = None, ttr = 5.0, wait = True):
        '''
        Solves the Inverse Kinematics for the right arm with given redundant parameter "phi"
        and takes the right arm endeffector to the target specified by self.rarm.xd and self.rarm.Rd
        if phi = None, then the optimum phi will be selected.
        '''
        self.rarm.set_target(self.rarm.xd, self.rarm.Rd)  # make sure the target parameters are set
        if phi == None:
            if self.rarm.inverse_update(optimize = True):
                qr = self.rarm.config.q
            else:
                print "Error from PyRidePR2.reach_target_rarm(): Could not find an IK solution for given target"
                return False
        else:
            C = self.rarm.permission_set_position()
            if phi in C:
                qr = self.rarm.IK_config(phi)
            else:
                print "Error from PyRidePR2.reach_target_rarm(): Given phi is not in the permission set"
                return False
        if qr == None:
            print "Error from PyRidePR2.reach_target_rarm(): No IK solution for the given redundant parameter phi."
            return False
        else:
            if self.rarm.config.set_config(qr):
                self.q[0:7] = qr
            else:
                print "Error from PyRidePR2.reach_target_rarm(): This should not happen! Check your code."
                return False
        
        # return self.set_config_smooth(self.q, ttr = ttr, max_time = 12.0, target_list = ['rarm'])
        self.sync_robot(ttr = ttr, wait = wait)
    
    def reference_arm(self):
        if self.larm_reference:
            return(self.larm)
        else:
            return(self.rarm)

    def reach_target_larm(self, phi = None, ttr = 5.0, wait = True):
        '''
        Solves the Inverse Kinematics for the left  arm with given redundant parameter "phi"
        and takes the right arm endeffector to the target specified by self.larm.xd and self.larm.Rd
        if phi = None, then the optimum phi will be selected.
        '''
        
        self.larm.set_target(self.larm.xd, self.larm.Rd)  # make sure the target parameters are set
        if phi == None:
            if self.larm.inverse_update(optimize = True):
                ql = self.larm.config.q
            else:
                print "Error from PyRidePR2.reach_target_larm(): Could not find an IK solution for given target"
                return False
        else:
            C = self.larm.permission_set_position()
            if phi in C:
                ql = self.larm.IK_config(phi)
            else:
                print "Error from PyRidePR2.reach_target_larm(): Given phi is not in the permission set"
                return False
        if ql == None:
            print "Error from PyRidePR2.reach_target_larm(): No IK solution for the given redundant parameter phi."
            return False
        else:
            if self.larm.config.set_config(ql):
                self.q[11:18] = ql
            else:
                print "Error from PyRidePR2.reach_target_larm(): This should not happen! Check your code."
                return False
        
        self.sync_robot(ttr = ttr, wait = wait)

    def arm_target(self, phi = None, ttr = 5.0, wait = True):
        if self.larm_reference:
            self.reach_target_larm(phi = phi, ttr = ttr, wait = wait)
        else:
            self.reach_target_rarm(phi = phi, ttr = ttr, wait = wait)

    def reach_target(self):
        '''
        # should change to inverse_update() calling the inverse_update() function of the super class    
        Solves the IK in the current control mode and takes the robot endeffector to the desired pose specified by self.xd and self.Rd
        '''
        if self.larm_reference:
            tl = ['larm']
        else:
            tl = ['rarm']

        if self.control_mode == "free_base":
            tl.append('body')

        if self.inverse_update():
            self.set_config_smooth(qd = self.q, target_list = tl)
            return True
        else:   
            self.sync_object()    
            return False

    def arm_back(self, dx = 0.1, relative = False):
        '''
        pulls the arm back as much as dx (m) maintaining the orientation
        '''    
        ttr = dx/self.arm_speed
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            n = ori[:,2]
        else:
            n = rot.i_uv

        pos = pos - dx*n

        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr)

    def arm_forward(self, dx = 0.1, relative = False):
        '''
        pulls the arm back as much as dx (m) maintaining the orientation
        '''    
        ttr = dx/self.arm_speed
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            n = ori[:,2]
        else:
            n = rot.i_uv
        pos = pos + dx*n
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr)

    def arm_down(self, dx = 0.1, relative = False):
        '''
        moves the arm downward as much as dx (m) maintaining the orientation
        '''    
        ttr = dx/self.arm_speed
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            h = ori[:,1]
        else:
            h = rot.k_uv
        pos = pos - dx*h
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr)

    def arm_up(self, dx = 0.1, relative = False):
        '''
        moves the arm upward as much as dx (m) maintaining the orientation
        '''    
        ttr = dx/self.arm_speed
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            h = ori[:,1]
        else:
            h = rot.k_uv
        pos = pos + dx*h
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr)

    def arm_right(self, dx = 0.1, relative = False):
        '''
        moves the arm to the right as much as dx (m) maintaining the orientation
        '''    
        ttr = dx/self.arm_speed
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            w = ori[:,0]
        else:
            w = rot.j_uv
        pos = pos - dx*w
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr)

    def arm_left(self, dx = 0.1, relative = False):
        '''
        moves the arm to the left as much as dx (m) maintaining the orientation
        '''    
        ttr = dx/self.arm_speed
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            w = ori[:,0]
        else:
            w = rot.j_uv
        pos = pos + dx*w
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr)

    def arm_left_down(self, dx = 0.1, dy = 0.1, relative = False):
        '''
        moves the arm to the left as much as dx (m) maintaining the orientation
        '''    
        ttr = math.sqrt(dx*dx + dy*dy)/self.arm_speed
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            w = ori[:,0]
            h = ori[:,1]
        else:
            w = rot.j_uv
            h = rot.k_uv
        pos   = pos + dx*w - dy*h
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr)

    def arm_left_up(self, dx = 0.1, dy = 0.1, wait = True, relative = False):
        '''
        moves the arm to the left as much as dx (m) maintaining the orientation
        '''    
        ttr = math.sqrt(dx*dx + dy*dy)/self.arm_speed
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            w = ori[:,0]
            h = ori[:,1]
        else:
            w = rot.j_uv
            h = rot.k_uv
        pos   = pos + dx*w + dy*h
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr, wait = wait)

    def arm_right_up(self, dx = 0.1, dy = 0.1, wait = True, relative = False):
        '''
        moves the arm to the left as much as dx (m) maintaining the orientation
        '''    
        ttr = math.sqrt(dx*dx + dy*dy)/self.arm_speed
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            w = ori[:,0]
            h = ori[:,1]
        else:
            w = rot.j_uv
            h = rot.k_uv
        pos = pos - dx*w + dy*h
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr, wait = wait)
    
    def arm_right_down(self, dx = 0.1, dy = 0.1, wait = True, relative = False):
        '''
        moves the arm to the left as much as dx (m) maintaining the orientation
        '''    
        ttr = math.sqrt(dx*dx + dy*dy)/self.arm_speed
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        ori = arm.wrist_orientation()    
        if relative:
            w = ori[:,0]
            h = ori[:,1]
        else:
            w = rot.j_uv
            h = rot.k_uv
        pos = pos - dx*w - dy*h
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr, wait = wait)

    def arm_arc(self, center = numpy.array([0.0, -0.05, 0.0]), angle = math.pi, normal = numpy.array([1.0, 0.0, 0.0]), N = 100, wait = True):
        '''
        draws an arc around the given center starting from the current point with the given angle
        the arc is drawn in a plane specified by the given normal vector
        important: Center is given relative to the starting point and vectors center and normal must be perpendicular
        N specifies the number of points in the arc trajectory
        '''
        # assert genmath.equal(vecmat.angle(center, normal), 0.0), "Error from PyRide_PR2(): center and normal must be perpendicular"

        #tt = trajlib.Polynomial_Trajectory()
        #jt = trajlib.Polynomial_Trajectory(dimension = 7)
        d_theta = angle/N
        r       = numpy.linalg.norm(center)
        ttr     = r*d_theta/self.arm_speed

        arm  = self.reference_arm()
        p0   = arm.wrist_position()    
        ori  = arm.wrist_orientation()    
        '''
        J    = [- vecmat.normalize(center)]
        J    = numpy.append(J, [vecmat.normalize(normal)])
        Jdag = vecmat.right_pseudo_inverse(J)        
        '''    
        #tt.add_point(phi = 0.0, pos = p0)
        #jt.add_point(phi = 0.0, pos = arm.config.q)
        if self.larm_reference:
            g   = pint.gen_larm_joint_posvel_dict(arm.config.q, numpy.zeros(7), 0.0)
        else:
            g   = pint.gen_rarm_joint_posvel_dict(arm.config.q, numpy.zeros(7), 0.0)

        config_list = [g]
        
        for i in range(N):
            theta = (i+1)*d_theta
            '''
            b     = numpy.array([math.cos(theta), 0.0])
            x     = numpy.dot(Jdag, b)
            pos   = p0 + center + r*normalize(x)
            '''
            q0   = numpy.copy(arm.config.q)
            ax   = numpy.append(theta, vecmat.normalize(normal))
            R    = rot.rotation_matrix(ax, parametrization = 'angle_axis')
            pos  = p0 + center - numpy.dot(R, center)
            arm.set_target(pos, ori)
            arm.config.qm = numpy.copy(arm.config.q)
            if arm.move_towards_target(max_speed = self.arm_max_speed, ttr = d_theta):
                if i == N - 1:
                    vel = numpy.zeros(7)
                else:
                    vel = (arm.config.q - q0)/ttr

                if self.larm_reference:
                    g   = pint.gen_larm_joint_posvel_dict(arm.config.q, vel, ttr)
                else:
                    g   = pint.gen_rarm_joint_posvel_dict(arm.config.q, vel, ttr)
            else:
                print "Error from PyRide_PR2.arm_arc(): The arc is not in the workspace!"
                return False

            config_list.append(g)

        if self.larm_reference:
            pint.larm_failed  = False
            pint.larm_reached = False
            moving_limbs = ['larm']
        else:
            pint.rarm_failed  = False
            pint.rarm_reached = False
            moving_limbs = ['rarm']

        PyPR2.moveArmWithJointTrajectoryAndSpeed(config_list)
        arm.config.qm = 0.5*(arm.config.ql+arm.config.qh)
        if wait:
            pint.wait_until_finished(target_list = moving_limbs, max_time = 10*N*ttr)
        self.sync_object()

    def arm_point(self, direction = 'forward', ttr = 2.0, wait = True):
        '''
        Changes the arm orientation so that it points towards the given direction maintaining the wrist position
        '''   
        if direction == 'upward':
            ori = rot.point_upward_orientation
        elif direction == 'downward':
            ori = rot.point_downward_orientation
        elif direction == 'forward':
            ori = rot.point_forward_orientation
        elif direction == 'backward':
            ori = rot.point_backward_orientation
        elif direction == 'right':
            ori = rot.point_right_orientation
        elif direction == 'left':
            ori = rot.point_left_orientation
        else:
            print "Error from PyRide_PR2.point_gripper(): Given direction is unknown!"
            return None

        self.sync_object()
        arm = self.reference_arm()
        pos = arm.wrist_position()    
        arm.set_target(pos, ori)
        return self.arm_target(ttr = ttr, wait = wait)

    def activate_base_laser(self):
        pint.bl_cnt = 0
        PyPR2.registerBaseScanCallback( pint.on_base_laser )

    def activate_tilt_laser(self):
        pint.tl_cnt = 0
        PyPR2.registerTiltScanCallback( pint.on_tilt_laser )

    def front_line_tilt(self):
        '''
        Returns the front line of tilt scan
        '''
        if not pint.tl_active:
            self.activate_tilt_laser()

        (a, b, r) = lss.front_line(dist = pint.tl_dist, position_range = self.tilt_laser_scan_range, angle_min = -0.829031407833, angle_step = 0.00436332309619)

        return (a, b, numpy.linalg.norm(r) )

    def front_line_base(self):
        '''
        Returns the front line of tilt scan
        '''
        if not pint.bl_active:
            self.activate_base_laser()

        (a, b, r) = lss.front_line(dist = pint.bl_dist, position_range = self.base_laser_scan_range, angle_min = -2.26892805099, angle_step = 0.00436332309619)

        return (a, b, numpy.linalg.norm(r) )

    '''
    def front_plane(self):

        # Start:
        dist = copy.copy(pint.dist)

        (beta, intercept, residuals) = front_line(dist)
        stay = True
        while stay:
            outliers = rlib.which(residuals , '<' , 0.1 )
            dist     = rlib.pick(dist, - outliers)
            (beta, intercept, residuals) = front_line(dist)
    '''

    def deactivate_base_laser(self):
        PyPR2.registerBaseScanCallback( None )
        pint.bl_active = False
        
    def deactivate_tilt_laser(self):
        PyPR2.registerBaseScanCallback( None )
        pint.tl_active = False

    def arm_trajectory(self, pos_traj, ori_traj = None, resolution = 20, relative = True, wait = True):
        '''
        First, projects the given taskspace pose trajectory into the jointspace 
        using both velocity-based and position-based inverse kinematics of the arm and
        creates a joint trajectory.
        Then the joint trajectory is given to function run_config_trajectory of pyride_interpreter to run on the robot.
        '''
        L = sum(pos_traj.points_dist())  # You will need to accommodate for orientation as well

        self.sync_object()
        arm     = self.reference_arm()

        if ori_traj == None:
            ori_traj = trajlib.Orientation_Trajectory()
            ori_traj.current_orientation = arm.wrist_orientation()

        assert resolution > 4, "Error from PyRide_PR2.arm_trajectory(): Invalid Resolution"

        arm = self.reference_arm()
        jt  = arm.project_to_js(pos_traj, ori_traj, phi_start = 0.0, delta_phi = pos_traj.phi_end/resolution, max_speed = self.arm_max_speed, relative = relative)

        jt.consistent_velocities()
        '''
        jt.plot()
        jt.plot(1)
        jt.plot(4)
        
        (pt, ot) = arm.project_to_ts(jt)
        pt.plot3d()
        '''    

        pint.run_config_trajectory(jt, duration = L/self.arm_speed, dt = None, phi_dot = None, is_left_arm = self.larm_reference)

        if self.larm_reference:
            moving_limbs = ['larm']
        else:
            moving_limbs = ['rarm']

        if wait:
            pint.wait_until_finished(target_list = moving_limbs, max_time = 10*L/self.arm_speed)
        
        self.sync_object()
