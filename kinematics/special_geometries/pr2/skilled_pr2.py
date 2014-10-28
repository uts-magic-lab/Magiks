'''   Header
@file:          skilled_pr2.py
@brief:    	    Contains a class inherited from PyRide_PR2 in pyride_synchronizer.py which is connected to a real-time robot 
                A real PR2 or PR2 in simulation that have special skills like writing
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

@version:	    2.0
Start date:     17 September 2014
Last Revision:  29 October 2014

Attention:

All actions of this module do not have any collision avoidance feature. So be prepared to stop the robot if any collision is predicted !

'''
import numpy as np

import pyride_synchronizer as ps
import pyride_interpreter as pint


import packages.nima.robotics.kinematics.task_space.trajectory as traj
import packages.nima.robotics.kinematics.task_space.trajectory_shapes as tsh
import packages.nima.mathematics.rotation as rot
import packages.nima.mathematics.general as gen

'''
import sys
sys.path.append()
'''

class Skilled_PR2(ps.PyRide_PR2):
    def __init__(self):
        super(Skilled_PR2, self).__init__(vts = True)
        self.height = 0.1
        self.width  = 0.1
        self.depth  = 0.03    
        self.write_from_shape = True
        self.write_from_shape_complete = False
        self.writing_speed = 0.05  # 5 cm/sec

        self.larm_startpoint      = None
        self.rarm_startpoint      = None
        self.larm_endpoint_width  = None
        self.rarm_endpoint_width  = None
        self.larm_endpoint_height = None
        self.rarm_endpoint_height = None

        d = 0.5
        self.rarm_startpoint      = np.array([ d-0.05,  - 0.1,  0.0])
        self.larm_startpoint      = np.array([ d-0.05,  + 0.4,  0.0])
        self.rarm_endpoint_width  = np.array([ d-0.05,  - 0.4,  0.0])
        self.larm_endpoint_width  = np.array([ d-0.05,  + 0.1,  0.0])
        self.rarm_endpoint_height = np.array([ d-0.05,  - 0.1,  0.2])
        self.larm_endpoint_height = np.array([ d-0.05,  + 0.4,  0.2])

        self.box_width_rarm       = 0.3
        self.box_height_rarm      = 0.2
        self.box_width_larm       = 0.3
        self.box_height_larm      = 0.2

    def keep_reference(self, q = None):
        '''
        Sets the configuration given by q as the reference configuration
        If q is not given, it reads the robot arm joints and sets them as reference configuration
        '''
        if q == None:
            self.ref_config = np.copy(self.q)
        else:
            keep_config = np.copy(self.q)
            assert self.set_config(q), "Error from Skilled_PR2.set_reference(): Given joints are not feasible"
            self.ref_config = self.q
            self.q = np.copy(keep_config)

    def set_reference_arms(self, ttr = 2.5):
        if self.set_config(self.ref_config):
            qr = self.rarm.config.q
            ql = self.larm.config.q
            pint.take_rarm_to(qr, time_to_reach = ttr)
            pint.take_larm_to(ql, time_to_reach = ttr)
            self.sync_object()        
        else:
            assert False, "This should not happen"

    def calibrate_startpoint(self):
        self.sync_object()
        if self.larm_reference:
            self.larm_startpoint = np.copy(self.larm.wrist_position())
        else:
            self.rarm_startpoint = np.copy(self.rarm.wrist_position())

    def calibrate_width(self):
        self.sync_object()
        if self.larm_reference:
            self.larm_endpoint_width  = np.copy(self.larm.wrist_position())
        else:
            self.rarm_endpoint_width = np.copy(self.rarm.wrist_position())

    def calibrate_height(self):
        self.sync_object()
        if self.larm_reference:
            self.larm_endpoint_height  = np.copy(self.larm.wrist_position())
        else:
            self.rarm_endpoint_height = np.copy(self.rarm.wrist_position())

    def praying_posture(self):
        '''
        The robot takes arms to the reference posture. Reference posture is the configuration when you launch PR2 in Gazebo.
        '''
        qr = np.array([ -6.85916588e-06,   1.59104071e+00,   1.04472774e-03, -2.49505670e-01,  -3.26593628e-03,  -3.35053472e-01, -1.65636744e-03])        
        ql = np.array([ -3.57102700e-04,   1.59100714e+00,  -5.27852219e-04, -2.48833571e-01,   3.85286996e-03,  -3.36172240e-01,  1.43031683e-03])
        pint.take_rarm_to(qr, time_to_reach = 3.0)
        pint.take_larm_to(ql, time_to_reach = 3.0)
        self.sync_object()

    def writing_posture(self):
        if self.larm_startpoint == None:
            print "Left Arm is not calibrated!"
        else:
            pos = self.larm_startpoint 
            if (self.larm_endpoint_width == None) or (self.larm_endpoint_height == None):
                ori = ps.rot.point_forward_orientation 
            else:
                z   = self.larm_endpoint_height - self.larm_startpoint
                self.box_height_larm = np.linalg.norm(z)
                z   = z/self.box_height_larm
                y   = self.larm_endpoint_width - self.larm_startpoint
                if y[1] < 0:
                    y   = y/self.box_width_larm
                else:
                    y   = - y/self.box_width_larm
                self.box_width_larm = np.linalg.norm(y)
                y   = - y/self.box_width_larm
                x   = np.cross(y, z)
                x   = x/np.linalg.norm(x)
                # Correct z:
                z   = np.cross(x, y)
                ori = np.append(np.append([y],[z],axis = 0), [x], axis = 0).T
                assert gen.equal(np.linalg.det(ori), 1.0)
            n = ori[:,2]
            self.larm.set_target(pos - self.depth*n, ori)

        if self.rarm_startpoint == None:
            print "Right Arm is not calibrated!"
        else:
            pos = self.rarm_startpoint
            if (self.rarm_endpoint_width == None) or (self.rarm_endpoint_height == None):
                ori = ps.rot.point_forward_orientation 
            else:
                z   = self.rarm_endpoint_height - self.rarm_startpoint
                self.box_height_rarm = np.linalg.norm(z)
                z   = z/self.box_height_rarm
                y   = self.rarm_endpoint_width - self.rarm_startpoint
                self.box_width_rarm = np.linalg.norm(y)
                if y[1] < 0:
                    y   = - y/self.box_width_rarm
                else:
                    y   = y/self.box_width_rarm
                x   = np.cross(y, z)
                x   = x/np.linalg.norm(x)
                # Correct z:
                z   = np.cross(x, y)
                ori = np.append(np.append([y],[z],axis = 0), [x], axis = 0).T
                assert gen.equal(np.linalg.det(ori), 1.0)
            n = ori[:,2]
            self.rarm.set_target(pos-self.depth*n, ori)

        self.reach_target_larm(wait = False)
        self.reach_target_rarm(wait = False)
        pint.wait_until_finished(target_list = ['rarm', 'larm'])
        self.sync_object()
        '''
        '''
    
    def draw_shape(self, shape_trajectory):# add wait argument later
        arm = self.reference_arm()
        ot = traj.Orientation_Trajectory()
        ot.current_orientation = arm.wrist_orientation()
        jt = arm.project_to_js(shape_trajectory, ot, phi_end = shape_trajectory.phi_end, delta_phi = shape_trajectory.phi_end/100, relative = False)
        # jt.fix_points()
        jt.consistent_velocities()
        # jt.plot()
        if not gen.equal(jt.phi_end, shape_trajectory.phi_end):
            print "Warning from PyRide_PR2.draw_shape(): All the shape is not in the workspace. Part of it will be drawn !"
        
        L = sum(shape_trajectory.points_dist())
        pint.run_config_trajectory(jt, duration = L/self.writing_speed, is_left_arm = self.larm_reference)

    def switch_arm(self):
        if self.larm_reference:
            self.larm_reference = False
        else:
            self.larm_reference = True
            
    def draw_from_canvas(self, canvas_shapes, height = 0.2, width = 0.3, width_pixels = 640, height_pixels = 480, plot = False, silent=True):
        self.sync_object()
        ori_l = self.larm.wrist_orientation()
        ori_r = self.rarm.wrist_orientation()
        wr = - ori_r[:,0]
        hr =   ori_r[:,1]
        nr =   ori_r[:,2]
        wl = - ori_l[:,0]
        hl =   ori_l[:,1]
        nl =   ori_l[:,2]
        
        self.control_mode   = "fixed_base"
        self.larm_reference = False
        '''
        rarm_lower_right    = np.array([0.5, - width - 0.1, self.box_offset_z - height/2])
        rarm_lower_left     = np.array([0.5,         - 0.1, self.box_offset_z - height/2])
        larm_lower_right    = np.array([0.5,           0.1, self.box_offset_z - height/2])
        larm_lower_left     = np.array([0.5,   width + 0.1, self.box_offset_z - height/2])
        '''

        pr_list = []
        pl_list = []
        
        for cs in canvas_shapes:
            if cs.arm == "R":
                phi     = 1.0
                (x0,y0) = cs.pts[0]
                pos0    = self.rarm_startpoint + self.box_width_rarm*x0*wr/width_pixels + self.box_height_rarm*(height_pixels-y0)*hr/height_pixels
                pr      = traj.Polynomial_Trajectory(dimension = 3)
                pr.add_point(0.0, np.copy(pos0), np.zeros(3))
                for (x,y) in cs.pts[1:]:
                    pos  = self.rarm_startpoint + self.box_width_rarm*x*wr/width_pixels + self.box_height_rarm*(height_pixels-y)*hr/height_pixels
                    pr.add_point(phi, pos)
                    pr.new_segment()
                    phi += 1.0

                pr.consistent_velocities()
                pr_list.append(pr)
                if plot:
                    pr.plot3d()
            elif cs.arm == "L":     
                phi     = 1.0
                (x0,y0) = cs.pts[0]
                pos0    = self.larm_startpoint + self.box_width_larm*x0*wl/width_pixels + self.box_height_larm*(height_pixels-y0)*hl/height_pixels
                pl      = traj.Polynomial_Trajectory(dimension = 3)
                pl.add_point(0.0, np.copy(pos0), np.zeros(3))
                for (x,y) in cs.pts[1:]:
                    pos  = self.larm_startpoint + self.box_width_larm*x*wl/width_pixels + self.box_height_larm*(height_pixels-y)*hl/height_pixels
                    pl.add_point(phi, pos)
                    pl.new_segment()
                    phi += 1.0

                pl.consistent_velocities()
                pl_list.append(pl)
                if plot:
                    pr.plot3d()
            else:
                assert False
            
        
        ir   = 0
        il   = 0
        nrt  = len(pr_list)
        nlt  = len(pl_list)
        step_r = 0
        step_l = 0
        while ((ir < nrt) or (il < nlt)) and (not pint.rarm_failed) and (not pint.larm_failed):

            # initial positioning:
            if (ir < nrt) and (step_r == 0):
                if not silent:
                    print "Trajectory: ", ir, " Step 0 for Right Arm"
                pr     = pr_list[ir]
                pos    = np.copy(pr.segment[0].point[0].pos)
                pos    = pos - self.depth*nr
                self.rarm.set_target(pos, ori_r)
                assert self.rarm.inverse_update(optimize = True)
                pint.take_rarm_to(self.rarm.config.q, time_to_reach = 1.0)
                step_r += 1    

            if (il < nlt) and (step_l == 0):
                if not silent:
                    print "Trajectory: ", il, " Step 0 for Left Arm"
                pl     = pl_list[il]
                pos    = np.copy(pl.segment[0].point[0].pos)
                pos    = pos - self.depth*nl
                self.larm.set_target(pos, ori_l)
                assert self.larm.inverse_update(optimize = True)
                pint.take_larm_to(self.larm.config.q, time_to_reach = 1.0)
                step_l += 1    
            # head forward:
            if (pint.rarm_reached) and (step_r == 1):
                if not silent:
                    print "Trajectory: ", ir, " Step 1 for Right Arm"
                pos    = np.copy(pr.segment[0].point[0].pos)
                self.rarm.set_target(pos, ori_r)
                assert self.rarm.inverse_update(optimize = True)
                pint.take_rarm_to(self.rarm.config.q, time_to_reach = 0.5)
                step_r += 1    

            if (pint.larm_reached) and (step_l == 1):
                if not silent:
                    print "Trajectory: ", il, " Step 1 for Left Arm"
                pos    = np.copy(pl.segment[0].point[0].pos)
                self.larm.set_target(pos, ori_l)
                assert self.larm.inverse_update(optimize = True)
                pint.take_larm_to(self.larm.config.q, time_to_reach = 0.5)
                step_l += 1    

            # run trajectory:
            if (pint.rarm_reached) and (step_r == 2):
                if not silent:
                    print "Trajectory: ", ir, " Step 2 for Right Arm"
                self.larm_reference = False
                # self.arm_trajectory(pr, relative = False, speed = self.writing_speed, delta_phi = 0.5, wait = False)
                self.draw_shape(pr)
                step_r += 1    

            if (pint.larm_reached) and (step_l == 2):
                if not silent:
                    print "Trajectory: ", il, " Step 2 for Left Arm"
                self.larm_reference = True
                # self.arm_trajectory(pl, relative = False, speed = self.writing_speed, delta_phi = 0.5, wait = False)
                self.draw_shape(pl)
                step_l += 1    

            if (pint.rarm_reached) and (step_r == 3):
                if not silent:
                    print "Trajectory: ", ir, " Step 3 for Right Arm"
                ir    += 1
                step_r = 0

            if (pint.larm_reached) and (step_l == 3):
                if not silent:
                    print "Trajectory: ", il, " Step 3 for Left Arm"
                il    += 1
                step_l = 0

        self.sync_object()
    
    def write_A(self):
        # speed unit is: m/sec
        if self.larm_reference:
            ori = self.larm.wrist_orientation()
        else:
            ori = self.rarm.wrist_orientation()
        if self.write_from_shape_complete:
            A_complete = tsh.A(width = self.width, height = self.height, complete = True, direction = ori)
            self.arm_trajectory(A_complete, speed = self.writing_speed, delta_phi = 0.5)
        else:
            self.arm_forward(self.depth, speed = self.writing_speed/2, relative = True)
            if self.write_from_shape:
                A = tsh.A(width = self.width, height = self.height, direction = ori)
                self.arm_trajectory(A, speed = self.writing_speed, delta_phi = 0.5)
            else:
                self.arm_right_up(dx   = 0.4*self.width, dy = self.height, speed = self.writing_speed, relative = True)
                self.arm_right_down(dx = 0.4*self.width, dy = self.height, speed = self.writing_speed, relative = True)

            self.arm_back(self.depth, speed = self.writing_speed/2, relative = True)
            self.arm_left_up(0.6*self.width, 0.5*self.height, speed = self.writing_speed, relative = True)
            self.arm_forward(self.depth, speed = self.writing_speed/2, relative = True)
            self.arm_right(0.4*self.width, speed = self.writing_speed, relative = True)
            self.arm_back(self.depth, speed = self.writing_speed/2, relative = True)
            self.arm_right_down(0.3*self.width, 0.5*self.height, speed = self.writing_speed, relative = True)

    def write_B(self):
        if self.larm_reference:
            ori = self.larm.wrist_orientation()
        else:
            ori = self.rarm.wrist_orientation()
        if self.write_from_shape_complete:
            B_complete = tsh.B(width = 0.4*self.height, height = self.height, complete = True, direction = ori)
            self.arm_trajectory(B_complete, speed = self.writing_speed, delta_phi = 0.5)
        else:
            self.arm_forward(self.depth, speed = self.writing_speed/2, relative=True)
            if self.write_from_shape:
                B = tsh.B(width = 0.4*self.height, height = self.height, direction = ori)
                self.arm_trajectory(B, speed = self.writing_speed, delta_phi = 0.5)
            else:
                self.arm_up(self.height, speed = self.writing_speed, relative=True)
                self.arm_arc(center = np.array([0.0, 0.0, - 0.25*self.height]), speed = self.writing_speed)
                self.arm_arc(center = np.array([0.0, 0.0, - 0.25*self.height]), speed = self.writing_speed)
            self.arm_back(self.depth, speed = self.writing_speed/2, relative=True)
            self.arm_right(0.5*self.height, speed = self.writing_speed, relative=True)

    def write_P(self):
        if self.larm_reference:
            ori = self.larm.wrist_orientation()
        else:
            ori = self.rarm.wrist_orientation()
        if self.write_from_shape_complete:
            P_complete = tsh.P(width = 0.4*self.height, height = self.height, complete = True, direction = ori)
            self.arm_trajectory(P_complete, speed = self.writing_speed, delta_phi = 0.5)
        else:
            self.arm_forward(self.depth, speed = self.writing_speed/2, relative=True)
            if self.write_from_shape:
                P = tsh.P(width = 0.4*self.height, height = self.height, direction = ori)
                self.arm_trajectory(P, speed = self.writing_speed, delta_phi = 0.5)
            else:
                self.arm_up(self.height, speed = self.writing_speed, relative=True)
                self.arm_arc(center = np.array([0.0, 0.0, - 0.25*self.height]), speed = self.writing_speed)
            self.arm_back(self.depth, speed = self.writing_speed/2, relative=True)
            self.arm_right_down(dy=0.5*self.height, dx=0.5*self.height, speed = self.writing_speed, relative=True)

    def write_N(self):
        if self.larm_reference:
            ori = self.larm.wrist_orientation()
        else:
            ori = self.rarm.wrist_orientation()
        if self.write_from_shape_complete:
            N_complete = tsh.N(width = 0.4*self.height, height = self.height, complete = True, direction = ori)
            self.arm_trajectory(P_complete, speed = self.writing_speed, delta_phi = 0.5)
        else:
            self.arm_forward(self.depth, speed = self.writing_speed/2, relative=True)
            if self.write_from_shape:
                N = tsh.N(width = 0.4*self.height, height = self.height, direction = ori)
                self.arm_trajectory(N, speed = self.writing_speed, delta_phi = 0.5)
            else:
                assert False,"Not supported yet!"
            self.arm_back(self.depth, speed = self.writing_speed/2, relative=True)
            self.arm_right_down(dy=self.height, dx=0.1*self.height, speed = self.writing_speed, relative=True)

    def write_W(self):
        if self.larm_reference:
            ori = self.larm.wrist_orientation()
        else:
            ori = self.rarm.wrist_orientation()
        if self.write_from_shape_complete:
            W_complete = tsh.W(width = 0.4*self.height, height = self.height, complete = True, direction = ori)
            self.arm_trajectory(W_complete, speed = self.writing_speed, delta_phi = 0.5)
        else:
            self.arm_up(self.height, speed = self.writing_speed, relative=True)
            self.arm_forward(self.depth, speed = self.writing_speed/2, relative=True)
            if self.write_from_shape:
                W = tsh.W(width = 0.4*self.height, height = self.height, direction = ori)
                self.arm_trajectory(W, speed = self.writing_speed, delta_phi = 0.5)
            else:
                assert False,"Not supported yet!"
            self.arm_back(self.depth, speed = self.writing_speed/2, relative=True)
            self.arm_right_down(dy=self.height, dx=0.1*self.height, speed = self.writing_speed, relative=True)

    def write_char(self, char):
        # ori = self.endeffector_orientation()
        self.sync_object()
        if char == "A":
            self.write_A()
        elif char == "B":
            self.write_B()
        elif char == "P":
            self.write_P()

    def write_larm(self, c, height = 0.1):
        '''
        Writes the given charachter c with left arm
        Before calling this function, make sure that the pen tip is 2 cm away from the board, 
        because the arm will come forward 2cm to write the letter and will move backward again after the letter is written.
        The arm will be placed a 0.1*height after the end of the written letter (down) to be prepared to write the next letter.
        '''
        self.sync_object()
        self.larm_reference = True
        if c == "W":
            self.arm_up(height)
            self.arm_forward(0.02)
            self.arm_right_down(dx = 0.2*height, dy = height)
            self.arm_right_up(dx   = 0.2*height, dy = 0.6*height)
            self.arm_right_down(dx = 0.2*height, dy = 0.6*height)
            self.arm_right_up(dx   = 0.2*height, dy = height)
            self.arm_back(0.02)
            self.arm_right_down(0.1*height, height)
        elif c == "E":       
            self.arm_right(0.8*height)
            self.arm_forward(0.02)
            self.arm_left(0.8*height)
            self.arm_up(height)
            self.arm_right(0.8*height)
            self.arm_back(0.02)
            self.arm_left_down(0.8*height, 0.5*height)
            self.arm_forward(0.02)
            self.arm_right(0.8*height)
            self.arm_back(0.02)
            self.arm_right_down(0.1*height, 0.5*height)
        elif c == "M":
            self.arm_forward(0.02)
            self.arm_up(height)
            self.arm_right_down(dx   = 0.4*height, dy = 0.6*height)
            self.arm_right_up(dx     = 0.4*height, dy = 0.6*height)
            self.arm_down(height)
            self.arm_back(0.02)
            self.arm_right(0.1*height)
        elif c == "V":
            self.arm_up(height)
            self.arm_forward(0.02)
            self.arm_right_down(dx = 0.4*height, dy = height)
            self.arm_right_up(dx   = 0.4*height, dy = height)
            self.arm_back(0.02)
            self.arm_right_down(0.1*height, height)
        else:
            print "Error from Skilled_PR2.write_larm(): Given Charachter not known"
