## @file        	skilled_pr2.py
#  @brief     		Contains a class inherited from PyRide_PR2 in pyride_synchronizer.py which is connected to a real-time robot,
#                   a real PR2 or PR2 in simulation that have special skills like drawing
#  @author      	Nima Ramezani Taghiabadi 
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. :   04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	3.0
# 
#  Start date:      17 September 2014
#  Last Revision:  	06 January 2015

import numpy as np
import time
import pyride_synchronizer
import pyride_interpreter as pint

from math_tools import general_math as gen
from math_tools.geometry import trajectory as traj, rotation as rot

'''
import sys
sys.path.append()
'''

'''
writing posture:
array([ -1.26299826e-01,   1.77046412e+00,  -1.02862191e+00,
        -1.36864905e+00,  -2.31195189e+00,  -1.29253137e+00,
         1.71195615e-01,   8.02176174e-01,  -2.12167293e-03,
         2.32811863e-04,   7.05358701e-03,   4.01010384e-01,
         2.44565260e+00,   7.10476515e-01,  -1.30808585e+00,
         1.15357810e+00,  -4.49485156e-01,  -2.46943329e+00])
'''
class Skilled_PR2(pyride_synchronizer.PyRide_PR2):
    def __init__(self, run_magiks = False):
        super(Skilled_PR2, self).__init__(run_magiks = run_magiks)
        self.height = 0.05
        self.width  = 0.05
        self.depth  = 0.02    
        self.write_from_shape = True
        self.write_from_shape_complete = False
        self.timeout = 20.0
        self.arm_speed = 0.05

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

        self.board_offset = np.array([0.1, 0.2, 0.05])

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

    def catch_the_board(self):
        # self.switch_arm()
        qr = np.array([ 0.08842831,  2.34773988,  0.07718497, -1.32087472, -0.71994221, -1.02121596, -1.56155048])
        self.rarm.set_config(qr)
        self.sync_robot()
        pint.set_rg(2)
        self.say("Please,  put the bord in my right hand")
        time.sleep(10)
        pint.set_rg(0)
        self.say("Thank you")

    def catch_the_pen(self):
        # self.switch_arm()
        pint.set_lg(2)
        self.say("Please,  put the pen in my left hand")
        time.sleep(5)
        pint.set_lg(1)
        self.say("Thank you")

    def prepare_to_write(self):
        self.say("Let's draw something. Show me a board")
        self.sync_object()
        pint.activate_tilt_laser()
        while pint.tl_dist == None:
            time.sleep(0.01)

        N = len(pint.tl_dist)/2
        d = pint.tl_dist[N]
        while d > 1.0:
            d = pint.tl_dist[N]
        
        self.catch_the_board()
        time.sleep(3)
        self.catch_the_pen()
        time.sleep(5)
        # self.get_ready(False)
        pint.deactivate_tilt_laser()

    def show_the_people(self, return_back = True):
        ql = np.array([ 1.1742804 ,  2.04188103,  1.35847037, -1.99603627, -1.22890376, -1.16233047,  0.80086808])
        self.larm.set_config(ql)
        self.sync_robot() 
        qr = np.array([-0.37128888,  2.08050452, -1.08169936, -1.27411378,  0.92031194, -1.3185062 ,  1.23221083])
        self.rarm.set_config(qr)
        self.sync_robot()   

        self.say("How was it?") 
        if return_back:
            time.sleep(5)
            qr = np.array([ 0.08842831,  2.34773988,  0.07718497, -1.32087472, -0.71994221, -1.02121596, -1.56155048])
            self.rarm.set_config(qr)
            self.sync_robot()   
            self.larm_reference = True
            self.get_ready(False)    

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

    def get_ready(self, only_orientation = True, reverse = False):
        self.sync_object()
        if self.larm_reference:
            ori = self.rarm.wrist_orientation()
            pos = self.rarm_end_position()
        else:
            ori = self.larm.wrist_orientation()
            pos = self.larm_end_position()

        if reverse:
            ori = rot.reverse(ori)            
        R   = rot.orthogonal(ori)

        if only_orientation:
            arm = self.reference_arm()
            p   = arm.wrist_position()
            arm.set_target(p, R)
            self.arm_target()
        else:
            n = R[:,2]
            h = R[:,1]
            w = R[:,0]

            p   = pos - (self.board_offset[0]+self.depth)*n + self.board_offset[2]*h + self.board_offset[1]*w
                
            self.set_target(p, R)
            self.goto_target()
            self.sync_robot()        

        if self.larm_reference:
            pint.look_lg()
        else:
            pint.look_rg()

        self.calibrate_startpoint()        

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

        self.larm_target(wait = False)
        self.rarm_target(wait = False)
        pint.wait_until_finished(target_list = ['rarm', 'larm'])
        self.sync_object()
        '''
        '''
    
    def draw_shape(self, shape_trajectory):# add wait argument later
        arm = self.reference_arm()
        keep_dt = arm.dt
        arm.dt  = shape_trajectory.phi_end/100
        jt      = arm.js_project(shape_trajectory, relative = False, traj_type = 'polynomial')
        arm.dt  = keep_dt
        # jt.fix_points()
        jt.consistent_velocities()
        # jt.plot()
        if not gen.equal(jt.phi_end, shape_trajectory.phi_end):
            print "Warning from PyRide_PR2.draw_shape(): All the shape is not in the workspace. Part of it will be drawn !"
        
        L = sum(shape_trajectory.points_dist())
        '''
        shape_trajectory.plot3d()
        for j in range(7):
            jt.plot(axis = j, show_points = True)
        '''
        pint.run_config_trajectory(jt, duration = L/self.arm_speed, is_left_arm = self.larm_reference)

    def switch_arm(self):
        if self.larm_reference:
            self.larm_reference = False
        else:
            self.larm_reference = True
            
    def draw_from_canvas(self, canvas_shapes, width_pixels = 640, height_pixels = 480, plot = False, silent=True):
        self.sync_object()

        ori_l = self.larm.wrist_orientation()
        ori_r = self.rarm.wrist_orientation()
        wr = - ori_r[:,0]
        hr =   ori_r[:,1]
        nr =   ori_r[:,2]
        wl = - ori_l[:,0]
        hl =   ori_l[:,1]
        nl =   ori_l[:,2]
        
        self.control_mode   = "Fixed-Base"
        self.larm_reference = False

        self.rarm_startpoint = self.rarm_startpoint + nr*self.depth
        self.larm_startpoint = self.larm_startpoint + nl*self.depth

        pr_list = []
        pl_list = []
        
        for cs in canvas_shapes:
            if cs.arm == "R":
                phi     = 1.0
                (x0,y0) = cs.pts[0]
                pos0    = self.rarm_startpoint + self.box_width_rarm*x0*wr/width_pixels + self.box_height_rarm*(height_pixels-y0)*hr/height_pixels
                pr      = traj.Trajectory_Polynomial(dimension = 3)
                pr.add_point(0.0, np.copy(pos0), np.zeros(3))
                for (x,y) in cs.pts[1:]:
                    pos  = self.rarm_startpoint + self.box_width_rarm*x*wr/width_pixels + self.box_height_rarm*(height_pixels-y)*hr/height_pixels
                    pr.add_point(phi, pos)
                    # pr.new_segment()
                    phi += 1.0

                pr.consistent_velocities()
                pr_list.append(pr)
                if plot:
                    pr.plot3d()
            elif cs.arm == "L":     
                phi     = 1.0
                (x0,y0) = cs.pts[0]
                pos0    = self.larm_startpoint + self.box_width_larm*x0*wl/width_pixels + self.box_height_larm*(height_pixels-y0)*hl/height_pixels
                pl      = traj.Trajectory_Polynomial(dimension = 3)
                pl.add_point(0.0, np.copy(pos0), np.zeros(3))
                for (x,y) in cs.pts[1:]:
                    pos  = self.larm_startpoint + self.box_width_larm*x*wl/width_pixels + self.box_height_larm*(height_pixels-y)*hl/height_pixels
                    pl.add_point(phi, pos)
                    # pl.new_segment()
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
        t0     = time.time()

        t      = 0.0
        # while ((ir < nrt) or (il < nlt)) and (not pint.rarm_failed) and (not pint.larm_failed) and (t < self.timeout):
        while ((ir < nrt) or (il < nlt)) and (not pint.rarm_failed) and (not pint.larm_failed):

            # initial positioning:
            if (ir < nrt) and (step_r == 0):
                t0 = time.time()
                if not silent:
                    print "Trajectory: ", ir, " Step 0 for Right Arm"
                pr     = pr_list[ir]
                pos    = np.copy(pr.segment[0].point[0].pos)
                pos    = pos - self.depth*nr
                self.rarm.set_target(pos, ori_r)
                assert self.rarm.goto_target(optimize = True)
                pint.take_rarm_to(self.rarm.config.q, time_to_reach = 2.0)
                step_r += 1    

            if (il < nlt) and (step_l == 0):
                t0 = time.time()
                if not silent:
                    print "Trajectory: ", il, " Step 0 for Left Arm"
                pl     = pl_list[il]
                pos    = np.copy(pl.segment[0].point[0].pos)
                pos    = pos - self.depth*nl
                self.larm.set_target(pos, ori_l)
                assert self.larm.goto_target(optimize = True)
                pint.take_larm_to(self.larm.config.q, time_to_reach = 2.0)
                step_l += 1    
            # head forward:
            if (pint.rarm_reached) and (step_r == 1):
                t0 = time.time()
                if not silent:
                    print "Trajectory: ", ir, " Step 1 for Right Arm"
                pos    = np.copy(pr.segment[0].point[0].pos)
                self.rarm.set_target(pos, ori_r)
                assert self.rarm.goto_target(optimize = True)
                pint.take_rarm_to(self.rarm.config.q, time_to_reach = 1.0)
                step_r += 1    

            if (pint.larm_reached) and (step_l == 1):
                t0 = time.time()
                if not silent:
                    print "Trajectory: ", il, " Step 1 for Left Arm"
                pos    = np.copy(pl.segment[0].point[0].pos)
                self.larm.set_target(pos, ori_l)
                assert self.larm.goto_target(optimize = True)
                pint.take_larm_to(self.larm.config.q, time_to_reach = 1.0)
                step_l += 1    

            # run trajectory:
            if (pint.rarm_reached) and (step_r == 2):
                t0 = time.time()
                if not silent:
                    print "Trajectory: ", ir, " Step 2 for Right Arm"
                self.larm_reference = False
                # self.arm_trajectory(pr, relative = False, delta_phi = 0.5, wait = False)
                self.draw_shape(pr)
                step_r += 1    

            if (pint.larm_reached) and (step_l == 2):
                t0 = time.time()
                if not silent:
                    print "Trajectory: ", il, " Step 2 for Left Arm"
                self.larm_reference = True
                # self.arm_trajectory(pl, relative = False, delta_phi = 0.5, wait = False)
                nseg = len(pl.segment)
                # print "len(pl.segment)", nseg
                # print "num.points.last.seg:", len(pl.segment[nseg-1].point)
                #print pl.segment[nseg-1].point[0]
                #print pl.segment[nseg-1].point[1]
                #print pl.segment[nseg-1].points_dist()

                self.draw_shape(pl)
                step_l += 1    

            # head backward:
            if (pint.rarm_reached) and (step_r == 3):
                t0 = time.time()
                if not silent:
                    print "Trajectory: ", ir, " Step 3 for Right Arm"
                
                self.sync_object()
                pos    = self.rarm.wrist_position() - nr*self.depth
                self.rarm.set_target(pos, ori_r)
                assert self.rarm.goto_target(optimize = True)
                pint.take_rarm_to(self.rarm.config.q, time_to_reach = 1.0)
                step_r += 1    

            if (pint.larm_reached) and (step_l == 3):
                t0 = time.time()
                if not silent:
                    print "Trajectory: ", il, " Step 3 for Left Arm"

                self.sync_object()
                pos    = self.larm.wrist_position() - nl*self.depth
                self.larm.set_target(pos, ori_l)
                assert self.larm.goto_target(optimize = True)
                pint.take_larm_to(self.larm.config.q, time_to_reach = 1.0)
                '''
                self.larm_reference = True
                self.arm_back(dx = self.depth, relative = True)
                '''
                step_l += 1    

            if (pint.rarm_reached) and (step_r == 4):
                t0 = time.time()
                if not silent:
                    print "Trajectory: ", ir, " Step 4 for Right Arm"
                ir    += 1
                step_r = 0

            if (pint.larm_reached) and (step_l == 4):
                t0 = time.time()
                if not silent:
                    print "Trajectory: ", il, " Step 4 for Left Arm"
                il    += 1
                step_l = 0

            time.sleep(0.100)
            t = time.time() - t0

        self.sync_object()
        self.rarm_startpoint = self.rarm_startpoint - nr*self.depth
        self.larm_startpoint = self.larm_startpoint - nl*self.depth

        if t > self.timeout:
            print "Action not completed! Time Out!"        
        # Return back to start point:
        if nrt > 0:
            self.rarm.set_target(self.rarm_startpoint, ori_r)
            self.rarm_target(wait = True)    

        if nlt > 0:
            self.larm.set_target(self.larm_startpoint, ori_l)
            self.larm_target(wait = True)    
    

