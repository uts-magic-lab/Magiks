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

    def IK(self, dy = 0):    
        '''
        Returns the value of theta and change in x and y to reach the target position
        the problem is redundant so you can select dy, the default value is 0.
        '''
        # Calculate the relative position:
        p  = 
        R  = 

        numpy.dot(numpy.linalg.inv(self.current_orientation()), self.pd - self.current_position())
        
        px = 
        py = r*math.cos(psi)

        p_WR_BR = numpy.array([px, py, pz])

        # Calculate the relative endeffector orientation:
        R_B     = rotlib.rot_z(tau)  # orientation of robot base
        R_WR    = self.Rd
        p_EFR   = self.xd   
        R_WR_B  = numpy.dot(R_B.T, R_WR)

        #set the new target for the arm associated with the given phi
        
        self.rarm.set_target(p_WR_BR, R_WR_B)

        #Find all the solutions for the arm IK
        solution_set = []
        arm_solution_set = self.rarm.all_IK_solutions(phi[0])
        
        if len(arm_solution_set) == 0:
            print "IK Error: No solution for the arm for given phi"
        else:
            
            # Calculate the telescopic prismatic joint:
    
            h_ts  = - pz + self.xd[2] - self.Rd[2,2]*self.d7

            #calculate the base position:

            p_BR_BO = numpy.array([0.0, self.l0, h_ts])

            p_BO    = p_EFR - numpy.dot(R_B,(p_BR_BO + p_WR_BR)) - numpy.dot(R_WR, self.p_EFR_WR)
            '''
            print "in IK: p_BO    = ", p_BO
            print "in IK: p_EFR   = ", p_EFR
            print "in IK: R_B     = ", R_B
            print "in IK: p_BR_BO = ", p_BR_BO
            print "in IK: h_ts    = ", h_ts
            print "in IK: pz      = ", pz
            print "in IK: xd[2]   = ", self.xd[2]
            print

            print "in IK: p_WR_BR = ", p_WR_BR
            print "in IK: R_WR_B  = ", R_WR_B
            print "in IK: theta0  = ", phi[0]

            '''
            assert gen.equal(p_BO[2], 0.0) # z coordinate of the base must be zero

            # Now we have all the joints(all 11 degrees of freedom) insert extra DOFs to the arm solution set

            for arm_solution in arm_solution_set:
                solution = numpy.zeros(11)
                solution[0:7] = arm_solution
                solution[7]   = h_ts     # append q[7] = h_ts
                solution[8]   = p_BO[0]  # append q[8] = X_BO
                solution[9]   = p_BO[1]  # append q[9] = Y_BO
                solution[10]  = tau      # append q[10] = tau
                solution_set.append(solution) # append the solution to the solution set

        #return back the previous target of the arm

        self.rarm.set_target(save_arm_target_position, save_arm_target_orientation)
        

        return solution_set

    def objective_function_customized(self, q):    
        '''
        Returns the value of objective function for a given configuration "q"
        Why has this function been written?
        Sometimes you want to find the value of the objective function for a configuration but 
        you don't want to set that configuration (you don't want to take the manipulator to that configuration)
        In this case, you can not use function "self.objective_function()" which gives the value of the objective 
        function for the current configuration of the robot.
        '''

        if self.ofuncode == 0:
            delta = trig.angles_standard_range(q - self.q)
            ofun = numpy.dot(delta.T, self.W*delta)
        elif self.ofuncode == 1:
            delta = trig.angles_standard_range(q - self.qm)
            ofun = numpy.dot(delta.T, self.W*delta)
        else:
            print "IK_config error: Value ",self.ofun," for property ofun is not supported"
            assert False
        return ofun

    def IK_config(self, phi):    
        '''
        Finds the solution of the Inverse Kinematic problem for given redundant parameter vector "phi"
        In case of redundant solutions, the one corresponding to the lowest objective function is selected.
        property "self.ofuncode" specifies the objective function. look at the ofuncode legend in __init__().
            ofuncode = 0 (Default) the solution closest to current joint angles will be selected 
            ofuncode = 1 the solution corresponding to the lowest midrange distance is selected
        This function does NOT set the configuration so the joints do not change
        ''' 
        solution_set = self.all_IK_solutions(phi)

        if len(solution_set) == 0:
            print "IK_config error: No solution found within the feasible joint ranges"
            return []

        ofun_min = 1000
        for i in range(0, len(solution_set)):
            solution = solution_set[i]
            ofun = self.objective_function_customized(solution)

            if ofun < ofun_min:
                ofun_min = ofun
                i_min = i
            
        return solution_set[i_min]

    """
    def rarm_grip_ori_wrt_tor(self)
        '''
        Returns the right arm gripper rotation matrix (endeffector orientation) with respect to the torso
        '''
        return self.rarm.wrist_orientation()
    """

    def pos_rarm_grip_wrt_tor_shpan(self):
        '''
        Returns the right arm gripper position vector(endeffector position) with respect to the torso shoulder pan joint center
        '''
        R_WR_B   = self.rarm.wrist_orientation()
        p_WR_BR  = self.rarm.wrist_position()
        p_EFR_BR = p_WR_BR + numpy.dot(R_WR_B, self.p_EFR_WR)
        return(p_EFR_BR)

    def pos_larm_grip_wrt_tor_shpan(self):
        '''
        Returns the left arm gripper position vector(endeffector position) with respect to the torso shoulder pan joint center
        '''
        R_WL_B   = self.larm.wrist_orientation()
        p_WL_BL  = self.larm.wrist_position()
        p_EFL_BL = p_WL_BL + numpy.dot(R_WL_B, self.p_EFL_WL)
        return(p_EFL_BL)

    def pos_larm_grip_wrt_tor(self):
        '''
        Returns the left arm gripper position vector(endeffector position) with respect to the torso at the origin.
        The torso origin is at the floor footprint (projection) of the middle point between the two shoulder pan joint centers.
        '''
        p_EFL_BL = self.pos_larm_grip_wrt_tor_shpan()
        p_EFL_BO = self.p_BL_BO + p_EFL_BL
        return(p_EFL_BO)

    def pos_rarm_grip_wrt_tor(self):
        '''
        Returns the right arm gripper position vector(endeffector position) with respect to the torso at the origin.
        The torso origin is at the floor footprint (projection) of the middle point between the two shoulder pan joint centers.
        '''
        p_EFR_BR = self.pos_rarm_grip_wrt_tor_shpan()
        p_EFR_BO = self.p_BR_BO + p_EFR_BR
        return(p_EFR_BO)

    def pos_rarm_grip(self):
        '''
        Returns the global cartesian coordiantes of the end of right arm gripper.
        '''    
        p_EFR_BO = self.pos_rarm_grip_wrt_tor()
        return(self.p_BO + p_EFR_BO)

    def pos_larm_grip(self):
        '''
        Returns the global cartesian coordiantes of the end of left arm gripper.
        '''    
        p_EFL_BO = self.pos_larm_grip_wrt_tor()
        return(self.p_BO + p_EFL_BO)

    def rarm_endeffector_position(self):        
        '''
        Returns the cartesian coordiantes of the end of gripper as the endeffector of the right arm.
        '''    
        if not self.rarm_endeff_position_updated:

            self.R_WR    = self.rarm_endeffector_orientation()
            self.p_WR_BR = self.rarm.wrist_position()

            self.p_EFR   = self.p_BO + numpy.dot(self.R_B,(self.p_BR_BO + self.p_WR_BR)) + numpy.dot(self.R_WR, self.p_EFR_WR)
            self.rarm_endeff_position_updated = True

        return copy.copy(self.p_EFR)
               
    def larm_endeffector_position(self):        
        '''
        Returns the cartesian coordiantes of the end of gripper as the endeffector of the left arm.
        '''    
        if not self.larm_endeff_position_updated:

            self.R_WL    = self.larm_endeffector_orientation()
            self.p_WL_BL = self.larm.wrist_position()

            self.p_EFL   = self.p_BO + numpy.dot(self.R_B,(self.p_BL_BO + self.p_WL_BL)) + numpy.dot(self.R_WL, self.p_EFL_WL)
            self.larm_endeff_position_updated = True

        return copy.copy(self.p_EFL)

    def rarm_endeffector_orientation(self):        
        '''
        Returns the cartesian coordiantes of the right arm gripper as the endeffector frame.
        '''    
        if not self.rarm_endeff_orientation_updated:

            self.R_WR_B  = self.rarm.wrist_orientation()
            self.R_WR    = numpy.dot(self.R_B, self.R_WR_B)
            self.rarm_endeff_orientation_updated = True

        return copy.copy(self.R_WR)

    def larm_endeffector_orientation(self):        
        '''
        Returns the cartesian coordiantes of the left arm gripper as the endeffector frame.
        '''    
        if not self.larm_endeff_orientation_updated:

            self.R_WL_B  = self.larm.wrist_orientation()
            self.R_WL    = numpy.dot(self.R_B, self.R_WL_B)
            self.larm_endeff_orientation_updated = True

        return copy.copy(self.R_WL)

    def div_theta_err(self):
        '''
        Returns the divergence of the vector of kinematic constraints "e" in respect with the joints
        It is a 9*9 matrix
        e_ij = rond e_i / rond q_j  (for i,j = 0,..,8)   
        '''
        if not self.div_theta_err_updated:

            self.E = numpy.zeros((9,9))
            self.F = numpy.zeros((9,5))

            phi = numpy.zeros(5)                

            [s0, s1, s2, s3, s4, s5, s6] = self.rarm.config.s
            [c0, c1, c2, c3, c4, c5, c6] = self.rarm.config.c

            tau = self.q[10]
            C5  = math.cos(tau)
            S5  = math.sin(tau)

            [s10]         = self.rarm.config.s1_mult1
            [s20,s21, s22, s210]  = self.rarm.config.s2_mult1
            [s30, s31, s32, s310, s320, s321, s322, s33,s3210] = self.rarm.config.s3_mult1
            [c0s0,c0s1,c0s2,c0s10,c0s20,c0s21,c0s30,c0s31,c0s32,c0s321] = self.rarm.config.c0_mult
            [c10, c1s0, c1s1, c1s2, c1s3, c1s10, c1s20,c1s30,c1s32,c1s320] = self.rarm.config.c1_mult
            [c20,c21,c22,c210,c2s0,c2s1,c2s2,c2s3,c2s10,c2s20,c2s30,c2s31,c2s310,c21s30,c20s31] = self.rarm.config.c2_mult
            [c30,c31,c32,c33,c3s0,c3s1,c3s2,c3s3,c3s10,c3s20,c3s21,c3s30,c310,c320,c321,c3210,c32s0,c32s10] = self.rarm.config.c3_mult
            [c21s0,c31s0,c321s0]  = self.rarm.config.s0_mult
            [c10s1,c20s1,c30s1, c32s1,c320s1] = self.rarm.config.s1_mult2
            [c10s2,c20s2,c30s2, c32s2,c310s2]  = self.rarm.config.s2_mult2
            [c10s3, c20s3, c30s3, c21s3, c210s3,s332]  = self.rarm.config.s3_mult2
            [c10s32, c31s20, s54] = self.rarm.config.cs_mult
            [c54, c5s4, c5s5] = self.rarm.config.c5_mult
            [s64, s65, c4s6, c5s6, c54s6, s654] = self.rarm.config.s6_mult
            [c64, c65, c6s4, c6s5, c654, C6s54] = self.rarm.config.c6_mult

            phi = self.redundant_parameters()
        
            Rd = self.rarm_endeffector_orientation()
            xd = self.rarm_endeffector_position()

            [Q, Q2, d42, d44, a00, d22_plus_d44, foura00d44, alpha] = self.rarm.additional_dims

            r2   = phi[1]**2
            rho2 = r2 + phi[2]**2
            R2   = rho2 - self.rarm.l**2
            a0   = self.rarm.a0
            d2   = self.rarm.d2
            d4   = self.rarm.d4

            a = Rd[:,2]
            n = Rd[:,0]
            b = xd - self.d7*a

            ax_c5_m_ay_s5 = a[0]*C5 - a[1]*S5
            ax_s5_p_ay_c5 = a[0]*S5 + a[1]*C5

            nx_c5_m_ny_s5 = n[0]*C5 - n[1]*S5
            nx_s5_p_ny_c5 = n[0]*S5 + n[1]*C5

            C14           = math.cos(phi[0] + phi[3])
            S14           = math.sin(phi[0] + phi[3])
            C45           = math.cos(phi[4] + phi[3])
            S45           = math.sin(phi[4] + phi[3])

            self.E[0,2] = - 2*d42*s3
            self.E[2,1] =   self.rarm.d4*s321
            self.E[3,4] =   -s5
            self.E[4,3] =   c4*s5
            self.E[4,4] =   c5*s4

            self.E[5,4] =   c65
            self.E[5,5] = - s65
            self.E[8,8] =   1.0
            self.E[2,6] =   1.0

            self.E[1,1] = 2*a00*d44*s33*c2s2
            self.E[1,2] = 2*a0*d42*s3*phi[1]*S14 + (a0**2)*(d4**2)*(s2**2)*(2*c3*s3)
            # Alternative:  self.E[1,2] = - s3*d42*(2*c3*d42 - R2) + 2*c3s3*(a0**2)*(d4**2)*(s2**2) 

            self.E[6,6] = 1.0
            self.F[6,2] = 1.0

            self.E[2,0] = - self.rarm.d2*s1 - self.rarm.d4*(c3s1 + c21s3) 
            self.E[2,2] =  - self.rarm.d4*(c1s3 + c32s1) 
            self.E[3,0] =  ax_c5_m_ay_s5*(c20s31 - c310) + ax_s5_p_ay_c5*(c2s310 - c31s0) + a[2]*(c3s1 + c21s3) 
            self.E[3,1] =  ax_c5_m_ay_s5*(c2s30 + c10s32) - ax_s5_p_ay_c5*(c20s3 - c1s320) - a[2]*s321 
            self.E[3,2] =  ax_c5_m_ay_s5*(c3s20 - c3210 + c0s31) - ax_s5_p_ay_c5*(c30s2 + c321s0 - s310) + a[2]*(c1s3 + c32s1)
            self.E[4,0] =  - ax_c5_m_ay_s5*c0s21 - ax_s5_p_ay_c5*s210 - a[2]*c1s2
            self.E[4,1] =  ax_c5_m_ay_s5*(c210 -s20)  + ax_s5_p_ay_c5*(c0s2 + c21s0) - a[2]*c2s1
                             
            self.E[5,0] =  nx_c5_m_ny_s5*(- c20s31 + c310) + nx_s5_p_ny_c5*(- c2s310 + c31s0) - n[2]*(c3s1 + c21s3) 
            self.E[5,1] =  - nx_c5_m_ny_s5*(c2s30 + c10s32) + nx_s5_p_ny_c5*(c20s3 - c1s320) + n[2]*s321     
            self.E[5,2] =    nx_c5_m_ny_s5*(- c3s20 + c3210 - c0s31) + nx_s5_p_ny_c5*(c30s2 + c321s0 - s310) - n[2]*(c1s3 + c32s1) 
            self.E[7,7] =  1.0
            self.E[8,8] =  1.0

            self.F[0,2] = -2*phi[2]

            self.F[7,1] =  S45
            self.F[8,1] =  C45

            self.F[0,0] =   2*self.rarm.a0*phi[1]*C14 
            self.F[0,1] =   2*self.rarm.a0*S14 - 2*phi[1]
            self.F[0,3] =   2*self.rarm.a0*phi[1]*C14 

            self.F[1,1] =   2*phi[1]*self.rarm.a0*(phi[1]*S14 - self.rarm.a0)
            self.F[1,2] =   2*phi[1]*self.rarm.a0*S14*phi[2]

            self.F[3,0] =   ax_c5_m_ay_s5 *(c0s32 + c21s30 + c3s10) + ax_s5_p_ay_c5*(s320 - c210s3 - c30s1) 
            self.F[3,4] =   - ax_s5_p_ay_c5*(s320 - c210*s3 - c30*s1) - ax_c5_m_ay_s5*(c0*s32 + c21*s30 + c3s10)
            self.F[4,0] =   ax_c5_m_ay_s5*(c20 - c1s20) + ax_s5_p_ay_c5*(c2s0 + c10s2)
            self.F[4,4] = - ax_c5_m_ay_s5*(c20 - c1s20) - ax_s5_p_ay_c5*(c2s0 + c10s2)  
            self.F[5,0] = - nx_c5_m_ny_s5*(c0s32 + c21s30 + c3s10) + nx_s5_p_ny_c5*(-s320 + c210s3 + c30s1) 
            self.F[5,4] = + nx_c5_m_ny_s5*(c0s32 + c21s30 + c3s10) - nx_s5_p_ny_c5*(-s320 + c210s3 + c30s1)
            self.F[7,4] =   phi[1]*C45 + C5*self.l0
            self.F[8,4] =   - phi[1]*S45 - S5*self.l0
            self.F[7,3] =   phi[1]*C45
            self.F[8,3] =   - phi[1]*S45

            self.div_theta_err_updated = True

        return copy.copy(self.E)

    def redundant_parameters(self):
        '''
        returns vector of redundant parameters phi according to the parametrization specified:

        phi[0] = The first arm joint angle (Shoulder Pan joint)
        phi[1] = The "r" coordinate of the relative wrist position(in cylinderical coordinates) phi[1]^2 = px^2 + py^2
        phi[2] = The "z" coordinate of the relative wrist position(in cylinderical coordinates) phi[2]   = pz
        phi[3] = The "psi" coordinate of the relative wrist position(in cylinderical coordinates) phi[3] = arctan(x/y)
        phi[4] = The rotation of the base. Same as "tau" or the 10th joint angle (phi[4] = q[10] = tau)
        '''
        if not self.redundant_parameters_updated:
            p = self.rarm.wrist_position()
            r2 = p[0]**2 + p[1]**2
            self.phi = numpy.zeros(5)
            self.phi[0] = self.rarm.config.q[0]
            self.phi[1] = math.sqrt(r2)
            self.phi[2] = p[2] 
            self.phi[3] = math.atan2(p[0],p[1]) 
            self.phi[4] = self.q[10]
            self.redundant_parameters_updated = True
        return copy.copy(self.phi)    

    def div_phi_err(self):
        '''
        Returns the divergence of the vector of kinematic constraints "e" in respect with the redundant parameters "phi"
        It is a 9*5 matrix
        f_ij = rond e_i / rond phi_j  (for i = 0,..,8 & j = 0,..,4) 
        the value of "self.F" is computed in function "div_theta_e" to avoid repeated calculations
        '''
        if not self.div_theta_err_updated:    
            self.div_theta_err()
        return copy.copy(self.F)

    def redundancy_jacobian(self):
        '''
        Returns the redundancy jacobian
        Redundancy Jacobian is the partial derivative of the joints in respect with redundant parameters
        RJ is a 9*5 matrix because the first and the last joints (q[0] and q[10]) are themselves selected as the redundant parameters
        '''
        if not self.redun_jacob_updated:

            d42 = self.rarm.d2*self.rarm.d4
    
            [s0, s1, s2, s3, s4, s5, s6] = self.rarm.config.s
            [c0, c1, c2, c3, c4, c5, c6] = self.rarm.config.c

            s321 = s3*s2*s1
            c65  = c6*c5
            s65  = s6*s5

    
            p = self.endeffector_position()

            r2 = p[0]**2 + p[1]**2
            p1 = math.sqrt(r2)
            p2 = self.rarm.xd[2]
        

            C4 = math.cos(self.q[10])
            S4 = math.sin(self.q[10])

            RJ = numpy.zeros((9,5))        

            E = self.div_theta_err()
            F = self.div_phi_err()


            RJ[2,0] =  F[0,0]/(2*d42*s3)
            RJ[2,1] =  F[0,1]/(2*d42*s3)
            RJ[2,2] =  -p2/(d42*s3)
            RJ[2,3] =  F[0,3]/(2*d42*s3)

            RJ[8,1] =  -F[8,1]
            RJ[8,3] =  -F[8,3]
            RJ[8,4] =  -F[8,4]

            RJ[6,2] =  - 1.0

            RJ[ 7 , 1 ] =  -F[7,1]
            RJ[ 7 , 3 ] =  -F[7,3]
            RJ[ 7 , 4 ] =  -F[7,4]
            '''
            RJ[7,0] =  -RJ[8,0]*E[7,8]/E[7,7]
            RJ[7,1] =  (2*p1 - RJ[8,1]*E[7,8])/E[7,7]
            RJ[7,2] =  -RJ[8,2]*E[7,8]/E[7,7]
            RJ[7,3] =  -RJ[8,3]*E[7,8]/E[7,7]
            RJ[7,4] =  -(F[7,4] + RJ[8,4]*E[7,8])/E[7,7]
            '''
            RJ[1,0] =  -RJ[2,0]*E[1,2]/E[1,1]
            RJ[1,1] =  -(F[1,1] + RJ[2,1]*E[1,2])/E[1,1]
            RJ[1,2] =  -(F[1,2] + RJ[2,2]*E[1,2])/E[1,1]
            RJ[1,3] =  -RJ[2,3]*E[1,2]/E[1,1]
            RJ[1,4] =  -RJ[2,4]*E[1,2]/E[1,1]

            RJ[0,0] =  -(RJ[6,0] + RJ[2,0]*E[2,2] + RJ[1,0]*self.rarm.d4*s321)/E[2,0]
            RJ[0,1] =  -(RJ[6,1] + RJ[2,1]*E[2,2] + RJ[1,1]*self.rarm.d4*s321)/E[2,0]
            RJ[0,2] =  -(RJ[6,2] + RJ[2,2]*E[2,2] + RJ[1,2]*self.rarm.d4*s321)/E[2,0]
            RJ[0,3] =  -(RJ[6,3] + RJ[2,3]*E[2,2] + RJ[1,3]*self.rarm.d4*s321)/E[2,0]
            RJ[0,4] =  -(RJ[6,4] + RJ[2,4]*E[2,2] + RJ[1,4]*self.rarm.d4*s321)/E[2,0]

            RJ[4,0] =  (F[3,0] + RJ[0,0]*E[3,0] + RJ[1,0]*E[3,1] + RJ[2,0]*E[3,2])/s5
            RJ[4,1] =  (RJ[0,1]*E[3,0] + RJ[1,1]*E[3,1] + RJ[2,1]*E[3,2])/s5
            RJ[4,2] =  (RJ[0,2]*E[3,0] + RJ[1,2]*E[3,1] + RJ[2,2]*E[3,2])/s5
            RJ[4,3] =  (RJ[0,3]*E[3,0] + RJ[1,3]*E[3,1] + RJ[2,3]*E[3,2])/s5
            RJ[4,4] =  (- F[3,0] + RJ[0,4]*E[3,0] + RJ[1,4]*E[3,1] + RJ[2,4]*E[3,2])/s5

            RJ[3,0] =  -(F[4,0] + RJ[0,0]*E[4,0] + RJ[1,0]*E[4,1] + RJ[4,0]*c5*s4)/(c4*s5)
            RJ[3,1] =  -(RJ[0,1]*E[4,0] + RJ[1,1]*E[4,1] + RJ[4,1]*c5*s4)/(c4*s5)
            RJ[3,2] =  -(RJ[0,2]*E[4,0] + RJ[1,2]*E[4,1] + RJ[4,2]*c5*s4)/(c4*s5)
            RJ[3,3] =  -(RJ[0,3]*E[4,0] + RJ[1,3]*E[4,1] + RJ[4,3]*c5*s4)/(c4*s5)
            RJ[3,4] =  -(F[4,4] + RJ[0,4]*E[4,0] + RJ[1,4]*E[4,1] + RJ[4,4]*c5*s4)/(c4*s5)

            RJ[5,0] =  (F[5,0] + RJ[0,0]*E[5,0] + RJ[1,0]*E[5,1] + RJ[2,0]*E[5,2] + RJ[4,0]*c65)/s65
            RJ[5,1] =  (RJ[0,1]*E[5,0] + RJ[1,1]*E[5,1] + RJ[2,1]*E[5,2] + RJ[4,1]*c65)/s65
            RJ[5,2] =  (RJ[0,2]*E[5,0] + RJ[1,2]*E[5,1] + RJ[2,2]*E[5,2] + RJ[4,2]*c65)/s65
            RJ[5,3] =  (RJ[0,3]*E[5,0] + RJ[1,3]*E[5,1] + RJ[2,3]*E[5,2] + RJ[4,3]*c65)/s65
            RJ[5,4] =  (- F[5,0] + RJ[0,4]*E[5,0] + RJ[1,4]*E[5,1] + RJ[2,4]*E[5,2] + RJ[4,4]*c65)/s65

            self.RJ = RJ    
            self.redun_jacob_updated = True
        return copy.copy(self.RJ)

    def redundancy_jacobian_extended(self):
        '''
        returns the extended redundancy jacobian which is "11 x 5" matrix.
        It is the redundancy jacobian with two rows inserted At its first and last rows. 
        These rows are corresponding to the first and last joints (shoulder pan joint and base rotation angle).
        '''    
        if not self.redun_jacob_ext_updated:
            self.ERJ = numpy.zeros((11,5))
            RJ  = self.redundancy_jacobian()
            for i in range(9):
                self.ERJ[i+1,:] = RJ[i,:]
            self.ERJ[0,0]  = 1.0
            self.ERJ[10,4] = 1.0
            self.redun_jacob_ext_updated = True
        return copy.copy(self.ERJ)

    def div_theta_ofun(self):
        '''
        Returns the divergence of the objective function in respect with the joint angles
        argument "ofun" specifies the code of the objective function which is set to 0 by default 
        '''        
        if not self.div_theta_ofun_updated:
            self.DTG = numpy.zeros(11)
            for i in range(11):
                self.DTG[i] = 2*self.W[i]*(self.q[i] - self.qm[i])    
            self.div_theta_ofun_updated = True
        return copy.copy(self.DTG)    

    def div_phi_ofun(self):
        '''
        Returns the divergence of the objective function in respect with the redundant parameters "phi"
        argument "ofun" specifies the code of the objective function which is set to 0 by default. 
        The code legend is like function "div_theta_ofun" 
        '''
        if not self.div_phi_ofun_updated:
            J = self.redundancy_jacobian_extended()
            self.DFG = numpy.dot(J.T, self.div_theta_ofun())
            self.div_phi_ofun_updated = True
        return copy.copy(self.DFG)

    def joint_stepsize_interval(self, direction):
        '''
        Returns an interval for the step size by which the joints are allowed to move in the given "direction"
        "direction" is a "n X 1" vector and specifies the direction of motion in the jointspace. 
        '''
        stepsize_interval = interval([-inf, inf])
        for i in range(11):  #  for each joint
            if self.W[i] != 0: # if the joint is limited
                joint_correction_interval = interval([self.ql[i], self.qh[i]]) - interval(self.q[i])  # find the interval for joint correction
                '''
                print "joint_interval[",i,"]           = ", interval([self.ql[i], self.qh[i]])
                print "q[",i,"]                        = ", self.q[i]
                print "joint_correction_interval[",i,"]= ", joint_correction_interval
                print
                '''
            else:
                joint_correction_interval = interval([-inf, inf])

            stepsize_interval = stepsize_interval & joint_correction_interval/direction[i]

            '''
            print "direction :                         = ", direction[i]
            print "stepsize_interval:                  = ", stepsize_interval
            print
            '''
        
        return stepsize_interval
                
    def optimum_joint_stepsize(self, direction, safety_factor = 0.99):
        '''
        returns the optimum feasible step size that minimizes the objective function 
        if the joints are constrained to move in the given "direction".
        The "safety_factor" specifies how much we can approach to the borders of joint limits
        '''
        den = numpy.dot(direction.T, self.W * direction)
        num = numpy.dot(direction.T, self.W * (self.q - self.qm))

        if not gen.equal(num,0.0):
            eta = - num/den
        else:
            print "optim_joint_stepsize Error: Moving in the given direction does not influence the objective function"

        interval_eta   = self.joint_stepsize_interval(direction)

        (eta_l, eta_h) = interval_eta[0]
        
        if eta < eta_l:
            return safety_factor*eta_l
        elif eta < eta_h:
            return eta
        else:
            return safety_factor*eta_h


    def steepest_descent_redundancy_correction(self):
        '''
        Finds the steepest descent direction of the objective function for redundancy vector "phi"
        '''
        steepest_descent_redundancy_direction = - self.div_phi_ofun()
        steepest_descent_joint_direction      = numpy.dot(self.redundancy_jacobian_extended(), steepest_descent_redundancy_direction)

        eta = self.optimum_joint_stepsize(steepest_descent_joint_direction)

        
        return eta*steepest_descent_redundancy_direction


    def endeffector_in_target(self):
        '''
        Returns true is endeffector is in target
        '''
        if not self.in_target_evaluated:
            self.in_target = vecmat.equal(self.xd, self.endeffector_position()) and vecmat.equal(self.Rd, self.endeffector_orientation())
            self.in_target_evaluated = True
        return self.in_target

    def optimal_internal_motion(self):
        '''
        Assuming the endeffector is in target,
        this function changes the robot configuration without changing the endeffector pose
        until the minimum possible value of objective function is achieved
        Note:
        This function is a procedure and does not return any thing. Running this function, changes the robot configuration. 
        If the current tip pose is not in target, this function fails with an error
        '''    
        if not self.endeffector_in_target():
            print "optimal_internal_motion Error: Endeffector is not in target pose"
            return None
        
        ofun_old = self.objective_function()
        k = 1.0
        ofun_changed   = True
        solution_exists = True

        while ofun_changed:
            phi = self.redundant_parameters()
            # Find the steepest descent correction for redundancy:    
            delta_phi = self.steepest_descent_redundancy_correction() 
            # Correct the redundancy
            phi = phi + k*delta_phi
            # Run IK to find the joints with the corrected redundancy:
            solution = self.IK_config(phi)
            # Compute the value of the objective function for the new configuration
            ofun_new = self.objective_function_customized(solution)
            # set the new configuration
            ofun_reduced = ofun_new < ofun_old - gen.epsilon
            ofun_raised  = ofun_new > ofun_old + gen.epsilon
            ofun_changed = ofun_reduced or ofun_raised
            '''
            print "------------------"
            if ofun_reduced:
                print "ofun reduced"
            elif ofun_raised:
                print "ofun raised"
            else:
                print "ofun not changed"
            print 
            print "ofun old: ", ofun_old 
            print "ofun new: ", ofun_new 
            '''

            if ofun_reduced:  # if the objective function has significantly reduced
                solution_exists = self.set_config(solution)
            if solution_exists:
                ofun_old        = self.objective_function()
                
            if (not solution_exists) or (ofun_raised):  # if error raised or solution does not exist
                k = k/10.0
            
    def inverse_update(self):
        '''
        Finds the inverse kinematic solution closest to the current configuration
        The new joint angles will be set if all the kinematic equations are satisfied. 
        All kinematic parameters will be updated.
        THIS FUNCTION IS NOT COMPLETE AND i NEED TO WORK ON IT MORE ...
        '''

        #Step 1: Check if a solution exists
        nz = self.Rd[2,0]
        zd = self.xd[2]
        lower_bound = self.rarm.d2*math.cos(self.qh[1]) - self.rarm.d4 + nz*self.d7 + self.ql[7]
        upper_bound = math.cos(self.ql[1])*(self.rarm.d2 + self.rarm.d4*math.cos(self.ql[3])) + nz*self.d7 + self.qh[7]
        if (zd < lower_bound):
            print "inverse_update Error: Desired endeffector height too low. Unable to reach" 
            return None
        if (zd > upper_bound):
            print "inverse_update Error: Desired endeffector height too high. Unable to reach" 
            return None

        #Step 2: Find a proper correction for q[1], q[2], q[3], q[7]
        Delta_q = numpy.zeros(11)
        [s0, s1, s2, s3, s4, s5, s6] = self.rarm.config.s
        [c0, c1, c2, c3, c4, c5, c6] = self.rarm.config.c

        tau = self.q[10]
        C5  = math.cos(tau)
        S5  = math.sin(tau)

        e = self.q[7] + c1*self.rarm.d2 + (c3*c1 - c2*s3*s1)*self.rarm.d4 + nz*self.d7 - zd

        print "e = ", e

        while abs(e) > 0.0001:

            j7 = 1.0 # rond e / rond q7
            j1 = - s1*self.rarm.d2 -self.rarm.d4*(c3*s1+c2*c1*s3)
            j2 = s3*s2*s1*self.rarm.d4
            j3 = -self.rarm.d4*(s3*c1 + c3*c2*s1)

            den = j1**2 + j2**2+ j3**2 + j7**2

            Delta_q[1] = - e*j1/den
            Delta_q[2] = - e*j2/den 
            Delta_q[3] = - e*j3/den 
            Delta_q[7] = - e*j7/den 

            JSI = self.joint_stepsize_interval(Delta_q)

            (eta_l, eta_h) = JSI[0]
            
            if 1 > eta_h:
                eta = 0.99*eta_h
            else:
                eta = 1.0

            print
            print "eta = ", eta
            print
            
            q2= self.q + Delta_q*eta

            assert self.set_config(q2)

            [s0, s1, s2, s3, s4, s5, s6] = self.rarm.config.s
            [c0, c1, c2, c3, c4, c5, c6] = self.rarm.config.c

            e = self.q[7] + c1*self.rarm.d2 + (c3*c1 - c2*s3*s1)*self.rarm.d4 + nz*self.d7 - zd

            print "e = ", e

        #compare with the previous e, make sure it has reduced
        
        
                        
