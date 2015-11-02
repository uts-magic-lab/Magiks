## @file        	writer_pr2.py
#  @brief     		Contains a class inherited from Skilled_PR2 in skilled_pr2.py which is connected to a real-time robot,
#                   a real PR2 or PR2 in simulation that have special writing skills
#  @author      	Nima Ramezani Taghiabadi 
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. :   04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	2.0
# 
#  Start date:      21 November 2014
#  Last Revision:  	06 January 2015
#  \b Attention:
#  All actions of this module do not have any collision avoidance feature. So be prepared to stop the robot if any collision happens !

import numpy as np
import time, math

import skilled_pr2 as spr
import pyride_interpreter as pint

from math_tools import general_math as gen
from math_tools.geometry import trajectory as traj, rotation as rot, shape_trajectories as tsh

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
class Writer_PR2(spr.Skilled_PR2):
    def __init__(self):
        super(Writer_PR2, self).__init__()
        self.height = 0.05
        self.width  = 0.02
        self.arm_speed = 0.01

        self.board_offset   = np.array([ 0.092,  0.2 ,  0.1])
        self.larm_reference = True
    
    def write_A(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_forward(self.depth, relative = True)
        self.arm_right_up(dx   = 0.5*self.width, dy = self.height, relative = True)
        self.arm_right_down(dx = 0.5*self.width, dy = self.height, relative = True)

        self.arm_back(self.depth, relative = True)
        self.arm_left_up(0.75*self.width, 0.5*self.height, relative = True)
        self.arm_forward(self.depth, relative = True)
        self.arm_right(0.5*self.width, relative = True)
        self.arm_back(self.depth, relative = True)
        self.arm_right_down(0.6*self.width, 0.5*self.height, relative = True)

    def write_B(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_forward(self.depth, relative=True)
        B = tsh.B(height = self.height, direction = ori)
        self.arm_trajectory(B)
        self.arm_back(self.depth, relative=True)
        self.arm_right(0.25*self.height + 0.3*self.width, relative=True)

    def write_C(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_right_up(dy = (self.height- self.width/2) , dx = self.width, relative=True)
        self.arm_forward(self.depth, relative=True)
        C = tsh.C(height = self.height, width = self.width, direction = ori, adjust = False)
        self.arm_trajectory(C)
        self.arm_back(self.depth, relative=True)
        self.arm_right_down(dy = self.width/2 , dx = 0.3*self.width, relative=True)

    def write_D(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_forward(self.depth, relative=True)
        D = tsh.D(height = self.height, width = self.width, direction = ori)
        self.arm_trajectory(D)
        self.arm_back(self.depth, relative=True)
        self.arm_right(1.3*self.width, relative=True)

    def write_E(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_right(self.width, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_left(self.width, relative=True)
        self.arm_up(self.height, relative=True)
        self.arm_right(self.width, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_left_down(self.width, 0.5*self.height, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_right(0.8*self.width, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_right_down(dx = 0.6*self.width, dy = 0.5*self.height, relative = True)

    def write_F(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_forward(self.depth, relative=True)
        self.arm_up(self.height, relative=True)
        self.arm_right(self.width, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_left_down(self.width, 0.5*self.height, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_right(0.8*self.width, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_right_down(dx = 0.6*self.width, dy = 0.5*self.height, relative = True)

    def write_G(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_right_up(dx = self.width, dy = self.height- self.width/2, relative=True)
        self.arm_forward(self.depth, relative=True)
        G = tsh.G(height = self.height, width = self.width, direction = ori, adjust = False)
        self.arm_trajectory(G)
        '''
        self.arm_back(self.depth, relative=True)
        self.arm_left(dx= 0.25*self.height, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_right(dx= 0.4*self.height, relative=True)
        '''
        self.arm_back(self.depth, relative=True)
        self.arm_right_down(dx= 0.55*self.width, dy=self.height/2.0, relative=True)

    def write_H(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_forward(self.depth, relative=True)
        self.arm_up(self.height, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_down(0.5*self.height, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_right(self.width, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_up(0.5*self.height, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_down(self.height, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_right(0.3*self.width, relative=True)

    def write_I(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_forward(self.depth, relative=True)
        self.arm_right(0.6*self.width, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_left(0.3*self.width, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_up(self.height, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_left(0.3*self.width, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_right(0.6*self.width, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_right_down(0.3*self.width, self.height, relative=True)

    def write_J(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_up(self.width/2, relative=True)
        self.arm_forward(self.depth, relative=True)
        J = tsh.J(width = self.width, height = self.height, direction = ori)
        self.arm_trajectory(J)
        self.arm_back(self.depth, relative=True)
        self.arm_right_down(dy=self.height, dx = 0.4*self.width, relative=True)

    def write_U(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_up(self.height, relative=True)
        self.arm_forward(self.depth, relative=True)
        U = tsh.U(width = self.width, height = self.height, direction = ori)
        self.arm_trajectory(U)
        self.arm_back(self.depth, relative=True)
        self.arm_right_down(dy=self.height, dx = 0.4*self.width, relative=True)

    def write_K(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_forward(self.depth, relative=True)
        self.arm_up(self.height, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_right(self.width, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_left_down(self.width, self.height/2, relative=True)
        self.arm_right_down(self.width, self.height/2, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_right(0.3*self.width, relative=True)

    def write_L(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_up(self.height, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_down(self.height, relative=True)
        self.arm_right(self.width, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_right(0.3*self.width, relative=True)

    def write_M(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_forward(self.depth, relative=True)
        self.arm_up(self.height, relative=True)
        self.arm_right_down(dx = 0.6*self.width, dy = 0.5*self.height, relative=True)
        self.arm_right_up(  dx = 0.6*self.width, dy = 0.5*self.height, relative=True)
        self.arm_down(self.height, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_right(0.3*self.width, relative=True)

    def write_N(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_forward(self.depth, relative=True)
        N = tsh.N(width = self.width, height = self.height, direction = ori)
        self.arm_trajectory(N)
        self.arm_back(self.depth, relative=True)
        self.arm_right_down(dy=self.height, dx=0.3*self.width, relative=True)

    def write_O(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_up(self.height - self.width/2, relative=True)
        self.arm_forward(self.depth, relative=True)
        O = tsh.O(height = self.height, width = self.width, direction = ori, adjust = False)
        self.arm_trajectory(O)
        self.arm_back(self.depth, relative=True)
        self.arm_right_down(dx= 1.2*self.width, dy = self.height - self.width/2, relative=True)

    def write_P(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_forward(self.depth, relative=True)
        P = tsh.P(width = 0.4*self.height, height = self.height, direction = ori)
        self.arm_trajectory(P)
        self.arm_back(self.depth, relative=True)
        self.arm_right_down(dy=0.5*self.height, dx=0.25*self.height + 0.3*self.width, relative=True)

    def write_Q(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_up(self.height - self.width/2, relative=True)
        self.arm_forward(self.depth, relative=True)
        O = tsh.O(height = self.height, width = self.width, direction = ori, adjust = False)
        self.arm_trajectory(O)
        self.arm_back(self.depth, relative=True)
        self.arm_right_down(dx= 0.3*self.height, dy = 0.6*self.height, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_right_down(dx= 0.3*self.height, dy = 0.1*self.height, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_right_down(dx= 0.3*self.width, dy = - self.width/2 + 0.3*self.height, relative=True)

    def write_R(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_forward(self.depth, relative=True)
        P = tsh.P(width = 0.4*self.height, height = self.height, direction = ori)
        self.arm_trajectory(P)
        self.arm_right_down(dy=0.5*self.height, dx=0.25*self.height, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_right(dx=0.3*self.width, relative=True)

    def write_S(self):
        cth = math.cos(math.pi/3)
        sth = math.sin(math.pi/3)
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()

        self.arm_right_up(dy = 0.25*self.height*(3+cth), dx = 0.4*self.width + 0.25*self.height*(1+sth), relative=True)
        self.arm_forward(self.depth, relative=True)
        S = tsh.S(height = self.height, direction = ori)
        self.arm_trajectory(S)
        self.arm_back(self.depth, relative=True)
        self.arm_right_down(dy = 0.25*self.height*(1-cth), dx = 0.4*self.width + 0.6*self.width*(1+sth), relative=True)

    def write_T(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_right(self.width/2, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_up(self.height, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_left(self.width/2, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_right(self.width, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_right_down(0.3*self.width, self.height, relative=True)

    def write_V(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_up(self.height, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_right_down(0.5*self.width, self.height, relative=True)
        self.arm_right_up(0.5*self.width, self.height, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_right_down(0.3*self.width, self.height, relative=True)

    def write_W(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_up(self.height, relative=True)
        self.arm_forward(self.depth, relative=True)
        W = tsh.W(width = 0.4*self.height, height = self.height, direction = ori)
        self.arm_trajectory(W)
        self.arm_back(self.depth, relative=True)
        self.arm_right_down(dy=self.height, dx=0.3*self.width, relative=True)

    def write_X(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_forward(self.depth, relative=True)
        self.arm_right_up(self.width, self.height, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_left(self.width, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_right_down(self.width, self.height, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_right(0.3*self.width, relative=True)

    def write_Y(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_up(self.height, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_right_down(self.width/2, self.height/2, relative=True)
        self.arm_right_up(self.width/2, self.height/2, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_left_down(self.width/2, self.height/2, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_down(self.height/2, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_right(0.8*self.width, relative=True)

    def write_Z(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_up(self.height, relative=True)
        self.arm_forward(self.depth, relative=True)
        self.arm_right(self.width, relative=True)
        self.arm_left_down(self.width, self.height, relative=True)
        self.arm_right(self.width, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_right(0.3*self.width, relative=True)

    def write_M(self):
        arm  = self.reference_arm()
        ori  = arm.wrist_orientation()
        self.arm_forward(self.depth, relative=True)
        self.arm_up(self.height, relative=True)
        self.arm_right_down(dx = 0.5*self.width, dy = 0.5*self.height, relative=True)
        self.arm_right_up(  dx = 0.5*self.width, dy = 0.5*self.height, relative=True)
        self.arm_down(self.height, relative=True)
        self.arm_back(self.depth, relative=True)
        self.arm_right(0.3*self.width, relative=True)

    def write_char(self, char):
        # ori = self.endeffector_orientation()
        self.sync_object()
        if char == "A":
            self.write_A()
        elif char == "B":
            self.write_B()
        elif char == "C":
            self.write_C()
        elif char == "D":
            self.write_D()
        elif char == "E":
            self.write_E()
        elif char == "F":
            self.write_F()
        elif char == "G":
            self.write_G()
        elif char == "H":
            self.write_H()
        elif char == "I":
            self.write_I()
        elif char == "J":
            self.write_J()
        elif char == "K":
            self.write_K()
        elif char == "L":
            self.write_L()
        elif char == "M":
            self.write_M()
        elif char == "N":
            self.write_N()
        elif char == "O":
            self.write_O()
        elif char == "P":
            self.write_P()
        elif char == "Q":
            self.write_Q()
        elif char == "R":
            self.write_R()
        elif char == "S":
            self.write_S()
        elif char == "T":
            self.write_T()
        elif char == "U":
            self.write_U()
        elif char == "V":
            self.write_V()
        elif char == "W":
            self.write_W()
        elif char == "X":
            self.write_X()
        elif char == "Y":
            self.write_Y()
        elif char == "Z":
            self.write_Z()
        elif char == " ":
            self.arm_right(0.7*self.width, relative=True)

        else:
            print "Error from Skilled_PR2.write_char(): Charachter Unknown !"

    def write(self, s):
        for i in range(len(s)):
            self.write_char(s[i])


