'''   Header
File:           special_trajectories.py
Brief:    	    This module provides classes inherited from class Trajectory() in module trajectory.py
                containing a number of special trajectories in the jointspace or taskspace, like walking trajectories for 
                special humanoid robots like nao
                
Author:         Nima Ramezani Taghiabadi
                PhD Researcher
                Faculty of Engineering and Information Technology
                University of Technology Sydney (UTS)
                Broadway, Ultimo, NSW 2007, Australia
                Room No.: CB11.W7.101.07
                Mobile:   04 5027 4611
                Email(1): Nima.RamezaniTaghiabadi@student.uts.edu.au 
                Email(2): nima.ramezani@gmail.com
                Email(3): nima_ramezani@yahoo.com
                Email(4): ramezanitn@alum.sharif.edu
Version:	    1
Started:        17 July 2014
Last Revision:  17 July 2014

'''

import matplotlib.pyplot as plt
import trajectory as trajlib
import numpy as np
import copy, math

from packages.nima.mathematics import polynomials as pl
from packages.nima.mathematics import trigonometry as trig
from packages.nima.mathematics import general as gen

zv = np.zeros(3)

class Circle_3D(trajlib.Trajectory):

    def __init__(self, center = np.zeros(3), start_point = np.array([0.0, -1.0, 0.0]), normal_vector = np.array([1.0, 0.0, 0.0]), T = 2*math.pi):
        super(Circle_3D, self).__init__(dimension = 3)
        self.phi_end = T # phase period
        self.center  = center
        self.r       = np.linalg.norm(start_point - center)
        i_vect       = (start_point - center)/self.r
        j_vect       = np.cross(normal_vector, i_vect)
        self.R       = np.append(np.append([i_vect],[j_vect],axis = 0), [normal_vector], axis = 0).T
        self.current_position = start_point

    def set_phi(self, phi):
        self.current_phi = phi
        w  = 2*math.pi/self.phi_end
        w2 = w*w
        X  = self.r*math.cos(w*phi)
        Y  = self.r*math.sin(w*phi)
        self.current_position     = self.center + np.dot(self.R, np.array([  X   ,   Y   , 0.0]))
        self.current_velocity     = np.dot(self.R, np.array([- w*Y ,   w*X , 0.0]))
        self.current_acceleration = np.dot(self.R, np.array([- w2*X, - w2*Y, 0.0]))
                

class Colin_Graf_Trajectory(trajlib.Trajectory):
    '''
    A three dimensional walking taskspace trajectory for any humanoid robot like NAO based on paper written by: Colin Graf et.al. 2009
    "A Robust Closed-Loop Gait for the Standard Platform League Humanoid" 
    Proceedings of the 4th Workshop on Humanoid Soccer Robots

    It is a trajectory with dim 6. The first three are x,y,z coordinates of the position of right foot w.r.t. the COM.
    the next three are x,y,z coordinates of the position of left foot w.r.t. the COM.
    '''

    def __init__(self, first_half = True, xl = 0.22, yl=0.55, xm=0.22, ym=0.55, h=30.0, dx=0.0, dy=120.0, Ol = zv, Or = zv, Cs = zv, Ss = np.array([20.0,0.0,0.0])):
        super(Colin_Graf_Trajectory, self).__init__(dimension = 6)            

        self.xl  = xl
        self.yl  = yl
        self.xm  = xm
        self.ym  = ym
        self.h   = h
        self.dx  = dx
        self.dy  = dy
        self.Ol  = Ol
        self.Or  = Or
        self.Cs  = Cs
        self.Ss  = Ss

        self.phi_end = 0.5
        self.first_half = first_half
        self.which_trajectory = "pl_com"
        
    def tl_lift(self, phi):
        if (2*phi > self.xl) and (2*phi < self.xl + self.yl):
            return 0.5*(1.0- math.cos(2*math.pi*(2*phi-self.xl)/self.yl))
        else:
            return 0.0

    def tr_lift(self, phi):
        return self.tl_lift(phi-0.5)

    def tl_move(self, phi):
        if (2*phi > self.xm) and (2*phi < self.xm + self.ym):
            return 0.5*(1.0- math.cos(math.pi*(2*phi-self.xm)/self.ym))
        elif (2*phi >= self.xm + self.ym) and (2*phi <= 1.0):
            return 1.0
        else:
            return 0.0

    def tr_move(self, phi):
        return self.tl_move(phi-0.5)
            
    def tl(self, phi):
        if phi < 0.5:
            return 0.5*(1.0 - math.cos(2.0*math.pi*phi))
        else:
            return 0.0

    def tr(self, phi):
        if phi >= 0.5:
            return 0.5*(1.0 - math.cos(2.0*math.pi*(phi-0.5)))
        else:
            return 0.0

    def t_com(self, phi):    
        def ss(t):
            return math.sin(2*math.pi*t)
        def rr(t):
            s = ss(t)
            return math.sqrt(abs(s))*gen.sign(s)
        def ll(t):
            if (t < 0.25):
                return 4*t
            elif (t >= 0.25) and (t < 0.75):
                return 2 - 4*t
            else:
                return 4*(t - 1)
        xc = 20.0
        yc = 50.0
        zc = 2.0
        return (xc*ss(phi) + yc*rr(phi) + zc*ll(phi))/(xc + yc + zc)

    def t_lin(self, phi):
        if (phi < 0.5):
            return 2*phi
        else:
            return 2*phi - 1.0

    def pl_rel(self, phi):
        sl_lift = np.array([0.0, 0.0, self.h])
        sl      = np.array([self.dx, self.dy, 0.0])
        sr      = sl
        if (phi < 0.5):
            return self.Ol + sl_lift*self.tl_lift(phi) + sl*self.tl_move(phi) + sr*(1.0 - self.tl_move(phi))*self.tl(phi)
        else:
            return self.Ol + sl_lift*self.tl_lift(phi) + sl*(1.0 - self.tr(phi))
             
    def pr_rel(self, phi):
        sr_lift = np.array([0.0, 0.0, self.h])
        sl      = np.array([self.dx, self.dy, 0.0])
        sr      = sl 
        if (phi >= 0.5):
            return self.Or + sr_lift*self.tr_lift(phi) + sr*self.tr_move(phi) - sl*(1.0 - self.tr_move(phi))*self.tr(phi)
        else:
            return self.Or + sr_lift*self.tr_lift(phi) + sr*(1.0 - self.tl(phi))

    def pl_com(self, phi):
        sl      = np.array([self.dx, self.dy, 0.0])
        sr      = sl
        if (phi < 0.5):
            return - self.Cs + self.Ss*self.t_com(phi) + self.Ol - 0.5*(sl*self.t_lin(phi) - sr*(1.0 - self.t_lin(phi)))
        else:
            return self.pr_com(phi) - self.pr_rel(phi) + self.pl_rel(phi)

    def pr_com(self, phi):
        sl      = np.array([self.dx, self.dy, 0.0])
        sr      = sl
        if (phi >= 0.5):
            #return - self.Cs + self.Ss*self.t_com(phi) + self.Or - 0.5*(sr*self.t_lin(phi) - sl*(1.0 - self.t_lin(phi))) - np.array([0.0, 120.0, 0.0])
            return - self.Cs + self.Ss*self.t_com(phi) + self.Or - 0.5*(sr*self.t_lin(phi) - sl*(1.0 - self.t_lin(phi)))
        else:
            # return self.pl_com(phi) - self.pl_rel(phi) + self.pr_rel(phi) + np.array([0.0, 120.0, 0.0])
            return self.pl_com(phi) - self.pl_rel(phi) + self.pr_rel(phi)

    def set_phi(self, phi):
        if not self.first_half:
            phi = phi + 0.5

        self.current_phi      = phi
        if self.which_trajectory == "pl_com":
            self.current_position = self.pl_com(phi)
        elif self.which_trajectory == "pr_com":
            self.current_position = self.pr_com(phi)
        elif self.which_trajectory == "t_com":
            self.current_position = self.t_com(phi)

        '''
        Other trajectories to be added if required
        '''    

        # velocities and accelerations to be calculated later if required
                

    def plot_special(self, wtp = 'pl_com', axis = 0, n = 100):
        '''
        wtp (what to plot) is a string indicating which path to be plotted
        n specifies how many points need to be created in the path function domain
        '''
        if wtp in ['position', 'velocity', 'acceleration']:
            super(Colin_Graf_Trajectory, self).plot(wtp = wtp, n = n, axis = axis)
        else:
            x = np.arange(0.0, 1.0, 1.0/n)
            y = copy.copy(x)
            for i in range(n):
                if wtp == 'pl_com':
                    y[i] = self.pl_com(x[i])[axis]
                elif wtp == 'pr_com':
                    y[i] = self.pr_com(x[i])[axis]
                elif wtp == 'pl_rel':
                    y[i] = self.pl_rel(x[i])[axis]
                elif wtp == 'pr_rel':
                    y[i] = self.pr_rel(x[i])[axis]
                elif wtp == 'tl_lift':
                    y[i] = self.tl_lift(x[i])
                elif wtp == 'tr_lift':
                    y[i] = self.tr_lift(x[i])
                elif wtp == 't_com':
                    y[i] = self.t_com(x[i])
                elif wtp == 'tl_move':
                    y[i] = self.tl_move(x[i])
                elif wtp == 'tr_move':
                    y[i] = self.tr_move(x[i])
                elif wtp == 'tr':
                    y[i] = self.tr(x[i])
                elif wtp == 'tl':
                    y[i] = self.tl(x[i])
                
            plt.plot(x, y) 
            plt.ylabel(wtp)
            plt.xlabel('phi')
            plt.show()
                
def nima_walk_ts_traj_for_com_wrt_lft(foot_dist, step_length, com_height, side_move):
    L     = 0.5*step_length

    p1    = np.array([foot_dist/2, 0.0, com_height])  # start of walk
    p2    = np.array([foot_dist/2, L  , com_height])  # end of step right foot 

    v0    = np.zeros(3)
    v1    = np.array([0.0, L/1.3, 0.0])  # start of walk

    '''
    totally, COM slides by 2*L in time T
    '''    
    t1 = trajlib.Polynomial_Trajectory()
    t1.interpolate([0.0, 1.3]     , positions = [p1, p2])

    t1.traj[0] = trig.Fourier_Series(N = 2)
    t1.traj[0].B[0] = side_move
    t1.traj[0].A[1] = - side_move
    t1.traj[0].T    = 2.6
    
    return(t1)

def nima_walk_ts_traj_for_com_wrt_rft(foot_dist, step_length, com_height, side_move):
    L     = 0.5*step_length

    p1    = np.array([- foot_dist/2, 0.0, com_height])  # start of walk
    p2    = np.array([- foot_dist/2, L  , com_height])  # end of step right foot 

    v0    = np.zeros(3)
    v1    = np.array([0.0, L/1.3, 0.0])  # start of walk

    '''
    totally, COM slides by 2*L in time T
    '''    
    t1 = trajlib.Polynomial_Trajectory()
    t1.interpolate([0.0, 1.3]     , positions = [p1, p2])

    t1.traj[0] = trig.Fourier_Series(N = 2)
    t1.traj[0].B[0] = - side_move
    t1.traj[0].A[1] = side_move
    t1.traj[0].T    = 2.6

    return(t1)


