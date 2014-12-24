'''   Header
@file:          NAO_kinematics.py
@brief:    	    Contains specific functions that define all geometric and kinematic parameters for nao robot

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
                Email(3): nima_ramezani@yahoo.com
                Email(4): ramezanitn@alum.sharif.edu
@version:	    2.0
Last Revision:  26 May 2014

Changes from previous version:
After forward kinematics was finalized, 

'''
import packages.nima.mathematics.general as gen

import numpy, math
import packages.nima.mathematics.general as genlib
import packages.nima.mathematics.trigonometry as triglib
import packages.nima.mathematics.rotation as rotlib
import packages.nima.mathematics.vectors_and_matrices as vecmatlib

drc     = math.pi/180.00

class NAO():
    '''
    '''
    def __init__(self, tibia_length = 100.0, thigh_length = 120.0, hip_offset = [0.0, 0.0, 0.0], foot_offset = [0.0, 0.0, 0.0]):
        '''
        The defaults are extracted from NAO model ... :
        link
        '''
        self.tibia_length = tibia_length
        self.thigh_length = thigh_length
        self.foot_offset  = foot_offset
        self.hip_offset   = hip_offset # Specifies the offset of hip JC(Joint Center) w.r.t. the torso in torso CS(Coordinate System)

        self.q            = numpy.zeros(11) # Now, we have considered only the leg joints, we should add arm and head joints as well
        
    def pos_rfoot(self):
        '''
        FK function. Returns the position of the right foot w.r.t. the torso
        '''
        a4 = self.tibia_length
        a3 = self.thigh_length
        h  = self.hip_offset[1]

        c0 = math.cos(self.q[0])
        c1 = math.cos(self.q[1] - math.pi/4)
        c2 = math.cos(self.q[2])
        c3 = math.cos(self.q[3])

        s0 = math.sin(self.q[0])
        s1 = math.sin(self.q[1] - math.pi/4)
        s2 = math.sin(self.q[2])
        s3 = math.sin(self.q[3])

        u = math.cos(math.pi/4)

        xr = a3*(c0*s2 - c1*c2*s0) + a4*(c3*(c0*s2 - c1*c2*s0) + s3*(c0*c2 + c1*s0*s2))
        yr = a3*(c2*(-c0*c1 - s1) - s0*s2) + a4*(c3*(-c0*c1*c2 - s1*c2 - s0*s2) - s3*(c2*s0 + -c0*c1*s2 - s1*s2))
        zr = a3*(-c0*c1*c2 + s1*c2- s0*s2) + a4*(-c0*c1*c2*c3 + s1*c2*c3- s0*s2*c3 - c2*s0*s3 - -c0*c1*s2*s3 - s1*s2*s3 )
        px = xr
        py = - h - u*yr
        pz = u*zr

        return numpy.array([px, py, pz])
    
    def pos_lfoot(self):
        '''
        FK function. Returns the position of the right foot w.r.t. the torso
        '''
        a4 = self.tibia_length
        a3 = self.thigh_length
        h  = self.hip_offset[1]

        c0  = math.cos(self.q[0])
        c6  = math.cos(self.q[6] - math.pi/4)
        c7  = math.cos(self.q[7])
        c8  = math.cos(self.q[8])

        s0  = math.sin(self.q[0])
        s6  = math.sin(self.q[6] - math.pi/4)
        s7  = math.sin(self.q[7])
        s8  = math.sin(self.q[8])

        u = math.cos(math.pi/4)

        xl = a3*(c0*s7 - c6*c7*s0) + a4*(c8*(c0*s7 - c6*c7*s0) + s8*(c0*c7 + c6*s0*s7))
        yl = a3*(c7*(-c0*c6 - s6) - s0*s7) + a4*(c8*(-c0*c6*c7 - s6*c7 - s0*s7) - s8*(c7*s0 + -c0*c6*s7 - s6*s7))
        zl = a3*(-c0*c6*c7 + s6*c7- s0*s7) + a4*(-c0*c6*c7*c8 + s6*c7*c8- s0*s7*c8 - c7*s0*s8 - -c0*c6*s7*s8 - s6*s7*s8 )
        px = xl
        py = h + u*yl
        pz = u*zl

        return numpy.array([px, py, pz])

    


