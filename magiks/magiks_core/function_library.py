
# HEADER
'''   
@file:          function_library.py
@brief:    	    This module provides a library of functions used in manipulator kinematic contrl
@author:        Nima Ramezani Taghiabadi
                PhD Researcher
                Faculty of Engineering and Information Technology
                University of Technology Sydney
                Broadway, Ultimo, NSW 2007
                Room No.: CB10.03.512
                Phone:    02 9514 4621
                Mobile:   04 5027 4611
                Email(1): Nima.RamezaniTaghiabadi@student.uts.edu.au 
                Email(2): nima.ramezani@gmail.com
                Email(3): nima_ramezani@yahoo.com
                Email(4): ramezanitn@alum.sharif.edu
@version:	    2.0
Last Revision:  23 August 2015
'''

'''
Changes from last version:
    1- a new task cost function added:  Liegeois_Midrange_Deviance() used for Jacobian Nullspace Gradient Projection to avoid joint limits as a second-priority cost function

'''
# BODY

from math_tools.algebra import functions as flib
import math, numpy as np

class Zghal_Function(flib.Function):
    '''
    '''
    def __init__(self, xl, xh):
        '''
        '''
        n         = len(xl)
        assert n == len(xh), "\n Error from Zghal_Function.__init__(): \n \n Given vectors do not match" 
        self.xl   = xl
        self.xh   = xh
        self.rng  = (xh - xl)**2

    def value(self, x):
        den = 4.0*(self.xh - x)*(x - self.xl)
        return sum(self.rng/den)
            
    def gradient(self, x):
        den  =   (self.xh - x)**2
        den *= 4*(x - self.xl)**2
        num  = 2*x - self.xh - self.xl
        num *= self.rng
        return num/den
