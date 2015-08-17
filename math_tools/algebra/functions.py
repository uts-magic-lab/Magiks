'''   
@file:          functions.py
@brief:    	    This module provides an abstract class representing a general mathematical function
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
@version:	    1.0
Last Revision:  10 August 2015
'''

import numpy as np, math

class Function(object):
    def __init__(self, name = 'Identity'):
        name = name
    
    def value(self, x):
        return x

    def gradient(self, x):
        return 1.0

    def hessian(self, x):
        return 0.0
