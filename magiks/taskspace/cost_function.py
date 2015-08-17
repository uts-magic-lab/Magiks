
# HEADER
'''   
@file:          cost_function.py
@brief:    	    This module provides a class representing a cost function which can be used for a criterion in the jointspace or taskspace
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
Last Revision:  09 August 2015
'''
# BODY

all_purposes = [None, 'Main Task', 'Gradient Projection', 'Joint Damping']
all_input_refs      = ['Joint Values', 'Taskpoint Error', 'Taskframe Error']

import numpy

class Cost_Function(object):
    '''
    '''
    def __init__(self, input_ref, purpose = None, weight = 1.0):
        '''
        '''
        assert input_ref in all_input_refs
        assert purpose in all_purposes
        self.input_ref = input_ref
        self.purpose   = purpose
        self.weight    = weight
        self.function  = None 
        self.ref_num   = 0 

        self.abs_grad  = None    

    def value(self, endeff):
        if self.input_ref == 'Joint_Values':
            x = endeff.free_config(endeff.q)
        elif self.input_ref == 'Taskpoint Error':
            tp = endeff.task_point[self.ref_num]
            H  = endeff.transfer_matrices()
            x  = tp.error.value(tp.position(H), tp.rd)
        else:
            assert False, "Not Supported!"
        return self.function.value(x)

    def gradient(self, endeff):
        if self.input_ref == 'Joint Values':
            x = endeff.free_config(endeff.q)
            return self.function.gradient(x)
        elif self.input_ref == 'Taskpoint Error':
            tp = endeff.task_point[self.ref_num]
            H  = endeff.transfer_matrices()
            x  = tp.error.value(tp.position(H), tp.rd)
            EJ = tp.error_jacobian(H, endeff.ajac)
            return numpy.dot(EJ.T, self.function.gradient(x))
        elif self.input_ref == 'Taskframe Error':
            tf = endeff.task_frame[self.ref_num]
            H  = endeff.transfer_matrices()
            x  = tf.error.value(tf.position(H), tf.rd)
            EJ = tf.error_jacobian(H, endeff.ajac)
            return numpy.dot(EJ.T, self.function.gradient(x))
        else:
            assert False, "Not Supported!"
        
