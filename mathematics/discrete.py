'''   Header
@file:          discrete.py
@brief:    	    This module provides some functions regarding descrete mathematics.
@author:        Nima Ramezani; DFKI Bremen
@start date:    February 2010
@version:	    0.1
Last Revision:  31 March 2011
'''

import math, numpy

def increase(limits_array,values_array):
    '''
    Increases the value of one element of the given "values_array" by one unit. The elements change in order of their position and switch 
    to next element when the value reaches its equivalent in limits_array
    '''
    n = len(limits_array)
    va = values_array
    assert n == len(values_array)
    notoverflow = True
    for i in range(0,n):
        if va[i] > limits_array[i]:
            va[i] = limits_array[i]

    i = 0

    while notoverflow and i < n :
        notoverflow = (va[i] < limits_array[i])
        if notoverflow:
            va[i] = va[i] + 1
            notoverflow = False
        else:
            va[i] = 0
            i = i + 1
            notoverflow = True
    return va     

def number_in_base(N, base, n_digits):
    '''
    transform number "N" into base representation to base: "base"
    the output is an array of "n_digits" elements
    '''
    assert base != 0
    A = numpy.zeros((n_digits))
    assert N < base**n_digits
    M = N       
    i = 0 
    while M > 0 :
        A[i] = M % base    
        M = M / base
        i = i + 1
    return A    
