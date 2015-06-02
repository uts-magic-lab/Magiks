## @file        	discrete.py
#  @brief     		This module provides some functions regarding discrete mathematics
#  @author      	Nima Ramezani Taghiabadi 
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. :   04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	1.0
# 
#  Start date:      Februaty 2010
#  Last Revision:  	03 January 2015 

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
