'''   Header
@file:          general.py
@brief:    	    Contains some general functions

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
Last Revision:  09 September 2014

'''

import numpy as np

def show(s, value, silent):
    if not silent:
        print s, value    

def replace_if_none(a, b):
    if a == None:
        return b
    else:
        return a

def err_str(file_path, class_name, function_name, err_message):
    s  = '\n \n' + 'Error from: ' + '\n \n'

    s += file_path

    if class_name != '':
        s += '.' + class_name

    if function_name != '':
        s += '.' + function_name            

    if (class_name != '') or (function_name != ''):
        s += '()'

    s += ': ' + '\n \n' + err_message + '\n'
    return  s

def check_type(var, type_list, file_path='', class_name='', function_name='', var_name='', array_length = None, shape_length = None, shape = None, default = None):
    if var == None:
        var = default
    assert type(var) in type_list, err_str(file_path, class_name, function_name, 'Argument '+ var_name + ' must be ' + str(type_list))
    if array_length != None:
        assert len(var) == array_length, err_str(file_path, class_name, function_name, 'Argument '+ var_name + ' must have ' + str(array_length) + ' elements')
    if (shape_length != None) and (type(var) == np.ndarray):
        assert len(var.shape) == shape_length, err_str(file_path, class_name, function_name, 'Argument '+ var_name + ' must have dimension ' + str(shape_length))
    if (shape != None) and (type(var) == np.ndarray):
        assert var.shape == shape, err_str(file_path, class_name, function_name, 'Argument '+ var_name + ' must have shape ' + str(shape))
    return var

def check_valid(var, valid_list, file_path, class_name, function_name, var_name):
    assert var in valid_list, genpy.err_str(file_path, class_name, function_name, str(var) + " is an invalid value for " + var_name)


## Returns the most common element in the given list
def most_common(lst):
    return max(set(lst), key=lst.count)


