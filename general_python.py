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

def none_repeat(x, n):
    if x == None:
        x = [None for i in range(n)]
    else:
        assert len(x) == n, err_str(__name__, '', 'none_repeat', 'Argument x must have ' + str(n) + ' elements')
    return x

def check_type(var, type_list, file_path='', class_name='', function_name='', var_name='', array_length = None, shape_length = None, shape = None, default = None):
    if var == None:
        var = default
    assert type(var) in type_list, err_str(file_path, class_name, function_name, 'Argument '+ var_name + ' must be ' + str(type_list))
    if array_length != None:
        if type(array_length) == int:
            assert len(var) == array_length, err_str(file_path, class_name, function_name, 'Argument '+ var_name + ' must have ' + str(array_length) + ' elements')
        elif type(array_length) == list:
            assert (len(var) in array_length), err_str(file_path, class_name, function_name, 'Number of elements of argument '+ var_name + ' must be one of these: \n' + str(array_length))
        else: 
            assert False, err_str(__name__, '', sys._getframe().f_code.co_name, 'Argument array_length must be either integer or a list of integers')
    if (shape_length != None) and (type(var) == np.ndarray):
        assert len(var.shape) == shape_length, err_str(file_path, class_name, function_name, 'Argument '+ var_name + ' must have dimension ' + str(shape_length))
    if (shape != None) and (type(var) == np.ndarray):
        assert var.shape == shape, err_str(file_path, class_name, function_name, 'Argument '+ var_name + ' must have shape ' + str(shape))
    return var

def check_types(variables, type_lists, file_path='', class_name='', function_name='', var_names=None, array_lengths = None, shape_lengths = None, shapes = None, defaults = None):
    '''
    check_type(var, [list, tuple], file_path = __name__, function_name = sys._getframe().f_code.co_name, var_name = 'var', shape_length = 1)
    check_type(type_list, [list, tuple], file_path = __name__, function_name = sys._getframe().f_code.co_name, var_name = 'type_list', shape_length = 1)
    check_type(file_path, [str], file_path = __name__, function_name = sys._getframe().f_code.co_name, var_name = 'file_path')
    check_type(class_name, [str], file_path = __name__, function_name = sys._getframe().f_code.co_name, var_name = 'class_name')
    '''
    n = len(variables)
    type_lists    = none_repeat(type_lists, n)
    var_names     = none_repeat(var_names, n)
    array_lengths = none_repeat(array_lengths, n)
    shapes        = none_repeat(shapes, n)
    defaults      = none_repeat(defaults, n)

    varlist = []
    for i in range(n):
        varlist.append(check_type(variables[i], type_lists[i], file_path, class_name, function_name, var_name = var_names[i], array_length = array_lengths[i], shape_length = shape_lengths[i], shape = shapes[i], default = defaults[i]))
    return varlist

def check_valid(var, valid_list, file_path, class_name, function_name, var_name):
    assert var in valid_list, err_str(file_path, class_name, function_name, str(var) + " is an invalid value for " + var_name)

def check_range(var, lower_bound = None, higher_bound = None, file_path='', class_name='', function_name='', var_name='', equality_valid = True):
    if lower_bound != None:
        if equality_valid:
            permit = var >= lower_bound
            eqstr  = 'or equal to '
        else:
            permit = var > lower_bound
            eqstr  = ''
    else:
        permit = True
    
    assert permit, err_str(file_path, class_name, function_name, 'Argument ' + var_name + ' must be higher than ' + eqstr + str(lower_bound))

    if higher_bound != None:
        if equality_valid:
            permit = var <= higher_bound
        else:
            permit = var < higher_bound
    else:
        permit = True
    
    assert permit, err_str(file_path, class_name, function_name, 'Argument ' + var_name + ' must be lower than ' + eqstr + str(higher_bound))

## Returns the most common element in the given list
def most_common(lst):
    return max(set(lst), key=lst.count)

def show(message, silent = False):
    if not silent:
        print message
