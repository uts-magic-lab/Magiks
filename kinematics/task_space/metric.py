
# HEADER
'''   
@file:          metric.py
@brief:    	    This module provides a class representing the residual error between 
                the actual and desired endeffector poses including methods for calculating residual functions based on various conventions for 
                both position and orientation.
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

@version:	    0.3
Last Revision:  23 October 2012
'''

# BODY

import numpy, math

from packages.nima.mathematics import vectors_and_matrices as vecmat
from packages.nima.mathematics import quaternions
from packages.nima.mathematics import rotation

"""
class Metric_Settings():
    '''
    '''
    def __init__()
        '''
        '''
        
"""

class Metric:
    '''
    Metric includes everything regarding the error between two positions or orientations.
    '''    
    def __init__(self):
        '''    
        '''
        # "error.value" is a 3 X 1 vector which represents the value of the error function or error between actual and desired positions.
        # Elements of "value" can be calculated according to various formulation depending on the values of "W" property

        self.value = numpy.zeros((3))
        self.rate  = numpy.zeros((3))

        # property "in_target" is True when the actual and desired task points are fully or partially identical according to the defined weighting matrix and power array
        self.in_target = False

        '''
        Property "power" determines the power of elements of basis error in the final vector of errors:
        Please refer to the comments of property: "W" in this file.

            0: Not considered for the corresponding coordinate
            1:  rd - ra
            2: (rd - ra)^2
            3: (rd - ra)^3
            .
            .
            .
            
         where:
            "ra" is the actual position
            "rd" is the desired position

        and r can be replaced by: X, Y or Z
        '''

        self.power = numpy.array([1, 1, 1])
            
        '''
        property "W" is a 3 X 3 Weighting matrix. This matrix will be multiplied by the basis error and basis jacobian.  
    
        Matrix W is by default the identity matrix defining 3 constraints as:

            Xd - X = 0
            Yd - Y = 0
            Zd - Z = 0

        Example 1:
    
        "power" array = [1, 1, 1] and "W" is a (3 X 3) matrix
    
                                      X   Y   Z

        1st Row of "W":               1   0   0       (Xd - Xa) = 0
        2nd Row of "W":               0   1   0       (Yd - Ya) = 0
        3rd Row of "W":               0   0   1       (Zd - Za) = 0

        Example 2:
        
        "power" array = [2, 2, 1] and "W" is a (2 X 3) matrix
        
                                    X   Y   Z
    
        1st Row of "W":             1   1   0       (Xd - Xa)^2 + (Yd - Ya)^2 = 0
        2nd Row of "W":             0   0   1       (Zd - Za) = 0
    
        Example 3: (When only two coordinations "X" and "Z" are important to be identical)
        In this example, it is important to also set the property "required_identical_coordinate" to: [True, False, True] 
        Please refer to the of comments for property "required_identical_coordinate".

        "power" array = [1, 0, 1] and "W" is a (2 X 3) matrix
        
                                      X   Y   Z
    
        1st Row of "W":               1   0   0       (Xd - Xa) = 0
        2nd Row of "W":               0   0   1       (Zd - Za) = 0
    
        the same for the orientation ...    
            
        '''
        
        self.W = numpy.eye(3)
        
        '''
        "self.required_identical_coordinate" is an array of booleans and indicates which coordinates (x, y ,z) should be considered
        as a criteria in determining the confirmity of the actual and desired positions and orientations.
        For example if "required_identical_coordinate" = [True, False, True] the actual and desired positions are considered as identical only when 
        their  "x" and "z" coordinates are equal.
        (the "y" coordinate will be neglected)
        Please refer to "example 3" of the comments provided for property: "W" in this file.
        '''
        self.required_identical_coordinate = [True, True, True]
          
    
    def basis_error(self, current, target ) : 
        '''
        return the distance between current and target 
    
        excluding weighting stuff         
        abstract method 
        '''
        return None 
    
    
    def update(self, current, target ) : 
        '''       
        change the "weighted distance" of current and target; 
        save it in the class property "value" 
        
        abstract method 
        '''
        return None 

class Position_Metric(Metric):
    '''
    includes everything regarding the error between two positions
    '''
    
    def __init__(self, basis_error_function = 'differential_cartesian_coordinates'):
        '''
        '''
        Metric.__init__(self)
        #set "differential_cartesian_coordinates" as basis error function for position error        
        self.basis_error_function = basis_error_function
        '''
        set "precision" or termination criteria by default as: 2 cm. It means actual and desired positions are considered "identical" 
        if for each position coordinate (x, y, z), the absolute value of the difference of current and desired, does not exceed: 2 cm.
        '''
        self.precision            = 0.02
        self.precision_base       = "Coordinate Difference"
        # Another value is "Error Function"

        # define the default weighting matrix as Identity. It considers all three position coordinates in the error function
        self.W                    = numpy.eye(3)
        self.C                    = numpy.zeros((3))

        # define the default power array. (Please refer to the documentation)
        self.power                = numpy.array([1, 1, 1])
        

    def basis_error(self, current, target ) : 
        # def basis_position_error(self, current, target):
        '''
        return the value of basis_position_error
        '''
        if self.basis_error_function == 'differential_cartesian_coordinates':
            p = 3
            assert len(self.power) == p
            f = numpy.zeros((p))
            for k in range (0,p):
                if self.power[k] == 0:
                    err = 0
                    f[k] = 1
                else:
                    err = current[k] - target[k]
                    
                    if self.power[k] == 1:
                        f[k] = err
                    else:            
                        f[k] = err**self.power[k]

        elif self.basis_error_function == 'differential_cylindrical_coordinates':
            p = 3
            assert len(self.power) == p
            f = numpy.zeros((p))
            assert False
        elif self.basis_error_function == 'differential_spherical_coordinates':
            p = 3
            assert len(self.power) == p
            f = numpy.zeros((p))
            assert False
        else:
            print 'Wrong basis error function: ' + self.basis_error_function
            assert False

        return f

    """
    def basis_error_rate(self, x, xd, x_dot, xd_dot) : 
        '''
        return the rate of basis_position_error
        '''
        if self.basis_error_function == 'differential_cartesian_coordinates':
            p = 3
            assert len(self.power) == p
            f_dot = numpy.zeros((p))
            for k in range (0,p):
                if self.power[k] == 0:
                    f_dot[k] = 0
                else:
                    if self.power[k] == 1:
                        f[k] = x_dot[k] - xd_dot[k]
                    else:            
                        err  = x[k] - xd[k]
                        f[k] = self.power[k]*(err**(self.power[k] - 1))*(x_dot[k] - xd_dot[k])

        elif self.basis_error_function == 'differential_cylindrical_coordinates':
            p = 3
            assert len(self.power) == p
            f_dot = numpy.zeros((p))
            assert False
        elif self.basis_error_function == 'differential_spherical_coordinates':
            p = 3
            assert len(self.power) == p
            f_dot = numpy.zeros((p))
            assert False
        else:
            print 'Wrong basis error function: ' + self.basis_error_function
            assert False

        return f_dot

    def update_rate(self, x, xd, x_dot, xd_dot):
        '''
        Calculates the error rate vector between the actual and desired positions of the corresponding taskpoint
        '''
        ber = self.basis_error(x, xd, x_dot, xd_dot)
        assert self.W.shape[1] == len(ber)
        self.rate = numpy.dot(self.W, ber)
    """
        
    def update(self, current, target ) : 
        # def update_for_position(self,actual_position, target):
        '''
        Calculates the error vector between the actual and desired positions of the corresponding taskpoint
        '''
        be = self.basis_error(current, target)
        assert self.W.shape[1] == len(be)
        self.value = numpy.dot(self.W, be)
        self.in_target = True
        for k in range(0,3):
            if self.required_identical_coordinate[k]:
                deviation = abs(target[k] - current[k])
                self.in_target = self.in_target and (deviation < self.precision) 


class Orientation_Metric(Metric):
    '''
    includes everything regarding the error between two orientations
    '''
    
    def __init__(self, basis_error_function = 'Axis Inner Product'):
        '''
        '''     
        Metric.__init__(self)
        #set "Axis Inner Product" as basis error function for position error        
        self.basis_error_function = basis_error_function
        self.precision_base = 'Axis Angle'
        if self.basis_error_function == 'Axis Inner Product':
            # define the default weighting matrix as Identity. It considers all three axis in the error function
            self.W                    = numpy.eye(3)
            self.C                    = numpy.array([-1.0, -1.0, -1.0])

            # define the default power array. (Please refer to the documentation)
            self.P   = numpy.array([1, 1, 1])
        else:
            assert False   
        
        '''
        set "precision" or termination criteria by default as: 2.0 degrees. It means actual and desired orientations are considered "identical" 
        if for each frame axis (i, j, k), the absolute value of the angle between current and desired frames, does not exceed: 2.0 degrees.
        '''
        self.precision = 2.0


    def basis_error(self, current, target ) : 
        # def basis_orientation_error(self, current, target):
        '''
        '''
        if self.basis_error_function == 'Axis Inner Product':
            p = 3
            assert len(self.power) == p
            f = numpy.zeros((p))
            for k in range (0,p):
                if self.power[k] == 0:
                    err = 0
                    f[k] = 1
                else:
                    err = numpy.dot(vecmat.uvect(target, k), vecmat.uvect(current, k))
                    if self.power[k] == 1 :
                        f[k] = err
                    else:            
                        f[k] = err**self.power[k]

        elif self.basis_error_function == 'relative_rotation_matrix_trace_minus_three':
            '''
            represents the orientation error by calculating the trace of relative rotation matrix minus three: Trace(R_a * R_d^T) - 3
            '''
            p = 1
            assert len(self.power) == p
            f = numpy.zeros((p))

            z = numpy.trace(numpy.dot(current, target.T)) - 3
            
            f[0] = z**self.power[0]
            
        elif self.basis_error_function == 'differential_quaternions':
            p = 3
            assert len(self.power) == p
            f = numpy.zeros((p))
            z = numpy.zeros((p))
            qn  = quaternions.unit_quaternion(current)
            
            '''
            Remember that the following code line is implemented many times resulting the same value, so it is better to be defined once
            '''
            qnd = quaternions.unit_quaternion(target)
            
            for k in range (0,p):
                z[k] = - qnd[p]*qn[k] + qn[p]*qnd[k]    
                f[k] = z[k]**self.power[k]

        elif self.basis_error_function == 'relative_rotation_angle':
            p = 1
            assert len(self.power) == p
            f = numpy.zeros((p))

            z = rotation.relative_rotation_angle(current,target)
            
            f[0] = z**self.power[0]

        elif self.basis_error_function == 'relative_rotation_vector_identity':
            p = 3
            assert len(self.power) == p
            f = numpy.zeros((p))
            z = rotation.relative_rotation_vector(current,target,'vectorial_identity')
            for k in range (0,p):
                f[k] = z[k]**self.power[k]

        elif self.basis_error_function == 'relative_rotation_vector_Cayley_Gibbs_Rodrigues':
            p = 3
            assert len(self.power) == p
            f = numpy.zeros((p))

            RRM = numpy.dot(target,current.T)
            z = mathlib.rotation_vector(RRM,mathlib.param_dict['pm_Cayley_Gibbs_Rodrigues'])

            for k in range (0,p):
                f[k] = z[k]**self.power[k]

        elif self.basis_error_function == 'relative_rotation_vector_linear':
            p = 3
            assert len(self.power) == p
            f = numpy.zeros((p))
            z = mathlib.relative_rotation_vector(current,target,mathlib.param_dict['pm_vectorial_linear'])
            
            for k in range (0,p):
                f[k] = z[k]**self.power[k]

        elif self.basis_error_function == 'differential_rotation_matrix':
            p = 9
            assert len(self.power) == p
            f = numpy.zeros((p))
            
            k = 0
            for i in range(0,3):
                for j in range(0,3):
                    err = target[i,j] - current[i,j]
                    f[k] = err**self.power[k]
                    k = k + 1

        else:
            assert False

        return f
    """
    def basis_error_rate(self, R, Rd, R_dot, Rd_dot ) : 
        '''
        '''
        if self.basis_error_function == 'Axis Inner Product':
            p = 3
            assert len(self.power) == p
            f_dot = numpy.zeros((p))
            for k in range (0,p):
                if self.power[k] == 0:
                    f_dot[k] = 0
                else:
                    err_rate = numpy.dot(vecmat.uvect(Rd_dot, k), vecmat.uvect(R, k)) + numpy.dot(vecmat.uvect(Rd, k), vecmat.uvect(R_dot, k))
                    if self.power[k] == 1 :
                        f_dot[k] = err_rate
                    else:            
                        err = numpy.dot(vecmat.uvect(Rd, k), vecmat.uvect(R, k))
                        f_dot[k] = self.power[k]*err_rate*(err**(self.power[k]-1))

        else:
            assert False

        return f_dot

    """
    def update(self, current, target ) : 
        '''
        Calculates the error vector between the actual and desired orientations of the corresponding taskframe
        '''
        be = self.basis_error(current, target)

        assert self.W.shape[1] == len(be)
        
        self.value = numpy.dot(self.W, be) + self.C

        if self.precision_base == 'Axis Angle':
        
            self.in_target = True
            for k in range(0,3):
                if self.required_identical_coordinate[k]:
                    cos_teta = numpy.dot(vecmat.uvect(target,k),vecmat.uvect(current,k))
                    if (cos_teta < 1.00000) and (cos_teta > -1.00000):
                        deviation = abs((180.0/math.pi)*math.acos(cos_teta))
                        self.in_target = self.in_target and (deviation < self.precision) 

                    self.in_target =  self.in_target and (cos_teta > -1.00000)
        elif self.precision_base == 'Error Function':
            self.in_target = (numpy.linalg.norm(self.value) < self.precision)
        else:
            assert False

    """
    def update_rate(self, R, Rd, R_dot, Rd_dot ) : 
        '''
        Calculates the error rate vector between the actual and desired orientations of the corresponding taskframe
        '''
        ber = self.basis_error(R, Rd, R_dot, Rd_dot)
        assert self.W.shape[1] == len(ber)
        self.rate = numpy.dot(self.W, ber)
    """


class Pose_Metric(Metric):
    '''
    (outlook)
    '''
        
    def __init__(self):
        '''
        '''
        self.orientation_metric = Orientation_Metric() 
        self.position_metric = Position_Metric() 
        
        Metric.__init__(self)

    
    def basis_error(self, current, target ) : 
        '''
        '''
        self.orientation_metric.basis_error() 
        
        
        self.position_metric.basis_error() 
        
    
    
    def update(self, current, target ) : 
        '''
        '''
        self.orientation_metric.update() 
        self.position_metric.update() 
        

