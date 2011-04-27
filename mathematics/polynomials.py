'''   Header
@file:          interpolatepy.py
@brief:    	    This module provides tools by which you can fit a curve through a set of points.
@author:        Nima Ramezani; DFKI Bremen
@start date:    February 2011
@version:	    1.1
Last Revision:  28 April 2011
'''


import math, numpy

class Polynomial1_Interpolator:
    '''
    A polynomial of degree three in the form:
    
    f(t) = a * t + b

    the coefficients are: a, b
    
    each coefficient can be n-element vector or matrix (or a multi-dimensional numpy array) 
    
    '''
    
    def __init__(self):
        self.a = 0.0
        self.b = 0.0

    def find_coefficients(self, total_time, start_position, end_position,start_velocity, end_velocity):
        '''
        returns two coefficients of a linear polynomial which generates position as a function of time according to the given boundary conditions as
        the coefficients are: a, b
        property "total_time" indicates the total time of motion
        each coefficient is a n-element vector (numpy array) 
    
        f(t) = a * t + b

        the output of the polynomial has the same structure of the coefficients
    
        '''

        n = len(start_position)

        B = numpy.zeros(2)

        assert len(end_position) == n

        self.b = start_position
        self.a = (end_position - start_position) / total_time

    def interpolated_position(self, t):
        '''
        return "f(t)" at time: "t"
        where:
    
        f(t) = a * t + b
    
        coefficients a, b can be any vector or multi-dimensional numpy array
        the output has the same structure of the coefficients
    
        '''
        
        pos = self.a*t + self.b

        return pos

class Polynomial2_Interpolator:
    '''
    A polynomial of degree three in the form:
    
    f(t) = a * t^2 + b * t + c

    the coefficients are: a, b, c
    
    each coefficient can be n-element vector or matrix (or a multi-dimensional numpy array) 
    
    '''
    
    def __init__(self):
        self.a = 0.0
        self.b = 0.0
        self.c = 0.0

    def find_coefficients(self, total_time, start_position, end_position,start_velocity, end_velocity):
        '''
        returns two coefficients of a linear polynomial which generates position as a function of time according to the given boundary conditions as
        the coefficients are: a, b, c
        property "total_time" indicates the total time of motion
        each coefficient is a n-element vector (numpy array) 
    
        f(t) = a * t^2 + b * t + c

        the output of the polynomial has the same structure of the coefficients
    
        '''

        n = len(start_position)

        B = numpy.zeros(2)

        assert len(end_position) == n

        self.c = numpy.copy(start_position)
        self.b = numpy.copy(start_velocity)
        self.a = (end_position - self.c - self.b*total_time)/(total_time**2)

    def interpolated_position(self, t):
        '''
        return "f(t)" at time: "t"
        where:
    
        f(t) = a * t^2 + b * t + c
    
        coefficients a, b can be any vector or multi-dimensional numpy array
        the output has the same structure of the coefficients
    
        '''
        
        pos = self.a*(t**2) + self.b*t + self.c

        return pos


class Polynomial3_Interpolator:
    '''
    A polynomial of degree three in the form:
    
    f(t) = a * t^3 + b * t^2 + c * t + d 

    the coefficients are: a, b, c, d
    
    each coefficient can be n-element vector or matrix (or a multi-dimensional numpy array) 
    
    '''
    
    def __init__(self):
        self.a = 0.0
        self.b = 0.0
        self.c = 0.0
        self.d = 0.0

    def find_coefficients(self, total_time, start_position, end_position, start_velocity, end_velocity):
        '''
        returns four coefficients of a polynomial of third degree which generates position as a function of time according to the given boundary conditions as
        the coefficients are: a, b, c, d
        property "total_time" indicates the total time of motion
        each coefficient is a n-element vector (numpy array) 
    
        f(t) = a * t^3 + b * t^2 + c * t + d 

        the output of the polynomial has the same structure of the coefficients
    
        '''

        n = len(start_position)

        B = numpy.zeros(2)

        assert len(end_position) == n
        assert len(start_velocity) == n
        assert len(end_velocity) == n

        self.d = start_position
        self.c = start_velocity

        self.a = numpy.zeros(n)
        self.b = numpy.zeros(n)
    
        t2 = total_time ** 2
    
        T = numpy.array([[total_time*t2, t2],[3*t2, 2*total_time]])
        Tinv = numpy.linalg.inv(T)
    
        for j in range(0,n):

            B[0] = end_position[j] - self.c[j]*total_time - self.d[j]
            B[1] = end_velocity[j] - self.c[j]
    
            X = numpy.dot(Tinv,B)
        
            self.a[j] = X[0]
            self.b[j] = X[1]


    def interpolated_position(self, t):
        '''
        return "f(t)" at time: "t"
        where:
    
        f(t) = a * t^3 + b * t^2 + c * t + d 
    
        coefficients a, b, c, d are given via "coeff" and can be any vector or multi-dimensional numpy array
        the output has the same structure of the coefficients
    
        '''
        t2 = t*t
        pos = self.a*t*t2 + self.b*t2 + self.c*t + self.d 

        return pos

def interpolated_velocity(self, t):
    '''
    return the derivateve of "f(t)" at time: "t"
    where:
    
    f(t)  = a * t^3 + b * t^2 + c * t + d 
    f'(t) = 3 * a * t^2 + 2 * b * t + c
    
    coefficients a, b, c, d are given via "coeff" and can be any vector or multi-dimensional numpy array
    the output has the same structure of the coefficients. (Obviously coefficient "d" is not used)
    '''

    vel = 3*self.a*t*t + 2*self.b*t + self.c

    return vel

"""
def interpolate_polynomial(A,X,Y):
	
	'''
    #This function is not complete#
    
	A : numpy array of n+1 elements
	X : numpy array of  m  elements
	Y : numpy array of  m  elements
	
	
	This function assigns coefficients a_0 to a_n so that polynomial defined as:
	
	f(x) = a_0 + a_1*x + a_2*x^2 + a_3*x^3 + ... + a_n*x^n 
	
	fits through points (x_1,y_1) , (x_2,y_2) , ... , (x_m,y_m)
	
	The procedures follows three different algorythms in three different possible states:
	
	1) m > n :
	
	   It is not possible to define n degree polynomial to exactly pass through m points, 
	   but it is possible to calculate coefficients of n degree polynomial with a minimum 
	   sum of squares of distances between the function and the given points. 
	   
	   Cost function:
	   
	     J = Sigma_{i=0}^{m}{(y_i-f(x_i))^2}
	   
	   For example:
	   
	   A very special case is n=1 when this function returns coefficients of regression line  
	 
	 2) m = n :
	    
	    The problem has only one solution. Only one n degree polynomial can be defined passing exactly through 
	    the given points. This function returns the coefficients of this function.
	    
	 3) m < n :
	 
	    Unlimited number of functions can be found to pass exactly through the given points. This 
	    function selects only one of these solutions so that sum of squares of its coefficients is minimized.
	    
	    Cost function :
	    
	    J = Sigma_{i=0}^{n}{a_i^2}
	    
	    
	'''
	m = X.size
	n = A.size
	
	print "m = ", m
	print "n = ", n
	
"""
