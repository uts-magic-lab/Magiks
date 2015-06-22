## @file        	polynomials.py
#  @brief     		This module provides everything you need to work with polynomials 
#                   including tools by which you can fit a curve through a set of points.
#  @author      	Nima Ramezani Taghiabadi 
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. :   04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	4.0
#
#  start date:    February 2011 
#  Last Revision:  	03 January 2015 

silent = True

import math, numpy, sys
import general_python as genpy

class Point(object):
    def __init__(self, t = 0.0, x = 0.0, v = None, a = None):
        self.t = t
        self.x = x    
        self.v = v
        self.a = a

    def __str__( self ):
        s  = "Scalar point:"  + '\n'
        s += "t = " + str(self.t) + '\n'
        s += "x = " + str(self.x) + '\n'
        s += "v = " + str(self.v) + '\n'
        s += "a = " + str(self.a) + '\n'
        return s

    def count_constraints(self):
        nc = 0
        if self.x != None:
            nc = nc + 1
        if self.v != None:
            nc = nc + 1
        if self.a !=None:
            nc = nc + 1
        return nc    

class Polynomial(object):
    '''
    '''
    def __init__(self, degree = 0):
        self.degree = degree
        self.coeff  = numpy.zeros(self.degree + 1)

    def interpolate_smart(self, points):
        n = len(points)
        m = 0
        for p in points:
            m = m + p.count_constraints()

        self.degree = m - 1        
        A = numpy.zeros((m,m))
        i = 0
        u = numpy.array([])
        for p in points:
            if p.x != None:
                A[i, 0] = 1
                for j in range(1, m):
                    A[i, j] = A[i, j-1]*p.t
                i = i + 1
                u = numpy.append(u, p.x)

            if p.v != None:
                A[i, 0] = 0.0
                A[i, 1] = 1.0
                for j in range(2, m):
                    A[i, j] = j*(p.t**(j-1))
                i = i + 1
                u = numpy.append(u, p.v)

            if p.a != None:
                A[i, 0:2] = 0.0
                A[i, 2]   = 2.0
                for j in range(3, m):
                    A[i, j] = j*(j-1)*(p.t**(j-2))
                i = i + 1
                u = numpy.append(u, p.a)

        try:
            self.coeff = numpy.dot(numpy.linalg.inv(A), u)
        except:
            print genpy.err_str(__name__, self.__class__.__name__, sys._getframe().f_code.co_name, 'The matrix of coefficients is singular')
            print "Matrix of coefficients:" 
            print A
            print 
            print "Determinent = ", numpy.linalg.det(A)
            print
            assert False 

    def interpolate(self, t, u, v = []):
        '''
        Generates the coefficients of a spline passing through key positions and velocities specified by u and v at corresponding t
        t, u, v must have the same length
        if v is not specified (keep the default), only positions are considered
        '''
        n  = len(t)
        nu = len(u)
        nv = len(v)
        assert n == nu

        if (nv == 0):
            self.degree = n - 1        
            A = numpy.zeros((n,n))
            for i in range(n):
                A[i, 0] = 1.0
                for j in range(1, n):
                    A[i, j] = A[i, j-1]*t[i]

            self.coeff = numpy.dot(numpy.linalg.inv(A), u)
        elif (nv == n):
            self.degree = 2*n - 1
            A = numpy.zeros((2*n, 2*n))
            for i in range(n):
                A[i,   0] = 1.0
                A[n+i, 0] = 0.0
                A[n+i, 1] = 1.0
                for j in range(1, 2*n):
                    A[i, j] = A[i, j-1]*t[i]
                    A[n + i, j] = j*(t[i]**(j-1))

            self.coeff = numpy.dot(numpy.linalg.inv(A), numpy.append(u, v))
        else:
            print "Error: Size of input vector v is different from t and u"

    def position(self, t):
        '''
        Returns the value of the polynomial at time t
        '''
        n = self.degree + 1
        s = self.coeff[0]
        for i in range(1,n):
            s = s + self.coeff[i]*(t**i)

        return(s)

    def velocity(self, t):
        '''
        Returns the derivative of the polynomial at time t
        '''
        n = self.degree
        if n < 1:
            v = 0.0
        else:
            v = self.coeff[1]
        for i in range(1, n):
            v = v + (i+1)*self.coeff[i+1]*(t**i)
        return(v)

    def acceleration(self, t):
        '''
        Returns the derivative of the polynomial at time t
        '''
        n = self.degree - 1
        if n < 2:
            a = 0.0
        else:
            a = 2*self.coeff[2]
        for i in range(1, n):
            a = a + (i+1)*(i+2)*self.coeff[i+2]*(t**i)
        return(a)

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
