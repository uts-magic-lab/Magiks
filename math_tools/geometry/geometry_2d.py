## @file        	geometry_2d.py
#  @brief     		This module contains a comprehensive data structure with relevant method and properties
#                   for identities in two-dimensional geometry like a point, an ellipsoid and etc.
#  @author      	Nima Ramezani Taghiabadi 
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. :   04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	3.0
#
#  start date:      16 July 2015 
#  Last Revision:  	16 July 2015 

import math, sys, numpy as np, trigonometry as trig, rotation as rot, general_python as genpy

from math_tools.algebra import vectors_and_matrices as vm, quaternions as quat, optimization as opt
from math_tools import general_math as gen

def symbolic_term(a, x, plus_sign = True):
    if plus_sign and (a > 0):
        ps = '+ '
    else:
        ps = ''
    if x == '':
        star = ''
    else:
        star = '*'
    if gen.equal(a , 0.0):
        s = ''
    elif gen.equal(a , 1.0) and (star == '*'):
        s = ps + x + ' '
    elif gen.equal(a , - 1.0) and (star == '*'):
        s = '- ' + x + ' '
    else:
        s = ps + str(a) + star + x
    return s

## This class, introduces a structure for a point in the two-dimensional space. 
class Point_2D(object):
    
    ## Class Constructor
    #  @param pos The desired position vector at the key point
    #  @param vel The desired velocity vector at the key point
    #  @param acc The desired acceleration vector at the key point   
    def __init__(self, pos, vel = None, acc = None, representation = 'cartesian'):
       
       ## An integer indicating the dimension of space in which the kepy point is defined.
       #  This number specifies the number of elements of position, velocity and acceleration vectors  
       self.dim     = len(pos) 
       assert self.dim == 2
       self.__setitem__(representation, pos)
       self.set_velocity(vel, representation)
       self.set_acceleration(acc, representation)

    def clear(self):
       # A numpy vector indicating the position of the point in cartesian coordinates
       self.x = None

       # A numpy vector indicating the position of the point in polar coordinates
       self.p = None

    def clear_velocity(self):
       # A numpy vector indicating the velocity of the point in cartesian coordinates
       self.xd     = None

       # A numpy vector indicating the velocity of the point in polar coordinates
       self.pd     = None

    def clear_acceleration(self):
       # A numpy vector indicating the acceleration of the point in cartesian coordinates
       self.xdd    = None
       # A numpy vector indicating the acceleration of the point in polar coordinates
       self.pdd    = None

    def __getitem__(self, representation):
       if representation == 'cartesian':
           return self.cartesian()
       elif representation == 'cartesian_velocity':
           return self.cartesian_velocity()
       elif representation == 'cartesian_acceleration':     
           return self.cartesian_acceleration()
       elif representation == 'polar':
           return self.polar()
       elif representation == 'polar_velocity':
           return self.polar_velocity()
       elif representation == 'polar_acceleration':     
           return self.polar_acceleration()
       else:
           assert False, genpy.err_str(__name__ , self.__class__.__name__ , '__getitem__', representation + ' is not a valid value for representation')

    def is_none(self) :
       return (self.p == None) and (self.x == None)     

    def velocity_is_none(self) :
       return (self.pd == None) and (self.xd == None)     

    def acceleration_is_none(self) :
       return (self.pdd == None) and (self.xdd == None)      

    def set_velocity(self, value, representation): 
       self.clear_velocity() 
       if representation == 'cartesian':
           self.xd   = value
       elif representation == 'polar':
           self.pd   = value
       else:
           assert False, genpy.err_str(__name__ , self.__class__.__name__ , 'set_velocity', representation + ' is not a valid value for representation')

    def perpend_to(self, line):
        [x0, y0] = self.cartesian()
        if line.m == np.inf:
            return Line_2D(0.0, 1.0, - y0, representation = 'equation') 
        else:
            return Line_2D(1.0, line.m, - line.m*y0 - x0, representation = 'equation') 

    def set_acceleration(self, value, representation): 
       self.clear_acceleration() 
       if representation == 'cartesian':
           self.xdd   = value
       elif representation == 'polar':
           self.pdd   = value
       else:
           assert False, genpy.err_str(__name__ , self.__class__.__name__ , 'set_acceleration', representation + ' is not a valid value for representation')

    def __setitem__(self, representation, value):
        self.clear() 
        if representation == 'cartesian':
            self.x   = value
        elif representation == 'polar':
            '''
            p[0] = r
            p[1] = theta
            '''
            self.p   = value
        else:
            assert False, genpy.err_str(__name__ , self.__class__.__name__ , '__setitem__', representation + ' is not a valid value for representation')

    def cartesian(self):
        if self.x != None:
            return self.x
        elif self.p != None:
            self.x    = np.zeros(2)
            self.x[0] = self.p[0]*math.cos(self.p[1])
            self.x[1] = self.p[0]*math.sin(self.p[1])
            return self.x
        else:
            return None            

    def polar(self):
        if self.p == None:
            self.p    = np.zeros(2)
            [x, y] = self.cartesian()
            self.p[0] = math.sqrt(x*x + y*y)  # r
            self.p[1] = math.atan2(y, x)      # theta
        return self.p

    def dist(self, point):
        p1 = self.cartesian()
        p2 = point.cartesian()
        return np.linalg.norm(p1 - p2)

    ## This function is the string representation of the key point
    #  @param None
    #  @return A string representing all the information about the key point  
    def __str__( self ):
       s  = "Point Dimension: " + str(self.dim) + '\n' 

       if self.x != None: 
           s += "Position       : " + str(self.x) + '\n'
       if self.xd != None: 
           s += "Velocity       : " + str(self.xd) + '\n'
       if self.xdd != None: 
           s += "Acceleration   : " + str(self.xdd) + '\n'
       return s

    ## Use this function to get the current value of position, velocity or acceleration in a desired dimension 
    #  @param field_name A string, must be selected from 
    #                    set: ['position', 'velocity', 'acceleration'  
    #                    specifying which vector is desired.  
    #  @param axis A non-negative integer specifying which element of the vector should be returned. 
    #             (Must not be greater than the space dimension) 
    #  @return A float containing the value of the element specified by argument \b axis from the vector specified by argument \b field_name   
    def value(self, field_name = 'position', axis = 0):
        assert (axis <= self.dim), "Error from " + __name__ + func_name + ": Argument axis must not exceed the space dimension"
        assert (axis >= 0), "Error from " + __name__ + func_name + ": Argument axis can not have a negative value"  
        
        if field_name == 'position':
            return self.x[axis]
        elif field_name == 'velocity':
            return self.xd[axis]
        elif field_name == 'acceleration':
            return self.xdd[axis]
        else:
            print "Error from " + __name__ + func_name + ": " + field_name + " is not not a valid value for argument field_name"

    def __add__(p1, p2):
        return Point_2D(pos = p1.cartesian() + p2.cartesian())

    def __sub__(p1, p2):
        return Point_2D(pos = p1.cartesian() - p2.cartesian())

    def __neg__(p):
        return Point_2D(pos = - p.cartesian())

class Line_2D(object):
    def __init__(self, a = Point_2D(np.zeros(2)), b = Point_2D([1.0, 1.0]), c = None, representation = 'points'):
        '''
        Equation of the line in 2D space:    
        a*x + b*y + c = 0
        '''         
        if representation == 'points':
            [x1, y1] = a.cartesian()
            [x2, y2] = b.cartesian()
            assert (not gen.equal(x1, x2)) or (not gen.equal(y1, y2)), "The two given points are identical"
            self.a = y2 - y1
            self.b = x1 - x2
            self.c = - x2*self.a - y2*self.b
        elif representation == 'point-angle':
            [x0, y0] = a.cartesian()
            if gen.equal(math.cos(b), 0.0):
                self.a = 1.0
                self.b = 0.0
                self.c = - x0
            else:
                self.a = math.tan(b)
                self.b = - 1.0
                self.c = y0 - self.a*x0

        elif representation == 'equation':
            self.a = a
            self.b = b
            self.c = c
        else:
            assert False, "Unknown representation"
        
        self.theta = math.atan2(- self.a, self.b)

        if gen.equal(self.b, 0.0):  # Equation: x = h  
            self.m = np.inf
            self.h = - self.c/ self.a
        else:   # Equation: y = m*x + h  
            self.m = - self.a / self.b
            self.h = - self.c / self.b
        
    def possess(self, point):
        x = point.cartesian()
        return gen.equal(self.a*x[0] + self.b*x[1] + self.c, 0.0)    

    def dist(self, point):
        ro = math.sqrt(self.a*self.a + self.b*self.b)
        return abs(self.a*x0 + self.b*y0 + self.c)/ro

    def __str__(self):
        return symbolic_term(self.a, 'x', plus_sign = False) + symbolic_term(self.b, 'y')  + ' = ' + symbolic_term(- self.c, '', plus_sign = False)

    def intersection(self, line):
        M = np.array([[self.a, self.b],[line.a, line.b]])
        c = np.array([- self.c, - line.c])
        if gen.equal(np.linalg.det(M), 0.0):
            return None
        else:
            return Point_2D(np.dot(np.linalg.inv(M), c))

class Circle_2D(object):
    ## Class Constructor
    #  Creates a circle in 2D space with given radius and center. The equation of the circle is given as:
    #  $$
    #  **(x- c)**^T \cdot **(x - c)** = R^2 
    #  $$
    #  @param R      A float variable representing the circle radius.
    #  @param center An instance of class Point_2D() specifying the ellipsoid center
    def __init__(self, R = 1.0, center = Point_2D(pos = np.zeros(2))):
        # Checking arguments:
        func_name = sys._getframe().f_code.co_name
        genpy.check_type(R, [float], __name__, self.__class__.__name__, func_name, 'R')
        #
        self.R      = R
        self.center = center
        
    def area(self):
        return math.pi*self.R*self.R
        
    def intersection(self, line):
        '''    
        a*x+b*y+c=0
        y = m*x + h
        (x - x0)^2 + (m*x + h - y0)^2 = R^2
        (1+m^2)*x^2 + 2*[m*(h-y0) - x0]*x + (h - y0)^2 - R^2 = 0
        '''
        [x0, y0] = self.center.cartesian()
        sol   = []
        h_y0  = line.h - y0
        alpha = 1.0 + line.m*line.m
        if line.m == np.inf:
            delta = self.R*self.R - (x0 - line.h)**2  
            if gen.equal(delta , 0.0):
                sol.append(Point_2D(np.array([xp,y0])))
            elif delta > 0:
                sd = math.sqrt(delta)
                sol.append(Point_2D(np.array([line.h, y0 - sd])))
                sol.append(Point_2D(np.array([line.h, y0 + sd])))
        else:
            betta = line.m*h_y0 - x0
            gamma = x0*x0 + h_y0*h_y0 - self.R*self.R
            delta = betta*betta - alpha*gamma
            if gen.equal(delta , 0.0):
                x = - betta / alpha      
                y = line.m*x + line.h
                sol.append(Point_2D(np.array([x,y])))
            elif delta > 0:
                sd = math.sqrt(delta)
                x1 = (- betta - sd)/alpha
                y1 = line.m*x1 + line.h
                sol.append(Point_2D(np.array([x1,y1])))
                x2 = (- betta + sd)/alpha
                y2 = line.m*x2 + line.h
                sol.append(Point_2D(np.array([x2,y2])))
        return sol    
    '''
    def min_dist(self, point):
        p0 = np.dot(self.R, point.cartesian() - self.center['cartesian'])
        (theta, phi) = opt.distance_from_ellipsoid(a  = self.a, b = self.b, c = self.c, x0 = p0[0], y0 = p0[1], z0 = p0[2], silent = False)
        px  = self.a*math.cos(theta)*math.cos(phi)
        py  = self.b*math.sin(theta)*math.cos(phi)
        pz  = self.c*math.sin(phi)
        pp  = np.array([px, py, pz])
        return Point_3D(pos = np.dot(self.R, pp) + self.center['cartesian'])
        
    def max_dist(self, point):
        p0 = np.dot(self.R, point.cartesian() - self.center['cartesian'])
        (theta, phi) = opt.distance_from_ellipsoid(a  = self.a, b = self.b, c = self.c, x0 = p0[0], y0 = p0[1], z0 = p0[2], silent = False, maximum = True)
        px  = self.a*math.cos(theta)*math.cos(phi)
        py  = self.b*math.sin(theta)*math.cos(phi)
        pz  = self.c*math.sin(phi)
        pp  = np.array([px, py, pz])
        return Point_3D(pos = np.dot(self.R, pp) + self.center['cartesian'])
    '''

    def possess(self, point):
        x  = point.cartesian() - self.center.cartesian()
        r2 = sum(x*x)
        return gen.equal(r2, self.R*self.R)
