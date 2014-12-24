'''   Header
@file:          test_poly3.py
@brief:    	    This module runs a test for class "Polynomial3_Interpolator"
@author:        Nima Ramezani; DFKI Bremen
@start date:    April 2011
@version:	    0.1
Last Revision:  29 April 2011
'''

import numpy, polynomials

poly = polynomials.Polynomial3_Interpolator()

initial_position = numpy.array([1, -1,  2])
final_position   = numpy.array([3,  2, -5])

initial_velocity = numpy.array([0, 0, 0])
final_velocity   = numpy.array([0, 0, 0])

poly.find_coefficients(1.0, initial_position, final_position, initial_velocity, final_velocity)

print "Position at t = 0.0 : ", poly.interpolated_position(0.0)
print "Position at t = 0.5 : ", poly.interpolated_position(0.5)
print "Position at t = 1.0 : ", poly.interpolated_position(1.0)

