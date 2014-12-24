'''
@file    complexpy.py
@brief   this module provides tools by ehich you can work with complex numbers.
@author  Nima Ramezani; DFKI Bremen
@date    2010
@version 0.1
'''
 

import math

class Complex_Number:

    def __init__(self):

	    self.ro   = 0
	    self.teta = 0
	    self.re   = 0
	    self.im   = 0
        

    def polar_to_cartesian(self):
		
	    self.re = self.ro*math.cos(self.teta)
	    self.im = self.ro*math.sin(self.teta)

    def cartesian_to_polar(self):
		
	    self.ro =   math.sqrt(self.re*self-re+self.im*self.im)
	    self.teta = math.atan2(self.im,self.re)


		


def complex_multiply(c1,c2):
	
	'''
	This function multiplies c1 and c2 and returns the product
	'''
	
	c3.re = c1.re*c2.re-c1.im*c2.im
	c3.im = c1.re*c2.im+c1.im*c2.re

	c3.ro   = c1.ro * c2.ro	
	c3.teta = c1.teta + c2.teta	

	while c3.teta > 2*math.pi:
		  c3.teta = c3.teta-2*math.pi
		  		  
	while c3.teta < -2*math.pi:
          c3.teta = c3.teta+2*math.pi
          
