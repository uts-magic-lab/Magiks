## @file:           task_frame.py
#  @brief:    	    This module provides a class representing the reference orientation of an endeffector and the desired value for it.
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
#  Last Revision:  	11 January 2015

import numpy, math

from packages.nima.robotics.kinematics.jacobian import jacobian as jaclib

import packages.nima.mathematics.algebra.vectors_and_matrices as vecmat

from packages.nima.mathematics.geometry import metric, geometry as geo, trajectory as trajlib 


