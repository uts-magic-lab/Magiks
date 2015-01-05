'''   Header
@file:          test_polynomials.py
@brief:    	    This module tests the performance of polynomials.py
@author:        Nima Ramezani; UTS
@start date:    February 2011
@version:	    2.0
Last Revision:  04 June 2014
'''


p1 = pn.Point(v_free = False, a_free = False)
p2 = pn.Point(t = 1.0, x = -10.0)
p3 = pn.Point(t = 2.0, x = 15.0, v = -10, v_free = False)
p4 = pn.Point(t = 3.0, x = 5.0, v = 7.5, v_free = False, a = 2.0, a_free = False)
p5 = pn.Point(t = 4.0, x = 12.0, v = 0.0, v_free = False, a = 1.0, a_free = False)

points=[p1,p2,p3,p4,p5]
p = pn.Polynomial()
p.interpolate_smart(points)

