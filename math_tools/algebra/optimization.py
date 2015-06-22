## @file        	optimization.py
#  @brief     		This module provides fast solution to some typical optimization problems
#  @author      	Nima Ramezani Taghiabadi 
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. :   04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	1.0
#
#  start date:      June 2015
#  Last Revision:  	19 June 2015 

import numpy as np, math

from math_tools import general_math as gen

max_iter = 20

def distance_from_ellipsoid(a, b, c, x0, y0, z0, t0 = None, silent = True, maximum = False):
    global max_iter
    '''
    Find theta and phi that minimize function:
    f(theta, phi) = ( a*cos(theta)*cos(phi) - x0 )^2 + ( b*sin(theta)*cos(phi) - y0 )^2 + ( c*sin(phi) - z0 )^2

    The method is numerical using the Hessian matrix. 
    The iterations start from theta_0 and phi_0
    '''
    a2   = a*a
    b2   = b*b
    c2   = c*c

    if t0 == None:
        if gen.equal(x0, 0.0) and gen.equal(y0, 0.0) and gen.equal(z0, 0.0):
            if maximum:
                if max(a,b,c) == a:  # To maximize the function, min must be replaced by max
                    t0 = np.zeros(2)
                elif max(a,b,c) == b:
                    t0 = np.array([math.pi/2, 0.0])
                else:
                    t0 = np.array([0.0, math.pi/2])
            else:
                if min(a,b,c) == a:  # To maximize the function, min must be replaced by max
                    t0 = np.zeros(2)
                elif min(a,b,c) == b:
                    t0 = np.array([math.pi/2, 0.0])
                else:
                    t0 = np.array([0.0, math.pi/2])
        else:
            if maximum:
                ssign = - 1.0
            else:
                ssign = 1.0
            s   = ssign/math.sqrt(x0*x0/a2 + y0*y0/b2 + z0*z0/c2) # To find the max distance s must be negative s = -1.0/math.sqrt(...)
            the = math.atan2(a*y0, b*x0)
            phi = math.atan2(z0*s/c, s*math.sqrt(x0*x0/a2 + y0*y0/b2))
            assert gen.equal(a*math.cos(the)*math.cos(phi), x0*s)
            assert gen.equal(b*math.sin(the)*math.cos(phi), y0*s)
            assert gen.equal(c*math.sin(phi), z0*s)
            t0  = np.array([the,phi])
            
    H  = np.zeros((2,2))
    df = np.zeros(2)

    t    = t0
    stay = True
    cnt  = 0

    while stay and (cnt < max_iter):

        ct  = math.cos(t[0])
        cf  = math.cos(t[1])
        st  = math.sin(t[0])
        sf  = math.sin(t[1])
        ct2 = ct*ct
        st2 = st*st
        cf2 = cf*cf
        sf2 = sf*sf
        c2t = ct2 - st2
        c2f = cf2 - sf2
        s2t = 2*st*ct
        s2f = 2*sf*cf

        df[0]  = 0.5*(b2 - a2)*s2t*cf2 + cf*(a*x0*st - b*y0*ct)
        df[1]  = - 0.5*s2f*(a2*ct2 + b2*st2 - c2) + sf*(a*x0*ct + b*y0*st) - c*z0*cf

        stay = not gen.equal(np.linalg.norm(df), 0.0)
        if stay:
            H[0,0] = (b2 - a2)*c2t*cf2 + cf*(a*x0*ct + b*y0*st)
            H[0,1] = 0.5*(a2 - b2)*s2t*s2f  - sf*(a*x0*st - b*y0*ct)
            H[1,0] = H[0,1]
            H[1,1] = - c2f*(a2*ct2 + b2*st2 - c2) + cf*(a*x0*ct + b*y0*st) + c*z0*sf
    
            t = t - np.dot(np.linalg.inv(H), df)

        cnt += 1    
        if not silent:
            print 
            print 'iteration:', cnt, ' cost function value:', (a*ct*cf - x0)**2 + (b*st*cf - y0)**2 + (c*sf - z0)**2, ' norm(div_f):', np.linalg.norm(df)

    return (t[0], t[1])
