## @file        	jacobian.py
#  @brief     		This module provides three classes representing the geometric, analytic and error jacobians of a manipulator
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
#  Last Revision:  	11 January 2015


import numpy, math

from math_tools.geometry import rotation
from math_tools.algebra import quaternions, vectors_and_matrices as vecmatlib 

## This class contains the standard analytic jacobian of a manipulator with required methods for calculating it.
class Analytic_Jacobian(object):
    
	## The Class Constructor:
	#  @param config_settings An instance of class 
    #         \link magiks.jointspace.manipulator_configuration.Manipulator_Configuration_Settings Manipulator_Configuration_Settings \endlink 
    #         containing the configuration settings of the manipulator
    def __init__(self, config_settings):
        
        
        self.config_settings = config_settings    
        self.clear()
    
        
# \cond
    ##  Clears the transfer matrices and their derivatives (properties \c H and \c U)
    def clear(self):
        ## A two dimensional list of <tt> 4 X 4 </tt> hemogeneous matrices. 
        #  <tt> U[i][j] </tt> represents the partial derivative of <tt> T[i] </tt>  
        #  (Transformation matrix of link \a i) with respect to the j-th joint variation (q[j])
        self.U      = [ None for i in range(0, self.config_settings.njoint) ]
        
        self.H      = None

    ## Returns the analytic jacobian for a specified link
    #  @param link_number An integer specifying the link number for which the analytic Jacobian is required
    def __getitem__(self, link_number):
        if self.U[link_number] == None:
            self.U[link_number] = [numpy.eye(4) for j in range(0, link_number + 1)]
            j = 0
            for jj in range(0, link_number + 1):
                if self.config_settings.free[jj]:
                    if self.config_settings.prismatic[jj]:
                        self.U[link_number][j][0,0] = 0.00 
                        self.U[link_number][j][1,1] = 0.00 
                        self.U[link_number][j][2,2] = 0.00 
                        if j==0:
                            U[link_number][j][2,3] = 1.00 
                        else:
                            self.U[link_number][j][0,3] = self.H[jj-1][0,2]
                            self.U[link_number][j][1,3] = self.H[jj-1][1,2]
                            self.U[link_number][j][2,3] = self.H[jj-1][2,2]
                    else:
                        if j==0:
                            # pj_1: position of the origin of frame link "j-1"
                            pj_1 = [0,0,0]
                            # szj_1: skew of unit vector "z" of frame link "j-1"
                            szj_1 = numpy.array( [[0 , -1, 0 ],
                                                  [1 ,  0, 0 ],
                                                  [0 ,  0, 0 ]])
                        else:
                            pj_1  = self.H[jj-1][0:3,3]
                            szj_1 = rotation.skew(self.H[jj-1][0:3,2])
            
                        # RRQJ: rond (partial der)/ rond of q_j (A multiplier which derivates in respect with q_j)
                        RRQJ = vecmatlib.extended_matrix(szj_1, - numpy.dot(szj_1,pj_1) )
                        self.U[link_number][j] = numpy.dot(RRQJ, self.H[link_number])
                    j = j + 1
                    
        return self.U[link_number]    
# \endcond

## Contains the Geometric jacobian of an end-effector that can be established of a number of reference positions (Task Points) or 
#  reference orientations (Task Frames) with methods for creating and calculating them.
class Geometric_Jacobian(object):
    
	## The Class Constructor:
	#  @param config_settings An instance of class 
    #         \link magiks.jointspace.manipulator_configuration.Manipulator_Configuration_Settings Manipulator_Configuration_Settings \endlink 
    #         containing the configuration settings of the manipulator
    def __init__(self, config_settings):
        "Definition of the geometric jacobian matrix"
        self.value    = numpy.zeros((3 , config_settings.DOF))
        self.config_settings  = config_settings

# \cond
    def clear(self):
        self.value = None

    def update_for_position(self, task_point, analytic_jacobian):
        '''
        Create and calculate the geometric jacobian for the given position references (reference_positions)
        '''
        # print "8 ****** ", numpy.sum(analytic_jacobian.U[task_point.lp[0].ln])
        # print "9 ****** ", numpy.sum(analytic_jacobian[task_point.lp[0].ln][0])

        self.value = numpy.zeros((3, self.config_settings.DOF))
        gj = [numpy.zeros((3)) for i in range(0, self.config_settings.DOF)]
        for j in range(0, len(task_point.lp)):
            k = 0
            for kk in range(0,task_point.lp[j].ln + 1):
                if self.config_settings.free[kk]:
                    x = numpy.dot(analytic_jacobian[task_point.lp[j].ln][k],vecmatlib.extend_vector(task_point.lp[j].pv))
                    gj[k] = gj[k] + task_point.lp[j].w * x[0:3]
                    k = k + 1
        for i in range(0,3):
            for j in range(0, self.config_settings.DOF):
                self.value[i,j] = gj[j][i]

    def update_for_orientation(self, tskfrm, transfer_matrices):
        '''
        Create and calculate the geometric jacobian for the given orientation references (reference_orientations)
        '''
        self.value = numpy.zeros((3, self.config_settings.DOF))

        self.value[0,0] = 0
        self.value[1,0] = 0
        self.value[2,0] = 1

        for ii in range(0,tskfrm.ln + 1):
            i = 0
            if self.config_settings.free[ii]:
                if self.config_settings.prismatic[i]:
                    v = numpy.zeros((3))            
                else:
                    if i == 0:
                        v = numpy.array([0,0,1])
                    else:
                        v = vecmatlib.uvect(transfer_matrices[ii-1],2)
            
                self.value[0,i] = v[0]
                self.value[1,i] = v[1]
                self.value[2,i] = v[2]
                i = i + 1
# \endcond

##  Contains the Error Jacobian of an end-effector established from one or more reference positions (Task Points) 
#   or reference orientations (Task Framess) with required methods for creating and calculating it
class Error_Jacobian(object):
# \cond
    def __init__(self, config_settings):
        self.config_settings = config_settings
        #Creation of the error jacobian matrix
        self.value = None

    def clear(self):
        self.value = None

    def basis_error_jacobian_for_position(self, tskpnt, ajac):
        '''
        Calculate and return the error jacobian (Jf) according to the basis error function for position represented by: tskpnt.error.basis_error_function
        In this version only one representation of position error is supported:  "differential_cartesian_coordinates"
        '''
        if tskpnt.error.settings.representation == 'Cartesian Coordinates':
            ''' Alternative
            j  = 0
            for jj in range(0,self.config_settings.DOF):
                if self.config_settings.free[jj]:
                    e   = tskpnt.error.value
                    de  = tskpnt.geometric_jacobian.value[:,j]
                    b   = self.settings.power*de*(e**(self.settings.power - numpy.ones(4)))    
                    if j == 0:
                        J = [b]
                    else:
                        J = np.append(J, [b], axis = 0) 
                    j   = j + 1            
            '''
            p = 3
            assert len(tskpnt.error.settings.power) == p
            Jf  = numpy.copy(tskpnt.geometric_jacobian(ajac))

            for k in range (0,p):
                if (tskpnt.error.settings.power[k] != 0) and (tskpnt.error.settings.power[k] != 1):
                    err = tskpnt.error.power[k]*((tskpnt.r[k] - tskpnt.rd[k])**(tskpnt.error.settings.power[k] - 1))
                    j = 0
                    for jj in range(0,self.config_settings.DOF):
                        if self.config_settings.free[jj]:
                            Jf[k,j] = tskpnt.geometric_jacobian.value[k,j]*err
                            j = j + 1
                            
        elif tskpnt.error.settings.representation == 'Cylindrical Coordinates':
            assert False
        elif tskpnt.error.settings.representation == 'Spherical Coordinates':
            assert False
        else:
            assert False
        
        return Jf

    def basis_error_jacobian_for_orientation(self, tskfrm, ajac):
        '''
        Calculate and return the error jacobian (Jf) according to the basis error function for orientation represented by: tskfrm.error.basis_error_function
        In this version six representations of orientation error are supported:  
        "Axis Inner Product" , "relative_rotation_matrix_trace_minus_three" , "differential_quaternions" , "relative_rotation_angle", 
        "relative_rotation_vector_Cayley_Gibbs_Rodrigues" and "differential_rotation_matrix"
        '''
        rpn  = tskfrm.error.settings.representation
        pwr  = tskfrm.error.settings.power
        if rpn == 'vector':
            tskfrm.ra.set_parametrization( tskfrm.error.settings.parametrization )
            tskfrm.rd.set_parametrization( tskfrm.error.settings.parametrization )
        if tskfrm.error.settings.metric_type == 'relative':
            p = len(pwr)
            j  = 0
            for jj in range(0, tskfrm.ln + 1):
                if self.config_settings.free[jj]:
                    ra = tskfrm.orientation(ajac.H)
                    ra.set_velocity(ajac[tskfrm.ln][j][0:3,0:3])
                    Oe   = ra / tskfrm.rd
                    e    = Oe[rpn]
                    de   = Oe[rpn + '_velocity']
                    if rpn == 'matrix':
                        e  = e.flatten()   
                        de = de.flatten()   
                    b    = pwr*de*(e**(pwr - numpy.ones(p)))    
                    if j == 0:
                        J = [b]
                    else:
                        J = numpy.append(J, [b], axis = 0) 
                    j   = j + 1            

        elif tskfrm.error.settings.metric_type   == 'differential':

            p  = len(pwr)
            j  = 0
            e  = tskfrm.ra[rpn] - tskfrm.rd[rpn]
            if rpn == 'matrix':
                e  = e.flatten()
            
            for jj in range(0,tskfrm.ln + 1):
                if self.config_settings.free[jj]:
                    tskfrm.ra.set_velocity(ajac[tskfrm.ln][j][0:3,0:3])
                    de   = tskfrm.ra[rpn + '_velocity']
                    if rpn == 'matrix':
                        de = de.flatten()   
                    b    = pwr*de*(e**(pwr - numpy.ones(p)))    
                    if j == 0:
                        J = [b]
                    else:
                        J = numpy.append(J, [b], axis = 0) 
                    j   = j + 1            

        elif tskfrm.error.settings.metric_type   == 'special':

            if rpn   == 'AxInPr':
                p = 3
                rd = tskfrm.rd['matrix']
                Jf = numpy.zeros((p,self.config_settings.DOF))
                for k in range (0,p):
                    if pwr[k] == 0:
                        err  = 0
                        errp = 0
                    elif pwr[k] == 1:
                        err  = 1
                        errp = 0
                    elif pwr[k] == 2:
                        err  = 2*(numpy.dot(tskfrm.rd.frame_axis(k),tskfrm.ra.frame_axis(k)) - 1)
                        errp = 2
                    else:
                        err = tskfrm.error.power[k]*((numpy.dot(tskfrm.rd.frame_axis(k),tskfrm.ra.frame_axis(k)) - 1)**(tskfrm.error.power[k] - 1))
                    j = 0
                    for jj in range(0,tskfrm.ln + 1):
                        if self.config_settings.free[jj]:                    
                            jz1_k_j = numpy.dot(ajac[tskfrm.ln][j][0:3,k],rd[0:3,k])
                            Jf[k,j] = jz1_k_j*err
                            j = j + 1

            elif rpn   == 'DiNoQu':
                p = 3
                Jf = numpy.zeros((p,self.config_settings.DOF))
                j   = 0
                qnd = tskfrm.rd['quaternion']
                for jj in range(0,tskfrm.ln + 1):
                    if self.config_settings.free[jj]:
                        uqns = quaternions.unit_quaternion_velocity(tskfrm.ra['matrix'], ajac[tskfrm.ln][j])
                        for k in range (0,3):
                            if pwr[k] != 0:
                                jz_k_j = qnd[k+1]*uqns[0] - qnd[0]*uqns[k+1] 
                                if pwr[k] == 1:
                                   Jf[k,j] = jz_k_j
                                else:
                                    qn  = tskfrm.ra['quaternion']
                                    err = pwr[k]*((qnd[0]*qn[k+1] - qn[0]*qnd[k+1])**(pwr[k] - 1))
                                    Jf[k,j] = jz_k_j*err
                        j = j + 1

            elif rpn   == 'AxInPr + DiNoQu':
                p = 6
                Jf = numpy.zeros((p,self.config_settings.DOF))
                for k in range (0,3):
                    if pwr[k] == 0:
                        err  = 0
                        errp = 0
                    elif pwr[k] == 1:
                        err  = 1
                        errp = 0
                    elif pwr[k] == 2:
                        err  = 2*(numpy.dot(tskfrm.rd.frame_axis(k),tskfrm.ra.frame_axis(k)) - 1)
                        errp = 2
                    else:
                        err = tskfrm.error.power[k]*((numpy.dot(tskfrm.rd.frame_axis(k),tskfrm.ra.frame_axis(k)) - 1)**(tskfrm.error.power[k] - 1))
                    j = 0
                    for jj in range(0,tskfrm.ln + 1):
                        if self.config_settings.free[jj]:                    
                            jz1_k_j = numpy.dot(analytic.U[tskfrm.ln][j][0:3,k],tskfrm.rd.frame_axis(k))
                            Jf[k,j] = jz1_k_j*err
                            j = j + 1
                j   = 0
                qnd = tskfrm.rd['quaternion']
                for jj in range(0,tskfrm.ln + 1):
                    if self.config_settings.free[jj]:
                        uqns = quaternions.unit_quaternion_velocity(tskfrm.ra['matrix'], analytic.U[tskfrm.ln][j])
                        for k in range (3,6):
                            if pwr[k] != 0:
                                jz_k_j = qnd[k-2]*uqns[0] - qnd[0]*uqns[k-2] 
                                if pwr[k] == 1:
                                   Jf[k,j] = jz_k_j
                                else:
                                    qn  = tskfrm.ra['quaternion']
                                    err = pwr[k]*((qnd[0]*qn[k-2] - qn[0]*qnd[k-2])**(pwr[k] - 1))
                                    Jf[k,j] = jz_k_j*err
                        j = j + 1

            elif rpn   == 'ReRoAn + ReOrVe':
                p = 4
                assert p == len(pwr)
                e  = numpy.zeros(p)
                de = numpy.zeros(p)
                j  = 0
                for jj in range(0, tskfrm.ln + 1):
                    if self.config_settings.free[jj]:
                        tskfrm.ra.set_velocity(analytic.U[tskfrm.ln][j][0:3,0:3])
                        Oe   = tskfrm.ra / tskfrm.rd
                        
                        e[0]    = Oe['angle']    
                        e[1:4]  = Oe['vector']
                        de[0]   = Oe['angle_velocity']
                        de[1:4] = Oe['vector_velocity']
                        b    = pwr*de*(e**(pwr - numpy.ones(p)))    
                        if j == 0:
                            J = [b]
                        else:
                            J = numpy.append(J, [b], axis = 0) 
                        j   = j + 1            
                Jf = J.T   

            else:
                assert False        
            return Jf

        else:
            assert False, gen.err_msg(__name__, "basis_error_jacobian_for_orientation", tskfrm.error.settings.metric_type + "is an invalid metric type")        

        return J.T

    def update_for_position(self, tskpnt, ajac):
        '''
        Calculate and Update the error jacobian for position
        First calculate the basis error jacobian and then multiplied by the weighting matrix
        '''
        bej = self.basis_error_jacobian_for_position(tskpnt, ajac)
        self.value = numpy.dot(tskpnt.error.settings.weight, bej)
    
    def update_for_orientation(self, tskfrm, ajac):
        '''
        Calculate and Update the error jacobian for orientation
        First calculate the basis error jacobian and then multiplied by the weighting matrix
        '''
        bej = self.basis_error_jacobian_for_orientation(tskfrm, ajac)
        self.value = numpy.dot(tskfrm.error.settings.weight, bej)

# \endcond



