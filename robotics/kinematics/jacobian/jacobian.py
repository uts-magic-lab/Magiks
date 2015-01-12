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

from packages.nima.mathematics import rotation, quaternions, vectors_and_matrices as vecmatlib 

class Analytic_Jacobian:
    '''
    Contains the standard analytic jacobian of a manipulator with methods for calculating it.
    '''
    def __init__(self, njoint):
        '''
        U : A two dimensional list of 4 X 4 matrices. U[i][j] represents the partial derivative of T[i] (Transformation matrix of link i) with respect to j-th joint variation (q[j])
        '''
        self.U      = [ [ numpy.eye(4) for j in range(0,i + 1)] for i in range(0,njoint) ]
        # A list of booleans representing whether the jacoian for the corresponding list is required and should be calculated
        self.jacobian_required_for_link      = [False for i in range(0,njoint)]

    def evaluate_requirements(self, tskpnt, tskfrm):
        '''
        Evaluate which links require jacobian. 
        (Only if the positions and orientations of the endeffector refer to a link, the analytic Jacobian of that link needs to be calculated.)
        '''
        for i in range(0, len(tskpnt)):
            for j in range(0, len(tskpnt[i].lp)):
                self.jacobian_required_for_link[tskpnt[i].lp[j].ln] = True
        for i in range(0, len(tskfrm)):
            self.jacobian_required_for_link[tskfrm[i].ln] = True

    def update_analytic_jacobian_for_link(self, link_number, trans_mat, config):
        '''
        Calculates the analytic jacobian for one link identified by link_number
        '''
        j = 0
        for jj in range(0, link_number + 1):
            if config.settings.free[jj]:
                if config.settings.prismatic[jj]:
                    self.U[link_number][j][0,0] = 0.00 
                    self.U[link_number][j][1,1] = 0.00 
                    self.U[link_number][j][2,2] = 0.00 
                    if j==0:
                        U[link_number][j][2,3] = 1.00 
                    else:
                        self.U[link_number][j][0,3] = trans_mat.H[jj-1][0,2]
                        self.U[link_number][j][1,3] = trans_mat.H[jj-1][1,2]
                        self.U[link_number][j][2,3] = trans_mat.H[jj-1][2,2]
                else:
                    if j==0:
                        # pj_1: position of the origin of frame link "j-1"
                        pj_1 = [0,0,0]
                        # szj_1: skew of unit vector "z" of frame link "j-1"
                        szj_1 = numpy.array( [[0 , -1, 0 ],
                                              [1 ,  0, 0 ],
                                              [0 ,  0, 0 ] ] )
                    else:
                        pj_1  = trans_mat.H[jj-1][0:3,3]
                        szj_1 = rotation.skew(trans_mat.H[jj-1][0:3,2])
        
                    # RRQJ: rond (partial der)/ rond of q_j (A multiplier which derivates in respect with q_j)
                    RRQJ = vecmatlib.extended_matrix(szj_1, - numpy.dot(szj_1,pj_1) )
                    self.U[link_number][j] = numpy.dot(RRQJ,trans_mat.H[link_number])
                j = j + 1


    def update(self, tm, config):
        '''
        Calculates the analytic jacobian for all required links
        '''
        for i in range(1, len(self.jacobian_required_for_link)):
            if self.jacobian_required_for_link[i]:
                self.update_analytic_jacobian_for_link(i, tm, config)
        
class Geometric_Jacobian:
    '''
    Contains the Geometric jacobian of a reference_position or reference_orientation with methods for creating and calculating it
    '''    
    def __init__(self, number_of_joints):
        "Definition of the geometric jacobian matrix"
        self.value = numpy.zeros((3 , number_of_joints))

    def update_for_position(self, reference_position, config, analytic):
        '''
        Create and calculate the geometric jacobian for the given position references (reference_positions)
        '''
        gj = [numpy.zeros((3)) for i in range(0,config.settings.DOF)]
        for j in range(0, len(reference_position.lp)):
            k = 0
            for kk in range(0,reference_position.lp[j].ln + 1):
                if config.settings.free[kk]:
                    x = numpy.dot(analytic.U[reference_position.lp[j].ln][k],vecmatlib.extend_vector(reference_position.lp[j].pv))
                    gj[k] = gj[k] + reference_position.lp[j].w * x[0:3]
                    k = k + 1
        for i in range(0,3):
            for j in range(0,config.settings.DOF):
                self.value[i,j] = gj[j][i]

    def update_for_orientation(self, tskfrm, forward_kinematics):
        '''
        Create and calculate the geometric jacobian for the given orientation references (reference_orientations)
        '''
        self.value = numpy.zeros((3, forward_kinematics.configuration.settings.DOF))

        self.value[0,0] = 0
        self.value[1,0] = 0
        self.value[2,0] = 1

        for ii in range(0,tskfrm.ln + 1):
            i = 0
            if forward_kinematics.configuration.settings.free[ii]:
                if forward_kinematics.configuration.settings.prismatic[i]:
                    v = numpy.zeros((3))            
                else:
                    if i == 0:
                        v = numpy.array([0,0,1])
                    else:
                        v = vecmatlib.uvect(transfer_matrices.H[ii-1],2)
            
                self.value[0,i] = v[0]
                self.value[1,i] = v[1]
                self.value[2,i] = v[2]
                i = i + 1
        return 0

class Error_Jacobian:
    '''
    Contains the Error jacobian of a reference_position or reference_orientation with methods for creating and calculating it
    '''    
    def __init__(self, number_of_joints):
        #Creation of the error jacobian matrix
        self.value = numpy.zeros((3 ,number_of_joints))

    def basis_error_jacobian_for_position(self, tskpnt, config):
        '''
        Calculate and return the error jacobian (Jf) according to the basis error function for position represented by: tskpnt.error.basis_error_function
        In this version only one representation of position error is supported:  "differential_cartesian_coordinates"
        '''
        if tskpnt.error.settings.representation == 'Cartesian Coordinates':
            ''' Alternative
            j  = 0
            for jj in range(0,config.settings.DOF):
                if config.settings.free[jj]:
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
            Jf  = numpy.copy(tskpnt.geometric_jacobian.value)

            for k in range (0,p):
                if (tskpnt.error.settings.power[k] != 0) and (tskpnt.error.settings.power[k] != 1):
                    err = tskpnt.error.power[k]*((tskpnt.r[k] - tskpnt.rd[k])**(tskpnt.error.settings.power[k] - 1))
                    j = 0
                    for jj in range(0,config.settings.DOF):
                        if config.settings.free[jj]:
                            Jf[k,j] = tskpnt.geometric_jacobian.value[k,j]*err
                            j = j + 1
                            
        elif tskpnt.error.settings.representation == 'Cylindrical Coordinates':
            assert False
        elif tskpnt.error.settings.representation == 'Spherical Coordinates':
            assert False
        else:
            assert False
        
        return Jf

    def basis_error_jacobian_for_orientation(self, tskfrm, config, analytic):
        '''
        Calculate and return the error jacobian (Jf) according to the basis error function for orientation represented by: tskfrm.error.basis_error_function
        In this version six representations of orientation error are supported:  
        "Axis Inner Product" , "relative_rotation_matrix_trace_minus_three" , "differential_quaternions" , "relative_rotation_angle", 
        "relative_rotation_vector_Cayley_Gibbs_Rodrigues" and "differential_rotation_matrix"
        '''
        rpn  = tskfrm.error.settings.representation
        pwr  = tskfrm.error.settings.power
        if rpn == 'vector':
            tskfrm.ra.set_generating_function( tskfrm.error.settings.generating_function )
            tskfrm.rd.set_generating_function( tskfrm.error.settings.generating_function )
        if tskfrm.error.settings.metric_type == 'relative':
            p = len(pwr)
            j  = 0
            for jj in range(0, tskfrm.ln + 1):
                if config.settings.free[jj]:
                    tskfrm.ra.set_speed(analytic.U[tskfrm.ln][j][0:3,0:3])
                    Oe   = tskfrm.ra / tskfrm.rd
                    e    = Oe[rpn]
                    de   = Oe[rpn + '_speed']
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
                if config.settings.free[jj]:
                    tskfrm.ra.set_speed(analytic.U[tskfrm.ln][j][0:3,0:3])
                    de   = tskfrm.ra[rpn + '_speed']
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
                Jf = numpy.zeros((p,config.settings.DOF))
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
                        if config.settings.free[jj]:                    
                            jz1_k_j = numpy.dot(analytic.U[tskfrm.ln][j][0:3,k],rd[0:3,k])
                            Jf[k,j] = jz1_k_j*err
                            j = j + 1

            if rpn   == 'AxInPr + DiNoQu':
                p = 6
                Jf = numpy.zeros((p,config.settings.DOF))
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
                        if config.settings.free[jj]:                    
                            jz1_k_j = numpy.dot(analytic.U[tskfrm.ln][j][0:3,k],tskfrm.rd.frame_axis(k))
                            Jf[k,j] = jz1_k_j*err
                            j = j + 1
                j   = 0
                qnd = tskfrm.rd['quaternion']
                for jj in range(0,tskfrm.ln + 1):
                    if config.settings.free[jj]:
                        uqns = quaternions.unit_quaternion_speed(tskfrm.ra['matrix'], analytic.U[tskfrm.ln][j])
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
            else:
                assert False        
            return Jf

        else:
            assert False, gen.err_msg(__name__, "basis_error_jacobian_for_orientation", tskfrm.error.settings.metric_type + "is an invalid metric type")        

        return J.T
    
    def update_for_position(self, tskpnt, cnfg):
        '''
        Calculate and Update the error jacobian for position
        First calculate the basis error jacobian and then multiplied by the weighting matrix
        '''
        bej = self.basis_error_jacobian_for_position(tskpnt, cnfg)
        self.value = numpy.dot(tskpnt.error.settings.weight, bej)
    
    def update_for_orientation(self, tskfrm, cnfg, analytic):
        '''
        Calculate and Update the error jacobian for orientation
        First calculate the basis error jacobian and then multiplied by the weighting matrix
        '''
        bej = self.basis_error_jacobian_for_orientation(tskfrm, cnfg, analytic)
        self.value = numpy.dot(tskfrm.error.settings.weight, bej)

        



