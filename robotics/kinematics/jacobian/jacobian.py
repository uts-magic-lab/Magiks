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
#  @version     	3.0
# 
#  Last Revision:  	03 January 2015


import numpy, math

from packages.nima.mathematics import quaternions, vectors_and_matrices as vecmatlib 

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
                        pj_1 = vecmatlib.uvect(trans_mat.H[jj-1],3)
                        szj_1 = vecmatlib.skew(vecmatlib.uvect(trans_mat.H[jj-1],2))
        
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

class Task_Jacobian:
    '''
    Contains the Task jacobian of a reference_position or reference_orientation with methods for creating and calculating it
    '''    
    def __init__(self):
        '''
        For example, for rotation matrix, number_of_parametrization_elements is 9
        '''
        #Creation of the task jacobian matrix
        self.value = None # Abstract definition, this property contains the Task Jacobian matrix
    

    def basis_task_jacobian_for_position(self, tskpnt):
        '''
        Calculate and return the basis task jacobian (Jft) according to the basis error function for position represented by: tskpnt.error.basis_error_function
        In this version only one representation of position error is supported:  "differential_cartesian_coordinates"
        '''
        if tskpnt.error.basis_error_function == 'differential_cartesian_coordinates':
            assert len(tskpnt.error.power) == 3
            Jft  = - numpy.eye(3)

            for k in range (0, 3):
                if (tskpnt.error.power[k] != 0) and (tskpnt.error.power[k] != 1):
                    c = tskpnt.error.power[k]*((tskpnt.r[k] - tskpnt.rd[k])**(tskpnt.error.power[k] - 1))
                    Jft[k,k] = Jft[k,k]*c
        else:
            assert False
        
        return Jft

    def basis_task_jacobian_for_orientation(self, tskfrm):
        '''
        Calculate and return the task jacobian (Jft) according to the basis error function for orientation represented by: tskfrm.error.basis_error_function
        In this version only one representation of orientation error is supported:  
        "Axis Inner Product"
        '''
        if tskfrm.error.basis_error_function   == 'Axis Inner Product':
            Jft = numpy.zeros((3, 9))
            for k in range(3):
                if (tskfrm.error.power[k] != 0) and (tskfrm.error.power[k] != 1):
                    err = numpy.dot(tskfrm.ra[:,k], tskfrm.rd[:,k]) - 1.0
                    c   = tskfrm.error.power[k]*(err**(tskfrm.error.power[k] - 1))
                else:
                    c = 1.0     
                for j in range(3):
                    Jft[k, 3*j+k] = c*tskfrm.ra.T[k,j]
        else:
            assert False # Any other basis error functions are not supported
        return Jft
    
    def update_for_position(self, tskpnt):
        '''
        Calculate and Update the task jacobian for position
        First calculate the basis task jacobian and then multiplied by the weighting matrix
        '''
        btj = self.basis_task_jacobian_for_position(tskpnt)
        self.value = numpy.dot(tskpnt.error.W, btj)
    
    def update_for_orientation(self, tskfrm):
        '''
        Calculate and Update the task jacobian for orientation
        First calculate the basis task jacobian and then multiplied by the weighting matrix
        '''
        btj = self.basis_task_jacobian_for_orientation(tskfrm)
        self.value = numpy.dot(tskfrm.error.W, btj)

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
        if tskpnt.error.basis_error_function == 'differential_cartesian_coordinates':
            p = 3
            assert len(tskpnt.error.power) == p
            Jf  = numpy.copy(tskpnt.geometric_jacobian.value)

            for k in range (0,p):
                if (tskpnt.error.power[k] != 0) and (tskpnt.error.power[k] != 1):
                    err = tskpnt.error.power[k]*((tskpnt.r[k] - tskpnt.rd[k])**(tskpnt.error.power[k] - 1))
                    j = 0
                    for jj in range(0,config.settings.DOF):
                        if config.settings.free[jj]:
                            Jf[k,j] = tskpnt.geometric_jacobian.value[k,j]*err
                            j = j + 1

        elif tskpnt.error.basis_error_function == 'differential_cylindrical_coordinates':
            assert False
        elif tskpnt.error.basis_error_function == 'differential_spherical_coordinates':
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
        if tskfrm.error.basis_error_function   == 'Axis Inner Product':
            p = 3
            Jf = numpy.zeros((p,config.settings.DOF))
            for k in range (0,p):
                if tskfrm.error.power[k] == 0:
                    err  = 0
                    errp = 0
                elif tskfrm.error.power[k] == 1:
                    err  = 1
                    errp = 0
                elif tskfrm.error.power[k] == 2:
                    err  = 2*(numpy.dot(vecmatlib.uvect(tskfrm.rd,k),vecmatlib.uvect(tskfrm.r,k)) - 1)
                    errp = 2
                else:
                    err = tskfrm.error.power[k]*((numpy.dot(vecmatlib.uvect(tskfrm.rd,k),vecmatlib.uvect(tskfrm.r,k)) - 1)**(tskfrm.error.power[k] - 1))
                j = 0
                for jj in range(0,tskfrm.ln + 1):
                    if config.settings.free[jj]:                    
                        jz1_k_j = numpy.dot(vecmatlib.uvect(analytic.U[tskfrm.ln][j],k),vecmatlib.uvect(tskfrm.rd,k))
                        Jf[k,j] = jz1_k_j*err
                        j = j + 1
        elif tskfrm.error.basis_error_function == 'relative_rotation_matrix_trace_minus_three':
            p = 1
            Jf = numpy.zeros((p,config.settings.DOF))
            for i in range(0,3):
                for t in range(0,3):
                    j = 0
                    for jj in range(0, tskfrm.ln + 1):
                        if config.settings.free[jj]:
                            Jf[0,j] += tskfrm.rd[i,t]*analytic.U[tskfrm.ln][j][i,t]       
                            j += 1
                
        elif tskfrm.error.basis_error_function == 'differential_quaternions':
            p = 3
            Jf = numpy.zeros((p,config.settings.DOF))
            ''''
            Remember that the following code line is implemented many times resulting the same value, so it is better to be defined once
            '''
            qnd = quaternions.unit_quaternion(tskfrm.rd)

            j = 0
            for jj in range(0,tskfrm.ln + 1):
                if config.settings.free[jj]:
                    uqns = quaternions.unit_quaternion_speed(tskfrm.ra, analytic.U[tskfrm.ln][j])
                    for k in range (0,p):
                        if tskfrm.error.power[k] != 0:
                            jz_k_j = qnd[k]*uqns[p] - qnd[p]*uqns[k] 
                            if tskfrm.error.power[k] == 1:
                               Jf[k,j] = jz_k_j
                            else:
                                qn  = quaternions.unit_quaternion(tskfrm.ra)
                                err = tskfrm.error.power[k]*((qnd[p]*qn[k] - qn[p]*qnd[k])**(tskfrm.error.power[k] - 1))
                                Jf[k,j] = jz_k_j*err
                    j = j + 1

        elif tskfrm.error.basis_error_function == 'relative_rotation_angle':
            p = 1
            Jf = numpy.zeros((p,config.settings.DOF))

            for i in range(0,3):
                for t in range(0,3):
                    j = 0
                    for jj in range(0, tskfrm.ln + 1):
                        if config.settings.free[jj]:
                            sin_err = math.sin(tskfrm.error.value[0])
                            if abs(sin_err) < 0.000001:
                                sin_err = 0.000001
                            Jf[0,j] = Jf[0,j] - 0.5*(tskfrm.rd[i,t]/sin_err)*analytic.U[tskfrm.ln][j][i,t]       
                            j = j + 1


        elif tskfrm.error.basis_error_function == 'relative_rotation_vector_identity':
            p = 3
            Jf = tskfrm.geometric_jacobian.value
            
        elif tskfrm.error.basis_error_function == 'relative_rotation_vector_Cayley_Gibbs_Rodrigues':
            p = 3
            RRM = numpy.dot(tskfrm.rd,tskfrm.r.T)
            trace_RRM = RRM[0,0] + RRM[1,1] + RRM[2,2]
            t2_plus_1 = 4/(1 + trace_RRM)
            Jf = t2_plus_1*tskfrm.Jg
        elif tskfrm.error.basis_error_function == 'differential_rotation_matrix':
            p = 9
            Jf = numpy.zeros((p,config.settings.DOF))
            k = 0
            for i in range(0,3):
                for t in range(0,3):
                    if tskfrm.error.power[k] != 0:
                        if tskfrm.error.power[k] == 1:
                            err = 1
                        if tskfrm.error.power[k] == 2:
                            err  = 2*(tskfrm.rd[i,t] - tskfrm.ra[i,t])
                            errp = 2
                        else:
                            err = tskfrm.error.power[k]*(tskfrm.rd[i,t] - tskfrm.ra[i,t])**(tskfrm.error.power[k] - 1)
                        j = 0
                        for jj in range(0,tskfrm.ln + 1):
                            if config.settings.free[jj]:
                                jz1_k_j = analytic.U[tskfrm.ln][j][i,t]                        
                                Jf[k,j] = -jz1_k_j*err
                                j = j + 1
                    k = k + 1
        else:
            assert False # Any other basis error functions are not supported
        return Jf
    
    def update_for_position(self, tskpnt, cnfg):
        '''
        Calculate and Update the error jacobian for position
        First calculate the basis error jacobian and then multiplied by the weighting matrix
        '''
        bej = self.basis_error_jacobian_for_position(tskpnt, cnfg)
        self.value = numpy.dot(tskpnt.error.W, bej)
    
    def update_for_orientation(self, tskfrm, cnfg, analytic):
        '''
        Calculate and Update the error jacobian for orientation
        First calculate the basis error jacobian and then multiplied by the weighting matrix
        '''
        bej = self.basis_error_jacobian_for_orientation(tskfrm, cnfg, analytic)
        self.value = numpy.dot(tskfrm.error.W, bej)

        



