'''   Header
@file:          pyride_interpreter.py
@brief:    	    Contains simplified functions to control NAO using pyride engine
@author:        Nima Ramezani Taghiabadi
                PhD Researcher
                Faculty of Engineering and Information Technology
                University of Technology Sydney (UTS)
                Broadway, Ultimo, NSW 2007, Australia
                Room No.: CB10.03.512
                Phone:    02 9514 4621
                Mobile:   04 5027 4611
                Email(1): Nima.RamezaniTaghiabadi@uts.edu.au 
                Email(2): Nima.RamezaniTaghiabadi@student.uts.edu.au 
                Email(3): N.RamezaniTaghiabadi@uws.edu.au 
                Email(4): nima.ramezani@gmail.com
                Email(5): nima_ramezani@yahoo.com
                Email(6): ramezanitn@alum.sharif.edu

@version:	    1.0
Start date:     20 June 2014
Last Revision:  20 June 2014

Changes from ver 1.0:
'''

import PyNAO, numpy, math
import packages.nima.mathematics.general as gen
import packages.nima.mathematics.vectors_and_matrices as vecmat
import packages.nima.mathematics.rotation as rot

# Service event functions

q  = numpy.zeros(21)
a  = numpy.zeros(3)
Fl = numpy.zeros(3)
Fr = numpy.zeros(3)

ax  = []
ay  = []
az  = []

Flx = []
Fly = []
Flz = []

Frx = []
Fry = []
Frz = []

history = False

def start_history():
    global history
    history = True

def stop_history():
    global history
    history = False

def sensor_data( data ):
    global q
    global a,ax,ay,az
    global Fl,Flx,Fly,Flz
    global Fr,Frx,Fry,Frz
    global history
    '''
    reads joint values from the server and stores it in global variable q
    '''
      
    # additional sensor info from the server.
    # "acc_x", "acc_y", "acc_z", "gyr_x", "gyr_y",
    #"angle_x", "angle_y",
    #"l_fsr_press_x", "l_fsr_press_y", "l_fst_total_w",
    #"r_fsr_press_x", "r_fsr_press_y", "r_fst_total_w",

    #print data
    #print "right ankle pitch input", data['r_ankle_pitch_joint']
    q[0]  = data['l_hip_yaw_pitch_joint']
    q[1]  = data['r_hip_roll_joint']
    q[2]  = data['r_hip_pitch_joint']
    q[3]  = data['r_knee_pitch_joint']
    q[4]  = data['r_ankle_pitch_joint']
    q[5]  = data['r_ankle_roll_joint']
    q[6]  = data['l_hip_roll_joint']
    q[7]  = data['l_hip_pitch_joint']
    q[8]  = data['l_knee_pitch_joint']
    q[9]  = data['l_ankle_pitch_joint']
    q[10] = data['l_ankle_roll_joint']
    q[11] = data['r_shoulder_pitch_joint']
    q[12] = data['r_shoulder_roll_joint']
    q[13] = data['r_elbow_yaw_joint']
    q[14] = data['r_elbow_roll_joint']
    q[15] = data['l_shoulder_pitch_joint']
    q[16] = data['l_shoulder_roll_joint']
    q[17] = data['l_elbow_yaw_joint']
    q[18] = data['l_elbow_roll_joint']
    q[19] = data['head_yaw_joint']
    q[20] = data['head_pitch_joint']

    a[0]  = data['acc_x']
    a[1]  = data['acc_y']
    a[2]  = data['acc_z']

    Fl[0]  = data['l_fsr_press_x']
    Fl[1]  = data['l_fsr_press_y']
    Fl[2]  = data['l_fst_total_w']

    Fr[0]  = data['r_fsr_press_x']
    Fr[1]  = data['r_fsr_press_y']
    Fr[2]  = data['r_fst_total_w']

    if history:
        ax.append(a[0])
        ay.append(a[1])
        az.append(a[2])

        Flx.append(Fl[0])
        Fly.append(Fl[1])
        Flz.append(Fl[2])

        Frx.append(Fr[0])
        Fry.append(Fr[1])
        Frz.append(Fr[2])

PyNAO.onSensorData = sensor_data

def reset_history():
    global q
    global a
    global Fl
    global Fr
    q  = numpy.zeros(21)
    a  = numpy.zeros(3)
    Fl = numpy.zeros(3)
    Fr = numpy.zeros(3)

    ax  = []
    ay  = []
    az  = []

    Flx = []
    Fly = []
    Flz = []

    Frx = []
    Fry = []
    Frz = []
        
    

# Conversion Functions

def gen_joint_dict(q):
    g = {};
    g['head_yaw_joint']         = q[19]
    g['head_pitch_joint']       = q[20]
    g['l_shoulder_pitch_joint'] = q[15]
    g['l_shoulder_roll_joint']  = q[16]
    g['l_elbow_yaw_joint']      = q[17]
    g['l_elbow_roll_joint']     = q[18]
    g['l_hip_yaw_pitch_joint']  = q[0]
    g['l_hip_roll_joint']       = q[6]
    g['l_hip_pitch_joint']      = q[7]
    g['l_knee_pitch_joint']     = q[8]
    g['l_ankle_pitch_joint']    = q[9]
    g['l_ankle_roll_joint']     = q[10]
    g['r_hip_yaw_pitch_joint']  = q[0]
    g['r_hip_roll_joint']       = q[1]
    g['r_hip_pitch_joint']      = q[2]
    g['r_knee_pitch_joint']     = q[3]
    g['r_ankle_pitch_joint']    = q[4]
    g['r_ankle_roll_joint']     = q[5]
    g['r_shoulder_pitch_joint'] = q[11]
    g['r_shoulder_roll_joint']  = q[12]
    g['r_elbow_yaw_joint']      = q[13]
    g['r_elbow_roll_joint']     = q[14]
    return(g)

# Sensory Functions


# Actuating Functions

def take_rarm_to(qd):
    '''
    takes the right arm to the given configuration qd.
    qd must be a vector of 4 elements
    '''
    assert len(qd) == 4
    q[11:15] = qd[0:4]    
    take_robot_to(q)

def take_larm_to(qd):
    '''
    takes the left arm to the given configuration qd.
    qd must be a vector of 4 elements
    '''
    assert len(qd) == 4
    q[15:19] = qd[0:4]    
    take_robot_to(q)

def take_robot_to(qd):
    g = gen_joint_dict(qd)
    PyNAO.moveBodyWithJointPos( **g )    

