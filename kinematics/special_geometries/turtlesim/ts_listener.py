'''   Header
@file:          ts_listener.py
@brief:    	    Contains simplified functions to get information
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
Start date:     9 April 2014
Last Revision:  9 April 2014

Changes from ver 1.0:

Note; Before running this program, make sure that:
    1- ROS is installed and roscore is running
    2- turtlesim node is running
when you type rostopic list. you should see these topics in the list:

/rosout
/turtle1/color_sensor
/turtle1/command_velocity
/turtle1/pose

'''
import roslib, rospy, numpy, math; 
from turtlesim.msg import Pose
from turtlesim.msg import Velocity

x     = 0.0
y     = 0.0
theta = 0.0

v_x   = 0.0
v_y   = 0.0
v     = 0.0
omega = 0.0

def update_pose(data):
    global x,y, theta    
    x     = data.x    
    y     = data.y
    theta = data.theta

def update_speed(vel):
    global v_x, v_y, v, omega

    v   = vel.linear
    v_x = v*math.cos(theta)
    v_y = v*math.sin(theta)
    omega = vel.angular

def listen():
    rospy.init_node('ts_listener', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, update_pose)
    rospy.Subscriber('/turtle1/command_velocity', Velocity, update_speed) # for ROS Hydro use '/turtle1/cmd_vel'
    

def current_position():
    return numpy.array([x, y])

def current_angle(in_degrees = True):
    if in_degrees:
        gain = math.pi/180.0
    else:
        gain = 1.0     
    return theta*gain


