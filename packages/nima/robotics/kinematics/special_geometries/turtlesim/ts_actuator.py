'''   Header
@file:          ts_actuator.py
@brief:    	    Contains simplified functions to manipulate turtlesim
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
Start date:     16 May 2014
Last Revision:  16 May 2014

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

def talk():
    global pub
    pub = rospy.Publisher('/turtle1/command_velocity', Velocity)
    rospy.init_node('ts_talker')

def send_speed(v = 1.0, omega = 1.0):
    vel = Velocity()
    vel.linear  = v
    vel.angular = omega
    '''
    Publish the speed
    '''
    pub.publish(vel)
    

