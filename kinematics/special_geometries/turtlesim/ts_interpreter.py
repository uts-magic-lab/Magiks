#!/usr/bin/env python

import roslib; 
# roslib.load_manifest('turtle_control')
import rospy, time
from turtlesim.msg import Pose

x = 7.0
y = 4.0
t = 1.2

def callback(data):
    global x,y,t    
    x = data.x    
    y = data.y
    t = data.theta

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, callback)

if __name__ == '__main__':
    listener()
    time.sleep(1)
    print "x = ", x
    print "y = ", y   
    print "t = ", t

