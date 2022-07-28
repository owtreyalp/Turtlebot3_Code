#!/usr/bin/env python      #Specify work enviorment



import rospy               #Import packages
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

linear_vel = 0.1            #Define variables
stop_distance = 0.5
twist = Twist()

rospy.init_node('stop')     #Initialize ros node


def callback(msg):          #Create function named callback with parameter msg

    if msg.ranges[0] < stop_distance:           #if distance < stop distance
        twist.linear.x = 0                      #Tells turtlebot how to move
        twist.angular.z = 0.0
        rospy.loginfo ('Stop')
        pub.publish(twist)                      #Publishes twist commands

    else:
        twist.linear.x = linear_vel
        twist.angular.z = 0.0
        pub.publish(twist)
        rospy.loginfo('Go')


pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)       #Create publisher
sub = rospy.Subscriber ('scan', LaserScan, callback)        #Create subscriber
rospy.spin()             #Tells subscriber to keep looking for messages   

        