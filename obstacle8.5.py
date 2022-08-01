#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
from sensor_msgs.msg import LaserScan

START = 1
LINEAR_VEL = 0.1
STOP_DISTANCE = float
STOP_DISTANCE = 0.5
SMALL_STOP_DISTANCE = 0.3
SMALLER_STOP_DISTANCE = 0.2
front = START 
left = START
right = START
fleft = START
fright = START
rospy.init_node('object_avoidance')

def callback(msg):
    global front
    global left
    global right
    global fleft
    global fright

    front = msg.ranges[0]
    left = msg.ranges[90]
    right = msg.ranges[-90]
    fleft = min(msg.ranges[40:50:1])
    fright = min(msg.ranges[-40:-50:-1])
if __name__ == '__main__':
    try:
        twist = Twist()
        
        while not rospy.is_shutdown():
            pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
            sub = rospy.Subscriber('scan', LaserScan, callback)

            #big turn
            if front < STOP_DISTANCE:
                rospy.loginfo('Frontal is %s', front)
                rospy.loginfo('Left is %s', left)
                rospy.loginfo('right is %s', right)   
                rospy.loginfo('fleft is %s',fleft)
                rospy.loginfo('fright is %s',fright)

                #choosing a direction
                #if more space on the left
                if left > right:
                    #turn left until front is greater than stop distance
                    while front < STOP_DISTANCE:
                        twist.linear.x = 0.0
                        twist.angular.z = 3.0
                        rospy.loginfo('turn left')
                        pub.publish(twist)
                elif right > left:
                    #turn right until front is greater than stop distance
                    while front < STOP_DISTANCE:
                        twist.linear.x = 0.0
                        twist.angular.z = -3.0
                        rospy.loginfo('turn left')
                        pub.publish(twist)

            elif front > STOP_DISTANCE:
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
                rospy.loginfo('forward')
                pub.publish(twist)

            elif fleft <SMALL_STOP_DISTANCE or fright <SMALL_STOP_DISTANCE:
                if fleft > fright:
                    while fright < SMALL_STOP_DISTANCE:
                        twist.linear.x = 0.0
                        twist.angular.z = 3.0
                        rospy.loginfo('slight left')
                        pub.publish(twist)

                elif fright > fleft:
                    while fleft < SMALL_STOP_DISTANCE:
                        twist.linear.x =  0.0
                        twist.angular.z = -3.0
                        rospy.loginfo('slight right')
                        pub.publish(twist)

            elif left < SMALLER_STOP_DISTANCE or right < SMALLER_STOP_DISTANCE:
                if left > right:
                    while right < SMALLER_STOP_DISTANCE:
                        twist.linear.x = 0.0
                        twist.angular.z = 3.0
                        rospy.loginfo('adjust left')
                        pub.publish(twist)

                elif right > left:
                    while left < SMALLER_STOP_DISTANCE:
                        twist.linear.x =  0.0
                        twist.angular.z = -3.0
                        rospy.loginfo('adjust right')
                        pub.publish(twist)


    except rospy.ROSInterruptException:
            pass
