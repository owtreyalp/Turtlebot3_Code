#!/usr/bin/env python



from turtle import distance
import rospy
from geometry_msgs.msg import Twist
import math
from sensor_msgs.msg import LaserScan

LINEAR_VEL = 0.1
STOP_DISTANCE = 0.5
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE_R = STOP_DISTANCE + LIDAR_ERROR
SAFE_STOP_DISTANCE_L = STOP_DISTANCE + LIDAR_ERROR


class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()


#Basic Scan
    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []

        samples = len(scan.ranges)

        samples_view = 1

        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            scan_filter.append(scan.ranges[0])
        else:
             left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
             right_lidar_samples_ranges = samples_view//2

             left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
             right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
             scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] == 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0

        return scan_filter

#Scan Left

    def get_scan_left(self):
            scan = rospy.wait_for_message('scan', LaserScan)
            scan_filter_L = []

            samples = len(scan.ranges)

            samples_view = 1

            if samples_view > samples:
                samples_view = samples

            if samples_view is 1:
                scan_filter_L.append(scan.ranges[90])
            else:
                left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
                right_lidar_samples_ranges = samples_view//2

                left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
                #right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
                scan_filter_L.extend(left_lidar_samples)

            for i in range(samples_view):
                if scan_filter_L[i] == float('Inf'):
                    scan_filter_L[i] == 3.5
                elif math.isnan(scan_filter_L[i]):
                    scan_filter_L[i] = 0

            return scan_filter_L

#Scan Right

    def get_scan_right(self):
            scan = rospy.wait_for_message('scan', LaserScan)
            scan_filter_R = []

            samples = len(scan.ranges)

            samples_view = 1

            if samples_view > samples:
                samples_view = samples

            if samples_view is 1:
                scan_filter_R.append(scan.ranges[-90])
            else:
                #left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
                right_lidar_samples_ranges = samples_view//2

                #left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
                right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
                scan_filter_R.extend(right_lidar_samples)

            for i in range(samples_view):
                if scan_filter_R[i] == float('Inf'):
                    scan_filter_R[i] == 3.5
                elif math.isnan(scan_filter_R[i]):
                    scan_filter_R[i] = 0

            return scan_filter_R
   
   #########################################
   
    def obstacle(self):
        
        twist = Twist()
        turtlebot_moving = True

        while not rospy.is_shutdown():
            lidar_distances = self.get_scan()
            lidar_distances_left = self.get_scan_left()
            lidar_distances_right = self.get_scan_right()
            min_distance_left = min(lidar_distances_left)
            min_distance_right = min(lidar_distances_right)
            min_distance = min(lidar_distances)

            if min_distance < STOP_DISTANCE:
                if min_distance_left < min_distance_right:
                    if turtlebot_moving:
                        twist.linear .x = 0.0
                        twist.angular .z = -5.0
                        self._cmd_pub.publish(twist)
                        turtlebot_moving = False
                        rospy.loginfo('Turn Right')

                elif min_distance_left > min_distance_right:
                    if turtlebot_moving:
                        twist.linear.x = 0.0
                        twist.angular.z = 5.0
                        self._cmd_pub.publish(twist)
                        turtlebot_moving = False
                        rospy.loginfo('Turn Left')

            else:
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0 
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                rospy.loginfo('Distance to collision : %f meters', min_distance)
                rospy.loginfo('left distance : %s meters', lidar_distances_left )
                rospy.loginfo('right distance : %s meters', lidar_distances_right )

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()




