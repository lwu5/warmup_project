#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3, Twist

class DriveInASquare(object):
    """ This note drive the robot in a square """

    def __init__(self):
        rospy.init_node('drive_square')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    def run(self):
        rospy.sleep(1)
        my_linear = Vector3(1.0, 0.0, 0.0)
        my_angular = Vector3(0.0, 0.0, 0.0)
        my_twist = Twist(linear = my_linear, angular = my_angular) 

        i = 0
        while i < 4:
            my_twist.linear.x = 1.0
            my_twist.angular.z = 0.0
            self.publisher.publish(my_twist)
            rospy.sleep(3)
            my_twist.linear.x = 0.0
            my_twist.angular.z = 9.3
            self.publisher.publish(my_twist)
            rospy.sleep(1.1)
            i = i+1

        my_twist.linear.x = 0.0
        my_twist.angular.z = 0.0
        self.publisher.publish(my_twist)

if __name__ == '__main__':
    node = DriveInASquare()
    node.run()