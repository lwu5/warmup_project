#!/usr/bin/env python3
import rospy

# msgs needed for /cmd_vel
from geometry_msgs.msg import Vector3, Twist

class DriveInASquare(object):
    """ This note drives the robot in a square """

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('drive_square')
        # setup publisher to the cmd_vel ROS topic
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    def run(self):
        # allow the publisher enough time to set up
        rospy.sleep(1)
        # initialize and setup the Twist message
        my_linear = Vector3(1.0, 0.0, 0.0)
        my_angular = Vector3(0.0, 0.0, 0.0)
        my_twist = Twist(linear = my_linear, angular = my_angular) 

        i = 0
        # the loop runs 4 times,
        # each iteration includes:
        #   3 sec straight w/ 1.0 linear & 0 angular velocity
        #   1.1 sec turn around w/ 0 linear & 9.3 angular velocity
        while i < 4:
            # go straight
            my_twist.linear.x = 1.0
            my_twist.angular.z = 0.0
            self.publisher.publish(my_twist) # publish the msg
            rospy.sleep(3)
            # turn around
            my_twist.linear.x = 0.0
            my_twist.angular.z = 9.3
            self.publisher.publish(my_twist) # publish the msg
            rospy.sleep(1.1)
            i = i+1

        # stop the robot afterwards
        my_twist.linear.x = 0.0
        my_twist.angular.z = 0.0
        self.publisher.publish(my_twist) # publish the msg

if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveInASquare()
    node.run()