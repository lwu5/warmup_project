#!/usr/bin/env python3


# TOPICS:
#   cmd_vel: publish to, used for setting robot velocity
#   scan   : subscribing, where the wall is

import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist, Vector3

# We keep the robot (distance +/- buffer) from the wall.
distance = 0.3
buffer = 0.2

class WallFollower(object):
    """ This node walks the robot to wall and move around the wall """

    def __init__(self):
        # Start rospy node.
        rospy.init_node("wall_follower")

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Get a publisher to the cmd_vel topic.
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

    def process_scan(self, data):
        # Determine direction and closeness to wall by looking at scan data 
        #   to find the minimum non_zero angule and value around the robot,
        #   set angular velocity based on that information to keep the wall
        #   on the right side of robot and roughly within a certain distance
        #   while the linear velocity keeps constant, and publish to cmd_vel.
        
        # non_zero flag: if the LiDAR detacts any non_zero value around robot,
        #   this value would be set to 1; otherwise, 0.
        non_zero = 0

        # find the minimum non_zero angle around robot and that is where the
        #   wall is.
        for i in range(0,360):
            if (data.ranges[i] > 0): # we only care about non_zero value
                if (non_zero == 0): # set index of first non_zero value as min_i
                    min_i = i
                else: # there's already a min_i, we need to compare with it
                    if (data.ranges[i] < data.ranges[min_i]): # keep tracking of min_i
                        min_i = i
                non_zero = 1

        if (non_zero == 0): # if nothing arond robot, the robot does not change direction
            self.twist.angular.z = 0
        else: # the robot finds a wall
            if (data.ranges[min_i] <= distance + buffer): # robot is within the ideal
                                                          #     distance from the wall
                k_a = -0.1
                e_a = 270 - min_i # e_a controls robot's direction:
                                  #    we always want the wall to be at robot's 270 degrees 
                                  #    position (i.e., always on the right side of the robot).
                                  #    we change robot's angular velocity if the wall (i.e.
                                  #    min_i) is not at 270 degrees.

                k_d = 0.1
                e_d = distance - data.ranges[min_i] # e_d controls robot's distance from the wall:
                                                    #    we always want the robot to be roughly  
                                                    #    at a certain distance from the wall
                                                    #    if it's too close to the wall, we change
                                                    #    its direction towards outside; if it's too
                                                    #    far from the wall, we change its direction
                                                    #    towards inside; if within the buffer range,
                                                    #    this value does not affect robot's direciton

                # both e_a and e_d contributes to robot's angular velocity
                self.twist.angular.z = (k_a * e_a) + (k_d * e_d)
            else: # if robot is too far from the wall, we do not set angular velocity yet,
                  #    but simply drive it forward to find the wall first.
                self.twist.angular.z = 0

        self.twist.linear.x = 0.2 # robot always moves forward, regardless whether there
                                  #     there's anything around or not.

        # Publish msg to cmd_vel.
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = WallFollower()
    node.run()