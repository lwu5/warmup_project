#!/usr/bin/env python3


# TOPICS:
#   cmd_vel: publish to, used for setting robot velocity
#   scan   : subscribing, where the person is

import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist, Vector3

# How close we will get to the person.
distance = 0.5

class PersonFollower(object):
    """ This node walks the robot to person and stops """

    def __init__(self):
        # Start rospy node.
        rospy.init_node("person_follower")

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
        # Determine direction of robot and its closeness to a person by looking 
        #   at scan data to find the minimum non_zero angule and value around 
        #   the robot, set linear and angular velocity based on that information, 
        #   and publish to cmd_vel.

        # non_zero flag: if the LiDAR detacts any non_zero value around robot,
        #   this value would be set to 1; otherwise, 0.
        non_zero = 0
    
        # find the minimum non_zero angle around robot and that is where the
        #   person is.
        for i in range(0,360):
            if (data.ranges[i] > 0): # we only care about non_zero value
                if (non_zero == 0): # set index of first non_zero value as min_i
                    min_i = i
                else: # there's already a min_i, we need to compare with it
                    if (data.ranges[i] < data.ranges[min_i]): # keep tracking of min_i
                        min_i = i
                non_zero = 1

        if (non_zero == 0): # if nothing arond robot, the robot does nothing
            self.twist.angular.z = 0
            self.twist.linear.x = 0
        else: # there's a person / object around robot
            k_a = -0.08 
            if (min_i > 179):
                e_a = 359 - min_i # error positive
            else:
                e_a = 0 - min_i # error negative
            if e_a > 20: # if the robot's direction is out of 20 degree buffer zone
                         #  regarding to absolute 0 degree, we change angular velocity
                         #  to make it faces person
                self.twist.angular.z = k_a * e_a
            else: # robot faces the person, no need to change direction
                self.twist.angular.z = 0

        if (data.ranges[min_i] > distance): # go forward if not close enough to the person
            k_l = -0.3
            e_l = (distance - data.ranges[min_i])
            self.twist.linear.x = k_l * e_l
        else: # close enough to the person, stop.
            self.twist.linear.x = 0
            
        # Publish msg to cmd_vel.
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = PersonFollower()
    node.run()