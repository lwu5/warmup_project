# warmup_project

## Driving in a Square

04/04/2022

1. High-Level Description

- This subproject drives the turtlebot in a square. I use a loop that iterates four times to complete this behavior, and, for each iteration, I let the robot go straight for couple seconds and then make a 90-degree turn for couple seconds and stop the robot afterwards.
   
2. Code Explanation

- Function `__init__` : initialize the ROS node and set up publisher to the cmd_vel ROS topic
- Function `run` : first, sleep for a sec to allow the publisher enough time to set up and initialize and setup the Twist message; then, there is a loop that runs four times, and for each iteration, we firstly set linear vel to 1.0 and angular vel to 0 to let robot go straight for 3 sec, and secondly, set linear vel to 0 and angular vel to 9.3 to let robot turn 90 degrees (accomplished in 1.1 sec using `rospy.sleep()`); at the end, set both linear and angular velocity to 0 to stop the robot.
- Function `__main__` : instaniate the ROS node and run it

3. Behavior (Gif)

- ![IMG_1078](https://user-images.githubusercontent.com/59663733/161677416-e3756877-e1a9-4ddb-af9c-ac87dc773179.gif)

## Wall Follower

1. High-Level Description

- This subproject let the robot find a wall and follow it roughly with a certain distance. I first identify if there's any object around the robot (by around, I mean within designated distance + buffer); if no, simply drive robot forward to find / get closer to the wall; if yes, we use PID to control the angular velocity to keep the wall on the right side (270 degress) of the robot with roughly a designated distance.
   
2. Code Explanation

- Function `__init__` : initialize the ROS node and set up publisher to the cmd_vel ROS topic
- Function `run` : first, we iterate through `data.ranges` list to identify if there's any object around robot (i.e., any non_zero value in the list); if yes, we keep track of the index of the min non_zero value `min_i`; if not, we set the `non_zero` flag to 0; and secondly, based on the `min_i` and `non_zero` values we do the following: if there's nothing around robot (i.e., `non_zero == 0`) or the robot is too far from the wall (i.e., `data.ranges[min_i]` too big), we set the angular velocity to zero to not change robot's direction; otherwise, we use PID on robot's angular velocity - both wall's angular location (i.e., `min_i`) and wall's distance to the robot (i.e., `data.ranges[min_i]`) contribute to robot's angular velocity. for the linear velocity, we set it a positve constant value because we always want to drive the robot forward and publish the twist at the end. 

- Function `__main__` : instaniate the ROS node and run it

3. Behavior (Gif)
- ![ezgif-3-6e1b49a054](https://user-images.githubusercontent.com/59663733/162842552-91a5c656-ffb0-47f1-b535-7150ee869d66.gif)

## Person Follower

1. High-Level Description
   
2. Code Explanation

3. Behavior (Gif)
- ![img-1260-2_L4oXYl2i](https://user-images.githubusercontent.com/59663733/162643201-69591d7d-b445-4ab3-ae7f-d0ad226d397e.gif)

## Challenges!

## Future Work

## Takeaway
