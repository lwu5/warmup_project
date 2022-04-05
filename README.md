# warmup_project

## Driving in a Square

1. High-Level Description

This subproject drives the turtlebot in a square. I use a loop that iterates four times to complete this behavior, and, for each iteration, I let the robot go straight for couple seconds and then make a 90-degree turn for couple seconds and stop the robot afterwards.
   
2. Code Explanation

- Function `__init__` : initialize the ROS node and set up publisher to the cmd_vel ROS topic
- Function `run` : first, sleep for a sec to allow the publisher enough time to set up and initialize and setup the Twist message; then, there is a loop that runs four times, and for each iteration, we firstly set linear vel to 1.0 and angular vel to 0 to let robot go straight for 3 sec, and secondly, set linear vel to 0 and angular vel to 9.3 to let robot turn 90 degrees (accomplished in 1.1 sec using `rospy.sleep()`); at the end, set both linear and angular velocity to 0 to stop the robot.
- Function `__main__` : instaniate the ROS node and run it

3. Behavior (Gif)

![IMG_1078](https://user-images.githubusercontent.com/59663733/161677416-e3756877-e1a9-4ddb-af9c-ac87dc773179.gif)

## Wall Follower

1. High-Level Description
   
2. Code Explanation

3. Behavior (Gif)

## Person Follower

1. High-Level Description
   
2. Code Explanation

3. Behavior (Gif)

## Challenges

## Future Work

## Takeaway
