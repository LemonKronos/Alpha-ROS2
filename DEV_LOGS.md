# Dev logs

## Ready the simulation for RL
- Working on the Acrobatic OA node, I realize that I need to completely ditch PX4 in training. Thus, I need to make end to end replacement ROS2 node to act as PX4, including odometry and actuator control.
- Addded 2 node `gz_ground_truth` to publish Gazebo truth odometry and `px4_emulator` to act as PX4 endpoint in training, provide arm state and send actuator command dirrectly to Gazebo. **Noise are not being handled yet**.
