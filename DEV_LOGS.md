# Dev logs

## Current ⚠️⚠️⚠️
- Fix the auto training error: the problem is `self.gz_node = GZNode()` keep making new TCP and not clean up, thus caught `strerror: Too many open files`. Not fix yet.
- Multi threaded Gazebo physics? We got fast CPU physics first

## Ready the simulation for RL - Acrobatic OA node
- Working on the Acrobatic OA node, I realize that I need to completely ditch PX4 in training. Thus, I need to make end to end replacement ROS2 node to act as PX4, including odometry and actuator control.
- Addded 2 node `gz_ground_truth` to publish Gazebo truth odometry and `px4_emulator` to act as PX4 endpoint in training, provide arm state and send actuator command dirrectly to Gazebo. **Noise are not being handled yet**.
- Arobatic OA, `WorldHandling` still try to zero drone speed, ready for switch to `reset_all` instead.
- `ToyTrainer` now able to spam the `Step` and collect `StepResult`.
- Making the training environment more resilient and auto reset.
- House keeping on logs and path. Also appiled Fast DDS config for lo multicast-allow Gazebo-ROS2 communication fully offline.
- Make the setup fully local host and not depend on network (it just a simple `GZ_IP` set).

