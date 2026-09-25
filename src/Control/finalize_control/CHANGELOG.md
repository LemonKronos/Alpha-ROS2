# Finalize Control



# Version 0.1.0
Last ROS2 node to bring signal to PX4

## 0.1.0
- Added in_hover to send position when in hover to avoid altitude drip

## 0.1.1
- Now use `MultiThreadedExecutor` with sensor and control mutual-exclusive callback group
- Using px4 msg namespace
- Added time analyzer
- Optimizing Eigen math
- Still contain unstable offboard control bug
