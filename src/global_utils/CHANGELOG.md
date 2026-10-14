# Global Utils


___
# Version 0.1.0
Keep all global use for alpha drone, including system config, utilities and outside libs.
- Added global ALLOW_DEBUG macro.
- Added some Sensor::VOXEL_ settings.
- Removed the clock intentional skew settings, now clock are more rounded numbers.
- Added Physisc namespace for Gazebo physisc.
- LiDAR down and 2D topic now use pointcloud instead of lazer scan.
- Added Sensor info for LiDAR down and 2D instead of dynamically read from msg.
___