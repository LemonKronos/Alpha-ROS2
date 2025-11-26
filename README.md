# ROS2
Contain ROS2 logic for Alpha

BUGs:
- ReactiveOA node:
    - Unstable: cause lidar 2D read drone 2D
- Record Arobatic:
    - Depth cam just black
    - It take too much network banwidth
- Finalize Control:
    - Attitude setpoint never good enough


TODOs:
- Record Acrobatic:
    - Name episode to numerically increase, not a timebase name
    - Release network banwidth
    - Fix depth cam record
- Finalize Control:
    - Try refine Attitude setpoint
