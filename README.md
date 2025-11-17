# ROS2
Contain ROS2 logic for Alpha

BUGs:
- Global: wrong frame, currently using PX4 FRD
- ReactiveOA node:
    - Actract to obstacles that are too close
    - Unstable
    - Lose altitude after Reactive OA take control

TODOs:
- Change all frame to ROS2 FLU from PX4 FRD