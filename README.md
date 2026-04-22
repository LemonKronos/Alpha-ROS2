The ReactiveOA is WORKING
___
1. I change the colcon build instead of having `--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --parallel-workers 8` in each task, I do a `colcon_defaults.yaml` file
2. Update C_cpp to use `compile_commands.json` instead of include path
3. Make `scripts/setupROS2Build.sh` for source ROS2 in build bash terminal