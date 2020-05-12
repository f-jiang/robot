# `robot_base` launch files

- `base.launch`: Top-level launch file. Starts up all nodes required to interface with robot actuators and sensors, whether in real-life or simulation. Also loads robot description to allow for publishing of tf frames. Can be launched directly (will need to provide a `cmd_vel` msg).
- `drive_base.launch`: Launches nodes required to drive the robot, whether in real-life or simulation. Can be launched directly if wishing to drive the physical robot without sensors, and will need to provide a `cmd_vel` msg.
- `sensors_base.launch`: Launches sensors on the physical robot. Can be launched directly, although `tf` won't work.
- `gazebo.launch`: Launches Gazebo and spawns the robot along with simulated sensors. Not meant to be launched directly.
- `robot_hw.launch`: Launches firmware node and PID controllers on the physical robot. Not meant to be launched directly.
- `tune_pid.launch`: Utility for tuning robot's PID controllers.

