# Omron MoMa with OnRobot Grippers

Combined LD250 and TM12X descriptions and MoveIt configurations are available
for OnRobot RG2, RG6, VG10, and four-cup VGC10. The per-gripper MoveIt packages
share common semantic and launch logic but contain their own robot semantics,
kinematics, and controller configuration.

## Build

Install the ROS 2 Jazzy dependencies of `onrobot_grippers`, `omron_moma`,
MoveIt 2, Gazebo Harmonic, and `gz_ros2_control`, then build the packages:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/grasping_omron_moma_onrobot
source install/setup.bash
```

The build command assumes the other dependencies have already been built in the
workspace and sourced. Use `colcon test --base-paths src/grasping_omron_moma_onrobot`
to run the combination checks.

## Gazebo

Start the fixed-base Gazebo simulation and MoveIt with one of `rg2`, `rg6`,
`vg10`, or `vgc10`:

```bash
ros2 launch grasping_onrobot_omron_moma gazebo.launch.py gripper:=rg2
```

For a headless server, add `gz_args:="-r -s"`. Set `motion_execution:=true`
to include the generic grasping motion-execution and feasibility nodes. The
world is empty and the LD250 base is fixed; mobile driving, Nav2, a wrist
camera, and physical hardware bringup are not configured here.

For RG2 and RG6, the simulated action server exposes
`/onrobot_rg2/gripper_command` or `/onrobot_rg6/gripper_command`. Its position
is an approximate aperture in meters and `max_effort` must be zero:

```bash
ros2 action send_goal /onrobot_rg2/gripper_command control_msgs/action/GripperCommand \
	'{command: {position: 0.03, max_effort: 0.0}}'
```

VG10 and VGC10 use their fixed-link models; suction and vacuum dynamics are not
simulated. The OnRobot hardware action servers do not yet execute commands.

## Calibration

`mount_xyz`, `mount_rpy`, `tcp_xyz`, and `tcp_rpy` are launch arguments in
meters and radians. The TCP transform is relative to the gripper base link.
Their zero defaults are placeholders, not calibrated adapter or tool poses.
Calibrate the mounting, TCP, joint limits, and collision exclusions before any
hardware use. The RG linkage collision exclusions are intended for simulation
and require validation on the actual assembly.

Each MoveIt package has a `move_group.launch.py` entry point, for example:

```bash
ros2 launch ld250_tm12x_rg2_moveit_config move_group.launch.py
```

That launch requires an existing robot state publisher and trajectory controller,
such as those started by the Gazebo launch; it does not start hardware.
