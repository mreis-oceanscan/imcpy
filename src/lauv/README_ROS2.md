# LAUV ROS2 Description Package

This is a ROS2 version of the Light Autonomous Underwater Vehicle (LAUV) description package, converted from the original ROS1 UUV Simulator package.

## Overview

This package provides URDF/xacro robot description files and RViz2 visualization for the LAUV underwater robot. It includes:

- Simplified URDF model compatible with ROS2
- RViz2 configuration for robot visualization  
- Launch files for easy startup
- Demo script showing animated joint movements

## Features

- **Base link**: Main torpedo-shaped body with proper inertial properties
- **Fins**: Four control fins (top, bottom, port, starboard) with joint control
- **Propeller**: Rear propulsion unit with continuous rotation
- **Meshes**: Uses original DAE/STL mesh files for realistic visualization
- **RViz2**: Pre-configured visualization setup

## Requirements

- ROS2 (tested on Humble/Iron)
- Standard ROS2 packages:
  - `robot_state_publisher`
  - `joint_state_publisher`
  - `joint_state_publisher_gui`
  - `xacro`
  - `rviz2`
  - `launch_ros`

## Installation

1. Create a ROS2 workspace (if you don't have one):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone or copy this package to your workspace:
```bash
# If you have this as part of a larger repository:
cp -r /path/to/lauv-simplified ~/ros2_ws/src/

# Or clone if it's a separate repository:
# git clone <repository_url> ~/ros2_ws/src/lauv-simplified
```

3. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select lauv-simplified
source install/setup.bash
```

## Usage

### Basic Visualization

Launch RViz2 with the robot model:

```bash
ros2 launch lauv-simplified display.launch.py
```

This will open:
- RViz2 with the robot model displayed
- Joint State Publisher GUI for manual joint control
- Robot State Publisher for TF broadcasting

### Arguments

You can customize the launch with arguments:

```bash
# Change robot namespace
ros2 launch lauv-simplified display.launch.py namespace:=my_lauv

# Enable simulation time
ros2 launch lauv-simplified display.launch.py use_sim_time:=true
```

### Demo Animation

Run the demo script to see animated joint movements:

```bash
# In terminal 1 - Start the robot description
ros2 launch lauv-simplified robot_state_publisher.launch.py

# In terminal 2 - Start RViz2
rviz2 -d ~/ros2_ws/src/lauv-simplified/rviz/lauv_rviz2.rviz

# In terminal 3 - Run the demo animation
python3 ~/ros2_ws/src/lauv-simplified/scripts/lauv_demo.py
```

The demo will show:
- Oscillating fin movements simulating control inputs
- Continuous propeller rotation
- Real-time joint state updates in RViz2

## Package Structure

```
lauv-simplified/
├── CMakeLists.txt          # ROS2 build configuration
├── package.xml             # Package metadata and dependencies
├── urdf/
│   └── lauv_ros2.xacro    # Main robot description file
├── meshes/                 # Visual and collision meshes
│   ├── body.dae           # Main body visual mesh
│   ├── body.stl           # Main body collision mesh
│   ├── red_fin.dae        # Top/bottom fin meshes
│   ├── black_fin.dae      # Port/starboard fin meshes
│   └── propeller.dae      # Propeller mesh
├── launch/                 # ROS2 launch files
│   ├── display.launch.py   # Complete visualization launch
│   └── robot_state_publisher.launch.py  # Basic robot description
├── rviz/
│   └── lauv_rviz2.rviz    # RViz2 configuration
└── scripts/
    └── lauv_demo.py       # Animation demo script
```

## Robot Specifications

- **Mass**: 18 kg
- **Length**: ~1.3 m
- **Radius**: ~0.1 m
- **Fins**: 4 control surfaces with ±0.5 radian range
- **Propeller**: Continuous rotation thruster

## Customization

### Modifying the Robot

Edit `urdf/lauv_ros2.xacro` to:
- Change physical properties (mass, dimensions)
- Add new links/joints
- Modify visual appearance
- Add sensors or other components

### Adding Sensors

You can extend the URDF to include sensors like:
- Cameras
- IMU
- Depth sensors
- Sonar

Example sensor addition:
```xml
<link name="$(arg namespace)/camera_link">
  <!-- Camera link definition -->
</link>

<joint name="$(arg namespace)/camera_joint" type="fixed">
  <parent link="$(arg namespace)/base_link"/>
  <child link="$(arg namespace)/camera_link"/>
  <origin xyz="0.5 0 0" rpy="0 0 0"/>
</joint>
```

## Troubleshooting

### Common Issues

1. **Mesh files not found**: Ensure the package is properly installed and sourced
2. **RViz2 not showing robot**: Check that robot_description topic is being published
3. **Joint movements not visible**: Verify joint_state_publisher is running

### Debugging Commands

```bash
# Check if robot description is published
ros2 topic echo /lauv/lauv-simplified

# List all topics
ros2 topic list

# Check joint states
ros2 topic echo /lauv/joint_states

# Verify TF tree
ros2 run tf2_tools view_frames
```

## Integration with Simulation

This package provides the robot description that can be used with:
- Gazebo simulation
- Navigation stack
- Control systems
- Custom underwater simulation environments

## Contributing

Feel free to contribute improvements:
- Enhanced robot models
- Additional sensors
- Better visualization
- Documentation improvements

## License

Apache 2.0 - See LICENSE file for details.

## Original Credits

Based on the original LAUV model from:
- [LSTS - Porto University](https://www.lsts.pt/)
- [OceanScan-MST](http://www.oceanscan-mst.com/)
- [UUV Simulator](https://github.com/uuvsimulator/uuv_simulator)