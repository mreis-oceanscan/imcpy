# Multi-Vehicle Updater Node Parameters

The `vehicle_updater` node now supports multiple vehicles with configurable ROS2 parameters:

## Parameters

### `frequency` (double, default: 10.0)
- **Description**: Node update frequency in Hz
- **Range**: > 0.0
- **Purpose**: Controls how often the node publishes vehicle transforms and joint states

### `ports` (integer_array, default: [6004])
- **Description**: List of port numbers for localhost connections
- **Format**: Array of integers representing port numbers
- **Purpose**: Each port corresponds to a different vehicle (lauv1, lauv2, etc.)

### `namespace_prefix` (string, default: "lauv")
- **Description**: Vehicle namespace prefix for naming
- **Format**: String that will be combined with incremental numbers
- **Purpose**: Creates vehicle names like lauv1, lauv2, etc. based on the number of ports

## Multi-Vehicle Features

- **Separate Transforms**: Each vehicle gets its own transform tree (`lauv1/base_link`, `lauv2/base_link`, etc.)
- **Individual Joint States**: Each vehicle publishes to its own joint state topic (`/lauv1/joint_states`, `/lauv2/joint_states`, etc.)
- **Independent Robot Models**: Each vehicle can have its own URDF with proper namespacing
- **Port-to-Vehicle Mapping**: Port 6004 → lauv1, Port 6005 → lauv2, etc.

## Usage Examples

### 1. Using Multi-Vehicle Launch File

```bash
# Launch with default 2 vehicles
ros2 launch lauv multi_vehicle.launch.py

# Launch with custom parameters
ros2 launch lauv multi_vehicle.launch.py ports:="[6004, 6005, 6006]" namespace_prefix:="auv"

# Dynamic multi-vehicle launch (recommended)
ros2 launch lauv dynamic_multi_vehicle.launch.py ports:="[6004, 6005]"
```

### 2. Using Single Node Launch File

```bash
# Default single vehicle
ros2 launch lauv update_vehicle.launch.py

# Multiple vehicles with custom namespace
ros2 launch lauv update_vehicle.launch.py ports:="[6004, 6005, 6006]" namespace_prefix:="robot"
```
```

### 3. Setting Parameters at Runtime

```bash
# Change frequency
ros2 param set /vehicle_updater frequency 25.0

# List current parameters
ros2 param list /vehicle_updater

# Get parameter values
ros2 param get /vehicle_updater ports
ros2 param get /vehicle_updater namespace_prefix
```

### 4. Direct Node Execution with Parameters

```bash
# Run node with multiple vehicles
ros2 run lauv update_vehicle.py --ros-args -p frequency:=15.0 -p ports:="[6004, 6005]" -p namespace_prefix:="lauv"
```

## ROS2 Topics Published

For each vehicle (e.g., lauv1, lauv2), the following topics are published:

- `/lauv1/joint_states` - Joint state information for vehicle 1
- `/lauv2/joint_states` - Joint state information for vehicle 2
- `/tf` - Transform tree with frames like `lauv1/base_link`, `lauv2/base_link`

## Vehicle Identification

The node determines which vehicle an IMC message belongs to using:
1. **Source Entity ID** - If available in the IMC message
2. **Sequential Assignment** - As a fallback method

You may need to customize the `determine_vehicle_from_message()` method based on your specific IMC setup.

## Notes

- The `frequency` parameter can be changed dynamically at runtime
- Changes to the `ports` or `namespace_prefix` parameters require a node restart to take effect
- If no valid connections are configured, the node will fall back to `localhost:6004` for `lauv1`
- All connections are automatically made to localhost - only port numbers need to be specified
- Invalid port numbers will be logged as errors but won't prevent the node from starting
- Each vehicle gets its own robot_state_publisher when using the multi-vehicle launch files
- Vehicle naming follows the pattern: `{namespace_prefix}{incremental_number}` (e.g., lauv1, lauv2)

## Launch Files

- `update_vehicle.launch.py` - Single node launcher (vehicles without robot_state_publisher)
- `multi_vehicle.launch.py` - Complete multi-vehicle setup with robot_state_publishers
- `dynamic_multi_vehicle.launch.py` - Dynamic vehicle creation based on ports parameter

## Example Configuration File

See `src/lauv/config/vehicle_params.yaml` for a complete parameter configuration example.