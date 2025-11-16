# odom_path

Simple ROS2 node that subscribes to odometry messages and publishes a path for visualization in RViz.

## Features

- Subscribes to odometry messages (nav_msgs/Odometry)
- Publishes accumulated path (nav_msgs/Path)
- Fully remappable topics via parameters
- Configurable maximum path length to prevent memory issues
- Lightweight and efficient

## Building

```bash
cd /path/to/workspace
colcon build --packages-select odom_path
source install/setup.bash
```

## Usage

### Run with default settings

```bash
ros2 run odom_path odom_path_node
```

Default topics:
- Input: `/mavros/odometry/out`
- Output: `odom_path`

### Run with custom topics

```bash
ros2 run odom_path odom_path_node --ros-args \
    -p odom_topic:=/custom/odom \
    -p path_topic:=/custom/path \
    -p max_path_length:=2000
```

### Using launch file

```bash
ros2 launch odom_path odom_path.launch.py
```

With custom arguments:

```bash
ros2 launch odom_path odom_path.launch.py \
    odom_topic:=/custom/odom \
    path_topic:=/custom/path \
    max_path_length:=2000
```

## Parameters

- `odom_topic` (string, default: `/mavros/odometry/out`): Input odometry topic
- `path_topic` (string, default: `odom_path`): Output path topic
- `max_path_length` (int, default: 1000): Maximum number of poses to keep in path

## Visualization in RViz

1. Add a Path display
2. Set the topic to `odom_path` (or your custom path topic)
3. Adjust the color and style as desired
