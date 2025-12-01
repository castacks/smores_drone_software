# ROS2 Bag Image Extractor

A utility to extract images from ROS2 bag files and save them as either individual PNG frames or as an MP4 video.

## Features

- Extract images from any image topic in a ROS2 bag file
- Save as individual PNG frames or MP4 video
- Configurable FPS for video output
- Automatic rosbag playback handling

## Installation

Build the package in your ROS2 workspace:

```bash
cd /external/smores_drone_software
colcon build --packages-select rosbag_image_extractor
source install/setup.bash
```

## Usage

### Save as Individual Images

```bash
ros2 run rosbag_image_extractor extract_images \
  -b /path/to/rosbag \
  -t /camera/image_raw \
  -o /output/directory \
  -f images
```

This will create PNG files named `frame_000000.png`, `frame_000001.png`, etc.

### Save as MP4 Video

```bash
ros2 run rosbag_image_extractor extract_images \
  -b /path/to/rosbag \
  -t /camera/image_raw \
  -o /output/directory \
  -f mp4 \
  --fps 30
```

This will create a file named `output.mp4` in the specified directory.

## Arguments

- `-b, --bag`: Path to the ROS2 bag file (required)
- `-t, --topic`: Image topic to extract (required)
- `-o, --output`: Output directory (required)
- `-f, --format`: Output format - either `images` or `mp4` (default: images)
- `--fps`: Frames per second for MP4 output (default: 30)

## Examples

### Extract thermal camera images

```bash
ros2 run rosbag_image_extractor extract_images \
  -b ~/bags/thermal_flight_001 \
  -t /thermal/left/image_raw \
  -o ~/extracted_thermal \
  -f images
```

### Create video from depth images

```bash
ros2 run rosbag_image_extractor extract_images \
  -b ~/bags/depth_test \
  -t /depth/image \
  -o ~/depth_video \
  -f mp4 \
  --fps 15
```

## Notes

- The script automatically starts and stops rosbag playback
- Images are converted to BGR8 format before saving
- For MP4 output, the video codec used is 'mp4v'
- Press Ctrl+C to stop extraction early
