# Cloud to Map ROS Package

A ROS package that converts point cloud data (PCD files) to 2D occupancy grid maps. The package provides functionality to load PCD files and publish them as occupancy grid maps for navigation and visualization purposes.

## Features

- Converts PCD files to 2D occupancy grid maps
- Configurable Z-axis filtering
- Adjustable map resolution
- Continuous map publishing
- ROS1 and ROS2 compatible implementations

## Dependencies

- ROS (Noetic or later)
- PCL (Point Cloud Library)
- sensor_msgs
- nav_msgs
- pcl_ros
- pcl_conversions

## Installation

```bash
cd ~/catkin_ws/src
git clone https://github.com/emirhancibir/cloud_to_map.git
cd ~/catkin_ws
catkin_make
```
## Usage

1. Launch the node with default parameters:
```bash
roslaunch cloud_to_map cloud_to_map.launch
```

2. Configure parameters in the launch file:
```bash
<param name="pcd_file" value="path/to/your/map.pcd"/>
<param name="min_z" value="0.0"/>
<param name="max_z" value="2.5"/>
```

## Parameters
-** pcd_file:** Path to the input PCD file
- **min_z:** Minimum height for point filtering
- **max_z:** Maximum height for point filtering
- **resolution:** Map resolution (default: 0.05 meters)

## Topics
### Published Topics
- **/map (nav_msgs/OccupancyGrid):** The generated 2D occupancy grid map
