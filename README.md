# Map_of_racetrack
This package is a ros2 node that transforms the incoming lidar pointcloud from the lidar frame into the world frame.

# Build the package    
Ubuntu and ROS2 Humble is required for this package. 

Clone this repository into the src folder in your ros2 workspace.

Install the required packages:

```
sudo apt install python3-rosdep2
```
```
rosdep update &&  rosdep install --from-paths src --ignore-src -r -y
```

Source the following .bash files:

```
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source ~/ros2_ws/install/setup.bash
```


Modify the paths in the launch file

Modify the paths under the "# To be modified" comments

```
measurement_path = '<path_to_your_measurement_folder>'
```

```
static_map_path = "<path_to_map_of_racetrack_pkg/static_map>"
```

Its also requierd to place the measurement (.mcap and .yaml file) in this folder.


Build the package with colcon.
```
cd <your_ros2_ws>
colcon build
```

# How to run

```
ros2 launch map_of_racetrack launch.py
```


# Visualization

**Option 1:**

After ros2 launch, rviz will open automatically.
You can follow how the track is drawn.

**Option 2:**

After you terminate the node, two file will be created: racetrack.pcb and voxelized_racetrack.pcb

 - racetrack.pcb: contains all the measured points

 - voxelized_racetrack.pcb: cointain the filtered points only

You can open the file with pcl_viewer

```
cd <path_to_map_of_racetrack_pkg/static_map>
```
```
pcl_viewer racetrack.pcd
```
```
pcl_viewer voxelized_racetrack.pcd
```



