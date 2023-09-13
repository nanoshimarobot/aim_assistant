# aim_assistant
This package was developed for ER, in the ABU Robocon 2023.  

## Rules
The game rules are [here](https://official-robocon.com/gakusei/).

## Subscribe
- '/livox/mid_70' : PointCloud data from LIVOX MID-70 (sensor_msgs/msg/PointCloud2)
- '/livox/mid_360' : PointCloud data from LIVOX MID-360 (sensor_msgs/msg/PointCloud2)
- '/velodyne_points' : PointCloud data from Velodyne VLP-16 (sensor_msgs/msg/PointCloud2)

## Publish
- '/ER/aim_assistant_3d/dbg_poles' : Pole data recognized and visualized for debug (visualization_msgs/msg/MarkerArray)
- '/ER/aim_assistant_3d/target_poles' : Pole data, actually used for Robot control. This data are sent to the package of shooting unit, and used for aim. (aim_assistant_msgs/msg/PoleArray)

## Usage
By following steps, run this package in demo mode.
### make workspace and clone this repository
```sh
mkdir -p colcon_ws/src
cd colon_ws/src
git clone https://github.com/nanoshimarobot/aim_assistant.git
git clone https://github.com/nanoshimarobot/aim_assistant_msgs.git
```

### Download rosbag data
```sh
cd ~/colcon_ws/src/aim_assistant/
. rosbag_downloader.sh
```

### Build package
```sh
cd ~/colcon_ws
colcon build --symlink-install
```

### Launch nodes
```sh
. install/local_setup.bash
ros2 launch aim_assistant aim_assistant_demo.launch.py
```