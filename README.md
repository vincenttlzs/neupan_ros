# neupan_ros

This is the ROS wrapper for [NeuPAN Planner](https://github.com/hanruihua/neupan).

## Prerequisites

- ROS Noetic
- Python >= 3.10
- [NeuPAN Planner](https://github.com/hanruihua/neupan).

## Installation

```bash
mkdir -p ~/neupan_ws/src
cd ~/neupan_ws/src
git clone https://github.com/hanruihua/neupan_ros
cd ~/neupan_ws && catkin_make
cd ~/neupan_ws/src/neupan_ros 
sh source_setup.sh && source ~/neupan_ws/devel/setup.sh && rosdep install neupan_ros 
```

## Demonstration

### Dynamic collision avoidance

We provide the dynamic collision avoidance examples in Gazebo shown as follows. To run these examples, please see example/gazebo_limo for detail.

https://github.com/user-attachments/assets/db9edbfe-94d9-4a58-98ee-6b30e64dd3d9

## Node API 

Publish Topics

| Topic Name             | Message Type                     | Description                                |
| ---------------------- | -------------------------------- | ------------------------------------------ |
| `/neupan_cmd_vel`      | `geometry_msgs/Twist`            | The velocity command to the robot.         |
| `/neupan_plan`         | `nav_msgs/Path`                  | The NeuPAN planned path.                   |
| `/neupan_initial_path` | `nav_msgs/Path`                  | The initial path for NeuPAN visualization. |
| `/neupan_ref_state`    | `nav_msgs/Path`                  | The current reference state visualization. |
| `/dune_point_markers`  | `visualization_msgs/MarkerArray` | The DUNE points markers visualization.     |
| `/nrmp_point_markers`  | `visualization_msgs/MarkerArray` | The NRMP points markers visualization.     |
| `/robot_marker`        | `visualization_msgs/Marker`      | The robot marker visualization.            |

Subscribe Topics

| Topic Name      | Message Type                | Description                                                                                        |
| --------------- | --------------------------- | -------------------------------------------------------------------------------------------------- |
| `/scan`         | `sensor_msgs/LaserScan`     | The laser scan data.                                                                               |
| `/initial_path` | `nav_msgs/Path`             | The initial path for NeuPAN.                                                                       |
| `/neupan_goal`  | `geometry_msgs/PoseStamped` | The goal for NeuPAN. You can publish the goal to this topic to give the NeuPAN planner a new goal. |

ROS Parameters

| Parameter Name          | Type / Default Value | Description                                              |
| ----------------------- | -------------------- | -------------------------------------------------------- |
| `~config_file`          | `str` / None         | The path of the NeuPAN config YAML file.                 |
| `~map_frame`            | `str` / `map`        | The map frame name.                                      |
| `~base_frame`           | `str` / `base_link`  | The base frame name.                                     |
| `~lidar_frame`          | `str` / `laser_link` | The lidar frame name.                                    |
| `~marker_size`          | `float` / `0.05`     | The marker size to show the DUNE points and NRMP points. |
| `~marker_z`             | `float` / `1.0`      | The marker z to show the height of the robot.            |
| `~scan_angle_range`     | `str` / `-3.14 3.14` | The angle range of the laser scan.                       |
| `~scan_downsample`      | `int` / `1`          | The downsample rate of the laser scan.                   |
| `~scan_range`           | `str` / `0.0, 5.0`   | The distance range of the laser scan.                    |
| `~dune_checkpoint`      | `str` / `None`       | The path of the DUNE checkpoint file.                    |
| `~refresh_initial_path` | `bool` / `False`     | Whether to refresh the initial path.                     |
| `~flip_angle`           | `bool` / `False`     | Whether to flip the angle of the scan data.              |


## Citation

If you find our work helpful in your research, you can star our repo and consider citing:

```bibtex

@article{han2024neupan,
  title={NeuPAN: Direct Point Robot Navigation with End-to-End Model-based Learning},
  author={Han, Ruihua and Wang, Shuai and Wang, Shuaijun and Zhang, Zeqing and Chen, Jianjun and Lin, Shijie and Li, Chengyang and Xu, Chengzhong and Eldar, Yonina C and Hao, Qi and Pan, Jia},
  journal={arXiv preprint arXiv:2403.06828},
  year={2025} 
}
```

