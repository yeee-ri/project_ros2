# project_ros2
Using slam_toolbox
```
ros2 launch sllidar_ros2 sllidar_s1_launch.py serial_port:=/dev/ttyUSB1 

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint 

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link

ros2 run tf2_ros static_transform_publisher 0 0 0.2 0 0 0 base_link laser

ros2 launch slam_toolbox online_sync_launch.py scan_topic:=/scan use_sim_time:=false

```


