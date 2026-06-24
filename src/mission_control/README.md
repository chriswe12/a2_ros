# Mission Planner

    ros2 run mission_control mission_control [--ros-args -p mission_timeout_s:=120.0]

## Trigger actions manually

    ros2 topic pub --once /save_map_trigger std_msgs/msg/Empty "{}"

    ros2 topic pub --once /return_home_trigger std_msgs/msg/Empty "{}"

    ros2 topic pub --once /goal_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'odom'}, point: {x: 0.059738870710134506, y: 0.0006765986327081919, z: 0.15790770947933197}}"



