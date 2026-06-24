# Mission Planner

    ros2 run mission_control mission_control [--ros-args -p mission_timeout_s:=120.0]

## Trigger actions manually

    ros2 topic pub --once /save_map_trigger std_msgs/msg/Empty "{}"

    ros2 topic pub --once /end_exploration_trigger std_msgs/msg/Empty "{}"



