# Waypoint Navigation with vox_nav

vox_nav allows a user to do waypoint navigation with both map coordinates and GPS coordinates. 
The GPS waypoint following node converts GPS poses to map poses using `robot_localization`s `fromLL` service. 

You may instantiate each of the `server` nodes as follows;

```bash
    navigate_through_poses_server_node = Node(
        package='vox_nav_navigators',
        executable='navigate_through_poses_server_node',
        name='navigate_through_poses_server_node',
        namespace='',
        output='screen',
        parameters=[params],
    )
    navigate_through_gps_poses_server_node = Node(
        package='vox_nav_navigators',
        executable='navigate_through_gps_poses_server_node',
        name='navigate_through_gps_poses_server_node',
        namespace='',
        output='screen',
        parameters=[params],
    )
```

The server nodes for waypoint navigation are written in `vox_nav_navigators` where they are implemented with behavior trees. There are no specific parameters that need to be passed to the server nodes, except for `use_sim_time``. 

The actual client nodes for the waypoint following are hosted in `vox_nav_waypoint_nav_clients`, see the launch folder and params folders. 