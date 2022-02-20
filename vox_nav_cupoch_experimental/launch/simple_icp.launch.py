from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(package='vox_nav_cupoch_experimental',
                                executable='simple_icp',
                                name='simple_icp',
                                output='screen',
                                #prefix=['xterm -e gdb -ex run --args'],
                                #prefix=['valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no'],
                                remappings=[('/ouster/points', '/velodyne_points'),
                                            ('vox_nav/map_server/octomap_pointcloud', 'vox_nav/map_server/octomap_pointcloud')],
                                parameters=[{'sequence_horizon': 2},
                                            {'dt': 0.01},
                                            {'sensor_height': 0.0}])
    ])
