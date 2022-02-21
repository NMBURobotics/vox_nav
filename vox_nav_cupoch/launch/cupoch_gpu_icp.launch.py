from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(package='vox_nav_cupoch',
                                executable='cupoch_gpu_icp',
                                name='cupoch_gpu_icp',
                                output='screen',
                                #prefix=['xterm -e gdb -ex run --args'],
                                #prefix=['valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no'],
                                remappings=[('/ouster/points', '/ouster/points'),
                                            ('vox_nav/map_server/octomap_pointcloud',
                                             'vox_nav/map_server/octomap_pointcloud'),
                                            ('odometry/gps', 'odometry/gps')],
                                parameters=[{'sequence_horizon': 2},
                                            {'dt': 0.01},
                                            {'sensor_height': 0.0}])
    ])
