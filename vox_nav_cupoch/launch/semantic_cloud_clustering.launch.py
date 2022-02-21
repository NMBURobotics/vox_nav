from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(package='vox_nav_cupoch',
                                executable='semantic_cloud_clustering',
                                name='semantic_cloud_clustering',
                                output='screen',
                                #prefix=['xterm -e gdb -ex run --args'],
                                #prefix=['valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no'],
                                remappings=[('points', '/ouster/points/segmented'),
                                            ('odom', '/odometry/global'),
                                            ('imu', '/ouster/imu')],
                                parameters=[{'sequence_horizon': 2},
                                            {'dt': 0.01},
                                            {'sensor_height': 0.0}])
    ])
