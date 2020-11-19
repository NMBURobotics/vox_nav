from setuptools import setup

package_name = 'botanbot_utilities'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fetullah Atas',
    maintainer_email='fetulahatas1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'gps_waypoint_collector = botanbot_utilities.collect_gps_waypoints:main',
        ],

    },
)
