from glob import glob
import os

from setuptools import setup


package_name = 'vox_nav_drake'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=' ',
    maintainer_email=' ',
    description=' ',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drake_test_node = vox_nav_drake.drake_test_node:main'
        ],
    },
)
