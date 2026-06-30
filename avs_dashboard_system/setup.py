from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'avs_dashboard_system'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required resource marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AVS Team',
    maintainer_email='avs@robot.local',
    description='AVS Robot Control Center — ROS 2 dashboard backend',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Backend nodes (suffix _control to distinguish from vision nodes)
            'telemetry_aggregator_control = avs_dashboard_system.telemetry_aggregator_control_node:main',
            'cmd_vel_mux_control          = avs_dashboard_system.cmd_vel_mux_control_node:main',
            'controller_supervisor_control= avs_dashboard_system.controller_supervisor_control_node:main',
            'experiment_recorder_control  = avs_dashboard_system.experiment_recorder_control_node:main',
            'experiment_analyzer_control  = avs_dashboard_system.experiment_analyzer_control_node:main',
            'process_manager_control      = avs_dashboard_system.process_manager_control_node:main',
            'gazebo_manager_control       = avs_dashboard_system.gazebo_manager_control_node:main',
            'dashboard_api_control        = avs_dashboard_system.dashboard_api_control_node:main',
        ],
    },
)
