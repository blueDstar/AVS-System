from setuptools import setup, find_packages

package_name = 'avs_controlsystem'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avs',
    maintainer_email='avs@example.com',
    description='AVS control system for lane following',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pur_persuit_pd_mainlane_following = avs_controlsystem.pur_persuit_pd_mainlane_following:main',
            'pur_persuit_mainlane_following = avs_controlsystem.pur_persuit_mainlane_following:main',
            'mainlane_following_controlerror = avs_controlsystem.mainlane_following_controlerror:main',
            'safe_error_cmdvel_node = avs_controlsystem.safe_error_cmdvel_node:main',
            'avs_lane_cmdvel_node = avs_controlsystem.avs_lane_cmdvel_node:main',
            'lane_parser_node = avs_controlsystem.lane_parser_node:main',
            'lane_lidar_follower_node = avs_controlsystem.lane_lidar_follower_node:main',
            'smooth_lane_lidar_follower_node = avs_controlsystem.smooth_lane_lidar_follower_node:main',
        ],
    },
)
