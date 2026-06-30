"""
AVS Dashboard System — Main Launch File
File: launch/avs_dashboard.launch.py

Launches all dashboard backend nodes:
  - telemetry_aggregator_control
  - cmd_vel_mux_control
  - controller_supervisor_control
  - experiment_recorder_control
  - experiment_analyzer_control
  - process_manager_control
  - gazebo_manager_control
  - dashboard_api_control

Usage:
  ros2 launch avs_dashboard_system avs_dashboard.launch.py \\
      runtime:=real_robot web_port:=8080

Parameters:
  runtime           : real_robot | gazebo (default: real_robot)
  web_port          : HTTP/WS server port (default: 8080)
  web_host          : Server bind host (default: 0.0.0.0)
  enable_cmd_mux    : Enable cmd_vel mux node (default: true)
  enable_recorder   : Enable experiment recorder (default: true)
  enable_gazebo_mgr : Enable Gazebo manager (default: true)
  ros_domain_id     : ROS_DOMAIN_ID (default: 20)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, LogInfo
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # -------------------------------------------------------------------------
    # Declare launch arguments
    # -------------------------------------------------------------------------
    args = [
        DeclareLaunchArgument(
            'runtime',
            default_value='real_robot',
            description='Operation mode: real_robot or gazebo',
            choices=['real_robot', 'gazebo'],
        ),
        DeclareLaunchArgument(
            'web_port',
            default_value='8080',
            description='Port for the web dashboard HTTP/WS server',
        ),
        DeclareLaunchArgument(
            'web_host',
            default_value='0.0.0.0',
            description='Host to bind the web server',
        ),
        DeclareLaunchArgument(
            'ros_domain_id',
            default_value='20',
            description='ROS_DOMAIN_ID',
        ),
        DeclareLaunchArgument(
            'enable_cmd_mux',
            default_value='true',
            description='Enable cmd_vel multiplexer node',
        ),
        DeclareLaunchArgument(
            'enable_recorder',
            default_value='true',
            description='Enable experiment recorder node',
        ),
        DeclareLaunchArgument(
            'enable_gazebo_mgr',
            default_value='true',
            description='Enable Gazebo manager node',
        ),
        DeclareLaunchArgument(
            'v_max',
            default_value='0.3',
            description='Maximum linear velocity (m/s)',
        ),
        DeclareLaunchArgument(
            'omega_max',
            default_value='2.0',
            description='Maximum angular velocity (rad/s)',
        ),
        DeclareLaunchArgument(
            'publish_hz',
            default_value='10.0',
            description='Telemetry publish Hz',
        ),
    ]

    # Shorthand for LaunchConfiguration
    runtime     = LaunchConfiguration('runtime')
    web_port    = LaunchConfiguration('web_port')
    web_host    = LaunchConfiguration('web_host')
    domain_id   = LaunchConfiguration('ros_domain_id')
    v_max       = LaunchConfiguration('v_max')
    omega_max   = LaunchConfiguration('omega_max')
    pub_hz      = LaunchConfiguration('publish_hz')

    # -------------------------------------------------------------------------
    # Common parameters shared by all nodes
    # -------------------------------------------------------------------------
    common_params = [
        {'ros_domain_id': domain_id},
    ]

    # -------------------------------------------------------------------------
    # Node definitions
    # -------------------------------------------------------------------------

    # 1. Telemetry Aggregator
    telemetry_node = Node(
        package='avs_dashboard_system',
        executable='telemetry_aggregator_control',
        name='telemetry_aggregator_control',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'publish_hz': pub_hz,
            'hz_window_s': 2.0,
            'ros_domain_id': domain_id,
            'runtime_mode': runtime,
            'timeout_odom_s': 1.0,
            'timeout_imu_s': 2.0,
            'timeout_scan_s': 2.0,
            'timeout_control_error_s': 3.0,
            # Topics (defaults — can be overridden)
            'topic_cmd_vel': '/cmd_vel',
            'topic_odom_raw': '/odom_raw',
            'topic_imu': '/imu',
            'topic_scan': '/scan',
            'topic_control_error': '/avs/control_error',
            'topic_telemetry': '/avs/telemetry',
            'topic_telemetry_rw': '/avs/telemetry_realworld',
            'topic_main_pd_debug': '/avs/main_following_pd_debug',
            'topic_lane_pd_state': '/avs/lane_pd_state',
            'topic_wheel_pd_state': '/avs/wheel_pd_state',
            'topic_cascade_state': '/avs/cascade_control_state',
            'topic_selected_ctrl': '/avs/selected_controller',
            'topic_emergency_stop': '/avs/emergency_stop',
            'topic_dashboard_state': '/avs/dashboard_state',
        }],
    )

    # 2. cmd_vel Mux (safety critical)
    mux_node = Node(
        package='avs_dashboard_system',
        executable='cmd_vel_mux_control',
        name='cmd_vel_mux_control',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'publish_hz': 20.0,
            'status_hz': 5.0,
            'cmd_source_timeout_s': 0.5,
            'v_max': v_max,
            'v_min': -0.1,
            'omega_max': omega_max,
            'max_accel': 0.5,
            'max_alpha': 1.0,
            'topic_cmd_vel': '/cmd_vel',
            'topic_selected_ctrl': '/avs/selected_controller',
            'topic_emergency_stop': '/avs/emergency_stop',
            'topic_mux_state': '/avs/cmd_vel_mux_state',
        }],
    )

    # 3. Controller Supervisor
    supervisor_node = Node(
        package='avs_dashboard_system',
        executable='controller_supervisor_control',
        name='controller_supervisor_control',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'ros_domain_id': domain_id,
            'topic_selected_ctrl': '/avs/selected_controller',
            'topic_emergency_stop': '/avs/emergency_stop',
            'topic_supervisor_cmd': '/avs/supervisor_cmd',
            'topic_controller_list': '/avs/controller_list',
        }],
    )

    # 4. Experiment Recorder
    recorder_node = Node(
        package='avs_dashboard_system',
        executable='experiment_recorder_control',
        name='experiment_recorder_control',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'base_dir': '~/avs_experiments',
            'csv_flush_interval_s': 1.0,
            'auto_plot': True,
            'record_hz': 10.0,
            'topic_cmd_vel': '/cmd_vel',
            'topic_odom_raw': '/odom_raw',
            'topic_imu': '/imu',
            'topic_scan': '/scan',
            'topic_control_error': '/avs/control_error',
            'topic_dashboard_state': '/avs/dashboard_state',
            'topic_experiment_cmd': '/avs/experiment/cmd',
            'topic_experiment_status': '/avs/experiment_status',
        }],
    )

    # 5. Experiment Analyzer
    analyzer_node = Node(
        package='avs_dashboard_system',
        executable='experiment_analyzer_control',
        name='experiment_analyzer_control',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'base_dir': '~/avs_experiments',
            'topic_analyzer_cmd': '/avs/analyzer_cmd',
            'topic_exp_summary': '/avs/experiment_summary',
            'topic_comparison': '/avs/controller_comparison',
            'topic_exp_list': '/avs/experiment_list',
        }],
    )

    # 6. Process Manager
    process_mgr_node = Node(
        package='avs_dashboard_system',
        executable='process_manager_control',
        name='process_manager_control',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'ros_domain_id': domain_id,
            'topic_process_cmd': '/avs/process_cmd',
            'topic_process_status': '/avs/process_status',
            'status_hz': 2.0,
            'processes_config_path': os.path.join(
                get_package_share_directory('avs_dashboard_system'),
                'config', 'processes.yaml'
            ),
        }],
    )

    # 7. Gazebo Manager
    gazebo_mgr_node = Node(
        package='avs_dashboard_system',
        executable='gazebo_manager_control',
        name='gazebo_manager_control',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'ros_domain_id': domain_id,
            'default_runtime': runtime,
            'topic_gazebo_cmd': '/avs/gazebo_cmd',
            'topic_gazebo_status': '/avs/gazebo_status',
            'topic_emergency_stop': '/avs/emergency_stop',
            'topic_supervisor_cmd': '/avs/supervisor_cmd',
        }],
    )

    # 8. Dashboard API (aiohttp WebSocket + HTTP server)
    api_node = Node(
        package='avs_dashboard_system',
        executable='dashboard_api_control',
        name='dashboard_api_control',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'web_host': web_host,
            'web_port': web_port,
            'telemetry_push_hz': pub_hz,
            'base_dir': '~/avs_experiments',
            'static_dir': '',
            'topic_dashboard_state': '/avs/dashboard_state',
            'topic_process_status': '/avs/process_status',
            'topic_controller_list': '/avs/controller_list',
            'topic_gazebo_status': '/avs/gazebo_status',
            'topic_experiment_status': '/avs/experiment_status',
            'topic_experiment_list': '/avs/experiment_list',
            'topic_experiment_summary': '/avs/experiment_summary',
            'topic_controller_comparison': '/avs/controller_comparison',
            'topic_cmd_manual': '/avs/cmd_vel/manual',
            'topic_supervisor_cmd': '/avs/supervisor_cmd',
            'topic_process_cmd': '/avs/process_cmd',
            'topic_gazebo_cmd': '/avs/gazebo_cmd',
            'topic_experiment_cmd': '/avs/experiment/cmd',
            'topic_analyzer_cmd': '/avs/analyzer_cmd',
            'topic_emergency_stop': '/avs/emergency_stop',
        }],
    )

    return LaunchDescription(
        args + [
            LogInfo(msg=['[AVS Dashboard] Launching with runtime=', runtime,
                        ' port=', web_port]),
            telemetry_node,
            mux_node,
            supervisor_node,
            recorder_node,
            analyzer_node,
            process_mgr_node,
            gazebo_mgr_node,
            api_node,
        ]
    )
