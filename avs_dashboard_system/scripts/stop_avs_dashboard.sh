#!/bin/bash
# =============================================================================
# AVS Robot Control Center — Stop Script
# File: scripts/stop_avs_dashboard.sh
# =============================================================================

echo "Stopping AVS Dashboard System..."

# Stop specific python nodes by matching executable name
pkill -f "telemetry_aggregator_control"
pkill -f "cmd_vel_mux_control"
pkill -f "controller_supervisor_control"
pkill -f "experiment_recorder_control"
pkill -f "experiment_analyzer_control"
pkill -f "process_manager_control"
pkill -f "gazebo_manager_control"
pkill -f "dashboard_api_control"

echo "Nodes terminated."
