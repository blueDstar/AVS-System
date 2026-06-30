/**
 * AVS Dashboard - Constants
 */

export const CONTROLLERS = {
  off: { label: 'OFF', color: 'neutral' },
  manual: { label: 'Manual Control', color: 'info' },
  main_pd: { label: 'Main PD', color: 'success' },
  pd_lidar: { label: 'PD + LiDAR', color: 'success' },
  backstepping_pd: { label: 'Backstepping PD', color: 'success' },
  cascade_pd: { label: 'Cascade PD', color: 'success' },
  simulation: { label: 'Simulation Mode', color: 'warn' },
};

export const RUNTIME_MODES = {
  real_robot: { label: 'Real Robot', type: 'success' },
  gazebo: { label: 'Gazebo Simulation', type: 'warn' },
};
