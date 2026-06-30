/**
 * AVS Robot Control Center — Global State Store
 * File: src/services/store.jsx
 *
 * React Context + useReducer for global state management.
 * All telemetry from WebSocket is funneled through this store.
 *
 * Performance notes:
 *  - appendToBuffer() mutates in-place via TypedArray-like approach
 *    to avoid O(N) spread operations at 10Hz × 14 channels
 *  - History arrays are capped with efficient slice only when needed
 */

import { createContext, useContext, useReducer } from 'react';

// ============================================================================
// Initial State
// ============================================================================

const MAX_HISTORY = 1200;  // 120s × 10Hz
const MAX_PATH    = 5000;
const MAX_LOGS    = 500;

const makeHistory = () => ({
  timestamps:   [],
  cmd_v:        [],
  cmd_omega:    [],
  odom_v:       [],
  odom_omega:   [],
  imu_wz:       [],
  epsilon_x_mm: [],
  theta_rad:    [],
  front_min:    [],
  left_min:     [],
  right_min:    [],
  v_left_ref:   [],
  v_right_ref:  [],
  v_left_meas:  [],
  v_right_meas: [],
});

const INITIAL_STATE = {
  // Connection
  connected: false,

  // Dashboard telemetry
  telemetry: {
    time: 0,
    system: {
      ros_domain_id: 20,
      mode: 'real_robot',
      active_controller: 'off',
      emergency_stop: false,
      cmd_vel_hz: 0,
    },
    cmd_vel: { v: 0, omega: 0, hz: 0 },
    odom:  { x: 0, y: 0, yaw: 0, v: 0, omega: 0, hz: 0, timeout: true },
    imu:   { yaw: 0, wz: 0, hz: 0, timeout: true },
    lidar: { front_min: 9.9, left_min: 9.9, right_min: 9.9, hz: 0, timeout: true },
    lane:  {
      valid: false, state: 'UNKNOWN',
      epsilon_x_mm: 0, epsilon_y_mm: 0,
      theta_rad: 0, fps_est: 0, hz: 0, timeout: true,
    },
    controller_debug: {},
  },

  // Controller list (from WS message type: controller_list)
  controllerList: {
    active_controller: 'off',
    emergency_stop: false,
    controllers: [],
  },

  // Process status (from WS message type: process_status)
  processStatus: { processes: [] },
  rosIntrospection: {},

  // Gazebo status (from WS message type: gazebo_status)
  gazeboStatus: {
    gazebo_status: 'stopped',
    current_world: null,
    target_runtime: 'real_robot',
    available_worlds: {},
  },

  // Experiment
  experimentStatus: { recording: false, row_count: 0, duration_s: 0, metadata: {} },
  experimentList:   { experiments: [], total: 0 },
  experimentSummary: {},
  controllerComparison: { experiments: [] },

  // Realtime history (rolling buffers)
  realtimeHistory: makeHistory(),

  // Odom path for map viewer
  odomPath: [],

  // Application log stream
  logs: [],
};

// ============================================================================
// Buffer helpers — O(1) amortized push, O(1) trim
// ============================================================================

/** Push value and trim to max length. Returns same array reference when no trim. */
function push(arr, val, max) {
  arr.push(val);
  if (arr.length > max) {
    arr.splice(0, arr.length - max);
  }
  return arr;
}

/** Immutable push — returns new array. Used when React needs to see a new reference. */
function appendBuf(arr, val, max) {
  const next = arr.length < max ? [...arr, val] : [...arr.slice(-(max - 1)), val];
  return next;
}

// ============================================================================
// Reducer
// ============================================================================

function reducer(state, action) {
  switch (action.type) {

    case 'SET_CONNECTED':
      return { ...state, connected: action.payload };

    case 'UPDATE_TELEMETRY': {
      const t   = action.payload;
      const ts  = t.time ?? Date.now() / 1000;
      const h   = state.realtimeHistory;
      const cd  = t.controller_debug ?? {};

      // Build new history with immutable append
      const newHistory = {
        timestamps:   appendBuf(h.timestamps,   ts,                          MAX_HISTORY),
        cmd_v:        appendBuf(h.cmd_v,        t.cmd_vel?.v    ?? 0,        MAX_HISTORY),
        cmd_omega:    appendBuf(h.cmd_omega,    t.cmd_vel?.omega ?? 0,       MAX_HISTORY),
        odom_v:       appendBuf(h.odom_v,       t.odom?.v       ?? 0,        MAX_HISTORY),
        odom_omega:   appendBuf(h.odom_omega,   t.odom?.omega   ?? 0,        MAX_HISTORY),
        imu_wz:       appendBuf(h.imu_wz,       t.imu?.wz       ?? 0,        MAX_HISTORY),
        epsilon_x_mm: appendBuf(h.epsilon_x_mm, t.lane?.epsilon_x_mm ?? 0,   MAX_HISTORY),
        theta_rad:    appendBuf(h.theta_rad,    t.lane?.theta_rad    ?? 0,   MAX_HISTORY),
        front_min:    appendBuf(h.front_min,    t.lidar?.front_min   ?? 9.9, MAX_HISTORY),
        left_min:     appendBuf(h.left_min,     t.lidar?.left_min    ?? 9.9, MAX_HISTORY),
        right_min:    appendBuf(h.right_min,    t.lidar?.right_min   ?? 9.9, MAX_HISTORY),
        v_left_ref:   appendBuf(h.v_left_ref,   cd.v_left_ref  ?? cd.left_ref  ?? cd.left_speed_ref ?? cd.v_l_ref ?? 0, MAX_HISTORY),
        v_right_ref:  appendBuf(h.v_right_ref,  cd.v_right_ref ?? cd.right_ref ?? cd.right_speed_ref ?? cd.v_r_ref ?? 0, MAX_HISTORY),
        v_left_meas:  appendBuf(h.v_left_meas,  cd.v_left_meas ?? cd.left_meas ?? cd.left_speed_meas ?? cd.v_l_meas ?? 0, MAX_HISTORY),
        v_right_meas: appendBuf(h.v_right_meas, cd.v_right_meas ?? cd.right_meas ?? cd.right_speed_meas ?? cd.v_r_meas ?? 0, MAX_HISTORY),
      };

      // Append odom path (only if odom available and not same pos as last)
      let newPath = state.odomPath;
      if (t.odom) {
        const last = newPath[newPath.length - 1];
        const nx = t.odom.x, ny = t.odom.y;
        if (!last || Math.abs(last.x - nx) > 0.001 || Math.abs(last.y - ny) > 0.001) {
          newPath = [...newPath, { x: nx, y: ny }];
          if (newPath.length > MAX_PATH) newPath = newPath.slice(-MAX_PATH);
        }
      }

      return {
        ...state,
        telemetry:      t,
        realtimeHistory: newHistory,
        odomPath:       newPath,
      };
    }

    case 'UPDATE_CONTROLLER_LIST':
      return { ...state, controllerList: action.payload };

    case 'UPDATE_PROCESS_STATUS':
      return { ...state, processStatus: action.payload };

    case 'UPDATE_GAZEBO_STATUS':
      return { ...state, gazeboStatus: action.payload };

    case 'UPDATE_EXPERIMENT_STATUS':
      return { ...state, experimentStatus: action.payload };

    case 'UPDATE_EXPERIMENT_LIST':
      return { ...state, experimentList: action.payload };

    case 'UPDATE_EXPERIMENT_SUMMARY':
      return { ...state, experimentSummary: action.payload };

    case 'UPDATE_CONTROLLER_COMPARISON':
      return { ...state, controllerComparison: action.payload };

    case 'UPDATE_ROS_INTROSPECTION':
      return { ...state, rosIntrospection: action.payload };

    case 'ADD_LOG': {
      const logs = [...state.logs, action.payload];
      return {
        ...state,
        logs: logs.length > MAX_LOGS ? logs.slice(-MAX_LOGS) : logs,
      };
    }

    case 'RESET_ODOM_PATH':
      return { ...state, odomPath: [] };

    case 'RESET_HISTORY':
      return { ...state, realtimeHistory: makeHistory() };

    default:
      return state;
  }
}

// ============================================================================
// Context
// ============================================================================

const StoreContext    = createContext(null);
const DispatchContext = createContext(null);

export function StoreProvider({ children }) {
  const [state, dispatch] = useReducer(reducer, INITIAL_STATE);
  return (
    <StoreContext.Provider value={state}>
      <DispatchContext.Provider value={dispatch}>
        {children}
      </DispatchContext.Provider>
    </StoreContext.Provider>
  );
}

export function useStore()    { return useContext(StoreContext); }
export function useDispatch() { return useContext(DispatchContext); }

// ---- Convenience selectors ----
export const useConnected   = () => useStore().connected;
export const useTelemetry   = () => useStore().telemetry;
export const useEmergency   = () => useStore().telemetry?.system?.emergency_stop ?? false;
export const useActiveCtrl  = () => useStore().telemetry?.system?.active_controller ?? 'off';
export const useRuntime     = () => useStore().telemetry?.system?.mode ?? 'real_robot';
export const useHistory     = () => useStore().realtimeHistory;
export const useOdomPath    = () => useStore().odomPath;
export const useLogs        = () => useStore().logs;
export const useExperiments = () => useStore().experimentList;
export const useGazebo      = () => useStore().gazeboStatus;
export const useProcesses   = () => useStore().processStatus;
export const useControllers = () => useStore().controllerList;
export const useRosIntrospection = () => useStore().rosIntrospection;
