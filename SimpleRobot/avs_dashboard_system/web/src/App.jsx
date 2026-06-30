/**
 * AVS Robot Control Center — App.jsx
 * Root component: WebSocket initialization + routing
 */
import { useEffect } from 'react'
import { Routes, Route, Navigate } from 'react-router-dom'
import { wsService } from './services/websocket.js'
import { useDispatch, useStore } from './services/store.jsx'
import Layout from './components/Layout/Layout.jsx'
import Overview from './pages/Overview.jsx'
import ManualControl from './pages/ManualControl.jsx'
import Controller from './pages/Controller.jsx'
import LaneMonitor from './pages/LaneMonitor.jsx'
import RealtimePlots from './pages/RealtimePlots.jsx'
import CompareControllers from './pages/CompareControllers.jsx'
import MapViewer from './pages/MapViewer.jsx'
import GazeboSimulation from './pages/GazeboSimulation.jsx'
import Parameters from './pages/Parameters.jsx'
import NodeManager from './pages/NodeManager.jsx'
import LogsExport from './pages/LogsExport.jsx'
import Settings from './pages/Settings.jsx'

export default function App() {
  const dispatch = useDispatch()

  useEffect(() => {
    // Connect WebSocket
    wsService.connect()

    // Subscribe to connection state
    const unsubConn = wsService.onConnectionChange(connected => {
      dispatch({ type: 'SET_CONNECTED', payload: connected })
    })

    // Subscribe to all message types
    const unsubTelemetry = wsService.on('dashboard_state', data => {
      dispatch({ type: 'UPDATE_TELEMETRY', payload: data })
    })
    const unsubCtrl = wsService.on('controller_list', data => {
      dispatch({ type: 'UPDATE_CONTROLLER_LIST', payload: data })
    })
    const unsubProcess = wsService.on('process_status', data => {
      dispatch({ type: 'UPDATE_PROCESS_STATUS', payload: data })
    })
    const unsubGazebo = wsService.on('gazebo_status', data => {
      dispatch({ type: 'UPDATE_GAZEBO_STATUS', payload: data })
    })
    const unsubExpStatus = wsService.on('experiment_status', data => {
      dispatch({ type: 'UPDATE_EXPERIMENT_STATUS', payload: data })
    })
    const unsubExpList = wsService.on('experiment_list', data => {
      dispatch({ type: 'UPDATE_EXPERIMENT_LIST', payload: data })
    })
    const unsubExpSummary = wsService.on('experiment_summary', data => {
      dispatch({ type: 'UPDATE_EXPERIMENT_SUMMARY', payload: data })
    })
    const unsubComparison = wsService.on('controller_comparison', data => {
      dispatch({ type: 'UPDATE_CONTROLLER_COMPARISON', payload: data })
    })
    const unsubRosIntrospection = wsService.on('ros_introspection', data => {
      dispatch({ type: 'UPDATE_ROS_INTROSPECTION', payload: data })
    })
    const unsubLog = wsService.on('log', data => {
      dispatch({ type: 'ADD_LOG', payload: {
        time: data.time ?? Date.now() / 1000,
        level: data.level ?? 'info',
        msg: data.msg ?? '',
      }})
    })

    // Request initial data
    wsService.send('get_status', {})
    wsService.send('list_experiments', {})

    return () => {
      unsubConn()
      unsubTelemetry()
      unsubCtrl()
      unsubProcess()
      unsubGazebo()
      unsubExpStatus()
      unsubExpList()
      unsubExpSummary()
      unsubComparison()
      unsubRosIntrospection()
      unsubLog()
    }
  }, [dispatch])

  return (
    <Layout>
      <Routes>
        <Route path="/"           element={<Navigate to="/overview" replace />} />
        <Route path="/overview"   element={<Overview />} />
        <Route path="/manual"     element={<ManualControl />} />
        <Route path="/controller" element={<Controller />} />
        <Route path="/lane"       element={<LaneMonitor />} />
        <Route path="/plots"      element={<RealtimePlots />} />
        <Route path="/compare"    element={<CompareControllers />} />
        <Route path="/map"        element={<MapViewer />} />
        <Route path="/gazebo"     element={<GazeboSimulation />} />
        <Route path="/params"     element={<Parameters />} />
        <Route path="/nodes"      element={<NodeManager />} />
        <Route path="/logs"       element={<LogsExport />} />
        <Route path="/settings"   element={<Settings />} />
        <Route path="*"           element={<Navigate to="/overview" replace />} />
      </Routes>
    </Layout>
  )
}
