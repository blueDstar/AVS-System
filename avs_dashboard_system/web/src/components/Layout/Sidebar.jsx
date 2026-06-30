import React, { useState } from 'react';
import { NavLink } from 'react-router-dom';
import {
  LayoutDashboard, Gamepad2, Cpu, Eye,
  LineChart, GitCompare, Map as MapIcon,
  Box, Sliders, FileText, Settings, Network,
  PanelLeftClose, PanelLeftOpen
} from 'lucide-react';

const NAV_ITEMS = [
  { path: '/overview', icon: LayoutDashboard, label: 'Overview' },
  { path: '/manual', icon: Gamepad2, label: 'Manual Control' },
  { path: '/controller', icon: Cpu, label: 'Controllers' },
  { path: '/lane', icon: Eye, label: 'Lane Monitor' },
  { path: '/plots', icon: LineChart, label: 'Realtime Plots' },
  { path: '/compare', icon: GitCompare, label: 'Compare' },
  { path: '/map', icon: MapIcon, label: 'Map / RViz' },
  { path: '/gazebo', icon: Box, label: 'Gazebo' },
  { path: '/params', icon: Sliders, label: 'Parameters' },
  { path: '/nodes', icon: Network, label: 'ROS Nodes' },
  { path: '/logs', icon: FileText, label: 'Logs & Export' },
  { path: '/settings', icon: Settings, label: 'Settings' },
];

export default function Sidebar({ collapsed, onToggle }) {
  return (
    <>
      <aside
        className="app-sidebar flex-col"
        style={{
          background: 'var(--color-surface)',
          borderRight: '1px solid var(--color-border)',
          width: collapsed ? '56px' : 'var(--sidebar-width)',
          minWidth: collapsed ? '56px' : 'var(--sidebar-width)',
          transition: 'width 0.25s ease, min-width 0.25s ease',
          overflow: 'hidden',
        }}
      >
        {/* Header / Logo */}
        <div className="flex flex-col gap-2 border-b" style={{
          borderColor: 'var(--color-border)',
          padding: collapsed ? '12px 8px' : '16px',
        }}>
          {!collapsed && (
            <h1 className="font-bold text-accent tracking-widest text-sm flex items-center justify-center gap-2">
              <span>🤖</span> AVS CENTER
            </h1>
          )}
          {collapsed && (
            <div className="flex items-center justify-center">
              <span style={{ fontSize: '1.2rem' }}>🤖</span>
            </div>
          )}
          {!collapsed && (
            <a
              href="/"
              className="flex items-center justify-center gap-2 px-3 py-1.5 rounded transition-all duration-200 text-xs font-semibold bg-[rgba(255,255,255,0.05)] hover:bg-[rgba(255,255,255,0.1)] text-[var(--color-text-muted)] hover:text-white border border-[var(--color-border)]"
            >
              <span>⬅️</span> Back to Base
            </a>
          )}
          {collapsed && (
            <a href="/" className="flex items-center justify-center py-1 rounded hover:bg-[rgba(255,255,255,0.1)] text-[var(--color-text-muted)]" title="AVS VISION DASHBOARD">
              <span>⬅️</span>
            </a>
          )}
        </div>

        {/* Toggle button */}
        <button
          onClick={onToggle}
          className="flex items-center justify-center py-2 mx-2 my-1 rounded transition-all duration-200 hover:bg-[rgba(255,255,255,0.08)] text-[var(--color-text-muted)] hover:text-white"
          title={collapsed ? 'Expand sidebar' : 'Collapse sidebar'}
          style={{ border: 'none', background: 'transparent', cursor: 'pointer' }}
        >
          {collapsed
            ? <PanelLeftOpen size={18} strokeWidth={2} />
            : <PanelLeftClose size={18} strokeWidth={2} />
          }
          {!collapsed && <span className="ml-2 text-xs font-medium">Thu gọn</span>}
        </button>

        {/* Navigation */}
        <nav className="flex-1 overflow-y-auto flex flex-col gap-0.5" style={{ padding: collapsed ? '4px' : '8px 12px' }}>
          {NAV_ITEMS.map((item) => (
            <NavLink
              key={item.path}
              to={item.path}
              title={item.label}
              className={({ isActive }) =>
                `flex items-center rounded transition-all duration-200 ${isActive
                  ? 'bg-[var(--color-accent-glow)] text-[var(--color-accent)] font-semibold'
                  : 'text-[var(--color-text-muted)] hover:bg-[rgba(255,255,255,0.05)] hover:text-[var(--color-text)]'
                }`
              }
              style={{
                gap: collapsed ? '0' : '10px',
                padding: collapsed ? '10px 0' : '10px 14px',
                justifyContent: collapsed ? 'center' : 'flex-start',
                fontSize: '0.875rem',
              }}
            >
              <item.icon size={20} strokeWidth={2} />
              {!collapsed && <span>{item.label}</span>}
            </NavLink>
          ))}
        </nav>

        {/* Footer */}
        <div className="border-t text-xs text-center text-dim" style={{
          borderColor: 'var(--color-border)',
          padding: collapsed ? '8px 4px' : '16px',
        }}>
          {collapsed ? 'v1' : 'v1.0.0 (ROS 2 Humble)'}
        </div>
      </aside>
    </>
  );
}
