import React, { useState } from 'react';
import Sidebar from './Sidebar';
import TopBar from './TopBar';
import { useEmergency } from '../../services/store';

export default function Layout({ children }) {
  const isEmergency = useEmergency();
  const [sidebarCollapsed, setSidebarCollapsed] = useState(false);

  return (
    <div
      className={`app-layout ${isEmergency ? 'emergency-active' : ''}`}
      style={{
        gridTemplateColumns: sidebarCollapsed ? '56px 1fr' : 'var(--sidebar-width) 1fr',
        transition: 'grid-template-columns 0.25s ease',
      }}
    >
      <TopBar />
      <Sidebar collapsed={sidebarCollapsed} onToggle={() => setSidebarCollapsed(prev => !prev)} />
      <main className="app-main relative">
        {/* Background glow effect based on mode */}
        <div className="absolute inset-0 pointer-events-none opacity-20"
             style={{
               background: isEmergency
                ? 'radial-gradient(circle at center, rgba(239,68,68,0.2) 0%, transparent 70%)'
                : 'radial-gradient(circle at top right, rgba(0,212,170,0.1) 0%, transparent 50%)'
             }} />

        {/* Page Content */}
        <div className="relative h-full z-10">
          {children}
        </div>
      </main>
    </div>
  );
}
