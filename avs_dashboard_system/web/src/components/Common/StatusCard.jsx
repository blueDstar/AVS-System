import React from 'react';

/**
 * StatusCard
 * @param {string} title - Card title
 * @param {ReactNode} icon - Icon component
 * @param {string} value - Main metric value
 * @param {string} unit - Metric unit
 * @param {string} status - 'ok', 'warn', 'error', 'neutral'
 * @param {string} subtitle - Small text below value
 * @param {ReactNode} children - Additional content (like mini charts)
 */
export default function StatusCard({ title, icon, value, unit, status = 'neutral', subtitle, children, className = '' }) {
  
  const statusColors = {
    ok: 'text-success',
    warn: 'text-warning',
    error: 'text-danger',
    neutral: 'text-text',
    info: 'text-info',
  };
  
  const valColor = statusColors[status] || statusColors.neutral;

  return (
    <div className={`card flex flex-col ${className}`}>
      <div className="flex items-center justify-between mb-3">
        <div className="card-title flex items-center gap-2 m-0 border-0 p-0">
          {icon && <span className="text-muted">{icon}</span>}
          {title}
        </div>
        {status !== 'neutral' && (
          <div className={`w-2 h-2 rounded-full ${valColor === 'text-success' ? 'bg-success animate-pulse' : valColor.replace('text-', 'bg-')}`} />
        )}
      </div>
      
      <div className="flex items-baseline gap-1 mt-auto">
        <span className={`metric-value ${valColor}`}>{value}</span>
        {unit && <span className="metric-unit">{unit}</span>}
      </div>
      
      {subtitle && (
        <div className="text-xs text-dim mt-1 truncate">{subtitle}</div>
      )}
      
      {children && (
        <div className="mt-4 pt-3 border-t border-[rgba(255,255,255,0.05)] flex-1">
          {children}
        </div>
      )}
    </div>
  );
}
