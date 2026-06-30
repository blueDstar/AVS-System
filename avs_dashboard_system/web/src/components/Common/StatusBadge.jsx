import React from 'react';

/**
 * StatusBadge
 * @param {string} status - 'ok', 'warn', 'error', 'info', 'neutral'
 * @param {string} label - Text to display
 * @param {boolean} dot - Show pulsing dot
 */
export default function StatusBadge({ status = 'neutral', label, dot = true }) {
  return (
    <span className={`badge badge-${status}`}>
      {dot && <span className="badge-dot" />}
      {label}
    </span>
  );
}
