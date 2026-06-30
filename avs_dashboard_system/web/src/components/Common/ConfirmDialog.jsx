import React, { useState } from 'react';

/**
 * ConfirmDialog component for critical actions
 */
export default function ConfirmDialog({ isOpen, title, message, onConfirm, onCancel, confirmText = "Confirm", cancelText = "Cancel", isDanger = false }) {
  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center p-4 bg-black/60 backdrop-blur-sm animate-fade-in">
      <div className="card w-full max-w-md bg-surface border-border shadow-2xl p-6" onClick={e => e.stopPropagation()}>
        <h2 className="text-xl font-bold mb-2 text-text">{title}</h2>
        <p className="text-muted mb-6 text-sm">{message}</p>
        
        <div className="flex justify-end gap-3">
          <button onClick={onCancel} className="btn btn-secondary">
            {cancelText}
          </button>
          <button 
            onClick={onConfirm} 
            className={`btn ${isDanger ? 'btn-danger-solid' : 'btn-primary'}`}
          >
            {confirmText}
          </button>
        </div>
      </div>
    </div>
  );
}
