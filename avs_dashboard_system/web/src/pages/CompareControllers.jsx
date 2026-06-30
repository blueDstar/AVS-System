import React, { useEffect, useState } from 'react';
import { wsService } from '../services/websocket';
import { useStore } from '../services/store';
import { GitCompare, Play, Download, BarChart2 } from 'lucide-react';
import { apiService } from '../services/api';
import { formatNumber } from '../utils/formatters';

export default function CompareControllers() {
  const store = useStore();
  const comparison = store.controllerComparison;
  const experimentsList = store.experimentList?.experiments || [];
  
  const [selectedIds, setSelectedIds] = useState([]);
  const [scenario, setScenario] = useState('All');

  useEffect(() => {
    // Initial fetch if list is empty
    if (experimentsList.length === 0) {
      wsService.send('list_experiments', {});
    }
  }, []);

  const toggleSelect = (id) => {
    setSelectedIds(prev => 
      prev.includes(id) ? prev.filter(x => x !== id) : [...prev, id]
    );
  };

  const handleCompare = () => {
    if (selectedIds.length === 0) return alert('Select at least one experiment');
    wsService.send('compare_experiments', { ids: selectedIds, scenario });
  };

  const handleExport = () => {
    if (selectedIds.length === 0) return alert('Select at least one experiment');
    // Using aiohttp REST to trigger export logic on backend
    // Could also send via WS if backend handles CSV saving.
    wsService.send('analyzer_cmd', { action: 'export_comparison', experiment_ids: selectedIds });
    alert("Comparison CSV export triggered on backend (saved to ~/avs_experiments).");
  };

  const exps = comparison?.experiments || [];

  return (
    <div className="page-container flex flex-col h-full">
      <div className="flex justify-between items-end mb-6 shrink-0">
        <div>
          <h1 className="page-title">Compare Controllers</h1>
          <p className="page-subtitle mb-0">Select multiple recorded experiments to compare performance metrics.</p>
        </div>
      </div>

      <div className="grid grid-cols-3 gap-6 flex-1 min-h-0">
        
        {/* Left: Selector */}
        <div className="card col-span-1 flex flex-col overflow-hidden p-0">
          <div className="p-4 border-b border-[rgba(255,255,255,0.1)] bg-[rgba(255,255,255,0.02)]">
            <h3 className="font-semibold mb-2">Select Experiments</h3>
            <div className="flex gap-2">
              <button onClick={handleCompare} className="btn btn-primary flex-1 flex justify-center gap-2" disabled={selectedIds.length === 0}>
                <GitCompare size={16} /> COMPARE
              </button>
            </div>
          </div>
          
          <div className="flex-1 overflow-y-auto p-2">
            {experimentsList.length === 0 && <div className="p-4 text-center text-muted text-sm">No experiments found.</div>}
            {experimentsList.map(exp => (
              <label key={exp.id} className="flex items-start gap-3 p-3 hover:bg-[rgba(255,255,255,0.05)] rounded cursor-pointer border-b border-[rgba(255,255,255,0.02)]">
                <input 
                  type="checkbox" 
                  className="mt-1"
                  checked={selectedIds.includes(exp.id)}
                  onChange={() => toggleSelect(exp.id)}
                />
                <div className="flex-1 min-w-0">
                  <div className="font-semibold text-sm text-text truncate">
                    {exp.controller_name || exp.id}
                  </div>
                  <div className="text-xs text-dim truncate">
                    {exp.scenario_name || 'N/A'} • {new Date(exp.id.replace('exp_','').replace(/_/g,':').replace(':', ' ')).toLocaleDateString() || exp.id}
                  </div>
                  {exp.summary && (
                    <div className="text-xs text-info mt-1">
                      RMSE: {formatNumber(exp.summary.rmse_lateral_error_mm, 1)}mm
                    </div>
                  )}
                </div>
              </label>
            ))}
          </div>
        </div>

        {/* Right: Results Table */}
        <div className="card col-span-2 flex flex-col overflow-hidden">
          <div className="flex justify-between items-center mb-4">
            <h3 className="font-semibold flex items-center gap-2">
              <BarChart2 size={18} /> Comparison Results
            </h3>
            {exps.length > 0 && (
              <button onClick={handleExport} className="btn btn-secondary btn-sm flex items-center gap-2">
                <Download size={14} /> EXPORT CSV
              </button>
            )}
          </div>

          <div className="flex-1 overflow-auto rounded border border-[rgba(255,255,255,0.1)]">
            <table className="w-full text-left text-sm whitespace-nowrap">
              <thead className="bg-[rgba(255,255,255,0.05)] sticky top-0">
                <tr>
                  <th className="p-3 font-semibold text-muted border-b border-[rgba(255,255,255,0.1)]">Controller</th>
                  <th className="p-3 font-semibold text-muted border-b border-[rgba(255,255,255,0.1)]">Scenario</th>
                  <th className="p-3 font-semibold text-muted border-b border-[rgba(255,255,255,0.1)] text-right">Lat RMSE (mm)</th>
                  <th className="p-3 font-semibold text-muted border-b border-[rgba(255,255,255,0.1)] text-right">Lat Max (mm)</th>
                  <th className="p-3 font-semibold text-muted border-b border-[rgba(255,255,255,0.1)] text-right">Mean V (m/s)</th>
                  <th className="p-3 font-semibold text-muted border-b border-[rgba(255,255,255,0.1)] text-right">Track Ratio %</th>
                  <th className="p-3 font-semibold text-muted border-b border-[rgba(255,255,255,0.1)] text-right">Lane Lost (Count)</th>
                  <th className="p-3 font-semibold text-muted border-b border-[rgba(255,255,255,0.1)] text-right">Main Lane Seen %</th>
                  <th className="p-3 font-semibold text-muted border-b border-[rgba(255,255,255,0.1)] text-right">Stop Lines</th>
                </tr>
              </thead>
              <tbody>
                {exps.length === 0 && (
                  <tr>
                    <td colSpan={9} className="p-8 text-center text-muted">
                      Select experiments and click Compare to view results.
                    </td>
                  </tr>
                )}
                {exps.map((e, idx) => (
                  <tr key={idx} className="hover:bg-[rgba(255,255,255,0.02)] border-b border-[rgba(255,255,255,0.05)]">
                    <td className="p-3 font-semibold">{e.controller}</td>
                    <td className="p-3 text-muted">{e.scenario}</td>
                    <td className={`p-3 text-right font-mono ${e.rmse_lateral_error_mm < 50 ? 'text-success' : 'text-warning'}`}>
                      {formatNumber(e.rmse_lateral_error_mm, 2)}
                    </td>
                    <td className="p-3 text-right font-mono">
                      {formatNumber(e.max_abs_lateral_error_mm, 2)}
                    </td>
                    <td className="p-3 text-right font-mono">
                      {formatNumber(e.mean_cmd_v, 3)}
                    </td>
                    <td className={`p-3 text-right font-mono ${e.tracking_valid_ratio > 95 ? 'text-success' : 'text-danger'}`}>
                      {formatNumber(e.tracking_valid_ratio, 1)}%
                    </td>
                    <td className="p-3 text-right font-mono text-danger">
                      {e.perception_summary?.lane_lost_count || 0}
                    </td>
                    <td className="p-3 text-right font-mono text-success">
                      {formatNumber((e.perception_summary?.main_lane_seen_ratio || 0) * 100, 1)}%
                    </td>
                    <td className="p-3 text-right font-mono text-info">
                      {e.perception_summary?.stop_line_detect_count || 0}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
          
        </div>
      </div>
    </div>
  );
}
