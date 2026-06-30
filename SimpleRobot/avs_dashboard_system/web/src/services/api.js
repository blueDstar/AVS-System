/**
 * AVS Robot Control Center — HTTP API Service
 * File: src/services/api.js
 *
 * Wrapper for HTTP REST endpoints (non-realtime operations).
 */

const API_BASE = `http://${window.location.hostname}:8080/api`;

export const apiService = {
  /** Check server health */
  async checkHealth() {
    try {
      const res = await fetch(`http://${window.location.hostname}:8080/health`);
      if (!res.ok) throw new Error('Network response was not ok');
      return await res.json();
    } catch (err) {
      console.error('[API] Health check failed:', err);
      return { status: 'error' };
    }
  },

  /** Get full status (if WS is disconnected) */
  async getStatus() {
    const res = await fetch(`${API_BASE}/status`);
    return await res.json();
  },

  /** Get list of experiments */
  async getExperiments() {
    const res = await fetch(`${API_BASE}/experiments`);
    return await res.json();
  },

  /** Get summary of a specific experiment */
  async getExperimentSummary(id) {
    const res = await fetch(`${API_BASE}/experiments/${id}/summary`);
    if (!res.ok) throw new Error('Summary not found');
    return await res.json();
  },

  /** Download experiment CSV */
  downloadCsvUrl(id) {
    return `${API_BASE}/experiments/${id}/csv`;
  },

  /** Get URL for a plot image */
  getPlotUrl(id, plotName) {
    return `${API_BASE}/experiments/${id}/plot/${plotName}`;
  },

  /** Trigger comparison on backend */
  async triggerComparison(ids, scenario) {
    const res = await fetch(`${API_BASE}/experiments/compare`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ ids, scenario }),
    });
    return await res.json();
  }
};
