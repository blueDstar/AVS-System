/**
 * AVS Robot Control Center — WebSocket Service
 * File: src/services/websocket.js
 *
 * Singleton WebSocket manager with:
 * - Auto-reconnect with exponential backoff
 * - Message queue for commands sent while disconnected
 * - Event emitter pattern for React hooks to subscribe
 */

const WS_URL = (() => {
  const host = window.location.hostname;
  const port = '8080'; // Explicitly point to AVS ROS2 backend port
  return `ws://${host}:${port}/ws`;
})();

const RECONNECT_BASE_MS   = 1000;
const RECONNECT_MAX_MS    = 10000;
const RECONNECT_FACTOR    = 1.5;
const PING_INTERVAL_MS    = 15000;

class WebSocketService {
  constructor() {
    this._ws = null;
    this._listeners = {};        // type → Set<callback>
    this._reconnectDelay = RECONNECT_BASE_MS;
    this._reconnectTimer = null;
    this._pingTimer = null;
    this._connected = false;
    this._messageQueue = [];     // commands queued while disconnected
    this._connectionListeners = new Set();
  }

  connect() {
    if (this._ws &&
        (this._ws.readyState === WebSocket.OPEN ||
         this._ws.readyState === WebSocket.CONNECTING)) {
      return;
    }
    try {
      this._ws = new WebSocket(WS_URL);
      this._ws.onopen    = () => this._onOpen();
      this._ws.onmessage = (e) => this._onMessage(e);
      this._ws.onclose   = (e) => this._onClose(e);
      this._ws.onerror   = (e) => this._onError(e);
    } catch (err) {
      console.error('[WS] Connection failed:', err);
      this._scheduleReconnect();
    }
  }

  disconnect() {
    clearTimeout(this._reconnectTimer);
    clearInterval(this._pingTimer);
    if (this._ws) {
      this._ws.onclose = null;  // prevent reconnect
      this._ws.close();
      this._ws = null;
    }
    this._connected = false;
    this._notifyConnectionChange(false);
  }

  /** Send a command to the backend. */
  send(action, data = {}) {
    const payload = JSON.stringify({ type: 'cmd', action, data });
    if (this._connected && this._ws?.readyState === WebSocket.OPEN) {
      this._ws.send(payload);
    } else {
      // Queue for later
      this._messageQueue.push(payload);
      if (this._messageQueue.length > 50) this._messageQueue.shift();
    }
  }

  /** Subscribe to messages of a specific type. Returns unsubscribe fn. */
  on(type, callback) {
    if (!this._listeners[type]) this._listeners[type] = new Set();
    this._listeners[type].add(callback);
    return () => this._listeners[type].delete(callback);
  }

  /** Subscribe to connection state changes. */
  onConnectionChange(callback) {
    this._connectionListeners.add(callback);
    // Immediately notify of current state
    callback(this._connected);
    return () => this._connectionListeners.delete(callback);
  }

  get connected() { return this._connected; }

  // -------------------------------------------------------------------------

  _onOpen() {
    console.log('[WS] Connected to', WS_URL);
    this._connected = true;
    this._reconnectDelay = RECONNECT_BASE_MS;
    this._notifyConnectionChange(true);

    // Flush queued messages
    while (this._messageQueue.length > 0) {
      const msg = this._messageQueue.shift();
      this._ws.send(msg);
    }

    // Start ping to keep connection alive
    clearInterval(this._pingTimer);
    this._pingTimer = setInterval(() => {
      if (this._ws?.readyState === WebSocket.OPEN) {
        this._ws.send(JSON.stringify({ type: 'ping' }));
      }
    }, PING_INTERVAL_MS);
  }

  _onMessage(event) {
    let msg;
    try {
      msg = JSON.parse(event.data);
    } catch {
      return;
    }
    const type = msg.type;
    const listeners = this._listeners[type];
    if (listeners) {
      listeners.forEach(cb => cb(msg.data ?? msg));
    }
    // Also notify wildcard listeners
    const allListeners = this._listeners['*'];
    if (allListeners) {
      allListeners.forEach(cb => cb(msg));
    }
  }

  _onClose(event) {
    this._connected = false;
    clearInterval(this._pingTimer);
    this._notifyConnectionChange(false);
    if (event.code !== 1000) {
      console.warn('[WS] Disconnected. Reconnecting...');
      this._scheduleReconnect();
    }
  }

  _onError(event) {
    console.error('[WS] Error:', event);
    // onclose will fire next and handle reconnect
  }

  _scheduleReconnect() {
    clearTimeout(this._reconnectTimer);
    this._reconnectTimer = setTimeout(() => {
      this._reconnectDelay = Math.min(
        this._reconnectDelay * RECONNECT_FACTOR,
        RECONNECT_MAX_MS
      );
      this.connect();
    }, this._reconnectDelay);
  }

  _notifyConnectionChange(connected) {
    this._connectionListeners.forEach(cb => cb(connected));
  }
}

// Singleton
export const wsService = new WebSocketService();
