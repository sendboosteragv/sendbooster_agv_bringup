import { useEffect, useRef, useCallback } from 'react';
import { useRobotStore } from '../store/robotStore';
import type { WSMessage, RobotState } from '../types';

const WS_URL = import.meta.env.VITE_WS_URL || `ws://${window.location.host}/ws`;
const RECONNECT_INTERVAL = 3000;

export function useWebSocket() {
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<number | null>(null);

  const { setConnected, setRobotState, addNotification } = useRobotStore();

  const connect = useCallback(() => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      return;
    }

    console.log('[WS] Connecting to', WS_URL);
    const ws = new WebSocket(WS_URL);

    ws.onopen = () => {
      console.log('[WS] Connected');
      setConnected(true);
    };

    ws.onclose = () => {
      console.log('[WS] Disconnected');
      setConnected(false);

      // Attempt to reconnect
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
      }
      reconnectTimeoutRef.current = window.setTimeout(() => {
        console.log('[WS] Attempting to reconnect...');
        connect();
      }, RECONNECT_INTERVAL);
    };

    ws.onerror = (error) => {
      console.error('[WS] Error:', error);
    };

    ws.onmessage = (event) => {
      try {
        const message: WSMessage = JSON.parse(event.data);
        handleMessage(message);
      } catch (error) {
        console.error('[WS] Failed to parse message:', error);
      }
    };

    wsRef.current = ws;
  }, [setConnected]);

  const handleMessage = useCallback(
    (message: WSMessage) => {
      switch (message.type) {
        case 'robot_state':
          setRobotState(message.data as RobotState);
          break;

        case 'path_update':
          // Path is included in robot_state, but can also be sent separately
          break;

        case 'navigation_result': {
          const data = message.data as { status: string; message: string };
          addNotification({
            level: data.status === 'succeeded' ? 'success' : 'warning',
            title: 'Navigation',
            message: data.status === 'succeeded' ? 'Goal reached!' : `Navigation ${data.status}`,
          });
          break;
        }

        case 'notification': {
          const notif = message.data as {
            level: 'info' | 'warning' | 'error' | 'success';
            title: string;
            message: string;
          };
          addNotification(notif);
          break;
        }

        case 'pong':
          // Heartbeat response
          break;

        default:
          console.log('[WS] Unknown message type:', message.type);
      }
    },
    [setRobotState, addNotification]
  );

  const disconnect = useCallback(() => {
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
    }
    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }
  }, []);

  const send = useCallback((type: string, data?: unknown) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ type, data }));
    }
  }, []);

  // Connect on mount, disconnect on unmount
  useEffect(() => {
    connect();
    return () => disconnect();
  }, [connect, disconnect]);

  // Heartbeat
  useEffect(() => {
    const interval = setInterval(() => {
      send('ping');
    }, 30000);
    return () => clearInterval(interval);
  }, [send]);

  return { send };
}
