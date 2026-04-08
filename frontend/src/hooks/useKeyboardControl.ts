import { useEffect, useCallback, useRef } from 'react';
import { robotApi } from '../services/api';

interface KeyboardControlOptions {
  enabled: boolean;
  linearSpeed?: number;
  angularSpeed?: number;
}

export function useKeyboardControl({
  enabled,
  linearSpeed = 0.31,
  angularSpeed = 0.28,
}: KeyboardControlOptions) {
  const keysPressed = useRef<Set<string>>(new Set());
  const intervalRef = useRef<number | null>(null);

  const sendVelocity = useCallback(async () => {
    let linear = 0;
    let angular = 0;

    if (keysPressed.current.has('w') || keysPressed.current.has('ArrowUp')) {
      linear += linearSpeed;
    }
    if (keysPressed.current.has('s') || keysPressed.current.has('ArrowDown')) {
      linear -= linearSpeed;
    }
    if (keysPressed.current.has('a') || keysPressed.current.has('ArrowLeft')) {
      angular += angularSpeed;
    }
    if (keysPressed.current.has('d') || keysPressed.current.has('ArrowRight')) {
      angular -= angularSpeed;
    }

    try {
      await robotApi.sendCmdVel('sendbooster_agv', linear, angular);
    } catch (error) {
      console.error('Failed to send cmd_vel:', error);
    }
  }, [linearSpeed, angularSpeed]);

  const handleKeyDown = useCallback(
    (e: KeyboardEvent) => {
      if (!enabled) return;

      const key = e.key.toLowerCase();
      if (['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
        e.preventDefault();
        keysPressed.current.add(e.key);

        // Start sending velocity commands if not already
        if (intervalRef.current === null) {
          sendVelocity();
          intervalRef.current = window.setInterval(sendVelocity, 100);
        }
      }

      // Space bar for emergency stop
      if (e.key === ' ') {
        e.preventDefault();
        robotApi.stop('sendbooster_agv');
        keysPressed.current.clear();
        if (intervalRef.current !== null) {
          clearInterval(intervalRef.current);
          intervalRef.current = null;
        }
      }
    },
    [enabled, sendVelocity]
  );

  const handleKeyUp = useCallback(
    (e: KeyboardEvent) => {
      if (!enabled) return;

      keysPressed.current.delete(e.key);

      // If no keys pressed, stop sending and send zero velocity
      if (keysPressed.current.size === 0 && intervalRef.current !== null) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
        robotApi.sendCmdVel('sendbooster_agv', 0, 0);
      }
    },
    [enabled]
  );

  useEffect(() => {
    if (enabled) {
      window.addEventListener('keydown', handleKeyDown);
      window.addEventListener('keyup', handleKeyUp);
    }

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      if (intervalRef.current !== null) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
    };
  }, [enabled, handleKeyDown, handleKeyUp]);

  return {
    isActive: keysPressed.current.size > 0,
  };
}
