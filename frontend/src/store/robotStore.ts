import { create } from 'zustand';
import type { RobotState, MapInfo, Waypoint } from '../types';

interface RobotStore {
  // Connection state
  isConnected: boolean;
  setConnected: (connected: boolean) => void;

  // Robot state
  robotState: RobotState | null;
  setRobotState: (state: RobotState) => void;

  // Active map
  activeMap: MapInfo | null;
  setActiveMap: (map: MapInfo | null) => void;

  // UI state
  selectedTool: 'navigate' | 'waypoint' | 'pose' | null;
  setSelectedTool: (tool: 'navigate' | 'waypoint' | 'pose' | null) => void;

  // Waypoint editing
  editingWaypoints: Waypoint[];
  addWaypoint: (waypoint: Waypoint) => void;
  removeWaypoint: (index: number) => void;
  clearWaypoints: () => void;
  setWaypoints: (waypoints: Waypoint[]) => void;

  // Notifications
  notifications: Array<{
    id: string;
    level: 'info' | 'warning' | 'error' | 'success';
    title: string;
    message: string;
    timestamp: Date;
  }>;
  addNotification: (notification: Omit<RobotStore['notifications'][0], 'id' | 'timestamp'>) => void;
  removeNotification: (id: string) => void;
}

export const useRobotStore = create<RobotStore>((set) => ({
  // Connection
  isConnected: false,
  setConnected: (connected) => set({ isConnected: connected }),

  // Robot state
  robotState: null,
  setRobotState: (state) => set({ robotState: state }),

  // Active map
  activeMap: null,
  setActiveMap: (map) => set({ activeMap: map }),

  // UI state
  selectedTool: null,
  setSelectedTool: (tool) => set({ selectedTool: tool }),

  // Waypoints
  editingWaypoints: [],
  addWaypoint: (waypoint) =>
    set((state) => ({
      editingWaypoints: [...state.editingWaypoints, waypoint],
    })),
  removeWaypoint: (index) =>
    set((state) => ({
      editingWaypoints: state.editingWaypoints.filter((_, i) => i !== index),
    })),
  clearWaypoints: () => set({ editingWaypoints: [] }),
  setWaypoints: (waypoints) => set({ editingWaypoints: waypoints }),

  // Notifications
  notifications: [],
  addNotification: (notification) =>
    set((state) => ({
      notifications: [
        ...state.notifications,
        {
          ...notification,
          id: Math.random().toString(36).substr(2, 9),
          timestamp: new Date(),
        },
      ],
    })),
  removeNotification: (id) =>
    set((state) => ({
      notifications: state.notifications.filter((n) => n.id !== id),
    })),
}));
