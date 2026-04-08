import axios from 'axios';
import type { MapInfo, Task, Waypoint, HistoryItem } from '../types';

const api = axios.create({
  baseURL: '/api',
  headers: {
    'Content-Type': 'application/json',
  },
});

// Robot API
export const robotApi = {
  list: () => api.get('/robots'),
  getStatus: (robotId: string) => api.get(`/robots/${robotId}/status`),
  sendGoal: (robotId: string, x: number, y: number, yaw: number = 0) =>
    api.post(`/robots/${robotId}/goal`, { x, y, yaw }),
  sendWaypoints: (robotId: string, waypoints: Waypoint[]) =>
    api.post(`/robots/${robotId}/waypoints`, { waypoints }),
  cancelGoal: (robotId: string) => api.delete(`/robots/${robotId}/goal`),
  setInitialPose: (robotId: string, x: number, y: number, yaw: number) =>
    api.post(`/robots/${robotId}/initial_pose`, { x, y, yaw }),
  sendCmdVel: (robotId: string, linear: number, angular: number) =>
    api.post(`/robots/${robotId}/cmd_vel`, { linear, angular }),
  stop: (robotId: string) => api.post(`/robots/${robotId}/stop`),
};

// Map API
export const mapApi = {
  list: () => api.get<MapInfo[]>('/maps'),
  get: (mapId: number) => api.get<MapInfo>(`/maps/${mapId}`),
  getImageUrl: (mapId: number) => `/api/maps/${mapId}/image`,
  upload: (formData: FormData) =>
    api.post('/maps', formData, {
      headers: { 'Content-Type': 'multipart/form-data' },
    }),
  delete: (mapId: number) => api.delete(`/maps/${mapId}`),
  activate: (mapId: number) => api.post(`/maps/${mapId}/activate`),
  getActive: () => api.get('/maps/active/info'),
};

// Task API
export const taskApi = {
  list: () => api.get<Task[]>('/tasks'),
  get: (taskId: number) => api.get<Task>(`/tasks/${taskId}`),
  create: (name: string, waypoints: Waypoint[], description?: string) =>
    api.post('/tasks', { name, waypoints, description }),
  update: (taskId: number, data: Partial<Task>) =>
    api.put(`/tasks/${taskId}`, data),
  delete: (taskId: number) => api.delete(`/tasks/${taskId}`),
  execute: (taskId: number, robotId: string = 'sendbooster_agv') =>
    api.post(`/tasks/${taskId}/execute`, null, { params: { robot_id: robotId } }),
};

// History API
export const historyApi = {
  list: (params?: { limit?: number; offset?: number; robot_id?: string; status?: string }) =>
    api.get<{ total: number; items: HistoryItem[] }>('/history', { params }),
  get: (historyId: number) => api.get<HistoryItem>(`/history/${historyId}`),
};

// Server status
export const getServerStatus = () => api.get('/status');

export default api;
