// Robot state types

export interface Position {
  x: number;
  y: number;
  yaw: number;
  yaw_degrees: number;
}

export interface Velocity {
  linear: number;
  angular: number;
}

export interface NavigationGoal {
  x: number | null;
  y: number | null;
  yaw: number | null;
}

export interface Navigation {
  status: 'idle' | 'navigating' | 'succeeded' | 'failed' | 'cancelled';
  goal: NavigationGoal;
}

export interface Sensors {
  lidar: boolean;
  imu: boolean;
}

export interface Timestamps {
  pose: string | null;
  odom: string | null;
}

export interface RobotState {
  position: Position;
  velocity: Velocity;
  navigation: Navigation;
  path: [number, number][];
  sensors: Sensors;
  battery: number;
  timestamps: Timestamps;
}

// Map types
export interface MapInfo {
  id: number;
  name: string;
  filename: string;
  resolution: number;
  origin: {
    x: number;
    y: number;
    yaw: number;
  };
  width: number;
  height: number;
  is_active: boolean;
  created_at: string;
}

// Task types
export interface Waypoint {
  x: number;
  y: number;
  yaw: number;
}

export interface Task {
  id: number;
  name: string;
  description: string | null;
  waypoints: Waypoint[];
  created_at: string;
  updated_at: string | null;
}

// History types
export interface HistoryItem {
  id: number;
  robot_id: string;
  task_id: number | null;
  task_name: string | null;
  status: string;
  started_at: string;
  completed_at: string | null;
  path_taken: [number, number][];
}

// WebSocket message types
export interface WSMessage {
  type: string;
  data: unknown;
  timestamp?: string;
}

export interface RobotStateMessage extends WSMessage {
  type: 'robot_state';
  data: RobotState;
}

export interface PathUpdateMessage extends WSMessage {
  type: 'path_update';
  data: { path: [number, number][] };
}

export interface NavResultMessage extends WSMessage {
  type: 'navigation_result';
  data: { status: string; message: string };
}

export interface NotificationMessage extends WSMessage {
  type: 'notification';
  data: {
    level: 'info' | 'warning' | 'error' | 'success';
    title: string;
    message: string;
  };
}
