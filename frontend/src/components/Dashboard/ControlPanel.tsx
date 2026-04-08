import { useState } from 'react';
import {
  Navigation,
  MapPin,
  Crosshair,
  XCircle,
  Play,
  Square,
  RotateCcw,
  Keyboard,
  Pause,
} from 'lucide-react';
import { useRobotStore } from '../../store/robotStore';
import { robotApi, taskApi } from '../../services/api';
import { useKeyboardControl } from '../../hooks/useKeyboardControl';

export function ControlPanel() {
  const {
    selectedTool,
    setSelectedTool,
    editingWaypoints,
    clearWaypoints,
    robotState,
  } = useRobotStore();

  const [loading, setLoading] = useState(false);
  const [keyboardEnabled, setKeyboardEnabled] = useState(false);

  useKeyboardControl({ enabled: keyboardEnabled });

  const handleCancelNavigation = async () => {
    try {
      setLoading(true);
      await robotApi.cancelGoal('sendbooster_agv');
    } catch (error) {
      console.error('Failed to cancel navigation:', error);
    } finally {
      setLoading(false);
    }
  };

  const handleStop = async () => {
    try {
      setLoading(true);
      await robotApi.stop('sendbooster_agv');
    } catch (error) {
      console.error('Failed to stop robot:', error);
    } finally {
      setLoading(false);
    }
  };

  const handleNavReset = async () => {
    try {
      setLoading(true);
      // Cancel any ongoing navigation
      await robotApi.cancelGoal('sendbooster_agv');
      // Send zero velocity
      await robotApi.sendCmdVel('sendbooster_agv', 0, 0);
    } catch (error) {
      console.error('Failed to reset navigation:', error);
    } finally {
      setLoading(false);
    }
  };

  const handleSendWaypoints = async () => {
    if (editingWaypoints.length === 0) return;

    try {
      setLoading(true);
      await robotApi.sendWaypoints('sendbooster_agv', editingWaypoints);
      clearWaypoints();
      setSelectedTool(null);
    } catch (error) {
      console.error('Failed to send waypoints:', error);
    } finally {
      setLoading(false);
    }
  };

  const isNavigating = robotState?.navigation.status === 'navigating';

  return (
    <div className="bg-white rounded-lg shadow p-4 space-y-4">
      <h2 className="text-lg font-semibold">Control</h2>

      {/* Emergency Controls */}
      <div className="space-y-2">
        <div className="text-sm text-gray-600">Emergency</div>
        <div className="flex gap-2">
          <button
            className="flex-1 flex items-center justify-center gap-2 px-3 py-2 bg-red-500 text-white rounded hover:bg-red-600 disabled:opacity-50"
            onClick={handleStop}
            disabled={loading}
          >
            <Square className="w-4 h-4" />
            <span className="text-sm">STOP</span>
          </button>
          <button
            className="flex-1 flex items-center justify-center gap-2 px-3 py-2 bg-orange-500 text-white rounded hover:bg-orange-600 disabled:opacity-50"
            onClick={handleNavReset}
            disabled={loading}
          >
            <RotateCcw className="w-4 h-4" />
            <span className="text-sm">NAV RESET</span>
          </button>
        </div>
      </div>

      {/* Keyboard Control Toggle */}
      <div className="space-y-2">
        <div className="text-sm text-gray-600">Manual Control</div>
        <button
          className={`w-full flex items-center justify-center gap-2 px-3 py-2 rounded border ${
            keyboardEnabled
              ? 'bg-green-500 text-white border-green-500'
              : 'bg-white text-gray-700 border-gray-300 hover:bg-gray-50'
          }`}
          onClick={() => setKeyboardEnabled(!keyboardEnabled)}
        >
          <Keyboard className="w-4 h-4" />
          <span className="text-sm">
            {keyboardEnabled ? 'Keyboard ON' : 'Keyboard OFF'}
          </span>
        </button>
        {keyboardEnabled && (
          <div className="text-xs text-gray-500 bg-gray-50 p-2 rounded">
            <div className="font-medium mb-1">Controls:</div>
            <div>W/Arrow Up: Forward</div>
            <div>S/Arrow Down: Backward</div>
            <div>A/Arrow Left: Turn Left</div>
            <div>D/Arrow Right: Turn Right</div>
            <div>Space: Emergency Stop</div>
          </div>
        )}
      </div>

      {/* Tool Selection */}
      <div className="space-y-2">
        <div className="text-sm text-gray-600">Navigation Tools</div>
        <div className="flex gap-2">
          <button
            className={`flex-1 flex items-center justify-center gap-2 px-3 py-2 rounded border ${
              selectedTool === 'navigate'
                ? 'bg-primary-500 text-white border-primary-500'
                : 'bg-white text-gray-700 border-gray-300 hover:bg-gray-50'
            }`}
            onClick={() => setSelectedTool(selectedTool === 'navigate' ? null : 'navigate')}
          >
            <Navigation className="w-4 h-4" />
            <span className="text-sm">Goal</span>
          </button>

          <button
            className={`flex-1 flex items-center justify-center gap-2 px-3 py-2 rounded border ${
              selectedTool === 'waypoint'
                ? 'bg-primary-500 text-white border-primary-500'
                : 'bg-white text-gray-700 border-gray-300 hover:bg-gray-50'
            }`}
            onClick={() => setSelectedTool(selectedTool === 'waypoint' ? null : 'waypoint')}
          >
            <MapPin className="w-4 h-4" />
            <span className="text-sm">Waypoints</span>
          </button>

          <button
            className={`flex-1 flex items-center justify-center gap-2 px-3 py-2 rounded border ${
              selectedTool === 'pose'
                ? 'bg-primary-500 text-white border-primary-500'
                : 'bg-white text-gray-700 border-gray-300 hover:bg-gray-50'
            }`}
            onClick={() => setSelectedTool(selectedTool === 'pose' ? null : 'pose')}
          >
            <Crosshair className="w-4 h-4" />
            <span className="text-sm">Pose</span>
          </button>
        </div>
      </div>

      {/* Tool Instructions */}
      {selectedTool && (
        <div className="text-sm text-gray-600 bg-gray-50 p-2 rounded">
          {selectedTool === 'navigate' && 'Click on the map to set navigation goal'}
          {selectedTool === 'waypoint' && 'Click on the map to add waypoints'}
          {selectedTool === 'pose' && 'Click on the map to set initial pose'}
        </div>
      )}

      {/* Waypoints List */}
      {editingWaypoints.length > 0 && (
        <div className="space-y-2">
          <div className="text-sm text-gray-600">
            Waypoints ({editingWaypoints.length})
          </div>
          <div className="max-h-32 overflow-y-auto space-y-1">
            {editingWaypoints.map((wp, idx) => (
              <div
                key={idx}
                className="flex items-center justify-between bg-gray-50 px-2 py-1 rounded text-sm"
              >
                <span>
                  {idx + 1}. ({wp.x.toFixed(2)}, {wp.y.toFixed(2)})
                </span>
              </div>
            ))}
          </div>
          <div className="flex gap-2">
            <button
              className="flex-1 flex items-center justify-center gap-2 px-3 py-2 bg-primary-500 text-white rounded hover:bg-primary-600 disabled:opacity-50"
              onClick={handleSendWaypoints}
              disabled={loading}
            >
              <Play className="w-4 h-4" />
              Execute
            </button>
            <button
              className="px-3 py-2 text-gray-600 border border-gray-300 rounded hover:bg-gray-50"
              onClick={clearWaypoints}
            >
              Clear
            </button>
          </div>
        </div>
      )}

      {/* Cancel Navigation */}
      {isNavigating && (
        <button
          className="w-full flex items-center justify-center gap-2 px-3 py-2 bg-yellow-500 text-white rounded hover:bg-yellow-600 disabled:opacity-50"
          onClick={handleCancelNavigation}
          disabled={loading}
        >
          <Pause className="w-4 h-4" />
          Cancel Navigation
        </button>
      )}
    </div>
  );
}
