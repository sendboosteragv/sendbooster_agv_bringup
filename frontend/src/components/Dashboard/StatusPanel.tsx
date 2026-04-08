import { Battery, Navigation, Gauge, Radio } from 'lucide-react';
import { useRobotStore } from '../../store/robotStore';

export function StatusPanel() {
  const { robotState, isConnected } = useRobotStore();

  if (!robotState) {
    return (
      <div className="bg-white rounded-lg shadow p-4">
        <h2 className="text-lg font-semibold mb-4">Robot Status</h2>
        <p className="text-gray-500">
          {isConnected ? 'Waiting for data...' : 'Not connected'}
        </p>
      </div>
    );
  }

  const { position, velocity, sensors, battery } = robotState;

  return (
    <div className="bg-white rounded-lg shadow p-4 space-y-4">
      <h2 className="text-lg font-semibold">Robot Status</h2>

      {/* Position */}
      <div className="space-y-2">
        <div className="flex items-center gap-2 text-sm text-gray-600">
          <Navigation className="w-4 h-4" />
          <span>Position</span>
        </div>
        <div className="grid grid-cols-3 gap-2 text-sm">
          <div className="bg-gray-100 rounded p-2">
            <div className="text-xs text-gray-500">X</div>
            <div className="font-mono">{position.x.toFixed(2)} m</div>
          </div>
          <div className="bg-gray-100 rounded p-2">
            <div className="text-xs text-gray-500">Y</div>
            <div className="font-mono">{position.y.toFixed(2)} m</div>
          </div>
          <div className="bg-gray-100 rounded p-2">
            <div className="text-xs text-gray-500">Yaw</div>
            <div className="font-mono">{position.yaw_degrees.toFixed(1)}&deg;</div>
          </div>
        </div>
      </div>

      {/* Velocity */}
      <div className="space-y-2">
        <div className="flex items-center gap-2 text-sm text-gray-600">
          <Gauge className="w-4 h-4" />
          <span>Velocity</span>
        </div>
        <div className="grid grid-cols-2 gap-2 text-sm">
          <div className="bg-gray-100 rounded p-2">
            <div className="text-xs text-gray-500">Linear</div>
            <div className="font-mono">{velocity.linear.toFixed(2)} m/s</div>
          </div>
          <div className="bg-gray-100 rounded p-2">
            <div className="text-xs text-gray-500">Angular</div>
            <div className="font-mono">{velocity.angular.toFixed(2)} rad/s</div>
          </div>
        </div>
      </div>

      {/* Battery */}
      <div className="space-y-2">
        <div className="flex items-center gap-2 text-sm text-gray-600">
          <Battery className="w-4 h-4" />
          <span>Battery</span>
        </div>
        <div className="flex items-center gap-2">
          <div className="flex-1 bg-gray-200 rounded-full h-3">
            <div
              className={`h-3 rounded-full ${
                battery > 50 ? 'bg-green-500' : battery > 20 ? 'bg-yellow-500' : 'bg-red-500'
              }`}
              style={{ width: `${battery}%` }}
            />
          </div>
          <span className="text-sm font-mono w-12">{battery.toFixed(0)}%</span>
        </div>
      </div>

      {/* Sensors */}
      <div className="space-y-2">
        <div className="flex items-center gap-2 text-sm text-gray-600">
          <Radio className="w-4 h-4" />
          <span>Sensors</span>
        </div>
        <div className="flex gap-4 text-sm">
          <div className="flex items-center gap-1">
            <div
              className={`w-2 h-2 rounded-full ${sensors.lidar ? 'bg-green-500' : 'bg-red-500'}`}
            />
            <span>LiDAR</span>
          </div>
          <div className="flex items-center gap-1">
            <div
              className={`w-2 h-2 rounded-full ${sensors.imu ? 'bg-green-500' : 'bg-red-500'}`}
            />
            <span>IMU</span>
          </div>
        </div>
      </div>
    </div>
  );
}
