import { Wifi, WifiOff, Bot } from 'lucide-react';
import { useRobotStore } from '../../store/robotStore';

export function Header() {
  const { isConnected, robotState } = useRobotStore();

  return (
    <header className="bg-gray-800 text-white px-4 py-3 flex items-center justify-between">
      <div className="flex items-center gap-3">
        <Bot className="w-8 h-8 text-primary-400" />
        <h1 className="text-xl font-bold">AGV Web Control</h1>
      </div>

      <div className="flex items-center gap-4">
        {/* Connection Status */}
        <div className="flex items-center gap-2">
          {isConnected ? (
            <>
              <Wifi className="w-5 h-5 text-green-400" />
              <span className="text-sm text-green-400">Connected</span>
            </>
          ) : (
            <>
              <WifiOff className="w-5 h-5 text-red-400" />
              <span className="text-sm text-red-400">Disconnected</span>
            </>
          )}
        </div>

        {/* Navigation Status */}
        {robotState && (
          <div className="flex items-center gap-2 px-3 py-1 bg-gray-700 rounded">
            <span className="text-xs text-gray-400">Status:</span>
            <span
              className={`text-sm font-medium ${
                robotState.navigation.status === 'navigating'
                  ? 'text-yellow-400'
                  : robotState.navigation.status === 'succeeded'
                  ? 'text-green-400'
                  : robotState.navigation.status === 'failed'
                  ? 'text-red-400'
                  : 'text-gray-300'
              }`}
            >
              {robotState.navigation.status}
            </span>
          </div>
        )}
      </div>
    </header>
  );
}
