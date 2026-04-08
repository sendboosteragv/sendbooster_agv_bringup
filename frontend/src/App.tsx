import { useEffect, useState, useRef } from 'react';
import { Header } from './components/common/Header';
import { Notifications } from './components/common/Notifications';
import { StatusPanel } from './components/Dashboard/StatusPanel';
import { ControlPanel } from './components/Dashboard/ControlPanel';
import { MapCanvas } from './components/Map/MapCanvas';
import { useWebSocket } from './hooks/useWebSocket';
import { useRobotStore } from './store/robotStore';

// Default map metadata (from default.yaml)
const DEFAULT_MAP = {
  id: 0,
  name: 'default',
  filename: 'default.png',
  resolution: 0.05,
  origin: { x: -26.9, y: -13.7, yaw: 0.0 },
  width: 1153,
  height: 1317,
  is_active: true,
  created_at: null,
};

function App() {
  useWebSocket();

  const { setActiveMap } = useRobotStore();
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const [mapSize, setMapSize] = useState({ width: 800, height: 600 });

  // Set default map on mount
  useEffect(() => {
    setActiveMap(DEFAULT_MAP);
  }, [setActiveMap]);

  // Handle resize
  useEffect(() => {
    const updateSize = () => {
      if (mapContainerRef.current) {
        setMapSize({
          width: mapContainerRef.current.clientWidth,
          height: mapContainerRef.current.clientHeight,
        });
      }
    };

    updateSize();
    window.addEventListener('resize', updateSize);
    return () => window.removeEventListener('resize', updateSize);
  }, []);

  return (
    <div className="min-h-screen bg-gray-100 flex flex-col">
      <Header />

      <main className="flex-1 p-4 flex gap-4">
        {/* Map Area */}
        <div
          ref={mapContainerRef}
          className="flex-1 bg-white rounded-lg shadow overflow-hidden"
        >
          <MapCanvas width={mapSize.width} height={mapSize.height} />
        </div>

        {/* Sidebar */}
        <div className="w-80 space-y-4 flex-shrink-0">
          <StatusPanel />
          <ControlPanel />
        </div>
      </main>

      <Notifications />
    </div>
  );
}

export default App;
