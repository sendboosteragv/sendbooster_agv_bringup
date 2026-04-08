import { useEffect, useRef, useState, useCallback } from 'react';
import { Stage, Layer, Image as KonvaImage, Circle, Line, Arrow } from 'react-konva';
import { useRobotStore } from '../../store/robotStore';
import { robotApi } from '../../services/api';

interface MapCanvasProps {
  width: number;
  height: number;
}

export function MapCanvas({ width, height }: MapCanvasProps) {
  const {
    robotState,
    activeMap,
    selectedTool,
    editingWaypoints,
    addWaypoint,
    setSelectedTool,
  } = useRobotStore();

  const [mapImage, setMapImage] = useState<HTMLImageElement | null>(null);
  const [keepoutImage, setKeepoutImage] = useState<HTMLImageElement | null>(null);
  const [scale, setScale] = useState(1);
  const [offset, setOffset] = useState({ x: 0, y: 0 });
  const stageRef = useRef<any>(null);

  // Map configuration
  const resolution = activeMap?.resolution || 0.05; // meters per pixel
  const originX = activeMap?.origin?.x || 0;
  const originY = activeMap?.origin?.y || 0;
  const mapWidth = activeMap?.width || 0;
  const mapHeight = activeMap?.height || 0;

  // Load map image
  useEffect(() => {
    if (activeMap) {
      const img = new Image();
      img.src = `/maps/${activeMap.filename}`;
      img.onload = () => {
        setMapImage(img);
        // Fit map to canvas
        const scaleX = width / img.width;
        const scaleY = height / img.height;
        setScale(Math.min(scaleX, scaleY) * 0.9);
        setOffset({
          x: (width - img.width * Math.min(scaleX, scaleY) * 0.9) / 2,
          y: (height - img.height * Math.min(scaleX, scaleY) * 0.9) / 2,
        });
      };

      // Load keepout overlay
      const keepout = new Image();
      keepout.src = '/maps/keepout.png';
      keepout.onload = () => setKeepoutImage(keepout);
      keepout.onerror = () => setKeepoutImage(null);
    }
  }, [activeMap, width, height]);

  // Convert world coordinates to canvas coordinates
  const worldToCanvas = useCallback(
    (worldX: number, worldY: number) => {
      // Map coordinates: origin is at bottom-left
      // Canvas coordinates: origin is at top-left
      const pixelX = (worldX - originX) / resolution;
      const pixelY = mapHeight - (worldY - originY) / resolution;
      return {
        x: offset.x + pixelX * scale,
        y: offset.y + pixelY * scale,
      };
    },
    [originX, originY, resolution, mapHeight, scale, offset]
  );

  // Convert canvas coordinates to world coordinates
  const canvasToWorld = useCallback(
    (canvasX: number, canvasY: number) => {
      const pixelX = (canvasX - offset.x) / scale;
      const pixelY = (canvasY - offset.y) / scale;
      return {
        x: pixelX * resolution + originX,
        y: (mapHeight - pixelY) * resolution + originY,
      };
    },
    [originX, originY, resolution, mapHeight, scale, offset]
  );

  // Handle canvas click
  const handleStageClick = async (e: any) => {
    if (!selectedTool) return;

    const stage = stageRef.current;
    const pointerPos = stage.getPointerPosition();

    // Account for stage position (drag offset)
    const stagePos = stage.position();
    const adjustedX = pointerPos.x - stagePos.x;
    const adjustedY = pointerPos.y - stagePos.y;

    const world = canvasToWorld(adjustedX, adjustedY);

    if (selectedTool === 'navigate') {
      try {
        await robotApi.sendGoal('sendbooster_agv', world.x, world.y, 0);
        setSelectedTool(null);
      } catch (error) {
        console.error('Failed to send goal:', error);
      }
    } else if (selectedTool === 'waypoint') {
      addWaypoint({ x: world.x, y: world.y, yaw: 0 });
    } else if (selectedTool === 'pose') {
      try {
        await robotApi.setInitialPose('sendbooster_agv', world.x, world.y, 0);
        setSelectedTool(null);
      } catch (error) {
        console.error('Failed to set initial pose:', error);
      }
    }
  };

  // Handle wheel zoom (zoom toward mouse pointer)
  const handleWheel = (e: any) => {
    e.evt.preventDefault();
    const scaleBy = 1.1;
    const stage = stageRef.current;
    const oldScale = scale;
    const pointerPos = stage.getPointerPosition();
    const stagePos = stage.position();

    // Mouse position relative to stage content
    const mousePointTo = {
      x: (pointerPos.x - stagePos.x - offset.x) / oldScale,
      y: (pointerPos.y - stagePos.y - offset.y) / oldScale,
    };

    const newScale = e.evt.deltaY > 0 ? oldScale / scaleBy : oldScale * scaleBy;
    const clampedScale = Math.max(0.1, Math.min(5, newScale));

    // Adjust offset to zoom toward mouse pointer
    const newOffset = {
      x: pointerPos.x - stagePos.x - mousePointTo.x * clampedScale,
      y: pointerPos.y - stagePos.y - mousePointTo.y * clampedScale,
    };

    setScale(clampedScale);
    setOffset(newOffset);
  };

  // Robot position in canvas coordinates
  const robotCanvas = robotState
    ? worldToCanvas(robotState.position.x, robotState.position.y)
    : null;

  // Goal position
  const goalCanvas =
    robotState?.navigation.goal.x != null
      ? worldToCanvas(robotState.navigation.goal.x, robotState.navigation.goal.y!)
      : null;

  // Path in canvas coordinates
  const pathCanvas = robotState?.path.map(([x, y]) => worldToCanvas(x, y)) || [];

  // Waypoints in canvas coordinates
  const waypointsCanvas = editingWaypoints.map((wp) => worldToCanvas(wp.x, wp.y));

  return (
    <Stage
      ref={stageRef}
      width={width}
      height={height}
      onClick={handleStageClick}
      onWheel={handleWheel}
      draggable
    >
      <Layer>
        {/* Map Image */}
        {mapImage && (
          <KonvaImage
            image={mapImage}
            x={offset.x}
            y={offset.y}
            scaleX={scale}
            scaleY={scale}
          />
        )}

        {/* Keepout Zone Overlay */}
        {keepoutImage && (
          <KonvaImage
            image={keepoutImage}
            x={offset.x}
            y={offset.y}
            scaleX={scale}
            scaleY={scale}
          />
        )}

        {/* Planned Path */}
        {pathCanvas.length > 1 && (
          <Line
            points={pathCanvas.flatMap((p) => [p.x, p.y])}
            stroke="#22c55e"
            strokeWidth={2}
            dash={[5, 5]}
          />
        )}

        {/* Editing Waypoints */}
        {waypointsCanvas.map((wp, idx) => (
          <Circle
            key={idx}
            x={wp.x}
            y={wp.y}
            radius={8}
            fill="#3b82f6"
            stroke="#1d4ed8"
            strokeWidth={2}
          />
        ))}

        {/* Goal */}
        {goalCanvas && (
          <Circle
            x={goalCanvas.x}
            y={goalCanvas.y}
            radius={10}
            fill="#f59e0b"
            stroke="#d97706"
            strokeWidth={2}
          />
        )}

        {/* Robot */}
        {robotCanvas && (
          <>
            {/* Robot body */}
            <Circle
              x={robotCanvas.x}
              y={robotCanvas.y}
              radius={15}
              fill="#ef4444"
              stroke="#b91c1c"
              strokeWidth={2}
            />
            {/* Robot direction arrow */}
            <Arrow
              x={robotCanvas.x}
              y={robotCanvas.y}
              points={[
                0,
                0,
                25 * Math.cos(-robotState!.position.yaw),
                25 * Math.sin(-robotState!.position.yaw),
              ]}
              pointerLength={8}
              pointerWidth={8}
              fill="#b91c1c"
              stroke="#b91c1c"
              strokeWidth={2}
            />
          </>
        )}
      </Layer>
    </Stage>
  );
}
