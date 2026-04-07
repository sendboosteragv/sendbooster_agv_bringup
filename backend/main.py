"""
AGV Web Control - Main FastAPI Application
"""
import asyncio
import uuid
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from config import HOST, PORT, MAPS_DIR, WS_BROADCAST_INTERVAL
from database import init_db
from websocket.manager import ws_manager
from ros2_bridge.node import ROS2BridgeThread

from routers import robots, maps, tasks, history


# Global ROS2 bridge instance
ros2_bridge: ROS2BridgeThread = None
broadcast_task: asyncio.Task = None


async def broadcast_robot_state():
    """Background task to broadcast robot state"""
    while True:
        try:
            if ros2_bridge and ws_manager.get_connection_count() > 0:
                state = ros2_bridge.get_state()
                await ws_manager.broadcast_robot_state(state)
            await asyncio.sleep(WS_BROADCAST_INTERVAL)
        except Exception as e:
            print(f"[Broadcast] Error: {e}")
            await asyncio.sleep(1.0)


def ros2_state_callback(event_type: str, state):
    """Callback from ROS2 bridge on state changes"""
    # This runs in ROS2 thread, schedule async broadcast
    if event_type == "nav_result":
        asyncio.run_coroutine_threadsafe(
            ws_manager.broadcast_nav_result(state.nav_status),
            asyncio.get_event_loop()
        )


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan - startup and shutdown"""
    global ros2_bridge, broadcast_task

    print("[Startup] Initializing database...")
    init_db()

    print("[Startup] Starting ROS2 bridge...")
    ros2_bridge = ROS2BridgeThread(state_callback=ros2_state_callback)
    ros2_bridge.start()

    # Set ROS2 bridge reference in routers
    robots.set_ros2_bridge(ros2_bridge)
    tasks.set_ros2_bridge(ros2_bridge)

    print("[Startup] Starting broadcast task...")
    broadcast_task = asyncio.create_task(broadcast_robot_state())

    print("[Startup] Server ready!")
    yield

    print("[Shutdown] Stopping broadcast task...")
    if broadcast_task:
        broadcast_task.cancel()
        try:
            await broadcast_task
        except asyncio.CancelledError:
            pass

    print("[Shutdown] Stopping ROS2 bridge...")
    if ros2_bridge:
        ros2_bridge.stop()

    print("[Shutdown] Complete")


# Create FastAPI app
app = FastAPI(
    title="AGV Web Control",
    description="Web-based control interface for AGV robots",
    version="1.0.0",
    lifespan=lifespan
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify allowed origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Mount static files for maps
app.mount("/static/maps", StaticFiles(directory=str(MAPS_DIR)), name="maps")

# Include routers
app.include_router(robots.router)
app.include_router(maps.router)
app.include_router(tasks.router)
app.include_router(history.router)


@app.get("/")
def root():
    """Root endpoint"""
    return {
        "name": "AGV Web Control",
        "version": "1.0.0",
        "status": "running"
    }


@app.get("/api/status")
def get_status():
    """Get server status"""
    return {
        "ros2_connected": ros2_bridge is not None and ros2_bridge.node is not None,
        "websocket_clients": ws_manager.get_connection_count()
    }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time updates"""
    client_id = str(uuid.uuid4())

    await ws_manager.connect(websocket, client_id)

    try:
        # Send initial state
        if ros2_bridge:
            state = ros2_bridge.get_state()
            await ws_manager.send_personal_message({
                "type": "robot_state",
                "data": state
            }, client_id)

        # Keep connection alive and handle messages
        while True:
            data = await websocket.receive_json()

            # Handle client messages
            msg_type = data.get("type")

            if msg_type == "ping":
                await ws_manager.send_personal_message({"type": "pong"}, client_id)

            elif msg_type == "get_state":
                if ros2_bridge:
                    state = ros2_bridge.get_state()
                    await ws_manager.send_personal_message({
                        "type": "robot_state",
                        "data": state
                    }, client_id)

    except WebSocketDisconnect:
        await ws_manager.disconnect(client_id)
    except Exception as e:
        print(f"[WS] Error: {e}")
        await ws_manager.disconnect(client_id)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host=HOST, port=PORT)
