"""
WebSocket Manager - Handles real-time communication with frontend
"""
import asyncio
import json
from typing import Dict, Set, Optional, Any
from datetime import datetime

from fastapi import WebSocket, WebSocketDisconnect


class WebSocketManager:
    """Manages WebSocket connections and broadcasts"""

    def __init__(self):
        self.active_connections: Dict[str, WebSocket] = {}
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket, client_id: str):
        """Accept new WebSocket connection"""
        await websocket.accept()
        async with self._lock:
            self.active_connections[client_id] = websocket
        print(f"[WS] Client connected: {client_id} (total: {len(self.active_connections)})")

    async def disconnect(self, client_id: str):
        """Handle WebSocket disconnection"""
        async with self._lock:
            if client_id in self.active_connections:
                del self.active_connections[client_id]
        print(f"[WS] Client disconnected: {client_id} (total: {len(self.active_connections)})")

    async def send_personal_message(self, message: dict, client_id: str):
        """Send message to specific client"""
        if client_id in self.active_connections:
            try:
                await self.active_connections[client_id].send_json(message)
            except Exception as e:
                print(f"[WS] Error sending to {client_id}: {e}")
                await self.disconnect(client_id)

    async def broadcast(self, message: dict):
        """Broadcast message to all connected clients"""
        if not self.active_connections:
            return

        disconnected = []
        async with self._lock:
            for client_id, websocket in self.active_connections.items():
                try:
                    await websocket.send_json(message)
                except Exception as e:
                    print(f"[WS] Error broadcasting to {client_id}: {e}")
                    disconnected.append(client_id)

        # Clean up disconnected clients
        for client_id in disconnected:
            await self.disconnect(client_id)

    async def broadcast_robot_state(self, state: dict):
        """Broadcast robot state update"""
        message = {
            "type": "robot_state",
            "data": state,
            "timestamp": datetime.now().isoformat()
        }
        await self.broadcast(message)

    async def broadcast_path_update(self, path: list):
        """Broadcast path update"""
        message = {
            "type": "path_update",
            "data": {"path": path},
            "timestamp": datetime.now().isoformat()
        }
        await self.broadcast(message)

    async def broadcast_nav_result(self, status: str, message_text: str = ""):
        """Broadcast navigation result"""
        message = {
            "type": "navigation_result",
            "data": {
                "status": status,
                "message": message_text
            },
            "timestamp": datetime.now().isoformat()
        }
        await self.broadcast(message)

    async def broadcast_notification(self, level: str, title: str, message_text: str):
        """Broadcast notification to all clients"""
        message = {
            "type": "notification",
            "data": {
                "level": level,  # info, warning, error, success
                "title": title,
                "message": message_text
            },
            "timestamp": datetime.now().isoformat()
        }
        await self.broadcast(message)

    def get_connection_count(self) -> int:
        """Get number of active connections"""
        return len(self.active_connections)

    def is_connected(self, client_id: str) -> bool:
        """Check if client is connected"""
        return client_id in self.active_connections


# Global WebSocket manager instance
ws_manager = WebSocketManager()
