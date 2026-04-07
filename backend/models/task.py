"""
Task Model - For storing waypoint sequences as tasks
"""
import json
from sqlalchemy import Column, Integer, String, Text, DateTime
from sqlalchemy.sql import func
from database import Base


class Task(Base):
    __tablename__ = "tasks"

    id = Column(Integer, primary_key=True, autoincrement=True)
    name = Column(String, nullable=False)
    description = Column(Text, nullable=True)
    waypoints = Column(Text, nullable=False)  # JSON array of {x, y, yaw}
    created_at = Column(DateTime, server_default=func.now())
    updated_at = Column(DateTime, onupdate=func.now())

    def get_waypoints(self):
        """Parse waypoints JSON"""
        try:
            return json.loads(self.waypoints)
        except (json.JSONDecodeError, TypeError):
            return []

    def set_waypoints(self, waypoints: list):
        """Serialize waypoints to JSON"""
        self.waypoints = json.dumps(waypoints)

    def to_dict(self):
        return {
            "id": self.id,
            "name": self.name,
            "description": self.description,
            "waypoints": self.get_waypoints(),
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None,
        }
