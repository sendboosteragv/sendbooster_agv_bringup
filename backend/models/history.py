"""
History and Log Models - For storing execution history and system logs
"""
import json
from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey
from sqlalchemy.sql import func
from database import Base


class History(Base):
    __tablename__ = "history"

    id = Column(Integer, primary_key=True, autoincrement=True)
    robot_id = Column(String, nullable=True)
    task_id = Column(Integer, ForeignKey("tasks.id"), nullable=True)
    task_name = Column(String, nullable=True)
    status = Column(String, nullable=False)  # completed, failed, cancelled
    started_at = Column(DateTime, nullable=True)
    completed_at = Column(DateTime, nullable=True)
    path_taken = Column(Text, nullable=True)  # JSON array of positions

    def get_path(self):
        """Parse path JSON"""
        try:
            return json.loads(self.path_taken) if self.path_taken else []
        except (json.JSONDecodeError, TypeError):
            return []

    def to_dict(self):
        return {
            "id": self.id,
            "robot_id": self.robot_id,
            "task_id": self.task_id,
            "task_name": self.task_name,
            "status": self.status,
            "started_at": self.started_at.isoformat() if self.started_at else None,
            "completed_at": self.completed_at.isoformat() if self.completed_at else None,
            "path_taken": self.get_path(),
        }


class Log(Base):
    __tablename__ = "logs"

    id = Column(Integer, primary_key=True, autoincrement=True)
    level = Column(String, nullable=False)  # info, warning, error
    source = Column(String, nullable=True)
    message = Column(Text, nullable=False)
    timestamp = Column(DateTime, server_default=func.now())

    def to_dict(self):
        return {
            "id": self.id,
            "level": self.level,
            "source": self.source,
            "message": self.message,
            "timestamp": self.timestamp.isoformat() if self.timestamp else None,
        }
