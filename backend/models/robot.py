"""
Robot Model - For fleet management (expandable)
"""
from sqlalchemy import Column, String, Text, DateTime
from sqlalchemy.sql import func
from database import Base


class Robot(Base):
    __tablename__ = "robots"

    id = Column(String, primary_key=True)
    name = Column(String, nullable=False)
    description = Column(Text, nullable=True)
    namespace = Column(String, default="")  # ROS2 namespace for fleet
    created_at = Column(DateTime, server_default=func.now())

    def to_dict(self):
        return {
            "id": self.id,
            "name": self.name,
            "description": self.description,
            "namespace": self.namespace,
            "created_at": self.created_at.isoformat() if self.created_at else None,
        }
