"""
Map Model - For storing map files and metadata
"""
from sqlalchemy import Column, Integer, String, Text, Float, Boolean, DateTime
from sqlalchemy.sql import func
from database import Base


class Map(Base):
    __tablename__ = "maps"

    id = Column(Integer, primary_key=True, autoincrement=True)
    name = Column(String, nullable=False)
    filename = Column(String, nullable=False)  # .pgm filename
    yaml_config = Column(Text, nullable=True)  # YAML content
    resolution = Column(Float, nullable=True)
    origin_x = Column(Float, nullable=True)
    origin_y = Column(Float, nullable=True)
    origin_yaw = Column(Float, nullable=True)
    width = Column(Integer, nullable=True)
    height = Column(Integer, nullable=True)
    is_active = Column(Boolean, default=False)
    created_at = Column(DateTime, server_default=func.now())

    def to_dict(self):
        return {
            "id": self.id,
            "name": self.name,
            "filename": self.filename,
            "resolution": self.resolution,
            "origin": {
                "x": self.origin_x,
                "y": self.origin_y,
                "yaw": self.origin_yaw,
            },
            "width": self.width,
            "height": self.height,
            "is_active": self.is_active,
            "created_at": self.created_at.isoformat() if self.created_at else None,
        }
