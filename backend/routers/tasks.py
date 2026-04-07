"""
Tasks API Router - Task/waypoint sequence management
"""
import json
from typing import List, Optional
from datetime import datetime

from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from sqlalchemy.orm import Session

from database import get_db
from models.task import Task
from models.history import History

router = APIRouter(prefix="/api/tasks", tags=["tasks"])

# Global reference to ROS2 bridge (set from main.py)
ros2_bridge = None


def set_ros2_bridge(bridge):
    """Set ROS2 bridge reference"""
    global ros2_bridge
    ros2_bridge = bridge


class Waypoint(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class TaskCreate(BaseModel):
    name: str
    description: Optional[str] = None
    waypoints: List[Waypoint]


class TaskUpdate(BaseModel):
    name: Optional[str] = None
    description: Optional[str] = None
    waypoints: Optional[List[Waypoint]] = None


@router.get("")
def list_tasks(db: Session = Depends(get_db)):
    """Get list of all tasks"""
    tasks = db.query(Task).all()
    return [t.to_dict() for t in tasks]


@router.get("/{task_id}")
def get_task(task_id: int, db: Session = Depends(get_db)):
    """Get task details"""
    task = db.query(Task).filter(Task.id == task_id).first()
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")
    return task.to_dict()


@router.post("")
def create_task(task_data: TaskCreate, db: Session = Depends(get_db)):
    """Create new task"""
    waypoints = [{"x": wp.x, "y": wp.y, "yaw": wp.yaw} for wp in task_data.waypoints]

    task = Task(
        name=task_data.name,
        description=task_data.description,
    )
    task.set_waypoints(waypoints)

    db.add(task)
    db.commit()
    db.refresh(task)

    return {
        "success": True,
        "message": f"Task '{task.name}' created",
        "task": task.to_dict()
    }


@router.put("/{task_id}")
def update_task(task_id: int, task_data: TaskUpdate, db: Session = Depends(get_db)):
    """Update existing task"""
    task = db.query(Task).filter(Task.id == task_id).first()
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")

    if task_data.name is not None:
        task.name = task_data.name
    if task_data.description is not None:
        task.description = task_data.description
    if task_data.waypoints is not None:
        waypoints = [{"x": wp.x, "y": wp.y, "yaw": wp.yaw} for wp in task_data.waypoints]
        task.set_waypoints(waypoints)

    db.commit()
    db.refresh(task)

    return {
        "success": True,
        "message": f"Task '{task.name}' updated",
        "task": task.to_dict()
    }


@router.delete("/{task_id}")
def delete_task(task_id: int, db: Session = Depends(get_db)):
    """Delete task"""
    task = db.query(Task).filter(Task.id == task_id).first()
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")

    task_name = task.name
    db.delete(task)
    db.commit()

    return {"success": True, "message": f"Task '{task_name}' deleted"}


@router.post("/{task_id}/execute")
def execute_task(task_id: int, robot_id: str = "sendbooster_agv", db: Session = Depends(get_db)):
    """Execute task (send waypoints to robot)"""
    if ros2_bridge is None:
        raise HTTPException(status_code=503, detail="ROS2 bridge not available")

    task = db.query(Task).filter(Task.id == task_id).first()
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")

    waypoints = task.get_waypoints()
    if not waypoints:
        raise HTTPException(status_code=400, detail="Task has no waypoints")

    # Send waypoints to robot
    success = ros2_bridge.send_waypoints(waypoints)
    if not success:
        raise HTTPException(status_code=503, detail="Failed to send waypoints")

    # Record execution in history
    history = History(
        robot_id=robot_id,
        task_id=task_id,
        task_name=task.name,
        status="started",
        started_at=datetime.now()
    )
    db.add(history)
    db.commit()

    return {
        "success": True,
        "message": f"Task '{task.name}' execution started",
        "history_id": history.id
    }
