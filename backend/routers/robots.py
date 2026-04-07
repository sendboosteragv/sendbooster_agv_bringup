"""
Robot API Router - Robot status and navigation control
"""
from typing import List, Optional
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from sqlalchemy.orm import Session

from database import get_db
from models.robot import Robot
from config import DEFAULT_ROBOT_ID

router = APIRouter(prefix="/api/robots", tags=["robots"])


class RobotCreate(BaseModel):
    id: str
    name: str
    description: Optional[str] = None
    namespace: Optional[str] = ""


class GoalRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class WaypointsRequest(BaseModel):
    waypoints: List[GoalRequest]


class InitialPoseRequest(BaseModel):
    x: float
    y: float
    yaw: float


# Global reference to ROS2 bridge (set from main.py)
ros2_bridge = None


def set_ros2_bridge(bridge):
    """Set ROS2 bridge reference"""
    global ros2_bridge
    ros2_bridge = bridge


@router.get("")
def list_robots(db: Session = Depends(get_db)):
    """Get list of all robots"""
    robots = db.query(Robot).all()

    # If no robots in DB, return default robot
    if not robots:
        return [{
            "id": DEFAULT_ROBOT_ID,
            "name": "Sendbooster AGV",
            "description": "Default AGV robot",
            "namespace": "",
        }]

    return [r.to_dict() for r in robots]


@router.get("/{robot_id}/status")
def get_robot_status(robot_id: str):
    """Get current robot status"""
    if ros2_bridge is None:
        raise HTTPException(status_code=503, detail="ROS2 bridge not available")

    state = ros2_bridge.get_state()
    return {
        "robot_id": robot_id,
        "connected": True,
        **state
    }


@router.post("/{robot_id}/goal")
def send_goal(robot_id: str, goal: GoalRequest):
    """Send navigation goal to robot"""
    if ros2_bridge is None:
        raise HTTPException(status_code=503, detail="ROS2 bridge not available")

    success = ros2_bridge.send_goal(goal.x, goal.y, goal.yaw)
    if not success:
        raise HTTPException(status_code=503, detail="Failed to send goal")

    return {
        "success": True,
        "message": f"Goal sent: ({goal.x}, {goal.y}, {goal.yaw})"
    }


@router.post("/{robot_id}/waypoints")
def send_waypoints(robot_id: str, request: WaypointsRequest):
    """Send multiple waypoints to robot"""
    if ros2_bridge is None:
        raise HTTPException(status_code=503, detail="ROS2 bridge not available")

    waypoints = [{"x": wp.x, "y": wp.y, "yaw": wp.yaw} for wp in request.waypoints]
    success = ros2_bridge.send_waypoints(waypoints)

    if not success:
        raise HTTPException(status_code=503, detail="Failed to send waypoints")

    return {
        "success": True,
        "message": f"Sent {len(waypoints)} waypoints"
    }


@router.delete("/{robot_id}/goal")
def cancel_goal(robot_id: str):
    """Cancel current navigation"""
    if ros2_bridge is None:
        raise HTTPException(status_code=503, detail="ROS2 bridge not available")

    ros2_bridge.cancel_goal()
    return {
        "success": True,
        "message": "Navigation cancelled"
    }


@router.post("/{robot_id}/initial_pose")
def set_initial_pose(robot_id: str, pose: InitialPoseRequest):
    """Set initial pose for localization"""
    if ros2_bridge is None:
        raise HTTPException(status_code=503, detail="ROS2 bridge not available")

    ros2_bridge.set_initial_pose(pose.x, pose.y, pose.yaw)
    return {
        "success": True,
        "message": f"Initial pose set: ({pose.x}, {pose.y}, {pose.yaw})"
    }


class CmdVelRequest(BaseModel):
    linear: float = 0.0
    angular: float = 0.0


@router.post("/{robot_id}/cmd_vel")
def send_cmd_vel(robot_id: str, cmd: CmdVelRequest):
    """Send velocity command for manual control"""
    if ros2_bridge is None:
        raise HTTPException(status_code=503, detail="ROS2 bridge not available")

    ros2_bridge.send_cmd_vel(cmd.linear, cmd.angular)
    return {
        "success": True,
        "linear": cmd.linear,
        "angular": cmd.angular
    }


@router.post("/{robot_id}/stop")
def stop_robot(robot_id: str):
    """Emergency stop - send zero velocity"""
    if ros2_bridge is None:
        raise HTTPException(status_code=503, detail="ROS2 bridge not available")

    ros2_bridge.send_cmd_vel(0.0, 0.0)
    ros2_bridge.cancel_goal()
    return {
        "success": True,
        "message": "Robot stopped"
    }
