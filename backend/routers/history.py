"""
History API Router - Execution history and logs
"""
from typing import Optional
from datetime import datetime, timedelta

from fastapi import APIRouter, HTTPException, Depends, Query
from sqlalchemy.orm import Session
from sqlalchemy import desc

from database import get_db
from models.history import History, Log

router = APIRouter(prefix="/api", tags=["history"])


@router.get("/history")
def get_history(
    limit: int = Query(50, ge=1, le=200),
    offset: int = Query(0, ge=0),
    robot_id: Optional[str] = None,
    status: Optional[str] = None,
    db: Session = Depends(get_db)
):
    """Get execution history"""
    query = db.query(History)

    if robot_id:
        query = query.filter(History.robot_id == robot_id)
    if status:
        query = query.filter(History.status == status)

    total = query.count()
    items = query.order_by(desc(History.started_at)).offset(offset).limit(limit).all()

    return {
        "total": total,
        "items": [h.to_dict() for h in items]
    }


@router.get("/history/{history_id}")
def get_history_detail(history_id: int, db: Session = Depends(get_db)):
    """Get history detail"""
    history = db.query(History).filter(History.id == history_id).first()
    if not history:
        raise HTTPException(status_code=404, detail="History not found")
    return history.to_dict()


@router.get("/logs")
def get_logs(
    limit: int = Query(100, ge=1, le=500),
    offset: int = Query(0, ge=0),
    level: Optional[str] = None,
    source: Optional[str] = None,
    since: Optional[str] = None,  # ISO format datetime
    db: Session = Depends(get_db)
):
    """Get system logs"""
    query = db.query(Log)

    if level:
        query = query.filter(Log.level == level)
    if source:
        query = query.filter(Log.source == source)
    if since:
        try:
            since_dt = datetime.fromisoformat(since)
            query = query.filter(Log.timestamp >= since_dt)
        except ValueError:
            pass

    total = query.count()
    items = query.order_by(desc(Log.timestamp)).offset(offset).limit(limit).all()

    return {
        "total": total,
        "items": [log.to_dict() for log in items]
    }


def add_log(db: Session, level: str, source: str, message: str):
    """Helper function to add log entry"""
    log = Log(
        level=level,
        source=source,
        message=message
    )
    db.add(log)
    db.commit()
    return log
