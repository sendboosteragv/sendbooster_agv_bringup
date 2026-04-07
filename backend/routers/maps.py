"""
Maps API Router - Map management
"""
import os
import shutil
import yaml
from typing import Optional
from pathlib import Path

from fastapi import APIRouter, HTTPException, Depends, UploadFile, File, Form
from fastapi.responses import FileResponse
from sqlalchemy.orm import Session
from PIL import Image

from database import get_db
from models.map import Map
from config import MAPS_DIR

router = APIRouter(prefix="/api/maps", tags=["maps"])


@router.get("")
def list_maps(db: Session = Depends(get_db)):
    """Get list of all maps"""
    maps = db.query(Map).all()
    return [m.to_dict() for m in maps]


@router.get("/{map_id}")
def get_map(map_id: int, db: Session = Depends(get_db)):
    """Get map details"""
    map_obj = db.query(Map).filter(Map.id == map_id).first()
    if not map_obj:
        raise HTTPException(status_code=404, detail="Map not found")
    return map_obj.to_dict()


@router.get("/{map_id}/image")
def get_map_image(map_id: int, db: Session = Depends(get_db)):
    """Get map image file"""
    map_obj = db.query(Map).filter(Map.id == map_id).first()
    if not map_obj:
        raise HTTPException(status_code=404, detail="Map not found")

    image_path = MAPS_DIR / map_obj.filename
    if not image_path.exists():
        raise HTTPException(status_code=404, detail="Map image file not found")

    return FileResponse(str(image_path), media_type="image/png")


@router.post("")
async def upload_map(
    name: str = Form(...),
    pgm_file: UploadFile = File(...),
    yaml_file: UploadFile = File(...),
    db: Session = Depends(get_db)
):
    """Upload new map (PGM + YAML)"""
    try:
        # Save PGM file
        pgm_filename = f"{name}.pgm"
        pgm_path = MAPS_DIR / pgm_filename
        with open(pgm_path, "wb") as f:
            content = await pgm_file.read()
            f.write(content)

        # Convert PGM to PNG for web display
        png_filename = f"{name}.png"
        png_path = MAPS_DIR / png_filename
        img = Image.open(pgm_path)
        img.save(png_path, "PNG")

        # Save and parse YAML
        yaml_content = (await yaml_file.read()).decode("utf-8")
        yaml_path = MAPS_DIR / f"{name}.yaml"
        with open(yaml_path, "w") as f:
            f.write(yaml_content)

        yaml_data = yaml.safe_load(yaml_content)

        # Get image dimensions
        width, height = img.size

        # Create database entry
        map_obj = Map(
            name=name,
            filename=png_filename,
            yaml_config=yaml_content,
            resolution=yaml_data.get("resolution", 0.05),
            origin_x=yaml_data.get("origin", [0, 0, 0])[0],
            origin_y=yaml_data.get("origin", [0, 0, 0])[1],
            origin_yaw=yaml_data.get("origin", [0, 0, 0])[2],
            width=width,
            height=height,
            is_active=False
        )

        db.add(map_obj)
        db.commit()
        db.refresh(map_obj)

        return {
            "success": True,
            "message": f"Map '{name}' uploaded successfully",
            "map": map_obj.to_dict()
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to upload map: {str(e)}")


@router.delete("/{map_id}")
def delete_map(map_id: int, db: Session = Depends(get_db)):
    """Delete map"""
    map_obj = db.query(Map).filter(Map.id == map_id).first()
    if not map_obj:
        raise HTTPException(status_code=404, detail="Map not found")

    # Delete files
    for ext in [".png", ".pgm", ".yaml"]:
        file_path = MAPS_DIR / f"{map_obj.name}{ext}"
        if file_path.exists():
            file_path.unlink()

    db.delete(map_obj)
    db.commit()

    return {"success": True, "message": f"Map '{map_obj.name}' deleted"}


@router.post("/{map_id}/activate")
def activate_map(map_id: int, db: Session = Depends(get_db)):
    """Set map as active (for Nav2)"""
    # Deactivate all maps
    db.query(Map).update({"is_active": False})

    # Activate selected map
    map_obj = db.query(Map).filter(Map.id == map_id).first()
    if not map_obj:
        raise HTTPException(status_code=404, detail="Map not found")

    map_obj.is_active = True
    db.commit()

    return {
        "success": True,
        "message": f"Map '{map_obj.name}' activated",
        "map": map_obj.to_dict()
    }


@router.get("/active/info")
def get_active_map(db: Session = Depends(get_db)):
    """Get currently active map"""
    map_obj = db.query(Map).filter(Map.is_active == True).first()
    if not map_obj:
        return {"active": False, "map": None}

    return {"active": True, "map": map_obj.to_dict()}
