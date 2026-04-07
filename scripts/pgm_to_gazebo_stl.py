#!/usr/bin/env python3
"""
PGM 맵 → Gazebo용 경량 STL 변환

벽 픽셀을 그룹화하여 박스 메시로 변환합니다.
- 수평으로 연속된 벽 픽셀을 하나의 박스로 병합 (폴리곤 수 대폭 감소)
- 가제보 크래시 방지를 위해 메시를 단순하게 유지
"""

import sys
import numpy as np
from PIL import Image
from stl import mesh as stl_mesh


def pgm_to_wall_boxes(pgm_path, yaml_resolution, yaml_origin, wall_height=1.0, occupied_thresh=50):
    """PGM에서 벽 픽셀을 수평 박스로 병합하여 추출"""
    img = Image.open(pgm_path)
    arr = np.array(img)

    # 벽 마스크 (어두운 픽셀 = 장애물)
    wall_mask = arr < occupied_thresh

    ox, oy = yaml_origin[0], yaml_origin[1]
    boxes = []

    # 행별로 연속된 벽 픽셀을 하나의 박스로 병합
    rows, cols = wall_mask.shape
    for r in range(rows):
        c = 0
        while c < cols:
            if wall_mask[r, c]:
                start_c = c
                while c < cols and wall_mask[r, c]:
                    c += 1
                end_c = c
                # 픽셀 → 월드 좌표 (PGM은 위에서 아래, y는 뒤집어야 함)
                x_min = ox + start_c * yaml_resolution
                x_max = ox + end_c * yaml_resolution
                y_min = oy + (rows - r - 1) * yaml_resolution
                y_max = oy + (rows - r) * yaml_resolution
                boxes.append((x_min, y_min, x_max, y_max))
            else:
                c += 1

    # 세로로도 병합: 같은 x 범위를 가진 연속 행의 박스를 합침
    boxes.sort(key=lambda b: (b[0], b[2], b[1]))
    merged = []
    i = 0
    while i < len(boxes):
        x_min, y_min, x_max, y_max = boxes[i]
        j = i + 1
        while j < len(boxes):
            bx_min, by_min, bx_max, by_max = boxes[j]
            if bx_min == x_min and bx_max == x_max and abs(by_min - y_max) < yaml_resolution * 1.01:
                y_max = by_max
                j += 1
            else:
                break
        merged.append((x_min, y_min, x_max, y_max))
        i = j

    print(f"  Raw boxes: {len(boxes)}, Merged: {len(merged)}")
    return merged, wall_height


def boxes_to_stl(boxes, wall_height, output_path):
    """박스 리스트 → STL 메시"""
    # 각 박스는 12개 삼각형 (6면 × 2)
    total_faces = len(boxes) * 12
    combined = np.zeros(total_faces, dtype=stl_mesh.Mesh.dtype)

    for i, (x0, y0, x1, y1) in enumerate(boxes):
        z0, z1 = 0.0, wall_height
        # 8개 꼭짓점
        v = np.array([
            [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],  # bottom
            [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],  # top
        ])
        # 12개 삼각형 (6면)
        faces = [
            [0,1,2], [0,2,3],  # bottom
            [4,6,5], [4,7,6],  # top
            [0,4,5], [0,5,1],  # front
            [2,6,7], [2,7,3],  # back
            [0,3,7], [0,7,4],  # left
            [1,5,6], [1,6,2],  # right
        ]
        for j, f in enumerate(faces):
            idx = i * 12 + j
            combined['vectors'][idx] = v[f]

    m = stl_mesh.Mesh(combined)
    m.save(output_path)
    size_mb = os.path.getsize(output_path) / 1024 / 1024
    print(f"  STL saved: {output_path} ({size_mb:.1f} MB, {total_faces} faces)")
    return size_mb


import os

def main():
    pgm_path = sys.argv[1] if len(sys.argv) > 1 else '/home/seongmin/ros2_ws/src/sendbooster_agv_bringup/map/my_map.pgm'
    output_path = sys.argv[2] if len(sys.argv) > 2 else '/home/seongmin/ros2_ws/src/sendbooster_agv_bringup/models/map_walls/meshes/map_walls.stl'

    # 파라미터 (--resolution 으로 오버라이드 가능)
    resolution = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
    origin = [-19.7, -48.6, 0]
    wall_height = 1.0

    print(f"Converting {pgm_path} → {output_path}")
    print(f"  Resolution: {resolution}m, Origin: {origin[:2]}, Wall height: {wall_height}m")

    boxes, height = pgm_to_wall_boxes(pgm_path, resolution, origin, wall_height)
    boxes_to_stl(boxes, height, output_path)
    print("Done!")


if __name__ == '__main__':
    main()
