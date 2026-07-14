#!/usr/bin/env python3
"""
OSCO geometry JSON -> Gazebo SDF world generator

입력:
  osco_scenario_map_v2_geometry.json

출력:
  osco_scenario_world.sdf

설계 원칙:
- Nav2 지도와 같은 geometry JSON을 사용한다.
- 문 틈은 벽에서 비워 둔다.
- 문짝 자체는 다음 단계에서 별도 동적 모델로 추가한다.
- 픽셀 좌표를 미터 좌표로 변환한다.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from xml.sax.saxutils import escape


DEFAULT_WALL_HEIGHT = 2.6
DEFAULT_WALL_THICKNESS = 0.18
DEFAULT_FLOOR_THICKNESS = 0.10


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a Gazebo SDF world from OSCO geometry JSON."
    )
    parser.add_argument(
        "geometry_json",
        type=Path,
        help="Path to osco_scenario_map_v2_geometry.json",
    )
    parser.add_argument(
        "output_sdf",
        type=Path,
        help="Output SDF world file",
    )
    parser.add_argument(
        "--wall-height",
        type=float,
        default=DEFAULT_WALL_HEIGHT,
        help=f"Wall height in meters (default: {DEFAULT_WALL_HEIGHT})",
    )
    parser.add_argument(
        "--wall-thickness",
        type=float,
        default=DEFAULT_WALL_THICKNESS,
        help=f"Wall thickness in meters (default: {DEFAULT_WALL_THICKNESS})",
    )
    return parser.parse_args()


def validate_geometry(data: dict) -> None:
    required = ["resolution_m_per_pixel", "source_crop_pixels", "walls", "columns"]
    missing = [key for key in required if key not in data]
    if missing:
        raise ValueError(f"Missing required keys: {', '.join(missing)}")

    crop = data["source_crop_pixels"]
    for key in ("width", "height"):
        if key not in crop:
            raise ValueError(f"source_crop_pixels.{key} is missing")


def split_intervals(start: float, end: float, gaps: list[list[float]]) -> list[tuple[float, float]]:
    """Split one wall interval around door gaps."""
    if end <= start:
        return []

    cursor = start
    segments: list[tuple[float, float]] = []

    for gap_start, gap_end in sorted(gaps):
        gap_start = max(start, min(end, gap_start))
        gap_end = max(start, min(end, gap_end))

        if gap_end <= gap_start:
            continue

        if gap_start > cursor:
            segments.append((cursor, gap_start))

        cursor = max(cursor, gap_end)

    if cursor < end:
        segments.append((cursor, end))

    return segments


def px_to_world_x(px: float, resolution: float) -> float:
    return px * resolution


def px_to_world_y(py: float, map_height_px: float, resolution: float) -> float:
    # Image coordinates grow downward.
    # Gazebo world coordinates grow upward in +Y.
    return (map_height_px - py) * resolution


def box_elements(
    name: str,
    x: float,
    y: float,
    z: float,
    size_x: float,
    size_y: float,
    size_z: float,
    color: tuple[float, float, float, float],
) -> str:
    r, g, b, a = color
    safe_name = escape(name)
    return f"""
      <collision name="{safe_name}_collision">
        <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
        <geometry>
          <box>
            <size>{size_x:.4f} {size_y:.4f} {size_z:.4f}</size>
          </box>
        </geometry>
      </collision>

      <visual name="{safe_name}_visual">
        <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
        <geometry>
          <box>
            <size>{size_x:.4f} {size_y:.4f} {size_z:.4f}</size>
          </box>
        </geometry>
        <material>
          <ambient>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</diffuse>
        </material>
      </visual>
"""


def generate_wall_boxes(
    data: dict,
    wall_height: float,
    wall_thickness: float,
) -> tuple[list[str], int]:
    resolution = float(data["resolution_m_per_pixel"])
    map_height_px = float(data["source_crop_pixels"]["height"])

    elements: list[str] = []
    count = 0

    for wall in data["walls"]:
        wall_id = str(wall["id"])
        gaps = wall.get("gaps_px", [])

        if wall["type"] == "horizontal":
            x1_px, y_px = wall["p1_px"]
            x2_px, _ = wall["p2_px"]

            for segment_index, (seg_start, seg_end) in enumerate(
                split_intervals(float(x1_px), float(x2_px), gaps)
            ):
                length_m = (seg_end - seg_start) * resolution
                if length_m <= wall_thickness:
                    continue

                center_x = ((seg_start + seg_end) / 2.0) * resolution
                center_y = px_to_world_y(float(y_px), map_height_px, resolution)

                elements.append(
                    box_elements(
                        name=f"{wall_id}_{segment_index}",
                        x=center_x,
                        y=center_y,
                        z=wall_height / 2.0,
                        size_x=length_m,
                        size_y=wall_thickness,
                        size_z=wall_height,
                        color=(0.70, 0.72, 0.75, 1.0),
                    )
                )
                count += 1

        elif wall["type"] == "vertical":
            x_px, y1_px = wall["p1_px"]
            _, y2_px = wall["p2_px"]

            for segment_index, (seg_start, seg_end) in enumerate(
                split_intervals(float(y1_px), float(y2_px), gaps)
            ):
                length_m = (seg_end - seg_start) * resolution
                if length_m <= wall_thickness:
                    continue

                center_x = px_to_world_x(float(x_px), resolution)
                center_y = px_to_world_y(
                    (seg_start + seg_end) / 2.0,
                    map_height_px,
                    resolution,
                )

                elements.append(
                    box_elements(
                        name=f"{wall_id}_{segment_index}",
                        x=center_x,
                        y=center_y,
                        z=wall_height / 2.0,
                        size_x=wall_thickness,
                        size_y=length_m,
                        size_z=wall_height,
                        color=(0.70, 0.72, 0.75, 1.0),
                    )
                )
                count += 1

        else:
            raise ValueError(f"Unknown wall type: {wall['type']}")

    return elements, count


def generate_column_boxes(
    data: dict,
    wall_height: float,
) -> tuple[list[str], int]:
    resolution = float(data["resolution_m_per_pixel"])
    map_height_px = float(data["source_crop_pixels"]["height"])

    elements: list[str] = []
    count = 0

    for column in data["columns"]:
        column_id = str(column["id"])

        if "rect_px" in column:
            x1_px, y1_px, x2_px, y2_px = column["rect_px"]
            size_x = (x2_px - x1_px) * resolution
            size_y = (y2_px - y1_px) * resolution
            center_x = ((x1_px + x2_px) / 2.0) * resolution
            center_y = px_to_world_y(
                (y1_px + y2_px) / 2.0,
                map_height_px,
                resolution,
            )
        else:
            center_x_px, center_y_px = column["center_px"]
            size_x_px, size_y_px = column["size_px"]

            size_x = size_x_px * resolution
            size_y = size_y_px * resolution
            center_x = center_x_px * resolution
            center_y = px_to_world_y(
                center_y_px,
                map_height_px,
                resolution,
            )

        elements.append(
            box_elements(
                name=column_id,
                x=center_x,
                y=center_y,
                z=wall_height / 2.0,
                size_x=size_x,
                size_y=size_y,
                size_z=wall_height,
                color=(0.45, 0.47, 0.50, 1.0),
            )
        )
        count += 1

    return elements, count


def generate_simulation_boundary(
    data: dict,
    wall_height: float,
    wall_thickness: float,
) -> list[str]:
    boundaries = data.get("artificial_boundaries", [])
    if not boundaries:
        return []

    resolution = float(data["resolution_m_per_pixel"])
    width_px = float(data["source_crop_pixels"]["width"])
    height_px = float(data["source_crop_pixels"]["height"])

    width_m = width_px * resolution
    height_m = height_px * resolution

    boundary = boundaries[0]
    gap_start_px, gap_end_px = boundary.get(
        "main_entry_gap_px",
        [width_px * 0.85, width_px * 0.92],
    )

    gap_start_m = gap_start_px * resolution
    gap_end_m = gap_end_px * resolution

    z = wall_height / 2.0
    color = (0.30, 0.32, 0.36, 1.0)

    elements = [
        # North
        box_elements(
            "simulation_boundary_north",
            width_m / 2.0,
            height_m,
            z,
            width_m,
            wall_thickness,
            wall_height,
            color,
        ),
        # West
        box_elements(
            "simulation_boundary_west",
            0.0,
            height_m / 2.0,
            z,
            wall_thickness,
            height_m,
            wall_height,
            color,
        ),
        # East
        box_elements(
            "simulation_boundary_east",
            width_m,
            height_m / 2.0,
            z,
            wall_thickness,
            height_m,
            wall_height,
            color,
        ),
    ]

    if gap_start_m > 0:
        elements.append(
            box_elements(
                "simulation_boundary_south_left",
                gap_start_m / 2.0,
                0.0,
                z,
                gap_start_m,
                wall_thickness,
                wall_height,
                color,
            )
        )

    if gap_end_m < width_m:
        remaining = width_m - gap_end_m
        elements.append(
            box_elements(
                "simulation_boundary_south_right",
                gap_end_m + remaining / 2.0,
                0.0,
                z,
                remaining,
                wall_thickness,
                wall_height,
                color,
            )
        )

    return elements


def build_sdf(
    data: dict,
    wall_height: float,
    wall_thickness: float,
) -> tuple[str, dict]:
    resolution = float(data["resolution_m_per_pixel"])
    map_width_px = float(data["source_crop_pixels"]["width"])
    map_height_px = float(data["source_crop_pixels"]["height"])

    floor_width = map_width_px * resolution
    floor_height = map_height_px * resolution

    wall_elements, wall_count = generate_wall_boxes(
        data,
        wall_height=wall_height,
        wall_thickness=wall_thickness,
    )
    column_elements, column_count = generate_column_boxes(
        data,
        wall_height=wall_height,
    )
    boundary_elements = generate_simulation_boundary(
        data,
        wall_height=wall_height,
        wall_thickness=wall_thickness,
    )

    all_structure_elements = "\n".join(
        wall_elements + column_elements + boundary_elements
    )

    floor_x = floor_width / 2.0
    floor_y = floor_height / 2.0

    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="osco_scenario_world">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>

    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>

    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>

    <scene>
      <ambient>0.65 0.65 0.65 1</ambient>
      <background>0.85 0.88 0.92 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="osco_floor">
      <static>true</static>
      <link name="floor_link">
        <collision name="floor_collision">
          <pose>{floor_x:.4f} {floor_y:.4f} {-DEFAULT_FLOOR_THICKNESS / 2.0:.4f} 0 0 0</pose>
          <geometry>
            <box>
              <size>{floor_width:.4f} {floor_height:.4f} {DEFAULT_FLOOR_THICKNESS:.4f}</size>
            </box>
          </geometry>
        </collision>
        <visual name="floor_visual">
          <pose>{floor_x:.4f} {floor_y:.4f} {-DEFAULT_FLOOR_THICKNESS / 2.0:.4f} 0 0 0</pose>
          <geometry>
            <box>
              <size>{floor_width:.4f} {floor_height:.4f} {DEFAULT_FLOOR_THICKNESS:.4f}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.82 0.82 0.82 1</ambient>
            <diffuse>0.82 0.82 0.82 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="osco_static_structure">
      <static>true</static>
      <link name="structure_link">
{all_structure_elements}
      </link>
    </model>

  </world>
</sdf>
"""

    summary = {
        "world_name": "osco_scenario_world",
        "map_width_m": round(floor_width, 3),
        "map_height_m": round(floor_height, 3),
        "wall_segments": wall_count,
        "columns_or_cores": column_count,
        "boundary_boxes": len(boundary_elements),
        "doors_in_json": len(data.get("doors", [])),
        "door_models_created": 0,
        "note": "Door gaps exist, but door panels are added in the next step.",
    }
    return sdf, summary


def main() -> int:
    args = parse_args()

    if not args.geometry_json.exists():
        print(f"Geometry JSON not found: {args.geometry_json}", file=sys.stderr)
        return 1

    try:
        data = json.loads(args.geometry_json.read_text(encoding="utf-8"))
        validate_geometry(data)

        sdf, summary = build_sdf(
            data,
            wall_height=args.wall_height,
            wall_thickness=args.wall_thickness,
        )

        args.output_sdf.parent.mkdir(parents=True, exist_ok=True)
        args.output_sdf.write_text(sdf, encoding="utf-8")

        summary_path = args.output_sdf.with_suffix(".summary.json")
        summary_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        print(f"Created SDF world: {args.output_sdf}")
        print(f"Created summary:   {summary_path}")
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0

    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"Generation failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
