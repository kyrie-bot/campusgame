#!/usr/bin/env python3
"""从可编辑的公开场地描图生成离线栅格，不连接任何比赛接口。"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


def positive_number(value, name, allow_zero=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(name + " must be a number")
    if not math.isfinite(value) or value < 0 or (value == 0 and not allow_zero):
        raise ValueError(name + " is outside its valid range")
    return float(value)


def validate_geometry(config):
    if config.get("schema_version") != 1:
        raise ValueError("Unsupported geometry schema")
    source = config["source"]
    grid = config["grid"]
    width_m = positive_number(source["nominal_width_m"], "width")
    height_m = positive_number(source["nominal_height_m"], "height")
    resolution = positive_number(grid["resolution_m"], "resolution")
    width = math.ceil(width_m / resolution - 1e-9)
    height = math.ceil(height_m / resolution - 1e-9)
    if width * height > 1000000:
        raise ValueError("Grid exceeds the offline generator size limit")
    positive_number(grid["boundary_m"], "boundary")
    positive_number(grid["robot_radius_m"], "robot radius", True)
    positive_number(grid["safety_margin_m"], "safety margin", True)
    if grid["boundary_m"] * 2 >= min(width_m, height_m):
        raise ValueError("Boundary covers the whole field")
    # 本工具只生成描图坐标，不提供世界坐标标定或可信地图认证。
    if grid["geometry_verified"] is not False or grid["registration_verified"] is not False:
        raise ValueError("This draft generator cannot certify geometry or registration")
    if grid["frame_id"] not in ("arena_draft", "arena_campus"):
        raise ValueError("Unregistered map must use arena_draft or arena_campus frame")
    names = set()
    for region in config["regions"]:
        if region["name"] in names or region["kind"] not in ("unknown", "obstacle", "platform", "walkable_highland"):
            raise ValueError("Duplicate region or invalid region kind")
        if region["kind"] == "walkable_highland" and region.get("user_confirmed_climbable") is not True:
            raise ValueError("Walkable highland requires explicit climbability confirmation")
        names.add(region["name"])
        vertices = region["vertices_uv"]
        if len(vertices) < 3:
            raise ValueError("Region needs at least three vertices")
        for point in vertices:
            if len(point) != 2:
                raise ValueError("Vertex needs u and v")
            for value in point:
                positive_number(value, "vertex", True)
                if value > 1:
                    raise ValueError("Vertex outside the source rectangle")
        contour = np.array(vertices, dtype=np.float32)
        if cv2.contourArea(contour) <= 1e-8:
            raise ValueError("Degenerate polygon")
    for wall in config.get("walls", []):
        if wall["name"] in names or len(wall["points_uv"]) < 2:
            raise ValueError("Duplicate wall or missing endpoints")
        names.add(wall["name"])
        positive_number(wall["thickness_m"], "wall thickness")
        for point in wall["points_uv"]:
            if len(point) != 2 or not all(isinstance(v, (int, float)) and math.isfinite(v) and 0 <= v <= 1 for v in point):
                raise ValueError("Invalid wall vertex")
    ramps = config.get("ramp_access", {})
    for entry in ramps.get("confirmed_entries", []):
        if entry.get("user_confirmed_position") is not True:
            raise ValueError("Ramp gate requires an explicitly confirmed position")
        vertices = entry["vertices_uv"]
        if len(vertices) < 3 or any(len(p) != 2 or any(not isinstance(v, (int, float)) or
                not math.isfinite(v) or not 0 <= v <= 1 for v in p) for p in vertices):
            raise ValueError("Invalid ramp gate polygon")
        surface = entry.get("surface_vertices_uv")
        if surface is not None and (len(surface) < 3 or any(len(p) != 2 or
                any(not isinstance(v, (int, float)) or not math.isfinite(v) or
                    not 0 <= v <= 1 for v in p) for p in surface)):
            raise ValueError("Invalid slope surface polygon")
    if any(r["kind"] == "platform" for r in config["regions"]):
        positive_number(ramps.get("ledge_thickness_m"), "ledge thickness")
    for entry in config.get("navigation_clearance_overrides", []):
        margin = positive_number(entry["safety_margin_m"], "local safety margin", True)
        if margin > grid["safety_margin_m"] or margin < .02:
            raise ValueError("Local clearance must retain at least 2 cm extra margin")
        vertices = entry["vertices_uv"]
        if len(vertices) < 3 or any(len(p) != 2 or any(not isinstance(v, (int, float)) or
                not math.isfinite(v) or not 0 <= v <= 1 for v in p) for p in vertices):
            raise ValueError("Invalid local clearance polygon")
    return width, height, resolution


def polygon_mask(vertices, width, height, width_m, height_m, resolution):
    # 四倍采样后取 max；窄的墙段也不会因只检查格子中心而消失。
    samples = 4
    points = []
    for u, v in vertices:
        x = round(u * width_m / resolution * samples)
        y = round((1 - v) * height_m / resolution * samples)
        points.append([x, y])
    mask = np.zeros((height * samples, width * samples), dtype=np.uint8)
    cv2.fillPoly(mask, [np.array(points, dtype=np.int32)], 1)
    cells = mask.reshape(height, samples, width, samples)
    return cells.max(axis=(1, 3)).astype(bool)


def region_mask(region, geometry, width, height, resolution):
    """Rasterize a mirrored pit from its upper partner to avoid half-cell bias.

    fillPoly uses inclusive integer endpoints; independently rounding mirrored
    contours can add a row to only the lower pit. Mirror the finished cell mask
    only when the vector polygons themselves are exact 180-degree partners.
    Other obstacles and deliberately asymmetric pits keep their own contours.
    """
    source = geometry['source']
    vertices = np.asarray(region['vertices_uv'], dtype=float)
    if region.get('terrain_feature') == 'pit' and region['kind'] == 'obstacle' and vertices[:, 1].mean() > .5:
        reflected = 1 - vertices
        for partner in geometry['regions']:
            if partner.get('terrain_feature') != 'pit' or partner['kind'] != 'obstacle':
                continue
            candidate = np.asarray(partner['vertices_uv'], dtype=float)
            if candidate.shape != reflected.shape or candidate[:, 1].mean() >= .5:
                continue
            if any(np.allclose(np.roll(order, shift, axis=0), reflected, atol=1e-9, rtol=0)
                   for order in (candidate, candidate[::-1]) for shift in range(len(candidate))):
                upper = polygon_mask(candidate, width, height, source['nominal_width_m'],
                                     source['nominal_height_m'], resolution)
                return np.flip(upper, axis=(0, 1)).copy()
    return polygon_mask(vertices, width, height, source['nominal_width_m'],
                        source['nominal_height_m'], resolution)


def polyline_mask(vertices_m, origin, resolution, width, height, thickness_m):
    # 以米为单位画墙和平台边缘；四倍采样避免漏掉窄边。
    samples = 4
    points = []
    for x_m, y_m in vertices_m:
        points.append([round((x_m - origin[0]) / resolution * samples),
                       round((y_m - origin[1]) / resolution * samples)])
    image = np.zeros((height * samples, width * samples), dtype=np.uint8)
    thickness = max(1, math.ceil(thickness_m / resolution * samples))
    cv2.polylines(image, [np.array(points, dtype=np.int32)], False, 1, thickness)
    return image.reshape(height, samples, width, samples).max(axis=(1, 3)).astype(bool)


def inflate_grid(raw, resolution, clearance):
    # 障碍和未知区域均不可通行，外围补零确保空气墙也有净空。
    free = (raw == 0).astype(np.uint8)
    padded = np.pad(free, 1, constant_values=0)
    distance = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    distance = distance[1:-1, 1:-1] * resolution
    # EDT 是格心距离；减半格对角线，保守估计到障碍格边缘的距离。
    distance = np.maximum(0, distance - resolution * math.sqrt(2) / 2)
    blocked = np.full(raw.shape, 100, dtype=np.int8)
    blocked[(raw == 0) & (distance > clearance + 1e-7)] = 0
    return blocked, distance


def terrain_masks(config, width, height, resolution):
    """Share physical slopes and closed ledges between occupancy and annotations.

    A confirmed slope replaces an automatically generated cliff, including the
    cliff's raster thickness at the slope border. Explicit obstacles/pits are
    applied afterwards and can never be erased by this opening.
    """
    source = config["source"]
    def mask(vertices):
        return polygon_mask(vertices, width, height, source["nominal_width_m"],
                            source["nominal_height_m"], resolution)
    platform = np.zeros((height, width), dtype=np.uint8)
    gates = np.zeros_like(platform)
    surfaces = np.zeros_like(platform)
    for region in config["regions"]:
        if region["kind"] == "platform":
            platform[mask(region["vertices_uv"])] = 1
    if not platform.any():
        return platform, gates, surfaces, np.zeros_like(platform, dtype=bool)
    ramps = config["ramp_access"]
    pixels = max(1, math.ceil(ramps["ledge_thickness_m"] / resolution))
    kernel = np.ones((2 * pixels + 1, 2 * pixels + 1), dtype=np.uint8)
    ledge = cv2.dilate(platform, kernel) != cv2.erode(platform, kernel)
    for number, entry in enumerate(ramps.get("confirmed_entries", []), 1):
        gate = mask(entry["vertices_uv"])
        if "surface_vertices_uv" in entry:
            surface = mask(entry["surface_vertices_uv"])
            # Join the finite-width automatic edge to the slope. Do not enlarge
            # the footprint into arbitrary adjacent ground or clear buildings.
            border = (cv2.dilate(surface.astype(np.uint8), kernel) != 0) & ledge
            if (surfaces[surface] != 0).any():
                raise ValueError("Overlapping slope surfaces")
            surfaces[surface] = number
            # The opening may remove a neighbouring raster cliff, but that
            # extra thickness is ordinary ground, never an extended grey slope.
            gate = gate | surface | border
        if (gates[gate] != 0).any():
            raise ValueError("Overlapping ramp approaches")
        gates[gate] = number
    return platform, gates, surfaces, ledge & (gates == 0)


def build_grid(config):
    width, height, resolution = validate_geometry(config)
    source = config["source"]
    grid = config["grid"]
    raw = np.zeros((height, width), dtype=np.int8)
    # 可行走的高地顶面与地面之间仍有陡边；只有明确坡道才允许跨边。
    _, _, _, ledge = terrain_masks(config, width, height, resolution)
    raw[ledge] = 100
    # Confirmed walkable small highlands retain free occupancy. Unlike a
    # platform with vertical ledges, they do not create an automatic cliff ring.
    # 先地形未知层，再实体层；实体不能被后面的地形覆盖成未知。
    for kind, value in [("unknown", -1), ("obstacle", 100)]:
        for region in config["regions"]:
            if region["kind"] == kind:
                mask = region_mask(region, config, width, height, resolution)
                raw[mask] = value
    for wall in config.get("walls", []):
        vertices = []
        for u, v in wall["points_uv"]:
            vertices.append([u * source["nominal_width_m"], (1 - v) * source["nominal_height_m"]])
        mask = polyline_mask(vertices, [0, 0], resolution, width, height, wall["thickness_m"])
        raw[mask] = 100
    for y in range(height):
        for x in range(width):
            left = x * resolution
            bottom = y * resolution
            right = (x + 1) * resolution
            top = (y + 1) * resolution
            if (left < grid["boundary_m"] or bottom < grid["boundary_m"] or
                    right > source["nominal_width_m"] - grid["boundary_m"] + 1e-9 or
                    top > source["nominal_height_m"] - grid["boundary_m"] + 1e-9):
                raw[y, x] = 100
    clearance = grid["robot_radius_m"] + grid["safety_margin_m"]
    blocked, distance = inflate_grid(raw, resolution, clearance)
    # Local road-apron margin only: preserve the full robot radius, occupied
    # cells and unknown terrain. Mirror the raster itself to avoid rounding
    # asymmetry between the two approaches.
    for entry in config.get("navigation_clearance_overrides", []):
        mask = polygon_mask(entry["vertices_uv"], width, height,
                            source["nominal_width_m"], source["nominal_height_m"], resolution)
        if entry.get("mirror_180", False):
            mask = mask | np.flip(mask, axis=(0, 1))
        required = grid["robot_radius_m"] + entry["safety_margin_m"]
        blocked[mask & (raw == 0) & (distance > required + 1e-7)] = 0
    return raw, blocked, distance


def write_pgm(path, cells):
    pixels = np.full(cells.shape, 205, dtype=np.uint8)
    pixels[cells == 0] = 254
    pixels[cells == 100] = 0
    # PGM 从上往下，OccupancyGrid/CSV 从下往上。
    if not cv2.imwrite(str(path), np.flipud(pixels)):
        raise OSError("Cannot write " + str(path))


def write_previews(output, config, raw, blocked, source_path):
    colors = np.full((raw.shape[0], raw.shape[1], 3), 245, dtype=np.uint8)
    colors[raw == -1] = [135, 135, 135]
    colors[raw == 100] = [25, 25, 25]
    inflated = colors.copy()
    inflated[(raw == 0) & (blocked == 100)] = [80, 165, 245]
    panels = []
    for title, pixels in [("RAW OCCUPANCY (DRAFT)", colors), ("BLOCKED + CLEARANCE (DRAFT)", inflated)]:
        panel = np.full((710, 1160, 3), 255, dtype=np.uint8)
        cv2.putText(panel, title, (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 20, 20), 2)
        image = cv2.resize(np.flipud(pixels), (1120, 600), interpolation=cv2.INTER_NEAREST)
        panel[50:650, 20:1140] = image
        # 米制网格；u/v 只是描图坐标，不能拿来发送 Unity 目标。
        width_m = config["source"]["nominal_width_m"]
        height_m = config["source"]["nominal_height_m"]
        for metre in range(0, int(width_m) + 1, 2):
            x = 20 + round(metre / width_m * 1119)
            cv2.line(panel, (x, 50), (x, 649), (180, 180, 180), 1)
            cv2.putText(panel, str(metre), (x, 672), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (40, 40, 40), 1)
        for metre in range(0, int(height_m) + 1, 3):
            y = 649 - round(metre / height_m * 599)
            cv2.line(panel, (20, y), (1139, y), (180, 180, 180), 1)
        cv2.putText(panel, "White: prior ground | Black: structure | Grey: unverified terrain | Orange: clearance",
                    (20, 697), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 40, 40), 1)
        panels.append(panel)
    cv2.imwrite(str(output / "preview.png"), np.vstack(panels))
    source = cv2.imread(str(source_path))
    if source is None:
        return
    for number, region in enumerate(config["regions"]):
        points = []
        for u, v in region["vertices_uv"]:
            points.append([round(u * (source.shape[1] - 1)), round(v * (source.shape[0] - 1))])
        color = (0, 80, 255)
        if region["kind"] == "unknown":
            color = (0, 210, 255)
        contour = np.array(points, dtype=np.int32)
        cv2.polylines(source, [contour], True, color, 2)
        centre = contour.mean(axis=0).astype(int)
        cv2.putText(source, str(number + 1), tuple(centre), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.imwrite(str(output / "source_overlay.png"), source)


def export_grid(config_path, output):
    config = yaml.safe_load(config_path.read_text())
    raw, blocked, distance = build_grid(config)
    output.mkdir(parents=True, exist_ok=True)
    for name, cells in [("occupancy", raw), ("blocked", blocked)]:
        write_pgm(output / (name + ".pgm"), cells)
        metadata = {"image": name + ".pgm", "mode": "trinary", "resolution": config["grid"]["resolution_m"],
                    "origin": [0.0, 0.0, 0.0], "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196}
        (output / (name + ".yaml")).write_text(yaml.safe_dump(metadata, sort_keys=False))
    np.savetxt(output / "occupancy.csv", raw, fmt="%d", delimiter=",")
    np.savetxt(output / "blocked.csv", (blocked != 0).astype(int), fmt="%d", delimiter=",")
    np.savetxt(output / "clearance_m.csv", distance, fmt="%.3f", delimiter=",")
    parameters = {"frame_id": config["grid"]["frame_id"], "resolution_m": config["grid"]["resolution_m"],
                  "width_cells": raw.shape[1], "height_cells": raw.shape[0],
                  "map_directory": output.name, "geometry_verified": False, "registration_verified": False}
    runtime = {"static_grid_publisher": {"ros__parameters": parameters}}
    (output / "runtime.yaml").write_text(yaml.safe_dump(runtime, sort_keys=False))
    summary = {"frame_id": config["grid"]["frame_id"], "row_order": "bottom_to_top", "width_cells": raw.shape[1],
               "height_cells": raw.shape[0], "resolution_m": config["grid"]["resolution_m"],
               "geometry_verified": False, "registration_verified": False,
               "clearance_m": config["grid"]["robot_radius_m"] + config["grid"]["safety_margin_m"],
               "geometry_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
               "source": config["source"], "regions": config["regions"], "landmarks": config.get("landmarks", []),
               "terrain_edges": config.get("terrain_edges", []),
               "unresolved_structures": config.get("unresolved_structures", []),
               "terrain_assumptions": config.get("terrain_assumptions", []),
               "ramp_access": config.get("ramp_access", {}),
               "counts": {"prior_free": int((raw == 0).sum()), "unknown": int((raw == -1).sum()),
                          "occupied": int((raw == 100).sum()), "planning_free": int((blocked == 0).sum())}}
    (output / "manifest.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    source_path = config_path.parent / config["source"]["image"]
    write_previews(output, config, raw, blocked, source_path)
    print(json.dumps(summary["counts"], ensure_ascii=False))
    print("Draft map saved to " + str(output))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    package = Path(__file__).resolve().parent.parent
    parser.add_argument("--geometry", type=Path, default=package / "config/arena_geometry.yaml")
    parser.add_argument("--output", type=Path, default=package / "maps/arena_draft")
    args = parser.parse_args()
    export_grid(args.geometry.resolve(), args.output.resolve())


if __name__ == "__main__":
    main()
