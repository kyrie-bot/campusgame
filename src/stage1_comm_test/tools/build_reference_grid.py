#!/usr/bin/env python3
"""Generate a rule-sized vector terrain reference; never infer metric geometry from pixels."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import yaml
from build_arena_grid import build_grid, terrain_masks, polygon_mask, write_pgm, validate_geometry

GATE_NAMES = ['upper_left_direct', 'upper_right_turn', 'lower_left_turn', 'lower_right_direct']


def surface_triangles(cells, resolution):
    triangles = []
    for y, row in enumerate(cells != 0):
        edges = np.diff(np.pad(row.astype(int), (1, 1)))
        for left, right in zip(np.where(edges == 1)[0], np.where(edges == -1)[0]):
            x0, x1 = left * resolution, right * resolution
            y0, y1 = y * resolution, (y + 1) * resolution
            triangles.extend([[x0, y0], [x1, y0], [x1, y1],
                              [x0, y0], [x1, y1], [x0, y1]])
    return triangles


def build_reference(config_path, output):
    config = yaml.safe_load(config_path.read_text())
    geometry_path = (config_path.parent / config['geometry_file']).resolve()
    geometry = yaml.safe_load(geometry_path.read_text())
    width, height, resolution = validate_geometry(geometry)
    source = geometry['source']
    for field in ['nominal_width_m', 'nominal_height_m']:
        if config[field] != source[field]:
            raise ValueError('Reference dimensions disagree with rule authority')
    for field in ['resolution_m', 'robot_radius_m', 'safety_margin_m']:
        if config[field] != geometry['grid'][field]:
            raise ValueError('Reference grid configuration mismatch: ' + field)
    if config['frame_id'] != 'arena_reference':
        raise ValueError('Unregistered reference needs arena_reference frame')
    if (source['scene_units_per_m'] != 3.0 or source['image_numbers_used_as_measurements'] is not False):
        raise ValueError('Campus rule units are required; image measurements are not allowed')
    entries = geometry['ramp_access']['confirmed_entries']
    if [entry['name'] for entry in entries] != GATE_NAMES:
        raise ValueError('Only the four user-confirmed approaches may cross highland edges')
    raw, blocked, distance = build_grid(geometry)
    for entry in entries:
        if entry.get('surface_vertices_uv') is None:
            raise ValueError('Ramp needs a separate physical surface footprint')
    platform, gates, surfaces, ledge = terrain_masks(geometry, width, height, resolution)
    # Grey is a terrain overlay, not occupancy=unknown: normal slopes are free.
    ramp_surface = np.where(raw == 0, surfaces, 0).astype(np.uint8)
    highland_surface = np.zeros(raw.shape, dtype=np.uint8)
    for number, region in enumerate([r for r in geometry['regions'] if r['kind'] == 'walkable_highland'], 1):
        mask = polygon_mask(region['vertices_uv'], width, height,
                            source['nominal_width_m'], source['nominal_height_m'], resolution)
        highland_surface[mask & (raw == 0)] = number
    output.mkdir(parents=True, exist_ok=True)
    for name, cells in [('occupancy', raw), ('blocked', blocked)]:
        write_pgm(output / (name + '.pgm'), cells)
        metadata = {'image': name + '.pgm', 'mode': 'trinary', 'resolution': resolution,
                    'origin': [0., 0., 0.], 'negate': 0, 'occupied_thresh': .65, 'free_thresh': .196}
        (output / (name + '.yaml')).write_text(yaml.safe_dump(metadata, sort_keys=False))
    for name, cells, fmt in [('occupancy', raw, '%d'), ('blocked', blocked != 0, '%d'),
                             ('clearance_m', distance, '%.3f'), ('highland_top', platform, '%d'),
                             ('ramp_gates', gates, '%d'), ('ramp_surfaces', ramp_surface, '%d'),
                             ('walkable_highlands', highland_surface, '%d'),
                             ('closed_ledge', ledge, '%d')]:
        np.savetxt(output / (name + '.csv'), cells, fmt=fmt, delimiter=',')
    parameters = {'frame_id': 'arena_reference', 'width_cells': width, 'height_cells': height,
                  'resolution_m': resolution, 'origin_x_m': 0., 'origin_z_m': 0.,
                  'map_directory': output.name, 'publish_standard_map': False,
                  'publish_terrain_annotations': True, 'geometry_verified': False, 'registration_verified': False}
    (output / 'runtime.yaml').write_text(yaml.safe_dump({'static_grid_publisher': {'ros__parameters': parameters}}, sort_keys=False))
    summary = {'frame_id': 'arena_reference', 'width_cells': width, 'height_cells': height,
               'resolution_m': resolution, 'nominal_size_m': [source['nominal_width_m'], source['nominal_height_m']],
               'origin_xy_m': [0., 0.], 'row_order': 'bottom_to_top',
               'geometry_verified': False, 'registration_verified': False,
               'source': source, 'sampling': 'explicit vector contours; no bitmap sampling',
               'config_sha256': hashlib.sha256(config_path.read_bytes()).hexdigest(),
               'geometry_sha256': hashlib.sha256(geometry_path.read_bytes()).hexdigest(),
               'pit_rasterization': 'exact_vector_pairs_use_180_degree_cell_mirror_v1',
               'clearance_m': config['robot_radius_m'] + config['safety_margin_m'],
               'navigation_clearance_overrides': geometry.get('navigation_clearance_overrides', []),
               'occupied_cells': int((raw == 100).sum()), 'unknown_cells': int((raw == -1).sum()),
               'ramp_access': geometry['ramp_access'], 'regions': geometry['regions'],
               'walls': geometry.get('walls', []),
               'ramp_surface_triangles_xy_m': surface_triangles(ramp_surface, resolution),
               'walkable_highland_triangles_xy_m': surface_triangles(highland_surface, resolution),
               'unresolved_structures': geometry['unresolved_structures'],
               'note': 'Campus rule dimensions; unsurveyed interior contours. No fixed screenshot-to-world mapping.'}
    (output / 'manifest.json').write_text(json.dumps(summary, indent=2) + '\n')
    # Show the whole field, its walkable top, access slopes, and blocked fly/tunnel areas.
    colors = np.full((*raw.shape, 3), 250, dtype=np.uint8)
    colors[platform != 0] = [225, 236, 240]
    colors[raw == -1] = [145, 145, 145]
    colors[raw == 100] = [20, 20, 20]
    colors[ramp_surface != 0] = [180, 180, 180]
    colors[highland_surface != 0] = [180, 180, 180]
    view = cv2.resize(np.flipud(colors), (1120, 600), interpolation=cv2.INTER_NEAREST)
    panel = np.full((700, 1160, 3), 255, dtype=np.uint8)
    panel[45:645, 20:1140] = view
    cv2.putText(panel, '2027 RULE SIZE: 28 x 15 m | 3 scene units/m | UNSURVEYED CONTOURS',
                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 0), 1)
    cv2.putText(panel, 'Black: obstacles | Light grey: walkable slopes / small highlands | Dark grey: unverified terrain',
                (20, 678), cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 0, 0), 1)
    cv2.imwrite(str(output / 'preview.png'), panel)
    cv2.imwrite(str(output / 'outline.png'), np.flipud(np.where(raw == 0, 254, np.where(raw == -1, 205, 0)).astype(np.uint8)))
    # Keep the actual layout source beside the product and make contour review
    # possible. Pixel coordinates here are only for drawing this diagnostic.
    if source.get('image'):
        source_path = (geometry_path.parent / source['image']).resolve()
        original = cv2.imread(str(source_path))
        if original is None:
            raise ValueError('Missing tactical layout source: ' + str(source_path))
        cv2.imwrite(str(output / 'reference_source.png'), original)
        overlay = original.copy()
        h, w = overlay.shape[:2]
        def points(vertices):
            return np.array([[round(u*(w-1)), round(v*(h-1))] for u,v in vertices], np.int32)
        for region in geometry['regions']:
            color = (0, 230, 230) if region['kind'] == 'platform' else (0, 150, 255)
            cv2.polylines(overlay, [points(region['vertices_uv'])], True, color, 1)
        for entry in entries:
            cv2.polylines(overlay, [points(entry['surface_vertices_uv'])], True, (80, 255, 80), 2)
        for wall in geometry.get('walls', []):
            cv2.polylines(overlay, [points(wall['points_uv'])], False, (40, 40, 255), 2)
        cv2.imwrite(str(output / 'source_geometry_overlay.png'), overlay)
    return summary


def main():
    package = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=package / 'config/reference_map.yaml')
    parser.add_argument('--output', type=Path, default=package / 'maps/arena_reference')
    args = parser.parse_args()
    summary = build_reference(args.config, args.output)
    print(json.dumps({key: summary[key] for key in ['frame_id', 'nominal_size_m', 'occupied_cells', 'unknown_cells', 'geometry_sha256']}, indent=2))


if __name__ == '__main__':
    main()
