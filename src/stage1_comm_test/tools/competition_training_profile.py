#!/usr/bin/env python3
"""Validate the fixed public-landmark projection for explicit Unity training."""
import hashlib
import json
from pathlib import Path
import numpy as np
import yaml


def training_parameters(package, cruise_speed=4.2):
    package=Path(package)
    if not np.isfinite(cruise_speed) or not .2 <= cruise_speed <= 4.5:
        raise ValueError('cruise_speed_mps must be between 0.2 and 4.5')
    report=json.loads((package/'maps/arena_reference/landmark_alignment.json').read_text())
    manifest=json.loads((package/'maps/arena_reference/manifest.json').read_text())
    geometry_hash=hashlib.sha256((package/'config/reference_terrain.yaml').read_bytes()).hexdigest()
    if geometry_hash!=manifest['geometry_sha256'] or geometry_hash!=report['geometry_sha256']:
        raise ValueError('Map changed: regenerate grid and recheck public-landmark alignment before training')
    if (report.get('fit_valid') is not True or report.get('scale_fitted') is not False or
        not 0<=report['max_error_m']<=.15 or not report.get('holdout_labels') or
        report.get('geometry_verified') is not False):
        raise ValueError('Unity training needs a valid rigid landmark report with independent checks')
    planner=yaml.safe_load((package/'config/arena_navigation.yaml').read_text())['dijkstra_planner']['ros__parameters']
    follower=yaml.safe_load((package/'config/arena_following.yaml').read_text())['path_follower']['ros__parameters']
    matrix=np.array(report['unity_to_reference'],dtype=float).reshape(2,3)
    if (not np.isfinite(matrix).all() or
        not np.allclose(matrix[:,:2].T@matrix[:,:2],np.eye(2),atol=1e-9,rtol=0) or
        abs(np.linalg.det(matrix[:,:2])-1)>1e-9 or
        planner['unity_to_reference']!=report['unity_to_reference'] or
        follower['unity_to_reference']!=report['unity_to_reference'] or
        planner['scene_units_per_m']!=3. or follower['scene_units_per_m']!=3.):
        raise ValueError('Planner, follower, and checked metre-scale projection must agree')
    follower.update(simulation=False,follow_planner_projection=False,map_alignment_verified=False,
        max_speed_mps=float(cruise_speed),max_acceleration_mps2=8.5, braking_acceleration_mps2=1.2, max_lateral_acceleration_mps2=2.8,lookahead_m=2.0,max_path_deviation_m=2.0,
        tracking_reserve_m=.03, collision_horizon_s=.4,
        **{'terrain.max_speed_mps':float(cruise_speed), 'terrain.lookahead_m':1.6},
        position_tolerance_m=.15,release_tolerance_m=.25, **{'command.enabled':True,
        'command.training_prior':True,'command.landmark_alignment_checked':True})
    # The captured game stream is commonly 110–220 ms old. Account for that
    # age in every rollout; retain a bounded 280 ms transport watchdog (220 ms plus two 20 ms cycles
    # and 20 ms scheduling reserve).
    if follower['state_timeout_s'] > .28:
        raise ValueError('Training feedback watchdog exceeds 280 ms')
    follower['state_timeout_s']=.28
    follower['enemy_recovery_margin_m']=float(planner['dynamic_margin_m'])
    surfaces=np.loadtxt(package/'maps/arena_reference/ramp_surfaces.csv',delimiter=',',dtype=int)
    follower['terrain.ramp_cells']=[int(i) for i in np.flatnonzero(surfaces.ravel())]
    if (follower['braking_acceleration_mps2']<.7 or follower['state_timeout_s']>.28 or
        follower['plan_timeout_s']>.45 or follower['enemy_timeout_s']>2.5):
        raise ValueError('Training must retain braking and feedback watchdogs')
    # Only the rune firing station is ground-only. Transit still uses the
    # original Dijkstra map and all robot combat remains unrestricted.
    tops=np.loadtxt(package/'maps/arena_reference/highland_top.csv',delimiter=',',dtype=int)
    side_tops=np.loadtxt(package/'maps/arena_reference/walkable_highlands.csv',delimiter=',',dtype=int)
    planner['mission.buff_ground_excluded_cells']=[int(i) for i in np.flatnonzero(
        ((tops!=0)|(side_tops!=0)|(surfaces!=0)).ravel())]
    low_ledges=np.loadtxt(package/'maps/arena_reference/closed_ledge.csv',delimiter=',',dtype=int)
    planner['mission.buff_low_ledge_cells']=[int(i) for i in np.flatnonzero(low_ledges.ravel())]
    # Stable firing surface for robot 4 and the following sentry only.
    # Exclude slopes and erode 30 cm so the chassis stays on the deck.
    top=((tops!=0)|(side_tops!=0)) & (surfaces==0)
    padded=np.pad(top,3,constant_values=False);inside=top.copy()
    for dy in range(-3,4):
        for dx in range(-3,4):
            if dx*dx+dy*dy<=9:
                inside &= padded[3+dy:3+dy+top.shape[0],3+dx:3+dx+top.shape[1]]
    cells=[int(i) for i in np.flatnonzero(inside.ravel())]
    planner['mission.robot_highland_cells']=cells
    planner['mission.four_station_enabled']=True
    planner['mission.four_station_xy_m']=[11.15,1.45]
    planner['mission.field_robot_order']=True
    follower['mission.field_robot_order']=True
    follower['combat.highland_cells']=cells
    planner['mission.opening_ramp_enabled']=True
    planner['mission.opening_ramp_cells']=[int(i) for i in np.flatnonzero(np.isin(surfaces,(1,4)).ravel())]
    planner['mission.rune_transit_ramp_cells']=[int(i) for i in np.flatnonzero(surfaces.ravel())]
    planner['mission.sentry_last']=True
    planner['mission.sentry_anchor_enabled']=True
    planner['mission.sentry_anchor_radius_m']=2.
    planner['allow_live_alignment']=False
    planner['display_only']=False
    # Assemble the complete planner/follower contract here. Launch and model
    # tests must use the same stopping-time forecast, including at high speed.
    forecast=(follower['collision_horizon_s']+follower['drive_response_time_s']+
        follower['max_speed_mps']/follower['braking_acceleration_mps2']+1.+follower['plan_timeout_s'])
    if not np.isfinite(forecast) or not 0 <= forecast <= 10:
        raise ValueError('enemy_forecast_s must be finite and in [0, 10]')
    planner['enemy_forecast_s']=float(forecast)
    return planner,follower
