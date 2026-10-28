"""Launch configuration only; both modes run the same C++ self-aim pipeline."""
from pathlib import Path
import yaml


def autoaim_parameters(path, player=1, display=True, *, coordinated=False, strict_order=False):
    if player not in (1, 2):
        raise ValueError('player must be 1 or 2')
    config = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    result = {}
    for name in ('armor_detector', 'armor_tracker', 'aim_solver'):
        settings = dict(config.get('/**', {}).get('ros__parameters', {}))
        settings.update(config.get(name, {}).get('ros__parameters', {}))
        settings['display'] = display and settings.get('display', True)
        result[name] = settings
    result['armor_detector']['detect_color'] = 0 if player == 1 else 1
    result['aim_solver']['player'] = player
    if coordinated:
        # Preserve the single-body lock and number/plate fire checks. The
        # coordinated owner also needs tentative targets to interrupt search.
        result['armor_tracker']['mission.strict_order'] = strict_order
        result['armor_tracker']['mission.field_robot_order'] = strict_order
        # Stop the search on a genuine, identified tentative observation.
        # Confirmation and all firing gates stay in the native C++ solver.
        result['armor_tracker']['target.acquire_detecting'] = True
        result['aim_solver']['selection.allow_detecting_aim'] = True
        result['aim_solver']['command.output_topic'] = '/combat/aim_candidate'
    return result
