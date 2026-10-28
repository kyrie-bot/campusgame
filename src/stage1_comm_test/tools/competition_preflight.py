#!/usr/bin/env python3
"""Read-only startup check. Never kills nodes or publishes game commands."""
import sys
import time
import rclpy


def main():
    player = next((int(a.split(':=', 1)[1]) for a in sys.argv[1:] if a.startswith('player_id:=')), 1)
    if player not in (1, 2):
        raise SystemExit('player_id 必须为 1 或 2')
    rclpy.init(args=[])
    node = rclpy.create_node('competition_startup_check')
    try:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        conflicts = []
        for topic in (f'/target_speed_player_{player}', f'/target_angles_player_{player}',
                      '/aim/selection', '/combat/aim_candidate'):
            owners = [p.node_name for p in node.get_publishers_info_by_topic(topic)]
            if owners:
                conflicts.append(f'{topic}: {", ".join(owners)}')
        if conflicts:
            print('检测到已运行的底盘/自瞄控制节点，无法再启动一套：', file=sys.stderr)
            print('\n'.join(conflicts), file=sys.stderr)
            print('请在旧自瞄和当前自动导航终端按 Ctrl+C，然后只启动 run_competition_auto.sh。', file=sys.stderr)
            return 2
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
