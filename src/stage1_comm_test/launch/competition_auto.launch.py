"""Ordered robot/outpost/base mission with fixed birth-point refill."""
from pathlib import Path
import sys
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def nodes(context):
    package=Path(get_package_share_directory('stage1_comm_test'))
    sys.path.insert(0,str(Path(get_package_prefix('stage1_comm_test'))/'lib/stage1_comm_test'))
    from competition_training_profile import training_parameters
    from autoaim_profile import autoaim_parameters
    planner,follower=training_parameters(package,float(LaunchConfiguration('cruise_speed_mps').perform(context)))
    combat=LaunchConfiguration('combat').perform(context).lower()=='true'
    autonomous=LaunchConfiguration('autonomous').perform(context).lower()=='true'
    planner['mission.enabled']=autonomous
    buff=autonomous and combat and LaunchConfiguration('buff').perform(context).lower()=='true'
    planner['mission.buff_enabled']=buff
    planner['mission.ignore_enemy_one']=LaunchConfiguration('ignore_enemy_one').perform(context).lower()=='true'
    planner['mission.outpost_min_range_m']=float(LaunchConfiguration('outpost_min_range_m').perform(context))
    planner['mission.outpost_preferred_range_m']=float(LaunchConfiguration('outpost_preferred_range_m').perform(context))
    planner['mission.outpost_max_range_m']=float(LaunchConfiguration('outpost_max_range_m').perform(context))
    for key in ('base_min_range_m','base_preferred_range_m','base_max_range_m'):
        planner['mission.'+key]=float(LaunchConfiguration(key).perform(context))
    strict_order=autonomous and bool(planner.get('mission.strict_order',False))
    follower['mission.strict_order']=strict_order
    follower['combat.enabled']=combat
    after_arrival=LaunchConfiguration('engage_after_arrival').perform(context).lower()=='true'
    follower['combat.after_arrival']=after_arrival
    follower['combat.structures']=autonomous
    follower['combat.acquire_unknown']=autonomous
    follower['combat.route_decision']=LaunchConfiguration('route_engagement_only').perform(context).lower()=='true'
    player=int(LaunchConfiguration('player_id').perform(context))
    if player not in (1,2):raise ValueError('player_id must be 1 (blue) or 2 (red)')
    planner['player_id']=follower['player_id']=player
    planner_node=Node(package='stage1_comm_test',executable='dijkstra_planner',output='screen',parameters=[planner],
        remappings=[('/autoaim/state',LaunchConfiguration('state_topic')),
                    ('/navigation/enemies',LaunchConfiguration('enemy_topic'))])
    result = [
        RegisterEventHandler(OnProcessExit(target_action=planner_node,on_exit=[
            LogInfo(msg='Dijkstra planner exited: stopping competition controllers; check the preceding planner error.'),
            EmitEvent(event=Shutdown(reason='Required Dijkstra planner exited'))])),
        Node(package='stage1_comm_test',executable='static_grid_publisher',output='screen',
            parameters=[str(package/'maps/arena_reference/runtime.yaml')],
            remappings=[('/arena/map','/reference/map'),('/arena/blocked_map','/reference/blocked_map'),
                ('/arena/terrain_markers','/planning/terrain_markers'),('/arena/map_status','/reference/map_status')]),
        Node(package='tf2_ros',executable='static_transform_publisher',output='screen',
            arguments=['--frame-id','arena_reference','--child-frame-id','arena_view']),
        planner_node,
        Node(package='stage1_comm_test',executable='path_follower',output='screen',parameters=[follower],
            remappings=[('/autoaim/state',LaunchConfiguration('state_topic'))]),
        Node(package='rviz2',executable='rviz2',output='screen',arguments=['-d',str(package/'config/arena_navigation.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')))]
    # Only the arbiter owns the game angle output. Ballistics remains the sole
    # source of combat angles and fire permission; scan commands never fire.
    if combat:
        config=autoaim_parameters(LaunchConfiguration('params_file').perform(context), player,
            LaunchConfiguration('aim_display').perform(context).lower()=='true',
            coordinated=True, strict_order=strict_order)
        config['armor_detector']['display.mask'] = LaunchConfiguration('mask_display').perform(context).lower()=='true'
        for executable,name in [('armor_detector','armor_detector'),('tracker','armor_tracker'),('aim_solver','aim_solver')]:
            settings=config[name]
            remaps=[]
            result.append(Node(package='stage1_comm_test',executable=executable,name=name,parameters=[settings],remappings=remaps,output='screen'))
        if buff:
            result.append(Node(package='stage1_comm_test',executable='buff_controller',output='screen',
                parameters=[{'player_id':player,'display':LaunchConfiguration('buff_display').perform(context).lower()=='true'}],
                remappings=[('/autoaim/state',LaunchConfiguration('state_topic'))]))
        result.append(Node(package='stage1_comm_test',executable='combat_gimbal',output='screen',
            parameters=[{'player_id':player,'scan.enabled':LaunchConfiguration('scan').perform(context).lower()=='true',
                'search.pitch_enabled':LaunchConfiguration('pitch_search').perform(context).lower()=='true',
                'command.fire_burst_hold_ms':float(LaunchConfiguration('fire_burst_hold_ms').perform(context)),
                'combat.after_arrival':after_arrival,
                'mission.strict_order':strict_order,
                'mission.field_robot_order':strict_order,
                'outpost.target_timeout_ms':float(config['aim_solver'].get('outpost.target_timeout_ms',280.)),
                'outpost.fire_coast_ms':float(config['aim_solver'].get('outpost.fire_coast_ms',100.)),
                'command.solution_timeout_s':float(config['aim_solver'].get('target_timeout_ms',200.))/1000.,
                'command.mobile_fire_coast_ms':float(config['aim_solver'].get('command.mobile_fire_coast_ms',250.)),
                'command.mobile_feedback_compensation':bool(config['aim_solver'].get('command.mobile_feedback_compensation',False)),
                'command.mobile_feedback_delay_s':float(config['aim_solver'].get('prediction_time_s',.04)),
                'unity_to_reference':planner['unity_to_reference']}],remappings=[('/autoaim/state',LaunchConfiguration('state_topic'))]))
    return result


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file',default_value=str(Path(get_package_share_directory('stage1_comm_test'))/'config/autoaim.yaml')),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('player_id',default_value='1',description='Unity player: 1 blue, 2 red; selects both chassis and gimbal output topics.'),
        DeclareLaunchArgument('cruise_speed_mps',default_value='4.2',description='Speed ceiling in metres/second, 0.2..4.5; ramps use the same ceiling, with checked turns and braking.'),
        DeclareLaunchArgument('autonomous',default_value='true',description='Choose attacks with Dijkstra and return to the fixed birth-point for refill; no clicked goal required.'),
        DeclareLaunchArgument('ignore_enemy_one',default_value='true',description='Skip the user-reported faulty enemy 1 without rewriting its health.'),
        DeclareLaunchArgument('outpost_min_range_m',default_value='2.8'),
        DeclareLaunchArgument('outpost_preferred_range_m',default_value='3.5'),
        DeclareLaunchArgument('outpost_max_range_m',default_value='4.2'),
        DeclareLaunchArgument('base_min_range_m',default_value='1.8'),
        DeclareLaunchArgument('base_preferred_range_m',default_value='2.5'),
        DeclareLaunchArgument('base_max_range_m',default_value='3.2'),
        DeclareLaunchArgument('scan',default_value='false',description='Normal horizontal scan; false follows travel heading. A blocked refill has separate stationary defensive search.'),
        DeclareLaunchArgument('pitch_search',default_value='true',description='Vertical search until real ballistic target acquisition; negative Unity pitch points upward.'),
        DeclareLaunchArgument('fire_burst_hold_ms',default_value='120',description='0..150 ms: continue an already aligned robot burst through brief missing observations; 0 disables it. No initial blind fire.'),
        DeclareLaunchArgument('route_engagement_only',default_value='false'),
        DeclareLaunchArgument('buff',default_value='true',description='Own rune from ground level on departure, then right-bottom ramp and robot cleanup; bounded timeouts preserve the assault mission.'),
        DeclareLaunchArgument('buff_display',default_value='false'),
        DeclareLaunchArgument('combat',default_value='true',description='Run the same C++ detector/tracker/solver as run_autoaim plus the sole gimbal arbiter.'),
        DeclareLaunchArgument('engage_after_arrival',default_value='false',description='False permits en-route robot engagement and blocked stationary defense; outpost stand-off arrival remains mandatory.'),
        DeclareLaunchArgument('aim_display',default_value='true'),
        DeclareLaunchArgument('mask_display',default_value='false',description='Show the armor detector binary mask independently of aim_display (combat must be enabled).'),
        DeclareLaunchArgument('state_topic',default_value='/autoaim/state'),
        DeclareLaunchArgument('enemy_topic',default_value='/navigation/enemies'),
        OpaqueFunction(function=nodes)])
