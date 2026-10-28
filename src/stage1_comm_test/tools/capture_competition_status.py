#!/usr/bin/env python3
"""Read-only snapshot of navigation and aiming; never emits game controls."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from std_msgs.msg import String
from tdt_interface.msg import AutoAimState,AutoAimFrame
from campus_aim_interfaces.msg import TrackFrame,NavigationPlan,AimSolution
from geometry_msgs.msg import Vector3Stamped

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--seconds',type=float,default=5.)
    p.add_argument('--output',type=Path);args=p.parse_args()
    rclpy.init();node=Node('read_only_competition_status');data={};counts={};ages={};states={}
    def record(name,value):
        data[name]=value;counts[name]=counts.get(name,0)+1
        if isinstance(value,dict) and 'age_s' in value:ages.setdefault(name,[]).append(value['age_s'])
        if isinstance(value,str):
            key=value.split(' goal=')[0] if name=='/navigation/follow_status' else value.split(' target=')[0]
            states.setdefault(name,{})[key]=states.setdefault(name,{}).get(key,0)+1
    def stamp_age(m):return node.get_clock().now().nanoseconds*1e-9-m.header.stamp.sec-m.header.stamp.nanosec*1e-9
    cached=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic in ('/mission/status','/planning/status','/navigation/follow_status','/combat/status','/aim/fire_state','/tracker/state'):
        node.create_subscription(String,topic,lambda m,t=topic:record(t,m.data),cached if topic in ('/mission/status','/planning/status','/navigation/follow_status') else 1)
    node.create_subscription(String,'/navigation/enemies',lambda m:record('/navigation/enemies',json.loads(m.data)),1)
    def state(m):
        record('/autoaim/state',dict(position=[m.position.x,m.position.y,m.position.z],velocity=[m.twist.linear.x,m.twist.linear.y,m.twist.linear.z],
            health=m.health,ammo=m.ammo_remaining,capacity=m.ammo_capacity,weakened=m.is_weakened,yaw=m.yaw_degrees,pitch=m.pitch_degrees,
            enemy_healths=list(m.enemy_healths),age_s=node.get_clock().now().nanoseconds/1e9-m.header.stamp.sec-m.header.stamp.nanosec/1e9))
    node.create_subscription(AutoAimState,'/autoaim/state',state,1)
    node.create_subscription(AutoAimFrame,'/autoaim/frame',lambda m:record('/autoaim/frame',dict(
        age_s=node.get_clock().now().nanoseconds/1e9-m.header.stamp.sec-m.header.stamp.nanosec/1e9,
        compressed_bytes=len(m.compressed_image.data))),1)
    node.create_subscription(TrackFrame,'/tracker/targets',lambda m:record('/tracker/targets',[
        dict(id=t.track_id,number=t.number,state=t.state,center=[t.center.x,t.center.y,t.center.z],
             variance=t.position_variance,age=t.last_observation_age) for t in m.targets]),1)
    node.create_subscription(AimSolution,'/aim/solution',lambda m:record('/aim/solution',dict(
        track_id=m.track_id,number=m.number,slot=m.slot,yaw=m.yaw,pitch=m.pitch,valid=m.valid,
        fire=m.fire,reason=m.reason,age_s=stamp_age(m))),1)
    node.create_subscription(NavigationPlan,'/planning/snapshot',lambda m:record('/planning/snapshot',dict(
        goal=m.goal_id,valid=m.valid,status=m.status,enemy_count=len(m.enemy_centres),poses=len(m.path.poses),
        enemy_radii=list(m.enemy_radii))),cached)
    for topic in ('/aim/framed_command','/aim/command_angles','/aim/selection'):
        node.create_subscription(Vector3Stamped,topic,lambda m,t=topic:record(t,dict(
            stamp_ns=m.header.stamp.sec*1000000000+m.header.stamp.nanosec,age_s=stamp_age(m),
            values=[m.vector.x,m.vector.y,m.vector.z])),1)
    until=time.monotonic()+args.seconds
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
    result=dict(samples=counts,latest=data,state_counts=states,
        age_statistics={topic:dict(min_s=min(values),max_s=max(values),mean_s=sum(values)/len(values)) for topic,values in ages.items()},
        angle_publishers={str(player):[n.node_name for n in node.get_publishers_info_by_topic('/target_angles_player_'+str(player))] for player in (1,2)},nodes=node.get_node_names())
    text=json.dumps(result,indent=2,ensure_ascii=False)
    if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(text+'\n')
    print(text);node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
