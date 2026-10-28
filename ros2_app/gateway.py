#!/usr/bin/env python3
"""ROS 2 <-> HTTP adapter. No scripted trajectory and no synthetic observations."""
from __future__ import annotations

import base64
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64

JOINTS = ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')
LO = (-2.,-1.4,-1.9,-1.5,-2.3,-.035)
HI = ( 2., 1.4, 1.9, 1.5, 2.3, 0.)
SLEW = (.05,.05,.05,.05,.07,.006)  # rad or metres per accepted action command

class Gateway(Node):
    def __init__(self):
        super().__init__('act_gazebo_gateway')
        self.lock=threading.Lock()
        self.positions={}
        self.image=None
        self.state_time=0.
        self.image_time=0.
        self._cmd_publishers={name:self.create_publisher(Float64, f'/arm/cmd/{name}', 10) for name in JOINTS}
        self.create_subscription(JointState,'/joint_states',self.on_joint,10)
        self.create_subscription(Image,'/camera/front/image_raw',self.on_image,qos_profile_sensor_data)
        self.get_logger().info('Waiting for Gazebo joint states and camera')

    def on_joint(self,msg):
        with self.lock:
            self.positions.update({name:float(pos) for name,pos in zip(msg.name,msg.position)})
            self.state_time=time.monotonic()

    def on_image(self,msg):
        if msg.encoding not in ('rgb8','bgr8'):
            self.get_logger().warn(f'Unsupported image encoding: {msg.encoding}',throttle_duration_sec=10)
            return
        record={'width':msg.width,'height':msg.height,'step':msg.step,
                'encoding':msg.encoding,'data':base64.b64encode(bytes(msg.data)).decode('ascii')}
        with self.lock:
            self.image=record
            self.image_time=time.monotonic()

    def get_snapshot(self):
        with self.lock:
            missing=[name for name in JOINTS if name not in self.positions]
            age=max(time.monotonic()-self.state_time,time.monotonic()-self.image_time)
            if missing or self.image is None or age>2.0:
                return None,{'error':'observation unavailable or stale','missing_joints':missing,
                             'has_image':self.image is not None,'max_age_s':round(age,3)}
            return {'joints':[self.positions[j] for j in JOINTS], 'image':dict(self.image),
                    'joint_names':list(JOINTS)},None

    def publish_action(self,targets):
        if not isinstance(targets,list) or len(targets)!=6:
            raise ValueError('positions must be list[6], in radians (gripper metres)')
        if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in targets):
            raise ValueError('positions must contain finite numbers')
        with self.lock:
            if any(j not in self.positions for j in JOINTS):
                raise ValueError('joint feedback not ready')
            current=[self.positions[j] for j in JOINTS]
        applied = []
        clipped_targets = []
        hit_joint_limit = []
        hit_slew_limit = []
        for v, x, lo, hi, step, name in zip(targets, current, LO, HI, SLEW, JOINTS):
            clipped = min(hi, max(lo, float(v)))
            limited = max(x - step, min(x + step, clipped))
            msg = Float64()
            msg.data = float(limited)
            self._cmd_publishers[name].publish(msg)
            clipped_targets.append(clipped)
            hit_joint_limit.append(abs(clipped - float(v)) > 1e-8)
            hit_slew_limit.append(abs(limited - clipped) > 1e-8)
            applied.append(limited)
        # Diagnostic fields only: do not modify control behavior.
        return {
            'applied': applied,
            'joint_names': list(JOINTS),
            'requested': [float(v) for v in targets],
            'measured_before': current,
            'clipped': clipped_targets,
            'joint_limit_hit': hit_joint_limit,
            'slew_limit_hit': hit_slew_limit,
        }


def main():
    rclpy.init()
    node=Gateway()
    executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
    th=threading.Thread(target=executor.spin,daemon=True);th.start()

    class Handler(BaseHTTPRequestHandler):
        def send_json(self,code,payload):
            body=json.dumps(payload).encode()
            self.send_response(code)
            self.send_header('Content-Type','application/json')
            self.send_header('Content-Length',str(len(body)))
            self.end_headers();self.wfile.write(body)
        def do_GET(self):
            if self.path=='/health':
                with node.lock:
                    self.send_json(200,{'ok':True,'received_joint_states':bool(node.positions),'received_image':node.image is not None})
                return
            if self.path!='/observation':
                self.send_json(404,{'error':'not found'});return
            obs,err=node.get_snapshot()
            self.send_json(200 if err is None else 503,obs if err is None else err)
        def do_POST(self):
            if self.path!='/action':
                self.send_json(404,{'error':'not found'});return
            try:
                n=int(self.headers.get('Content-Length','0'))
                if n<=0 or n>16384:raise ValueError('bad body size')
                payload=json.loads(self.rfile.read(n))
                self.send_json(200,node.publish_action(payload['positions']))
            except (ValueError,KeyError,TypeError,json.JSONDecodeError) as e:
                self.send_json(400,{'error':str(e)})
        def log_message(self, fmt,*args):
            if self.path!='/observation':super().log_message(fmt,*args)

    host=os.environ.get('ROBOT_API_HOST','127.0.0.1')
    port=int(os.environ.get('ROBOT_API_PORT','8765'))
    server=ThreadingHTTPServer((host,port),Handler)
    node.get_logger().info(f'HTTP gateway on http://{host}:{port}')
    try:server.serve_forever()
    finally:
        server.server_close();executor.shutdown();node.destroy_node();rclpy.shutdown()

if __name__=='__main__': main()
