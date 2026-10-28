#!/usr/bin/env python3
"""ACT checkpoint -> ROS 2 HTTP gateway. Inference only; never trains.

Default is observational dry-run. Enabling control requires an explicitly confirmed
calibration mapping because the SO-101 pretrained policy's action units cannot be
inferred from shape=(6,) alone.
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import numpy as np
import torch
from PIL import Image

from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors

DEFAULT_MODEL='shraavb/act_pick_place_cube'
BASE=os.getenv('ROBOT_API_URL','http://127.0.0.1:8765').rstrip('/')


def request_json(path, data=None):
    raw=None if data is None else json.dumps(data).encode('utf-8')
    req=Request(BASE+path,raw,headers={'Content-Type':'application/json'})
    try:
        with urlopen(req,timeout=5.0) as response:
            return json.load(response)
    except HTTPError as exc:
        raise RuntimeError(f'HTTP {exc.code} on {path}: {exc.read().decode()[:500]}') from exc
    except URLError as exc:
        raise RuntimeError(f'ROS 2 gateway unreachable: {BASE}: {exc}') from exc


def decode_image(msg, height, width):
    b=base64.b64decode(msg['data'])
    if msg['encoding'] not in ('rgb8','bgr8'):
        raise RuntimeError(f'Unsupported camera encoding: {msg["encoding"]}')
    # Respect ROS sensor_msgs/Image.step (bytes per row).
    row=np.frombuffer(b,dtype=np.uint8).reshape(msg['height'],msg['step'])
    pixels=row[:,:msg['width']*3].reshape(msg['height'],msg['width'],3)
    if msg['encoding']=='bgr8':pixels=pixels[:,:,::-1]
    pil=Image.fromarray(pixels,'RGB')
    if pil.size!=(width,height):pil=pil.resize((width,height),Image.Resampling.BILINEAR)
    return torch.from_numpy(np.asarray(pil).copy()).permute(2,0,1).float().div(255).unsqueeze(0)


def affine(values,scale,bias):
    if not (len(values)==len(scale)==len(bias)==6):
        raise ValueError('Expected six input, scale, and bias values')
    return [float(v)*float(s)+float(b) for v,s,b in zip(values,scale,bias)]


def load_calibration(path):
    if path is None: return None
    data=json.loads(Path(path).read_text())
    for key in ('state_to_policy','action_to_sim'):
        cfg=data[key]
        if len(cfg['scale'])!=6 or len(cfg['bias'])!=6:
            raise ValueError(f'Invalid {key}: 6D scale/bias required')
    return data


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--model',default=DEFAULT_MODEL)
    p.add_argument('--inspect',action='store_true',help='Only load model and print I/O spec')
    p.add_argument('--execute',action='store_true',help='Publish ACT actions into Gazebo via ROS2')
    p.add_argument('--debug-control',action='store_true',help='Show pre/post Gateway control values every action chunk')
    p.add_argument('--calibration',type=Path,help='Validated policy<->joint unit mapping (required for --execute)')
    p.add_argument('--steps',type=int,default=120)
    p.add_argument('--hz',type=float,default=10.0)
    p.add_argument('--replan-steps',type=int,default=10)
    args=p.parse_args()
    if args.steps<=0 or args.hz<=0 or not 1<=args.replan_steps<=100:
        p.error('steps/hz must be positive; replan-steps must be 1..100')
    mapping=load_calibration(args.calibration)
    if args.execute and (mapping is None or mapping.get('calibration_confirmed') is not True):
        p.error('--execute requires --calibration with calibration_confirmed=true after real validation')
    if mapping is None:
        print('NOTE: No calibration given: diagnostic policy inference uses Gazebo joint values as-is,')
        print('      which may be outside the checkpoint training distribution.')

    device='cuda' if torch.cuda.is_available() else 'cpu'
    print(f'Loading {args.model} on {device}',flush=True)
    policy=ACTPolicy.from_pretrained(args.model).to(device).eval()
    config=policy.config
    # Short feedback horizon for closed-loop visual control; weights unchanged.
    config.n_action_steps=args.replan_steps
    policy.reset()
    print('Inputs:',{k:tuple(v.shape) for k,v in config.input_features.items()})
    print('Outputs:',{k:tuple(v.shape) for k,v in config.output_features.items()})
    if args.inspect:return
    if set(config.input_features)!={'observation.state','observation.images.front'}:
        raise RuntimeError('This sample assumes the single-front-camera ACT checkpoint')
    shape=config.input_features['observation.images.front'].shape
    assert len(shape)==3 and shape[0]==3
    if tuple(config.output_features['action'].shape)!=(6,):
        raise RuntimeError('Expected a 6D action')

    pre,post=make_pre_post_processors(config,args.model,
        preprocessor_overrides={'device_processor':{'device':device}})
    request_json('/health')
    dt=1.0/args.hz
    start=time.monotonic()
    for i in range(args.steps):
        t0=time.monotonic()
        data=request_json('/observation')
        joints=data['joints']
        state=(affine(joints,**mapping['state_to_policy']) if mapping
               else joints)
        img=decode_image(data['image'],shape[1],shape[2])
        observation={
            'observation.state':torch.tensor(state,dtype=torch.float32).unsqueeze(0),
            'observation.images.front':img,
        }
        batch=pre(observation)
        with torch.inference_mode():
            action=post(policy.select_action(batch))
        values=action.detach().cpu().reshape(-1).tolist()
        if len(values)!=6 or not all(math.isfinite(v) for v in values):
            raise RuntimeError('ACT predicted an invalid action')
        if args.execute:
            target=affine(values,**mapping['action_to_sim'])
            result=request_json('/action',{'positions':target})
            print(f'{i:04d} ACT={np.round(values,3)} -> joint={np.round(result["applied"],3)}',flush=True)
            if args.debug_control and i % args.replan_steps == 0:
                print('  policy_to_sim_target=', np.round(target, 4), flush=True)
                print('  measured_before     =', np.round(result['measured_before'], 4), flush=True)
                print('  after_joint_limits  =', np.round(result['clipped'], 4), flush=True)
                print('  applied_command     =', np.round(result['applied'], 4), flush=True)
                print('  joint_limit_hit     =', result['joint_limit_hit'], flush=True)
                print('  slew_limit_hit      =', result['slew_limit_hit'], flush=True)
        else:
            print(f'{i:04d} ACT={np.round(values,3)} [DRY RUN, NOT SENT]',flush=True)
        delay=dt-(time.monotonic()-t0)
        if delay>0:time.sleep(delay)
    print(f'Completed {args.steps} steps in {time.monotonic()-start:.1f}s')

if __name__=='__main__':main()
