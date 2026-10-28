#!/usr/bin/env python3
"""Generate a self-contained, SO-101-like Gazebo joint-control demonstration world.
This is NOT a kinematically calibrated SO-101 representation.
"""
from pathlib import Path
from xml.etree import ElementTree as ET

OUT = Path(__file__).with_name('pick_place.sdf')

def el(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, {k:str(v) for k,v in attrs.items()})
    if text is not None:
        value = str(text)
        if tag == 'pose':
            parts = value.split()
            if len(parts) == 3:
                # SDF expects x y z roll pitch yaw. Default orientation is zero.
                value += ' 0 0 0'
            elif len(parts) != 6:
                raise ValueError(f'Invalid SDF pose ({len(parts)} values): {value}')
        e.text = value
    return e

def cube_geom(parent, size, color, collision=True):
    if collision:
        col = el(parent, 'collision', name='collision')
        geom = el(col, 'geometry'); el(geom, 'box'); el(geom.find('box'),'size',size)
        surf=el(col,'surface'); fr=el(surf,'friction'); ode=el(fr,'ode'); el(ode,'mu','2.0'); el(ode,'mu2','2.0')
    v = el(parent,'visual',name='visual'); g=el(v,'geometry'); el(g,'box'); el(g.find('box'),'size',size)
    m=el(v,'material'); el(m,'ambient',color); el(m,'diffuse',color)

def link(model,name,pose,size,mass,color,collide=True):
    l=el(model,'link',name=name);el(l,'pose',pose)
    inertial=el(l,'inertial');el(inertial,'mass',mass)
    i=el(inertial,'inertia');
    # positive diagonal inertia, deliberately approximate
    for t,v in [('ixx','.001'),('iyy','.001'),('izz','.001'),('ixy','0'),('ixz','0'),('iyz','0')]:el(i,t,v)
    cube_geom(l,size,color,collide);return l

def revolute(model, name, parent, child, pose, axis, lower,upper, effort=15):
    j=el(model,'joint',name=name,type='revolute');el(j,'pose',pose,relative_to='__model__');el(j,'parent',parent);el(j,'child',child)
    ax=el(j,'axis');el(ax,'xyz',axis);limits=el(ax,'limit');el(limits,'lower',lower);el(limits,'upper',upper);el(limits,'effort',effort)
    dyn=el(ax,'dynamics');el(dyn,'damping','0.3');return j

def prism(model,name,parent,child,pose,axis,lower,upper):
    j=el(model,'joint',name=name,type='prismatic');el(j,'pose',pose,relative_to='__model__');el(j,'parent',parent);el(j,'child',child)
    a=el(j,'axis');el(a,'xyz',axis);lim=el(a,'limit');el(lim,'lower',lower);el(lim,'upper',upper);el(lim,'effort','30');return j

def plugin(parent, filename, name):return el(parent,'plugin',name=name,filename=filename)

sdf=ET.Element('sdf',version='1.9'); world=el(sdf,'world',name='pick_place')
phys=el(world,'physics',name='physics',type='ignored');el(phys,'max_step_size','0.002');el(phys,'real_time_factor','1.0')
for fn,cl in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors')]:
    p=plugin(world,f'gz-sim-{fn}-system',f'gz::sim::systems::{cl}')
    if fn=='sensors':el(p,'render_engine','ogre2')
el(world,'gravity','0 0 -9.8')
scene=el(world,'scene');el(scene,'ambient','0.8 0.8 0.8 1');el(scene,'background','0.85 0.9 1 1')
light=el(world,'light',name='sun',type='directional');el(light,'pose','0 0 3 0 0 0');el(light,'direction','-0.3 -0.2 -1');el(light,'diffuse','1 1 1 1');el(light,'specular','.2 .2 .2 1')
# table surface at z=0.03
T=el(world,'model',name='table');el(T,'static','true')
link(T,'top','0.25 0 0.015','1.5 1.3 .03',5,'0.67 0.62 0.55 1')
# cube on top of table (z=.03), height .04
B=el(world,'model',name='cube');el(B,'pose','0.43 -0.16 0.05 0 0 0')
link(B,'body','0 0 0','.04 .04 .04',.04,'0.05 0.25 0.95 1')
G=el(world,'model',name='target_pad');el(G,'static','true');link(G,'pad','0.40 0.19 0.033','.16 .16 .006',.1,'0.18 0.80 0.28 1',False)
# 5 rotation + 1 prismatic jaw, not geometrically faithful to SO-101
R=el(world,'model',name='arm');el(R,'pose','0 0 0 0 0 0');el(R,'static','false')
link(R,'base','0 0 0.085','.14 .14 .17',3,'0.25 0.25 0.27 1')
j=el(R,'joint',name='base_fixed',type='fixed');el(j,'parent','world');el(j,'child','base')
link(R,'pan','0 0 0.19','.09 .09 .06',.5,'0.86 0.45 0.20 1')
link(R,'upper_arm','.115 0 .24','.23 .055 .06',.5,'0.94 0.54 0.24 1')
link(R,'forearm','.34 0 .24','.22 .05 .05',.4,'0.93 0.62 0.30 1')
link(R,'wrist_flex_link','.50 0 .24','.10 .05 .05',.18,'0.87 0.46 0.16 1')
link(R,'wrist_roll_link','.58 0 .24','.07 .06 .06',.12,'0.35 0.35 0.38 1')
# static finger fixed to wrist roll link via separate link
link(R,'fixed_finger','.67 -0.026 .24','.115 .012 .035',.07,'0.40 0.40 0.42 1')
link(R,'moving_finger','.67 0.028 .24','.115 .012 .035',.07,'0.40 0.40 0.42 1')
revolute(R,'shoulder_pan','base','pan','0 0 .17 0 0 0','0 0 1','-2.0','2.0')
revolute(R,'shoulder_lift','pan','upper_arm','0 0 .24 0 0 0','0 1 0','-1.4','1.4')
revolute(R,'elbow_flex','upper_arm','forearm','.23 0 .24 0 0 0','0 1 0','-1.9','1.9')
revolute(R,'wrist_flex','forearm','wrist_flex_link','.45 0 .24 0 0 0','0 1 0','-1.5','1.5')
revolute(R,'wrist_roll','wrist_flex_link','wrist_roll_link','.55 0 .24 0 0 0','1 0 0','-2.3','2.3')
j=el(R,'joint',name='finger_fixed',type='fixed');el(j,'parent','wrist_roll_link');el(j,'child','fixed_finger')
prism(R,'gripper','wrist_roll_link','moving_finger','.64 .028 .24 0 0 0','0 1 0','-.035','0.0')
JOINTS=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
for name in JOINTS:
    p=plugin(R,'gz-sim-joint-position-controller-system','gz::sim::systems::JointPositionController')
    el(p,'joint_name',name);el(p,'topic',f'/arm/cmd/{name}');el(p,'p_gain','25' if name!='gripper' else '60');el(p,'i_gain','0');el(p,'d_gain','1');el(p,'cmd_max','30');el(p,'cmd_min','-30')
p=plugin(R,'gz-sim-joint-state-publisher-system','gz::sim::systems::JointStatePublisher');el(p,'topic','/arm/joint_states');el(p,'update_rate','30')
# static camera aimed from the side of the table, +X axis is optical forward in Gazebo
C=el(world,'model',name='front_camera');el(C,'static','true')
cl=el(C,'link',name='camera_link');el(cl,'pose','1.06 -0.90 0.66 0 0.34 2.42')
cam=el(cl,'sensor',name='front',type='camera');el(cam,'always_on','true');el(cam,'update_rate','15');el(cam,'topic','/camera/front/image_raw')
c=el(cam,'camera');el(c,'horizontal_fov','1.2');im=el(c,'image');el(im,'width','640');el(im,'height','480');el(im,'format','R8G8B8');clip=el(c,'clip');el(clip,'near','.01');el(clip,'far','5')
# Explicit GUI config not necessary: Gazebo uses built-in default GUI.
ET.indent(sdf,space='  ')
OUT.write_text('<?xml version="1.0"?>\n'+ET.tostring(sdf,encoding='unicode'))
print(OUT)
