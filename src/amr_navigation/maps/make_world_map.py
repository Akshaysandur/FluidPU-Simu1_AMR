#!/usr/bin/env python3
"""Rasterise the Webots world's wall boxes into a Nav2 map (map frame == Webots world frame).

Why: the SLAM-built room_map is ~3-8 % smaller than the simulated world and lacks the
Station-1 left wall and pillar, so AMCL/planner disagree with what the lidar sees.
Usage: make_world_map.py [world.wbt] [out_basename]
"""
import json, math, re, sys
import numpy as np
from PIL import Image

WORLD = sys.argv[1] if len(sys.argv) > 1 else '/home/akshay/rosws/src/amr_simulation/webots/worlds/simulation.wbt'
OUT = sys.argv[2] if len(sys.argv) > 2 else '/home/akshay/rosws/maps/room_map_world'
RES, OX, OY, W, H = 0.025, -1.70, -1.45, 136, 116     # 3.40 x 2.90 m, world-frame origin
SS = 5                                                  # supersampling per map cell

txt = open(WORLD).read()
env_start = txt.index('Solid {\n  translation 0.01 0 0')
env = txt[env_start:txt.index('name "Env"')]
env_dx = float(re.search(r'translation ([-\d.e]+)', txt[env_start:]).group(1))
boxes = []
for m in re.finditer(r'Solid \{\s*translation ([-\d. e]+)\n(?:\s*rotation ([-\d. e]+)\n)?\s*children \[(.*?)\n\s*name "([^"]*)"', env, re.S):
    if m.group(4).startswith('Tag'):
        continue
    t = [float(v) for v in m.group(1).split()]
    r = [float(v) for v in m.group(2).split()] if m.group(2) else [0, 0, 1, 0]
    sz = re.search(r'geometry Box \{\s*size ([-\d. e]+)', m.group(3))
    if not sz:
        continue
    s = [float(v) for v in sz.group(1).split()]
    boxes.append((t[0] + env_dx, t[1], r[3] * (1 if r[2] >= 0 else -1), s[0], s[1]))

ys, xs = np.mgrid[0:H * SS, 0:W * SS]
wx = OX + (xs + .5) * RES / SS
wy = OY + (H * SS - ys - .5) * RES / SS
occ = np.zeros(wx.shape, bool)
for bx, by, yaw, sx, sy in boxes:
    dx, dy = wx - bx, wy - by
    c, s = math.cos(-yaw), math.sin(-yaw)
    occ |= (abs(c * dx - s * dy) <= sx / 2) & (abs(s * dx + c * dy) <= sy / 2)
cell = occ.reshape(H, SS, W, SS).any(axis=(1, 3))
Image.fromarray(np.where(cell, 0, 254).astype(np.uint8)).save(OUT + '.pgm')
open(OUT + '.yaml', 'w').write(
    f"image: {OUT.split('/')[-1]}.pgm\nmode: trinary\nresolution: {RES}\n"
    f"origin: [{OX}, {OY}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
print(f'{len(boxes)} boxes -> {OUT}.pgm/.yaml  ({W}x{H}, {cell.sum()} occupied cells)')
