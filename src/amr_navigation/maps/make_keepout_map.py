#!/usr/bin/env python3
"""Soft 'keep off the human lanes' map for the global costmap.

The floor art (layout.png) paints the human walkways green. They are not walls, so robots used to take
them as shortcuts. This writes maps/lanes_keepout.pgm/.yaml on the SAME grid as room_map_world (same size,
resolution, origin) in Nav2 'raw' mode: every green cell = LANE_VALUE, a short ramp beside it fades to 0.
Loaded as a second StaticLayer (use_maximum, non-trinary) in each robot's GLOBAL costmap only, so the planner
prefers the open floor next to a lane, while station mouths / nodes that lie on a lane stay reachable (cost is
high but below lethal, so crossing the lane is allowed, driving along it is expensive).

Usage: python3 make_keepout_map.py [layout.png] [room_map_world.yaml]
World <-> texture: floor box 3.0 x 2.5 m centred at x=+0.01 (Env solid), image top = +y (verified by matching
the wall map to the hazard lines of the artwork)."""
import os
import sys

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
LANE_VALUE = 30      # x 2.53 = costmap cost ~76: a gentle preference, must stay well below the cost next to walls
RAMP_VALUE = 10      # cost ~25 just outside the lane edge
RAMP_DIST = 0.10     # m


def main():
    tex = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, '..', '..', 'amr_simulation', 'webots', 'worlds', 'layout.png')
    ymlp = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, 'room_map_world.yaml')
    m = yaml.safe_load(open(ymlp))
    res, (ox, oy) = float(m['resolution']), m['origin'][:2]
    H, W = np.array(Image.open(os.path.join(os.path.dirname(ymlp), m['image']))).shape[:2]
    t = np.array(Image.open(tex).convert('RGB')).astype(int)
    th, tw = t.shape[:2]
    green = (t[:, :, 1] > 150) & (t[:, :, 0] < 60) & (t[:, :, 2] < 140)
    # sample each map cell on a 3x3 sub-grid; a cell is lane if most samples are green
    acc = np.zeros((H, W))
    for sy in (0.17, 0.5, 0.83):
        for sx in (0.17, 0.5, 0.83):
            cx = ox + (np.arange(W) + sx) * res
            cy = oy + (H - np.arange(H)[:, None] - 1 + (1 - sy)) * res
            u = (cx[None, :] + 1.49) / 3.0
            v = (1.25 - cy) / 2.5
            ok = (u >= 0) & (u < 1) & (v >= 0) & (v < 1)
            ui = np.clip((u * tw).astype(int), 0, tw - 1)
            vi = np.clip((v * th).astype(int), 0, th - 1)
            acc += np.where(ok, green[vi, ui], False)
    lane = acc >= 5
    dist = ndimage.distance_transform_edt(~lane) * res
    out = np.zeros((H, W), np.uint8)
    ramp = np.clip(1.0 - dist / RAMP_DIST, 0.0, 1.0) * RAMP_VALUE
    out[:] = ramp.astype(np.uint8)
    out[lane] = LANE_VALUE
    base = os.path.join(HERE, 'lanes_keepout')
    Image.fromarray(out, 'L').save(base + '.pgm')
    with open(base + '.yaml', 'w') as f:
        yaml.safe_dump({'image': 'lanes_keepout.pgm', 'mode': 'raw', 'resolution': res,
                        'origin': [float(ox), float(oy), 0.0], 'negate': 0,
                        'occupied_thresh': 0.65, 'free_thresh': 0.25}, f, sort_keys=False)
    print(f'lanes_keepout: {int(lane.sum())} lane cells of {H * W}')


if __name__ == '__main__':
    main()
