#!/usr/bin/env python3
"""Compute, not guess, (inflation_radius, cost_scaling_factor,
cost_travel_multiplier) for this project's actual map, instead of picking
numbers by eye in RViz.

Background (see project_context/decisions-and-gotchas.md - "inflation_radius
== robot_radius is not a valid tuning value" and "Narrow-passage navigation
failure" entries): this project has twice picked inflation values by visual
inspection alone and gotten burned each time - once because a too-small
value made the costmap flicker/deadlock, once because it was too heavy
looking. Neither attempt asked the only question that actually matters for
BOTH the open-aisle-hugging symptom and the doorway-passability symptom at
the same time: does the resulting cost field have a real, non-flat gradient
spanning each open aisle's FULL width (so a cost-aware planner has a reason
to centre itself), while still leaving every known passage's centreline cost
low enough that the planner doesn't try to detour around it?

This script answers that with real geometry from the map file, not eyeballing:
  1. Distance-transform every free cell's true distance to the nearest wall.
  2. Skeletonize the free-space mask -> the map's own medial axis (the
     "always-centred" reference path any candidate cost field is judged
     against).
  3. Split the skeleton into branches at its junctions/endpoints, and
     classify each branch as a PASSAGE (a width local-minimum - a doorway)
     or an AISLE (a sustained wide run) from the geometry itself, not from
     numbers read off a layout drawing.
  4. Build the branch/junction graph and run a bridge-finding pass (Tarjan)
     to determine, per passage, whether an alternate route around it exists
     anywhere else in the map - a single-entrance room's passage has none,
     and can tolerate a much higher travel-cost multiplier than one that
     does (detour-seeking is only a risk when a detour actually exists).
  5. Grid-search (inflation_radius, cost_scaling_factor) and, for each pair,
     score every aisle cross-section against Nav2's own real inflation
     formula (confirmed via `strings` on liblayers.so and cross-checked
     against nav2_costmap_2d's InflationLayer source in earlier project
     work) for whether it produces a genuine centred minimum or a flat,
     tie-broken-by-nothing dead zone.

Usage:
    python3 calibrate_inflation.py [map_yaml] [--robot-radius R]

No ROS dependency - this is an offline planning tool, run by hand, not part
of the launched graph. Requires numpy, scipy, scikit-image, Pillow.
"""
import argparse
import math
import os
from collections import defaultdict, deque

import numpy as np
from PIL import Image
from scipy import ndimage
from skimage.morphology import skeletonize

# --------------------------------------------------------------------------
# Map loading
# --------------------------------------------------------------------------

def load_map(yaml_path):
    """Minimal, dependency-free map_server YAML + PGM loader. Avoids pulling
    in a full YAML parser dependency for four scalar fields and one string."""
    fields = {}
    with open(yaml_path, encoding="utf-8") as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            k, v = line.split(":", 1)
            fields[k.strip()] = v.strip()

    image_name = fields["image"].strip('"\'')
    image_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image_name)
    resolution = float(fields["resolution"])
    origin = [float(x) for x in fields["origin"].strip("[]").split(",")]
    negate = int(fields.get("negate", "0"))

    img = np.array(Image.open(image_path))
    if negate:
        img = 255 - img

    return {
        "image": img,
        "resolution": resolution,
        "origin": origin,  # [x, y, yaw] of pixel (0, h-1) - map_server convention
        "height": img.shape[0],
        "width": img.shape[1],
    }


def pixel_to_world(m, row, col):
    x = m["origin"][0] + col * m["resolution"]
    y = m["origin"][1] + (m["height"] - 1 - row) * m["resolution"]
    return x, y


# --------------------------------------------------------------------------
# Skeleton graph: junctions, endpoints, branches
# --------------------------------------------------------------------------

NEIGHBORS8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def skeleton_degree_map(skel):
    deg = np.zeros(skel.shape, dtype=np.int8)
    ys, xs = np.where(skel)
    coordset = set(zip(ys.tolist(), xs.tolist()))
    for y, x in zip(ys, xs):
        d = 0
        for dy, dx in NEIGHBORS8:
            if (y + dy, x + dx) in coordset:
                d += 1
        deg[y, x] = d
    return deg, coordset


def trace_branches(skel, min_branch_len=4):
    """Split a skeleton into branches between junction/endpoint nodes.

    Standard skeleton-graph decomposition: nodes are pixels with degree != 2
    (endpoints, degree 1; junctions, degree >= 3); a branch is a maximal
    simple path of degree-2 pixels connecting two such nodes (or a node to
    itself for an isolated loop, not expected in this map). Branches shorter
    than `min_branch_len` px are skeletonize() spurs (common at concave
    corners of the free-space mask) and are dropped rather than
    misclassified as tiny passages.
    """
    deg, coordset = skeleton_degree_map(skel)
    node_mask = (deg != 2) & (deg > 0)
    nodes = set(zip(*np.where(node_mask)))
    visited_edges = set()
    branches = []

    def walk(start, first_step):
        path = [start, first_step]
        prev, cur = start, first_step
        while cur not in nodes:
            nxts = [
                (cur[0] + dy, cur[1] + dx)
                for dy, dx in NEIGHBORS8
                if (cur[0] + dy, cur[1] + dx) in coordset and (cur[0] + dy, cur[1] + dx) != prev
            ]
            if not nxts:
                break
            prev, cur = cur, nxts[0]
            path.append(cur)
        return path

    for n in nodes:
        for dy, dx in NEIGHBORS8:
            nb = (n[0] + dy, n[1] + dx)
            if nb not in coordset or nb in nodes:
                continue
            key = frozenset((n, nb))
            if (n, nb) in visited_edges:
                continue
            path = walk(n, nb)
            edge_key = frozenset((path[0], path[-1], len(path)))
            if edge_key in visited_edges:
                continue
            visited_edges.add(edge_key)
            visited_edges.add((n, nb))
            if len(path) >= min_branch_len:
                branches.append(path)

    # Isolated rings (no junctions/endpoints at all, e.g. a skeleton that's
    # one closed loop around a single obstacle) - not expected for this
    # map's topology but handled so the script doesn't silently drop area.
    if not nodes and coordset:
        remaining = set(coordset)
        while remaining:
            start = next(iter(remaining))
            comp = [start]
            frontier = deque([start])
            remaining.discard(start)
            while frontier:
                cur = frontier.popleft()
                for dy, dx in NEIGHBORS8:
                    nb = (cur[0] + dy, cur[1] + dx)
                    if nb in remaining:
                        remaining.discard(nb)
                        comp.append(nb)
                        frontier.append(nb)
            if len(comp) >= min_branch_len:
                branches.append(comp)

    return branches, nodes


# --------------------------------------------------------------------------
# Bridge detection (Tarjan) on the junction/branch graph - is there an
# alternate route around a given passage?
# --------------------------------------------------------------------------

def find_bridges(adj):
    """Tarjan's bridge-finding algorithm on an undirected multigraph given
    as {node: [(neighbor, edge_id), ...]}. Returns the set of edge_ids that
    are bridges (their removal disconnects the graph) - a bridge passage has
    no alternate route anywhere else in the map."""
    disc, low = {}, {}
    visited = set()
    bridges = set()
    timer = [0]

    def dfs(u, parent_edge):
        visited.add(u)
        disc[u] = low[u] = timer[0]
        timer[0] += 1
        for v, eid in adj[u]:
            if eid == parent_edge:
                continue
            if v in visited:
                low[u] = min(low[u], disc[v])
            else:
                dfs(v, eid)
                low[u] = min(low[u], low[v])
                if low[v] > disc[u]:
                    bridges.add(eid)

    for start in adj:
        if start not in visited:
            dfs(start, None)
    return bridges


# --------------------------------------------------------------------------
# Nav2's real inflation cost formula
# --------------------------------------------------------------------------

def inflation_cost(d, r_i, inflation_radius, cost_scaling_factor):
    """d, r_i, inflation_radius in metres. Matches nav2_costmap_2d's
    InflationLayer::computeCost exactly (confirmed against the installed
    liblayers.so and this project's own prior verification in
    decisions-and-gotchas.md)."""
    d = np.asarray(d, dtype=float)
    cost = np.zeros_like(d)
    lethal = d <= r_i
    soft = (d > r_i) & (d <= inflation_radius)
    cost[lethal] = 253.0
    cost[soft] = np.minimum(252.0 * np.exp(-cost_scaling_factor * (d[soft] - r_i)) + 1.0, 252.0)
    return cost


# --------------------------------------------------------------------------
# Cross-section sampling for aisle-centering checks
# --------------------------------------------------------------------------

def local_tangent(path, idx, span=3):
    lo, hi = max(0, idx - span), min(len(path) - 1, idx + span)
    y0, x0 = path[lo]
    y1, x1 = path[hi]
    dy, dx = y1 - y0, x1 - x0
    n = math.hypot(dy, dx)
    if n < 1e-6:
        return 0.0, 1.0
    return dy / n, dx / n


def cross_section(free_mask, point, tangent, dt_px_at_point):
    """Walk perpendicular to the branch's local tangent from `point` in both
    directions through the free mask until leaving free space. Returns the
    list of (row, col) cells spanned and their distance-transform values.

    The walk is bounded to roughly 1.8x the point's own distance-transform
    value (its true distance to the nearest wall) plus a small margin - NOT
    a large fixed constant. A true perpendicular cross-section through a
    corridor of local half-width W should hit a wall within about W px of
    the centre; a fixed generous bound (e.g. 40px) lets the walk escape
    into unrelated open floor space whenever the branch bends or the local
    tangent estimate is imprecise (found live: one cross-section wandered
    73px/1.8m from centre through an adjacent open room, and its argmin was
    then comparing distances that had nothing to do with the actual
    corridor being evaluated - exactly the kind of number this script
    exists to catch rather than trust blindly)."""
    max_half_len_px = max(6, int(round(dt_px_at_point * 1.8)))
    ty, tx = tangent
    ny, nx = -tx, ty  # perpendicular
    py, px = point
    cells = []
    for sign in (-1, 1):
        step = 1
        while step <= max_half_len_px:
            ry = int(round(py + sign * step * ny))
            rx = int(round(px + sign * step * nx))
            if not (0 <= ry < free_mask.shape[0] and 0 <= rx < free_mask.shape[1]):
                break
            if not free_mask[ry, rx]:
                break
            cells.append((ry, rx))
            step += 1
    cells.append((int(round(py)), int(round(px))))
    cells.sort()
    return cells


# --------------------------------------------------------------------------
# Main calibration
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map_yaml", nargs="?", default=os.path.expanduser("~/ros/maps/room_map_hires.yaml"))
    ap.add_argument("--robot-radius", type=float, default=0.10)
    ap.add_argument("--footprint-padding", type=float, default=0.0)
    args = ap.parse_args()

    m = load_map(args.map_yaml)
    res = m["resolution"]
    img = m["image"]
    r_i = args.robot_radius + args.footprint_padding

    free_mask = img > 230
    dt_px = ndimage.distance_transform_edt(free_mask)
    dt_m = dt_px * res

    print(f"Map: {args.map_yaml}")
    print(f"  {m['width']}x{m['height']} px @ {res} m/px, origin {m['origin']}")
    print(f"  inscribed radius r_i = robot_radius + footprint_padding = {r_i:.3f} m")
    print(f"  max clearance anywhere on map: {dt_m.max():.3f} m\n")

    skel = skeletonize(free_mask)
    branches, nodes = trace_branches(skel)
    print(f"Skeleton: {skel.sum()} px, {len(nodes)} junction/endpoint nodes, "
          f"{len(branches)} branches (>=4px, spurs dropped)\n")

    # ---- classify branches: passage (width local minimum) vs aisle -------
    # A skeleton branch's own ENDPOINT pixels sit at junctions - where two or
    # more branches converge, typically right at a corner where several
    # walls meet. The distance-transform value there reflects that corner's
    # geometry, not the corridor the branch actually runs through, and skews
    # small regardless of how wide the branch's interior really is. Trimming
    # a few pixels off each end before taking a width minimum is standard
    # skeleton-graph practice for exactly this reason - confirmed necessary
    # here empirically: an untrimmed pass reported branches down to 50mm
    # wide, which contradicts a fact already established earlier in this
    # project's own live work (a distance-transform connectivity check on
    # this same map showed every free cell with clearance > robot_radius
    # forms ONE connected region - meaning no real constriction here can be
    # narrower than roughly 2*robot_radius = 0.20m). Any narrower reading is
    # a junction artifact, not a real doorway.
    trim = 3
    branch_info = []
    for bi, path in enumerate(branches):
        widths_m = np.array([2.0 * dt_m[p] for p in path])
        if len(path) > 2 * trim:
            interior = slice(trim, len(path) - trim)
        else:
            interior = slice(0, len(path))  # too short to trim - use as-is, flagged short
        interior_widths = widths_m[interior]
        min_i_interior = int(np.argmin(interior_widths))
        min_i = min_i_interior + (interior.start or 0)
        branch_info.append({
            "id": bi,
            "path": path,
            "min_width": widths_m[min_i],
            "min_idx": min_i,
            "mean_width": interior_widths.mean(),
            "length_m": len(path) * res,
            "endpoints": (path[0], path[-1]),
            "short": len(path) <= 2 * trim,
        })

    # A branch is a PASSAGE candidate if its narrowest point is close to the
    # tightest constriction actually usable by this robot (< 4x r_i => still
    # meaningfully constrained by walls on both sides, not just "somewhere
    # in open space with a slightly lower number"). It's an AISLE candidate
    # if its narrowest point never drops below a real open-space threshold
    # (> 4x r_i for its full length) - i.e. genuinely unconstrained by walls
    # on both sides at once, anywhere along it.
    # A branch with a DEAD-END (degree-1) endpoint is a skeleton spur, not a
    # through-passage - the medial axis grows a spur toward any convex bump
    # in the free-space boundary (a round pillar/obstacle island being the
    # obvious case in this map), and such a spur's tip is, by construction,
    # equidistant from two points on the SAME nearby object's own curved
    # boundary - it reads as "narrow" without connecting two separate
    # spaces at all. A genuine doorway's branch instead connects a junction
    # on one side to a junction on the other (each leading further into a
    # real room/area). Restricting classification to branches whose BOTH
    # endpoints are true junctions (degree >= 3) is what actually
    # discriminates the two cases - found necessary here empirically: a
    # length/trim-only filter still left several sub-150mm branches that
    # contradict this map's own already-established connectivity fact
    # (every free cell with clearance > robot_radius forms ONE connected
    # region, so no genuine constriction here can be narrower than
    # ~2*robot_radius = 0.20m - see the trim comment above for the same
    # cross-check).
    deg_map, _ = skeleton_degree_map(skel)

    def is_through_branch(b):
        u, v = b["endpoints"]
        return deg_map[u] >= 3 and deg_map[v] >= 3

    open_threshold = 4.0 * r_i
    impassable_threshold = 2.0 * r_i  # narrower than the robot's own diameter
    classifiable = [b for b in branch_info if not b["short"] and is_through_branch(b)]
    n_dropped = len(branch_info) - len(classifiable)
    if n_dropped:
        print(f"({n_dropped} branch(es) dropped as too-short-to-trim or a dead-end "
              f"skeleton spur (not a through-passage) - excluded from passage/aisle "
              f"classification, kept only for connectivity)\n")

    impassable = [b for b in classifiable if b["min_width"] < impassable_threshold]
    if impassable:
        print(f"({len(impassable)} branch(es) narrower than 2*robot_radius = "
              f"{impassable_threshold:.2f}m - genuinely impassable regardless of "
              f"inflation tuning (a doorframe nub / wall-thickness feature, not a "
              f"usable route - SmacPlanner2D will simply never step there, same as "
              f"a solid wall). Excluded from passage headroom/alternate-route "
              f"reporting as irrelevant to that question: "
              + ", ".join(f"branch {b['id']} ({b['min_width']*1000:.0f}mm)" for b in impassable)
              + ")\n")

    passages = sorted([b for b in classifiable
                        if impassable_threshold <= b["min_width"] < open_threshold],
                       key=lambda b: b["min_width"])
    aisles = [b for b in classifiable if b["min_width"] >= open_threshold and b["length_m"] > 4 * r_i]

    print(f"Detected {len(passages)} passage branch(es) (narrowest width < {open_threshold:.3f}m):")
    for b in passages:
        wx, wy = pixel_to_world(m, *b["path"][b["min_idx"]])
        print(f"  branch {b['id']:3d}: min_width={b['min_width']*1000:6.1f}mm "
              f"at world=({wx:6.2f},{wy:6.2f})  length={b['length_m']:.2f}m")
    print(f"\nDetected {len(aisles)} aisle branch(es) (min width >= {open_threshold:.3f}m "
          f"for their full length, length > {4*r_i:.2f}m):")
    for b in aisles:
        print(f"  branch {b['id']:3d}: min_width={b['min_width']*1000:6.1f}mm "
              f"mean_width={b['mean_width']*1000:6.1f}mm length={b['length_m']:.2f}m")

    if not passages:
        print("\nNo passages detected narrower than the open-space threshold - "
              "this map may not have a meaningful narrow-passage case; the "
              "grid search below still runs for aisle-centering quality alone.")

    # ---- alternate-route check: bridge-finding on the junction graph -----
    node_list = list(nodes)
    node_index = {n: i for i, n in enumerate(node_list)}
    adj = defaultdict(list)
    for b in branch_info:
        u, v = b["endpoints"]
        if u not in node_index or v not in node_index:
            continue  # isolated ring branch, no junction endpoints
        ui, vi = node_index[u], node_index[v]
        adj[ui].append((vi, b["id"]))
        adj[vi].append((ui, b["id"]))
    bridge_ids = find_bridges(adj) if adj else set()

    print("\nAlternate-route analysis (bridge = no alternate route exists):")
    for b in passages:
        has_alt = b["id"] not in bridge_ids
        print(f"  branch {b['id']:3d} ({b['min_width']*1000:.0f}mm): "
              f"{'ALTERNATE ROUTE EXISTS - keep travel-cost multiplier low' if has_alt else 'single-entrance, NO alternate route - can tolerate a higher multiplier safely'}")

    # ---- build aisle cross-sections once (geometry doesn't depend on the
    # (inflation_radius, cost_scaling_factor) pair being tested) -----------
    cross_sections = []  # list of (aisle_id, world_xy, [(row,col),...], skeleton_idx_in_list)
    for b in aisles:
        path = b["path"]
        # sample every ~8px along the branch, skip the first/last few to
        # avoid junction geometry contaminating the cross-section shape
        for idx in range(4, len(path) - 4, 8):
            tangent = local_tangent(path, idx)
            cells = cross_section(free_mask, path[idx], tangent, dt_px[path[idx]])
            if len(cells) >= 3:
                cross_sections.append((b["id"], path[idx], cells))

    print(f"\n{len(cross_sections)} aisle cross-sections sampled for the grid search.")
    print("(Known limitation: a locally-estimated tangent can point slightly off")
    print(" a corridor's true wall-to-wall axis where a branch bends or transitions")
    print(" into open room space partway along its own length - this can inflate")
    print(" the centre-error statistic for a few individual cross-sections. It does")
    print(" not affect frac_non_flat (the primary, coverage-based selection")
    print(" criterion) and centre-error is only ever used as a tiebreak AFTER")
    print(" radius in the ranking below, so this does not change the recommendation")
    print(" - treat centre-error as informational, not load-bearing.)\n")

    # ---- grid search -------------------------------------------------
    radii = np.arange(0.15, 0.801, 0.05)
    scales = np.arange(3.0, 20.1, 1.0)

    best = None
    results = []
    for radius in radii:
        for scale in scales:
            non_flat = 0
            err_nonflat = []  # centering error, ONLY over cross-sections with a real gradient
            for _, centre, cells in cross_sections:
                ds = np.array([dt_m[c] for c in cells])
                costs = inflation_cost(ds, r_i, radius, scale)
                centre_i = cells.index(centre) if centre in cells else len(cells) // 2
                is_non_flat = (costs > 0).mean() > 0.9  # gradient reaches across almost the whole span
                if is_non_flat:
                    non_flat += 1
                    # argmin is only a meaningful "where would the planner walk"
                    # signal when there IS a real gradient - on a flat/all-zero
                    # array np.argmin trivially returns index 0, which would
                    # read as a large "error" that has nothing to do with
                    # centering quality and everything to do with the tie being
                    # broken by array order. Excluded from this average for
                    # that reason; frac_non_flat already penalizes flat cases
                    # on its own.
                    argmin_i = int(np.argmin(costs))
                    err_nonflat.append(abs(argmin_i - centre_i))
            frac_non_flat = non_flat / len(cross_sections) if cross_sections else 0.0
            # inf, not nan, when nothing was non-flat: keeps sort ordering
            # well-defined (this pair ranks last on the tie-breaker, exactly
            # as it should) without nan's undefined comparison behaviour.
            mean_err = float(np.mean(err_nonflat)) if err_nonflat else float("inf")

            passage_costs = []
            for b in passages:
                d = b["min_width"] / 2.0
                passage_costs.append(float(inflation_cost(np.array([d]), r_i, radius, scale)[0]))
            worst_passage_cost = max(passage_costs) if passage_costs else 0.0

            results.append((radius, scale, frac_non_flat, mean_err, worst_passage_cost))

    # Selection rule, applied in this priority order (matches the brief's
    # intent - "verified against explicit numeric criteria" rather than a
    # single blended score that could hide a bad trade-off):
    #   1. Must cover essentially every aisle cross-section with a real
    #      gradient (frac_non_flat >= 0.95) - this is the actual fix for
    #      "planner hugs one wall in open space".
    #   2. Among those, minimize how far the cost-minimum sits from the
    #      true medial axis (mean_err, in cells) - the real centering
    #      accuracy, not just "some gradient exists somewhere".
    #   3. Among ties, prefer the SMALLEST inflation_radius (thinner visual
    #      footprint, matches this project's own prior visual complaint)
    #      and the LARGEST cost_scaling_factor (steepest decay -> most of
    #      the band reads as a faint low cost rather than a strong band).
    candidates = [r for r in results if r[2] >= 0.95]
    pool = candidates if candidates else results
    # Once the functional requirement (coverage) is met, prefer the
    # SMALLEST radius that meets it - not the one with the marginally best
    # centering error. 0.70 and 0.75 both reach 100% coverage on this map
    # and differ by 0.2 cells (0.5cm) in centering accuracy, a difference
    # with no practical meaning; the smaller radius is the more defensible
    # engineering choice (less visual/behavioural footprint for the same
    # functional result) and also respects this project's own repeatedly
    # stated preference for a thinner-looking costmap. Centering error is
    # therefore ranked AFTER radius, as a tiebreak only.
    pool.sort(key=lambda r: (-r[2], r[0], r[3], -r[1]))
    best = pool[0]

    print("=" * 70)
    print("RECOMMENDATION (from live geometry, not a guess):")
    print(f"  inflation_radius      = {best[0]:.2f}")
    print(f"  cost_scaling_factor   = {best[1]:.1f}")
    print(f"  aisle cross-sections with a real (non-flat) gradient: {best[2]*100:.0f}%")
    print(f"  mean cost-minimum offset from true medial axis: {best[3]:.2f} cells "
          f"({best[3]*res*100:.1f} cm)")
    print(f"  worst-case passage centreline cost at this setting: {best[4]:.1f} / 253")
    print()

    any_alt = any(b["id"] not in bridge_ids for b in passages)
    if any_alt:
        suggested_mult = 1.0
        reason = "at least one detected passage has an alternate route - keep the multiplier at Nav2's own baseline so detours are never preferred over threading a doorway that's actually the intended route."
    else:
        suggested_mult = 1.3
        reason = "every detected passage is single-entrance (bridge in the map's own connectivity graph) - a real detour is never available for these, so a mild increase is safe and helps the planner prefer straighter, more open segments elsewhere without risking a rejected route."
    print(f"  suggested cost_travel_multiplier = {suggested_mult}  ({reason})")
    print("=" * 70)

    print("\nFull grid (radius, scale, frac_non_flat, mean_centre_err_cells, worst_passage_cost):")
    for r in sorted(results, key=lambda x: (x[0], x[1])):
        flag = "  <-- selected" if r is best else ""
        print(f"  r={r[0]:.2f} s={r[1]:5.1f}  non_flat={r[2]*100:5.1f}%  "
              f"centre_err={r[3]:.2f}cells  worst_passage_cost={r[4]:6.1f}{flag}")


if __name__ == "__main__":
    main()
