#!/usr/bin/env python3
"""Measure how much of the floor a run actually swept.

This is the benchmark instrument for phase 1. It scores a recorded trajectory
against the known scene rather than against the robot's own map, so the number
does not depend on how good the robot's SLAM happened to be that run. The scene
is fixed, so the reachable floor is a constant and two runs are directly
comparable.

Record the robot's ground-truth pose, then:

    python3 sim/tools/coverage_metrics.py run1.csv \\
        --world sim/worlds/coverage_test.world --duration 135

Trajectory file: whitespace or comma separated, optional header, columns
`t x y [yaw]`. Anything after the third column that is not yaw is ignored.
Gazebo's `libgazebo_ros_p3d` plugin writes a compatible file; so does

    ros2 topic echo /ground_truth/odom --field pose.pose.position

collected into a CSV.

Stdlib only, on purpose: this has to run on whatever machine recorded the run.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_scene import (  # noqa: E402
    ROBOT_DIAMETER,
    blocks_floor,
    collect,
    room_bounds,
)

RESOLUTION = 0.05   # m per grid cell
STEP = 0.02         # m between interpolated footprints


def load_trajectory(path: Path) -> list[tuple[float, float, float]]:
    """Return (t, x, y) triples. Yaw is read but not stored: the swept footprint
    is a circle, so orientation does not change what gets marked."""
    poses: list[tuple[float, float, float]] = []
    for lineno, raw in enumerate(path.read_text().splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.replace(",", " ").split()
        try:
            vals = [float(v) for v in parts]
        except ValueError:
            if lineno == 1:      # header row
                continue
            sys.exit(f"{path}:{lineno}: not numeric: {line!r}")
        if len(vals) < 3:
            sys.exit(f"{path}:{lineno}: need at least t x y, got {len(vals)} columns")
        # Column 4 is yaw if present; anything beyond it is ignored.
        poses.append((vals[0], vals[1], vals[2]))
    if len(poses) < 2:
        sys.exit(f"{path}: need at least two poses")
    return poses


def build_grid(cols, bounds, res):
    x0, y0, x1, y1 = bounds
    nx = int(math.ceil((x1 - x0) / res))
    ny = int(math.ceil((y1 - y0) / res))
    free = bytearray(nx * ny)
    for j in range(ny):
        cy = y0 + (j + 0.5) * res
        for i in range(nx):
            cx = x0 + (i + 0.5) * res
            if any(blocks_floor(c) and c.xmin <= cx <= c.xmax and c.ymin <= cy <= c.ymax
                   for c in cols):
                continue
            free[j * nx + i] = 1
    return free, nx, ny


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trajectory", type=Path)
    ap.add_argument("--world", type=Path,
                    default=Path("sim/worlds/coverage_test.world"))
    ap.add_argument("--diameter", type=float, default=ROBOT_DIAMETER,
                    help=f"robot body diameter in metres "
                         f"(default {ROBOT_DIAMETER}, from check_scene)")
    ap.add_argument("--duration", type=float, default=None,
                    help="score only the first N seconds, so runs of different "
                         "length can be compared fairly")
    ap.add_argument("--resolution", type=float, default=RESOLUTION)
    args = ap.parse_args()

    cols, _, _ = collect(str(args.world))
    bounds = room_bounds(cols)
    free, nx, ny = build_grid(cols, bounds, args.resolution)
    x0, y0, _, _ = bounds

    poses = load_trajectory(args.trajectory)
    if args.duration is not None:
        t_start = poses[0][0]
        poses = [p for p in poses if p[0] - t_start <= args.duration] or poses[:2]

    hits = bytearray(nx * ny)
    radius = args.diameter / 2
    r2 = radius * radius
    path = 0.0

    def stamp(px: float, py: float) -> None:
        i0 = max(0, int((px - radius - x0) / args.resolution))
        i1 = min(nx - 1, int((px + radius - x0) / args.resolution))
        j0 = max(0, int((py - radius - y0) / args.resolution))
        j1 = min(ny - 1, int((py + radius - y0) / args.resolution))
        for j in range(j0, j1 + 1):
            cy = y0 + (j + 0.5) * args.resolution
            dy = cy - py
            for i in range(i0, i1 + 1):
                cx = x0 + (i + 0.5) * args.resolution
                dx = cx - px
                if dx * dx + dy * dy <= r2:
                    k = j * nx + i
                    if hits[k] < 255:
                        hits[k] += 1

    for k in range(len(poses) - 1):
        _, ax, ay = poses[k]
        _, bx, by = poses[k + 1]
        seg = math.hypot(bx - ax, by - ay)
        path += seg
        n = max(1, int(seg / STEP))
        for s in range(n):
            f = s / n
            stamp(ax + (bx - ax) * f, ay + (by - ay) * f)
    stamp(poses[-1][1], poses[-1][2])

    cell = args.resolution ** 2
    total = sum(free)
    covered = sum(1 for k in range(nx * ny) if free[k] and hits[k])
    repeated = sum(1 for k in range(nx * ny) if free[k] and hits[k] > 1)
    wasted = sum(1 for k in range(nx * ny) if not free[k] and hits[k])

    wall = poses[-1][0] - poses[0][0]
    print(f"coverage report — {args.world.name}")
    print(f"  trajectory        {len(poses)} poses over {wall:.1f} s")
    print(f"  path length       {path:.1f} m")
    # Three decimals on purpose: the gap rig's tightest opening is 0.35 m and the
    # body is 0.349, so rounding this to 0.35 would hide the distinction the
    # scene is built around.
    print(f"  body diameter     {args.diameter:.3f} m")
    print(f"  reachable floor   {total * cell:.2f} m2  ({total} cells @ {args.resolution} m)")
    print()
    print(f"  covered           {covered * cell:.2f} m2  ->  {100 * covered / max(1, total):.1f} %")
    print(f"  swept two or more {repeated * cell:.2f} m2  "
          f"({100 * repeated / max(1, covered):.1f} % of covered)")
    print(f"  untouched         {(total - covered) * cell:.2f} m2  "
          f"({100 * (total - covered) / max(1, total):.1f} %)")
    print()
    print(f"  into obstacles    {wasted * cell:.2f} m2  "
          f"(footprint overlapping furniture, not a failure on its own)")

    if wall > 0:
        print(f"\n  rate              {covered * cell / wall:.3f} m2 swept per second")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
