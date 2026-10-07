#!/usr/bin/env python3
"""Validate coverage_test.world against the numbers its comments claim.

Checks that every collider parses, that nothing overlaps by accident, that the
gap rig really presents 0.35 / 0.50 / 0.70 m openings, and that the doorway and
the reachable floor area are what the scene description says they are.

Run after editing the world:

    python3 sim/tools/check_scene.py sim/worlds/coverage_test.world
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass

ROBOT_DIAMETER = 0.35   # m, the reference body. Change only if the robot changes.
FLOOR_BAND = 0.15       # m, anything whose lowest point is below this blocks the robot.
DRIVEABLE_HEIGHT = 0.03  # m, obstacles shorter than this are driven over, not around.

# Every benchmark run starts here. Fixed on purpose: a run that starts somewhere
# else is not comparable with the others. Keep this clear of furniture.
START_POSE = (-3.00, 0.60, 0.0)  # x, y, yaw (radians)


def blocks_floor(c: "Collider") -> bool:
    """Does this collider take floor away from the robot?"""
    return c.zmin < FLOOR_BAND and (c.zmax - c.zmin) > DRIVEABLE_HEIGHT


def pose_of(elem) -> tuple[float, float, float, float, float, float]:
    """Read a <pose> child, defaulting to the origin."""
    p = elem.find("pose")
    if p is None or not (p.text or "").strip():
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    vals = [float(v) for v in p.text.split()]
    while len(vals) < 6:
        vals.append(0.0)
    return tuple(vals[:6])  # type: ignore[return-value]


@dataclass
class Collider:
    model: str
    link: str
    kind: str
    cx: float
    cy: float
    zmin: float
    zmax: float
    sx: float  # x extent (full width)
    sy: float  # y extent (full width)

    @property
    def xmin(self) -> float:
        return self.cx - self.sx / 2

    @property
    def xmax(self) -> float:
        return self.cx + self.sx / 2

    @property
    def ymin(self) -> float:
        return self.cy - self.sy / 2

    @property
    def ymax(self) -> float:
        return self.cy + self.sy / 2


def collect(path: str) -> tuple[list[Collider], float, float]:
    root = ET.parse(path).getroot()
    world = root.find("world")
    if world is None:
        sys.exit("no <world> element")

    out: list[Collider] = []
    for model in world.findall("model"):
        mname = model.get("name", "?")
        mx, my, mz, mr, mp, myaw = pose_of(model)
        if any(abs(v) > 1e-9 for v in (mr, mp, myaw)):
            sys.exit(f"model {mname} is rotated; this checker only handles yaw=0")

        for link in model.findall("link"):
            lx, ly, lz, *_ = pose_of(link)
            for col in link.findall("collision"):
                geom = col.find("geometry")
                if geom is None:
                    continue
                box = geom.find("box")
                cyl = geom.find("cylinder")
                if box is not None:
                    sx, sy, sz = [float(v) for v in box.find("size").text.split()]
                    kind = "box"
                elif cyl is not None:
                    r = float(cyl.find("radius").text)
                    sz = float(cyl.find("length").text)
                    sx = sy = 2 * r
                    kind = "cylinder"
                else:
                    continue
                zc = mz + lz
                out.append(Collider(mname, link.get("name", "?"), kind,
                                    mx + lx, my + ly, zc - sz / 2, zc + sz / 2, sx, sy))
    return out, 0.0, 0.0


def room_bounds(cols: list[Collider]) -> tuple[float, float, float, float]:
    """Interior extents (x0, y0, x1, y1) taken from the four perimeter walls."""
    walls = {c.model: c for c in cols if c.model.startswith("wall_")}
    missing = {"wall_west", "wall_east", "wall_south", "wall_north"} - set(walls)
    if missing:
        sys.exit(f"missing perimeter walls: {', '.join(sorted(missing))}")
    return (walls["wall_west"].xmax, walls["wall_south"].ymax,
            walls["wall_east"].xmin, walls["wall_north"].ymin)


def overlaps(a: Collider, b: Collider) -> bool:
    return (a.xmin < b.xmax and b.xmin < a.xmax and
            a.ymin < b.ymax and b.ymin < a.ymax and
            a.zmin < b.zmax and b.zmin < a.zmax)


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else "sim/worlds/coverage_test.world"
    cols, _, _ = collect(path)
    print(f"parsed {len(cols)} colliders from {path}\n")

    problems: list[str] = []

    # 1. accidental overlap of distinct models
    print("overlap check")
    hits = 0
    for i, a in enumerate(cols):
        for b in cols[i + 1:]:
            if a.model != b.model and overlaps(a, b):
                problems.append(f"overlap: {a.model}/{a.link} intersects {b.model}/{b.link}")
                hits += 1
    print(f"  {'none' if not hits else str(hits) + ' collisions'}\n")

    # 2. gap rig openings
    print("gap rig")
    blocks = sorted((c for c in cols if c.model == "gap_rig"), key=lambda c: c.cx)
    if len(blocks) < 2:
        problems.append("gap_rig has fewer than two blocks")
    for a, b in zip(blocks, blocks[1:]):
        gap = b.xmin - a.xmax
        verdict = ("impassable" if gap < ROBOT_DIAMETER else
                   "tight" if gap < ROBOT_DIAMETER + 0.15 else "open")
        print(f"  {a.link:>8} -> {b.link:<8} {gap:.2f} m   {verdict} "
              f"(body {ROBOT_DIAMETER:.2f} m)")
    print()

    # 3. doorway
    print("doorway")
    parts = sorted((c for c in cols if c.model.startswith("partition")), key=lambda c: c.cy)
    if len(parts) == 2:
        opening = parts[1].ymin - parts[0].ymax
        print(f"  clear width {opening:.2f} m between the two partition segments")
        if opening < ROBOT_DIAMETER + 0.3:
            problems.append(f"doorway {opening:.2f} m is tight for a {ROBOT_DIAMETER} m body")
    thr = [c for c in cols if c.model == "threshold"]
    if thr:
        t = thr[0]
        print(f"  threshold height {t.zmax - t.zmin:.3f} m, spans x "
              f"{t.xmin:.2f}..{t.xmax:.2f}")
        if abs((t.zmax - t.zmin) - 0.02) > 1e-6:
            problems.append("threshold is not 20 mm")
    print()

    # 4. reachable floor
    print("floor")
    walls = [c for c in cols if c.model.startswith("wall_")]
    if walls:
        x0 = max(c.xmax for c in walls if c.model == "wall_west")
        x1 = min(c.xmin for c in walls if c.model == "wall_east")
        y0 = max(c.ymax for c in walls if c.model == "wall_south")
        y1 = min(c.ymin for c in walls if c.model == "wall_north")
        gross = (x1 - x0) * (y1 - y0)
        print(f"  interior {x1 - x0:.2f} x {y1 - y0:.2f} m = {gross:.2f} m2")

        blocked = 0.0
        for c in cols:
            if c.model.startswith("wall_") or not blocks_floor(c):
                continue
            w = min(c.xmax, x1) - max(c.xmin, x0)
            h = min(c.ymax, y1) - max(c.ymin, y0)
            if w > 0 and h > 0:
                blocked += w * h
        print(f"  blocked below {FLOOR_BAND} m: {blocked:.2f} m2")
        print(f"  reachable floor: {gross - blocked:.2f} m2")

        floating = [c for c in cols if c.zmin >= FLOOR_BAND]
        if floating:
            parts = ", ".join(sorted(f"{c.model}/{c.link} from {c.zmin:.2f} m" for c in floating))
            print(f"  floating clear of the robot: {parts}")
        drive = [c for c in cols if c.zmin < FLOOR_BAND and not blocks_floor(c)]
        if drive:
            parts = ", ".join(sorted(f"{c.model} ({(c.zmax - c.zmin) * 1000:.0f} mm)" for c in drive))
            print(f"  low enough to drive over: {parts}")
    print()

    # 5. the spawn point must be clear at body radius, or every run starts fouled
    print("start pose")
    sx, sy, syaw = START_POSE
    r = ROBOT_DIAMETER / 2
    blocking = [c for c in cols if blocks_floor(c) and
                c.xmin < sx + r and sx - r < c.xmax and
                c.ymin < sy + r and sy - r < c.ymax]
    if blocking:
        names = ", ".join(sorted({f"{c.model}/{c.link}" for c in blocking}))
        print(f"  ({sx:.2f}, {sy:.2f}) is fouled by {names}")
        problems.append("start pose overlaps an obstacle")
    else:
        print(f"  ({sx:.2f}, {sy:.2f}, {syaw:.2f} rad) clear, "
              f"{r:.3f} m body radius respected")
    x0w, y0w, x1w, y1w = room_bounds(cols)
    if not (x0w + r < sx < x1w - r and y0w + r < sy < y1w - r):
        problems.append("start pose is outside the room")
    print()

    if problems:
        print("FAIL")
        for p in problems:
            print(f"  - {p}")
        return 1
    print("OK — scene matches the description in its header comment")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
