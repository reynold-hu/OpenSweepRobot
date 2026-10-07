#!/usr/bin/env python3
"""Static checks on the vendored robot description and the Gazebo Classic port.

This exists because the port was authored without a simulator to run it in.
Nothing here proves the plugins will load — only a real launch does that. What it
does catch is the class of mistake that a first run would otherwise blame on
Gazebo: an xacro property that is referenced but never defined, a joint or link
named in a plugin that the URDF does not have, an include that points nowhere,
and a broken XML file.

It also prints every plugin filename and topic the port claims to produce, so
the first launch has a checklist to compare `ros2 topic list` against.

    python3 sim/tools/check_description.py
"""

from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

URDF_DIR = Path(__file__).resolve().parent.parent / "description" / "urdf"

# Both spellings are in active use and xacro accepts either; these vendored files
# use one for inertial.xacro and the other for the rest, so matching on a single
# URI silently finds no macros at all.
XACRO_NS = {
    "http://ros.org/wiki/xacro",
    "http://www.ros.org/wiki/xacro",
}

# Available inside every xacro expression without being declared.
BUILTINS = {
    "pi", "true", "false", "True", "False", "None",
    "radians", "degrees", "min", "max", "abs", "int", "float", "str", "len",
    "sin", "cos", "tan", "sqrt", "atan2",
}

IDENT = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
EXPR = re.compile(r"\$\{([^{}]*(?:\{[^{}]*\}[^{}]*)*)\}")
ARG_SUB = re.compile(r"\$\([^)]*\)")
STR_LIT = re.compile(r"'[^']*'|\"[^\"]*\"")


def xacro_elems(root: ET.Element, name: str) -> list[ET.Element]:
    """All xacro-namespaced <name> elements under root, under either URI."""
    out = []
    for el in root.iter():
        if not el.tag.startswith("{"):
            continue
        uri, _, local = el.tag[1:].partition("}")
        if local == name and uri in XACRO_NS:
            out.append(el)
    return out


def strip_args(expr: str) -> str:
    """Remove the parts of an expression that are not property references.

    $(arg name) substitutions are checked separately, and string literals must go
    because a comparison like ${'$(arg odom_source)' == 'robot_wheels'} otherwise
    leaves `robot_wheels` looking like an identifier.
    """
    return STR_LIT.sub(" ", ARG_SUB.sub(" ", expr))


def expressions(text: str) -> list[str]:
    return [strip_args(m) for m in EXPR.findall(text or "")]


def collect_expressions(tree: ET.ElementTree) -> list[tuple[str, str]]:
    """Every ${...} in the tree as (expr, where) pairs."""
    out: list[tuple[str, str]] = []

    def walk(el: ET.Element, where: str):
        here = f"{where}/{el.tag.split('}')[-1]}"
        for value in el.attrib.values():
            for e in expressions(value):
                out.append((e, here))
        if el.text:
            for e in expressions(el.text):
                out.append((e, here))
        for child in el:
            walk(child, here)

    for child in tree.getroot():
        walk(child, tree.getroot().tag)
    return out


def load(path: Path) -> ET.ElementTree:
    try:
        return ET.parse(path)
    except ET.ParseError as exc:
        print(f"FAIL  {path.name}: {exc}")
        sys.exit(1)


def main() -> int:
    problems: list[str] = []

    required = ["robot.urdf.xacro", "params.xacro", "inertial.xacro",
                "materials.xacro", "plugins_gazebo_classic.xacro"]
    for name in required:
        if not (URDF_DIR / name).exists():
            problems.append(f"missing file: {name}")
    if problems:
        for p in problems:
            print(f"FAIL  {p}")
        return 1

    files = {n: load(URDF_DIR / n) for n in required}
    print(f"parsed {len(files)} xacro files from {URDF_DIR.name}/\n")

    # ---- everything that can legally appear in a ${...} --------------------
    # Properties are collected from every file, not just params.xacro: several
    # are declared inside macros, where they are in scope for that macro's body.
    props: set[str] = set()
    args: set[str] = set()
    macro_params: set[str] = set()
    for tree in files.values():
        root = tree.getroot()
        for el in xacro_elems(root, "property"):
            if el.get("name"):
                props.add(el.get("name"))
        for el in xacro_elems(root, "arg"):
            if el.get("name"):
                args.add(el.get("name"))
        for el in xacro_elems(root, "macro"):
            # params="prefix n lump_base i:=0" — a name is everything before :=
            for tok in (el.get("params") or "").replace(",", " ").split():
                name = tok.split(":=")[0].strip()
                if name:
                    macro_params.add(name)

    print(f"defined: {len(props)} properties, {len(args)} args, "
          f"{len(macro_params)} macro parameters")

    # ---- links and joints -------------------------------------------------
    urdf = files["robot.urdf.xacro"].getroot()
    links = {el.get("name") for el in urdf.iter("link")}
    joints = {el.get("name") for el in urdf.iter("joint")}
    links.discard(None)
    joints.discard(None)
    print(f"defined: {len(links)} links, {len(joints)} joints\n")

    # ---- every ${...} must resolve ----------------------------------------
    print("expression check")
    unresolved = 0
    for name, tree in files.items():
        for expr, where in collect_expressions(tree):
            for ident in set(IDENT.findall(expr)):
                if (ident in BUILTINS or ident in props
                        or ident in args or ident in macro_params):
                    continue
                problems.append(f"{name}{where}: '${ident}' is not a defined property")
                unresolved += 1
    print(f"  {'all resolve' if not unresolved else str(unresolved) + ' unresolved'}\n")

    # ---- names referenced by the plugin file ------------------------------
    print("reference check")
    plug = files["plugins_gazebo_classic.xacro"].getroot()
    bad = 0
    for el in plug.iter():
        tag = el.tag.split("}")[-1]
        if tag == "gazebo" and "reference" in el.attrib:
            ref = el.get("reference")
            if ref not in links:
                problems.append(f"plugins: <gazebo reference=\"{ref}\"> is not a link")
                bad += 1
        if tag == "joint_name":
            jn = (el.text or "").strip()
            if jn not in joints:
                problems.append(f"plugins: joint_name {jn!r} is not a joint")
                bad += 1
    print(f"  {'every link and joint named exists' if not bad else str(bad) + ' bad references'}\n")

    # ---- includes ---------------------------------------------------------
    print("include check")
    seen = set()
    for name, tree in files.items():
        for el in xacro_elems(tree.getroot(), "include"):
            target = el.get("filename")
            ok = target and (URDF_DIR / target).exists()
            seen.add(target)
            if not ok:
                problems.append(f"{name}: include {target!r} not found")
    for t in sorted(x for x in seen if x):
        print(f"  {t}")
    print()

    # ---- what the first launch must confirm -------------------------------
    print("claims to verify on first launch")
    print("  (nothing below is proven; compare against `ros2 topic list`)")
    for el in plug.iter("plugin"):
        fn = el.get("filename")
        if fn:
            print(f"    {el.get('name', '?'):<20} {fn}")
    for el in plug.iter("sensor"):
        print(f"    sensor {el.get('name', '?'):<14} type={el.get('type')}")
    print()

    if problems:
        print("FAIL")
        for p in sorted(set(problems)):
            print(f"  - {p}")
        return 1
    print("OK — every property resolves, every named link and joint exists")
    print("     (this says nothing about whether Gazebo will load the plugins)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
