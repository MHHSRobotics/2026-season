#!/usr/bin/env python3
"""Generate fuel-free copies of the field and robot models.

models/field.xml ships 408 fuel balls. They are expensive, and because MuJoCo's reset button
calls mj_resetData - which restores qpos from the model - there is no runtime way to remove
them that survives a reset. A separate model is the only clean answer.

Regenerate after editing models/field.xml or models/robot.xml:

    python make_nofuel_models.py

Writes models/field_nofuel.xml and models/robot_nofuel.xml. Run the sim against them with:

    python main.py --model models/robot_nofuel.xml
"""

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

MODELS = Path(__file__).parent / "models"

# Hopper ball placement in the chassis frame. Base layer rests on top of the highest hopper
# roller (z=0.077, r=0.0255) plus the ball radius and a little clearance.
#
# Balls are 0.15 m across and the roller bed only spans x=-0.121..0.107, so only two fit side
# by side. Anything tighter overlaps, and MuJoCo resolves the penetration by flinging them
# apart on the first step - which is what happens on every reset, since reset restores exactly
# these positions. Hence two per layer, stacked upward with clearance.
BALL_RADIUS = 0.075
HOPPER_Y = -0.03
HOPPER_Z = 0.077 + 0.0255 + BALL_RADIUS + 0.005
HOPPER_X = (-0.077, 0.077)     # 0.154 apart, just over one ball diameter
HOPPER_LAYER_DZ = 0.158
HOPPER_MAX_LAYERS = 4

BANNER = ("\n  GENERATED FILE - do not edit.\n"
          "  Produced from %s by make_nofuel_models.py.\n"
          "  Re-run that script after changing the source model.\n")


def parse_keeping_comments(path):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return ET.parse(path, parser=parser)


def strip_fuel(tree):
    """Drop every <body> whose name mentions fuel, plus the comments introducing them."""
    removed = 0
    for parent in tree.iter():
        children = list(parent)
        doomed = []
        for i, child in enumerate(children):
            if child.tag == "body" and "fuel" in (child.get("name") or ""):
                doomed.append(i)
        if not doomed:
            continue
        # Also drop a comment immediately preceding a run of fuel bodies, so the output does
        # not keep headings like "Blue depot fuel: 6x4 grid" over nothing.
        drop = set(doomed)
        for i in doomed:
            j = i - 1
            if j >= 0 and j not in drop and not isinstance(children[j].tag, str):
                drop.add(j)
        for i in sorted(drop, reverse=True):
            parent.remove(children[i])
            if children[i].tag == "body":
                removed += 1
    return removed


def chassis_start_pose(robot_tree):
    """Where the chassis body starts, so preloaded balls can be placed in world coords."""
    for body in robot_tree.iter("body"):
        if body.get("name") == "chassis":
            if body.get("quat") or body.get("euler") or body.get("axisangle"):
                sys.exit("chassis has a start rotation; preload placement assumes identity")
            return [float(v) for v in body.get("pos").split()]
    sys.exit("no <body name=\"chassis\"> found in robot.xml")


def add_preload(field_tree, chassis_pos, count):
    """Add `count` fuel balls sitting on the hopper, in world coordinates.

    These have to live in the XML rather than being placed at runtime: MuJoCo's reset button
    calls mj_resetData, which restores qpos from the model, so anything positioned after load
    is lost on the first reset.

    Named blue_fuel_N so simulator.py's existing index caching picks them up as ball 0..N-1.
    """
    worldbody = field_tree.getroot().find("worldbody")
    if worldbody is None:
        sys.exit("field.xml has no <worldbody>")

    worldbody.append(ET.Comment(" Preloaded fuel, stacked on the hopper rollers "))

    locals_ = []
    for i in range(count):
        layer, slot = divmod(i, len(HOPPER_X))
        if layer >= HOPPER_MAX_LAYERS:
            print("  hopper full at %d balls" % len(locals_))
            break
        locals_.append((HOPPER_X[slot], HOPPER_Y, HOPPER_Z + HOPPER_LAYER_DZ * layer))

    # Overlapping spheres get resolved explosively on the first step, and since these
    # positions are what reset restores, that would happen on every reset.
    for i in range(len(locals_)):
        for j in range(i + 1, len(locals_)):
            gap = sum((a - b) ** 2 for a, b in zip(locals_[i], locals_[j])) ** 0.5
            if gap < 2 * BALL_RADIUS:
                sys.exit("preload balls %d and %d are %.3f m apart, need %.3f - they would be "
                         "flung apart on load" % (i, j, gap, 2 * BALL_RADIUS))

    placed = 0
    for i, (lx, ly, lz) in enumerate(locals_):
        pos = "%.4f %.4f %.4f" % (chassis_pos[0] + lx, chassis_pos[1] + ly, chassis_pos[2] + lz)
        body = ET.SubElement(worldbody, "body", {"name": "blue_fuel_%d" % i, "pos": pos})
        ET.SubElement(body, "freejoint", {"name": "blue_fuel_%d_jnt" % i})
        ET.SubElement(body, "geom", {
            "name": "blue_fuel_%d_geom" % i, "type": "sphere", "size": "0.0750",
            "mass": "0.2154", "material": "fuel_mat", "class": "fuel",
            "contype": "1", "conaffinity": "3",
        })
        placed += 1
    return placed


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--preload", type=int, default=0, metavar="N",
                        help="bake N fuel balls onto the hopper so they survive reset")
    args = parser.parse_args()

    field_src = MODELS / "field.xml"
    robot_src = MODELS / "robot.xml"
    for p in (field_src, robot_src):
        if not p.exists():
            sys.exit("missing %s" % p)

    # --- field ---------------------------------------------------------
    field = parse_keeping_comments(field_src)
    removed = strip_fuel(field)
    if args.preload > 0:
        chassis_pos = chassis_start_pose(parse_keeping_comments(robot_src))
        placed = add_preload(field, chassis_pos, args.preload)
        print("preloaded %d fuel ball(s) on the hopper at chassis start %s"
              % (placed, chassis_pos))
    field.getroot().insert(0, ET.Comment(BANNER % "field.xml"))
    field_out = MODELS / "field_nofuel.xml"
    field.write(field_out, encoding="unicode", xml_declaration=False)
    print("field_nofuel.xml: removed %d fuel bodies" % removed)

    # --- robot ---------------------------------------------------------
    robot = parse_keeping_comments(robot_src)
    swapped = 0
    for parent in robot.iter():
        for child in parent:
            if child.tag == "include" and child.get("file") == "field.xml":
                child.set("file", "field_nofuel.xml")
                swapped += 1
    if swapped != 1:
        sys.exit("expected exactly one <include file=\"field.xml\"/> in robot.xml, found %d"
                 % swapped)
    robot.getroot().set("model", "frc_swerve_robot_nofuel")
    robot.getroot().insert(0, ET.Comment(BANNER % "robot.xml"))
    robot_out = MODELS / "robot_nofuel.xml"
    robot.write(robot_out, encoding="unicode", xml_declaration=False)
    print("robot_nofuel.xml: include swapped to field_nofuel.xml")

    # --- verify --------------------------------------------------------
    fuel_bodies = [b.get("name") for b in ET.fromstring(field_out.read_text()).iter("body")
                   if "fuel" in (b.get("name") or "")]
    expected = args.preload if args.preload > 0 else 0
    if len(fuel_bodies) > expected:
        sys.exit("FAILED: %d fuel bodies present, expected at most %d"
                 % (len(fuel_bodies), expected))
    print("verified: %d fuel body/bodies remain%s"
          % (len(fuel_bodies), " (the preloads)" if fuel_bodies else ""))


if __name__ == "__main__":
    main()
