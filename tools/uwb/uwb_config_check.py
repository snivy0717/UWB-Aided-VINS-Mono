#!/usr/bin/env python3

import argparse


DEFAULT_ANCHORS = [
    [0.0, 0.0, 0.0],
    [5.0, 0.0, 0.0],
    [0.0, 5.0, 0.0],
]


def parse_args():
    parser = argparse.ArgumentParser(description="Check UWB factor configuration before enabling backend fusion.")
    parser.add_argument("--anchors", type=float, nargs="+", required=True,
                        help="Anchor coordinates as x1 y1 z1 x2 y2 z2 ...")
    parser.add_argument("--p_uwb_imu", type=float, nargs=3, required=True,
                        metavar=("X", "Y", "Z"))
    parser.add_argument("--uwb_world_aligned", type=int, choices=[0, 1], required=True)
    parser.add_argument("--use_uwb_factor", type=int, choices=[0, 1], required=True)
    return parser.parse_args()


def group_anchors(values):
    if len(values) % 3 != 0:
        raise ValueError("--anchors length must be a multiple of 3")
    return [values[i:i + 3] for i in range(0, len(values), 3)]


def is_default_example(anchors):
    if len(anchors) != len(DEFAULT_ANCHORS):
        return False
    for anchor, default_anchor in zip(anchors, DEFAULT_ANCHORS):
        for value, default_value in zip(anchor, default_anchor):
            if abs(value - default_value) > 1e-9:
                return False
    return True


def main():
    args = parse_args()
    anchors = group_anchors(args.anchors)

    print("anchor_count: {}".format(len(anchors)))
    for index, anchor in enumerate(anchors):
        print("anchor_{}: [{:.6f}, {:.6f}, {:.6f}]".format(index, *anchor))
    print("p_uwb_imu: [{:.6f}, {:.6f}, {:.6f}]".format(*args.p_uwb_imu))
    print("use_uwb_factor: {}".format(args.use_uwb_factor))
    print("uwb_world_aligned: {}".format(args.uwb_world_aligned))

    ready = True
    if len(anchors) < 3:
        ready = False
        print("[WARN] Fewer than 3 anchors. UWB factor geometry may be weak or unobservable.")
    if is_default_example(anchors):
        ready = False
        print("[WARN] Anchors match the default example coordinates. Replace them with measured anchor positions.")
    if args.use_uwb_factor and not args.uwb_world_aligned:
        ready = False
        print("[ERROR] use_uwb_factor=1 but uwb_world_aligned=0. Do not enable UWB factor before coordinate alignment.")
    if not args.uwb_world_aligned:
        ready = False
        print("[WARN] UWB anchor world and VINS world are not marked as aligned.")

    if ready:
        print("status: configuration is suitable for considering UWB factor enablement.")
    else:
        print("status: configuration is not ready for enabling UWB factor.")


if __name__ == "__main__":
    main()
