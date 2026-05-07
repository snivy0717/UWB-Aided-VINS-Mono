#!/usr/bin/env python3

import argparse
import math


def parse_args():
    parser = argparse.ArgumentParser(
        description="Check the UWB range residual formula independently from VINS-Mono optimization.")
    parser.add_argument("--pose_px", type=float, required=True)
    parser.add_argument("--pose_py", type=float, required=True)
    parser.add_argument("--pose_pz", type=float, required=True)
    parser.add_argument("--pose_qx", type=float, required=True)
    parser.add_argument("--pose_qy", type=float, required=True)
    parser.add_argument("--pose_qz", type=float, required=True)
    parser.add_argument("--pose_qw", type=float, required=True)
    parser.add_argument("--p_uwb_imu", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--anchor", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--range", dest="measured_range", type=float, required=True)
    parser.add_argument("--sigma", type=float, required=True)
    return parser.parse_args()


def normalize_quaternion(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 0.0:
        raise ValueError("quaternion norm must be positive")
    return [v / norm for v in q]


def rotate_vector(q_xyzw, v):
    qx, qy, qz, qw = normalize_quaternion(q_xyzw)
    x, y, z = v

    # R(q) * v, using the same qx,qy,qz,qw order as VINS-Mono para_Pose.
    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r01 = 2.0 * (qx * qy - qz * qw)
    r02 = 2.0 * (qx * qz + qy * qw)
    r10 = 2.0 * (qx * qy + qz * qw)
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
    r12 = 2.0 * (qy * qz - qx * qw)
    r20 = 2.0 * (qx * qz - qy * qw)
    r21 = 2.0 * (qy * qz + qx * qw)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

    return [
        r00 * x + r01 * y + r02 * z,
        r10 * x + r11 * y + r12 * z,
        r20 * x + r21 * y + r22 * z,
    ]


def main():
    args = parse_args()
    sigma = args.sigma if args.sigma > 0.0 else 0.1

    pose_p = [args.pose_px, args.pose_py, args.pose_pz]
    pose_q = [args.pose_qx, args.pose_qy, args.pose_qz, args.pose_qw]
    rotated_tag_offset = rotate_vector(pose_q, args.p_uwb_imu)
    tag_world = [pose_p[i] + rotated_tag_offset[i] for i in range(3)]

    diff = [tag_world[i] - args.anchor[i] for i in range(3)]
    predicted_range = math.sqrt(sum(v * v for v in diff))
    raw_residual = predicted_range - args.measured_range
    whitened_residual = raw_residual / sigma

    print("uwb_tag_world_position: [{:.9f}, {:.9f}, {:.9f}]".format(*tag_world))
    print("predicted_range: {:.9f}".format(predicted_range))
    print("measured_range: {:.9f}".format(args.measured_range))
    print("raw_residual: {:.9f}".format(raw_residual))
    print("whitened_residual: {:.9f}".format(whitened_residual))


if __name__ == "__main__":
    main()
