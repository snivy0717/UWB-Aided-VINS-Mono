#!/usr/bin/env python

from __future__ import print_function

import argparse
import math

import rosbag
from vins_estimator.msg import UWBRange


def parse_args():
    parser = argparse.ArgumentParser(
        description='Convert the official UVINS dataset bag to the topics and '
                    'UWBRange message format used by this VINS-Mono-UWB project.')
    parser.add_argument('--input_bag',
                        required=True,
                        help='Input UVINS rosbag path.')
    parser.add_argument('--output_bag',
                        required=True,
                        help='Output rosbag path.')
    parser.add_argument('--image_topic_in',
                        default='/camera/infra1/image_rect_raw',
                        help='Input image topic. (default: %(default)s)')
    parser.add_argument('--image_topic_out',
                        default='/cam0/image_raw',
                        help='Output image topic. (default: %(default)s)')
    parser.add_argument('--imu_topic_in',
                        default='/mavros/imu/data_raw',
                        help='Input IMU topic. (default: %(default)s)')
    parser.add_argument('--imu_topic_out',
                        default='/imu0',
                        help='Output IMU topic. (default: %(default)s)')
    parser.add_argument('--uwb_topic_in',
                        default='/UWB_module/distance',
                        help='Input UVINS UWB topic. (default: %(default)s)')
    parser.add_argument('--uwb_topic_out',
                        default='/uwb/range',
                        help='Output UWBRange topic. (default: %(default)s)')
    parser.add_argument('--copy_groundtruth',
                        action='store_true',
                        help='Copy the groundtruth pose topic to the output bag.')
    parser.add_argument('--groundtruth_topic_in',
                        default='/vrpn_client_node/XY_Bigbaby/pose',
                        help='Input groundtruth pose topic. (default: %(default)s)')
    parser.add_argument('--groundtruth_topic_out',
                        default='/ground_truth/pose',
                        help='Output groundtruth pose topic. (default: %(default)s)')
    parser.add_argument('--uwb_frame_id',
                        default='uwb',
                        help='frame_id for generated UWBRange messages. (default: %(default)s)')
    return parser.parse_args()


def is_valid_range(distance):
    try:
        distance = float(distance)
    except (TypeError, ValueError):
        return False
    return distance > 0.0 and distance == distance and not math.isinf(distance)


def create_uwb_range(stamp, frame_id, anchor_id, distance):
    msg = UWBRange()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.anchor_id = int(anchor_id)
    msg.range = float(distance)
    return msg


def convert_bag(args):
    topics = [args.image_topic_in, args.imu_topic_in, args.uwb_topic_in]
    if args.copy_groundtruth:
        topics.append(args.groundtruth_topic_in)

    image_count = 0
    imu_count = 0
    uwb_raw_count = 0
    uwb_range_count = 0
    groundtruth_count = 0
    skipped_invalid_uwb_count = 0

    with rosbag.Bag(args.input_bag, 'r') as in_bag:
        with rosbag.Bag(args.output_bag, 'w') as out_bag:
            for topic, msg, t in in_bag.read_messages(topics=topics):
                if topic == args.image_topic_in:
                    out_bag.write(args.image_topic_out, msg, t)
                    image_count += 1

                elif topic == args.imu_topic_in:
                    out_bag.write(args.imu_topic_out, msg, t)
                    imu_count += 1

                elif topic == args.uwb_topic_in:
                    uwb_raw_count += 1
                    stamp = msg.stamp
                    ranges = (msg.D0, msg.D1, msg.D2)

                    for anchor_id, distance in enumerate(ranges):
                        if not is_valid_range(distance):
                            skipped_invalid_uwb_count += 1
                            continue

                        uwb_msg = create_uwb_range(
                            stamp, args.uwb_frame_id, anchor_id, distance)
                        out_bag.write(args.uwb_topic_out, uwb_msg, stamp)
                        uwb_range_count += 1

                elif args.copy_groundtruth and topic == args.groundtruth_topic_in:
                    out_bag.write(args.groundtruth_topic_out, msg, t)
                    groundtruth_count += 1

    print('image written: {}'.format(image_count))
    print('imu written: {}'.format(imu_count))
    print('uwb raw messages: {}'.format(uwb_raw_count))
    print('uwb ranges written: {}'.format(uwb_range_count))
    print('groundtruth written: {}'.format(groundtruth_count))
    print('skipped invalid uwb ranges: {}'.format(skipped_invalid_uwb_count))
    print('output bag: {}'.format(args.output_bag))


def main():
    convert_bag(parse_args())


if __name__ == '__main__':
    main()
