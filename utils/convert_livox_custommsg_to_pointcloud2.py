#!/usr/bin/env python3
"""Convert livox_ros_driver2/msg/CustomMsg to sensor_msgs/msg/PointCloud2 in a rosbag.

Rewrites one topic of a rosbag from Livox's own CustomMsg format into the
PointCloud2 layout RESPLE's `Mid360` lidar_type expects (fields x, y, z,
intensity, tag, line, timestamp[float64, absolute epoch ns] — see
livox_mid360::Point in resple/include/utils/common_utils.h). Use this to play
a bag recorded with a LivoxCustomMsg-configured driver against a Mid360-style
config (e.g. config_mid360.yaml) without re-recording.

All other topics in the bag (IMU, etc.) are copied through unchanged.

Requires the workspace to be built and sourced first (`source install/setup.bash`),
so livox_ros_driver2's Python message bindings are importable.

Usage:
    python3 convert_livox_custommsg_to_pointcloud2.py \\
        --input /path/to/bag_in --output /path/to/bag_out \\
        [--topic /livox/lidar] [--output-topic /livox/points]
"""

import argparse
import struct

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg

POINT_STEP = 32  # x,y,z,intensity (f32 x4) + tag,line (u8 x2) + pad(6) + timestamp (f64)

FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='tag', offset=16, datatype=PointField.UINT8, count=1),
    PointField(name='line', offset=17, datatype=PointField.UINT8, count=1),
    PointField(name='timestamp', offset=24, datatype=PointField.FLOAT64, count=1),
]


def custommsg_to_pointcloud2(msg: CustomMsg) -> PointCloud2:
    n = len(msg.points)
    buf = bytearray(POINT_STEP * n)
    for i, p in enumerate(msg.points):
        base = i * POINT_STEP
        timestamp_ns = float(msg.timebase + p.offset_time)
        struct.pack_into('<4f', buf, base, p.x, p.y, p.z, float(p.reflectivity))
        struct.pack_into('<2B', buf, base + 16, p.tag, p.line)
        struct.pack_into('<d', buf, base + 24, timestamp_ns)

    cloud = PointCloud2()
    cloud.header = msg.header
    cloud.height = 1
    cloud.width = n
    cloud.fields = FIELDS
    cloud.is_bigendian = False
    cloud.point_step = POINT_STEP
    cloud.row_step = POINT_STEP * n
    cloud.is_dense = True
    cloud.data = bytes(buf)
    return cloud


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--input', required=True, help='Input bag directory')
    parser.add_argument('--output', required=True, help='Output bag directory (must not exist)')
    parser.add_argument('--topic', default='/livox/lidar', help='CustomMsg topic to convert (default: %(default)s)')
    parser.add_argument('--output-topic', default='/livox/lidar', help='PointCloud2 topic name to write (default: %(default)s)')
    parser.add_argument('--storage-id', default='sqlite3', help='rosbag2 storage plugin (default: sqlite3; use "mcap" for .mcap bags)')
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.input, storage_id=args.storage_id),
        rosbag2_py.ConverterOptions('', ''))

    topics_and_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.topic not in topics_and_types:
        raise SystemExit(f"Topic {args.topic!r} not found in {args.input}. Available: {sorted(topics_and_types)}")

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.output, storage_id=args.storage_id),
        rosbag2_py.ConverterOptions('', ''))
    for topic_id, (name, type_str) in enumerate(topics_and_types.items()):
        out_name = args.output_topic if name == args.topic else name
        out_type = 'sensor_msgs/msg/PointCloud2' if name == args.topic else type_str
        writer.create_topic(rosbag2_py.TopicMetadata(id=topic_id, name=out_name, type=out_type, serialization_format='cdr'))

    try:
        total = reader.get_metadata().message_count
    except Exception:
        total = 0

    n_converted = 0
    n_copied = 0
    n_total = 0
    last_pct = -1
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic == args.topic:
            custom_msg = deserialize_message(data, CustomMsg)
            cloud = custommsg_to_pointcloud2(custom_msg)
            writer.write(args.output_topic, serialize_message(cloud), t_ns)
            n_converted += 1
        else:
            writer.write(topic, data, t_ns)
            n_copied += 1

        n_total += 1
        if total:
            pct = n_total * 100 // total
            if pct != last_pct:
                print(f"\rProgress: {pct:3d}% ({n_total}/{total})", end='', flush=True)
                last_pct = pct
        elif n_total % 1000 == 0:
            print(f"\rProgress: {n_total} messages", end='', flush=True)
    print()

    print(f"Converted {n_converted} CustomMsg messages on {args.topic!r} -> {args.output_topic!r} (PointCloud2)")
    print(f"Copied {n_copied} messages on other topics unchanged")
    print(f"Wrote {args.output}")


if __name__ == '__main__':
    main()
