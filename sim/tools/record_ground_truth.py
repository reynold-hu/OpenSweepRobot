#!/usr/bin/env python3
"""Write the robot's true pose to a CSV that coverage_metrics.py can score.

    python3 sim/tools/record_ground_truth.py run1.csv

Subscribes to the ground-truth odometry published by the p3d plugin in
../description/urdf/plugins_gazebo_classic.xacro and writes `t,x,y,yaw` at a
fixed rate. Stop it with Ctrl-C; the file is flushed as it goes, so an
interrupted run still yields a usable trace.

Ground truth rather than wheel odometry on purpose. Wheel odometry drifts with
slip, and scoring a run against its own drift would quietly reward whatever the
SLAM stack did that day. The benchmark asks where the robot really went, not
where it thought it went.

Nothing here has been run against a live simulator.
"""

import argparse
import math
import sys
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
except ImportError:
    sys.exit("needs rclpy and nav_msgs — source your ROS 2 install first")


class GroundTruthRecorder(Node):
    def __init__(self, path: Path, topic: str, rate: float):
        super().__init__("ground_truth_recorder")
        self.fh = path.open("w", encoding="utf-8")
        self.fh.write("t,x,y,yaw\n")
        self.rate = rate
        self.last = None
        self.count = 0

        self.create_subscription(Odometry, topic, self.on_odom, 10)
        self.create_timer(1.0 / rate, self.write_sample)
        self.get_logger().info(f"recording {topic} -> {path} at {rate} Hz")

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # yaw from the quaternion, assuming the robot stays level. It does.
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last = (t, p.x, p.y, yaw)

    def write_sample(self) -> None:
        if self.last is None:
            return
        t, x, y, yaw = self.last
        self.fh.write(f"{t:.6f},{x:.6f},{y:.6f},{yaw:.6f}\n")
        self.count += 1
        if self.count % int(self.rate * 5) == 0:
            self.fh.flush()

    def close(self) -> None:
        self.fh.flush()
        self.fh.close()
        self.get_logger().info(f"wrote {self.count} samples")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("output", type=Path)
    ap.add_argument("--topic", default="/ground_truth/odom",
                    help="the p3d plugin's topic; confirm with `ros2 topic list`")
    ap.add_argument("--rate", type=float, default=30.0, help="samples per second")
    args = ap.parse_args()

    rclpy.init()
    node = GroundTruthRecorder(args.output, args.topic, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
