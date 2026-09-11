#!/usr/bin/env python3
"""Throw trigger - sends upward velocity to x500 model via gz-transport."""
import sys
import time

try:
    from gz.transport13 import Node
    from gz.msgs10 import pose_pb2
    from gz.msgs10 import twista_pb2
except ImportError:
    # Try alternative import paths
    from gz.transport import Node
    from gz.msgs import pose_pb2
    from gz.msgs import twista_pb2

WORLD = "default"
MODEL = "x500"
VELOCITY = 5.0  # m/s upward

def main():
    node = Node()

    topic = f"/world/{WORLD}/model/{MODEL}/velocity_command"

    # Create a Twist message (linear velocity only, z-axis up)
    from gz.msgs10 import twist_pb2
    msg = twist_pb2.Twist()
    msg.linear.z = VELOCITY

    # Publish
    pub = node.advertise(topic, type(msg))
    if not pub.valid():
        print(f"[throw] Failed to advertise on {topic}")
        sys.exit(1)

    print(f"[throw] Publishing {VELOCITY} m/s upward on {topic}")
    for _ in range(10):
        pub.publish(msg)
        time.sleep(0.05)

    print(f"[throw] Done")

if __name__ == "__main__":
    main()
