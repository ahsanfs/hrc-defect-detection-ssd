#!/usr/bin/env python3
import rosbag
from kinova_msgs.msg import SSDStates  # or correct message type for /current_states_GUI
import numpy as np

# === Set variables here ===
bag_file = "bagfile/HRC/TR2_3_HRC.bag"
start_time = 1752738353.157977773  # use precise goal accepted time

# === Topic want to monitor for task completion ===
target_topic = "/current_states_GUI"

end_time = None
found_transition = False
previous_id = None

print(f"Scanning {target_topic} to detect task completion...")

with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[target_topic]):
        current_id = list(msg.id)
        current_stamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9

        if previous_id is not None:
            if any(x != 0 for x in previous_id) and all(x == 0 for x in current_id):
                end_time = current_stamp
                found_transition = True
                print(f"Completion detected at: {current_stamp:.9f}")
                break

        previous_id = current_id

if found_transition:
    duration = end_time - start_time
    print(f"\nTask completion time: {duration:.2f} seconds")
else:
    print("Task completion not detected in this bag.")
