#!/usr/bin/env python3
import rosbag
from collections import deque

# === Configuration ===
bag_file = "bagfile/Human/TR2_3_Human.bag"
human_topic = "/update_states_box"
end_topic = "/current_states_GUI"

human_start_time = None
end_time = None
found_transition = False
previous_id = None

print("Scanning for human inspection start (state 7)...")
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[human_topic]):
        if 7 in msg.box:
            human_start_time = t.to_sec()
            print(f"Human inspection started at {human_start_time:.9f} (state 7 detected)")
            break

if not human_start_time:
    print("No state 7 found in /update_states_box — human inspection not detected.")
    exit()

print("Scanning for task completion in /current_states_GUI...")
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[end_topic]):
        current_id = list(msg.id)
        current_stamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9

        if previous_id is not None:
            if any(x != 0 for x in previous_id) and all(x == 0 for x in current_id):
                end_time = current_stamp
                found_transition = True
                print(f"Task completion detected at {end_time:.9f} (all IDs = 0)")
                break
        previous_id = current_id

if found_transition and human_start_time:
    duration = end_time - human_start_time
    print(f"\nHuman inspection time: {duration:.2f} seconds")
else:
    print("Could not compute duration (missing start or end time)")
