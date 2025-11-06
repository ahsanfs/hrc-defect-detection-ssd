#!/usr/bin/env python3
import rosbag
import csv
import os

bag_file = "/home/hucenrotia/spain_taiwan_ws/data_extraction_bag/bagfile/New_Experiment/CS_0.95_1.bag"
start_time = 1761668785.671794928
topic = "/current_states_GUI"
output_csv = "summary_across_bags_new_experiment.csv"

NUM_SLOTS = 12
HUMAN_DONE = [1, 3]
ROBOT_DONE = [2, 4, 5]

slot_prev_state = [None] * NUM_SLOTS
slot_prev_stamp = [None] * NUM_SLOTS

human_start = [None] * NUM_SLOTS
human_end   = [None] * NUM_SLOTS
human_dur   = [0.0] * NUM_SLOTS

robot_start = [None] * NUM_SLOTS
robot_end   = [None] * NUM_SLOTS
robot_dur   = [0.0] * NUM_SLOTS

doubtful_start = [None] * NUM_SLOTS
doubtful_end   = [None] * NUM_SLOTS
doubtful_dur   = [0.0] * NUM_SLOTS
doubtful_count = 0

robot_first_start = None
completion_time = None

print(f"Scanning bag: {bag_file}")

with rosbag.Bag(bag_file, 'r') as bag:
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        current_stamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
        current_id = list(msg.id)

        if slot_prev_state[0] is None:
            for i in range(NUM_SLOTS):
                slot_prev_state[i] = current_id[i]
                slot_prev_stamp[i] = current_stamp
            continue

        if any(x != 0 for x in slot_prev_state) and all(x == 0 for x in current_id):
            completion_time = current_stamp
            print(f"\n[COMPLETION DETECTED] at {completion_time:.9f}")
            break

        for i in range(NUM_SLOTS):
            prev = slot_prev_state[i]
            curr = current_id[i]

            if curr != prev:
                print(f"[SLOT {i+1}] {prev} → {curr} at {current_stamp:.9f}")

                # Human normal timing
                if prev == 0 and curr == 7:
                    human_start[i] = current_stamp

                elif prev == 7 and curr in HUMAN_DONE:
                    human_end[i] = current_stamp
                    if human_start[i] is not None:
                        human_dur[i] = human_end[i] - human_start[i]
                        print(f"[HUMAN END] Slot {i+1} at {human_end[i]:.9f} | Duration: {human_dur[i]:.9f}s")

                    if doubtful_start[i] is not None:
                        doubtful_end[i] = current_stamp
                        doubtful_dur[i] = doubtful_end[i] - doubtful_start[i]
                        doubtful_count += 1
                        human_dur[i] += doubtful_dur[i]
                        print(f"[DOUBTFUL CASE] Slot {i+1} | Duration: {doubtful_dur[i]:.9f}s")

                # Robot inspection start
                elif prev == 0 and curr in ROBOT_DONE:
                    robot_start[i] = slot_prev_stamp[i]
                    robot_end[i] = current_stamp
                    robot_dur[i] = robot_end[i] - robot_start[i]
                    if robot_first_start is None:
                        robot_first_start = robot_start[i]
                    print(f"[ROBOT END] Slot {i+1} at {robot_end[i]:.9f} | Duration: {robot_dur[i]:.9f}s")

                # Robot help (doubtful) → human needed
                elif prev == 5 and curr == 7:
                    doubtful_start[i] = current_stamp

                slot_prev_state[i] = curr
                slot_prev_stamp[i] = current_stamp

# === Final Summary ===
bag_name = os.path.basename(bag_file)
completion_duration = completion_time - start_time if completion_time else "N/A"
human_total_time = sum(human_dur)
robot_total_time = sum(robot_dur)
human_workload = sum(1 for t in human_dur if t > 0)
robot_workload = sum(1 for t in robot_dur if t > 0)

# === Print Summary ===
print("\n=== FINAL SUMMARY ===")
print(f"Bag: {bag_name}")
print(f"Completion time: {completion_duration}")
print(f"Human workload: {human_workload}")
print(f"Robot workload: {robot_workload}")
print(f"Human total time: {human_total_time:.9f} s")
print(f"Robot total time: {robot_total_time:.9f} s")
print(f"Number of doubtful cases (robot state 5 → 7 → 1/3): {doubtful_count}")

# === Write CSV ===
header = [
    "name", "completion_time", "human_workload", "robot_workload",
    "human_total_time", "robot_total_time", "doubtful_count"
]
row = [
    bag_name, completion_duration, human_workload, robot_workload,
    human_total_time, robot_total_time, doubtful_count
]

for i in range(NUM_SLOTS):
    header += [f"SSD_{i+1}_human_start", f"SSD_{i+1}_human_end", f"SSD_{i+1}_human_duration"]
    row += [human_start[i] or 0.0, human_end[i] or 0.0, human_dur[i]]

for i in range(NUM_SLOTS):
    header += [f"SSD_{i+1}_robot_start", f"SSD_{i+1}_robot_end", f"SSD_{i+1}_robot_duration"]
    row += [robot_start[i] or 0.0, robot_end[i] or 0.0, robot_dur[i]]

write_header = not os.path.exists(output_csv)
with open(output_csv, 'a', newline='') as f:
    writer = csv.writer(f)
    if write_header:
        writer.writerow(header)
    writer.writerow(row)
