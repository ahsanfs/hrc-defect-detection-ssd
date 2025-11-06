import pandas as pd
import os

# Load CSV
csv_path = "summary_across_bags.csv"
df = pd.read_csv(csv_path)

# Constants
NUM_SLOTS = 12

# Process each row
for index, row in df.iterrows():
    last_robot_end = 0.0
    total_robot_time = 0.0

    for i in range(NUM_SLOTS):
        start_col = f'SSD_{i+1}_robot_start'
        end_col = f'SSD_{i+1}_robot_end'
        dur_col = f'SSD_{i+1}_robot_duration'

        start_time = row[start_col]
        end_time = row[end_col]

        if end_time > 0 and start_time > 0:
            # Duration from initial robot start to end
            full_duration = end_time - start_time

            # Duration relative to previous robot end
            slot_duration = end_time - last_robot_end if last_robot_end > 0 else full_duration

            # Update the duration
            df.at[index, dur_col] = slot_duration

            # Update total
            total_robot_time += slot_duration
            last_robot_end = end_time
        else:
            df.at[index, dur_col] = 0.0

    # Update the total robot time
    df.at[index, 'robot_total_time'] = total_robot_time

# Save revised CSV
revised_path = "summary_across_bags_revised_multirow_70_90_95.csv"
df.to_csv(revised_path, index=False)