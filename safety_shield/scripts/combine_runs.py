"""Read in all ablation study runs and combine them to one file."""
import pandas as pd
import glob
import os
import csv
import numpy as np


PANDA = False


def read_last_line(filename):
    with open(filename, 'rb') as file:
        file.seek(-2, os.SEEK_END)  # Go to the second last byte before EOF
        while file.read(1) != b'\n':  # Keep moving backwards until you find a newline
            file.seek(-2, os.SEEK_CUR)
        last_line = file.readline().decode()  # Read and decode the last line
    return last_line


def read_first_line(filename):
    try:
        with open(filename, mode='r') as file:
            reader = csv.reader(file)
            for row in reader:
                # Return the first row
                return row
    except Exception as e:
        print(f"Error reading {filename}: {e}")
        return None


# Define the pattern to match the CSV files in the specified directory
if not PANDA:
    path_s_csv_files = glob.glob('./data/ablation_results/vicon_p_*_mocap_*_path_s.csv')
else:
    path_s_csv_files = glob.glob('./data/ablation_results_panda/vicon_p_*_mocap_*_path_s.csv')

# Prepare an empty list to store data from each file
data_list = []

# Loop through all matching CSV files
for file in path_s_csv_files:
    filename = os.path.basename(file)
    file_normal = file.replace('_path_s.csv', '.csv')
    parts = filename.split('_')

    n = parts[2]
    task = parts[3]
    mode = parts[5]
    eef = parts[6]
    config = "".join(parts[7:-2]).replace('.csv', '')
    # Read the current CSV file into a DataFrame
    # df = pd.read_csv(file, on_bad_lines='warn')
    first_line = read_first_line(file)
    if first_line is None:
        print(f"Skipping {file}")
        continue
    last_path_s = float(first_line[0])
    method = f'{mode}_{config}'

    # Read in all calculation times (col: calc_time) 
    calc_times = pd.read_csv(file_normal, usecols=[' calc_time'], on_bad_lines='skip').values
    data_list.append([method, n, task, mode, eef, config, last_path_s, np.median(calc_times)])

# Create a DataFrame from the compiled data list
columns = ['method', 'user', 'task', 'mode', 'eef', 'config', 'path_s', "step_time"]
result_table = pd.DataFrame(data_list, columns=columns)

## Calculate percentage of OFF method.
users = result_table['user'].unique()
tasks = result_table['task'].unique()

off_results = {}

for user in users:
    for task in tasks:
        user_task_off_row = result_table[(result_table['user'] == user) & (result_table['task'] == task) & (result_table['mode'] == 'OFF')]
        off_results[f"{user}_{task}"] = user_task_off_row['path_s'].values[0]

# Add new percentage column to df. Init with zero.
result_table['percentage'] = np.zeros(len(result_table))
for index, row in result_table.iterrows():
    off_value = off_results[f"{row['user']}_{row['task']}"]
    result_table.at[index, 'percentage'] = row['path_s'] / off_value * 100

# Display the result table
print(result_table)
if not PANDA:
    result_table.to_csv('./data/ablation_results/all_results.csv', index=False)
else:
    result_table.to_csv('./data/ablation_results_panda/all_results.csv', index=False)
