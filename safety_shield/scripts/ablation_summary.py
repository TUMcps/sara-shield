import pandas as pd
import glob
import os
import csv
import numpy as np
from scipy.stats import bootstrap


PAPER_FOLDER = '/home/jakob/Nextcloud/Promotion/Paper/General_Safety_Framework/Paper/results/ablation_study/'


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
path_s_csv_files = glob.glob('./data/ablation_results/vicon_p_*_mocap_*_path_s.csv')

# Prepare an empty list to store data from each file
data_list = []

# Method names
methods = {
  "OFF_humanreachTUMlab": "OFF",
  "PFL_humanreachTUMlab": "\shield{} (Ours)",
  "PFL_humanreachablationnocontactclass": "\shield{} without $c_{\mathrm{free}}$",
  "REDUCEDSPEED_humanreachTUMlab": "Reduced speed PFL \cite{iso_2021_RoboticsSafety}",
  "REFLECTEDMASS_humanreachTUMlab": "Reflected mass \cite{haddadin_2012_MakingRobots}",
  "SSM_humanreachTUMlab": "Dynamic SSM \cite{beckert_2017_OnlineVerification, thumm_2022_ProvablySafe}",
  "SWITCH_humanreachTUMlab": "Reduced speed zone \cite{svarny_2019_SafePhysical}",
  "ZONESSM_humanreachTUMlab": "SSM zone \cite{iso_2021_RoboticsSafety}",
}

sorting = [
    "ZONESSM_humanreachTUMlab",
    "REDUCEDSPEED_humanreachTUMlab",
    "SSM_humanreachTUMlab",
    "SWITCH_humanreachTUMlab",
    "REFLECTEDMASS_humanreachTUMlab",
    "PFL_humanreachablationnocontactclass",
    "PFL_humanreachTUMlab"
]

methods_with_edge_only = ["SWITCH_humanreachTUMlab", "SSM_humanreachTUMlab", "REDUCEDSPEED_humanreachTUMlab", "ZONESSM_humanreachTUMlab"]

calculation_times = {
  "OFF_humanreachTUMlab": np.array([]),
  "PFL_humanreachTUMlab": np.array([]),
  "PFL_humanreachablationnocontactclass": np.array([]),
  "REDUCEDSPEED_humanreachTUMlab": np.array([]),
  "REFLECTEDMASS_humanreachTUMlab": np.array([]),
  "SSM_humanreachTUMlab": np.array([]),
  "SWITCH_humanreachTUMlab": np.array([]),
  "ZONESSM_humanreachTUMlab": np.array([]),
}

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
    calculation_times[method] = np.append(calculation_times[method], calc_times)
    data_list.append([method, n, task, mode, eef, config, last_path_s])

# Create a DataFrame from the compiled data list
columns = ['method', 'user', 'task', 'mode', 'eef', 'config', 'path_s']
result_table = pd.DataFrame(data_list, columns=columns)

# Display the result table
print(result_table)
result_table.to_csv('./data/ablation_results/all_path_s.csv', index=False)


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


# Calculate mean and confidence interval, grouped by 'eef' and 'method'
grouped_means = result_table.groupby(['eef', 'method'])['percentage'].mean().reset_index()
grouped_means.rename(columns={'percentage': 'mean'}, inplace=True)
grouped_stds = result_table.groupby(['eef', 'method'])['percentage'].std().reset_index()
grouped_stds.rename(columns={'percentage': 'std'}, inplace=True)

# Function to calculate bootstrap confidence interval
def bootstrap_conf_interval(data, n_bootstrap=1000):
    # Using scipy's bootstrap function with BCa confidence interval
    resampled_means = bootstrap((data,), np.mean, n_resamples=n_bootstrap, method='bca')
    return resampled_means.confidence_interval

# Apply the bootstrap confidence interval function on each group
conf_intervals = result_table.groupby(['eef', 'method'])['percentage'].apply(lambda x: bootstrap_conf_interval(x)).reset_index()

# Splitting the confidence interval object into separate columns
conf_intervals[['CI_lower', 'CI_upper']] = pd.DataFrame(conf_intervals['percentage'].tolist(), index=conf_intervals.index)

# Drop the original combined CI column
conf_intervals.drop(columns=['percentage'], inplace=True)

# Merging mean and confidence interval data
final_results = pd.merge(grouped_means, grouped_stds, on=['eef', 'method'])
final_results = pd.merge(final_results, conf_intervals, on=['eef', 'method'])

print(final_results)
final_results.to_csv('./data/ablation_results/summary.csv', index=False)

# >>> Create latex table <<<
# Remove the "OFF" method
df_filtered = final_results[final_results['method'] != 'OFF_humanreachTUMlab']

# Repeat the edge values for the methods that only have edge measurements
edge_specific_methods_df = df_filtered[(df_filtered['method'].isin(methods_with_edge_only)) & (df_filtered['eef'] == 'EDGE')]
unique_eefs = df_filtered['eef'].unique()
unique_eefs = [eef for eef in unique_eefs if eef != 'EDGE']
rows_to_add = []
for eef in unique_eefs:
    temp_df = edge_specific_methods_df.copy()
    temp_df['eef'] = eef
    rows_to_add.append(temp_df)
df_filtered = pd.concat([df_filtered] + rows_to_add, ignore_index=True)

# Add custom sorting
df_filtered['method'] = pd.Categorical(df_filtered['method'], categories=sorting, ordered=True)
df_sorted = df_filtered.sort_values('method')
df_sorted = df_filtered.reset_index(drop=True)

# Make max values bold
max_means = df_sorted.groupby('eef')['mean'].transform(max) == df_sorted['mean']
df_sorted['mean_clean'] = df_sorted.apply(lambda row: f"{row['mean']:.1f}", axis=1)
df_sorted.loc[max_means, 'mean_clean'] = df_sorted.loc[max_means, 'mean'].apply(lambda x: f"\\textbf{{{x:.1f}}}")

# Build the mean +/- std string
df_sorted['mean_std'] = df_sorted.apply(lambda row: f"{row['mean_clean']} $\\pm$ {row['std']:.1f}", axis=1)

# Build the table to be columns = EEF type and rows = method
pivot_df = df_sorted.pivot(index='method', columns='eef', values='mean_std').reset_index()

# Add caculation times
calculation_times_list = []
for index, row in pivot_df.iterrows():
    method = row['method']
    calc_times = calculation_times[method]
    mean = np.mean(calc_times) * 1000
    std = np.std(calc_times) * 1000
    calculation_times_list.append(f'{mean:.2f} $\\pm$ {std:.2f}')
pivot_df['Calculation times [ms]'] = calculation_times_list

# Rename columns
column_renames = {
    'BLUNT': 'Blunt',
    'EDGE': 'Edge',
    'SHEET': 'Sheet',
    'WEDGE': 'Wedge',
    'method': 'Method',
}
pivot_df.rename(columns=column_renames, inplace=True)

# Rename methods
pivot_df['Method'] = pivot_df['Method'].apply(lambda x: methods[x])

# Create multicolumn table.
method_df = pivot_df[['Method']]
eef_df = pivot_df[['Blunt', 'Edge', 'Sheet', 'Wedge']]
calc_times_df = pivot_df[['Calculation times [ms]']]
out_df = pd.concat(
    {
        " ": method_df,
        "Efficiency per end-effector type [\%]": eef_df,
        "  ": calc_times_df,
    }, axis=1)

# Construct and save the latex table
latex_table = out_df.to_latex(index=False, header=True, column_format='l' + 'c' * len(pivot_df.columns[1:]), multicolumn_format='c',
                                escape=False, longtable=False)
with open('./data/ablation_results/summary_table.tex', 'w') as file:
    file.write(latex_table)

with open(PAPER_FOLDER + '/summary_table.tex', 'w') as file:
    file.write(latex_table)
