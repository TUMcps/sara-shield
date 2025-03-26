"""Create the ablation summary table.

Requires to run combine_runs.py first!
"""
import pandas as pd
import glob
import os
import csv
import numpy as np
from scipy.stats import bootstrap, shapiro, wilcoxon


PAPER_FOLDER = '/home/jakob/Nextcloud/Promotion/Paper/General_Safety_Framework/Paper/results/ablation_study/'


# Method names
methods = {
  "OFF_humanreachTUMlab": "OFF",
  "PFL_humanreachTUMlab": "\shield{} (\\textbf{ours})",
  "PFL_humanreachablationnocontactclass": "\shield{} without $c_{\mathrm{free}}$",
  "REDUCEDSPEED_humanreachTUMlab": "Reduced-speed PFL \cite{iso_2021_RoboticsSafety}",
  "REFLECTEDMASS_humanreachTUMlab": "Reflected mass \cite{haddadin_2012_MakingRobots}",
  "SSM_humanreachTUMlab": "Dynamic SSM \cite{beckert_2017_OnlineVerification, thumm_2022_ProvablySafe}",
  "SWITCH_humanreachTUMlab": "Reduced-speed zone \cite{svarny_2019_SafePhysical}",
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

result_table = pd.read_csv('./data/ablation_results/all_results.csv')

method_names = {
  "OFF_humanreachTUMlab": "OFF",
  "PFL_humanreachTUMlab": "\shield{} (\\textbf{ours})",
  "PFL_humanreachablationnocontactclass": "\shield{} without $c_{\mathrm{free}}$",
  "REDUCEDSPEED_humanreachTUMlab": "Reduced-speed PFL \cite{iso_2021_RoboticsSafety}",
  "REFLECTEDMASS_humanreachTUMlab": "Reflected mass \cite{haddadin_2012_MakingRobots}",
  "SSM_humanreachTUMlab": "Dynamic SSM \cite{beckert_2017_OnlineVerification, thumm_2022_ProvablySafe}",
  "SWITCH_humanreachTUMlab": "Reduced-speed zone \cite{svarny_2019_SafePhysical}",
  "ZONESSM_humanreachTUMlab": "SSM zone \cite{iso_2021_RoboticsSafety}",
}

methods = result_table['method'].unique()
users = result_table['user'].unique()
tasks = result_table['task'].unique()
eefs = result_table['eef'].unique()

# Calculate mean and confidence interval, grouped by 'eef' and 'method'
grouped_means = result_table.groupby(['eef', 'method'])['percentage'].mean().reset_index()
grouped_means.rename(columns={'percentage': 'mean'}, inplace=True)
grouped_stds = result_table.groupby(['eef', 'method'])['percentage'].std().reset_index()
grouped_stds.rename(columns={'percentage': 'std'}, inplace=True)

# Function to calculate bootstrap confidence interval
def bootstrap_conf_interval(data, n_bootstrap=1000):
    # Using scipy's bootstrap function with BCa confidence interval
    resampled_means = bootstrap((data,), np.mean, n_resamples=n_bootstrap, method='bca', confidence_level=0.999999426696856)
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

# >>> Add caculation times <<<
mean_calc_times = []
std_calc_times = []
for index, row in pivot_df.iterrows():
    method = row['method']
    calc_times = result_table.loc[(result_table['method'] == method), 'step_time']
    mean_calc_times.append(calc_times.mean() * 1000)
    std_calc_times.append(calc_times.std() * 1000)

pivot_df['mean_calc'] = mean_calc_times
pivot_df['std_calc'] = std_calc_times

# Make min calculation time bold
min_calc_times = np.argmin(mean_calc_times)
pivot_df['mean_clean_calc'] = pivot_df.apply(lambda row: f"{row['mean_calc']:.2f}", axis=1)
pivot_df.loc[min_calc_times, 'mean_clean_calc'] = f"\\textbf{{{pivot_df.loc[min_calc_times, 'mean_calc']:.2f}}}"

# Build the mean +/- std string
pivot_df['Computation Time [ms]'] = pivot_df.apply(lambda row: f"{row['mean_clean_calc']} $\\pm$ {row['std_calc']:.2f}", axis=1)

# >>> Perform statistical hypothesis analysis <<<
ours = "PFL_humanreachTUMlab"
hypothesis_p_values = []
# For all Methods
for index, row in pivot_df.iterrows():
    method = row['method']
    if method == ours or method == "OFF_humanreachTUMlab":
        hypothesis_p_values.append("--")
        continue
    n_trials = len(users) * len(tasks) * len(eefs)
    percentage_diffs = np.zeros((n_trials,))
    i = 0
    # For all users
    for user in users:
        # For all tasks
        for task in tasks:
            # For all EEF types
            for eef in eefs:
                result_ours = result_table.loc[
                    (result_table['method'] == ours) & 
                    (result_table['user'] == user) & 
                    (result_table['task'] == task) & 
                    (result_table['eef'] == eef), 
                    'percentage'
                ]
                # Get the value (if you expect a single value)
                percentage_ours = result_ours.iloc[0] if not result_ours.empty else None
                result_method = result_table.loc[
                    (result_table['method'] == method) & 
                    (result_table['user'] == user) & 
                    (result_table['task'] == task) & 
                    (result_table['eef'] == eef), 
                    'percentage'
                ]
                # For some methods, the results are the same across EEF types, so we only recorded EDGE type.
                if result_method.empty:
                    if method not in methods_with_edge_only:
                        raise ValueError(f"Method {method} has no values for EEF type {eef}!")
                    result_method = result_table.loc[
                        (result_table['method'] == method) & 
                        (result_table['user'] == user) & 
                        (result_table['task'] == task) & 
                        (result_table['eef'] == "EDGE"), 
                        'percentage'
                    ]
                percentage_method = result_method.iloc[0] if not result_method.empty else None
                percentage_diffs[i] = percentage_ours - percentage_method
                i += 1

    # Visualization to show normal distribution
    stat, p = shapiro(percentage_diffs)
    if p > 0.05:
        print(f"Performance difference between {ours} and {method} does is normally distributed! Paired t-test is prefferred over Wilcoxon signed-rank test!")
    w_stat, p_value_wilcoxon = wilcoxon(x=percentage_diffs, alternative="greater")
    hypothesis_p_values.append(p_value_wilcoxon)

pivot_df['p-value of H1'] = hypothesis_p_values
pivot_df['p-value of H1'] = pivot_df['p-value of H1'].apply(
    lambda x: f"{float(x):.2e}" if isinstance(x, (int, float)) else x
)

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
pivot_df['Method'] = pivot_df['Method'].apply(lambda x: method_names[x])

# Create multicolumn table.
method_df = pivot_df[['Method']]
eef_df = pivot_df[['Blunt', 'Edge', 'Sheet', 'Wedge']]
calc_times_df = pivot_df[['Computation Time [ms]']]
p_value_df = pivot_df[['p-value of H1']]
p_val_rename = {'p-value of H1': 'p-value of \\textbf{{H1}}'}
p_value_df.rename(columns=p_val_rename, inplace=True)
out_df = pd.concat(
    {
        " ": method_df,
        "Efficiency per End Effector Type [\%]": eef_df,
        "  ": p_value_df,
        "   ": calc_times_df
    }, axis=1)

# Construct and save the latex table
with pd.option_context("max_colwidth", 1000):
    latex_table = out_df.to_latex(index=False, header=True, column_format='l' + 'c' * len(pivot_df.columns[1:]), multicolumn_format='c',
                                escape=False, longtable=False)
    latex_list = latex_table.splitlines()
    latex_list.insert(3, '\\cmidrule(lr){2-5}')
    latex_table = '\n'.join(latex_list)

with open('./data/ablation_results/summary_table.tex', 'w') as file:
    file.write(latex_table)

with open(PAPER_FOLDER + '/summary_table.tex', 'w') as file:
    file.write(latex_table)
