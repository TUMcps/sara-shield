"""Analyze collision types in ablation study runs."""
import pandas as pd
import glob
import os
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict


PANDA = False
BASE_FOLDER = "./data/ablation_results_panda" if PANDA else "./data/ablation_results"


def analyze_collision_types():
    """Read all full CSV files and create collision type histograms."""

    # Find all CSV files that are NOT _path_s.csv files
    all_csv_files = glob.glob(f'{BASE_FOLDER}/vicon_p_*_mocap_*.csv')
    # Filter out the _path_s.csv files
    full_csv_files = [f for f in all_csv_files if not f.endswith('_path_s.csv')]

    # Store collision type counts for each file
    collision_data = []
    file_summaries = []

    print(f"Processing {len(full_csv_files)} files...")

    for file in full_csv_files:
        filename = os.path.basename(file)
        print(f"Processing: {filename}")

        # Parse filename to extract metadata
        parts = filename.split('_')
        n = parts[2]
        task = parts[3]
        mode = parts[5]
        eef = parts[6]
        config = "".join(parts[7:]).replace('.csv', '')
        method = f'{mode}_{config}'

        try:
            # Read only the collision_type column for efficiency (note the space after comma in header)
            df = pd.read_csv(file, usecols=[' collision_type'], on_bad_lines='skip')
            # Rename column to remove leading space
            df.columns = df.columns.str.strip()

            # Count collision types
            collision_counts = df['collision_type'].value_counts().sort_index()

            # Store data for each collision type
            for collision_type, count in collision_counts.items():
                collision_data.append({
                    'filename': filename,
                    'method': method,
                    'user': n,
                    'task': task,
                    'mode': mode,
                    'eef': eef,
                    'config': config,
                    'collision_type': collision_type,
                    'count': count,
                    'total_rows': len(df)
                })

            # Store file summary
            file_summaries.append({
                'filename': filename,
                'method': method,
                'user': n,
                'task': task,
                'mode': mode,
                'eef': eef,
                'config': config,
                'total_rows': len(df),
                'collision_types': dict(collision_counts)
            })

        except Exception as e:
            print(f"Error processing {filename}: {e}")
            continue

    # Create DataFrame from collision data
    collision_df = pd.DataFrame(collision_data)

    # Create summary DataFrame
    summary_df = pd.DataFrame(file_summaries)

    # Calculate percentages
    collision_df['percentage'] = (collision_df['count'] / collision_df['total_rows']) * 100

    print("\nCollision Type Analysis Complete!")
    print(f"Processed {len(file_summaries)} files successfully")
    print(f"Total collision type entries: {len(collision_df)}")

    # Save detailed results
    collision_df.to_csv(f'{BASE_FOLDER}/collision_type_analysis.csv', index=False)
    print("Detailed results saved to: f'{BASE_FOLDER}/collision_type_analysis.csv")

    # Create summary table
    create_summary_table(collision_df, summary_df)

    # Create histograms
    create_histograms(collision_df, summary_df)

    return collision_df, summary_df


def create_summary_table(collision_df, summary_df):
    """Create a summary table of collision types by method."""

    # Pivot table to show collision type counts by method
    pivot_table = collision_df.pivot_table(
        index=['method', 'user', 'task'],
        columns='collision_type',
        values='count',
        fill_value=0,
        aggfunc='sum'
    )

    # Add total column
    pivot_table['total'] = pivot_table.sum(axis=1)

    # Calculate percentages
    percentage_table = pivot_table.div(pivot_table['total'], axis=0) * 100
    percentage_table = percentage_table.drop('total', axis=1)

    print("\nCollision Type Summary (Counts):")
    print(pivot_table)

    print("\nCollision Type Summary (Percentages):")
    print(percentage_table.round(2))

    # Save summary tables
    pivot_table.to_csv(f'{BASE_FOLDER}/collision_type_summary_counts.csv')
    percentage_table.to_csv(f'{BASE_FOLDER}/collision_type_summary_percentages.csv')

    print("Summary tables saved to:")
    print(f"  - {BASE_FOLDER}/collision_type_summary_counts.csv")
    print(f"  - {BASE_FOLDER}/collision_type_summary_percentages.csv")


def create_histograms(collision_df, summary_df):
    """Create histograms of collision types."""

    # Get unique methods for plotting
    methods = collision_df['method'].unique()

    # Create figure with subplots
    n_methods = len(methods)
    cols = 3
    rows = (n_methods + cols - 1) // cols

    fig, axes = plt.subplots(rows, cols, figsize=(15, 5*rows))
    if rows == 1:
        axes = axes.reshape(1, -1)
    axes = axes.flatten()

    # Collect per-method histogram data for CSV export
    method_histogram_rows = []

    for i, method in enumerate(methods):
        method_data = collision_df[collision_df['method'] == method]

        # Aggregate counts across all files for this method
        collision_totals = method_data.groupby('collision_type')['count'].sum()
        total_steps = method_data.groupby('collision_type')['total_rows'].sum().iloc[0] if len(method_data) > 0 else 1
        collision_percentages = (collision_totals / collision_totals.sum()) * 100

        for collision_type in collision_totals.index:
            method_histogram_rows.append({
                'method': method,
                'collision_type': collision_type,
                'count': collision_totals[collision_type],
                'percentage': collision_percentages[collision_type],
            })

        # Create histogram
        ax = axes[i]
        collision_totals.plot(kind='bar', ax=ax, title=f'Collision Types - {method}')
        ax.set_xlabel('Collision Type')
        ax.set_ylabel('Count')
        ax.tick_params(axis='x', rotation=0)

        # Add count labels on bars
        for j, (collision_type, count) in enumerate(collision_totals.items()):
            ax.text(j, count + count*0.01, str(count), ha='center', va='bottom')

    # Export per-method histogram data as CSV
    method_histogram_df = pd.DataFrame(method_histogram_rows)
    method_histogram_df.to_csv(f'{BASE_FOLDER}/collision_type_histograms.csv', index=False)
    print(f"Histogram data saved to: {BASE_FOLDER}/collision_type_histograms.csv")

    # Hide unused subplots
    for i in range(n_methods, len(axes)):
        axes[i].set_visible(False)

    plt.tight_layout()
    plt.savefig(f'{BASE_FOLDER}/collision_type_histograms.png', dpi=300, bbox_inches='tight')
    print(f"Histograms saved to: {BASE_FOLDER}/collision_type_histograms.png")

    # Create overall histogram
    plt.figure(figsize=(10, 6))
    overall_totals = collision_df.groupby('collision_type')['count'].sum()
    overall_percentages = (overall_totals / overall_totals.sum()) * 100
    overall_totals.plot(kind='bar', title='Overall Collision Type Distribution')
    plt.xlabel('Collision Type')
    plt.ylabel('Total Count')
    plt.xticks(rotation=0)

    # Add count labels
    for i, (collision_type, count) in enumerate(overall_totals.items()):
        plt.text(i, count + count*0.01, str(count), ha='center', va='bottom')

    # Export overall histogram data as CSV
    overall_histogram_df = pd.DataFrame({
        'collision_type': overall_totals.index,
        'count': overall_totals.values,
        'percentage': overall_percentages.values,
    })
    overall_histogram_df.to_csv(f'{BASE_FOLDER}/collision_type_overall_histogram.csv', index=False)
    print(f"Overall histogram data saved to: {BASE_FOLDER}/collision_type_overall_histogram.csv")

    plt.tight_layout()
    plt.savefig(f'{BASE_FOLDER}/collision_type_overall_histogram.png', dpi=300, bbox_inches='tight')
    print(f"Overall histogram saved to: {BASE_FOLDER}/collision_type_overall_histogram.png")

    plt.show()


if __name__ == "__main__":
    collision_df, summary_df = analyze_collision_types()

    print("\nAnalysis complete! Files generated:")
    print("  - collision_type_analysis.csv (detailed data)")
    print("  - collision_type_summary_counts.csv (summary counts)")
    print("  - collision_type_summary_percentages.csv (summary percentages)")
    print("  - collision_type_histograms.png (histograms by method)")
    print("  - collision_type_histograms.csv (histogram data with percentages by method)")
    print("  - collision_type_overall_histogram.png (overall histogram)")
    print("  - collision_type_overall_histogram.csv (overall histogram data with percentages)")