#!/bin/bash

# Capture the current directory
ORIGINAL_DIR=$(pwd)
# Calculate build directory path from this script's location
BUILD_DIR="$(dirname "$0")/../build"
# Ensure the BUILD_DIR path is absolute
BUILD_DIR=$(realpath "${BUILD_DIR}")


user_numbers=(1 2 3 4 5 6)
tasks=("puzzle" "robco" "stacking" "tower" "screwing")

commands=(
  "./ablation_study PFL BLUNT human_reach_TUM_lab vicon_p_%user%_%task%_mocap"
  "./ablation_study PFL EDGE human_reach_TUM_lab vicon_p_%user%_%task%_mocap"
  "./ablation_study PFL SHEET human_reach_TUM_lab vicon_p_%user%_%task%_mocap"
  "./ablation_study PFL WEDGE human_reach_TUM_lab vicon_p_%user%_%task%_mocap"
  "./ablation_study SSM EDGE human_reach_TUM_lab vicon_p_%user%_%task%_mocap"
  "./ablation_study OFF EDGE human_reach_TUM_lab vicon_p_%user%_%task%_mocap"
  "./ablation_study PFL BLUNT human_reach_ablation_no_contact_class vicon_p_%user%_%task%_mocap"
  "./ablation_study PFL EDGE human_reach_ablation_no_contact_class vicon_p_%user%_%task%_mocap"
  "./ablation_study PFL SHEET human_reach_ablation_no_contact_class vicon_p_%user%_%task%_mocap"
  "./ablation_study PFL WEDGE human_reach_ablation_no_contact_class vicon_p_%user%_%task%_mocap"
  "./ablation_study REFLECTEDMASS BLUNT human_reach_TUM_lab vicon_p_%user%_%task%_mocap"
  "./ablation_study REFLECTEDMASS EDGE human_reach_TUM_lab vicon_p_%user%_%task%_mocap"
  "./ablation_study REFLECTEDMASS SHEET human_reach_TUM_lab vicon_p_%user%_%task%_mocap"
  "./ablation_study REFLECTEDMASS WEDGE human_reach_TUM_lab vicon_p_%user%_%task%_mocap"
  "./ablation_study REDUCEDSPEED EDGE human_reach_TUM_lab vicon_p_%user%_%task%_mocap"
  "./ablation_study SWITCH EDGE human_reach_TUM_lab vicon_p_%user%_%task%_mocap"
  "./ablation_study ZONESSM EDGE human_reach_TUM_lab vicon_p_%user%_%task%_mocap"
)

# Loop through each user and task, substituting placeholders
for user in "${user_numbers[@]}"; do
  for task in "${tasks[@]}"; do
    # For each command, replace placeholders and run in background
    for cmd_template in "${commands[@]}"; do
      # Replace placeholders with current user and task
      cmd=$(echo "$cmd_template" | sed "s/%user%/${user}/g; s/%task%/${task}/g")
      echo "Running command: $cmd"
      # Change to the build directory, execute command, and change back to the original directory
      # Use subshell to avoid affecting the parent shell's directory
      (
        cd "${BUILD_DIR}" || exit  # Exit if changing directory fails
        eval $cmd
      ) &
    done
    # Wait for all background processes to finish before moving to the next
    wait
  done
done

# return to the original directory
cd "${ORIGINAL_DIR}"
