#!/bin/bash
# 3-submap baseline evaluation: kf_spgo vs hloc_superpoint_splg vs hloc_disk_dilg
# Usage: bash run_traj_eval_3sub.sh
# Requires: traj_evaluation conda env

export Traj_Path=/Titan/dataset/data_opennavmap/traj_eval_data/map_merge_eval_data
export Proj_Path=/Titan/code/robohike_ws/src/slam_trajectory_evaluation
export Algorithm_Result_Path="${Traj_Path}/algorithms"
export Groundtruth_Path="${Traj_Path}/groundtruth"
export Report_Path="${Traj_Path}/report"
export Evaluation_Script_Path="${Proj_Path}/evaluation/rpg_trajectory_evaluation"
export PYTHON=/root/miniconda3/envs/traj_evaluation/bin/python

# Step 1: add eval_cfg.yaml to all algorithm dirs
${PYTHON} ${Evaluation_Script_Path}/scripts/add_eval_cfg_recursive.py \
    ${Algorithm_Result_Path}/ se3 -1

# Step 2: run evaluation (must cd to scripts/ for add_path.py to work)
cd ${Evaluation_Script_Path}/scripts

${PYTHON} analyze_trajectories_FusionPortable_dataset.py \
  --groundtruth_dir=${Groundtruth_Path} \
  --results_dir=${Algorithm_Result_Path} \
  --output_dir=${Report_Path} \
  --computer=laptop \
  --mul_trials=1 \
  --rmse_table \
  --rmse_table_alg_col \
  --plot_trajectories \
  --no_sort_names \
  map_merge_3sub.yaml

echo ""
echo "Report saved to: ${Report_Path}"
