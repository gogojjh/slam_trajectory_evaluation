# add eval_cfg.yaml
export Traj_Path=/Titan/dataset/data_opennavmap/traj_eval_data/map_merge_eval_data
export Proj_Path=/Titan/code/robohike_ws/src/slam_trajectory_evaluation

export Algorithm_Result_Path="$Traj_Path/algorithms"
export Groundtruth_Path="$Traj_Path/groundtruth"
export Report_Path="$Traj_Path/report"
export Evaluation_Script_Path="$Proj_Path/evaluation/rpg_trajectory_evaluation"

# add eval_cfg.yaml
python $Evaluation_Script_Path/scripts/add_eval_cfg_recursive.py $Algorithm_Result_Path/ se3 -1

# evalaution
python $Evaluation_Script_Path/scripts/analyze_trajectories_FusionPortable_dataset.py \
  --groundtruth_dir=$Groundtruth_Path \
  --results_dir=$Algorithm_Result_Path \
  --output_dir=$Report_Path \
  --computer=laptop \
  --mul_trials=0 \
  --overall_odometry_error \
  --odometry_error_per_dataset \
  --rmse_boxplot \
  --rmse_table \
  --rmse_table_alg_col \
  --plot_trajectories \
  --write_time_statistics \
  --no_sort_names \
  OpenNavMap_map_merge.yaml

  # --recalculate_errors \
