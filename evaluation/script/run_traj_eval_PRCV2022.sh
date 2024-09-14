# add eval_cfg.yaml
export Algorithm_Result_Path=/Titan/code/robohike_ws/src/slam_trajectory_evaluation/example_eval_data/benchmark/estimated
export Groundtruth_Path=/Titan/code/robohike_ws/src/slam_trajectory_evaluation/example_eval_data/groundtruth
export Evaluation_Script_Path=/Titan/code/robohike_ws/src/slam_trajectory_evaluation/evaluation/rpg_trajectory_evaluation

# add eval_cfg.yaml
python $Evaluation_Script_Path/scripts/add_eval_cfg_recursive.py $Algorithm_Result_Path/ se3 -1

# evalaution
python $Evaluation_Script_Path/scripts/analyze_trajectories_PRCV2022.py \
--groundtruth_dir=$Groundtruth_Path \
--results_dir=$Algorithm_Result_Path \
--output_dir=$Algorithm_Result_Path \
--computer=laptop \
--mul_trials=0 \
--odometry_error_per_dataset \
--overall_odometry_error \
--plot_trajectories \
--recalculate_errors \
--no_sort_names \
PRCV2022.yaml
