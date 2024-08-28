# add eval_cfg.yaml
export Algorithm_Result_Path=/Rocket_ssd/dataset/data_topo_loc/matterport3d/vloc_eval_data/algorithms
export Groundtruth_Path=/Rocket_ssd/dataset/data_topo_loc/matterport3d/vloc_eval_data/groundtruth
export Report_Path=/Rocket_ssd/dataset/data_topo_loc/matterport3d/vloc_eval_data/report
export EVALUATION_SCRIPT_PATH=/Titan/code/robohike_ws/src/slam_trajectory_evaluation/evaluation/rpg_trajectory_evaluation

python $EVALUATION_SCRIPT_PATH/scripts/add_eval_cfg_recursive.py $Algorithm_Result_Path/ se3 -1

# evalaution
python $EVALUATION_SCRIPT_PATH/scripts/analyze_trajectories_FusionPortable_dataset.py \
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
--recalculate_errors \
vloc_icra2025.yaml
