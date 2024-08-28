#!/usr/bin/env python2

# default
kNsToEstFnMapping = {
    'traj_est': 'stamped_traj_estimate',
    'pose_graph': 'stamped_pose_graph_estimate',
    'ba_estimate': 'stamped_ba_estimate'
}

# FusionPortable_dataset
FusionPortable_dataset_mapping = {
    'gt': 'groundtruth_traj',
    'vinsfusionvio': 'vinsfusion_vio_traj',
    'vinsfusionlc': 'vinsfusion_lc_traj',
    'vinsfusion-stereovio-lc': 'vinsfusion_stereovio_lc_traj',
    'orbslam2': 'orbslam2_traj',
    'orbslam3': 'orbslam3_traj',
    'orbslam3-stereo-lc': 'orbslam3_stereo_lc_traj',
    'orbslam3-stereovio-lc': 'orbslam3_stereovio_traj',
    'esvo': 'esvo_traj',
    'aloam': 'aloam_traj',
    'liomapping': 'liomapping_traj',
    'liosam': 'liosam_traj',
    'fastlio2': 'fastlio2_traj',
    'fasterlio2': 'fasterlio2_traj',
    'r3live': 'r3live_traj'
}

kNsToEstFnMapping.update(FusionPortable_dataset_mapping)
kNsToMatchFnMapping = kNsToEstFnMapping
print(kNsToEstFnMapping)

kFnExt = 'txt'
