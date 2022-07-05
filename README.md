<https://vision.in.tum.de/data/datasets/rgbd-dataset/tools>

<https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools/src/rgbd_benchmark_tools/>

# useage

## rpe

python3 evaluate_rpe.py /home/cw/thesis_dataset/eth-3d/plant_dark_mono/plant_dark/groundtruth.txt /home/cw/code/DirectMethod_demo/separate_book1_wrt_Last/res_plant-dark.txt --max_pairs 10000 --fixed_delta  --delta 1 --delta_unit f --plot figure.png --offset 0 --scale 1 --verbose

python3 evaluate_rpe.py /home/cw/thesis_dataset/eth-3d/plant_dark_mono/plant_dark/groundtruth.txt ./res_fr1-xyz.txt --max_pairs 10000 --fixed_delta  --delta 1 --delta_unit f --plot figure.png --offset 0 --scale 1 --verbose

### xyz: ORBSLAM2/ direct method
/home/cw/thesis_dataset/data_tum_rgbd/rgbd_dataset_freiburg1_xyz/groundtruth.txt
/home/cw/code/DirectMethod_demo/separate_book1_wrt_Last/CameraTrajectory.txt
/home/cw/code/DirectMethod_demo/separate_book1_wrt_Last/res_pfr1-xyz1.txt

/home/cw/thesis_dataset/eth-3d/plant_dark_mono/plant_dark/groundtruth.txt
/home/cw/code/DirectMethod_demo/separate_book1_wrt_Last/res_plant-dark.txt

--max_pairs
10000
--fixed_delta
--delta
1
--delta_unit
f
--plot
figure.png
--offset
0
--scale
1
--save
rpe_res.txt
--verbose

## RPE fr1xyz
### ORBSLAM2
compared_pose_pairs 786 pairs
translational_error.rmse 0.005854 m
translational_relative_pose_gt.rmse 0.011290 m
translational_error.mean 0.004842 m
translational_error.median 0.004173 m
translational_error.std 0.003291 m
translational_error.min 0.000118 m
translational_error.max 0.026168 m
rotational_error.rmse 0.392466 deg
rotational_error.mean 0.330342 deg
rotational_error.median 0.288955 deg
rotational_error.std 0.211906 deg
rotational_error.min 0.030543 deg
rotational_error.max 1.530799 deg
### DireceMethod
compared_pose_pairs 786 pairs
translational_error.rmse 0.005399 m
translational_relative_pose_gt.rmse 0.011290 m
translational_error.mean 0.004547 m
translational_error.median 0.003823 m
translational_error.std 0.002911 m
translational_error.min 0.000151 m
translational_error.max 0.018474 m
rotational_error.rmse 0.371366 deg
rotational_error.mean 0.319956 deg
rotational_error.median 0.289577 deg
rotational_error.std 0.188522 deg
rotational_error.min 0.008087 deg
rotational_error.max 1.165372 deg




## ate

python3 evaluate_ate.py /home/cw/thesis_dataset/eth-3d/plant_dark_mono/plant_dark/groundtruth.txt /home/cw/code/DirectMethod_demo/separate_book1_wrt_Last/res_plant-dark.txt --max_pairs 10000 --plot figure.png --offset 0 --scale 1 --verbose



## ATE fr1xyz
### ORBSLAM2
absolute_translational_error.rmse 0.009432 m
absolute_translational_error.mean 0.007925 m
absolute_translational_error.median 0.006856 m
absolute_translational_error.std 0.005114 m
absolute_translational_error.min 0.000438 m
absolute_translational_error.max 0.026650 m
### DirectMethod
compared_pose_pairs 789 pairs
absolute_translational_error.rmse 0.067651 m
absolute_translational_error.mean 0.064674 m
absolute_translational_error.median 0.064121 m
absolute_translational_error.std 0.019849 m
absolute_translational_error.min 0.011821 m
absolute_translational_error.max 0.115901 m















## ate eth3d
### DirectMethod
absolute_translational_error.rmse 0.862347 m
absolute_translational_error.mean 0.834247 m
absolute_translational_error.median 0.814220 m
absolute_translational_error.std 0.218345 m
absolute_translational_error.min 0.442291 m
absolute_translational_error.max 1.277627 m