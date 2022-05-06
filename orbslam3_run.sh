source Examples/ROS/ORB_SLAM3/build/devel/setup.bash



rosrun ORB_SLAM3 RGBD Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml

rosbag play rgbd_dataset_freiburg1_xyz.bag /camera/rgb/image_color:=/camera/rgb/image_raw /camera/depth/image:=/camera/depth_registered/image_raw



rosrun ORB_SLAM3 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml true

rosbag play MH_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw



rosrun ORB_SLAM3 Mono_Inertial Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/EuRoC.yaml

rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml true

rosbag play --pause V1_02_medium.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu


# RGBD-I

rosrun ORB_SLAM3 RGBD Vocabulary/ORBvoc.txt Examples_old/RGB-D-Inertial/OpenLORIS_Scene.yaml

rosbag play cafe1-1.bag /d400/color/image_raw:=/camera/rgb/image_raw /d400/aligned_depth_to_color/image_raw:=/camera/depth_registered/image_raw


rosrun ORB_SLAM3 RGBD_Inertial Vocabulary/ORBvoc.txt Examples_old/RGB-D-Inertial/OpenLORIS_Scene.yaml

rosbag play cafe1-1.bag /d400/color/image_raw:=/camera/rgb/image_raw /d400/aligned_depth_to_color/image_raw:=/camera/depth_registered/image_raw /d400/gyro/sample:=/imu_gyr /d400/accel/sample:=/imu_acc


# Fisheye-Depth

rosrun --prefix 'gdb --args' ORB_SLAM3 FisheyeDepth Vocabulary/ORBvoc.txt Examples_old/RGB-D-Inertial/RealSense_T265_D435i.yaml


rosrun ORB_SLAM3 RGBD Vocabulary/ORBvoc.txt Examples_old/RGB-D/RealSense_D435i.yaml

rosbag play d435i-t265-20220407.bag /D435I/color/image_raw:=/camera/rgb/image_raw /D435I/depth/image_rect_raw:=/camera/depth_registered/image_raw
