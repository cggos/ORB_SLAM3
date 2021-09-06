source Examples/ROS/ORB_SLAM3/build/devel/setup.bash

rosrun ORB_SLAM3 Mono_Inertial Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/EuRoC.yaml

rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml true

rosbag play --pause V1_02_medium.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu