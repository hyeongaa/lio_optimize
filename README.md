# Lio-PGO-Colorization Node

This node is backend process of Lidar SLAM. This node subscribe odometry, pointcloud and images(optinal) from any odometry algorithms(fast-lio, lio-sam, faster-lio...).

## 1. How to use?

     source devel/setup.bash
     roslaunch snu_colorization colorization.launch


## 2. Topic

This node subscribe total 3 topics. 


1. Topic list from lidar-odometry algorithm(Ex: FAST-LIO):

- Lidar Point Cloud: "/cloud_registered"
- Odoemtry: "/Odometry"

2. Topic list from sensor(or other odoemtry): compressed image

- /cam0/compressed

## 3. Parameter setting

config/mid360_cam_155.yaml 

camera:
- ic_extrinsic: extrinsic matrix between lidar and camera
    
- k1,k2,p1,p2,fx,fy,cx,cy: camera intrinsic.

- visualize_uncolored: decide wheter to publsih lidar points which are not in camera FOV. (1: point pulish, 0: do not publish)

- colored_leaf_size: decide the voxel size of colored point cloud

- uncolored_leaf_size: decide the voxel size of uncolored point cloud

those parameters can be modified.



