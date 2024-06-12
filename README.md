# LIO Opimization node
## 1. 사용법
fast_lio에서 topic을 받아온다는 가정하에 다음과 같이 작동합니다.

     roslaunch snu_colorization colorization.launch



## 2. Topic

총 3개의 topic을 받아옵니다.


1. fast-lio에서 받아오는 topic list

- Lidar Point Cloud: "/cloud_registered"
- Odoemtry: "/Odometry"




## 3. Parameter setting

config/mid360_cam_155.yaml 에서 수정하실 수 있습니다.

camera:
- ic_extrinsic: lidar와 camera와의 extrinsic matrix를 나타낸 것입니다.
    
- k1,k2,p1,p2,fx,fy,cx,cy: camera intrinsic입니다.

저 둘은 로봇 상황에 맞추어 calibration 진행 후 parameter 삽입 후 진행 가능합니다.

- visualize_uncolored: lidar에 맺힌 point들 중, camear FOV안에 안들어가는, 색이 안칠해지는 Point들을 색칠할지 말지 정합니다. (1: 색칠 안된 point pulish, 0: publish안함)

- colored_leaf_size: colorized 된 point들의 voxel 크기를 의미합니다

- uncolored_leaf_size: uncolorized 된 point들의 voxel 크기를 의미합니다.

저 셋은 환경에 맞추어 색칠이 잘되는 parameter로 정해주시면 됩니다.






