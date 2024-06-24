#ifndef LIDAR_HPP
#define LIDAR_HPP


#include <iostream>
#include <string>
#include <cstdlib> //for servers
#include <thread>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Trigger.h>  // for servers

//PCL
#include <pcl/point_types.h> //pt
#include <pcl/point_cloud.h> //cloud
#include <pcl/conversions.h> //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h> //icp


#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <deque>
#include <queue>
#include <vector>
#include <math.h>

//GTSAM
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "frame.hpp"


Eigen::Matrix4d performICP(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& target_cloud, bool& icp_success );
pcl::PointCloud<pcl::PointXYZINormal>::Ptr make_cloud_submap(int index, int submap_size);
void voxelize_pcd(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud);
bool keysubmap_check(int prev, int curr);
void make_key_map_index(int index);
void make_lidar_gtsam();

extern gtsam::NonlinearFactorGraph pgo_graph;
extern gtsam::Values initialEstimate;
extern std::mutex pgo_make;
extern std::mutex key_mutex;
extern gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
extern gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
extern double key_thr;
extern std::vector<int> key_submap_index;
extern double icp_score_thr;

#endif