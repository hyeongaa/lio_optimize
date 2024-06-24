#ifndef GTSAM_METHOD_HPP
#define GTSAM_METHOD_HPP
#include <iostream>
#include <thread>

#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
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
#include <gtsam/inference/Symbol.h>


#include "frame.hpp"
#include "loop_closure.hpp"
#include "lidar.hpp"

gtsam::Pose3 make_gtsam_pose3(frame_pose _oneframe);
gtsam::Pose3 eig_to_gtsam(Eigen::Matrix4d matrix);

void make_gtsam();
void isam_update();

extern gtsam::NonlinearFactorGraph pgo_graph;
extern gtsam::Values initialEstimate;
extern gtsam::ISAM2 *isam2;
extern gtsam::Values isam_estimation;

extern std::vector<frame_pose> frames;
extern std::vector<submap_pose> submap_poses;

extern bool gtsamInit;
extern int odom_size;
extern gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
extern gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;

extern std::mutex pgo_make;
extern std::mutex update_pose_mutex;
extern int recent_index;

#endif 
