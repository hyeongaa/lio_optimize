#ifndef TRANSFORMATION_HPP
#define TRANSFORMATION_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

void World_TO_ROBOT(pcl::PointXYZINormal *pi, pcl::PointXYZINormal *po, Eigen::Matrix4d transformation_matrix);
void IMU_TO_CAMERA(pcl::PointXYZINormal *pi, pcl::PointXYZINormal *po);
void Camera_to_IMU(pcl::PointXYZRGB *pi, pcl::PointXYZRGB *po);
void ROBOT_TO_WORLD_CORRECTION(pcl::PointXYZINormal *pi, pcl::PointXYZINormal *po, Eigen::Matrix4d corrected_matrix);

extern Eigen::Matrix4d extrinsic_matrix;

#endif 
