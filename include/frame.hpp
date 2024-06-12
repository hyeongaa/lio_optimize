#ifndef FRAME_HPP
#define FRAME_HPP

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

struct frame_pose {
    cv::Mat image;
    Eigen::Matrix4d transformation_matrix;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  colored_cloud;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr lidar_cloud;

    frame_pose(const cv::Mat& img, const Eigen::Matrix4d& tf_matrix, const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& pc)
        : image(img), transformation_matrix(tf_matrix), lidar_cloud(pc) {}
};

#endif // FRAME_HPP
