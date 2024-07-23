#ifndef FRAME_HPP
#define FRAME_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

struct frame_pose {
    cv::Mat image;
    cv::Mat orb_descriptors;
    std::vector<cv::KeyPoint> keypoints;

    Eigen::Matrix4d transformation_matrix;
    Eigen::Matrix4d corrected_matrix;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  colored_cloud;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr lidar_cloud;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr robot_frame_cloud;

    frame_pose(const cv::Mat& img, const Eigen::Matrix4d& tf_matrix, const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& pc)
        : image(img), transformation_matrix(tf_matrix), lidar_cloud(pc) {}
};

struct submap_pose {
    int index;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr lidar_cloud;
    Eigen::Matrix4d transformation_matrix;

    submap_pose(const int& _index, const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& _lidar_cloud, const Eigen::Matrix4d& matrix)
        : index(_index), lidar_cloud(_lidar_cloud), transformation_matrix(matrix) {}
};

#endif // FRAME_HPP
