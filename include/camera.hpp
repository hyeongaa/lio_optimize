#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Dense>

#include "frame.hpp"


int projection(pcl::PointXYZINormal *pi, cv::Mat image, pcl::PointXYZRGBA *po);
void extractORBFeatures(const cv::Mat& image, cv::Mat& descriptors, std::vector<cv::KeyPoint>& keypoints);
Eigen::Matrix4d image_RT(int source, int target, cv::Mat& img_matches);

// External variables
extern double k1, k2, p1, p2;
extern double fx, fy, cx, cy;
extern std::vector<frame_pose> frames;

#endif // CAMERA_HPP
