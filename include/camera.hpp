#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

int projection(pcl::PointXYZINormal *pi, cv::Mat image, pcl::PointXYZRGBA *po);

// External variables
extern double k1, k2, p1, p2;
extern double fx, fy, cx, cy;

#endif // CAMERA_HPP
