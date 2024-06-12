#include "camera.hpp"

double k1 = 0;
double k2 = 0;
double p1 = 0;
double p2 = 0;
double fx = 0;
double fy = 0;
double cx = 0;
double cy = 0;

int projection(pcl::PointXYZINormal *pi, cv::Mat image, pcl::PointXYZRGBA *po) {
    po->x = pi->x;
    po->y = pi->y;
    po->z = pi->z;

    double dist_x = pi->x / pi->z;
    double dist_y = pi->y / pi->z;
    double r_2 = dist_x * dist_x + dist_y * dist_y;
    double vari = 1 + k1 * r_2 + k2 * r_2 * r_2;
    double undist_x = dist_x * vari + 2 * p1 * dist_x * dist_y + p2 * (r_2 + 2 * dist_x * dist_x);
    double undist_y = dist_y * vari + p1 * (r_2 + 2 * dist_y * dist_y) + 2 * p2 * dist_x * dist_y;

    double x_coord = fx * undist_x + cx;
    double y_coord = fy * undist_y + cy;
    int u = static_cast<int>(x_coord);
    int v = static_cast<int>(y_coord);

    double alpha = x_coord - u;
    double beta = y_coord - v;
    int u_floor = static_cast<int>(std::floor(x_coord));
    int v_floor = static_cast<int>(std::floor(y_coord));
    int u_ceil = static_cast<int>(std::ceil(x_coord));
    int v_ceil = static_cast<int>(std::ceil(y_coord));

    if (u_floor >= 0 && u_ceil < image.cols && v_floor >= 0 && v_ceil < image.rows) {
        cv::Vec3b pixel1 = (1 - alpha) * (1 - beta) * image.at<cv::Vec3b>(v_floor, u_floor);
        cv::Vec3b pixel2 = alpha * (1 - beta) * image.at<cv::Vec3b>(v_floor, u_ceil);
        cv::Vec3b pixel3 = (1 - alpha) * beta * image.at<cv::Vec3b>(v_ceil, u_floor);
        cv::Vec3b pixel4 = alpha * beta * image.at<cv::Vec3b>(v_ceil, u_ceil);
        cv::Vec3b interpolated_pixel = pixel1 + pixel2 + pixel3 + pixel4;

        po->r = static_cast<uint8_t>(interpolated_pixel[2]);
        po->g = static_cast<uint8_t>(interpolated_pixel[1]);
        po->b = static_cast<uint8_t>(interpolated_pixel[0]);
        po->a = 255;
    } else {
        po->r = 255;
        po->g = 255;
        po->b = 255;
        po->a = 20;
        return 0;
    }
    return 1;
}
