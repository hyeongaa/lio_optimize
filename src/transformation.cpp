#include "transformation.hpp"

Eigen::Matrix4d extrinsic_matrix;

void World_TO_ROBOT(pcl::PointXYZINormal *pi, pcl::PointXYZINormal *po, Eigen::Matrix4d transformation_matrix) {
    Eigen::Vector4d p_world(pi->x, pi->y, pi->z, 1.0);
    Eigen::Vector4d p_body = transformation_matrix.inverse() * p_world;

    po->x = p_body(0);
    po->y = p_body(1);
    po->z = p_body(2);
    po->intensity = pi->intensity;
}

void IMU_TO_CAMERA(pcl::PointXYZINormal *pi, pcl::PointXYZINormal *po) {
    Eigen::Vector4d p_body(pi->x, pi->y, pi->z, 1);
    Eigen::Vector4d c_body = extrinsic_matrix * p_body;

    po->x = c_body(0);
    po->y = c_body(1);
    po->z = c_body(2);
    po->intensity = pi->intensity;
}

void Camera_to_IMU(pcl::PointXYZRGB *pi, pcl::PointXYZRGB *po) {
    Eigen::Matrix4d inverse_extrinsic = extrinsic_matrix.inverse();

    Eigen::Vector4d c_body(pi->x, pi->y, pi->z, 1.0);
    Eigen::Vector4d b_body = inverse_extrinsic * c_body;

    po->x = b_body(0);
    po->y = b_body(1);
    po->z = b_body(2);
    po->r = pi->r;
    po->g = pi->g;
    po->b = pi->b;
}

void ROBOT_TO_WORLD_CORRECTION(pcl::PointXYZINormal *pi, pcl::PointXYZINormal *po, Eigen::Matrix4d corrected_matrix)
{
    Eigen::Vector4d p_body(pi->x, pi->y, pi->z, 1.0);
    Eigen::Vector4d p_corrected_world = corrected_matrix * p_body;

    po->x = p_corrected_world(0);
    po->y = p_corrected_world(1);
    po->z = p_corrected_world(2);
    po->intensity = pi->intensity;

}