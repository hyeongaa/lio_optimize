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


void extractORBFeatures(const cv::Mat& image, cv::Mat& descriptors, std::vector<cv::KeyPoint>& keypoints)
{
    cv::Ptr<cv::ORB> orb = cv::ORB::create(
        1000, //nfeatures (500)
        1.5f, //scale factor (1.2f) 이미지 피라미드에서 이미지 크기 감소 비율을 결정, 더 큰 값이면 이미지가 더 많이 축소된다. 
        12, //image pyramid: nlevles (8) 더 큰 레벨을 사용하면 더 다양한 스케일에서 특징 검출이 가능합. 
        50, //edgeThreshold (31): 가장자리에서 특징 무시하기 위한 경계의 크기.. 더 큰 값이면 특징을 덜 검출하게 됨.
        0, //first level(0):  pyramid 첫번째 레벨 수를 지정
        2, //WTA_K(2) 각 BRIEF에 사용할 임계값 수를 지정..
        cv::ORB::HARRIS_SCORE, //score type
        31, //patch size(31)... orb descriptor에서 사용하는 크기
        50 //fast threshold(20) 더 큰 값을 사용하녀 민감도가 낮아져서 덜 중요한 특징을 무시하게 된다

    );
    cv::Mat gray;
    
    cv::cvtColor(image,gray,cv::COLOR_BGR2GRAY);

    // Intrinsic matrix
    cv::Mat K = (cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    // Distortion coefficients
    cv::Mat distCoeffs = (cv::Mat_<float>(1, 4) << k1, k2, p1, p2);

    //왜곡 보정
    cv::Mat undistortImage;
    cv::undistort(gray, undistortImage, K, distCoeffs);

    orb->detectAndCompute(undistortImage,cv::noArray(),keypoints,descriptors);
}


Eigen::Matrix4d image_RT(int source, int target, cv::Mat& img_matches)
{

    cv::Mat img1, img2;
    std::vector<cv::KeyPoint> keypoint1, keypoint2;
    cv::Mat descriptors1, descriptors2;

    img1 = frames.at(source).image;
    img2 = frames.at(target).image;

    keypoint1 = frames.at(source).keypoints;
    keypoint2 = frames.at(target).keypoints;

    descriptors1 = frames.at(source).orb_descriptors;
    descriptors2 = frames.at(target).orb_descriptors;


    if (descriptors1.empty() || descriptors2.empty()) {
        std::cout << "Descriptors are empty." << std::endl;
        cv::hconcat(img1, img2, img_matches);
        return Eigen::Matrix4d::Identity();
    }

    // BFMatcher create and matching
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(descriptors1, descriptors2, knn_matches, 2);
    
    // Lowe's ratio test
    const float ratio_thres = 0.75f;
    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i][0].distance < ratio_thres * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
        }  
    }

    // 매칭점으로부터 좌표 생성
    std::vector<cv::Point2f> points1, points2;
    for (int i = 0; i < good_matches.size(); i++) {
        points1.push_back(keypoint1[good_matches[i].queryIdx].pt);
        points2.push_back(keypoint2[good_matches[i].trainIdx].pt);
    }

    // Intrinsic matrix
    cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    // Distortion coefficients
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 4) << k1, k2, p1, p2);

    // 왜곡 보정
    if (!points1.empty() && !points2.empty()) {
        cv::undistortPoints(points1, points1, K, distCoeffs);
        cv::undistortPoints(points2, points2, K, distCoeffs);
    } else {
        std::cout << "Points are empty after undistort." << std::endl;
        cv::hconcat(img1, img2, img_matches);
        return Eigen::Matrix4d::Identity();
    }

    // 최소 매칭 개수 설정- 너무 없으면 거르게
    const int min_inliers = 10;
    if (points1.size() < min_inliers || points2.size() < min_inliers) {
        std::cout << "Not enough matches are found - " << points1.size() << "/" << min_inliers << std::endl;
        cv::hconcat(img1, img2, img_matches);
        return Eigen::Matrix4d::Identity();
    }

    std::vector<uchar> inliers;
    cv::Mat E = cv::findEssentialMat(points1, points2, K, cv::RANSAC, 0.999, 1.0, inliers);

    if (E.empty()) {
        std::cout << "Essential matrix is empty." << std::endl;
        cv::hconcat(img1, img2, img_matches);
        return Eigen::Matrix4d::Identity();
    }

    std::vector<cv::DMatch> inlier_matches;
    for (int i = 0; i < inliers.size(); i++) {
        if (inliers[i]) {
            inlier_matches.push_back(good_matches[i]);
        }
    }

    // Inlier에서도 거르기
    if (inlier_matches.size() < min_inliers) {
        std::cout << "Not enough matches are found - " << inlier_matches.size() << "/" << min_inliers << std::endl;
        cv::hconcat(img1, img2, img_matches);
        return Eigen::Matrix4d::Identity();
    }

    cv::drawMatches(img1, keypoint1, img2, keypoint2, inlier_matches, img_matches);

    // Essential Matrix 계산
    cv::Mat R, T;
    bool success = cv::recoverPose(E, points1, points2, K, R, T);
    if (!success || R.empty() || T.empty()) {
        std::cout << "Failed to recover pose." << std::endl;
        cv::hconcat(img1, img2, img_matches);
        return Eigen::Matrix4d::Identity();
    }

    Eigen::Matrix4d transform_image = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            transform_image(i, j) = R.at<double>(i, j);
        }
        transform_image(i, 3) = T.at<double>(i, 0);
    }

    return transform_image;
}
