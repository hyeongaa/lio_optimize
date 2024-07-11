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
#include <visualization_msgs/Marker.h>
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
#include <pcl/filters/statistical_outlier_removal.h>

//Transformation, common
#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <deque>
#include <queue>
#include <vector>
#include <math.h>

//IMAGE
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

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
#include <gtsam/inference/Symbol.h>


#include "transformation.hpp"
#include "camera.hpp"
#include "frame.hpp"
#include "gtsam_method.hpp"
#include "lidar.hpp"
#include "loop_closure.hpp"

using namespace std;
string root_dir = ROOT_DIR;

//for saving all the pcd
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_wait_save (new pcl::PointCloud<pcl::PointXYZRGBA>());
int pcd_save_interval = -1;
bool pcd_save_en = true;
int pcd_index =0;

// visualize uncolored point cloud yes/no
int visualize_uncolored = 1;
float colored_leaf_size = 0.03f;
float uncolored_leaf_size = 0.1f;

vector<double> extrinsic_matrix_vector;

//frames
std::vector<frame_pose> frames; //1_frame={image,odom,pointcloud}
std::vector<submap_pose> submap_poses;
std::vector<int> key_submap_index; //key frames index selection
std::vector<std::pair<int,int>> node_selection; //loop constraint candidates
std::vector<std::pair<int,int>> loop_closed_pair; //loop constraints index set

ros::Publisher pubcolorlaser;
ros::Publisher pubodom;
ros::Publisher pubpath;
ros::Publisher pubmarker;

//GTSAM
gtsam::NonlinearFactorGraph pgo_graph;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam2;
gtsam::Values isam_estimation;


gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
gtsam::noiseModel::Base::shared_ptr loopNoise;

std::mutex pgo_make;
std::mutex update_pose_mutex;
std::mutex key_mutex;

//loop closing
int recent_index =0 ;
bool loop_closed = false;
double key_thr;
double loop_distance_candidate;
double icp_score_thr;

//pointcloud publsih
pcl::PointCloud<pcl::PointXYZINormal>::Ptr pointcloud_publish(new pcl::PointCloud<pcl::PointXYZINormal>());

gtsam::noiseModel::mEstimator::Cauchy::shared_ptr cauchyEstimator;



void publish_colored_corrected_cloud(const ros::Publisher &pubcolorlaser)
{   

    pointcloud_publish->clear();

    for(int i=0; i<recent_index;i++)
    {
        frame_pose &p = frames[i];
        cv::Mat image = p.image;
        Eigen::Matrix4d not_corrected = p.transformation_matrix;
        Eigen::Matrix4d corrected = p.corrected_matrix;
        int size = p.robot_frame_cloud->points.size();

        //pcl::PointCloud<pcl::PointXYZINormal>::Ptr world_before_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
        //pcl::PointCloud<pcl::PointXYZINormal>::Ptr robot_frame_cloud(new pcl::PointCloud<pcl::PointXYZINormal>(size,1));
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr color_robot_cloud(new pcl::PointCloud<pcl::PointXYZINormal>(size,1));
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr color_world_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

        update_pose_mutex.lock();
        for(int i=0; i<size;i++)
        {
            //World_TO_ROBOT(&p.lidar_cloud->points[i], &robot_frame_cloud->points[i],not_corrected);
            //IMU_TO_CAMERA(&robot_frame_cloud->points[i], &color_robot_cloud->points[i]);
            ROBOT_TO_WORLD_CORRECTION(&p.robot_frame_cloud->points[i], &color_robot_cloud->points[i],corrected);

        }
        update_pose_mutex.unlock();
        *pointcloud_publish += *color_robot_cloud;

    }
    
    sensor_msgs::PointCloud2 lasermsgs;
    pcl::toROSMsg(*pointcloud_publish, lasermsgs);
    lasermsgs.header.frame_id = "camera_init";
    pubcolorlaser.publish(lasermsgs);
}

void publish_colored_cloud(const ros::Publisher & pubcolorlaser)
{
    cv::Mat image = frames.back().image;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr uncolored_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
    *uncolored_cloud = *(frames.back().lidar_cloud);
    Eigen::Matrix4d transformation_matrix = frames.back().transformation_matrix;
    

    int size = uncolored_cloud->points.size();
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr tranformed_cloud(new pcl::PointCloud<pcl::PointXYZINormal>(size,1));
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZINormal>(size,1));
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colorlidarcloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr uncoloredlidarcloud(new pcl::PointCloud<pcl::PointXYZRGBA>());



    for(int i=0; i<size; i++)
    {
        World_TO_ROBOT(&uncolored_cloud->points[i],&tranformed_cloud->points[i],transformation_matrix);
        *frames.back().robot_frame_cloud = *tranformed_cloud;
        IMU_TO_CAMERA(&tranformed_cloud->points[i],&colored_cloud->points[i]);
        pcl::PointXYZRGBA colorcloud;
        if(projection(&colored_cloud->points[i],image,&colorcloud))
        {   
            pcl::PointXYZRGBA validPoint;
            validPoint.x = uncolored_cloud->points[i].x;
            validPoint.y = uncolored_cloud->points[i].y;
            validPoint.z = uncolored_cloud->points[i].z;
            validPoint.r = colorcloud.r;
            validPoint.g = colorcloud.g;
            validPoint.b = colorcloud.b;

            colorlidarcloud -> points.push_back(validPoint);
        }else if(visualize_uncolored){
            pcl::PointXYZRGBA unvalidPoint;
            unvalidPoint.x = uncolored_cloud->points[i].x;
            unvalidPoint.y = uncolored_cloud->points[i].y;
            unvalidPoint.z = uncolored_cloud->points[i].z;
            unvalidPoint.r = colorcloud.r;
            unvalidPoint.g = colorcloud.g;
            unvalidPoint.b = colorcloud.b;
            unvalidPoint.a = colorcloud.a;

            uncoloredlidarcloud->points.push_back(unvalidPoint);        
        }
    }


    //for colored lidar cloud -> voxelize densely
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_filter;
    voxel_filter.setLeafSize(colored_leaf_size, colored_leaf_size, colored_leaf_size);
        
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colorlidarcloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
    voxel_filter.setInputCloud(colorlidarcloud);
    voxel_filter.filter(*colorlidarcloud_filtered);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_temp_save_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
    *pcl_temp_save_filtered += *colorlidarcloud_filtered;

    //for not colored lidar cloud -> voxelize sparsely
    if(visualize_uncolored){
           
        pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_filter_uncolored;
        voxel_filter_uncolored.setLeafSize(uncolored_leaf_size, uncolored_leaf_size, uncolored_leaf_size);
            
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr uncoloredcloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
        voxel_filter_uncolored.setInputCloud(uncoloredlidarcloud);
        voxel_filter_uncolored.filter(*uncoloredcloud_filtered);

        // add!(color + uncolored)
        *colorlidarcloud_filtered += *uncoloredcloud_filtered;
        *pcl_temp_save_filtered += *uncoloredcloud_filtered;
    }

    //publish
    sensor_msgs::PointCloud2 lasercloudmsg;
    pcl::toROSMsg(*colorlidarcloud_filtered,lasercloudmsg);
    //lasercloudmsg.header.stamp = ros::Time().fromSec(/*추가*/);
    lasercloudmsg.header.frame_id = "camera_init";
    pubcolorlaser.publish(lasercloudmsg);

    if (pcd_save_en)
    {

        *pcl_wait_save += *pcl_temp_save_filtered;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }


}

void publish_path(const ros::Publisher &pubodom, const ros::Publisher &pubpath)
{
    nav_msgs::Odometry corrected_odom;
    nav_msgs::Path corrected_path;

    corrected_path.header.frame_id = "camera_init";
    update_pose_mutex.lock();
    for(int i=0; i<recent_index ; i++)
    {
        frame_pose &p  = frames[i];

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "camera_init";
        pose_stamped.header.stamp = ros::Time::now();

        pose_stamped.pose.position.x = p.corrected_matrix(0, 3);
        pose_stamped.pose.position.y = p.corrected_matrix(1, 3);
        pose_stamped.pose.position.z = p.corrected_matrix(2, 3);

        // Set the orientation from the transformation matrix
        Eigen::Matrix3d rotation_matrix = p.corrected_matrix.block<3, 3>(0, 0);
        Eigen::Quaterniond quaternion(rotation_matrix);
        pose_stamped.pose.orientation.x = quaternion.x();
        pose_stamped.pose.orientation.y = quaternion.y();
        pose_stamped.pose.orientation.z = quaternion.z();
        pose_stamped.pose.orientation.w = quaternion.w();       
        
        //publish
        corrected_path.poses.push_back(pose_stamped);

        if(i ==recent_index-1)
        {
            corrected_odom.header.frame_id = "camera_init";
            corrected_odom.header.stamp = ros::Time::now();

            corrected_odom.pose.pose.position.x = p.corrected_matrix(0, 3);
            corrected_odom.pose.pose.position.y = p.corrected_matrix(1, 3);
            corrected_odom.pose.pose.position.z = p.corrected_matrix(2, 3);

            corrected_odom.pose.pose.orientation.x = quaternion.x();
            corrected_odom.pose.pose.orientation.y = quaternion.y();
            corrected_odom.pose.pose.orientation.z = quaternion.z();
            corrected_odom.pose.pose.orientation.w = quaternion.w();
        }
    }

    update_pose_mutex.unlock();
    pubodom.publish(corrected_odom);
    pubpath.publish(corrected_path);
}

void path_thread()
{
    ros::Rate rate(10);
    while(ros::ok())
    {
        rate.sleep();
        publish_path(pubodom, pubpath);
    }
}

void isam_thread()
{
    ros::Rate rate(1);
    while(ros::ok())
    {
        rate.sleep();
        isam_update();
        
    }
}

void key_thread()
{
    ros::Rate rate(1);
    while(ros::ok())
    {
        rate.sleep();
        key_select();
    }
}

void loop_thread()
{
    ros::Rate rate(1.0);
    while(ros::ok()){
        rate.sleep();
        loop_closed = perform_LC();
    }
}

void loop_vis_thread()
{
    ros::Rate rate(0.1);
    while(ros::ok()){
        
        rate.sleep();
        publish_loop_markers();

    }
}

void vis_points_thread()
{
    ros::Rate rate(0.1);
    while (ros::ok())
    {
        rate.sleep();
        publish_colored_corrected_cloud(pubcolorlaser);
    }
    
}

void synchronizedCallback(const sensor_msgs::PointCloud2ConstPtr &pointcloud, const sensor_msgs::CompressedImage::ConstPtr& image,const nav_msgs::Odometry::ConstPtr& odom)
{
    // position 
    double pos_x = odom->pose.pose.position.x;
    double pos_y = odom->pose.pose.position.y;
    double pos_z = odom->pose.pose.position.z;

    // orientation
    double ori_x = odom->pose.pose.orientation.x;
    double ori_y = odom->pose.pose.orientation.y;
    double ori_z = odom->pose.pose.orientation.z;
    double ori_w = odom->pose.pose.orientation.w;

    // Eigen R/T transformation
    Eigen::Quaterniond q(ori_w, ori_x, ori_y, ori_z);
    Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    transformation_matrix.block<3,3>(0,0) = rotation_matrix;
    transformation_matrix(0,3) = pos_x;
    transformation_matrix(1,3) = pos_y;
    transformation_matrix(2,3) = pos_z;



    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr->image;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
        pcl::fromROSMsg(*pointcloud, *pcl_cloud);

        /*pcl::PointCloud<pcl::PointXYZINormal>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
        sor.setInputCloud(pcl_cloud);
        sor.setMeanK(100);
        sor.setStddevMulThresh(0.5);
        sor.filter(*filtered_cloud);*/ //noise remove code

        frame_pose current_frame(img,transformation_matrix, pcl_cloud);
        
        int size = current_frame.lidar_cloud->points.size();
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr robot_cloud(new pcl::PointCloud<pcl::PointXYZINormal>(size,1));
        for(int i=0; i<size;i++)
        {
            World_TO_ROBOT(&pcl_cloud->points[i], &robot_cloud->points[i], transformation_matrix);

        }
        voxelize_map(robot_cloud);
        current_frame.robot_frame_cloud = robot_cloud;

        
        frames.push_back(current_frame);      

        ROS_DEBUG("Image timestamp: %f", image->header.stamp.toSec());
        ROS_DEBUG("PointCloud timestamp: %f", pointcloud->header.stamp.toSec());
        ROS_DEBUG("Odometry timestamp: %f", odom->header.stamp.toSec());
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    make_gtsam();
    //publish_colored_cloud(pubcolorlaser);

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "snu_colorization");
    ros::NodeHandle nh;
    //ros::Rate rate(5000);

    //parameters
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, true);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    //camera parameters
    nh.param<vector<double>>("camera/ic_extrinsic", extrinsic_matrix_vector, vector<double>());
    nh.param<double>("camera/k1",k1, -0.35866339052162377 );
    nh.param<double>("camera/k2",k2, 0.14886143788297318 );
    nh.param<double>("camera/p1",p1, 0.0002815532809810967 );
    nh.param<double>("camera/p2",p2, 0.00040207936847531234 );
    nh.param<double>("camera/fx",fx, 731.2592265895066);
    nh.param<double>("camera/fy",fy, 730.07196860144 );
    nh.param<double>("camera/cx",cx, 630.2460232287447 );
    nh.param<double>("camera/cy",cy, 353.5411952863725 );
    nh.param<int>("camera/visualize_uncolored",visualize_uncolored,1);
    nh.param<float>("camera/colored_leaf_size",colored_leaf_size,0.03);
    nh.param<float>("camera/uncolored_leaf_size",uncolored_leaf_size,0.1);
    nh.param<double>("key_threshold",key_thr, 2.0);
    nh.param<double>("loop_distance_candidate",loop_distance_candidate,4.0);
    nh.param<double>("icp_score_threshold", icp_score_thr, 0.5);
    //extrinsic matrix to eigen matrix!

    if (extrinsic_matrix_vector.size() == 16) 
    {
        extrinsic_matrix << extrinsic_matrix_vector[0], extrinsic_matrix_vector[1], extrinsic_matrix_vector[2], extrinsic_matrix_vector[3],
                                extrinsic_matrix_vector[4], extrinsic_matrix_vector[5], extrinsic_matrix_vector[6], extrinsic_matrix_vector[7],
                                extrinsic_matrix_vector[8], extrinsic_matrix_vector[9], extrinsic_matrix_vector[10], extrinsic_matrix_vector[11],
                                extrinsic_matrix_vector[12], extrinsic_matrix_vector[13], extrinsic_matrix_vector[14], extrinsic_matrix_vector[15];
    } else{
            ROS_ERROR("Invalid size for extrinsic matrix vector!");
            // Hanpubcolorlaserdle the error condition here
    }
    std::cout<<extrinsic_matrix<<std::endl;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //gtsam initialization
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam2 = new gtsam::ISAM2(parameters);
    
    //prior Noise setting
    gtsam::Vector priorNoisevector(6);
    priorNoisevector << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise == gtsam::noiseModel::Diagonal::Variances(priorNoisevector);
    
    //odom Noise setting
    gtsam::Vector odometryNoisevector(6);
    odometryNoisevector << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odometryNoise = gtsam::noiseModel::Diagonal::Variances(odometryNoisevector);

    //loop noise setting
    double noisecov = 0.1;
    gtsam::Vector loopNoisevector(6);
    cauchyEstimator = gtsam::noiseModel::mEstimator::Cauchy::Create(1);
    loopNoisevector << noisecov,noisecov,noisecov,noisecov,noisecov,noisecov;
    loopNoise = gtsam::noiseModel::Robust::Create(
        cauchyEstimator,
        gtsam::noiseModel::Diagonal::Variances(loopNoisevector)
    );
    //loopNoise = gtsam::noiseModel::Diagonal::Variances(loopNoisevector);
    

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //publishers
    //pubcolorlaser = nh.advertise<sensor_msgs::PointCloud2>("/colored_cloud",100000);
    pubodom = nh.advertise<nav_msgs::Odometry> ("corrected_odom",100000);
    pubpath = nh.advertise<nav_msgs::Path>("corrected_path",100000);
    pubmarker = nh.advertise<visualization_msgs::Marker>("loop_constraint",100000);
    pubcolorlaser = nh.advertise<sensor_msgs::PointCloud2>("corrected_points",100000);
    //subscribers
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(nh, "/cloud_registered", 1);
    message_filters::Subscriber<sensor_msgs::CompressedImage> image_sub(nh, "/cam0/compressed", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odometry_sub(nh,"/Odometry",1);

    //sync callbacks....
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CompressedImage, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pointcloud_sub, image_sub, odometry_sub);
    sync.registerCallback(boost::bind(&synchronizedCallback, _1, _2, _3));


    std::thread pub_path{path_thread};
    std::thread isam_updating(isam_thread);
    std::thread key_node_select(key_thread);
    std::thread loop_closing(loop_thread);
    std::thread loop_visualize(loop_vis_thread);
    std::thread pub_point(vis_points_thread);
    
    ros::spin();
 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //pcd save!
    if (pointcloud_publish->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        ROS_INFO("Current scan is saved to /PCD/%s\n",file_name.c_str());
        cout << "current scan is saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pointcloud_publish);
    }

    

    return 0;
}
