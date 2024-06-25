#include "lidar.hpp"
#include "gtsam_method.hpp" 

bool lidar_gtsamInit = false; 


Eigen::Matrix4d performICP(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& target_cloud, bool& icp_success)
{
    pcl::IterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    icp.setMaxCorrespondenceDistance(150); 
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    int score;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr Final(new pcl::PointCloud<pcl::PointXYZINormal>());
    icp.align(*Final);
    score = icp.getFitnessScore();
    if(icp.hasConverged() && score < icp_score_thr){
        icp_success = true;
        
        Eigen::Matrix4d matrix = icp.getFinalTransformation().cast<double>();
        


        //std::cout<<"ICP converged"<<std::endl;

        return matrix;
    }else{
        icp_success = false;
        std::cout<<"ICP not converged"<<std::endl;
        Eigen::Matrix4d matrix_failed = Eigen::Matrix4d::Identity();
        return matrix_failed;
    }
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr make_cloud_submap(int index, int submap_size)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZINormal>());
    Eigen::Matrix4d matrix;
    for(int i=-submap_size; i<submap_size; i++)
    {
        int curr = index + i;
        if(curr<0 || curr>=odom_size)
            continue;
        
        *pointcloud += *frames.at(curr).lidar_cloud;
    }
    //matrix = frames.at(index).transformation_matrix;

    //submap_pose pose(index, pointcloud, matrix);
    //std::cout<<"submap is created"<<std::endl;
    //submap_poses.push_back(pose);

    return pointcloud;
}

void voxelize_pcd(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud)
{
    pcl::VoxelGrid<pcl::PointXYZINormal> voxel_pcd;
    voxel_pcd.setLeafSize(0.2,0.2,0.2);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr before_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
    *before_cloud = *cloud;

    voxel_pcd.setInputCloud(before_cloud);
    voxel_pcd.filter(*cloud);

    return;
}

void voxelize_map(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud)
{
    pcl::VoxelGrid<pcl::PointXYZINormal> voxel_pcd;
    voxel_pcd.setLeafSize(0.5,0.5,0.5);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr before_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
    *before_cloud = *cloud;

    voxel_pcd.setInputCloud(before_cloud);
    voxel_pcd.filter(*cloud);

    return;
}


bool keysubmap_check(int prev, int curr)
{
    return key_thr < (frames.at(prev).transformation_matrix.block<3,1>(0,3)-frames.at(curr).transformation_matrix.block<3,1>(0,3)).norm();
}

void make_key_map_index(int index)
{
    key_submap_index.push_back(index);
}

void make_lidar_gtsam()
{
    /*
    if(!lidar_gtsamInit)
    {
        const int init =0; 
        gtsam::Pose3 pose_1 = make_gtsam_pose3(frames.at(init));
        gtsam::Symbol init_lidarkey('l',init);

        pgo_make.lock();
        pgo_graph.add(gtsam::PriorFactor<gtsam::Pose3>(init_lidarkey, pose_1, priorNoise));
        initialEstimate.insert(init_lidarkey,pose_1);
        pgo_make.unlock();

        lidar_gtsamInit = true;

        std::cout<<"lidar pgo start"<<std::endl;


    }else{
        const int prev_node = frames.size()-2;
        const int curr_node = frames.size()-1;

        gtsam::Symbol prev_key('l',prev_node);
        gtsam::Symbol curr_key('l',curr_node);
        gtsam::Pose3 pose_to = make_gtsam_pose3(frames.at(curr_node));

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr source = frames.at(prev_node).lidar_cloud;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr target = frames.at(curr_node).lidar_cloud;

        bool icp_success = false;
        gtsam::Pose3 icp_result = performICP(source, target, icp_success);
        
        pgo_make.lock();
        if(icp_success){
            pgo_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_key,curr_key,icp_result,odometryNoise));
            std::cout<<"lidar pgo successfully inserted"<<std::endl;
        }else{
            std::cout<<"icp converged failed"<<std::endl;
        }
        initialEstimate.insert(curr_key,pose_to);
        pgo_make.unlock();

       
    }*/
}