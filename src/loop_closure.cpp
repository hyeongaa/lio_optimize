#include "lidar.hpp"
#include "gtsam_method.hpp" 
#include "loop_closure.hpp"

void publish_loop_markers()
{
    visualization_msgs::Marker edges_;
    //edges_.type = 5u;
    edges_.type = 5u;

    edges_.scale.x = 0.12f; edges_.header.frame_id = "camera_init"; edges_.pose.orientation.w = 1.0f;
    edges_.color.r = 1.0f; edges_.color.g = 1.0f; edges_.color.b = 1.0f; edges_.color.a = 1.0f;

    edges_.points.clear(); // Clear previous points if any

    int loop_size = loop_closed_pair.size();
    
    if(loop_size==0) return;

    for(int i=0 ; i<loop_size;i++)
    {
        int _first = loop_closed_pair.at(i).first;
        int _second = loop_closed_pair.at(i).second;

        geometry_msgs::Point p1,p2;
        frame_pose& p1_pose = frames[_first];
        frame_pose& p2_pose = frames[_second];

        p1.x = p1_pose.corrected_matrix(0,3);
        p1.y = p1_pose.corrected_matrix(1,3);
        p1.z = p1_pose.corrected_matrix(2,3);

        p2.x = p2_pose.corrected_matrix(0,3);
        p2.y = p2_pose.corrected_matrix(1,3);
        p2.z = p2_pose.corrected_matrix(2,3);

        edges_.points.push_back(p1);
        edges_.points.push_back(p2);
    }

  


    pubmarker.publish(edges_);

}



void key_select()
{
    int size = key_submap_index.size();
    if(size<2) return;

    int stand_index = size -1;
    int close_index = -1;
    double distance = loop_distance_candidate;

    for(int i =0; i< size-1;i++)
    {
        double tmp = (frames.at(key_submap_index.at(i)).transformation_matrix.block<3,1>(0,3)-frames.at(key_submap_index.at(stand_index)).transformation_matrix.block<3,1>(0,3)).norm();
        if(tmp< key_thr){
            if(tmp<distance){
                distance = tmp;
                close_index = i;
            }
        }
    }

    if(close_index>0)
    {
        node_selection.push_back(std::pair<int,int>(key_submap_index.at(close_index),key_submap_index.at(stand_index)));
        std::cout<< "node "<<key_submap_index.at(close_index)<< " and " << key_submap_index.at(stand_index)<<" is paired"<<std::endl;

    }


}


bool perform_LC()
{
    bool update = false;
    while(!node_selection.empty())
    {
        key_mutex.lock();
        int source = node_selection.back().first;
        int target = node_selection.back().second;
        key_mutex.unlock();

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud = make_cloud_submap(source,10);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_cloud = make_cloud_submap(target,10);

        voxelize_pcd(source_cloud);
        voxelize_pcd(target_cloud);

        bool icp_success = false;
        Eigen::Matrix4d relative_pose = performICP(source_cloud, target_cloud, icp_success);
    
        if(icp_success){
            gtsam::Symbol source_key('o',source);
            gtsam::Symbol target_key('o',target);

            gtsam::Pose3 pose_from = eig_to_gtsam(relative_pose*frames.at(source).transformation_matrix);
            gtsam::Pose3 pose_to = make_gtsam_pose3(frames.at(target));

            pgo_make.lock();
            pgo_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(source_key, target_key, pose_from.between(pose_to), odometryNoise));
            pgo_make.unlock();

            update = true;

            std::pair<int,int> loop_pair;
            loop_pair.first = source;
            loop_pair.second = target;
            loop_closed_pair.push_back(loop_pair);

            std::cout<< "loop is paired between "<<source<<" and "<<target<<std::endl;
        }

        node_selection.pop_back();        
    }

    return update;
}

