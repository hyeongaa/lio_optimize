#include "gtsam_method.hpp" 

bool gtsamInit = false;
int odom_size =0;

gtsam::Pose3 make_gtsam_pose3(frame_pose _oneframe)
{   
    Eigen::Matrix4d transformation = _oneframe.transformation_matrix;
    gtsam::Point3 position(transformation(0, 3), transformation(1, 3), transformation(2, 3));
    Eigen::Matrix3d rotation_matrix = transformation.block<3, 3>(0, 0);
    gtsam::Rot3 orientation(rotation_matrix);

    return gtsam::Pose3(orientation, position);

}//frame에서 gtsam pose를 추출

gtsam::Pose3 eig_to_gtsam(Eigen::Matrix4d matrix)
{
    gtsam::Point3 position(matrix(0,3),matrix(1,3),matrix(2,3));
    Eigen::Matrix3d rotation = matrix.block<3,3>(0,0);
    gtsam::Rot3 orientation(rotation);

    return gtsam::Pose3(orientation,position);

}//eigen matrix를 gtsam pose로 변경


void make_gtsam()
{
    if(!gtsamInit)
    {
        const int init =0;
        gtsam::Pose3 pose_1 = make_gtsam_pose3(frames.at(init));
        gtsam::Symbol initkey('o', init);

        key_mutex.lock();
        make_key_map_index(init);
        key_mutex.unlock();

        //making
        pgo_make.lock();
        pgo_graph.add(gtsam::PriorFactor<gtsam::Pose3>(initkey,pose_1,priorNoise));
        initialEstimate.insert(initkey,pose_1);
        pgo_make.unlock();
        ///

        gtsamInit = true;
        odom_size++;

        std::cout<<"pgo make start!"<<std::endl;

    }else
    {
        const int prev_node = frames.size()-2;
        const int curr_node = frames.size()-1;
        
        //is it key frame? if add to the key_submap_index
        key_mutex.lock();
        if(keysubmap_check(key_submap_index.back(), curr_node))
        {
            make_key_map_index(curr_node);
            //key_select();
            //std::cout<<"key frame is added in node: "<<curr_node<<std::endl;
        }
        key_mutex.unlock();

        //graph add factor
        gtsam::Pose3 pose_from = make_gtsam_pose3(frames.at(prev_node));
        gtsam::Pose3 pose_to = make_gtsam_pose3(frames.at(curr_node));
        gtsam::Symbol prev_key('o', prev_node);
        gtsam::Symbol curr_key('o', curr_node);


        pgo_make.lock();
        pgo_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_key,curr_key,pose_from.between(pose_to),odometryNoise));
        initialEstimate.insert(curr_key,pose_to);
        odom_size++;
        pgo_make.unlock();

    }
}

void isam_update()
{
    if(gtsamInit)
    {
        pgo_make.lock();
        isam2->update(pgo_graph,initialEstimate);
        isam2->update();
        isam2->update();
        
        //LOOP CLOSURE
        if(loop_closed)
        {
            isam2->update();
            isam2->update();
            isam2->update();
            std::cout<<"loop closing update is done"<<std::endl;
            loop_closed = false;
        }
        //clear for memory..! ISAM will cover it. 
        pgo_graph.resize(0);
        initialEstimate.clear();
        isam_estimation = isam2->calculateEstimate();

        //update poses
        update_pose_mutex.lock();
        for(int i=0; i< odom_size; i++)
        {
            //frame_pose& p = frames[i];

            gtsam::Pose3 optimized_pose = isam_estimation.at<gtsam::Pose3>(gtsam::Symbol('o',i));

            Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
            transformation_matrix.block<3,3>(0,0) = optimized_pose.rotation().matrix();
            transformation_matrix(0,3) = optimized_pose.translation().x();
            transformation_matrix(1,3) = optimized_pose.translation().y();
            transformation_matrix(2,3) = optimized_pose.translation().z();
            frames[i].corrected_matrix = transformation_matrix;           

        }

        recent_index = int(frames.size()-1);

        update_pose_mutex.unlock();

        pgo_make.unlock();
    }
}