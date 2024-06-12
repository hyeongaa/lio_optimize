#include "gtsam_method.hpp" 

extern gtsam::NonlinearFactorGraph pgo_graph;
extern gtsam::Values initialEstimate;
extern gtsam::ISAM2 *isam2;
extern gtsam::Values isam_estimation;

bool gtsamInit = false;

extern gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
extern gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;

extern std::vector<frame_pose> frames;
extern std::vector<frame_pose> updated_frames;

extern std::mutex pgo_make;
extern std::mutex update_pose_mutex;
extern int recent_index;

gtsam::Pose3 make_gtsam_pose3(frame_pose _oneframe)
{   
    // 변환 행렬 추출
    Eigen::Matrix4d transformation = _oneframe.transformation_matrix;

    // 위치 추출
    gtsam::Point3 position(transformation(0, 3), transformation(1, 3), transformation(2, 3));

    // 회전 행렬 추출
    Eigen::Matrix3d rotation_matrix = transformation.block<3, 3>(0, 0);

    // Rot3 객체 생성 (Eigen::Matrix3d를 사용하여)
    gtsam::Rot3 orientation(rotation_matrix);

    // Pose3 객체 생성 및 반환
    return gtsam::Pose3(orientation, position);
}


void make_gtsam()
{
    if(!gtsamInit)
    {
        const int init =0;
        gtsam::Pose3 pose_1 = make_gtsam_pose3(frames.at(init));
        
        //making
        pgo_make.lock();
        pgo_graph.add(gtsam::PriorFactor<gtsam::Pose3>(init,pose_1,priorNoise));
        initialEstimate.insert(init,pose_1);
        pgo_make.unlock();
        ///

        gtsamInit = true;

        std::cout<<"pgo make start!"<<std::endl;

    }else
    {
        const int prev_node = frames.size()-2;
        const int curr_node = frames.size()-1;

        gtsam::Pose3 pose_from = make_gtsam_pose3(frames.at(prev_node));
        gtsam::Pose3 pose_to = make_gtsam_pose3(frames.at(curr_node));

        //making
        pgo_make.lock();
        pgo_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node,curr_node,pose_from.between(pose_to),odometryNoise));
        initialEstimate.insert(curr_node,pose_to);
        pgo_make.unlock();
        ////
        //std::cout<<"pgo node: "<<curr_node<<"is inserted"<<std::endl;

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
        std::cout<<"isam update is done"<<std::endl;
        
        //clear for memory..! ISAM will cover it. 
        pgo_graph.resize(0);
        initialEstimate.clear();
        isam_estimation = isam2->calculateEstimate();

        //update poses
        update_pose_mutex.lock();
        for(int i=0; i< int(isam_estimation.size()); i++)
        {
            frame_pose& p = frames[i];

            gtsam::Pose3 optimized_pose = isam_estimation.at<gtsam::Pose3>(i);

            Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
            transformation_matrix.block<3,3>(0,0) = optimized_pose.rotation().matrix();
            transformation_matrix(0,3) = optimized_pose.translation().x();
            transformation_matrix(1,3) = optimized_pose.translation().y();
            transformation_matrix(2,3) = optimized_pose.translation().z();
            p.transformation_matrix = transformation_matrix;           

        }

        recent_index = int(frames.size()-1);

        update_pose_mutex.unlock();

        pgo_make.unlock();
    }
}