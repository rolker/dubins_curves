#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseArray.h>
extern "C" {
    #include "dubins.h"
}
#include "dubins_curves/DubinsCurves.h"
#include <iostream>

int buildPath(double q[3], double t, void* user_data)
{
    dubins_curves::DubinsCurves::Response* res = reinterpret_cast<dubins_curves::DubinsCurves::Response*>(user_data);
    
    geometry_msgs::Pose pose;
    tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, q[2]);
    quaternionTFToMsg(quat, pose.orientation);
    
    pose.position.x = q[0];
    pose.position.y = q[1];
    
    res->path.poses.push_back(pose);
    
    return 0;
}

bool dubinsCurvesService(dubins_curves::DubinsCurves::Request &req, dubins_curves::DubinsCurves::Response &res)
{
    //std::cerr << req << std::endl;
    double rho = req.radius;
    //std::cerr << "radius: " << rho << std::endl;
    
    tf::Quaternion q0(req.startPose.orientation.x,
                     req.startPose.orientation.y,
                     req.startPose.orientation.z,
                     req.startPose.orientation.w);
    tf::Matrix3x3 m0(q0);
    double roll, pitch, yaw;
    m0.getRPY(roll, pitch, yaw);
    
    //std::cerr << "roll, pitch, yaw: " << roll << ", " << pitch << ", " << yaw << std::endl;
    
    double start[3];
    start[0] = req.startPose.position.x;
    start[1] = req.startPose.position.y;
    start[2] = yaw;
    
    tf::Quaternion q1(req.targetPose.orientation.x,
                     req.targetPose.orientation.y,
                     req.targetPose.orientation.z,
                     req.targetPose.orientation.w);
    tf::Matrix3x3 m1(q1);
    m1.getRPY(roll, pitch, yaw);
    //std::cerr << "roll, pitch, yaw: " << roll << ", " << pitch << ", " << yaw << std::endl;
    
    double target[3];
    target[0] = req.targetPose.position.x;
    target[1] = req.targetPose.position.y;
    target[2] = yaw;

    DubinsPath path;
    
    int dubins_ret = dubins_shortest_path(&path, start, target, rho);
    
    //std::cerr << "Dubins return value: " << dubins_ret << std::endl;
    
    //std::cerr << "type: " << path.type << " rho: " << path.rho << " seg lengths: " << path.param[0] << ", " << path.param[1] << ", " << path.param[2] << std::endl;
    
    if(dubins_ret == 0)
    {
        dubins_ret = dubins_path_sample_many(&path, req.samplingInterval, buildPath, &res);
        if(dubins_ret == 0)
            res.success = true;
//         if(res.path.poses.size() >= 5)
//         {
//             for(int i = 0; i < 5; ++i)
//                 std::cerr << i << ": " << res.path.poses[i] << std::endl;
//             for(int i = res.path.poses.size()-5; i < res.path.poses.size(); ++i)
//                 std::cerr << i << ": " << res.path.poses[i] << std::endl;
//         }
    }
    
    return true;    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "project11_transformation_node");
    
    ros::NodeHandle node;

    ros::ServiceServer service = node.advertiseService("dubins_curves",dubinsCurvesService);

    ros::spin();
    return 0;
}
