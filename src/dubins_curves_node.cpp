#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseArray.h>
extern "C" {
    #include "dubins.h"
}
#include "dubins_curves/DubinsCurves.h"
#include "dubins_curves/DubinsCurvesLatLong.h"
#include <iostream>
#include "project11/utils.h"

namespace p11 = project11;

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
    double rho = req.radius;
    
    tf::Quaternion q0(req.startPose.orientation.x,
                     req.startPose.orientation.y,
                     req.startPose.orientation.z,
                     req.startPose.orientation.w);
    tf::Matrix3x3 m0(q0);
    double roll, pitch, yaw;
    m0.getRPY(roll, pitch, yaw);
    
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
    
    double target[3];
    target[0] = req.targetPose.position.x;
    target[1] = req.targetPose.position.y;
    target[2] = yaw;

    DubinsPath path;
    
    int dubins_ret = dubins_shortest_path(&path, start, target, rho);
    
    if(dubins_ret == 0)
    {
        dubins_ret = dubins_path_sample_many(&path, req.samplingInterval, buildPath, &res);
        if(dubins_ret == 0)
            res.success = true;
    }
    
    return true;    
}

struct ResponseAndENU
{
    dubins_curves::DubinsCurvesLatLong::Response* res;
    gz4d::LocalENU* localENU;
};

int buildPathLatLong(double q[3], double t, void* user_data)
{
    ResponseAndENU * ret = reinterpret_cast<ResponseAndENU*>(user_data);
    
    geographic_msgs::GeoPose pose;
    tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, q[2]);
    quaternionTFToMsg(quat, pose.orientation);
    
    gz4d::Point<double> local(q[0],q[1],0.0);
    auto local_ll = ret->localENU->toLatLong(local);
    
    pose.position.latitude = local_ll[0];
    pose.position.longitude = local_ll[1];
    
    ret->res->path.push_back(pose);
    
    return 0;
}

bool dubinsCurvesServiceLatLong(dubins_curves::DubinsCurvesLatLong::Request &req, dubins_curves::DubinsCurvesLatLong::Response &res)
{
    double rho = req.radius;
    
    tf::Quaternion q0(req.startGeoPose.orientation.x,
                     req.startGeoPose.orientation.y,
                     req.startGeoPose.orientation.z,
                     req.startGeoPose.orientation.w);
    tf::Matrix3x3 m0(q0);
    double roll, pitch, yaw;
    m0.getRPY(roll, pitch, yaw);
    
    p11::LatLongDegrees start_ll;
    p11::fromMsg(req.startGeoPose.position, start_ll);
    start_ll.altitude() = 0.0;
    p11::ENUFrame geoReference(start_ll);
    auto start_local = geoReference.toLocal(start_ll);
    
    double start[3];
    start[0] = start_local[0];
    start[1] = start_local[1];
    start[2] = yaw;
    
    tf::Quaternion q1(req.targetGeoPose.orientation.x,
                     req.targetGeoPose.orientation.y,
                     req.targetGeoPose.orientation.z,
                     req.targetGeoPose.orientation.w);
    tf::Matrix3x3 m1(q1);
    m1.getRPY(roll, pitch, yaw);

    p11::LatLongDegrees target_ll(req.targetGeoPose.position.latitude, req.targetGeoPose.position.longitude,0.0);
    auto target_local = geoReference.toLocal(target_ll);
    double target[3];
    target[0] = target_local[0];
    target[1] = target_local[1];
    target[2] = yaw;

    DubinsPath path;
    
    int dubins_ret = dubins_shortest_path(&path, start, target, rho);
    
    
    if(dubins_ret == 0)
    {
        ResponseAndENU response;
        response.localENU = &geoReference;
        response.res = &res;
        dubins_ret = dubins_path_sample_many(&path, req.samplingInterval, buildPathLatLong, &response);
        if(dubins_ret == 0)
            res.success = true;
    }
    
    return true;    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dubins_curves");
    
    ros::NodeHandle node;

    ros::ServiceServer service = node.advertiseService("dubins_curves",dubinsCurvesService);
    ros::ServiceServer service_ll = node.advertiseService("dubins_curves_latlong",dubinsCurvesServiceLatLong);

    ros::spin();
    return 0;
}
