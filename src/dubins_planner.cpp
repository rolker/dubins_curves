#include <dubins_curves/dubins_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <nav_msgs/Path.h>

extern "C" {
  #include "dubins_curves/dubins.h"
}

PLUGINLIB_EXPORT_CLASS(dubins_planner::DubinsPlanner, nav_core::BaseGlobalPlanner)

namespace dubins_planner
{

DubinsPlanner::DubinsPlanner()
{

}

DubinsPlanner::DubinsPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}

DubinsPlanner::~DubinsPlanner()
{

}

void DubinsPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  ROS_INFO_STREAM("Initializing dubins_planner with name " << name);
  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("step_size", step_size_, step_size_);
  private_nh.param("radius", radius_, radius_);
  path_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 10, true);
}

bool DubinsPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
  if(start.header.frame_id != goal.header.frame_id)
  {
    ROS_ERROR_STREAM_THROTTLE(2.0, "frame_id missmatch. start: " << start.header.frame_id << " goal: " << goal.header.frame_id);
    return false;
  }

  double start_array[3];
  start_array[0] = start.pose.position.x;
  start_array[1] = start.pose.position.y;
  start_array[2] = tf2::getYaw(start.pose.orientation);

  double goal_array[3];
  goal_array[0] = goal.pose.position.x;
  goal_array[1] = goal.pose.position.y;
  goal_array[2] = tf2::getYaw(goal.pose.orientation);

  DubinsPath path;
    
  int dubins_ret = dubins_shortest_path(&path, start_array, goal_array, radius_);

  if(dubins_ret != 0)
  {
    ROS_ERROR_STREAM_THROTTLE(2.0, "Error finding Dubin's shortest path");
    return false;
  }

  dubins_ret = dubins_path_sample_many(&path, step_size_, buildPath, &plan);
  if(dubins_ret != 0)
  {
    ROS_ERROR_STREAM_THROTTLE(2.0, "Error sampling Dubin's path");
    return false;
  }

  ROS_INFO_STREAM("start time: " << start.header.stamp << " goal time: " << goal.header.stamp);
  double timespan = (goal.header.stamp - start.header.stamp).toSec();
  ROS_INFO_STREAM("timespan: " << timespan);
  double path_length = dubins_path_length(&path);
  ROS_INFO_STREAM("path length: " << path_length);
  for(auto& p: plan)
  {
    p.header.frame_id = start.header.frame_id;
    p.header.stamp = start.header.stamp + ros::Duration(timespan*p.header.stamp.toSec()/path_length);
  }

  nav_msgs::Path output_path;
  if(!plan.empty())
    output_path.header = plan.front().header;
  for(auto p: plan)
    output_path.poses.push_back(p);
  path_pub_.publish(output_path);

  return true;
}


int buildPath(double q[3], double t, void* user_data)
{
  std::vector<geometry_msgs::PoseStamped>* pose_vector = reinterpret_cast<std::vector<geometry_msgs::PoseStamped>*>(user_data);
    
  geometry_msgs::PoseStamped pose;
  tf2::Quaternion quat(tf2::Vector3(0,0,1), q[2]);
  pose.pose.orientation = tf2::toMsg(quat);
    
  pose.pose.position.x = q[0];
  pose.pose.position.y = q[1];

  pose.header.stamp.fromSec(t); // t is distance so effectivly 1 m/s
    
  pose_vector->push_back(pose);
    
  return 0;
}


}