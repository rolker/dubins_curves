#include <dubins_curves/dubins_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <nav_msgs/Path.h>

extern "C" {
  #include "dubins_curves/dubins.h"
}

PLUGINLIB_EXPORT_CLASS(dubins_curves::Dubins, project11_navigation::TaskToTaskWorkflow);

namespace dubins_curves
{

void Dubins::configure(std::string name, project11_navigation::Context::Ptr context)
{
  context_ = context;
  ROS_INFO_STREAM("Initializing Dubins plugin with name " << name);
  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("step_size", step_size_, step_size_);
  private_nh.param("radius", radius_, radius_);
}

void Dubins::setGoal(const std::shared_ptr<project11_navigation::Task>& input)
{
}

bool Dubins::running()
{
  return false;
}

bool Dubins::getResult(std::shared_ptr<project11_navigation::Task>& output)
{
  return false;  
}

} // namespace dubins_curves
