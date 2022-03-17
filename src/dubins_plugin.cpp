#include <dubins_curves/dubins_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>

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
  ros::NodeHandle nh("~/" + name);
  nh.param("step_size", step_size_, step_size_);
  nh.param("radius", radius_, radius_);
  nh.param("output_task_type", output_task_type_, output_task_type_);
  nh.param("output_task_name", output_task_name_, output_task_name_);
}

void Dubins::setGoal(const std::shared_ptr<project11_navigation::Task>& input)
{
  input_task_ = input;
}

bool Dubins::running()
{
  return false;
}

bool Dubins::getResult(std::shared_ptr<project11_navigation::Task>& output)
{
  if(input_task_)
  {
    auto caps = context_->getRobotCapabilities();

    double speed = caps.default_velocity.linear.x;
    double radius = caps.getTurnRadiusAtSpeed(speed);
    if(radius <= 0)
      radius = radius_;

    auto data = input_task_->data();
    if(data["dubins_radius"])
      radius = data["dubins_radius"].as<double>();
    if(data["speed"])
      speed = data["speed"].as<double>();
    double step_size = step_size_;
    if(data["dubins_sample_interval"])
      step_size = data["dubins_sample_interval"].as<double>();

    project11_nav_msgs::CurvedTrajectory curved_trajectory;
    std::vector<geometry_msgs::PoseStamped> sampled_trajectory;

    const project11_nav_msgs::Task& msg = input_task_->message();
    if(msg.poses.size() < 2)
      return false;

    ros::Time current_start_time = ros::Time::now();
    curved_trajectory.start = msg.poses.front();
    if(!curved_trajectory.start.header.stamp.isValid())
      curved_trajectory.start.header.stamp = current_start_time;
    for(auto p = msg.poses.begin(); p != msg.poses.end(); p++)
    {
      auto next_p = p;
      next_p++;
      if (next_p != msg.poses.end())
      {
        double start_array[3];
        start_array[0] = p->pose.position.x;
        start_array[1] = p->pose.position.y;
        start_array[2] = tf2::getYaw(p->pose.orientation);

        double goal_array[3];
        goal_array[0] = next_p->pose.position.x;
        goal_array[1] = next_p->pose.position.y;
        goal_array[2] = tf2::getYaw(next_p->pose.orientation);

        DubinsPath path;
    
        int dubins_ret = dubins_shortest_path(&path, start_array, goal_array, radius);

        if(dubins_ret != 0)
        {
          ROS_ERROR_STREAM_THROTTLE(2.0, "Error finding Dubin's shortest path");
          return false;
        }

        double total_distance = path.param[0] + path.param[1] + path.param[2];

        // if input pose timestamp make sense, use them, otherwise calculate times
        // from target speed and distances
        if(p->header.stamp.isValid())
          current_start_time = p->header.stamp;
        auto current_end_time = next_p->header.stamp;
        auto delta_t = current_end_time - current_start_time;
        if(delta_t > ros::Duration(0))
          speed = total_distance/delta_t.toSec();
        else
          current_end_time = current_start_time + ros::Duration(total_distance/speed);

        project11_nav_msgs::Curve c1;
        c1.length = path.param[0];
        c1.arrival_time = current_start_time + ros::Duration(c1.length/speed);
        c1.radius = radius;
        switch(path.type)
        {
          case LSL:
          case LSR:
          case LRL:
            c1.direction = c1.DIRECTION_LEFT;
            break;
          default:
            c1.direction = c1.DIRECTION_RIGHT;
        }
        curved_trajectory.curves.push_back(c1);

        project11_nav_msgs::Curve c2;
        c2.length = path.param[1];
        c2.arrival_time = c1.arrival_time + ros::Duration(c2.length/speed);
        c2.radius = radius;
        switch(path.type)
        {
          case RLR:
            c2.direction = c2.DIRECTION_LEFT;
            break;
          case LRL:
            c2.direction = c2.DIRECTION_RIGHT;
            break;
          default:
            c2.direction = c2.DIRECTION_STRAIGHT;
        }
        curved_trajectory.curves.push_back(c2);

        project11_nav_msgs::Curve c3;
        c3.length = path.param[2];
        c3.arrival_time = c2.arrival_time + ros::Duration(c3.length/speed);
        c3.radius = radius;
        switch(path.type)
        {
          case LSL:
          case LRL:
          case RSL:
            c3.direction = c3.DIRECTION_LEFT;
            break;
          default:
            c3.direction = c3.DIRECTION_RIGHT;
        }
        curved_trajectory.curves.push_back(c3);
        curved_trajectory.goal = *next_p;
        curved_trajectory.goal.header.stamp = current_end_time;

        if(step_size > 0.0)
        {
          SampleContext c;
          c.pose_vector = &sampled_trajectory;
          c.start_time = current_start_time;
          c.speed = speed;

          dubins_ret = dubins_path_sample_many(&path, step_size, buildPath, &c);
          if(dubins_ret != 0)
          {
            ROS_ERROR_STREAM_THROTTLE(2.0, "Error sampling Dubin's path");
          }
        }

        current_start_time = current_end_time;
      }
    }
    output = input_task_->getFirstChildOfTypeAndIDOrCreate(output_task_type_, output_task_name_);
    auto out_msg = output->message();
    out_msg.curved_trajectories.clear();
    out_msg.curved_trajectories.push_back(curved_trajectory);
    out_msg.poses = sampled_trajectory;
    output->update(out_msg);
    return true;

  }
  return false;
}


int buildPath(double q[3], double t, void* user_data)
{
  SampleContext* c = reinterpret_cast<SampleContext*>(user_data);
    
  geometry_msgs::PoseStamped pose;
  tf2::Quaternion quat(tf2::Vector3(0,0,1), q[2]);
  pose.pose.orientation = tf2::toMsg(quat);
    
  pose.pose.position.x = q[0];
  pose.pose.position.y = q[1];

  pose.header.stamp  = c->start_time + ros::Duration(t/c->speed);
  c->pose_vector->push_back(pose);
   
  return 0;
}

} // namespace dubins_curves
