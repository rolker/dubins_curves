#ifndef DUBINS_PLUGIN_H
#define DUBINS_PLUGIN_H

#include <project11_navigation/interfaces/task_to_task_workflow.h>

namespace dubins_curves
{

class Dubins: public project11_navigation::TaskToTaskWorkflow
{
public:
  void configure(std::string name, project11_navigation::Context::Ptr context) override;
  void setGoal(const project11_navigation::TaskPtr& input) override;
  bool running() override;
  bool getResult(project11_navigation::TaskPtr& output) override;
private:
  project11_navigation::Context::Ptr context_;
  project11_navigation::TaskPtr input_task_;
  project11_navigation::TaskPtr output_task_;

  /// Turn radius in meters
  double radius_ = 10;

  /// Segment length used for turning curves into segments
  double step_size_ = 2;

  std::string output_task_type_ = "follow_trajectory";
  std::string output_task_name_ = "navigation_trajectory";
};

struct SampleContext
{
  std::vector<geometry_msgs::PoseStamped>* pose_vector;
  ros::Time start_time;
  double speed;
  std::string frame_id;
};

/// Callback used to convert a curve to segments
int buildPath(double q[3], double t, void* user_data);


} // namespace dubins_curves

#endif
