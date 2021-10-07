#ifndef DUBBINS_PLANNER_H
#define DUBBINS_PLANNER_H

#include <nav_core/base_global_planner.h>

namespace dubins_planner
{

/// Generates a Dubin's path to the goal ignoring all obstacles.
class DubinsPlanner: public nav_core::BaseGlobalPlanner
{
public:
  DubinsPlanner();

  DubinsPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  ~DubinsPlanner();
  
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

  bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) override;

private:

  /// Turn radius in meters
  double radius_ = 10;

  /// Segment length used for turning curves into segments
  double step_size_ = 2;

  ros::Publisher path_pub_;
};


/// Callback used to convert a curve to segments
int buildPath(double q[3], double t, void* user_data);

}


#endif
