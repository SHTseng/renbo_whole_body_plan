#ifndef PLANNING_SERVICE_H_
#define PLANNING_SERVICE_H_

#include <renbo_whole_body_plan/renbo_planner.h>

namespace planner_control
{
class PlanningService
{
public:

  PlanningService();

  ~PlanningService();

  // service servers
  ros::ServiceServer generate_ds_database_;

  ros::ServiceServer get_goal_config_;

  ros::ServiceServer motion_planning_;

  RenboPlanner renbo_planner_;


};

}

#endif
