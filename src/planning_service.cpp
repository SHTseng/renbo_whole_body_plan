#include <renbo_whole_body_plan/planning_service.h>

namespace planner_control
{

PlanningService::PlanningService()
{
  ros::NodeHandle nh;

  generate_ds_database_ = nh.advertiseService("generate_ds_database", &RenboPlanner::generate_ds_database, &renbo_planner_);

}

PlanningService::~PlanningService()
{

}

}
