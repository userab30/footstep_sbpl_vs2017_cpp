#include <opencv2/opencv.hpp>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "tf/transform_datatypes.h"
//#include <advise_input/footstep_datatype.h>
#include  <footstep_planner/FootstepPlanner.h>
#include "gridmap_2d/GridMap2D.h"



using namespace cv;
using namespace std;
using namespace footstep_planner;


int main(int argc, char** argv)
{
	string yamlPath = "..\\footstep_sbpl_vs2017_cpp\\humanoid_navigation\\footstep_planner\\config\\";
	string fileName = "planning_params_all.yaml";
	//environment_params ivEnvironmentParams;
	footstep_planner::FootstepPlanner ivFootstepPlanner(yamlPath, fileName);

	//getFileParam(ivEnvironmentParams, ivFootstepPlanner, yamlPath, fileName);



	string envPath = "..\\document\\";
	string mapName = "env_map.yaml";
	string startName = "env_start.yaml";
	string goalName = "env_goal.yaml";

	gridmap_2d::GridMap2D gridMap;
	gridMap.getMap(0, envPath, mapName);
	
	//StartGoal::StartGoalInfo goalInfo;
	//goalInfo.getGoal(envPath, goalName);
	//PRINT_INFO("Goal pose set to (%f %f %f)\n", goalInfo.position.x, goalInfo.position.y, goalInfo.theta);

	//StartGoal::StartGoalInfo startInfo;
	//startInfo.getStart(envPath, startName);
	//PRINT_INFO("Start pose set to (%f %f %f)\n", startInfo.position.x, startInfo.position.y, startInfo.theta);


	//FootstepPlanner ivFootstepPlanner;
	return 0;
}