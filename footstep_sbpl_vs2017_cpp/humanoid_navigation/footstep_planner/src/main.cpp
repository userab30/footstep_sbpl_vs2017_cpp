#include <opencv2/opencv.hpp>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "tf/transform_datatypes.h"
//#include <advise_input/footstep_datatype.h>
#include  <footstep_planner/FootstepPlanner.h>




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

	int r = ivFootstepPlanner.LoadMap(envPath + mapName);
	if (r == -1) {
		PRINT_INFO("no map file \n");
		return r;
	}
	cv::Mat imagew_binaryMap = ivFootstepPlanner.watch_binaryMap;
	cv::Mat imagew_distMap = ivFootstepPlanner.watch_distMap;

	
	ivFootstepPlanner.LoadStartPose(envPath + startName);
	ivFootstepPlanner.LoadGoalPose(envPath + goalName);


	return 0;
}