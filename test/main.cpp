#include <opencv2/opencv.hpp>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "tf/transform_datatypes.h"
#include "footstep_datatype.h"

using namespace cv;
using namespace std;


int getFileParam(environment_params &ivEnvironmentParams, footstep_planner::FootstepPlanner &ivFootstepPlanner,std::string yamlPath, std::string fileName)
{

	std::string heuristic_type;
	double diff_angle_cost;

	cv::FileStorage fs_param;
	fs_param.open(yamlPath + fileName, cv::FileStorage::READ);
	if (!fs_param.isOpened())
	{
		std::cout << fileName << ": No file!" << std::endl;
		return -1;
	}


	heuristic_type = fs_param["heuristic_type"];
	ivEnvironmentParams.heuristic_scale = 1;// fs_param["heuristic_scale"];
	ivEnvironmentParams.hash_table_size = fs_param["max_hash_size"];
	ivEnvironmentParams.collision_check_accuracy = fs_param["accuracy"]["collision_check"];
	ivEnvironmentParams.cell_size = fs_param["accuracy"]["cell_size"];
	ivEnvironmentParams.num_angle_bins = fs_param["accuracy"]["num_angle_bins"];

	ivEnvironmentParams.step_cost = fs_param["step_cost"];
	diff_angle_cost = fs_param["diff_angle_cost"];

	ivFootstepPlanner.ivPlannerType = fs_param["planner_type"];
	ivFootstepPlanner.ivSearchUntilFirstSolution = fs_param["search_until_first_solution"]=="True";
	ivFootstepPlanner.ivMaxSearchTime = fs_param["allocated_time"];
	ivEnvironmentParams.forward_search = fs_param["forward_search"] == "True";
	ivFootstepPlanner.ivInitialEpsilon = fs_param["initial_epsilon"];
	ivFootstepPlanner.ivChangedCellsLimit = fs_param["changed_cells_limit"];
	ivEnvironmentParams.num_random_nodes = 20;// fs_param["num_random_nodes"];
	ivEnvironmentParams.random_node_distance = 1.0;// fs_param["random_node_dist"];

	// footstep settings
	ivEnvironmentParams.footsize_x = fs_param["foot"]["size"]["x"];
	ivEnvironmentParams.footsize_y = fs_param["foot"]["size"]["y"];
	ivEnvironmentParams.footsize_z = fs_param["foot"]["size"]["z"];
	ivFootstepPlanner.ivFootSeparation = fs_param["foot"]["separation"];
	ivEnvironmentParams.foot_origin_shift_x = fs_param["foot"]["origin_shift"]["x"];
	ivEnvironmentParams.foot_origin_shift_y = fs_param["foot"]["origin_shift"]["y"];
	ivEnvironmentParams.max_footstep_x = fs_param["foot"]["max"]["step"]["x"];
	ivEnvironmentParams.max_footstep_y = fs_param["foot"]["max"]["step"]["y"];
	ivEnvironmentParams.max_footstep_theta = fs_param["foot"]["max"]["step"]["theta"];
	ivEnvironmentParams.max_inverse_footstep_x = fs_param["foot"]["max"]["inverse"]["step"]["x"];
	ivEnvironmentParams.max_inverse_footstep_y = fs_param["foot"]["max"]["inverse"]["step"]["y"];
	ivEnvironmentParams.max_inverse_footstep_theta = fs_param["foot"]["max"]["inverse"]["step"]["theta"];
	fs_param.release();

	// footstep discretization  std::vector 
	double footsteps_x[] = { 0.00, 0.22, 0.00, -0.08, 0.12, 0.15, 0.08, -0.04, -0.10, 0.00, 0.15, 0.12, 0.12, 0.06 };
	double footsteps_y[] = { 0.14, 0.14, 0.26, 0.12, 0.22, 0.11, 0.22, 0.22, 0.14, 0.12, 0.14, 0.12, 0.18, 0.14 };
	double footsteps_theta[] = { 0.00, 0.00, 0.00, 0.70, 0.30,-0.40, 0.00, 0.30, 0.00, 0.00, 0.00, 0.00, 0.00,-0.25 };

	//XmlRpc::XmlRpcValue footsteps_x;
	//XmlRpc::XmlRpcValue footsteps_y;
	//XmlRpc::XmlRpcValue footsteps_theta;
	//nh_private.getParam("footsteps/x", );
	//nh_private.getParam("footsteps/y", footsteps_y);
	//nh_private.getParam("footsteps/theta", footsteps_theta);
	//if (footsteps_x.getType() != XmlRpc::XmlRpcValue::TypeArray)
	//	ROS_ERROR("Error reading footsteps/x from config file.");
	//if (footsteps_y.getType() != XmlRpc::XmlRpcValue::TypeArray)
	//	ROS_ERROR("Error reading footsteps/y from config file.");
	//if (footsteps_theta.getType() != XmlRpc::XmlRpcValue::TypeArray)
	//	ROS_ERROR("Error reading footsteps/theta from config file.");
	int size_x = length(footsteps_x);
	int size_y = length(footsteps_y);
	int size_t = length(footsteps_theta);
	if (size_x != size_y || size_x != size_t)
	{
		//ROS_ERROR("Footstep parameterization has different sizes for x/y/theta. Exit!");
		exit(2);
	}
	//////-------------------------flag------------------------------/////
	// create footstep set
	//ivEnvironmentParams.footstep_set.clear();
	double max_step_width = 0;
	for (int i = 0; i < length(footsteps_x); ++i)
	{
		double x = (double)footsteps_x[i];
		double y = (double)footsteps_y[i];
		double theta = (double)footsteps_theta[i];

		//////-------------------------flag------------------------------/////
		//Footstep f(x, y, theta,
		//	ivEnvironmentParams.cell_size,
		//	ivEnvironmentParams.num_angle_bins,
		//	ivEnvironmentParams.hash_table_size);
		//ivEnvironmentParams.footstep_set.push_back(f);

		double cur_step_width = sqrt(x*x + y * y);

		if (cur_step_width > max_step_width)
			max_step_width = cur_step_width;
	}


	double step_range_x[] = { 0.22, 0.22,-0.10,-0.10 };
	double step_range_y[] = { 0.28, 0.12, 0.12, 0.28 };

	
	ivEnvironmentParams.step_range.clear();
	ivEnvironmentParams.step_range.reserve(length(step_range_x));
	double x, y;
	double max_x = 0.0;
	double max_y = 0.0;
	double cell_size = ivEnvironmentParams.cell_size;
	for (int i = 0; i < length(step_range_x); ++i)
	{
		x = (double)step_range_x[i];
		y = (double)step_range_y[i];
		if (fabs(x) > max_x)
			max_x = fabs(x);
		if (fabs(y) > max_y)
			max_y = fabs(y);
		ivEnvironmentParams.step_range.push_back(
			std::pair<int, int>(disc_val(x, cell_size), disc_val(y, cell_size)));
	}
	// insert first point again at the end!
	ivEnvironmentParams.step_range.push_back(ivEnvironmentParams.step_range[0]);
	ivEnvironmentParams.max_step_width = sqrt(max_x*max_x + max_y * max_y) * 1.5;
	return 1;
}

int main(int argc, char** argv)
{
	string yamlPath = "..\\footstep_sbpl_vs2017_cpp\\humanoid_navigation\\footstep_planner\\config\\";
	string fileName = "planning_params_all.yaml";
	environment_params ivEnvironmentParams;
	footstep_planner::FootstepPlanner ivFootstepPlanner;

	getFileParam(ivEnvironmentParams, ivFootstepPlanner, yamlPath, fileName);

	std::cout << "ivEnvironmentParams.num_angle_bins:" << ivEnvironmentParams.num_angle_bins << endl;
	std::cout << "ivEnvironmentParams.forward_search: " << ivEnvironmentParams.forward_search << std::endl;
	std::cout << "ivEnvironmentParams.cell_size: " << ivEnvironmentParams.cell_size << std::endl;
	//std::cout << "ivEnvironmentParams.step_range: " << ivEnvironmentParams.step_range[0] << std::endl;
	std::cout << "ivEnvironmentParams.max_step_width: " << ivEnvironmentParams.max_step_width << std::endl;

	
	string envPath = "..\\document\\";
	string mapName = "env_map.yaml";
	string startName = "env_start.yaml";
	string goalName = "env_goal.yaml";

	gridmap_2d::GridMap2D gridMap;
	gridMap.getMap(0, envPath, mapName);
	ROS_INFO("GridMap2D created with %d x %d cells at %f resolution.\n", gridMap.m_mapInfo.width, gridMap.m_mapInfo.height, gridMap.m_mapInfo.resolution);

	StartGoal::StartGoalInfo goalInfo;
	goalInfo.getGoal(envPath, goalName);
	ROS_INFO("Goal pose set to (%f %f %f)\n", goalInfo.position.x, goalInfo.position.y, goalInfo.theta);

	StartGoal::StartGoalInfo startInfo;
	startInfo.getStart(envPath, startName);
	ROS_INFO("Start pose set to (%f %f %f)\n", startInfo.position.x, startInfo.position.y, startInfo.theta);


	return 0;
}