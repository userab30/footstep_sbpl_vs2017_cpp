#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

struct environment_params
{
	//std::vector<Footstep> footstep_set;
	//boost::shared_ptr<Heuristic> heuristic;

	/// Defines the area of performable (discrete) steps.
	std::vector<std::pair<int, int> > step_range;

	double diff_angle_cost;

	double footsize_x, footsize_y, footsize_z;
	double foot_origin_shift_x, foot_origin_shift_y;
	double max_footstep_x, max_footstep_y, max_footstep_theta;
	double max_inverse_footstep_x, max_inverse_footstep_y, max_inverse_footstep_theta;
	double step_cost;
	int    collision_check_accuracy;
	int    hash_table_size;
	double cell_size;
	int    num_angle_bins;
	bool   forward_search;
	double max_step_width;
	int    num_random_nodes;
	double random_node_distance;
	double heuristic_scale;
};

int getFileParam(environment_params &ivEnvironmentParams, std::string yamlPath, std::string fileName)
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

	//ivPlannerType = fs_param["planner_type"];
	//ivSearchUntilFirstSolution = fs_param["search_until_first_solution"];
	//ivMaxSearchTime = fs_param["allocated_time"];
	ivEnvironmentParams.forward_search = fs_param["forward_search"] == "True";
	//ivInitialEpsilon = fs_param["initial_epsilon"];
	//ivChangedCellsLimit = fs_param["changed_cells_limit"];
	ivEnvironmentParams.num_random_nodes = 20;// fs_param["num_random_nodes"];
	ivEnvironmentParams.random_node_distance = 1.0;// fs_param["random_node_dist"];

	// footstep settings
	ivEnvironmentParams.footsize_x = fs_param["foot"]["size"]["x"];
	ivEnvironmentParams.footsize_y = fs_param["foot"]["size"]["y"];
	ivEnvironmentParams.footsize_z = fs_param["foot"]["size"]["z"];
	//ivFootSeparation = fs_param["foot"]["separation"];
	ivEnvironmentParams.foot_origin_shift_x = fs_param["foot"]["origin_shift"]["x"];
	ivEnvironmentParams.foot_origin_shift_y = fs_param["foot"]["origin_shift"]["y"];
	ivEnvironmentParams.max_footstep_x = fs_param["foot"]["max"]["step"]["x"];
	ivEnvironmentParams.max_footstep_y = fs_param["foot"]["max"]["step"]["y"];
	ivEnvironmentParams.max_footstep_theta = fs_param["foot"]["max"]["step"]["theta"];
	ivEnvironmentParams.max_inverse_footstep_x = fs_param["foot"]["max"]["inverse"]["step"]["x"];
	ivEnvironmentParams.max_inverse_footstep_y = fs_param["foot"]["max"]["inverse"]["step"]["y"];
	ivEnvironmentParams.max_inverse_footstep_theta = fs_param["foot"]["max"]["inverse"]["step"]["theta"];
	fs_param.release();
	return 1;
}

int main(int argc, char** argv)
{
	string yamlPath = "..\\footstep_sbpl_vs2017_cpp\\humanoid_navigation\\footstep_planner\\config\\";
	string fileName = "planning_params_all.yaml";
	environment_params ivEnvironmentParams;

	getFileParam(ivEnvironmentParams, yamlPath, fileName);

	std::cout << "ivEnvironmentParams.num_angle_bins:" << ivEnvironmentParams.num_angle_bins << endl;
	std::cout << "ivEnvironmentParams.forward_search: " << ivEnvironmentParams.forward_search << std::endl;
	std::cout << "ivEnvironmentParams.cell_size: " << ivEnvironmentParams.cell_size << std::endl;

	string envPath = "..\\document\\";
	string mapName = "env_gif.yaml";
	string startName = "env_start.yaml";
	string goalName = "env_goal.yaml";
	return 0;
}