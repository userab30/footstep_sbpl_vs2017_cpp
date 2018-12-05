#include <opencv2/opencv.hpp>
#include <iostream>
#include<opencv2/highgui.hpp>
#include<opencv2/core/core.hpp>

using namespace cv;
using namespace std;

struct environment_params
{
	//std::vector<Footstep> footstep_set;
	//boost::shared_ptr<Heuristic> heuristic;

	std::string heuristic_type;
	std::string planner_type;
	bool search_until_first_solution;
	
	/// Defines the area of performable (discrete) steps.
	std::vector<std::pair<int, int> > step_range;

	double diff_angle_cost;

	double footsize_x, footsize_y, footsize_z;
	double foot_origin_shift_x, foot_origin_shift_y;
	double max_footstep_x, max_footstep_y, max_footstep_theta;
	double max_inverse_footstep_x, max_inverse_footstep_y,max_inverse_footstep_theta;
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

int main(int argc, char** argv)
{
	string yamlPath = "..\\footstep_sbpl_vs2017_cpp\\humanoid_navigation\\footstep_planner\\config\\";
	string fileName = "planning_params.yaml";

	FileStorage fs;
	fs.open(yamlPath + fileName, FileStorage::READ);
	if (!fs.isOpened())
	{
		cout << "No file!" << endl;
		return -1;
	}
	environment_params envParams;
	envParams.heuristic_type = fs["heuristic_type"];
	envParams.planner_type = fs["planner_type"];
	envParams.search_until_first_solution = (string)fs["search_until_first_solution"] == "True";
	cout<<envParams.planner_type;

	fs.release();
	return 0;
}
