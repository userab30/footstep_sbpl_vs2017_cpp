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

namespace StartGoal {
	struct oriInfo {
		double x;
		double y;
		double z;
		double w;
	};
	struct posInfo {
		double x;
		double y;
		double z;
	};

	class StartGoalInfo {
	public:
		int seq;
		std::string frame_id;
		long stamp_secs;
		long stamp_nsecs;
		posInfo position;
		oriInfo orientation;
		float theta;


		int getGoal(std::string yamlPath, std::string fileName);
		int getStart(std::string yamlPath, std::string fileName);
		//bool setGoal(float x, float y, float theta);

	};

	int StartGoalInfo::getGoal(std::string yamlPath, std::string fileName)
	{
		cv::FileStorage fs_param;
		fs_param.open(yamlPath + fileName, cv::FileStorage::READ);
		if (!fs_param.isOpened())
		{
			std::cout << fileName << ": No file!" << std::endl;
			return -1;
		}

		position.x = fs_param["pose"]["position"]["x"];
		position.y = fs_param["pose"]["position"]["y"];
		position.z = fs_param["pose"]["position"]["z"];

		orientation.x = fs_param["pose"]["orientation"]["x"];
		orientation.y = fs_param["pose"]["orientation"]["y"];
		orientation.z = fs_param["pose"]["orientation"]["z"];
		orientation.w = fs_param["pose"]["orientation"]["w"];
		//cout << orientation.w << endl << endl;

		frame_id = fs_param["header"]["frame_id"];

		theta = tf::getYaw(orientation.x, orientation.y, orientation.z, orientation.w);
		//static inline double getYaw(const tfScalar& x, const tfScalar& y, const tfScalar& z, const tfScalar& w)
		fs_param.release();
		return 1;
	};

	int StartGoalInfo::getStart(std::string yamlPath, std::string fileName)
	{
		cv::FileStorage fs_param;
		fs_param.open(yamlPath + fileName, cv::FileStorage::READ);
		if (!fs_param.isOpened())
		{
			std::cout << fileName << ": No file!" << std::endl;
			return -1;
		}

		position.x = fs_param["pose"]["pose"]["position"]["x"];
		position.y = fs_param["pose"]["pose"]["position"]["y"];
		position.z = fs_param["pose"]["pose"]["position"]["z"];

		orientation.x = fs_param["pose"]["pose"]["orientation"]["x"];
		orientation.y = fs_param["pose"]["pose"]["orientation"]["y"];
		orientation.z = fs_param["pose"]["pose"]["orientation"]["z"];
		orientation.w = fs_param["pose"]["pose"]["orientation"]["w"];
		//cout << orientation.w << endl << endl;

		frame_id = fs_param["header"]["frame_id"];

		theta = tf::getYaw(orientation.x, orientation.y, orientation.z, orientation.w);
		//static inline double getYaw(const tfScalar& x, const tfScalar& y, const tfScalar& z, const tfScalar& w)
		fs_param.release();
		return 1;
	};

}

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

	StartGoal::StartGoalInfo startInfo;
	startInfo.getStart(envPath, startName);
	PRINT_INFO("Start pose set to (%f %f %f)\n", startInfo.position.x, startInfo.position.y, startInfo.theta);

	StartGoal::StartGoalInfo goalInfo;
	goalInfo.getGoal(envPath, goalName);
	PRINT_INFO("Goal pose set to (%f %f %f)\n", goalInfo.position.x, goalInfo.position.y, goalInfo.theta);


	return 0;
}