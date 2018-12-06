#include <opencv2/opencv.hpp>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "tf/transform_datatypes.h"
//#include <tf/LinearMath/Quaternion.h>
//#include <tf/LinearMath/Matrix3x3.h>

//#define ROS_ERROR printf
//#define ROS_INFO printf
//#define ROS_ERROR_STREAM printf
//#define ROS_INFO_STREAM printf
//#define ROS_WARN printf
//#define ROS_DEBUG printf

using namespace cv;
using namespace std;
//using namespace footstep_planner;

template<class T>
int length(T& arr)
{
	return sizeof(arr) / sizeof(arr[0]);
}
inline int disc_val(double length, double cell_size)
{
	return int(floor((length / cell_size) + 0.5));
}
namespace gridmap_2d {
	/**
	 * @brief Stores a nav_msgs::OccupancyGrid in a convenient opencv cv::Mat
	 * as binary map (free: 255, occupied: 0) and as distance map (distance
	 * to closest obstacle in meter).
	 */
	struct MapInfo {
		//header
		int seq;
		std::string frame_id;
		long stamp_secs;
		long stamp_nsecs;

		//info
		long map_load_time_secs;
		long map_load_time_nsecs;
		float resolution;
		int width;
		int height;

		float origin_position_x;
		float origin_position_y;
		float origin_position_z;

		float origin_orientation_x;
		float origin_orientation_y;
		float origin_orientation_z;
		float origin_orientation_w;

		//vector data;
	};
	class GridMap2D {
		const static uchar FREE = 255;  ///< char value for "free": 255
		const static uchar OCCUPIED = 0; ///< char value for "free": 0
	public:
		cv::Mat m_binaryMap;	///< binary occupancy map. 255: free, 0 occupied.
		cv::Mat m_distMap;		///< distance map (in meter)
		//nav_msgs::MapMetaData m_mapInfo;
		MapInfo m_mapInfo;
		std::string m_frameId;	///< "map" frame where ROS OccupancyGrid originated from

		int getMap(bool unknown_as_obstacle, std::string yamlPath, std::string fileName);

	};

	int GridMap2D::getMap(bool unknown_as_obstacle,std::string yamlPath, std::string fileName)
	{
		cv::FileStorage fs_param;
		fs_param.open(yamlPath + fileName, cv::FileStorage::READ);
		if (!fs_param.isOpened())
		{
			std::cout << fileName << ": No file!" << std::endl;
			return -1;
		}


		m_mapInfo.height = fs_param["info"]["height"];
		m_mapInfo.width = fs_param["info"]["width"];
		m_mapInfo.resolution = fs_param["info"]["resolution"];

		m_frameId = fs_param["header"]["frame_id"];

		m_binaryMap = cv::Mat(m_mapInfo.width, m_mapInfo.height, CV_8UC1);
		m_distMap = cv::Mat(m_binaryMap.size(), CV_32FC1);


		FileNode fs_node = fs_param["data"];
		FileNodeIterator mapDataIter = fs_node.begin();
		//TODO check / param
		unsigned char map_occ_thres = 70;

		// iterate over map, store in image
		// (0,0) is lower left corner of OccupancyGrid
		for (unsigned int j = 0; j < m_mapInfo.height; ++j) {
			for (unsigned int i = 0; i < m_mapInfo.width; ++i) {
				if ((signed int)*mapDataIter > map_occ_thres
					|| (unknown_as_obstacle && (signed int)*mapDataIter < 0))
				{
					m_binaryMap.at<uchar>(i, j) = OCCUPIED;
					//cout<< m_binaryMap.at<uchar>(i, j);
				}
				else {
					m_binaryMap.at<uchar>(i, j) = FREE;
				}
				++mapDataIter;
			}
			//cout << "::::"<<(signed int)*mapDataIter <<"::::"<< int(m_binaryMap.at<uchar>(0, j))<< endl;
		}

		//////-------------------------flag------------------------------/////
		//updateDistanceMap();
		ROS_INFO("GridMap2D created with %d x %d cells at %f resolution.", m_mapInfo.width, m_mapInfo.height, m_mapInfo.resolution);

		fs_param.release();
	}

	typedef boost::shared_ptr< GridMap2D> GridMap2DPtr;
	//typedef boost::shared_ptr<const GridMap2D> GridMap2DConstPtr;

}

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

namespace tf
{
	struct orientation {
		double x;
		double y;
		double z;
		double w;
	};
	struct goal_pose {
		double x;
		double y;
		double z;
		struct orientation;
	};
	//static inline double getYaw(const struct orientation &msg_q) {
	//	Quaternion bt_q;
	//	quaternionMsgToTF(msg_q, bt_q);
	//	return getYaw(bt_q);
	//}
	static const double QUATERNION_TOLERANCE = 0.1f;
	//static inline void quaternionMsgToTF(const struct orientation &msg, Quaternion& bt)
	//{
	//	bt = Quaternion(msg.x, msg.y, msg.z, msg.w);
	//	if (fabs(bt.length2() - 1) > QUATERNION_TOLERANCE)
	//	{
	//		ROS_WARN("MSG to TF: Quaternion Not Properly Normalized");
	//		bt.normalize();
	//	}
	//};
	//static inline double getYaw(const Quaternion& bt_q) {
	//	tfScalar useless_pitch, useless_roll, yaw;
	//	tf::Matrix3x3(bt_q).getRPY(useless_roll, useless_pitch, yaw);
	//	return yaw;
	//}
}
namespace footstep_planner
{
	//typedef std::vector<State>::const_iterator state_iter_t;

	/**
	 * @brief A class to control the interaction between ROS and the footstep
	 * planner.
	 */
	class FootstepPlanner
	{
	
	public:
		double ivFootSeparation;
		double ivMaxStepWidth;
		int    ivCollisionCheckAccuracy;

		bool   ivStartPoseSetUp, ivGoalPoseSetUp;
		int    ivLastMarkerMsgSize;
		double ivPathCost;
		bool   ivSearchUntilFirstSolution;
		double ivMaxSearchTime;
		double ivInitialEpsilon;

		/**
		 * @brief If limit of changed cells is reached the planner starts a new
		 * task from the scratch.
		 */
		int ivChangedCellsLimit;

		std::string ivPlannerType;
		std::string ivMarkerNamespace;

		std::vector<int> ivPlanningStatesIds;
	};

	class Footstep
	{
	public:

		/// The (discretized) rotation of the footstep.
		int ivTheta;

		/// The parameter for the discretization of the translation.
		double ivCellSize;
		/// The parameter for the discretization of the rotation.
		int ivNumAngleBins;

		/// The maximal hash size.
		int ivMaxHashSize;

	};
}
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
		ROS_ERROR("Footstep parameterization has different sizes for x/y/theta. "
			"Exit!");
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



	std::cout << "gridMap.m_mapInfo.height: " << gridMap.m_mapInfo.height << std::endl;

	return 0;
}