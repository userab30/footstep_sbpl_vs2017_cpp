#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "tf/transform_datatypes.h"

#define PRINT_ERROR printf
#define PRINT_INFO printf
#define PRINT_ERROR_STREAM printf
#define PRINT_INFO_STREAM printf
#define PRINT_WARN printf
#define PRINT_DEBUG printf

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

template<class T>
int length(T& arr)
{
	return sizeof(arr) / sizeof(arr[0]);
}
inline int disc_val(double length, double cell_size)
{
	return int(floor((length / cell_size) + 0.5));
}

//namespace gridmap_2d {
//	/**
//	 * @brief Stores a nav_msgs::OccupancyGrid in a convenient opencv cv::Mat
//	 * as binary map (free: 255, occupied: 0) and as distance map (distance
//	 * to closest obstacle in meter).
//	 */
//	struct MapInfo {
//		//header
//		int seq;
//		std::string frame_id;
//		long stamp_secs;
//		long stamp_nsecs;
//
//		//info
//		long map_load_time_secs;
//		long map_load_time_nsecs;
//		float resolution;
//		int width;
//		int height;
//
//		float origin_position_x;
//		float origin_position_y;
//		float origin_position_z;
//
//		float origin_orientation_x;
//		float origin_orientation_y;
//		float origin_orientation_z;
//		float origin_orientation_w;
//
//		//vector data;
//	};
//	class GridMap2D {
//		const static uchar FREE = 255;  ///< char value for "free": 255
//		const static uchar OCCUPIED = 0; ///< char value for "free": 0
//	public:
//		cv::Mat m_binaryMap;	///< binary occupancy map. 255: free, 0 occupied.
//		cv::Mat m_distMap;		///< distance map (in meter)
//		//nav_msgs::MapMetaData m_mapInfo;
//		MapInfo m_mapInfo;
//		std::string m_frameId;	///< "map" frame where ROS OccupancyGrid originated from
//
//		int getMap(bool unknown_as_obstacle, std::string yamlPath, std::string fileName);
//
//	};
//
//	int GridMap2D::getMap(bool unknown_as_obstacle, std::string yamlPath, std::string fileName)
//	{
//		cv::FileStorage fs_param;
//		fs_param.open(yamlPath + fileName, cv::FileStorage::READ);
//		if (!fs_param.isOpened())
//		{
//			std::cout << fileName << ": No file!" << std::endl;
//			return -1;
//		}
//
//
//		m_mapInfo.height = fs_param["info"]["height"];
//		m_mapInfo.width = fs_param["info"]["width"];
//		m_mapInfo.resolution = fs_param["info"]["resolution"];
//
//		m_frameId = fs_param["header"]["frame_id"];
//
//		m_binaryMap = cv::Mat(m_mapInfo.width, m_mapInfo.height, CV_8UC1);
//		m_distMap = cv::Mat(m_binaryMap.size(), CV_32FC1);
//
//
//		FileNode fs_node = fs_param["data"];
//		FileNodeIterator mapDataIter = fs_node.begin();
//		//TODO check / param
//		unsigned char map_occ_thres = 70;
//
//		// iterate over map, store in image
//		// (0,0) is lower left corner of OccupancyGrid
//		for (unsigned int j = 0; j < m_mapInfo.height; ++j) {
//			for (unsigned int i = 0; i < m_mapInfo.width; ++i) {
//				if ((signed int)*mapDataIter > map_occ_thres
//					|| (unknown_as_obstacle && (signed int)*mapDataIter < 0))
//				{
//					m_binaryMap.at<uchar>(i, j) = OCCUPIED;
//					//cout<< m_binaryMap.at<uchar>(i, j);
//				}
//				else {
//					m_binaryMap.at<uchar>(i, j) = FREE;
//				}
//				++mapDataIter;
//			}
//			//cout << "::::"<<(signed int)*mapDataIter <<"::::"<< int(m_binaryMap.at<uchar>(0, j))<< endl;
//		}
//
//		//////-------------------------flag------------------------------/////
//		updateDistanceMap();
//		ROS_INFO("GridMap2D created with %d x %d cells at %f resolution.", m_mapInfo.width, m_mapInfo.height, m_mapInfo.resolution);
//
//		fs_param.release();
//	}
//	typedef boost::shared_ptr<GridMap2D> GridMap2DPtr;
//	//typedef boost::shared_ptr<const GridMap2D> GridMap2DConstPtr;
//
//}


namespace StartGoal{
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
		posInfo position ;
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
		
		theta=tf::getYaw(orientation.x, orientation.y, orientation.z, orientation.w);
		//static inline double getYaw(const tfScalar& x, const tfScalar& y, const tfScalar& z, const tfScalar& w)
		fs_param.release();
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
	};

}
//namespace footstep_planner
//{
	//typedef std::vector<State>::const_iterator state_iter_t;

	/**
	 * @brief A class to control the interaction between ROS and the footstep
	 * planner.
	 */
	//class FootstepPlanner
	//{

	//public:
	//	double ivFootSeparation;
	//	double ivMaxStepWidth;
	//	int    ivCollisionCheckAccuracy;

	//	bool   ivStartPoseSetUp, ivGoalPoseSetUp;
	//	int    ivLastMarkerMsgSize;
	//	double ivPathCost;
	//	bool   ivSearchUntilFirstSolution;
	//	double ivMaxSearchTime;
	//	double ivInitialEpsilon;

	//	/**
	//	 * @brief If limit of changed cells is reached the planner starts a new
	//	 * task from the scratch.
	//	 */
	//	int ivChangedCellsLimit;

	//	std::string ivPlannerType;
	//	std::string ivMarkerNamespace;

	//	std::vector<int> ivPlanningStatesIds;
	//};

	/*class Footstep
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

	};*/
//}



