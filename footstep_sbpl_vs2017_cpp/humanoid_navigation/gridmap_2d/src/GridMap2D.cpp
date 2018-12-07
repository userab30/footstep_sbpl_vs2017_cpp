/*
 * A simple 2D gridmap structure
 *
 * Copyright 2011 Armin Hornung, University of Freiburg
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "gridmap_2d/GridMap2D.h"
 //#include <ros/console.h>

namespace gridmap_2d {

	GridMap2D::GridMap2D()
		: m_frameId("/map")
	{

	}

	//GridMap2D::GridMap2D(const nav_msgs::OccupancyGridConstPtr& gridMap, bool unknown_as_obstacle) {
	//
	//  setMap(gridMap, unknown_as_obstacle);
	//
	//}

	GridMap2D::GridMap2D(const GridMap2D& other)
		: m_binaryMap(other.m_binaryMap.clone()),
		m_distMap(other.m_distMap.clone()),
		m_mapInfo(other.m_mapInfo),
		m_frameId(other.m_frameId)
	{

	}

	GridMap2D::~GridMap2D() {

	}

	void GridMap2D::updateDistanceMap() {
		cv::distanceTransform(m_binaryMap, m_distMap, CV_DIST_L2, CV_DIST_MASK_PRECISE);
		// distance map now contains distance in meters:
		m_distMap = m_distMap * m_mapInfo.resolution;
	}

	int GridMap2D::getMap(bool unknown_as_obstacle, std::string yamlPath, std::string fileName)
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

		m_mapInfo.origin.position.x = fs_param["info"]["origin"]["position"]["x"];
		m_mapInfo.origin.position.y = fs_param["info"]["origin"]["position"]["y"];
		m_mapInfo.origin.position.z = fs_param["info"]["origin"]["position"]["z"];

		m_mapInfo.origin.orientation.x = fs_param["info"]["origin"]["orientation"]["x"];
		m_mapInfo.origin.orientation.y = fs_param["info"]["origin"]["orientation"]["y"];
		m_mapInfo.origin.orientation.z = fs_param["info"]["origin"]["orientation"]["z"];
		m_mapInfo.origin.orientation.w = fs_param["info"]["origin"]["orientation"]["w"];


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
		fs_param.release();

		//////-------------------------flag------------------------------/////
		updateDistanceMap();
		PRINT_INFO("GridMap2D created with %d x %d cells at %f resolution.", m_mapInfo.width, m_mapInfo.height, m_mapInfo.resolution);

		typedef boost::shared_ptr<GridMap2D> GridMap2DPtr;
		typedef boost::shared_ptr<const GridMap2D> GridMap2DConstPtr;
		
	}

	////----------------------------flag--------------------------------------//

	//nav_msgs::OccupancyGrid GridMap2D::toOccupancyGridMsg() const {
	//	nav_msgs::OccupancyGrid msg;
	//	msg.header.frame_id = m_frameId;
	//	msg.header.stamp = ros::Time::now();
	//	msg.info = m_mapInfo;
	//	msg.data.resize(msg.info.height*msg.info.width);

	//	// iterate over map, store in data
	//	std::vector<signed char>::iterator mapDataIter = msg.data.begin();
	//	// (0,0) is lower left corner of OccupancyGrid
	//	for (unsigned int j = 0; j < m_mapInfo.height; ++j) {
	//		for (unsigned int i = 0; i < m_mapInfo.width; ++i) {
	//			if (m_binaryMap.at<uchar>(i, j) == OCCUPIED)
	//				*mapDataIter = 100;
	//			else
	//				*mapDataIter = 0;

	//			++mapDataIter;
	//		}
	//	}

	//	return msg;
	//}

	void GridMap2D::setMap(const cv::Mat& binaryMap) {
		m_binaryMap = binaryMap.clone();
		m_distMap = cv::Mat(m_binaryMap.size(), CV_32FC1);

		cv::distanceTransform(m_binaryMap, m_distMap, CV_DIST_L2, CV_DIST_MASK_PRECISE);
		// distance map now contains distance in meters:
		m_distMap = m_distMap * m_mapInfo.resolution;

		PRINT_INFO("GridMap2D copied from existing cv::Mat with %d x %d cells at %f resolution.", m_mapInfo.width, m_mapInfo.height, m_mapInfo.resolution);

	}

	void GridMap2D::inflateMap(double inflationRadius) {
		m_binaryMap = (m_distMap > inflationRadius);
		// recompute distance map with new binary map:
		cv::distanceTransform(m_binaryMap, m_distMap, CV_DIST_L2, CV_DIST_MASK_PRECISE);
		m_distMap = m_distMap * m_mapInfo.resolution;
	}

	// See costmap2D for mapToWorld / worldToMap implementations:

	void GridMap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const {
		wx = m_mapInfo.origin.position.x + (mx + 0.5) * m_mapInfo.resolution;
		wy = m_mapInfo.origin.position.y + (my + 0.5) * m_mapInfo.resolution;
	}



	void GridMap2D::worldToMapNoBounds(double wx, double wy, unsigned int& mx, unsigned int& my) const {
		mx = (int)((wx - m_mapInfo.origin.position.x) / m_mapInfo.resolution);
		my = (int)((wy - m_mapInfo.origin.position.y) / m_mapInfo.resolution);
	}

	bool GridMap2D::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const {
		if (wx < m_mapInfo.origin.position.x || wy < m_mapInfo.origin.position.y)
			return false;

		mx = (int)((wx - m_mapInfo.origin.position.x) / m_mapInfo.resolution);
		my = (int)((wy - m_mapInfo.origin.position.y) / m_mapInfo.resolution);

		if (mx < m_mapInfo.width && my < m_mapInfo.height)
			return true;

		return false;
	}

	bool GridMap2D::inMapBounds(double wx, double wy) const {
		unsigned mx, my;
		return worldToMap(wx, wy, mx, my);
	}

	float GridMap2D::distanceMapAt(double wx, double wy) const {
		unsigned mx, my;

		if (worldToMap(wx, wy, mx, my))
			return m_distMap.at<float>(mx, my);
		else
			return -1.0f;
	}


	uchar GridMap2D::binaryMapAt(double wx, double wy) const {
		unsigned mx, my;

		if (worldToMap(wx, wy, mx, my))
			return m_binaryMap.at<uchar>(mx, my);
		else
			return 0;
	}

	float GridMap2D::distanceMapAtCell(unsigned int mx, unsigned int my) const {
		return m_distMap.at<float>(mx, my);
	}


	uchar GridMap2D::binaryMapAtCell(unsigned int mx, unsigned int my) const {
		return m_binaryMap.at<uchar>(mx, my);
	}

	uchar& GridMap2D::binaryMapAtCell(unsigned int mx, unsigned int my) {
		return m_binaryMap.at<uchar>(mx, my);
	}


	bool GridMap2D::isOccupiedAtCell(unsigned int mx, unsigned int my) const {
		return (m_binaryMap.at<uchar>(mx, my) < 255);
	}


	bool GridMap2D::isOccupiedAt(double wx, double wy) const {
		unsigned mx, my;
		if (worldToMap(wx, wy, mx, my))
			return isOccupiedAtCell(mx, my);
		else
			return true;
	}

}


