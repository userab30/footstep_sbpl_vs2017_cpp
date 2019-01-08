/*
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/footstep_planner
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <footstep_planner/FootstepPlanner.h>
 //#include <humanoid_nav_msgs/ClipFootstep.h>
//#include "advise_input/footstep_datatype.h"

using namespace std;
using gridmap_2d::GridMap2D;
using gridmap_2d::GridMap2DPtr;

template<class T>
int length(T& arr)
{
	return sizeof(arr) / sizeof(arr[0]);
}
namespace footstep_planner
{
	FootstepPlanner::FootstepPlanner(std::string yamlPath, std::string fileName)
		: ivStartPoseSetUp(false),
		ivGoalPoseSetUp(false),
		ivLastMarkerMsgSize(0),
		ivPathCost(0),
		ivMarkerNamespace("")
	{
		std::string heuristic_type;
		double diff_angle_cost;

		cv::FileStorage fs_param;
		fs_param.open(yamlPath + fileName, cv::FileStorage::READ);
		if (!fs_param.isOpened())
		{
			std::cout << fileName << ": No file!" << std::endl;
			//return -1;
		}


		heuristic_type = fs_param["heuristic_type"];
		ivEnvironmentParams.heuristic_scale = 1;// fs_param["heuristic_scale"];
		ivEnvironmentParams.hash_table_size = fs_param["max_hash_size"];
		ivEnvironmentParams.collision_check_accuracy = fs_param["accuracy"]["collision_check"];
		ivEnvironmentParams.cell_size = fs_param["accuracy"]["cell_size"];
		ivEnvironmentParams.num_angle_bins = fs_param["accuracy"]["num_angle_bins"];

		ivEnvironmentParams.step_cost = fs_param["step_cost"];
		diff_angle_cost = fs_param["diff_angle_cost"];

		ivPlannerType = fs_param["planner_type"];
		ivSearchUntilFirstSolution = fs_param["search_until_first_solution"] == "True";
		ivMaxSearchTime = fs_param["allocated_time"];
		ivEnvironmentParams.forward_search = fs_param["forward_search"] == "True";
		ivInitialEpsilon = fs_param["initial_epsilon"];
		ivChangedCellsLimit = fs_param["changed_cells_limit"];
		ivEnvironmentParams.num_random_nodes = 20;// fs_param["num_random_nodes"];
		ivEnvironmentParams.random_node_distance = 1.0;// fs_param["random_node_dist"];

		// footstep settings
		ivEnvironmentParams.footsize_x = fs_param["foot"]["size"]["x"];
		ivEnvironmentParams.footsize_y = fs_param["foot"]["size"]["y"];
		ivEnvironmentParams.footsize_z = fs_param["foot"]["size"]["z"];
		ivFootSeparation = fs_param["foot"]["separation"];
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

		int size_x = length(footsteps_x);
		int size_y = length(footsteps_y);
		int size_t = length(footsteps_theta);
		if (size_x != size_y || size_x != size_t)
		{
			PRINT_ERROR("Footstep parameterization has different sizes for x/y/theta. Exit!");
			exit(2);
		}
		//////-------------------------flag------------------------------/////
		// create footstep set
		ivEnvironmentParams.footstep_set.clear();
		double max_step_width = 0;
		for (int i = 0; i < length(footsteps_x); ++i)
		{
			double x = (double)footsteps_x[i];
			double y = (double)footsteps_y[i];
			double theta = (double)footsteps_theta[i];

			//-------------------------flag------------------------------/////
			Footstep f(x, y, theta,
				ivEnvironmentParams.cell_size,
				ivEnvironmentParams.num_angle_bins,
				ivEnvironmentParams.hash_table_size);
			ivEnvironmentParams.footstep_set.push_back(f);

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

		thetaAdd = atan2(ivEnvironmentParams.footsize_y, ivEnvironmentParams.footsize_x);
		halfDiagonal = sqrt(pow(ivEnvironmentParams.footsize_y / 2, 2) + pow(ivEnvironmentParams.footsize_x / 2, 2));

		//std::cout << "ivEnvironmentParams.num_angle_bins:" << ivEnvironmentParams.num_angle_bins << std::endl;
		//std::cout << "ivEnvironmentParams.forward_search: " << ivEnvironmentParams.forward_search << std::endl;
		//std::cout << "ivEnvironmentParams.cell_size: " << ivEnvironmentParams.cell_size << std::endl;
		////std::cout << "ivEnvironmentParams.step_range: " << ivEnvironmentParams.step_range[0] << std::endl;
		//std::cout << "ivEnvironmentParams.max_step_width: " << ivEnvironmentParams.max_step_width << std::endl;

		// initialize the heuristic
		boost::shared_ptr<Heuristic> h;
		//if (heuristic_type == "EuclideanHeuristic")
		//{
		//  h.reset(
		//      new EuclideanHeuristic(ivEnvironmentParams.cell_size,
		//                             ivEnvironmentParams.num_angle_bins));
		//  PRINT_INFO("FootstepPlanner heuristic: euclidean distance");
		//}
		//else if(heuristic_type == "EuclStepCostHeuristic")
		//{
		//  h.reset(
		//      new EuclStepCostHeuristic(ivEnvironmentParams.cell_size,
		//                                ivEnvironmentParams.num_angle_bins,
		//                                ivEnvironmentParams.step_cost,
		//                                diff_angle_cost,
		//                                max_step_width));
		//  PRINT_INFO("FootstepPlanner heuristic: euclidean distance with step costs");
		//}
		//else 
		if (heuristic_type == "PathCostHeuristic")
		{
			// for heuristic inflation
			double foot_incircle =
				std::min((ivEnvironmentParams.footsize_x / 2.0 -
					std::abs(ivEnvironmentParams.foot_origin_shift_x)),
					(ivEnvironmentParams.footsize_y / 2.0 -
						std::abs(ivEnvironmentParams.foot_origin_shift_y)));
			assert(foot_incircle > 0.0);

			h.reset(
				new PathCostHeuristic(ivEnvironmentParams.cell_size,
					ivEnvironmentParams.num_angle_bins,
					ivEnvironmentParams.step_cost,
					diff_angle_cost,
					max_step_width,
					foot_incircle));
			PRINT_INFO("FootstepPlanner heuristic: 2D path euclidean distance with step "
				"costs \n");

			// keep a local ptr for visualization
			ivPathCostHeuristicPtr = boost::dynamic_pointer_cast<PathCostHeuristic>(h);
		}
		else
		{
			//PRINT_ERROR_STREAM("Heuristic " << heuristic_type << " not available, "
			//                 "exiting.");
			exit(1);
		}
		ivEnvironmentParams.heuristic = h;

		// initialize the planner environment
		ivPlannerEnvironmentPtr.reset(
			new FootstepPlannerEnvironment(ivEnvironmentParams));

		// set up planner
		if (ivPlannerType == "ARAPlanner" ||
			ivPlannerType == "ADPlanner" ||
			ivPlannerType == "RSTARPlanner")
		{
			PRINT_INFO_STREAM("Planning with %s\n" ,ivPlannerType.c_str());
		}
		else
		{
			PRINT_ERROR_STREAM("Planner %s not available \n ", ivPlannerType.c_str());
							 //"untested.");
			exit(1);
		}
		if (ivEnvironmentParams.forward_search)
		{
			PRINT_INFO_STREAM("Search direction: forward planning\n");
		}
		else
		{
			PRINT_INFO_STREAM("Search direction: backward planning\n");
		}
		setPlanner();
	}


	FootstepPlanner::~FootstepPlanner()
	{}


	void
		FootstepPlanner::setPlanner()
	{
		if (ivPlannerType == "ARAPlanner")
		{
			ivPlannerPtr.reset(
				new ARAPlanner(ivPlannerEnvironmentPtr.get(),
					ivEnvironmentParams.forward_search));
		}
		//else if (ivPlannerType == "ADPlanner")
		//{
		//  ivPlannerPtr.reset(
		//      new ADPlanner(ivPlannerEnvironmentPtr.get(),
		//                    ivEnvironmentParams.forward_search));
		//}
		//else if (ivPlannerType == "RSTARPlanner")
		//{
		//  RSTARPlanner* p =
		//      new RSTARPlanner(ivPlannerEnvironmentPtr.get(),
		//                       ivEnvironmentParams.forward_search);
		//  // new options, require patched SBPL
		//  //          p->set_local_expand_thres(500);
		//  //          p->set_eps_step(1.0);
		//  ivPlannerPtr.reset(p);
		//}
		//        else if (ivPlannerType == "ANAPlanner")
		//        	ivPlannerPtr.reset(new anaPlanner(ivPlannerEnvironmentPtr.get(),
		//        	                                  ivForwardSearch));
	}


	bool
		FootstepPlanner::run()
	{
		bool path_existed = (bool)ivPath.size();
		int ret = 0;
		MDPConfig mdp_config;
		std::vector<int> solution_state_ids;

		// commit start/goal poses to the environment
		ivPlannerEnvironmentPtr->updateStart(ivStartFootLeft, ivStartFootRight);
		ivPlannerEnvironmentPtr->updateGoal(ivGoalFootLeft, ivGoalFootRight);
		ivPlannerEnvironmentPtr->updateHeuristicValues();
		ivPlannerEnvironmentPtr->InitializeEnv(NULL);
		ivPlannerEnvironmentPtr->InitializeMDPCfg(&mdp_config);

		// inform AD planner about changed (start) states for replanning
		if (path_existed &&
			!ivEnvironmentParams.forward_search &&
			ivPlannerType == "ADPlanner")
		{
			std::vector<int> changed_edges;
			changed_edges.push_back(mdp_config.startstateid);
			// update the AD planner
			boost::shared_ptr<ADPlanner> ad_planner =
				boost::dynamic_pointer_cast<ADPlanner>(ivPlannerPtr);
			ad_planner->update_preds_of_changededges(&changed_edges);
		}

		// set up SBPL
		if (ivPlannerPtr->set_start(mdp_config.startstateid) == 0)
		{
			PRINT_ERROR("Failed to set start state.\n");
			return false;
		}
		if (ivPlannerPtr->set_goal(mdp_config.goalstateid) == 0)
		{
			PRINT_ERROR("Failed to set goal state\n");
			return false;
		}

		ivPlannerPtr->set_initialsolution_eps(ivInitialEpsilon);
		ivPlannerPtr->set_search_mode(ivSearchUntilFirstSolution);

		PRINT_INFO("Start planning (max time: %f, initial eps: %f (%f))\n",
			ivMaxSearchTime, ivInitialEpsilon,
			ivPlannerPtr->get_initial_eps());
		int path_cost;
		//ros::WallTime startTime = ros::WallTime::now();
		clock_t startTime = clock();
		try
		{
			ret = ivPlannerPtr->replan(ivMaxSearchTime, &solution_state_ids,
				&path_cost);
		}
		catch (const SBPL_Exception& e)
		{
			// PRINT_ERROR("SBPL planning failed (%s)", e.what());
			return false;
		}
		ivPathCost = double(path_cost) / FootstepPlannerEnvironment::cvMmScale;

		bool path_is_new = pathIsNew(solution_state_ids);
		if (ret && solution_state_ids.size() > 0)
		{
			if (!path_is_new)
				PRINT_WARN("Solution found by SBPL is the same as the old solution. This could indicate that replanning failed.\n");

			PRINT_INFO("Solution of size %zu found after %f s\n",
				solution_state_ids.size(), //总共的步数
				(double)(clock() - startTime) / CLOCKS_PER_SEC); //.toSec()
			
			for (int i = 0; i < solution_state_ids.size(); i++)
			{
				if (i == 0)
				{
					PRINT_INFO("the parameter of the bezier: %f\n", BEZIERPARA);
					//PRINT_INFO("The half of the width of the passage: %f\n" ,PASSWIDTH);
					PRINT_INFO("The sequense of the id from start to goal: ");
				}
				PRINT_INFO("%d ", solution_state_ids.at(i));
				if (i == solution_state_ids.size() - 1)
				{
					PRINT_INFO("\n");
				}
			}
			
			if (extractPath(solution_state_ids))
			{
				PRINT_INFO("Expanded states: %i total / %i new\n",
					ivPlannerEnvironmentPtr->getNumExpandedStates(),
					ivPlannerPtr->get_n_expands());
				PRINT_INFO("Final eps: %f\n", ivPlannerPtr->get_final_epsilon());
				PRINT_INFO("Path cost: %f (%i)\n", ivPathCost, path_cost);

				ivPlanningStatesIds = solution_state_ids;
				if (getPathSize() == 0)
				{
				   PRINT_INFO("no path has been extracted yet");
				   return false;
				}
				//clearFootstepPathVis(0);
				/*float thetaAdd = atan2(ivEnvironmentParams.footsize_y, ivEnvironmentParams.footsize_x);
				float halfDiagonal = sqrt(pow(ivEnvironmentParams.footsize_y / 2, 2) + pow(ivEnvironmentParams.footsize_x / 2, 2));*/
				 // add the missing start foot to the publish vector for visualization:
				if (ivPath.front().getLeg() == LEFT)
					FootstepPath(ivStartFootRight, watchBezier_binaryMap);
				else
					FootstepPath(ivStartFootLeft, watchBezier_binaryMap);
				//画出每个脚印
				for(state_iter_t path_iter = getPathBegin(); path_iter != getPathEnd(); ++path_iter)
			    {
					FootstepPath(*path_iter, watchBezier_binaryMap);
			    }
				//footPoseToMarker();
				//broadcastExpandedNodesVis();
				//broadcastRandomNodesVis();
				//broadcastFootstepPathVis();
				//broadcastPathVis();

				return true;
			}
			else
			{
				PRINT_ERROR("extracting path failed\n\n");
				return false;
			}
		}
		else
		{
			//broadcastExpandedNodesVis();
			//broadcastRandomNodesVis();

			PRINT_ERROR("No solution found\n");
			return false;
		}
	}
	void
		FootstepPlanner::FootstepPath(const State& foot_pose,cv::Mat& binaryMap)
	{

		float cos_theta = cos(foot_pose.getTheta()+ thetaAdd);
		float sin_theta = sin(foot_pose.getTheta()+ thetaAdd);
		
		int leftUpper_x = state_2_cell(foot_pose.getX() + cos_theta * halfDiagonal, ivEnvironmentParams.cell_size);
		int leftUpper_y = state_2_cell(foot_pose.getY() + sin_theta * halfDiagonal, ivEnvironmentParams.cell_size);


		int rightLower_x = state_2_cell(foot_pose.getX() - cos_theta * halfDiagonal, ivEnvironmentParams.cell_size);
		int rightLower_y = state_2_cell(foot_pose.getY() - sin_theta * halfDiagonal, ivEnvironmentParams.cell_size);

		cos_theta = cos(foot_pose.getTheta() - thetaAdd);
		sin_theta = sin(foot_pose.getTheta() - thetaAdd);

		int rightUpper_x = state_2_cell(foot_pose.getX() + cos_theta * halfDiagonal, ivEnvironmentParams.cell_size);
		int rightUpper_y = state_2_cell(foot_pose.getY() + sin_theta * halfDiagonal, ivEnvironmentParams.cell_size);


		int leftLower_x = state_2_cell(foot_pose.getX() - cos_theta * halfDiagonal, ivEnvironmentParams.cell_size);
		int leftLower_y = state_2_cell(foot_pose.getY() - sin_theta * halfDiagonal, ivEnvironmentParams.cell_size);

		//cv::imshow("Origin_watchBezier_binaryMap", watchBezier_binaryMap);
		//cv::waitKey();
		//watchBezier_binaryMap.at<uchar>(leftUpper_x, leftUpper_y) = 0;
		//watchBezier_binaryMap.at<uchar>(rightLower_x, rightLower_y) = 0;
		cv::line(binaryMap, cv::Point(leftUpper_y, leftUpper_x), cv::Point(rightUpper_y, rightUpper_x), cv::Scalar(0, 0, 255), 1);
		cv::line(binaryMap, cv::Point(leftUpper_y, leftUpper_x), cv::Point(leftLower_y, leftLower_x), cv::Scalar(0, 0, 255), 1);
		cv::line(binaryMap, cv::Point(rightUpper_y, rightUpper_x), cv::Point(rightLower_y, rightLower_x), cv::Scalar(0, 0, 255), 1);
		cv::line(binaryMap, cv::Point(leftLower_y, leftLower_x), cv::Point(rightLower_y, rightLower_x), cv::Scalar(0, 0, 255), 1);

		//cv::rectangle(watchBezier_binaryMap, cv::Point(leftUpper_y, leftUpper_x), cv::Point(rightLower_y, rightLower_x), cv::Scalar(0, 0, 0), 1, 1, 0);
		/*cv::imshow("watchBezier_binaryMap", watchBezier_binaryMap);
		cv::waitKey();*/

	}
	
	bool
	FootstepPlanner::extractPath(const std::vector<int>& state_ids)
	{
	  ivPath.clear();
	
	  State s;
	  State start_left;
	  std::vector<int>::const_iterator state_ids_iter = state_ids.begin();
	
	  // first state is always the robot's left foot
	  if (!ivPlannerEnvironmentPtr->getState(*state_ids_iter, &start_left))
	  {
	    ivPath.clear();
	    return false;
	  }
	  ++state_ids_iter;
	  if (!ivPlannerEnvironmentPtr->getState(*state_ids_iter, &s))
	  {
	    ivPath.clear();
	    return false;
	  }
	  ++state_ids_iter;
	
	  // check if the robot's left foot can be ommited as first state in the path,
	  // i.e. the robot's right foot is appended first to the path
	  if (s.getLeg() == LEFT)
	    ivPath.push_back(ivStartFootRight);
	  else
	    ivPath.push_back(start_left);
	  ivPath.push_back(s);
	
	  for(; state_ids_iter < state_ids.end(); ++state_ids_iter)
	  {
	    if (!ivPlannerEnvironmentPtr->getState(*state_ids_iter, &s))
	    {
	      ivPath.clear();
	      return false;
	    }
	    ivPath.push_back(s);
	  }
	
	  // add last neutral step
	  if (ivPath.back().getLeg() == RIGHT)
	    ivPath.push_back(ivGoalFootLeft);
	  else // last_leg == LEFT
	    ivPath.push_back(ivGoalFootRight);
	
	  return true;
	}


	void
		FootstepPlanner::reset()
	{
		PRINT_INFO("Resetting planner\n");
		// reset the previously calculated paths
		ivPath.clear();
		ivPlanningStatesIds.clear();
		// reset the planner
		// INFO: force_planning_from_scratch was not working properly the last time
		// checked; therefore instead of using this function the planner is manually
		// reset
		//ivPlannerPtr->force_planning_from_scratch();
		ivPlannerEnvironmentPtr->reset();
		setPlanner();
	}


	void
		FootstepPlanner::resetTotally()
	{
		PRINT_INFO("Resetting planner and environment");
		// reset the previously calculated paths
		ivPath.clear();
		ivPlanningStatesIds.clear();
		// reinitialize the planner environment
		ivPlannerEnvironmentPtr.reset(
			new FootstepPlannerEnvironment(ivEnvironmentParams));
		setPlanner();
	}


	bool
		FootstepPlanner::plan(bool force_new_plan)
	{
		if (!ivMapPtr)
		{
			PRINT_ERROR("FootstepPlanner has no map for planning yet.\n");
			return false;
		}
		if (!ivGoalPoseSetUp || !ivStartPoseSetUp)
		{
			PRINT_ERROR("FootstepPlanner has not set the start and/or goal pose "
				"yet.\n");
			return false;
		}

		if (force_new_plan
			|| ivPlannerType == "RSTARPlanner" || ivPlannerType == "ARAPlanner")
		{
			reset();
		}
		// start the planning and return success
		return run();
	}


	bool
		FootstepPlanner::replan()
	{
		return plan(false);
	}


	//bool
	//FootstepPlanner::plan(const geometry_msgs::PoseStampedConstPtr start,
	//                      const geometry_msgs::PoseStampedConstPtr goal)
	//{
	//  return plan(start->pose.position.x, start->pose.position.y,
	//              tf::getYaw(start->pose.orientation),
	//              goal->pose.position.x, goal->pose.position.y,
	//              tf::getYaw(goal->pose.orientation));
	//}


	bool
		FootstepPlanner::plan(float start_x, float start_y, float start_theta,
			float goal_x, float goal_y, float goal_theta)
	{
		//if (!(setStart(start_x, start_y, start_theta) &&
		//    setGoal(goal_x, goal_y, goal_theta)))
		//{
		//  return false;
		//}

		return plan(false);
	}


	//bool
	//FootstepPlanner::planService(humanoid_nav_msgs::PlanFootsteps::Request &req,
	//                             humanoid_nav_msgs::PlanFootsteps::Response &resp)
	//{
	//  bool result = plan(req.start.x, req.start.y, req.start.theta,
	//                     req.goal.x, req.goal.y, req.goal.theta);
	//
	//  resp.costs = getPathCosts();
	//  resp.footsteps.reserve(getPathSize());
	//  resp.final_eps = ivPlannerPtr->get_final_epsilon();
	//  resp.expanded_states = ivPlannerEnvironmentPtr->getNumExpandedStates();
	//  extractFootstepsSrv(resp.footsteps);
	//
	//  resp.result = result;
	//
	//  // return true since service call was successful (independent from the
	//  // success of the planning call)
	//  return true;
	//}


	//bool
	//FootstepPlanner::planFeetService(humanoid_nav_msgs::PlanFootstepsBetweenFeet::Request &req,
	//                             humanoid_nav_msgs::PlanFootstepsBetweenFeet::Response &resp)
	//{
	//	//plan_footsteps_feet
	//  // TODO check direction and change of states, force planning from scratch if does not fit
	//  setStart(State(req.start_left.pose.x, req.start_left.pose.y, req.start_left.pose.theta, LEFT),
	//           State(req.start_right.pose.x, req.start_right.pose.y, req.start_right.pose.theta, RIGHT));
	//  setGoal(State(req.goal_left.pose.x, req.goal_left.pose.y, req.goal_left.pose.theta, LEFT),
	//           State(req.goal_right.pose.x, req.goal_right.pose.y, req.goal_right.pose.theta, RIGHT));
	//
	//  bool result = plan(false);
	//
	//  resp.costs = getPathCosts();
	//  resp.footsteps.reserve(getPathSize());
	//  resp.final_eps = ivPlannerPtr->get_final_epsilon();
	//  resp.expanded_states = ivPlannerEnvironmentPtr->getNumExpandedStates();
	//  extractFootstepsSrv(resp.footsteps);
	//
	//  resp.result = result;
	//
	//  // return true since service call was successful (independent from the
	//  // success of the planning call)
	//  return true;
	//}

	//void
	//FootstepPlanner::extractFootstepsSrv(std::vector<humanoid_nav_msgs::StepTarget> & footsteps) const{
	//  humanoid_nav_msgs::StepTarget foot;
	//  state_iter_t path_iter;
	//  for (path_iter = getPathBegin(); path_iter != getPathEnd(); ++path_iter)
	//  {
	//    foot.pose.x = path_iter->getX();
	//    foot.pose.y = path_iter->getY();
	//    foot.pose.theta = path_iter->getTheta();
	//    if (path_iter->getLeg() == LEFT)
	//      foot.leg = humanoid_nav_msgs::StepTarget::left;
	//    else if (path_iter->getLeg() == RIGHT)
	//      foot.leg = humanoid_nav_msgs::StepTarget::right;
	//    else
	//    {
	//      PRINT_ERROR("Footstep pose at (%f, %f, %f) is set to NOLEG!",
	//                path_iter->getX(), path_iter->getY(),
	//                path_iter->getTheta());
	//      continue;
	//    }
	//
	//    footsteps.push_back(foot);
	//  }
	//
	//}

	//
	//void
	//FootstepPlanner::goalPoseCallback(
	//    const geometry_msgs::PoseStampedConstPtr& goal_pose)
	//{
	//  // update the goal states in the environment
	//  if (setGoal(goal_pose))
	//  {
	//    if (ivStartPoseSetUp)
	//    {
	//      // force planning from scratch when backwards direction
	//      plan(!ivEnvironmentParams.forward_search);
	//    }
	//  }
	//}
	//
	//void
	//	FootstepPlanner::goalPoseCallback(
	//		std::string yamlPath, std::string fileName)
	//{
	//	// update the goal states in the environment
	//	if (setGoal(goal_pose))
	//	{
	//		if (ivStartPoseSetUp)
	//		{
	//			// force planning from scratch when backwards direction
	//			plan(!ivEnvironmentParams.forward_search);
	//		}
	//	}
	//}
	//void
	//FootstepPlanner::startPoseCallback(
	//    const geometry_msgs::PoseWithCovarianceStampedConstPtr& start_pose)
	//{
	//  if (setStart(start_pose->pose.pose.position.x,
	//               start_pose->pose.pose.position.y,
	//               tf::getYaw(start_pose->pose.pose.orientation)))
	//  {
	//    if (ivGoalPoseSetUp)
	//    {
	//      // force planning from scratch when forward direction
	//      plan(ivEnvironmentParams.forward_search);
	//    }
	//  }
	//}
	//
	//
	//void
	//FootstepPlanner::mapCallback(
	//    const nav_msgs::OccupancyGridConstPtr& occupancy_map)
	//{
	//  GridMap2DPtr map(new GridMap2D(occupancy_map));
	//
	//  // new map: update the map information
	//  if (updateMap(map))
	//  {
	//    // NOTE: update map currently simply resets the planner, i.e. replanning
	//    // here is in fact a planning from the scratch
	//    plan(false);
	//  }
	//}
	//

	//
	//bool
	//FootstepPlanner::setGoal(const geometry_msgs::PoseStampedConstPtr goal_pose)
	//{
	//  return setGoal(goal_pose->pose.position.x,
	//                 goal_pose->pose.position.y,
	//                 tf::getYaw(goal_pose->pose.orientation));
	//}
	//
	//
	bool
	FootstepPlanner::setGoal(float x, float y, float theta)
	{
	  if (!ivMapPtr)
	  {
	    PRINT_ERROR("Distance map hasn't been initialized yet.\n");
	    return false;
	  }

	
	  State goal(x, y, theta, NOLEG);
	  State foot_left = getFootPose(goal, LEFT);
	  State foot_right = getFootPose(goal, RIGHT);
	  ivGoalFoot = goal;
	
	  if (ivPlannerEnvironmentPtr->occupiedStartGoal(foot_left) ||
	      ivPlannerEnvironmentPtr->occupiedStartGoal(foot_right))
	  {
	    PRINT_ERROR("Goal pose at (%f %f %f) not accessible.\n", x, y, theta);
	    ivGoalPoseSetUp = false;
	    return false;
	  }
	  ivGoalFootLeft = foot_left;
	  ivGoalFootRight = foot_right;
	
	  ivGoalPoseSetUp = true;
	  PRINT_INFO("Goal pose set to (%f %f %f)\n", x, y, theta);
	
	  return true;
	}
	
	//bool
	//FootstepPlanner::setGoal(const State& left_foot, const State& right_foot)
	//{
	//  if (ivPlannerEnvironmentPtr->occupied(left_foot) ||
	//      ivPlannerEnvironmentPtr->occupied(right_foot))
	//  {
	//    ivGoalPoseSetUp = false;
	//    return false;
	//  }
	//  ivGoalFootLeft = left_foot;
	//  ivGoalFootRight = right_foot;
	//
	//  ivGoalPoseSetUp = true;
	//
	//  return true;
	//}
	//
	//
	//bool
	//FootstepPlanner::setStart(const geometry_msgs::PoseStampedConstPtr start_pose)
	//{
	//  return setStart(start_pose->pose.position.x,
	//                  start_pose->pose.position.y,
	//                  tf::getYaw(start_pose->pose.orientation));
	//}
	
	
	bool
	FootstepPlanner::setStart(const State& left_foot, const State& right_foot)
	{
	  if (ivPlannerEnvironmentPtr->occupiedStartGoal(left_foot) ||
	      ivPlannerEnvironmentPtr->occupiedStartGoal(right_foot))
	  {
	    ivStartPoseSetUp = false;
	    return false;
	  }
	  ivStartFootLeft = left_foot;
	  ivStartFootRight = right_foot;
	
	  ivStartPoseSetUp = true;
	
	  return true;
	}
	
	
	bool
	FootstepPlanner::setStart(float x, float y, float theta)
	{
	  if (!ivMapPtr)
	  {
	    PRINT_ERROR("Distance map hasn't been initialized yet.");
	    return false;
	  }



	  State start(x, y, theta, NOLEG);
	  State foot_left = getFootPose(start, LEFT);
	  State foot_right = getFootPose(start, RIGHT);
	  ivStartFoot = start;
	
	  bool success = setStart(foot_left, foot_right);
	  if (success)
	    PRINT_INFO("Start pose set to (%f %f %f)\n", x, y, theta);
	  else
	    PRINT_ERROR("Start pose (%f %f %f) not accessible.\n", x, y, theta);
	
	  // publish visualization:
	  //geometry_msgs::PoseStamped start_pose;
	  //start_pose.pose.position.x = x;
	  //start_pose.pose.position.y = y;
	  //start_pose.pose.position.z = 0.025;
	  //start_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
	  //start_pose.header.frame_id = ivMapPtr->getFrameID();
	  //start_pose.header.stamp = ros::Time::now();
	  //ivStartPoseVisPub.publish(start_pose);
	
	  return success;
	}

	int FootstepPlanner::LoadMap(std::string mapfile)
	{
		GridMap2DPtr map(new GridMap2D(mapfile, 0));
		int r=map->LoadMap(mapfile, 0);
		watch_binaryMap = map->m_binaryMap;
		watch_distMap = map->m_distMap;
		if (r == -1)
			return r;
		r=updateMap(map);
		return r;
	}

	bool
	FootstepPlanner::updateMap(const GridMap2DPtr map)
	{
	  // store old map pointer locally
	  GridMap2DPtr old_map = ivMapPtr;
	  // store new map
	  ivMapPtr.reset();
	  ivMapPtr = map;
	
	  // check if a previous map and a path existed
	  if (old_map && (bool)ivPath.size())
	  {
	    updateEnvironment(old_map);
	    return true;
	  }
	  // ..otherwise the environment's map can simply be updated
	  ivPlannerEnvironmentPtr->updateMap(map);
	  return false;
	}


	void
	FootstepPlanner::updateEnvironment(const GridMap2DPtr old_map)
	{
	  PRINT_INFO("Reseting the planning environment.");
	  // reset environment
	  resetTotally();
	  // set the new map
	  ivPlannerEnvironmentPtr->updateMap(ivMapPtr);
	
	
	}

	int StartGoalInfo::getGoal(std::string fileName)
	{
		cv::FileStorage fs_param;
		fs_param.open(fileName, cv::FileStorage::READ);
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

	int StartGoalInfo::getStart(std::string fileName)
	{
		cv::FileStorage fs_param;
		fs_param.open(fileName, cv::FileStorage::READ);
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

	void FootstepPlanner::LoadStartPose(string filepath)
	{
		StartGoalInfo startInfo;
		startInfo.getStart(filepath);
		
		//BezierInfo BezierPara;

		oriInfo ori = startInfo.orientation;
		if (setStart(startInfo.position.x,startInfo.position.y,
			tf::getYaw(ori.x, ori.y, ori.z, ori.w)))
		{
			if (ivGoalPoseSetUp)
			{
				// force planning from scratch when forward direction
				plan(ivEnvironmentParams.forward_search);
			}
		}

	}

	void FootstepPlanner::LoadGoalPose(string filepath)
	{
		StartGoalInfo goalInfo;
		goalInfo.getGoal(filepath);
		
		// update the goal states in the environment
		oriInfo ori = goalInfo.orientation;
		bool status = setGoal(goalInfo.position.x,goalInfo.position.y,
			tf::getYaw(ori.x, ori.y, ori.z, ori.w));
		
		if (status) 
		{
			pointF0.first = ivStartFoot.getX();
			pointF0.second = ivStartFoot.getY();
			pointF3.first = ivGoalFoot.getX();
			pointF3.second = ivGoalFoot.getY();

			///*获取参数化贝塞尔曲线通道，然后筛选可行的开始和目标处的候选脚印*/
			//selectBezierPare()
			pointF1.first = pointF0.first + BEZIERPARA * cos(ivStartFoot.getTheta());
			pointF1.second = pointF0.second + BEZIERPARA * sin(ivStartFoot.getTheta());
			pointF2.first = pointF3.first - BEZIERPARA * cos(ivGoalFoot.getTheta());
			pointF2.second = pointF3.second - BEZIERPARA * sin(ivGoalFoot.getTheta());

			ivMapPtr->initBezierMap();
			ivMapPtr->updateBezierMap(pointF0, pointF1, pointF2, pointF3);
			watchBezier_binaryMap = ivMapPtr->bezier_binaryMap;
			watchBezier_distMap = ivMapPtr->bezier_distMap;
			if (ivStartPoseSetUp)
			{
				// force planning from scratch when backwards direction
				plan(!ivEnvironmentParams.forward_search);
			}
		}

	}
	//void FootstepPlanner::selectBezierPare()
	//{
	//	getStateArea_FootstepSet(ivStartFootLeft, ivGoalFootLeft);

	//	std::vector<bezierInfo> curvatureSet; //存储多条贝塞尔曲线的相关信息
	//	curvatureSet.clear();
	//	for (float bezierParaD1 = 0.1; bezierParaD1 <= 1; bezierParaD1 += 0.1)
	//	{
	//		for (float bezierParaD2 = 0.1; bezierParaD2 <= 1; bezierParaD2 += 0.1)
	//		{
	//			pointF1.first = pointF0.first + bezierParaD1 * cos(ivStartFoot.getTheta());
	//			pointF1.second = pointF0.second + bezierParaD1 * sin(ivStartFoot.getTheta());
	//			pointF2.first = pointF3.first - bezierParaD2 * cos(ivGoalFoot.getTheta());
	//			pointF2.second = pointF3.second - bezierParaD2 * sin(ivGoalFoot.getTheta());
	//			ivMapPtr->initBezierMap();
	//			ivMapPtr->creatBezierMap(pointF0, pointF1, pointF2, pointF3);

	//			bezierInfo bezierCurAndSucc;
	//			bezierCurAndSucc.bezierParaD1 = bezierParaD1;
	//			bezierCurAndSucc.bezierParaD2 = bezierParaD2;
	//			for (state_iter_t path_iter = ivStartSuccFoot.begin(); path_iter != ivStartSuccFoot.end(); ++path_iter)
	//			{
	//				if (!ivPlannerEnvironmentPtr->occupied(*path_iter))
	//				{
	//					bezierCurAndSucc.startSucc += 1;
	//					//FootstepPath(*path_iter, ivMapPtr->bezier_binaryMap);
	//				}

	//			}
	//			for (state_iter_t path_iter = ivGoalPredFoot.begin(); path_iter != ivGoalPredFoot.end(); ++path_iter)
	//			{
	//				if (!ivPlannerEnvironmentPtr->occupied(*path_iter))
	//				{
	//					bezierCurAndSucc.goalSucc += 1;
	//					//FootstepPath(*path_iter, ivMapPtr->bezier_binaryMap);
	//				}

	//			}
	//			bezierCurAndSucc.curvatureStart = getCurvature(0);
	//			bezierCurAndSucc.curvatureGoal = getCurvature(1);

	//			curvatureSet.push_back(bezierCurAndSucc);

	//			//显示候选的脚印
	//		}

	//	}

	//	//FootstepPath(ivStartFoot, ivMapPtr->bezier_binaryMap);
	//	//显示候选脚印
	//	//FootstepPath(ivStartSuccFoot.begin(), ivMapPtr->bezier_binaryMap);
	//	for (state_iter_t path_iter = ivStartSuccFoot.begin(); path_iter != ivStartSuccFoot.end(); ++path_iter)
	//	{
	//		FootstepPath(*path_iter, ivMapPtr->bezier_binaryMap);
	//	}
	//	watchBezier_binaryMap = ivMapPtr->bezier_binaryMap;
	//	ivMapPtr->drawBezierMap(); //imshow
	//}
	float FootstepPlanner::getF1t(bool flag,float r)
	{
		float F1t;
		if (!flag)
		{
			F1t = -pointF0.first * 3 * pow((1 - r), 2) + 3 * pointF1.first*(3 * pow(r, 2) - 4 * r + 1) + 3 * pointF2.first*(2 * r - 3 * pow(r, 2)) + 3 * pointF3.first*pow(r, 2);
		}
		else
		{
			F1t = -pointF0.second * 3 * pow((1 - r), 2) + 3 * pointF1.second*(3 * pow(r, 2) - 4 * r + 1) + 3 * pointF2.second*(2 * r - 3 * pow(r, 2)) + 3 * pointF3.second*pow(r, 2);
		}
		return F1t;

	}
	float FootstepPlanner::getF2t(bool flag, float r)
	{
		float F2t;
		if (!flag)
		{
			F2t = 6 * pointF0.first*(1 - r) + 3 * pointF1.first*(6 * pow(r, 2) - 4) + 3 * pointF2.first*(2 - 6 * r) + 6 * pointF3.first*r;
		}
		else
		{
			F2t = 6 * pointF0.second*(1 - r) + 3 * pointF1.second*(6 * pow(r, 2) - 4) + 3 * pointF2.second*(2 - 6 * r) + 6 * pointF3.second*r;
		}
		return F2t;

	}
	float FootstepPlanner::getCurvature(float r)
	{
		float F1t_x = getF1t(0, r);
		float F1t_y = getF1t(1, r);
		float F2t_x = getF2t(0, r);
		float F2t_y = getF2t(1, r);

		float Kt = abs(F1t_x *F2t_y - F1t_y * F2t_x) / pow(pow(F1t_x, 2) + pow(F1t_y, 2), 1.5);
		return Kt;
	}
	void
		FootstepPlanner::isOccupiedStateArea(const State& Start, const State& Goal)
	{
		ivStartSuccFoot.clear();
		ivStartSuccFoot.push_back(Start);

		ivGoalPredFoot.clear();
		ivGoalPredFoot.push_back(Goal);
		FootstepPath(Start, ivMapPtr->bezier_binaryMap);
		FootstepPath(Goal, ivMapPtr->bezier_binaryMap);
		watchBezier_binaryMap = ivMapPtr->bezier_binaryMap;

		PlanningState stateGoal(Goal, ivEnvironmentParams.cell_size, ivEnvironmentParams.num_angle_bins, ivEnvironmentParams.hash_table_size);
		PlanningState stateStart(Start, ivEnvironmentParams.cell_size, ivEnvironmentParams.num_angle_bins, ivEnvironmentParams.hash_table_size);
		std::vector<Footstep>::const_iterator footstep_set_iter;
		for (footstep_set_iter = ivEnvironmentParams.footstep_set.begin();
			footstep_set_iter != ivEnvironmentParams.footstep_set.end();
			++footstep_set_iter)
		{

			PlanningState pred = footstep_set_iter->reverseMeOnThisState(stateGoal);
			float x = cell_2_state(pred.getX(), ivEnvironmentParams.cell_size);
			float y = cell_2_state(pred.getY(), ivEnvironmentParams.cell_size);
			float theta = angle_cell_2_state(pred.getTheta(), ivEnvironmentParams.num_angle_bins);
			State goalPred(x, y, theta, pred.getLeg());
			ivGoalPredFoot.push_back(goalPred);
			FootstepPath(goalPred, ivMapPtr->bezier_binaryMap);
			watchBezier_binaryMap = ivMapPtr->bezier_binaryMap;

			PlanningState succ = footstep_set_iter->performMeOnThisState(stateStart);
			x = cell_2_state(succ.getX(), ivEnvironmentParams.cell_size);
			y = cell_2_state(succ.getY(), ivEnvironmentParams.cell_size);
			theta = angle_cell_2_state(succ.getTheta(), ivEnvironmentParams.num_angle_bins);
			State startSucc(x, y, theta, succ.getLeg());
			ivStartSuccFoot.push_back(startSucc);
			FootstepPath(startSucc, ivMapPtr->bezier_binaryMap);
			watchBezier_binaryMap = ivMapPtr->bezier_binaryMap;


		}
	}
	void
		FootstepPlanner::getStateArea_FootstepSet(const State& Start, const State& Goal)
	{
		ivStartSuccFoot.clear();
		ivStartSuccFoot.push_back(Start);

		ivGoalPredFoot.clear();
		ivGoalPredFoot.push_back(Goal);
		//FootstepPath(Start, ivMapPtr->bezier_binaryMap);
		//FootstepPath(Goal, ivMapPtr->bezier_binaryMap);
		//watchBezier_binaryMap = ivMapPtr->bezier_binaryMap;

		PlanningState stateGoal(Goal, ivEnvironmentParams.cell_size, ivEnvironmentParams.num_angle_bins, ivEnvironmentParams.hash_table_size);
		PlanningState stateStart(Start, ivEnvironmentParams.cell_size, ivEnvironmentParams.num_angle_bins, ivEnvironmentParams.hash_table_size);
		std::vector<Footstep>::const_iterator footstep_set_iter;
		for (footstep_set_iter = ivEnvironmentParams.footstep_set.begin();
			footstep_set_iter != ivEnvironmentParams.footstep_set.end();
			++footstep_set_iter)
		{
	
			PlanningState pred = footstep_set_iter->reverseMeOnThisState(stateGoal);
			float x = cell_2_state(pred.getX(), ivEnvironmentParams.cell_size);
			float y = cell_2_state(pred.getY(), ivEnvironmentParams.cell_size);
			float theta = angle_cell_2_state(pred.getTheta(), ivEnvironmentParams.num_angle_bins);
			State goalPred(x, y, theta ,pred.getLeg());
			ivGoalPredFoot.push_back(goalPred);
			//FootstepPath(goalPred, ivMapPtr->bezier_binaryMap);
			//watchBezier_binaryMap = ivMapPtr->bezier_binaryMap;
			
			PlanningState succ = footstep_set_iter->performMeOnThisState(stateStart);
			x = cell_2_state(succ.getX(), ivEnvironmentParams.cell_size);
			y = cell_2_state(succ.getY(), ivEnvironmentParams.cell_size);
			theta = angle_cell_2_state(succ.getTheta(), ivEnvironmentParams.num_angle_bins);
			State startSucc(x, y, theta, succ.getLeg());
			ivStartSuccFoot.push_back(startSucc);
			//FootstepPath(startSucc, ivMapPtr->bezier_binaryMap);
			//watchBezier_binaryMap = ivMapPtr->bezier_binaryMap;
			
			/*if (occupied(succ) || !reachable(left, succ))
				continue;
			p_state = createHashEntryIfNotExists(succ);
			ivStateArea.push_back(p_state->getId());

			succ = footstep_set_iter->performMeOnThisState(right);
			if (occupied(succ) || !reachable(right, succ))
				continue;
			p_state = createHashEntryIfNotExists(succ);
			ivStateArea.push_back(p_state->getId());*/
	
		}
	}
	State
		FootstepPlanner::getFootPose(const State& robot, Leg leg)
	{
		double shift_x = -sin(robot.getTheta()) * ivFootSeparation / 2.0;
		double shift_y = cos(robot.getTheta()) * ivFootSeparation / 2.0;

		double sign = -1.0;
		if (leg == LEFT)
			sign = 1.0;

		return State(robot.getX() + sign * shift_x,
			robot.getY() + sign * shift_y,
			robot.getTheta(),
			leg);
	}


	bool
		FootstepPlanner::pathIsNew(const std::vector<int>& new_path)
	{
		if (new_path.size() != ivPlanningStatesIds.size())
			return true;

		bool unequal = true;
		for (unsigned i = 0; i < new_path.size(); ++i)
			unequal = new_path[i] != ivPlanningStatesIds[i] && unequal;

		return unequal;
	}

	
	//void
	//FootstepPlanner::clearFootstepPathVis(unsigned num_footsteps)
	//{
	//  visualization_msgs::Marker marker;
	//  visualization_msgs::MarkerArray marker_msg;
	//
	//  marker.header.stamp = ros::Time::now();
	//  marker.header.frame_id = ivMapPtr->getFrameID();
	//
	//
	//  if (num_footsteps < 1)
	//    num_footsteps = ivLastMarkerMsgSize;
	//
	//  for (unsigned i = 0; i < num_footsteps; ++i)
	//  {
	//    marker.ns = ivMarkerNamespace;
	//    marker.id = i;
	//    marker.action = visualization_msgs::Marker::DELETE;
	//
	//    marker_msg.markers.push_back(marker);
	//  }
	//
	//  ivFootstepPathVisPub.publish(marker_msg);
	//  PRINT_INFO("clearFootstepPathVis called");
	//}


	//void
	//FootstepPlanner::broadcastExpandedNodesVis()
	//{
	//  if (ivExpandedStatesVisPub.getNumSubscribers() > 0)
	//  {
	//    sensor_msgs::PointCloud cloud_msg;
	//    geometry_msgs::Point32 point;
	//    std::vector<geometry_msgs::Point32> points;
	//
	//    State s;
	//    FootstepPlannerEnvironment::exp_states_2d_iter_t state_id_it;
	//    for(state_id_it = ivPlannerEnvironmentPtr->getExpandedStatesStart();
	//        state_id_it != ivPlannerEnvironmentPtr->getExpandedStatesEnd();
	//        ++state_id_it)
	//    {
	//      point.x = cell_2_state(state_id_it->first,
	//                             ivEnvironmentParams.cell_size);
	//      point.y = cell_2_state(state_id_it->second,
	//                             ivEnvironmentParams.cell_size);
	//      point.z = 0.01;
	//      points.push_back(point);
	//    }
	//    cloud_msg.header.stamp = ros::Time::now();
	//    cloud_msg.header.frame_id = ivMapPtr->getFrameID();
	//
	//    cloud_msg.points = points;
	//	
	//    ivExpandedStatesVisPub.publish(cloud_msg);
	//	PRINT_INFO("broadcastExpandedNodesVis called");
	//  }
	//}
	//
	//
	//void
	//FootstepPlanner::broadcastFootstepPathVis()
	//{
	//  if (getPathSize() == 0)
	//  {
	//    PRINT_INFO("no path has been extracted yet");
	//    return;
	//  }
	//
	//  clearFootstepPathVis(0);
	//
	//  visualization_msgs::Marker marker;
	//  visualization_msgs::MarkerArray broadcast_msg;
	//  std::vector<visualization_msgs::Marker> markers;
	//
	//  int markers_counter = 0;
	//
	//  marker.header.stamp = ros::Time::now();
	//  marker.header.frame_id = ivMapPtr->getFrameID();
	//
	//  // add the missing start foot to the publish vector for visualization:
	//  if (ivPath.front().getLeg() == LEFT)
	//    footPoseToMarker(ivStartFootRight, &marker);
	//  else
	//    footPoseToMarker(ivStartFootLeft, &marker);
	//  marker.id = markers_counter++;
	//  markers.push_back(marker);
	//
	//  // add the footsteps of the path to the publish vector
	//  for(state_iter_t path_iter = getPathBegin(); path_iter != getPathEnd();
	//      ++path_iter)
	//  {
	//    footPoseToMarker(*path_iter, &marker);
	//    marker.id = markers_counter++;
	//    markers.push_back(marker);
	//  }
	//
	//  broadcast_msg.markers = markers;
	//  ivLastMarkerMsgSize = markers.size();
	//
	//  ivFootstepPathVisPub.publish(broadcast_msg);
	//  PRINT_INFO("broadcastFootstepPathVis called");
	//}

	//
	//void
	//FootstepPlanner::broadcastRandomNodesVis()
	//{
	//  if (ivRandomStatesVisPub.getNumSubscribers() > 0){
	//    sensor_msgs::PointCloud cloud_msg;
	//    geometry_msgs::Point32 point;
	//    std::vector<geometry_msgs::Point32> points;
	//    visualization_msgs::Marker marker;
	//    visualization_msgs::MarkerArray broadcast_msg;
	//    std::vector<visualization_msgs::Marker> markers;
	//
	//    marker.header.stamp = ros::Time::now();
	//    marker.header.frame_id = ivMapPtr->getFrameID();
	//
	//    State s;
	//    FootstepPlannerEnvironment::exp_states_iter_t state_id_iter;
	//    for(state_id_iter = ivPlannerEnvironmentPtr->getRandomStatesStart();
	//        state_id_iter != ivPlannerEnvironmentPtr->getRandomStatesEnd();
	//        ++state_id_iter)
	//    {
	//      if (!ivPlannerEnvironmentPtr->getState(*state_id_iter, &s))
	//      {
	//        PRINT_WARN("Could not get random state %d", *state_id_iter);
	//      }
	//      else
	//      {
	//        point.x = s.getX();
	//        point.y = s.getY();
	//        point.z = 0.01;
	//        points.push_back(point);
	//      }
	//    }
	//    cloud_msg.header.stamp = ros::Time::now();
	//    cloud_msg.header.frame_id = ivMapPtr->getFrameID();
	//
	//    cloud_msg.points = points;
	//
	//    ivRandomStatesVisPub.publish(cloud_msg);
	//	PRINT_INFO("broadcastRandomNodesVis called");
	//  }
	//}
	//
	//
	//void
	//FootstepPlanner::broadcastPathVis()
	//{
	//  if (getPathSize() == 0)
	//  {
	//    PRINT_INFO("no path has been extracted yet");
	//    return;
	//  }
	//
	//  nav_msgs::Path path_msg;
	//  geometry_msgs::PoseStamped state;
	//
	//  state.header.stamp = ros::Time::now();
	//  state.header.frame_id = ivMapPtr->getFrameID();
	//
	//  state_iter_t path_iter;
	//  for(path_iter = getPathBegin(); path_iter != getPathEnd(); ++path_iter)
	//  {
	//    state.pose.position.x = path_iter->getX();
	//    state.pose.position.y = path_iter->getY();
	//    path_msg.poses.push_back(state);
	//  }
	//
	//  path_msg.header = state.header;
	//  ivPathVisPub.publish(path_msg);
	//  PRINT_INFO("broadcastPathVis called");
	//}
	//
	//
	//void
	//FootstepPlanner::footPoseToMarker(const State& foot_pose,
	//                                  visualization_msgs::Marker* marker)
	//{
	//  marker->header.stamp = ros::Time::now();
	//  marker->header.frame_id = ivMapPtr->getFrameID();
	//  marker->ns = ivMarkerNamespace;
	//  marker->type = visualization_msgs::Marker::CUBE;
	//  marker->action = visualization_msgs::Marker::ADD;
	//
	//  float cos_theta = cos(foot_pose.getTheta());
	//  float sin_theta = sin(foot_pose.getTheta());
	//  float x_shift = cos_theta * ivEnvironmentParams.foot_origin_shift_x -
	//                  sin_theta * ivEnvironmentParams.foot_origin_shift_y;
	//  float y_shift;
	//  if (foot_pose.getLeg() == LEFT)
	//    y_shift = sin_theta * ivEnvironmentParams.foot_origin_shift_x +
	//              cos_theta * ivEnvironmentParams.foot_origin_shift_y;
	//  else // leg == RLEG
	//    y_shift = sin_theta * ivEnvironmentParams.foot_origin_shift_x -
	//              cos_theta * ivEnvironmentParams.foot_origin_shift_y;
	//  marker->pose.position.x = foot_pose.getX() + x_shift;
	//  marker->pose.position.y = foot_pose.getY() + y_shift;
	//  marker->pose.position.z = ivEnvironmentParams.footsize_z / 2.0;
	//  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(foot_pose.getTheta()),
	//                        marker->pose.orientation);
	//
	//  marker->scale.x = ivEnvironmentParams.footsize_x; // - 0.01;
	//  marker->scale.y = ivEnvironmentParams.footsize_y; // - 0.01;
	//  marker->scale.z = ivEnvironmentParams.footsize_z;
	//
	//  // TODO: make color configurable?
	//  if (foot_pose.getLeg() == RIGHT)
	//  {
	//    marker->color.r = 0.0f;
	//    marker->color.g = 1.0f;
	//  }
	//  else // leg == LEFT
	//      {
	//    marker->color.r = 1.0f;
	//    marker->color.g = 0.0f;
	//      }
	//  marker->color.b = 0.0;
	//  marker->color.a = 0.6;
	//
	//  marker->lifetime = ros::Duration();
	//}

}


