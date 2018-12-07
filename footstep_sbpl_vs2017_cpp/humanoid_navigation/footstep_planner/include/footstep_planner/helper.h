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

#ifndef FOOTSTEP_PLANNER_HELPER_H_
#define FOOTSTEP_PLANNER_HELPER_H_

#define DEBUG_HASH 0
#define DEBUG_TIME 0



#include <gridmap_2d/GridMap2D.h>
#include <angles/angles.h>
//#include <tf/tf.h>

//#define _USE_MATH_DEFINES
#include <math.h>

#define PRINT_ERROR printf
#define PRINT_INFO printf
#define PRINT_ERROR_STREAM printf
#define PRINT_INFO_STREAM printf
#define PRINT_WARN printf
#define PRINT_DEBUG printf

namespace footstep_planner
{
static const double TWO_PI = 2 * M_PI;

static const double FLOAT_CMP_THR = 0.0001;

enum Leg { RIGHT=0, LEFT=1, NOLEG=2 };


/**
 * @return Squared euclidean distance between two integer coordinates
 * (cells).
 */
inline double euclidean_distance_sq(int x1, int y1, int x2, int y2)
{
  // note: do *not* use pow() to square!
  return (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
}


/// @return Euclidean distance between two integer coordinates (cells).
inline double euclidean_distance(int x1, int y1, int x2, int y2)
{
  return sqrt(double(euclidean_distance_sq(x1, y1, x2, y2)));
}


/// @return Euclidean distance between two coordinates.
inline double euclidean_distance(double x1, double y1, double x2, double y2)
{
  return sqrt(euclidean_distance_sq(x1, y1, x2, y2));
}


/// @return Squared euclidean distance between two coordinates.
inline double euclidean_distance_sq(double x1, double y1, double x2,
                                    double y2)
{
  // note: do *not* use pow() to square!
  return (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
}


/// @return The distance of two neighbored cell.
inline double grid_cost(int x1, int y1, int x2, int y2, float cell_size)
{
  int x = abs(x1 - x2);
  int y = abs(y1 - y2);

  if (x + y > 1)
    return M_SQRT2 * cell_size;
  else
    return cell_size;
}


/// @brief Discretize a (continuous) angle into a bin.
inline int angle_state_2_cell(double angle, int angle_bin_num)
{
  double bin_size_half = M_PI / angle_bin_num;
  return int(angles::normalize_angle_positive(angle + bin_size_half) /
             TWO_PI * angle_bin_num);
}


/// @brief Calculate the respective (continuous) angle for a bin.
inline double angle_cell_2_state(int angle, int angle_bin_num)
{
  double bin_size = TWO_PI / angle_bin_num;
  return angle * bin_size;
}


/**
 * @brief Discretize a (continuous) state value into a cell. (Should be
 * used to discretize a State to a PlanningState.)
 */
inline int state_2_cell(float value, float cell_size)
{
  return value >= 0 ? int(value / cell_size) : int(value / cell_size) - 1;
}


/**
 * @brief Calculate the respective (continuous) state value for a cell.
 * (Should be used to get a State from a discretized PlanningState.)
 */
inline double cell_2_state(int value, double cell_size)
{
  return (double(value) + 0.5) * cell_size;
}


/// @brief Discretize a (continuous) value into cell size.
// TODO: check consistency for negative values
inline int disc_val(double length, double cell_size)
{
  return int(floor((length / cell_size) + 0.5));
}


/**
 * @brief Calculates the continuous value for a length discretized in cell
 * size.
 */
// TODO: check consistency for negative values
inline double cont_val(int length, double cell_size)
{
  return double(length * cell_size);
}


/// @return The hash value of the key.
inline unsigned int int_hash(int key)
{
  key += (key << 12);
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}


/**
 * @return The hash tag for a PlanningState (represented by x, y, theta and
 * leg).
 */
inline unsigned int calc_hash_tag(int x, int y, int theta, int leg,
                                  int max_hash_size)
{
  return int_hash((int_hash(x) << 3) + (int_hash(y) << 2) +
                  (int_hash(theta) << 1) + (int_hash(leg)))
      % max_hash_size;
}


/// @brief Rounding half towards zero.
inline int round(double r)
{
  return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}


/**
 * @brief Checks if a footstep (represented by its center and orientation)
 * collides with an obstacle. The check is done by recursively testing if
 * either the circumcircle of the foot, the inner circle of the foot or the
 * area in between has an appropriate distance to the nearest obstacle.
 *
 * @param x Global position of the foot in x direction.
 * @param y Global position of the foot in y direction.
 * @param theta Global orientation of the foot.
 * @param height Size of the foot in x direction.
 * @param width Size of the foot in y direction.
 * @param accuracy (0) circumcircle of the foot; (1) incircle of the foot;
 * (2) circumcircle and incircle recursivly checked for the whole foot
 * @param distance_map Contains distance information to the nearest
 * obstacle.
 *
 * @return True if the footstep collides with an obstacle.
 */
//bool collision_check(double x, double y, double theta,
//	double height, double width, int accuracy,
//	const gridmap_2d::GridMap2D& distance_map);

//bool
//collision_check(double x, double y, double theta, double height,
//                double width, int accuracy,
//                const gridmap_2d::GridMap2D& distance_map)
//{
//  double d = distance_map.distanceMapAt(x, y);
//  if (d < 0.0) // if out of bounds => collision
//    return true;
//  d -= distance_map.getResolution();
//
//  const double r_o = sqrt(width*width + height*height) / 2.0;
//
//  if (d >= r_o)
//    return false;
//  else if (accuracy == 0)
//    return false;
//
//  const double h_half = height / 2.0;
//  const double w_half = width / 2.0;
//  const double r_i = std::min(w_half, h_half);
//
//  if (d <= r_i)
//    return true;
//  else if (accuracy == 1)
//    return true;
//
//  double h_new;
//  double w_new;
//  double delta_x;
//  double delta_y;
//  if (width < height)
//  {
//    const double h_clear = sqrt(d*d - w_half*w_half);
//    h_new = h_half - h_clear;
//    w_new = width;
//    delta_x = h_clear + h_new / 2.0;
//    delta_y = 0.0;
//  }
//  else // footWidth >= footHeight
//  {
//    const double w_clear = sqrt(d*d - h_half*h_half);
//    h_new = height;
//    w_new = w_half - w_clear;
//    delta_x = 0.0;
//    delta_y = w_clear + w_new / 2.0;
//  }
//  const double theta_cos = cos(theta);
//  const double theta_sin = sin(theta);
//  const double x_shift = theta_cos*delta_x - theta_sin*delta_y;
//  const double y_shift = theta_sin*delta_x + theta_cos*delta_y;
//
//  return (collision_check(x+x_shift, y+y_shift, theta, h_new, w_new,
//                          accuracy, distance_map) ||
//          collision_check(x-x_shift, y-y_shift, theta, h_new, w_new,
//                          accuracy, distance_map));
//}
/**
 * @brief Crossing number method to determine whether a point lies within a
 * polygon or not.
 * @param edges (x,y)-points defining the polygon.
 *
 * Check http://geomalgorithms.com/a03-_inclusion.html for further details.
 */
bool pointWithinPolygon(int x, int y,
                        const std::vector<std::pair<int, int> >& edges);
}
#endif  /* FOOTSTEP_PLANNER_HELPER_H_ */
