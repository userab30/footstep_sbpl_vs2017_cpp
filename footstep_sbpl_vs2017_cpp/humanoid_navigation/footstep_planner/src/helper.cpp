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

#include <footstep_planner/helper.h>

namespace footstep_planner
{

bool
pointWithinPolygon(int x, int y, const std::vector<std::pair<int, int> >& edges)
{
  int cn = 0;

  // loop through all edges of the polygon
  for(unsigned int i = 0; i < edges.size() - 1; ++i)
  {
    if ((edges[i].second <= y && edges[i + 1].second > y) ||
        (edges[i].second > y && edges[i + 1].second <= y))
    {
      float vt = (float)(y - edges[i].second) /
        (edges[i + 1].second - edges[i].second);
      if (x < edges[i].first + vt * (edges[i + 1].first - edges[i].first))
      {
        ++cn;
      }
    }
  }
  return cn & 1;
}
}
