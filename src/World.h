//
// Created by hvrigazov on 17.09.17.
//

#ifndef PATH_PLANNING_WORLD_H
#define PATH_PLANNING_WORLD_H

#include <math.h>
#include <string>
#include <fstream>
#include "CartesianPoint.h"
#include "FrenetPoint.h"
#include "Waypoint.h"
#include "WaypointsList.h"
#include "Utilities.h"

using namespace std;

class World {
public:
  World(const string & filename);

  const double max_s = 6945.554;

  CartesianPoint to_xy(double s, double d) const;

private:
  WaypointsList waypoints;

  size_t closest_waypoint_index(double s) const;
};



#endif //PATH_PLANNING_WORLD_H
