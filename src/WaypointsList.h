//
// Created by hvrigazov on 17.09.17.
//

#ifndef PATH_PLANNING_WAYPOINTSLIST_H
#define PATH_PLANNING_WAYPOINTSLIST_H


#include <cstddef>
#include <vector>
#include "Waypoint.h"

class WaypointsList {
public:
  std::vector<double> x, y, s, dx, dy;
  size_t size() const { return x.size(); }
  Waypoint operator[](int i) const { return {x[i],y[i],s[i],dx[i],dy[i]}; }
};



#endif //PATH_PLANNING_WAYPOINTSLIST_H
