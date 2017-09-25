//
// Created by hvrigazov on 17.09.17.
//

#include <limits>
#include "World.h"

CartesianPoint World::to_xy(double s, double d) const {
  size_t first_waypoint = closest_waypoint_index(s);
  size_t second_waypoint = (first_waypoint + 1) % waypoints.size();

  double heading = atan2(waypoints[second_waypoint].y - waypoints[first_waypoint].y,
                         waypoints[second_waypoint].x - waypoints[first_waypoint].x);
  double seg_s = s - waypoints[first_waypoint].s;
  double seg_x = waypoints[first_waypoint].x + seg_s * cos(heading);
  double seg_y = waypoints[first_waypoint].y + seg_s * sin(heading);
  double perp = heading - pi() / 2;

  double x = seg_x + d * cos(perp);
  double y = seg_y + d * sin(perp);

  return {x, y};
}

size_t World::closest_waypoint_index(double s) const {
  size_t i;
  for (i = 0; i < waypoints.size(); i++) {
    if (s <= waypoints[i].s) {
      return i - 1;
    }
  }
  return i;
}

World::World(const string &filename) {
  ifstream in_map_(filename, ifstream::in);
  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);

    double x, y, s, dx, dy;
    iss >> x >> y >> s >> dx >> dy;

    waypoints.x.push_back(x);
    waypoints.y.push_back(y);
    waypoints.s.push_back(s);
    waypoints.dx.push_back(dx);
    waypoints.dy.push_back(dy);
  }

  // Add the last point
  waypoints.s.push_back(max_s);
  waypoints.x.push_back(waypoints.x[0]);
  waypoints.y.push_back(waypoints.y[0]);
  waypoints.dx.push_back(waypoints.dx[0]);
  waypoints.dy.push_back(waypoints.dy[0]);
}
