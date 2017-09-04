//
// Created by hvrigazov on 31.08.17.
//

#ifndef PATH_PLANNING_CARPOSITION_H
#define PATH_PLANNING_CARPOSITION_H


#include <map>
#include "json.hpp"
#include "Constants.h"

using json = nlohmann::json;

class CarPosition {
public:
  // frenet coordinates
  double s;
  double d;
  // speed in m/s
  double speed;

  double distance_to_my_car;

  CarPosition() : s(0), d(0), speed(0) {}

  CarPosition(const json& json_data): s(json_data["s"]),
                               d(json_data["d"]),
                               speed(json_data["speed"]) {}

  CarPosition(double s, double d, double vx, double vy): s(s),
                                                         d(d),
                                                         speed(cartesian2polar(vx, vy)[0]) {}
};


#endif //PATH_PLANNING_CARPOSITION_H
