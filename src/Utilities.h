//
// Created by hvrigazov on 17.09.17.
//

#ifndef PATH_PLANNING_UTILITIES_H
#define PATH_PLANNING_UTILITIES_H


#include <limits>
#include <math.h>
#include "json.hpp"
#include "Constants.h"

using namespace std;

using json = nlohmann::json;

constexpr double pi() { return M_PI; }
constexpr double deg2rad(double x) { return x * (M_PI / 180.0); }
constexpr double rad2deg(double x) { return x * (180.0 / M_PI); }

constexpr double INF = std::numeric_limits<double>::infinity();

constexpr double mph2mps(double x) { return x * MPH2MPS; }
constexpr double mps2mph(double x) { return x / MPH2MPS; }

static constexpr double MILE2METER = 1609.34;

constexpr double miles2meters(double x) { return x * MILE2METER; }
constexpr double meters2miles(double x) { return x / MILE2METER; }

inline double dot(double x1, double y1, double x2, double y2) {
  return x1 * x2 + y1 * y2;
}

inline double norm(double x, double y) {
  return sqrt(x * x + y * y);
}

inline double distance(double x1, double y1, double x2, double y2) {
  return norm(x2 - x1, y2 - y1);
}

inline double lane_center(int lane) {
  return (0.5 + lane) * LANE_WIDTH;
}

inline int lane_at(double d) {
  return int(std::floor(d / LANE_WIDTH));
}

#endif //PATH_PLANNING_UTILITIES_H
