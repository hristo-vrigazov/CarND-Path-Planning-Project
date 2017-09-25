//
// Created by hvrigazov on 17.09.17.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H


#include <vector>
#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <functional>
#include <algorithm>
#include <cmath>
#include <random>
#include <iomanip>
#include "spline.h"
#include "World.h"
#include "Path.h"
#include "Car.h"
#include "TelemetryData.h"
#include "Lane.h"
#include "State.h"
#include "Constants.h"
#include "CarState.h"

using namespace std;

class PathPlanner {
public:
  PathPlanner(const World & world);
  Path plan(const TelemetryData &data);

private:
  World world;

  CarState current;

  int lane;

  CartesianPoint previous;

  Lane lane_gap_speeds[3];

  int target_lane = 1;

  double target_speed = 0;

  STATE state;

  void update(const TelemetryData &data);

  void compute_predictions(const TelemetryData &data);

  void create_plan(const TelemetryData &data);

  void speed_control();

  Path build_path(const TelemetryData &data);

  double get_speed_limit() const;

  double get_cross_track_error() const;

  double get_lane_cost(int i) const;

  int get_best_lane() const;

  void handle_start_state();

  void handle_keep_lane_state();

  void handle_lane_change();

  tk::spline create_spline() const;
};

#endif //PATH_PLANNING_PATHPLANNER_H
