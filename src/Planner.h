//
// Created by hvrigazov on 31.08.17.
//
#ifndef PLANNER_H
#define PLANNER_H

#include <map>
#include <vector>
#include <string>


#include "Map.h"
#include "TrajectoryPlanner.h"
#include "Planner.h"
#include "PlannedPoints.h"
#include "CarPosition.h"

using namespace std;

class Planner {
 private:
  double previous_s;
  double previous_d;
  double previous_v;
  bool initialized;

  TrajectoryPlanner trajectory_planner;

  Map map_;

  vector<CarPosition> findCarsThatAreNear();


public:

  Planner(Map map_file_);

  ~Planner();

  PlannedPoints plan(const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y);

  CarPosition my_car_;

  std::map<int, CarPosition> other_cars_;
};
#endif /* PLANNER_H */
