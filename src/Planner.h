//
// Created by hvrigazov on 31.08.17.
//
#ifndef PATHPLANNER_H
#define PATHPLANNER_H

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

public:
  Map map_;

  CarPosition my_car_;

  std::map<int, CarPosition> other_cars_;

  vector<double> next_x_values;
  vector<double> next_y_values;

  Planner(Map map_file_);

  ~Planner();

  PlannedPoints plan(const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y);

  void generateIntermediatePoints();

  vector<CarPosition> findCarsThatAreNear();


};
#endif /* PATHPLANNER_H */
