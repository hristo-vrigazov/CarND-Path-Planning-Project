//
// Created by hvrigazov on 31.08.17.
//
#include "Planner.h"

#include "Constants.h"
#include "CarPosition.h"

Planner::Planner(Map map_file_) : map_(map_file_) {
	initialized = false;
}

Planner::~Planner() {}

PlannedPoints Planner::plan(const std::vector<double> & previous_path_x,
														const std::vector<double> & previous_path_y) {
  if (!initialized) {
    previous_s = my_car_.s;
    previous_d = my_car_.d;
    map_.padSplines();
    initialized = true;
  }

  vector<double> next_x_values;
  vector<double> next_y_values;

  unsigned long path_size = previous_path_x.size();
  for (int i = 0; i < path_size; i++) {
    next_x_values.push_back(previous_path_x[i]);
    next_y_values.push_back(previous_path_y[i]);
  }

  vector<CarPosition> carsThatAreNear = findCarsThatAreNear();
  trajectory_planner.generateGoals(map_, my_car_, carsThatAreNear, previous_v);

  while(next_x_values.size() < NUMBER_OF_INTERMEDIATE_POINTS) {
    trajectory_planner.jmt_v_time += SAMPLING_RATE;
    previous_v = trajectory_planner.getDeltaV(trajectory_planner.jmt_v_time);

    previous_v = (previous_v > MAX_VELOCITY) ? MAX_VELOCITY : previous_v;

    previous_s = previous_s + previous_v * SAMPLING_RATE;

    trajectory_planner.jmt_d_time += SAMPLING_RATE;
    previous_d = trajectory_planner.getDeltaD(trajectory_planner.jmt_d_time);

    // Map to spline
    vector<double> new_xy = map_.getXY(previous_s, previous_d);
    next_x_values.push_back(new_xy[0]);
    next_y_values.push_back(new_xy[1]);
  }

  return PlannedPoints(next_x_values, next_y_values);
}

// C++ does not have stream API or something similar, so we are stuck with this implementation
vector<CarPosition> Planner::findCarsThatAreNear() {
	vector<CarPosition> cars;

	for (auto const & x : other_cars_) {
		CarPosition car_position = x.second;
		if (car_position.distance_to_my_car < CAR_IS_NEAR_THRESHOLD) {
			cars.push_back(car_position);
		}
	}

	return cars;
}



