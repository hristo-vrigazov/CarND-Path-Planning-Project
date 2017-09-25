//
// Created by hvrigazov on 17.09.17.
//

#ifndef PATH_PLANNING_TELEMETRYDATA_H
#define PATH_PLANNING_TELEMETRYDATA_H


#include "Path.h"
#include "FrenetPoint.h"
#include "Car.h"
#include "CarState.h"

class TelemetryData {
public:
  CarState car_state;
  Path previous_path;
  FrenetPoint end_path;
  vector<Car> other_cars;
};


#endif //PATH_PLANNING_TELEMETRYDATA_H
