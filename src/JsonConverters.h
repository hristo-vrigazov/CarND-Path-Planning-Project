//
// Created by hvrigazov on 17.09.17.
//

#ifndef PATH_PLANNING_JSONCONVERTERS_H
#define PATH_PLANNING_JSONCONVERTERS_H

#include "PathPlanner.h"
#include "Car.h"

// Those functions make it possible to easily convert from JSON to the given type

void from_json(const json & j, Car &sensor) {
  sensor.id = j[0].get<int>();
  sensor.x = j[1];
  sensor.y = j[2];
  sensor.vx = j[3];
  sensor.vy = j[4];
  sensor.s = j[5];
  sensor.d = j[6];
}

void from_json(const json & j, CartesianPoint & cartesianPoint) {
  cartesianPoint.x = j["x"];
  cartesianPoint.y = j["y"];
}

void from_json(const json & j, FrenetPoint & frenetPoint) {
  frenetPoint.s = j["s"];
  frenetPoint.d = j["d"];
}

void from_json(const json & j, CarState & carState) {
  carState.cartesian = j.get<CartesianPoint>();
  carState.yaw = deg2rad(j["yaw"]);
  carState.frenet = j.get<FrenetPoint>();
  carState.speed = mph2mps(j["speed"]);
}

void from_json(const json & j, TelemetryData &tele) {
  tele.car_state = j.get<CarState>();

  // Previous path data given to the Planner
  tele.previous_path.x = j["previous_path_x"].get<vector<double>>();
  tele.previous_path.y = j["previous_path_y"].get<vector<double>>();
  // Previous path's end s and d values
  tele.end_path.s = j["end_path_s"];
  tele.end_path.d = j["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  tele.other_cars = j["sensor_fusion"].get<vector<Car>>();
}

#endif //PATH_PLANNING_JSONCONVERTERS_H
