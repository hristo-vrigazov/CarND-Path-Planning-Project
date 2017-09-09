//
// Created by hvrigazov on 09.09.17.
//

#include "BehaviorExecutor.h"

void BehaviorExecutor::applyKeepLaneBehavior(double current_speed,
                                             const CarPosition &car,
                                             vector<CarPosition> nearby_cars,
                                             int lane,
                                             double jmt_v_finish) {

  vector<CarPosition> carsInFront = filterCarsInFrontOf(nearby_cars, car.s);
  vector<CarPosition> nearbyInLane = filterCarsByLane(carsInFront, lane);
  sortCarsByDistance(nearbyInLane, car);

  double min_dist = car.speed * 2;

  double target_speed = MAX_VELOCITY;

  if (nearbyInLane.size() > 0 && distanceS(nearbyInLane[0].s, car.s) < min_dist) {
    target_speed = nearbyInLane[0].speed - 5;
    if (distanceS(nearbyInLane[0].s, car.s) < 10) {
      cout << "Very close, almost colided!\n";
      target_speed -= 5;
    }
  }

  double targetDiff = fabs(target_speed - jmt_v_finish);
  if (targetDiff > 1.0) {
    adjustTargetSpeed(current_speed, target_speed, 0);
  }
}

vector<CarPosition> BehaviorExecutor::filterCarsInFrontOf(vector<CarPosition> cars, double s) {
  vector<CarPosition> filtered;
  for (auto car: cars) {
    if (isInFront(s, car.s)) {
      filtered.push_back(car);
    }
  }
  return filtered;

}

vector<CarPosition> BehaviorExecutor::filterCarsByLane(vector<CarPosition> cars, int lane) {
  vector<CarPosition> filtered;

  for (auto car: cars) {
    if (getLaneNumber(car.d) == lane) {
      filtered.push_back(car);
    }
  }

  return filtered;
}

void BehaviorExecutor::sortCarsByDistance(vector<CarPosition> &cars, const CarPosition &carPosition) {
  sort(cars.begin(), cars.end(), [&](const CarPosition & lhs, const CarPosition &rhs) {
      return distanceS(carPosition.s, lhs.s) < distanceS(carPosition.s, rhs.s);
  });
}

void BehaviorExecutor::adjustTargetSpeed(double currentSpeed, double targetSpeed, double deltaT) {

}

