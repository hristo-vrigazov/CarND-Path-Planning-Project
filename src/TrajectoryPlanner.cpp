//
// Created by hvrigazov on 31.08.17.
//
#include "TrajectoryPlanner.h"
#include "CarPosition.h"

TrajectoryPlanner::TrajectoryPlanner() {
  jmt_v.completed = true;
  jmt_d.completed = true;
  jmt_d.finish = getLaneCenter(INITIAL_LANE);
}

TrajectoryPlanner::~TrajectoryPlanner() {};


void TrajectoryPlanner::generateGoals(Map &map, CarPosition car, vector<CarPosition> nearby_cars, double prev_v) {
  // Check if a behavior is in progress
  if (jmt_d.completed && jmt_v.completed) {
    current_state = getMinimumCostBehavior(car, nearby_cars);

    int lane = getLaneNumber(car.d);
    switch (current_state) {
      case KEEP_LANE:
        applyKeepLaneBehavior(prev_v, car, nearby_cars, lane);
        return;
      case CHANGE_TO_LEFT_LANE:
        applyLaneChange(lane, lane - 1, car, nearby_cars);
        break;
      case CHANGE_TO_RIGHT_LANE:
        applyLaneChange(lane, lane + 1, car, nearby_cars);
        break;
    }
  }


}

BehaviorState TrajectoryPlanner::getMinimumCostBehavior(CarPosition car, vector<CarPosition> nearby_cars) {

  vector<double> costs;

  int lane = getLaneNumber(car.d);

  // Keep lane
  double kl_cost = logistic(MAX_VELOCITY / getLaneSpeed(lane, car, nearby_cars));
  costs.push_back(kl_cost);

  double lcl_cost = 1;
  if (lane != 0) {
    lcl_cost = calculateLaneChangeCost(lane, lane - 1, car, nearby_cars);
  }
  costs.push_back(lcl_cost);

  double rcl_cost = 1;
  if (lane != 2) {
    rcl_cost = calculateLaneChangeCost(lane, lane + 1, car, nearby_cars);
  }
  costs.push_back(rcl_cost);

  int min_index = 0;
  double min = 10;
  for (int i = 0; i < costs.size(); i++) {
    if (costs[i] < min) {
      min_index = i;
      min = costs[i];
    }
  }

  //cout << "KL: "<< kl_cost << " LCL: "<< lcl_cost << " RCL: "<< rcl_cost<<"\n";

  switch (min_index) {
    case 0:
      return BehaviorState::KEEP_LANE;
    case 1:
      return BehaviorState::CHANGE_TO_LEFT_LANE;
    case 2:
      return BehaviorState::CHANGE_TO_RIGHT_LANE;
    default:
      return BehaviorState::KEEP_LANE;
  }

}

double
TrajectoryPlanner::calculateLaneChangeCost(int from_lane, int to_lane, const CarPosition & car, vector<CarPosition> nearby_cars) {

  // check if the lane is free
  vector<CarPosition> nearbyInLane = filterCarsByLane(nearby_cars, to_lane);
  sortCarsByDistance(nearbyInLane, car);
  if (nearbyInLane.size() > 0 && distanceS(car.s, nearbyInLane[0].s) < 20) {
    return 1;
  }

  // check to see if you are even moving
  if (car.speed < 5) {
    return 1;
  }

  return logistic(MAX_VELOCITY / getLaneSpeed(to_lane, car, nearby_cars));

}


double TrajectoryPlanner::getLaneSpeed(int lane, const CarPosition & car, vector<CarPosition> nearby_cars) {

  vector<CarPosition> infront = filterCarsInFrontOf(nearby_cars, car.s);
  vector<CarPosition> nearbyInLane = filterCarsByLane(infront, lane);
  sortCarsByDistance(nearbyInLane, car);
  if (nearbyInLane.size() > 0 && distanceS(car.s, nearbyInLane[0].s) < 50)
    return nearbyInLane[0].speed;
  else
    return MAX_VELOCITY;
}

void TrajectoryPlanner::applyLaneChange(int from_lane, int to_lane, CarPosition car, vector<CarPosition> nearby_cars) {
  if (to_lane != getLaneNumber(jmt_d.finish)) {
    cout << "Change lane to " << to_lane << "\n";
    double lane_speed = getLaneSpeed(to_lane, car, nearby_cars);
    adjustTargetSpeed(car.speed, lane_speed, 0);
    adjustTargetLane(from_lane, to_lane, 0);
  }
}

void
TrajectoryPlanner::applyKeepLaneBehavior(double current_speed, const CarPosition & car, vector<CarPosition> nearby_cars, int lane) {

  vector<CarPosition> infront = filterCarsInFrontOf(nearby_cars, car.s);
  vector<CarPosition> nearbyInLane = filterCarsByLane(infront, lane);
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

  double targetDiff = fabs(target_speed - jmt_v.finish);
  if (targetDiff > 1.0) {
    adjustTargetSpeed(current_speed, target_speed, 0);
  }


}

double TrajectoryPlanner::getDeltaV(double t) {

  if (t >= jmt_v.duration) {
    jmt_v.completed = true;
    return jmt_v.finish;
  }

  vector<double> jmt = jmt_v.jmt;

  return (jmt[0] + jmt[1] * t + jmt[2] * t * t + jmt[3] * t * t * t + jmt[4] * t * t * t * t +
          jmt[5] * t * t * t * t * t);
}

double TrajectoryPlanner::getDeltaD(double t) {

  if (jmt_d.duration <= t) {
    jmt_d.completed = true;
    return adjustToCenterOfLane(jmt_d.finish);
  }

  vector<double> jmt = jmt_d.jmt;

  return (jmt[0] + jmt[1] * t + jmt[2] * t * t + jmt[3] * t * t * t + jmt[4] * t * t * t * t +
          jmt[5] * t * t * t * t * t);
}

void TrajectoryPlanner::adjustTargetSpeed(double currentSpeed, double targetSpeed, double deltaT) {
  targetSpeed = (targetSpeed > MAX_VELOCITY) ? MAX_VELOCITY : targetSpeed;

  double max = fabs(targetSpeed - currentSpeed) / TARGET_ACCEL;
  deltaT = (deltaT == 0 || deltaT > max) ? max : deltaT;

  cout << "Adjusting Speed from: " << currentSpeed << "m/s to :" << targetSpeed << " m/s  in " << deltaT << "s \n";


  jmt_v.jmt = JMT({currentSpeed, 0, 0}, {targetSpeed, 0, 0}, deltaT);
  jmt_v.finish = targetSpeed;
  jmt_v.duration = deltaT;
  jmt_v.completed = false;

  timeline = 0;
}

void TrajectoryPlanner::adjustTargetLane(int currentLane, int targetLane, double deltaT) {
  if (currentLane == targetLane) {
    deltaT = 0;
  } else {
    deltaT = (deltaT == 0 || deltaT > LANE_CHANGE_TIME) ? LANE_CHANGE_TIME : deltaT;
  }
  jmt_d.jmt = JMT({getLaneCenter(currentLane), 0, 0}, {getLaneCenter(targetLane), 0, 0}, deltaT);
  getLaneCenter(currentLane);
  jmt_d.finish = getLaneCenter(targetLane);
  jmt_d.duration = deltaT;
  jmt_d.completed = false;
}


void TrajectoryPlanner::sortCarsByDistance(vector<CarPosition> &cars, const CarPosition & carPosition) {

  sort(cars.begin(), cars.end(), [&](const CarPosition & lhs, const CarPosition &rhs) {
      return distanceS(carPosition.s, lhs.s) < distanceS(carPosition.s, rhs.s);
  });
}

vector<CarPosition> TrajectoryPlanner::filterCarsInFrontOf(vector<CarPosition> cars, double s) {

  vector<CarPosition> filtered;
  copy_if(cars.begin(), cars.end(),
          std::back_inserter(filtered),
          [&](const CarPosition & car) { return isInFront(s, car.s); });
  return filtered;

}

vector<CarPosition> TrajectoryPlanner::filterCarsByLane(vector<CarPosition> cars, int lane) {

  vector<CarPosition> filtered;
  copy_if(cars.begin(), cars.end(),
          std::back_inserter(filtered),
          [&](const CarPosition &car) { return getLaneNumber(car.d) == lane; });
  return filtered;

}

vector<double> TrajectoryPlanner::JMT(vector<double> start, vector<double> end, double T) {
  MatrixXd A = MatrixXd(3, 3);
  A << T * T * T, T * T * T * T, T * T * T * T * T,
    3 * T * T, 4 * T * T * T, 5 * T * T * T * T,
    6 * T, 12 * T * T, 20 * T * T * T;

  MatrixXd B = MatrixXd(3, 1);
  B << end[0] - (start[0] + start[1] * T + .5 * start[2] * T * T),
    end[1] - (start[1] + start[2] * T),
    end[2] - start[2];

  MatrixXd Ai = A.inverse();

  MatrixXd C = Ai * B;

  vector<double> result = {start[0], start[1], .5 * start[2]};
  for (int i = 0; i < C.size(); i++) {
    result.push_back(C.data()[i]);
  }

  return result;

}

