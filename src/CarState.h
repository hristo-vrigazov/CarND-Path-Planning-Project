//
// Created by hvrigazov on 21.09.17.
//

#ifndef PATH_PLANNING_CARSTATE_H
#define PATH_PLANNING_CARSTATE_H

/**
 * Represents the state of the car that is driving
 */
class CarState {
public:
  CartesianPoint cartesian;
  double yaw;
  double speed;
  FrenetPoint frenet;
};


#endif //PATH_PLANNING_CARSTATE_H
