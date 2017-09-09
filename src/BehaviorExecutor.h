//
// Created by hvrigazov on 09.09.17.
//

#ifndef PATH_PLANNING_BEHAVIOREXECUTOR_H
#define PATH_PLANNING_BEHAVIOREXECUTOR_H


#include "CarPosition.h"
#include "Utils.h"
#include <vector>

using namespace std;

class BehaviorExecutor {
public:
  void applyKeepLaneBehavior(double current_speed,
                             const CarPosition &car,
                             vector<CarPosition> nearby_cars,
                             int lane,
                             double jmt_v_finish);


private:
  vector<CarPosition> filterCarsInFrontOf(vector<CarPosition> cars, double s);

  vector<CarPosition> filterCarsByLane(vector<CarPosition> cars, int lane);

  void sortCarsByDistance(vector<CarPosition> & cars, const CarPosition & carPosition);

  void adjustTargetSpeed(double currentSpeed, double targetSpeed, double deltaT);
};


#endif //PATH_PLANNING_BEHAVIOREXECUTOR_H
