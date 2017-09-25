//
// Created by hvrigazov on 17.09.17.
//

#ifndef PATH_PLANNING_LANE_H
#define PATH_PLANNING_LANE_H


#include "Utilities.h"
#include "GapSpeed.h"

class Lane {
public:
  GapSpeed in_front;
  GapSpeed behind;

  Lane() : in_front(INF, INF), behind(INF, 0) {}

  bool change_is_possible() const {
    return (in_front.gap > NEEDED_DISTANCE_IN_FRONT_FOR_LANE_CHANGE) && (behind.gap > NEEDED_DISTANCE_FROM_BACK_FOR_LANE_CHANGE);
  }

  double cost() const {
    return log((behind.speed) / (in_front.gap + behind.gap + in_front.speed));
  }
};

#endif //PATH_PLANNING_LANE_H
