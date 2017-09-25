//
// Created by hvrigazov on 22.09.17.
//

#ifndef PATH_PLANNING_GAPSPEED_H
#define PATH_PLANNING_GAPSPEED_H


#include "Utilities.h"

class GapSpeed {
public:
  double gap;
  double speed;

  GapSpeed(double gap_, double speed_) : gap(gap_), speed(speed_) {}
};


#endif //PATH_PLANNING_GAPSPEED_H
