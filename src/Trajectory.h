//
// Created by hvrigazov on 06.09.17.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>

struct Trajectory {
  std::vector<double> jmt;
  double finish;
  double duration;
  bool completed;
};

#endif //PATH_PLANNING_TRAJECTORY_H
