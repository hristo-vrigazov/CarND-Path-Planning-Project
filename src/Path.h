//
// Created by hvrigazov on 17.09.17.
//

#ifndef PATH_PLANNING_PATH_H
#define PATH_PLANNING_PATH_H

#include <vector>
#include "CartesianPoint.h"

class Path {
public:
  std::vector<double> x, y;

  std::size_t size() const { return x.size(); }
  void append(const CartesianPoint xy) {
    x.push_back(xy.x);
    y.push_back(xy.y);
  }
  void append(const double x_, const double y_) {
    x.push_back(x_);
    y.push_back(y_);
  }
};


#endif //PATH_PLANNING_PATH_H
