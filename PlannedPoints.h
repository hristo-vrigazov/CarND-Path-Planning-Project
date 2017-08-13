//
// Created by hvrigazov on 13.08.17.
//

#ifndef PATH_PLANNING_PLANNEDPOINTS_H
#define PATH_PLANNING_PLANNEDPOINTS_H


#include <vector>

class PlannedPoints {
public:
    PlannedPoints()  {
      double dist_inc = 0.5;
      for(int i = 0; i < 50; i++)
      {
        next_x_vals.push_back(0.6);
        next_y_vals.push_back(0.9);
      }
    }

public:
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;
};


#endif //PATH_PLANNING_PLANNEDPOINTS_H
