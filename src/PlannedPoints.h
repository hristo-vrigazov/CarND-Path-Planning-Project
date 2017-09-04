//
// Created by hvrigazov on 31.08.17.
//

#ifndef PATH_PLANNING_PLANNEDPOINTS_H
#define PATH_PLANNING_PLANNEDPOINTS_H

#include <vector>

class PlannedPoints {
public:
    PlannedPoints(const std::vector<double> &next_x_values, const std::vector<double> &next_y_values) :
      next_x_values(next_x_values), next_y_values(next_y_values) {}

    std::vector<double> next_x_values;
    std::vector<double> next_y_values;
};


#endif //PATH_PLANNING_PLANNEDPOINTS_H
