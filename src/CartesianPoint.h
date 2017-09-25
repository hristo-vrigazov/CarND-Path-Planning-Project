//
// Created by hvrigazov on 17.09.17.
//

#ifndef PATH_PLANNING_CARTESIANPOINT_H
#define PATH_PLANNING_CARTESIANPOINT_H


class CartesianPoint {
public:
  double x;
  double y;

  CartesianPoint() {};

  CartesianPoint(double x, double y) : x(x), y(y) {}

  /**
   * Creates a cartesian point in the past based on current measurements
   * @param x current x
   * @param y current y
   * @param yaw current yaw
   */
  CartesianPoint(double x, double y, double yaw) : x(x - cos(yaw)), y(y - sin(yaw)) {}
};


#endif //PATH_PLANNING_CARTESIANPOINT_H
