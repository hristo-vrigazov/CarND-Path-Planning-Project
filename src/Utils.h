//
// Created by hvrigazov on 03.09.17.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H


#include <math.h>
#include <vector>

inline double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

inline std::vector<double> cartesian2polar(double vx, double vy) {
  double speed = sqrt(vx * vx + vy * vy);
  double theta = atan2(vy, vx);
  if(theta < 0) theta += 2 * M_PI;
  return {speed, theta};
}


inline int getLaneNumber(double d){
  return int(d / LANE_WIDTH);
}

inline double getLaneCenter(int lane){
  return lane*LANE_WIDTH + LANE_WIDTH/2;
}

inline double adjustToCenterOfLane(double d){
  return getLaneCenter(getLaneNumber(d));
}

inline double distanceS(const double from, const double to){
  double dist_f = fabs(from - (to+MAX_S));
  double dist_b = fabs(from - to);
  return (dist_f < dist_b)? dist_f: dist_b;
}

inline bool isInFront(double s_from, double s_to){
  double dist = fabs(s_from - (s_to+MAX_S));
  return (s_to > s_from) || (dist < 200);
}

/*
A function that returns a value between 0 and 1 for x in the
range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
Useful for cost functions.
*/
inline double logistic(double x){
  return 2.0 / (1 + exp(-x)) - 1.0;
}

#endif //PATH_PLANNING_UTILS_H
