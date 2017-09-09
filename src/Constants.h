//
// Created by hvrigazov on 31.08.17.
//
#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <math.h>

const double LANE_WIDTH = 4;

const double SAMPLING_RATE = 0.02;

const double TARGET_ACCEL = 3;

const double LANE_CHANGE_TIME = 4;

// 3 lanes, start in the middle
const int INITIAL_LANE = 1;

inline double mph2ms(double mph){
  return mph * 0.44704; 
}
const double MAX_VELOCITY = mph2ms(45);

inline double deg2rad(double x) { return x * M_PI / 180; }
inline double rad2deg(double x) { return x * 180 / M_PI; }

const double MAX_S = 6945.554;

const double CAR_IS_NEAR_THRESHOLD = 150;
const double NUMBER_OF_INTERMEDIATE_POINTS = 50;


#endif /* CONSTANTS_H */
