//
// Created by hvrigazov on 18.09.17.
//

#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

static constexpr double MPH2MPS = 0.44704;

const double ACCELERATION = 0.2;
const double GAP_THRESHOLD = 50;
const double NEEDED_DISTANCE_IN_FRONT_FOR_LANE_CHANGE = 20;
const double NEEDED_DISTANCE_FROM_BACK_FOR_LANE_CHANGE = 10;
const double NEEDED_SPEED_FOR_LANE_CHANGE = 30.0 * MPH2MPS;

const int NUMBER_OF_POINTS = 50;

const double SPEED_LIMIT = 50;
const double LANE_WIDTH = 4;
const double TIME_BETWEEN_EVENTS = 0.02;
const double KEEP_LANE_BONUS = -0.1;

const double HORIZON = 30;

#endif //PATH_PLANNING_CONSTANTS_H
