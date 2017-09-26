//
// Created by hvrigazov on 17.09.17.
//

#include "PathPlanner.h"

Path PathPlanner::plan(const TelemetryData &data) {
  update(data);
  lane = lane_at(current.frenet.d);
  compute_predictions(data);
  create_plan(data);
  adjust_speed();
  return generate_trajectory(data);
}

void PathPlanner::update(const TelemetryData &data) {
  if (data.previous_path.size() < 2) {
    current = data.car_state;
    previous = CartesianPoint(current.cartesian.x, current.cartesian.y, current.yaw);
    return;
  }

  current.cartesian.x = data.previous_path.x.back();
  current.cartesian.y = data.previous_path.y.back();

  previous = { data.previous_path.x.end()[-2], data.previous_path.y.end()[-2] };

  current.yaw = std::atan2(current.cartesian.y - previous.y, current.cartesian.x - previous.x);
  current.speed = distance(previous.x, previous.y, current.cartesian.x, current.cartesian.y) / TIME_BETWEEN_EVENTS;

  current.frenet.s = data.end_path.s;
  current.frenet.d = data.end_path.d;
}

void PathPlanner::compute_predictions(const TelemetryData &data) {
  for (Lane & lane: lane_gap_speeds) {
    lane = Lane();
  }

  for(const Car & other_car : data.other_cars) {
    int car_lane = lane_at(other_car.d);
    Lane & lane = lane_gap_speeds[car_lane];
    double car_speed = norm(other_car.vx, other_car.vy);
    // predict where the car will be
    double other_car_next_s = other_car.s + car_speed * data.previous_path.size() * TIME_BETWEEN_EVENTS;
    double car_next_s = current.frenet.s + current.speed * data.previous_path.size() * TIME_BETWEEN_EVENTS;
    double predicted_gap = other_car_next_s - current.frenet.s;
    double absolute_predicted_gap = fabs(predicted_gap);

    // if the car is in front and the gap is closer to
    // the latest registered gap in the lane, then this is the new
    // gap in front that we should take into account
    if (predicted_gap > 0 && absolute_predicted_gap < lane.in_front.gap) {
      lane.in_front.gap = absolute_predicted_gap;
      lane.in_front.speed = car_speed;
    } else if (absolute_predicted_gap < lane.behind.gap) {
      lane.behind.gap = absolute_predicted_gap;
      lane.behind.speed = car_speed;
    }
  }
}

int PathPlanner::get_best_lane() const {
  int best_lane = 0;

  for (int i = 1; i < 3; i++) {
    if (get_lane_cost(i) < get_lane_cost(best_lane)) {
      best_lane = i;
    }
  }

  return best_lane;
}

void PathPlanner::create_plan(const TelemetryData &data) {
  switch (state) {
    case STATE::START:
      handle_start_state();
      break;
    case STATE::KEEP_LANE:
      handle_keep_lane_state();
      break;
    case STATE::LANE_CHANGE:
      handle_lane_change();
      break;
  }
}

void PathPlanner::handle_lane_change() {
  double road_speed_limit = get_speed_limit();
  double cte = get_cross_track_error();
  bool lane_change_completed = lane == target_lane && fabs(cte) < 0.2;

  if (lane_change_completed) {
    state = STATE::KEEP_LANE;
    handle_keep_lane_state();
    return;
  }

  // increase speed
  target_speed = road_speed_limit;
  return;
}

void PathPlanner::handle_keep_lane_state() {
  int best_lane = get_best_lane();
  const double road_speed_limit = get_speed_limit();
  assert(target_lane == lane);

  target_speed = road_speed_limit;

  // Should I change lane?

  if (current.speed > NEEDED_SPEED_FOR_LANE_CHANGE && fabs(lane - best_lane) == 1 &&
    lane_gap_speeds[best_lane].change_is_possible()) {
    target_lane = best_lane;
    state = STATE::LANE_CHANGE;
    handle_lane_change();
    return;
  }

  // adjust speed
  if (lane_gap_speeds[target_lane].in_front.gap < GAP_THRESHOLD)
  {
    if (lane_gap_speeds[target_lane].in_front.gap < (GAP_THRESHOLD / 2.0))
      target_speed = 0;
    else
      target_speed = fmin(target_speed, lane_gap_speeds[target_lane].in_front.speed);
  }
  return;
}

void PathPlanner::handle_start_state() {
  target_lane = lane;
  state = STATE::KEEP_LANE;
  handle_keep_lane_state();
}

void PathPlanner::adjust_speed() {
  if (target_speed < current.speed) {
    target_speed = fmax(target_speed, current.speed - ACCELERATION);
  } else {
    target_speed = fmin(target_speed, current.speed + ACCELERATION);
  }
}

Path PathPlanner::generate_trajectory(const TelemetryData &data) {
  tk::spline spline = create_spline();

  Path path;
  path.x = data.previous_path.x;
  path.y = data.previous_path.y;

  const double target_dist = norm(HORIZON, spline(HORIZON));

  const double t = HORIZON / target_dist * TIME_BETWEEN_EVENTS;

  for(int i = 1; i <= NUMBER_OF_POINTS - path.x.size(); i++) {
    double target_x = i * t * target_speed;
    double target_y = spline(target_x);
    double x = target_x * cos(current.yaw) - target_y * sin(current.yaw) + current.cartesian.x;
    double y = target_x * sin(current.yaw) + target_y * cos(current.yaw) + current.cartesian.y;
    path.append(x, y);
  }

  return path;
}

tk::spline PathPlanner::create_spline() const {
  const auto target_d = lane_center(target_lane);

  Path trajectory_points;

  trajectory_points.append(previous.x, previous.y);
  trajectory_points.append(current.cartesian.x, current.cartesian.y);

  for (int i = 1; i <= 3; i++) {
    trajectory_points.append(world.to_xy(current.frenet.s + HORIZON * i, target_d));
  }

  for(int i = 0; i < trajectory_points.size(); i++) {
    const double dx = trajectory_points.x[i] - current.cartesian.x;
    const double dy = trajectory_points.y[i] - current.cartesian.y;
    trajectory_points.x[i] = dx * cos(-current.yaw) - dy * sin(-current.yaw);
    trajectory_points.y[i] = dx * sin(-current.yaw) + dy * cos(-current.yaw);
  }

  tk::spline spline;
  spline.set_points(trajectory_points.x, trajectory_points.y);
  return spline;
}

PathPlanner::PathPlanner(const World &world) : world(world), state(STATE::START) {
}

double PathPlanner::get_speed_limit() const {
  return mph2mps(SPEED_LIMIT) - 1.0;
}

double PathPlanner::get_cross_track_error() const {
  return (current.frenet.d - lane_center(target_lane));
}

double PathPlanner::get_lane_cost(int i) const {
  if (abs(lane - i) > 1) {
    return INF;
  }

  return lane_gap_speeds[i].cost() + (i == lane ? KEEP_LANE_BONUS : 0);
}
