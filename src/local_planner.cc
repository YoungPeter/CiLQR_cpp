#include <algorithm>
#include <cmath>
#include <iostream>

#include "basic_function.h"
#include "local_planner.h"

int LocalPlanner::GetClosestNodeIndex(const Pos2d& node) {
  int index = -1;
  double min_distance_square = std::numeric_limits<double>::max();
  for (int i = 0; i < global_plan_.size(); ++i) {
    double distance_square = std::pow(global_plan_[i].x - node.x, 2.0) +
                             std::pow(global_plan_[i].y - node.y, 2.0);
    if (distance_square < min_distance_square) {
      min_distance_square = distance_square;
      index = i;
    }
  }

  return index;
}

std::vector<Pos2d> LocalPlanner::GetLocalWaypoints() {
  const Pos2d node{ego_state_(0, 0), ego_state_(0, 1)};
  const int closet_index = GetClosestNodeIndex(node);
  std::vector<Pos2d> waypoints;
  for (int i = closet_index;
       i < number_of_local_wpts_ && i < global_plan_.size(); ++i) {
    waypoints.push_back(global_plan_[i]);
  }
  return waypoints;
}

std::pair<std::vector<Pos2d>, Eigen::MatrixXd> LocalPlanner::GetLocalPlan() {
  const std::vector<Pos2d> local_wpts = GetLocalWaypoints();
  Eigen::VectorXd x(local_wpts.size());
  Eigen::VectorXd y(local_wpts.size());
  for (int i = 0; i < local_wpts.size(); ++i) {
    x(i) = local_wpts[i].x;
    y(i) = local_wpts[i].y;
  }

  const Eigen::VectorXd coeffs = PolyFit(x, y, poly_order_);
  std::vector<Pos2d> local_path;
  local_path.resize(local_wpts.size());
  for (int i = 0; i < local_wpts.size(); ++i) {
    local_path[i].x = local_wpts[i].x;
    local_path[i].y = PolyEval(coeffs, local_wpts[i].x);
  }

  return std::make_pair(local_path, coeffs);
}