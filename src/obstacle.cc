#include <algorithm>
#include <cmath>
#include <iostream>

#include "obstacle.h"

std::tuple<double, Eigen::MatrixXd, Eigen::MatrixXd> Obstacle::BarrierFunction(
    double q1, double q2, double c, const Eigen::MatrixXd& c_dot) {
  const double exp_q2_c = std::exp(q2 * c);
  const double b = q1 * exp_q2_c;
  const auto b_dot = q1 * q2 * exp_q2_c * c_dot;
  const auto b_ddot =
      q1 * std::pow(q2, 2) * exp_q2_c * c_dot * c_dot.transpose();
  return std::make_tuple(b, b_dot, b_ddot);
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
Obstacle::GetObstacleCostDerivatives(const std::vector<State>& npc_traj, int i,
                                     const State& ego_state) {
  const double a =
      car_length_ +
      std::abs(npc_traj[i].v * std::cos(npc_traj[i].theta)) * args_.t_safe +
      args_.s_safe_a + args_.ego_rad;
  const double b =
      car_width_ +
      std::abs(npc_traj[i].v * std::sin(npc_traj[i].theta)) * args_.t_safe +
      args_.s_safe_b + args_.ego_rad;

  Eigen::MatrixXd P1 = Eigen::MatrixXd::Zero(4, 4);
  P1(0, 0) = 1 / (a * a);
  P1(1, 1) = 1 / (b * b);

  const double npc_theta = npc_traj[i].theta;
  const double ego_theta = ego_state.theta;

  Eigen::MatrixXd transformation_matrix(4, 4);
  transformation_matrix << std::cos(npc_theta), std::sin(npc_theta), 0.0, 0.0,
      -std::sin(npc_theta), std::cos(npc_theta), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0;

  const State ego_front{ego_state.x + std::cos(ego_theta) + args_.ego_lf,
                        ego_state.y + std::sin(ego_theta) + args_.ego_lf,
                        ego_state.v, ego_state.theta};
  Eigen::MatrixXd state_diff1(4, 1);
  state_diff1 << ego_front.x - npc_traj[i].x, ego_front.y - npc_traj[i].y,
      ego_front.v - npc_traj[i].v, ego_front.theta - npc_traj[i].theta;
  const auto diff1 = transformation_matrix * state_diff1;

  const auto temp_value1 = diff1.transpose() * P1 * diff1;
  const double c1 = 1.0 - temp_value1(0, 0);
  const Eigen::MatrixXd c_dot1 = -2.0 * P1 * diff1;
  const auto [b_f, b_dot_f, b_ddot_f] =
      BarrierFunction(args_.q1_front, args_.q2_front, c1, c_dot1);

  const State ego_rear{ego_state.x - std::cos(ego_theta) + args_.ego_lr,
                       ego_state.y - std::sin(ego_theta) + args_.ego_lf,
                       ego_state.v, ego_state.theta};
  Eigen::MatrixXd state_diff2(4, 1);
  state_diff2 << ego_rear.x - npc_traj[i].x, ego_rear.y - npc_traj[i].y,
      ego_rear.v - npc_traj[i].v, ego_rear.theta - npc_traj[i].theta;
  const auto diff2 = transformation_matrix * state_diff2;

  const auto temp_value2 = diff2.transpose() * P1 * diff2;
  const double c2 = 1.0 - temp_value2(0, 0);
  const Eigen::MatrixXd c_dot2 = -2.0 * P1 * diff2;
  const auto [b_r, b_dot_r, b_ddot_r] =
      BarrierFunction(args_.q1_front, args_.q2_front, c2, c_dot2);

  return std::make_pair(b_dot_f + b_dot_r, b_ddot_f + b_ddot_r);
}